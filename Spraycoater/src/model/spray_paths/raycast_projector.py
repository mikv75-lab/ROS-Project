# -*- coding: utf-8 -*-
# File: src/model/spray_paths/raycast_projector.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple
import concurrent.futures
import math
import os

import numpy as np
import pyvista as pv
from pyvista import _vtk


# ============================================================
# STRICT utilities (no silent fallbacks)
# ============================================================

def _require_finite_array(name: str, a: np.ndarray) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    if a.size and (not np.isfinite(a).all()):
        bad = np.where(~np.isfinite(a))
        raise ValueError(f"{name} contains non-finite values at indices {bad}.")
    return a


def _normalize_strict(v: np.ndarray, eps: float = 1e-12, name: str = "v") -> np.ndarray:
    v = _require_finite_array(name, np.asarray(v, dtype=float))
    if v.size == 0:
        return v
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    if (not np.isfinite(n).all()):
        raise ValueError(f"Cannot normalize {name}: non-finite norm encountered.")
    if np.any(n < eps):
        idx = np.where((n < eps).reshape(-1))[0]
        raise ValueError(f"Cannot normalize {name}: near-zero vectors at indices {idx.tolist()} (eps={eps}).")
    return v / n


def _mesh_center_xy(mesh: pv.PolyData) -> Tuple[float, float]:
    b = mesh.bounds
    return (0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]))


def raydir_radial_xy(P_mm: np.ndarray, center_xy: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """
    Radial inward ray direction in XY-plane:
      D = normalize(center_xy - P_xy), Z=0
    """
    P = _require_finite_array("P_mm", np.asarray(P_mm, dtype=float)).reshape(-1, 3)
    cx, cy = float(center_xy[0]), float(center_xy[1])
    V2 = np.array([cx, cy], dtype=float) - P[:, :2]
    V2 = _normalize_strict(V2, name="radial_xy_dirs")  # (N,2)
    return np.column_stack([V2, np.zeros((len(P),), dtype=float)])


def _side_dir(side: str) -> np.ndarray:
    """
    Axis-aligned ray direction INTO the substrate.

    Convention (matches typical "outside looking in"):
      - top   : cast downwards (-Z)
      - front : cast towards -Y (from +Y side into the object)
      - back  : cast towards +Y (from -Y side into the object)
      - left  : cast towards -X (from +X side into the object)
      - right : cast towards +X (from -X side into the object)
    """
    sd = str(side or "").strip().lower()
    base = {
        "top":   [0.0, 0.0, -1.0],
        "front": [0.0, -1.0, 0.0],
        "back":  [0.0,  1.0, 0.0],
        "left":  [-1.0, 0.0, 0.0],
        "right": [1.0,  0.0, 0.0],
    }.get(sd)
    if base is None:
        raise ValueError(f"Unknown axis-aligned side {side!r}. Allowed: top/front/back/left/right.")
    return _normalize_strict(np.asarray(base, dtype=float).reshape(1, 3), name="side_dir")[0]


def _points_poly(points: np.ndarray) -> pv.PolyData:
    pts = _require_finite_array("points", np.asarray(points, dtype=float)).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pv.PolyData()
    return pv.PolyData(pts)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


# ============================================================
# Data contract
# ============================================================

@dataclass
class RayProjectResult:
    valid: np.ndarray      # (N,) bool
    hit_mm: np.ndarray     # (N,3) float (undefined for invalid -> zeros)
    normal: np.ndarray     # (N,3) float (undefined for invalid -> zeros)
    refl_dir: np.ndarray   # (N,3) float (undefined for invalid -> zeros)
    tcp_mm: np.ndarray     # (N,3) float (undefined for invalid -> zeros)


# ============================================================
# Optimized raycasting core (STRICT)
# ============================================================

def _intersect_line_obb_fast(obb, p0, p1):
    pts, ids = _vtk.vtkPoints(), _vtk.vtkIdList()
    if obb.IntersectWithLine(p0, p1, pts, ids) <= 0:
        return None, None
    return np.array(pts.GetPoint(0), dtype=float), int(ids.GetId(0))


def _process_chunk(idx, starts, ends, D, obb, face_normals, cos_min: float, sin_min: float, tilt_enabled: bool):
    """
    cos_min = cos(max_tilt_rad)
    sin_min = sin(max_tilt_rad)
    If tilt_enabled: clamp normals to max tilt instead of rejecting hits.
    """
    N = len(idx)
    hits = np.zeros((N, 3), dtype=float)
    norms = np.zeros((N, 3), dtype=float)
    refl = np.zeros((N, 3), dtype=float)
    valid = np.zeros(N, dtype=bool)

    n_faces = int(face_normals.shape[0])

    for i in range(N):
        hit, cid = _intersect_line_obb_fast(obb, starts[i], ends[i])
        if hit is None:
            hit, cid = _intersect_line_obb_fast(obb, ends[i], starts[i])
            if hit is None:
                continue

        if cid is None or cid < 0 or cid >= n_faces:
            raise RuntimeError(f"OBB returned cell id out of range: cid={cid}, n_face_normals={n_faces}")

        n = face_normals[cid].copy()
        d = D[i]

        # ensure normal points "against" the ray direction
        if float(np.dot(n, d)) > 0.0:
            n = -n

        # incidence / tilt:
        # cos(theta) = -dot(d, n), theta=0 => perfect incidence (n aligned with -d).
        c = -float(np.dot(d, n))

        if tilt_enabled and (c < cos_min):
            # Clamp n to have exactly theta = max_tilt (i.e. cos(theta)=cos_min),
            # in the plane spanned by n and a=-d, choosing the closest such vector to n.
            a = -d
            na = float(np.linalg.norm(a))
            if na > 1e-12:
                a = a / na
            else:
                a = np.array([0.0, 0.0, 1.0], dtype=float)

            u = n - float(np.dot(n, a)) * a
            nu = float(np.linalg.norm(u))
            if nu < 1e-12:
                n = a
            else:
                u = u / nu
                n = (float(cos_min) * a) + (float(sin_min) * u)
                nn = float(np.linalg.norm(n))
                if nn > 1e-12:
                    n = n / nn

            c = -float(np.dot(d, n))

        valid[i] = True
        hits[i] = hit
        norms[i] = n
        refl[i] = d - 2.0 * float(np.dot(d, n)) * n

    return idx, valid, hits, norms, refl


# ============================================================
# High-Level API (STRICT; no `source`)
# ============================================================

def cast_rays_for_side(
    P_world_start: np.ndarray,
    *,
    sub_mesh_world: pv.PolyData,
    side: str,
    stand_off_mm: float,
    ray_len_mm: float = 1000.0,
    start_lift_mm: float = 10.0,
    invert_dirs: bool = False,
    radial_xy: Optional[bool] = None,
    max_nozzle_tilt_deg: float = 0.0,
) -> tuple[RayProjectResult, pv.PolyData, pv.PolyData, pv.PolyData]:
    """
    Raycast projection used by editor/preview.

    Direction selection (STRICT):
      - side in {"top","front","back","left","right"} -> AXIS-ALIGNED into substrate (see _side_dir)
      - side in {"helix","polyhelix"}                 -> RADIAL in XY (towards mesh center)
      - any other side string                         -> RADIAL in XY (towards mesh center)

    Nozzle tilt constraint (CLAMP, not reject):
      - max_nozzle_tilt_deg <= 0 : disabled (use raw face normals)
      - otherwise                : clamp surface normals such that incidence angle <= max_nozzle_tilt_deg

    Stand-off rule (FIX):
      - helix/polyhelix: stand_off_mm is NOT applied to tcp (tcp==hit in that sense)
      - axis sides      : stand_off_mm is applied as tcp = hit + normal*stand_off_mm
    """
    if not isinstance(sub_mesh_world, pv.PolyData):
        raise ValueError(f"sub_mesh_world must be pyvista.PolyData, got {type(sub_mesh_world).__name__}")

    P = _require_finite_array("P_world_start", np.asarray(P_world_start, dtype=float)).reshape(-1, 3)
    N = int(P.shape[0])

    if N == 0:
        empty = RayProjectResult(
            valid=np.zeros((0,), dtype=bool),
            hit_mm=np.zeros((0, 3), dtype=float),
            normal=np.zeros((0, 3), dtype=float),
            refl_dir=np.zeros((0, 3), dtype=float),
            tcp_mm=np.zeros((0, 3), dtype=float),
        )
        return empty, pv.PolyData(), pv.PolyData(), pv.PolyData()

    sd = str(side or "").strip().lower()
    axis_sides = {"top", "front", "back", "left", "right"}
    helix_sides = {"helix", "polyhelix"}

    # explicit selection rule (your request)
    if radial_xy is None:
        radial_xy = (sd in helix_sides) or (sd not in axis_sides)

    mesh = (
        sub_mesh_world.triangulate()
        if (hasattr(sub_mesh_world, "is_all_triangles") and not sub_mesh_world.is_all_triangles)
        else sub_mesh_world
    )
    if mesh.n_cells <= 0:
        raise ValueError("sub_mesh_world has no cells; cannot raycast.")

    obb = mesh.obbTree
    if obb is None:
        raise RuntimeError("sub_mesh_world.obbTree is None; cannot raycast.")

    face_norms = _normalize_strict(np.asarray(mesh.face_normals, dtype=float), name="face_normals")
    if face_norms.shape[0] != mesh.n_cells:
        raise RuntimeError(f"face_normals size mismatch: {face_norms.shape[0]} != n_cells {mesh.n_cells}")

    # 0..45 deg clamp parameters
    tilt = float(max_nozzle_tilt_deg or 0.0)
    tilt_enabled = tilt > 0.0
    if tilt_enabled:
        tilt = _clamp(tilt, 0.0, 45.0)
        tilt_rad = float(math.radians(tilt))
        cos_min = float(math.cos(tilt_rad))
        sin_min = float(math.sin(tilt_rad))
    else:
        cos_min = -1.0
        sin_min = 0.0

    # 1) Directions
    if radial_xy:
        cx, cy = _mesh_center_xy(mesh)
        D = raydir_radial_xy(P, (cx, cy))
    else:
        base = _side_dir(sd)
        D = np.tile(base.reshape(1, 3), (N, 1))

    if invert_dirs:
        D = -D

    # 2) Bracketing (start behind, end forward along ray dir)
    starts = P - D * float(start_lift_mm)
    ends = P + D * float(ray_len_mm)

    # 3) Parallel compute (STRICT: any worker error -> raise)
    valid = np.zeros((N,), dtype=bool)
    hits = np.zeros((N, 3), dtype=float)
    norms = np.zeros((N, 3), dtype=float)
    refl = np.zeros((N, 3), dtype=float)

    try:
        cpu = len(os.sched_getaffinity(0))
    except Exception:
        cpu = os.cpu_count() or 4
    cpu = max(1, int(cpu))

    chunk = int(math.ceil(N / cpu))
    if chunk <= 0:
        raise RuntimeError("Internal: chunk<=0")

    futures = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=cpu) as exc:
        for i in range(0, N, chunk):
            idx = np.arange(i, min(i + chunk, N))
            futures.append(
                exc.submit(
                    _process_chunk,
                    idx,
                    starts[i: i + chunk],
                    ends[i: i + chunk],
                    D[i: i + chunk],
                    obb,
                    face_norms,
                    cos_min,
                    sin_min,
                    tilt_enabled,
                )
            )

        for f in concurrent.futures.as_completed(futures):
            idx, v, h, n, r = f.result()
            valid[idx] = v
            hits[idx] = h
            norms[idx] = n
            refl[idx] = r

    # 4) Result (FIX: no stand-off for helix/polyhelix)
    eff_stand_off = float(stand_off_mm)
    if sd in helix_sides:
        eff_stand_off = 0.0

    tcp = hits + norms * eff_stand_off

    res = RayProjectResult(valid=valid, hit_mm=hits, normal=norms, refl_dir=refl, tcp_mm=tcp)

    # Debug geometry: point clouds
    hit_p = _points_poly(hits[valid]) if np.any(valid) else pv.PolyData()
    miss_p = _points_poly(P[~valid]) if np.any(~valid) else pv.PolyData()
    tcp_p = _points_poly(tcp[valid]) if np.any(valid) else pv.PolyData()

    return res, hit_p, miss_p, tcp_p
