# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Tuple, Optional
import numpy as np
import pyvista as pv

# ---------- Utilities ----------

def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True) + eps
    return v / n

def quat_from_two_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Quaternion (x,y,z,w), das Vektor a auf b rotiert. a,b: (N,3)."""
    a = _normalize(a)
    b = _normalize(b)
    v = np.cross(a, b)
    w = 1.0 + np.sum(a * b, axis=1, keepdims=True)
    q = np.concatenate([v, w], axis=1)
    q = _normalize(q)
    return q  # (N,4) xyzw

def raydir_topdown(P_mm: np.ndarray) -> np.ndarray:
    """Konstante Rays in -Z (Top-Down)."""
    return np.tile(np.array([0.0, 0.0, -1.0], dtype=float), (len(P_mm), 1))

def raydir_radial_xy(P_mm: np.ndarray, center_xy: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """Rays radial im XY (für Zylinder/Helix), Richtung -Z bleibt 0."""
    cx, cy = center_xy
    V = P_mm[:, :2] - np.array([cx, cy], dtype=float)
    V = _normalize(V)
    return np.column_stack([V, np.zeros((len(P_mm),))])

# ---------- Safe converters for ray_trace outputs ----------

def _first_point3(pts) -> Optional[np.ndarray]:
    """Gibt den ersten Schnittpunkt als (3,) float ndarray zurück, sonst None."""
    if pts is None:
        return None
    arr = np.asarray(pts, dtype=float)
    if arr.ndim == 2 and arr.shape[0] >= 1 and arr.shape[1] >= 3:
        return np.array(arr[0, :3], dtype=float)
    if arr.ndim == 1 and arr.size >= 3:
        return np.array(arr[:3], dtype=float)
    return None

def _first_cell_id(ids) -> Optional[int]:
    """Gibt die erste Cell-ID als int zurück, sonst None."""
    if ids is None:
        return None
    arr = np.asarray(ids)
    if arr.ndim == 1 and arr.size >= 1:
        try:
            return int(arr[0])
        except Exception:
            return None
    return None

# ---------- Result ----------

@dataclass
class RayProjectResult:
    valid: np.ndarray        # (N,) bool
    hit_mm: np.ndarray       # (N,3)
    normal: np.ndarray       # (N,3) (Flächennormalen)
    refl_dir: np.ndarray     # (N,3) normalisierte reflektierte Richtung
    tcp_mm: np.ndarray       # (N,3) = hit + stand_off * refl_dir

# ---------- Local-frame projector (robust) ----------

def project_path_via_rays_reflected(
    P_mm: np.ndarray,
    substrate_mesh: pv.PolyData,
    *,
    stand_off_mm: float,
    ray_dir_fn: Callable[[np.ndarray], np.ndarray],
    ray_len_mm: float = 10000.0,
) -> RayProjectResult:
    """
    Projiziert P_mm mit Rays (Richtung ray_dir_fn) auf substrate_mesh (gleicher Frame).
    """
    if not isinstance(substrate_mesh, pv.PolyData):
        raise TypeError("substrate_mesh muss pv.PolyData sein")

    if "Normals" not in substrate_mesh.cell_data:
        substrate_mesh = substrate_mesh.compute_normals(
            cell_normals=True, point_normals=False, inplace=False
        )

    P = np.asarray(P_mm, dtype=float).reshape(-1, 3)
    D = _normalize(np.asarray(ray_dir_fn(P), dtype=float).reshape(-1, 3))

    N = len(P)
    hits    = np.zeros((N, 3), dtype=float)
    normals = np.zeros((N, 3), dtype=float)
    valid   = np.zeros((N,), dtype=bool)

    START = P - D * (0.5 * float(ray_len_mm))
    END   = P + D * (0.5 * float(ray_len_mm))

    cell_normals = substrate_mesh.cell_data["Normals"]
    for i in range(N):
        pts, ids = substrate_mesh.ray_trace(START[i], END[i], first_point=True)
        hit = _first_point3(pts)
        cid = _first_cell_id(ids)

        if hit is None or cid is None:
            # Gegenrichtung versuchen
            pts2, ids2 = substrate_mesh.ray_trace(END[i], START[i], first_point=True)
            hit = _first_point3(pts2)
            cid = _first_cell_id(ids2)
            if hit is None or cid is None:
                continue

        valid[i] = True
        hits[i] = hit
        n = np.array(cell_normals[cid], dtype=float)
        n /= (np.linalg.norm(n) + 1e-12)
        normals[i] = n

    # Reflektion + TCP
    refl = np.zeros_like(P)
    for i in range(N):
        if not valid[i]:
            continue
        d = D[i]
        n = normals[i]
        r = d - 2.0 * float(np.dot(d, n)) * n
        r = _normalize(r.reshape(1, 3))[0]
        if np.dot(r, n) < 0:
            r = -r
        refl[i] = r

    tcp = hits + (refl * float(stand_off_mm)).astype(float)

    return RayProjectResult(
        valid=valid,
        hit_mm=hits,
        normal=normals,
        refl_dir=refl,
        tcp_mm=tcp,
    )

# ---------- Visualization helpers ----------

def _polydata_from_segments(A: np.ndarray, B: np.ndarray) -> pv.PolyData:
    """Erzeuge PolyData-Linien aus Segmenten A[i] -> B[i]."""
    A = np.asarray(A, dtype=float).reshape(-1, 3)
    B = np.asarray(B, dtype=float).reshape(-1, 3)
    M = min(len(A), len(B))
    if M == 0:
        return pv.PolyData()

    pts = np.vstack([A[:M], B[:M]])  # (2M,3)
    lines = np.empty((M, 3), dtype=np.int64)
    lines[:, 0] = 2
    lines[:, 1] = np.arange(0, M, dtype=np.int64)
    lines[:, 2] = np.arange(M, 2*M, dtype=np.int64)

    pd = pv.PolyData()
    pd.points = pts
    pd.lines = lines.reshape(-1)
    return pd

def _bounds_center(bounds) -> np.ndarray:
    return np.array([
        0.5 * (bounds[0] + bounds[1]),
        0.5 * (bounds[2] + bounds[3]),
        0.5 * (bounds[4] + bounds[5]),
    ], dtype=float)

# ---------- High-level world-frame caster used by the panel ----------

def cast_rays_for_side(
    P_world_start: np.ndarray,
    *,
    sub_mesh_world: pv.PolyData,
    side: str,
    source: str,
    stand_off_mm: float,
    ray_len_mm: float = 1000.0,
    start_lift_mm: float = 10.0,            # Rays starten 1 cm über der blauen Maske
    flip_normals_to_face_rays: bool = True, # Normalen so drehen, dass sie dem Ray entgegen zeigen
    invert_dirs: bool = False,              # wenn nötig, gesamte Ray-Richtung invertieren
    lock_xy: bool = True,                   # Startpunkte bleiben auf der Maske/Ebene
):
    """
    Welt-Frame: erzeugt Rays abhängig von side/source.
    Gibt (RayProjectResult, rays_hit_poly, tcp_poly) zurück.
      - rays_hit_poly: Linien von Start (über dem Pfad) zum Hit (hellblau)
      - tcp_poly:      Linien vom Hit zum TCP (optional)
    """
    if "Normals" not in sub_mesh_world.cell_data:
        sub_mesh_world = sub_mesh_world.compute_normals(
            cell_normals=True, point_normals=False, inplace=True
        )

    P = np.asarray(P_world_start, dtype=float).reshape(-1, 3)
    N = len(P)
    if N == 0:
        empty = RayProjectResult(
            valid=np.zeros((0,), bool),
            hit_mm=np.zeros((0, 3), float),
            normal=np.zeros((0, 3), float),
            refl_dir=np.zeros((0, 3), float),
            tcp_mm=np.zeros((0, 3), float),
        )
        return empty, pv.PolyData(), pv.PolyData()

    # Basis-Richtung je Seite (Ray-Zielflugrichtung)
    side = str(side or "").lower()
    base_dir = {
        "top":   np.array([0.0,  0.0, -1.0], dtype=float),
        "front": np.array([0.0,  1.0,  0.0], dtype=float),
        "back":  np.array([0.0, -1.0,  0.0], dtype=float),
        "left":  np.array([1.0,  0.0,  0.0], dtype=float),
        "right": np.array([-1.0, 0.0,  0.0], dtype=float),
    }.get(side, np.array([0.0, 0.0, -1.0], dtype=float))

    # Helix / spiral_cylinder: radiale Richtung (nach innen)
    use_radial = ("spiral_cylinder" in (source or "").lower()) or ("helix" in (source or "").lower())
    if use_radial:
        c = _bounds_center(sub_mesh_world.bounds)[:2]
        V = P[:, :2] - c[None, :]
        nrm = np.linalg.norm(V, axis=1, keepdims=True) + 1e-12
        per_dirs = np.column_stack([-V / nrm, np.zeros((N, 1), float)])  # nach innen
        D = _normalize(per_dirs)
    else:
        D = np.tile(_normalize(base_dir), (N, 1))

    # Optional gesamte Richtung invertieren (falls nötig)
    if invert_dirs:
        D = -D

    # Rays sollen knapp über der Maske starten:
    # Start = P - D * start_lift_mm  (bei top: D=-Z -> Start = +Z über P)
    ray_len = float(ray_len_mm)
    starts = P - D * float(start_lift_mm)
    ends   = P + D * ray_len

    # Ray-Trace
    hits = np.zeros((N, 3), dtype=float)
    normals = np.zeros((N, 3), dtype=float)
    valid = np.zeros((N,), dtype=bool)
    cell_normals = sub_mesh_world.cell_data["Normals"]

    for i in range(N):
        pts, ids = sub_mesh_world.ray_trace(starts[i], ends[i], first_point=True)
        hit = _first_point3(pts)
        cid = _first_cell_id(ids)

        if hit is None or cid is None:
            # Gegenrichtung versuchen
            pts2, ids2 = sub_mesh_world.ray_trace(ends[i], starts[i], first_point=True)
            hit = _first_point3(pts2)
            cid = _first_cell_id(ids2)
            if hit is None or cid is None:
                continue

        valid[i] = True
        hits[i] = hit
        n = np.array(cell_normals[cid], dtype=float)
        n /= (np.linalg.norm(n) + 1e-12)

        # Optional: Normalen so drehen, dass sie dem Ray entgegenzeigen
        if flip_normals_to_face_rays and np.dot(n, D[i]) > 0.0:
            n = -n

        normals[i] = n

    # Reflektion + TCP (für gültige Indizes)
    refl = np.zeros_like(D)
    for i in range(N):
        if not valid[i]:
            continue
        d = D[i]
        n = normals[i]
        r = d - 2.0 * float(np.dot(d, n)) * n
        r = _normalize(r.reshape(1, 3))[0]
        if np.dot(r, n) < 0:
            r = -r
        refl[i] = r

    tcp = hits + (refl * float(stand_off_mm)).astype(float)

    # Visualisierung: nur gültige Segmente
    if np.any(valid):
        rays_hit_poly = _polydata_from_segments(starts[valid], hits[valid])
        tcp_poly      = _polydata_from_segments(hits[valid], tcp[valid])
    else:
        rays_hit_poly = pv.PolyData()
        tcp_poly      = pv.PolyData()

    rc = RayProjectResult(
        valid=valid,
        hit_mm=hits,
        normal=normals,
        refl_dir=refl,
        tcp_mm=tcp,
    )
    return rc, rays_hit_poly, tcp_poly
