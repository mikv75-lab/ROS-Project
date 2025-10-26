# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Tuple
import numpy as np
import pyvista as pv


# ---------- Utilities ----------

def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    n = np.linalg.norm(v, axis=-1, keepdims=True) + eps
    return v / n


def quat_from_two_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Quaternion (x,y,z,w), das Vektor a auf b rotiert. a,b: (N,3).
    """
    a = _normalize(a)
    b = _normalize(b)
    v = np.cross(a, b)
    w = 1.0 + np.sum(a * b, axis=1, keepdims=True)
    # numerisch robust
    q = np.concatenate([v, w], axis=1)
    q = _normalize(q)
    return q  # (N,4) xyzw


def raydir_topdown(P_mm: np.ndarray) -> np.ndarray:
    """Konstante Rays in -Z (Top-Down)."""
    return np.tile(np.array([0.0, 0.0, -1.0], dtype=float), (len(P_mm), 1))


def raydir_radial_xy(P_mm: np.ndarray, center_xy: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    """Rays radial im XY (für Zylinder/Helix), Richtung -Z bleibt 0."""
    cx, cy = center_xy
    V = P_mm[:, :2] - np.array([cx, cy])
    V = _normalize(V)
    return np.column_stack([V, np.zeros((len(P_mm),))])


@dataclass
class RayProjectResult:
    valid: np.ndarray        # (N,) bool
    hit_mm: np.ndarray       # (N,3)
    normal: np.ndarray       # (N,3) (Flächennormalen)
    refl_dir: np.ndarray     # (N,3) normalisierte reflektierte Richtung
    tcp_mm: np.ndarray       # (N,3) = hit + stand_off * refl_dir


# ---------- Core ----------

def project_path_via_rays_reflected(
    P_mm: np.ndarray,
    substrate_mesh: pv.PolyData,
    *,
    stand_off_mm: float,
    ray_dir_fn: Callable[[np.ndarray], np.ndarray],
    ray_len_mm: float = 10000.0,
) -> RayProjectResult:
    """
    Projiziert jeden Punkt P_mm mit einem Ray (Richtung aus ray_dir_fn) auf das Substrat.
    - Schneidet der Ray das Mesh → nutze Hit-Point + reflektierte Richtung für TCP.
    - Reflektion: r = d - 2*(d·n)*n mit n = Flächennormale an der getroffenen Zelle.
    - Units mm.
    """
    if not isinstance(substrate_mesh, pv.PolyData):
        raise TypeError("substrate_mesh muss pv.PolyData sein")

    if "Normals" not in substrate_mesh.cell_data:  # sichere Normale an Zellen
        substrate_mesh = substrate_mesh.compute_normals(cell_normals=True, point_normals=False, inplace=False)

    P = np.asarray(P_mm, dtype=float).reshape(-1, 3)
    D = _normalize(np.asarray(ray_dir_fn(P), dtype=float).reshape(-1, 3))

    N = len(P)
    hits = np.zeros((N, 3), dtype=float)
    normals = np.zeros((N, 3), dtype=float)
    valid = np.zeros((N,), dtype=bool)

    # Für jeden Punkt ein langes Segment in Ray-Richtung
    # Start etwas entgegen der Richtung, damit wir bei Topdown auch oberhalb beginnen
    START = P - D * (0.5 * ray_len_mm)
    END   = P + D * (0.5 * ray_len_mm)

    # PyVista hat ray_trace(start, end) → liefert (points, cell_ids)
    for i in range(N):
        pts, ids = substrate_mesh.ray_trace(START[i], END[i], first_point=True)
        if pts is None or len(pts) == 0 or ids is None or len(ids) == 0:
            # Versuch in Gegenrichtung (falls die erste Richtung „daneben“ lag)
            pts2, ids2 = substrate_mesh.ray_trace(END[i], START[i], first_point=True)
            if pts2 is None or len(pts2) == 0 or ids2 is None or len(ids2) == 0:
                continue
            hit = np.array(pts2[0], dtype=float)
            cid = int(ids2[0])
            d_used = -D[i]  # Gegenrichtung
        else:
            hit = np.array(pts[0], dtype=float)
            cid = int(ids[0])
            d_used = D[i]

        valid[i] = True
        hits[i] = hit

        # Zellnormale
        cell_normals = substrate_mesh.cell_data["Normals"]
        n = np.array(cell_normals[cid], dtype=float)
        n = _normalize(n.reshape(1, 3))[0]
        normals[i] = n

        # Reflektierte Richtung
        r = d_used - 2.0 * np.dot(d_used, n) * n
        r = _normalize(r.reshape(1, 3))[0]
        # Sicherstellen, dass r „weg“ von der Oberfläche zeigt
        if np.dot(r, n) < 0:
            r = -r
        # TCP = Hit + StandOff * r
        hits[i] = hit
        normals[i] = n
        D[i] = d_used

    refl = np.zeros_like(D)
    for i in range(N):
        if not valid[i]:
            continue
        d = D[i]; n = normals[i]
        r = d - 2.0 * np.dot(d, n) * n
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
