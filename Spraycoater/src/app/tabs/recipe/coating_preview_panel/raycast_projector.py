# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Tuple, Optional
import numpy as np
import pyvista as pv
from pyvista import _vtk  # für OBBTree/VTK-Objekte

# ---------- Utilities ----------

def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    n = np.where(n < eps, 1.0, n)  # vermeidet Division durch ~0
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

# ---------- Safe converters ----------

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

# ---------- OBBTree Intersection ----------

def _intersect_line_obb(mesh: pv.PolyData, p0: np.ndarray, p1: np.ndarray) -> tuple[Optional[np.ndarray], Optional[int]]:
    """Erstes Line-Segment-Intersection via OBBTree. Gibt (hit_xyz, cell_id) zurück oder (None, None)."""
    obb = mesh.obbTree
    pts = _vtk.vtkPoints()
    ids = _vtk.vtkIdList()
    n_hits = obb.IntersectWithLine(p0, p1, pts, ids)
    if n_hits <= 0:
        return None, None
    # erstes Ergebnis (entspricht first_point=True)
    hit = np.array(pts.GetPoint(0), dtype=float)
    try:
        cid = int(ids.GetId(0))
    except Exception:
        cid = None
    return hit, cid

def _intersect_bidirectional(mesh: pv.PolyData, p0: np.ndarray, p1: np.ndarray) -> tuple[Optional[np.ndarray], Optional[int], bool]:
    """
    Versucht Intersection p0->p1, sonst p1->p0.
    Rückgabe: (hit, cid, flipped) wobei flipped=True bedeutet, dass in Gegenrichtung getroffen wurde.
    """
    hit, cid = _intersect_line_obb(mesh, p0, p1)
    if hit is not None and cid is not None:
        return hit, cid, False
    hit, cid = _intersect_line_obb(mesh, p1, p0)
    if hit is not None and cid is not None:
        return hit, cid, True
    return None, None, False

# ---------- Result ----------

@dataclass
class RayProjectResult:
    valid: np.ndarray        # (N,) bool
    hit_mm: np.ndarray       # (N,3)
    normal: np.ndarray       # (N,3) (Flächennormalen; gegen Ray gedreht)
    refl_dir: np.ndarray     # (N,3) normalisierte reflektierte Richtung (zur Visualisierung)
    tcp_mm: np.ndarray       # (N,3) = hit + stand_off * normal  (Offset ENTLANG Normale)

# ---------- Local-frame projector (robust) ----------

def project_path_via_rays_reflected(
    P_mm: np.ndarray,
    substrate_mesh: pv.PolyData,
    *,
    stand_off_mm: float,
    ray_dir_fn: Callable[[np.ndarray], np.ndarray],
    ray_len_mm: float = 10000.0,
    use_point_normals: bool = True,         # glattere Normale (empfohlen)
    grazing_cos_min: float = 1e-3,          # ~3.3° Grenzwinkel
) -> RayProjectResult:
    """
    Projiziert P_mm mit Rays (ray_dir_fn) auf substrate_mesh (gleicher Frame).
    - Normale via Properties (point_normals/face_normals)
    - Intersections via OBBTree (schnell/stabil)
    - TCP-Offset ENTlang der Normale
    """
    if not isinstance(substrate_mesh, pv.PolyData):
        raise TypeError("substrate_mesh muss pv.PolyData sein")

    mesh = substrate_mesh.triangulate()
    face_normals = np.asarray(mesh.face_normals)   # alias zu cell_normals
    point_normals = np.asarray(mesh.point_normals)

    P = np.asarray(P_mm, dtype=float).reshape(-1, 3)
    D = _normalize(np.asarray(ray_dir_fn(P), dtype=float).reshape(-1, 3))

    N = len(P)
    hits    = np.zeros((N, 3), dtype=float)
    normals = np.zeros((N, 3), dtype=float)
    refl    = np.zeros((N, 3), dtype=float)
    valid   = np.zeros((N,), dtype=bool)

    half_len = 0.5 * float(ray_len_mm)
    START = P - D * half_len
    END   = P + D * half_len

    for i in range(N):
        hit, cid, flipped = _intersect_bidirectional(mesh, START[i], END[i])
        if hit is None or cid is None:
            continue

        n = face_normals[cid].astype(float, copy=False)  # echte Flächennormale

        n = _normalize(n.reshape(1, 3))[0]

        # Normale gegen die "effektive" Rayrichtung drehen
        # (falls wir rückwärts getroffen haben, ist die effektive Rayrichtung -D[i])
        eff_dir = -D[i] if flipped else D[i]
        if float(np.dot(n, eff_dir)) > 0.0:
            n = -n

        # Grazing-Einfall filtern (stabiler TCP)
        cos_inc = -float(np.dot(eff_dir, n))
        if cos_inc < float(grazing_cos_min):
            continue

        # Reflexion nur zur Visualisierung
        r = eff_dir - 2.0 * float(np.dot(eff_dir, n)) * n
        r = _normalize(r.reshape(1, 3))[0]
        if np.dot(r, n) < 0.0:
            r = -r

        valid[i]   = True
        hits[i]    = hit
        normals[i] = n
        refl[i]    = r

    tcp = hits + normals * float(stand_off_mm)

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

# ---------- High-level world-frame caster ----------

def cast_rays_for_side(
    P_world_start: np.ndarray,
    *,
    sub_mesh_world: pv.PolyData,
    side: str,
    source: str,
    stand_off_mm: float,
    ray_len_mm: float = 1000.0,
    start_lift_mm: float = 10.0,            # Rays starten 1 cm über der blauen Maske
    flip_normals_to_face_rays: bool = True, # glatte Punktnormale bevorzugen
    invert_dirs: bool = False,              # gesamte Ray-Richtung invertieren (falls nötig)
    lock_xy: bool = True,                   # Startpunkte bleiben auf der Maske/Ebene
) -> tuple[RayProjectResult, pv.PolyData, pv.PolyData]:
    """
    Welt-Frame: erzeugt Rays abhängig von side/source und wirft sie auf sub_mesh_world.
    Gibt (RayProjectResult, rays_hit_poly, tcp_poly) zurück.
    """
    mesh = sub_mesh_world.triangulate()
    face_normals = np.asarray(mesh.face_normals)
    point_normals = np.asarray(mesh.point_normals)

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

    side = str(side or "").lower()
    base_dir = {
        "top":   np.array([0.0,  0.0, -1.0], dtype=float),
        "front": np.array([0.0,  1.0,  0.0], dtype=float),
        "back":  np.array([0.0, -1.0,  0.0], dtype=float),
        "left":  np.array([1.0,  0.0,  0.0], dtype=float),
        "right": np.array([-1.0, 0.0,  0.0], dtype=float),
    }.get(side, np.array([0.0, 0.0, -1.0], dtype=float))

    use_radial = ("spiral_cylinder" in (source or "").lower()) or ("helix" in (source or "").lower())
    if use_radial:
        c = _bounds_center(mesh.bounds)[:2]
        V = P[:, :2] - c[None, :]
        nrm = np.linalg.norm(V, axis=1, keepdims=True)
        nrm = np.where(nrm < 1e-12, 1.0, nrm)
        per_dirs = np.column_stack([-V / nrm, np.zeros((N, 1), float)])  # nach innen
        D = _normalize(per_dirs)
    else:
        D = np.tile(_normalize(base_dir), (N, 1))

    if invert_dirs:
        D = -D

    # Starts: knapp über der Maske
    if lock_xy:
        starts = P.copy()
        starts += (-D) * float(start_lift_mm)
    else:
        starts = P - D * float(start_lift_mm)

    ray_len = float(ray_len_mm)
    ends    = P + D * ray_len

    hits = np.zeros((N, 3), dtype=float)
    normals = np.zeros((N, 3), dtype=float)
    refl = np.zeros((N, 3), dtype=float)
    valid = np.zeros((N,), dtype=bool)

    for i in range(N):
        hit, cid, flipped = _intersect_bidirectional(mesh, starts[i], ends[i])
        if hit is None or cid is None:
            continue

        n = face_normals[cid].astype(float, copy=False)

        n = _normalize(n.reshape(1, 3))[0]

        eff_dir = -D[i] if flipped else D[i]
        if float(np.dot(n, eff_dir)) > 0.0:
            n = -n

        cos_inc = -float(np.dot(eff_dir, n))
        if cos_inc < 1e-3:
            continue

        r = eff_dir - 2.0 * float(np.dot(eff_dir, n)) * n
        r = _normalize(r.reshape(1, 3))[0]
        if np.dot(r, n) < 0.0:
            r = -r

        valid[i]   = True
        hits[i]    = hit
        normals[i] = n
        refl[i]    = r

    tcp = hits + normals * float(stand_off_mm)

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
