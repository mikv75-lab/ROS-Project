# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Tuple, Optional, Any, Dict, List, Iterable
import numpy as np
import pyvista as pv
from pyvista import _vtk  # VTK-Objekte (vtkPoints/vtkIdList)
import concurrent.futures
import math
import os

# ---------- Utilities ----------

def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    n = np.where(n < eps, 1.0, n)
    return v / n

def quat_from_two_vectors(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    a = _normalize(a)
    b = _normalize(b)
    v = np.cross(a, b)
    w = 1.0 + np.sum(a * b, axis=1, keepdims=True)
    q = np.concatenate([v, w], axis=1)
    q = _normalize(q)
    return q

def raydir_topdown(P_mm: np.ndarray) -> np.ndarray:
    return np.tile(np.array([0.0, 0.0, -1.0], dtype=float), (len(P_mm), 1))

def raydir_radial_xy(P_mm: np.ndarray, center_xy: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    cx, cy = center_xy
    # Richtung von Außen auf das Zentrum (XY)
    V = np.array([cx, cy], dtype=float) - P_mm[:, :2]
    V = _normalize(V)
    return np.column_stack([V, np.zeros((len(P_mm),))])

# ---------- Safe converters ----------

def _first_point3(pts) -> Optional[np.ndarray]:
    if pts is None:
        return None
    arr = np.asarray(pts, dtype=float)
    if arr.ndim == 2 and arr.shape[0] >= 1 and arr.shape[1] >= 3:
        return np.array(arr[0, :3], dtype=float)
    if arr.ndim == 1 and arr.size >= 3:
        return np.array(arr[:3], dtype=float)
    return None

def _first_cell_id(ids) -> Optional[int]:
    if ids is None:
        return None
    arr = np.asarray(ids)
    if arr.ndim == 1 and arr.size >= 1:
        try:
            return int(arr[0])
        except Exception:
            return None
    return None

# ---------- Visualization helpers ----------

def _polydata_from_segments(A: np.ndarray, B: np.ndarray) -> pv.PolyData:
    A = np.asarray(A, dtype=float).reshape(-1, 3)
    B = np.asarray(B, dtype=float).reshape(-1, 3)
    M = min(len(A), len(B))
    if M == 0:
        return pv.PolyData()
    pts = np.vstack([A[:M], B[:M]])
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

# ---------- Result ----------

@dataclass
class RayProjectResult:
    valid: np.ndarray
    hit_mm: np.ndarray
    normal: np.ndarray
    refl_dir: np.ndarray
    tcp_mm: np.ndarray

# ---------- Optimized Raycasting (Threaded Core) ----------

def _intersect_line_obb_fast(obb, p0, p1):
    pts = _vtk.vtkPoints()
    ids = _vtk.vtkIdList()
    code = obb.IntersectWithLine(p0, p1, pts, ids)
    if code <= 0:
        return None, None
    hit = np.array(pts.GetPoint(0), dtype=float)
    cid = int(ids.GetId(0))
    return hit, cid

def _process_chunk(
    indices: np.ndarray,
    starts: np.ndarray,
    ends: np.ndarray,
    D: np.ndarray,
    mesh_obb,
    face_normals: np.ndarray,
    grazing_cos_min: float
):
    count = len(indices)
    c_hits = np.zeros((count, 3), dtype=float)
    c_normals = np.zeros((count, 3), dtype=float)
    c_refl = np.zeros((count, 3), dtype=float)
    c_valid = np.zeros((count,), dtype=bool)
    
    for i in range(count):
        p0, p1 = starts[i], ends[i]
        # Vorwärts-Versuch
        hit, cid = _intersect_line_obb_fast(mesh_obb, p0, p1)
        flipped = False
        
        # Rückwärts-Versuch (Sicherheit)
        if hit is None:
            hit, cid = _intersect_line_obb_fast(mesh_obb, p1, p0)
            flipped = True
            
        if hit is None:
            continue
            
        n = face_normals[cid] 
        eff_dir = -D[i] if flipped else D[i]
        
        # Normale so drehen, dass sie gegen den Ray zeigt
        if np.dot(n, eff_dir) > 0.0:
            n = -n
            
        # Grazing Filter
        cos_inc = -np.dot(eff_dir, n)
        if cos_inc < grazing_cos_min:
            continue
            
        # Reflexion berechnen
        r = eff_dir - 2.0 * np.dot(eff_dir, n) * n
        
        c_valid[i] = True
        c_hits[i] = hit
        c_normals[i] = n
        c_refl[i] = r
        
    return indices, c_valid, c_hits, c_normals, c_refl

# ---------- High-Level Functions ----------

def cast_rays_for_side(
    P_world_start: np.ndarray,
    *,
    sub_mesh_world: pv.PolyData,
    side: str,
    source: str,
    stand_off_mm: float,
    ray_len_mm: float = 1000.0,
    start_lift_mm: float = 10.0,
    flip_normals_to_face_rays: bool = True,
    invert_dirs: bool = False,
    lock_xy: bool = True,
) -> tuple[RayProjectResult, pv.PolyData, pv.PolyData, pv.PolyData]:
    
    mesh = sub_mesh_world.triangulate() if not sub_mesh_world.is_all_triangles else sub_mesh_world
    _ = mesh.obbTree 
    obb_obj = mesh.obbTree
    face_normals = _normalize(np.asarray(mesh.face_normals, dtype=float))

    P = np.asarray(P_world_start, dtype=float).reshape(-1, 3)
    N = len(P)
    
    if N == 0:
        empty = RayProjectResult(np.zeros((0,), bool), np.zeros((0,3), float), np.zeros((0,3), float), np.zeros((0,3), float), np.zeros((0,3), float))
        return empty, pv.PolyData(), pv.PolyData(), pv.PolyData()

    # 1. Richtungsbestimmung (Planar vs. Radial)
    src_lower = str(source or "").lower()
    use_radial = ("spiral" in src_lower and "cylinder" in src_lower) or ("helix" in src_lower)
    
    if use_radial:
        b = mesh.bounds
        center_xy = (0.5 * (b[0] + b[1]), 0.5 * (b[2] + b[3]))
        D = raydir_radial_xy(P, center_xy)
    else:
        side_key = str(side or "").lower()
        base_dir = {
            "top":   np.array([0.0, 0.0, -1.0]),
            "front": np.array([0.0, 1.0, 0.0]),
            "back":  np.array([0.0, -1.0, 0.0]),
            "left":  np.array([1.0, 0.0, 0.0]),
            "right": np.array([-1.0, 0.0, 0.0]),
        }.get(side_key, np.array([0.0, 0.0, -1.0]))
        D = np.tile(_normalize(base_dir).reshape(1,3), (N, 1))

    if invert_dirs:
        D = -D

    # 2. Start- und Endpunkte (bracketing)
    starts = P + (-D) * float(start_lift_mm)
    ends = P + D * float(ray_len_mm)

    # 3. Threading Setup
    try:
        cpu_count = len(os.sched_getaffinity(0)) 
    except AttributeError:
        cpu_count = os.cpu_count() or 4
        
    chunk_size = math.ceil(N / cpu_count)
    final_valid = np.zeros((N,), dtype=bool)
    final_hits = np.zeros((N, 3), dtype=float)
    final_normals = np.zeros((N, 3), dtype=float)
    final_refl = np.zeros((N, 3), dtype=float)

    with concurrent.futures.ThreadPoolExecutor(max_workers=cpu_count) as executor:
        futures = []
        for i in range(0, N, chunk_size):
            sl = slice(i, min(i + chunk_size, N))
            idx_range = np.arange(i, min(i + chunk_size, N))
            futures.append(executor.submit(
                _process_chunk, idx_range, starts[sl], ends[sl], D[sl], 
                obb_obj, face_normals, 1e-3
            ))

        for f in concurrent.futures.as_completed(futures):
            try:
                idxs, c_val, c_hit, c_norm, c_ref = f.result()
                final_valid[idxs] = c_val
                final_hits[idxs] = c_hit
                final_normals[idxs] = c_norm
                final_refl[idxs] = c_ref
            except Exception:
                pass

    # 4. Nachbearbeitung & Visualisierung
    tcp = final_hits + final_normals * float(stand_off_mm)
    
    rays_hit_poly = _polydata_from_segments(starts[final_valid], final_hits[final_valid]) if np.any(final_valid) else pv.PolyData()
    tcp_poly = _polydata_from_segments(final_hits[final_valid], tcp[final_valid]) if np.any(final_valid) else pv.PolyData()
    rays_miss_poly = _polydata_from_segments(starts[~final_valid], ends[~final_valid]) if np.any(~final_valid) else pv.PolyData()

    rc = RayProjectResult(final_valid, final_hits, final_normals, final_refl, tcp)
    return rc, rays_hit_poly, rays_miss_poly, tcp_poly

def project_path_via_rays_reflected(
    P_mm: np.ndarray,
    substrate_mesh: pv.PolyData,
    *,
    stand_off_mm: float,
    ray_dir_fn: Callable[[np.ndarray], np.ndarray],
    ray_len_mm: float = 10000.0,
    grazing_cos_min: float = 1e-3,
) -> RayProjectResult:
    """Fallback / Lokaler Projektor-Aufruf."""
    P = np.asarray(P_mm, dtype=float).reshape(-1, 3)
    # Richtungsfunktion auf die Punkte anwenden
    dirs = np.asarray(ray_dir_fn(P), dtype=float).reshape(-1, 3)
    D = _normalize(dirs)
    
    mesh = substrate_mesh.triangulate()
    _ = mesh.obbTree
    face_normals = np.asarray(mesh.face_normals, dtype=float)
    
    N = len(P)
    starts = P - D * (ray_len_mm * 0.5)
    ends   = P + D * (ray_len_mm * 0.5)
    
    _, valid, hits, normals, refl = _process_chunk(
        np.arange(N), starts, ends, D, 
        mesh.obbTree, face_normals, grazing_cos_min
    )
    
    tcp = hits + normals * float(stand_off_mm)
    return RayProjectResult(valid, hits, normals, refl, tcp)