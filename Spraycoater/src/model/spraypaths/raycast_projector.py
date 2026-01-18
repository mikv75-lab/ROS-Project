# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple
import concurrent.futures
import math
import os
import numpy as np
import pyvista as pv
from pyvista import _vtk

# ---------- Utilities ----------
def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    n = np.where(n < eps, 1.0, n)
    return v / n

def raydir_radial_xy(P_mm: np.ndarray, center_xy: Tuple[float, float] = (0.0, 0.0)) -> np.ndarray:
    cx, cy = center_xy
    V = np.array([cx, cy], dtype=float) - np.asarray(P_mm, dtype=float)[:, :2]
    V = _normalize(V)
    return np.column_stack([V, np.zeros((len(P_mm),), dtype=float)])

# ---------- Data contracts ----------
@dataclass
class RayProjectResult:
    valid: np.ndarray
    hit_mm: np.ndarray
    normal: np.ndarray
    refl_dir: np.ndarray
    tcp_mm: np.ndarray

# ---------- Optimized Raycasting ----------
def _intersect_line_obb_fast(obb, p0, p1):
    pts, ids = _vtk.vtkPoints(), _vtk.vtkIdList()
    if obb.IntersectWithLine(p0, p1, pts, ids) <= 0: return None, None
    return np.array(pts.GetPoint(0), dtype=float), int(ids.GetId(0))

def _process_chunk(idx, starts, ends, D, obb, face_normals, cos_min):
    N = len(idx)
    hits, norms, refl = np.zeros((N,3)), np.zeros((N,3)), np.zeros((N,3))
    valid = np.zeros(N, bool)
    
    for i in range(N):
        hit, cid = _intersect_line_obb_fast(obb, starts[i], ends[i])
        if hit is None: # try reverse
            hit, cid = _intersect_line_obb_fast(obb, ends[i], starts[i])
            if hit is None: continue
            
        n = face_normals[cid]
        d = D[i]
        if np.dot(n, d) > 0: n = -n
        if -np.dot(d, n) < cos_min: continue
        
        valid[i] = True
        hits[i] = hit
        norms[i] = n
        refl[i] = d - 2 * np.dot(d, n) * n
        
    return idx, valid, hits, norms, refl

# ---------- High-Level API ----------
def cast_rays_for_side(
    P_world_start: np.ndarray,
    *,
    sub_mesh_world: pv.PolyData,
    side: str,
    source: str,
    stand_off_mm: float,
    ray_len_mm: float = 1000.0,
    start_lift_mm: float = 10.0,
    invert_dirs: bool = False,
    **kwargs
) -> tuple[RayProjectResult, pv.PolyData, pv.PolyData, pv.PolyData]:
    
    mesh = sub_mesh_world.triangulate() if not sub_mesh_world.is_all_triangles else sub_mesh_world
    obb = mesh.obbTree
    face_norms = _normalize(np.asarray(mesh.face_normals))
    
    P = np.asarray(P_world_start, float).reshape(-1, 3)
    N = len(P)
    
    if N == 0: return RayProjectResult([],[],[],[],[]), None, None, None

    # 1. Directions
    use_radial = any(k in str(source).lower() for k in ("spiral", "cylinder", "helix"))
    if use_radial:
        b = mesh.bounds
        D = raydir_radial_xy(P, (0.5*(b[0]+b[1]), 0.5*(b[2]+b[3])))
    else:
        sd = str(side).lower()
        base = {
            "front": [0,1,0], "back": [0,-1,0],
            "left": [1,0,0], "right": [-1,0,0]
        }.get(sd, [0,0,-1]) # default top
        D = np.tile(_normalize([base]), (N, 1))
        
    if invert_dirs: D = -D
        
    # 2. Bracketing
    starts = P - D * float(start_lift_mm)
    ends = P + D * float(ray_len_mm)
    
    # 3. Parallel Compute
    valid = np.zeros(N, bool)
    hits, norms, refl = np.zeros((N,3)), np.zeros((N,3)), np.zeros((N,3))
    
    try: cpu = len(os.sched_getaffinity(0))
    except: cpu = os.cpu_count() or 4
    
    chunk = int(math.ceil(N/cpu))
    with concurrent.futures.ThreadPoolExecutor(cpu) as exc:
        futs = [exc.submit(_process_chunk, np.arange(i, min(i+chunk, N)), starts[i:i+chunk], ends[i:i+chunk], D[i:i+chunk], obb, face_norms, 1e-3) for i in range(0, N, chunk)]
        for f in concurrent.futures.as_completed(futs):
            try:
                idx, v, h, n, r = f.result()
                valid[idx], hits[idx], norms[idx], refl[idx] = v, h, n, r
            except: pass

    # 4. Result
    tcp = hits + norms * float(stand_off_mm)
    
    # Debug geometry
    def mk_lines(A, B):
        if len(A) == 0: return None
        return pv.lines_from_points(np.column_stack([A, B]).reshape(-1,3)) # simplified

    # (Note: SceneManager expects polydata with 'lines' attribute set correctly)
    # Using simple pyvista construct here for brevity, SceneManager handles None checks.
    
    return RayProjectResult(valid, hits, norms, refl, tcp), None, None, None