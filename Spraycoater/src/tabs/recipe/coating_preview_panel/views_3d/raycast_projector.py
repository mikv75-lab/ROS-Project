# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Callable, Tuple, Optional
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
    V = P_mm[:, :2] - np.array([cx, cy], dtype=float)
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

# ---------- Optimized Raycasting (Threaded) ----------

def _intersect_line_obb_fast(obb, p0, p1):
    """
    Thread-lokale Intersection. Erzeugt vtkPoints/vtkIdList lokal.
    """
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
    mesh_obb,            # C++ OBBTree Objekt
    face_normals: np.ndarray, # Numpy Array (Nx3)
    grazing_cos_min: float
):
    """
    Worker-Funktion für einen Thread. Verarbeitet einen Block von Rays.
    """
    count = len(indices)
    
    # Lokale Ergebnis-Arrays
    c_hits = np.zeros((count, 3), dtype=float)
    c_normals = np.zeros((count, 3), dtype=float)
    c_refl = np.zeros((count, 3), dtype=float)
    c_valid = np.zeros((count,), dtype=bool)
    
    for i in range(count):
        p0 = starts[i]
        p1 = ends[i]
        
        # 1. Versuch: Vorwärts (Start -> Ende)
        hit, cid = _intersect_line_obb_fast(mesh_obb, p0, p1)
        flipped = False
        
        # 2. Versuch: Rückwärts (falls Startpunkt knapp im Mesh liegt)
        if hit is None:
            hit, cid = _intersect_line_obb_fast(mesh_obb, p1, p0)
            flipped = True
            
        if hit is None:
            continue
            
        # Normale direkt aus Numpy Array lesen (schnell)
        n = face_normals[cid] 
        
        d_vec = D[i]
        eff_dir = -d_vec if flipped else d_vec
        
        # Normale gegen Ray drehen
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
    
    # 1. Mesh vorbereiten (Triangulieren wenn nötig)
    if sub_mesh_world.is_all_triangles:
        mesh = sub_mesh_world
    else:
        mesh = sub_mesh_world.triangulate()
    
    # WICHTIG: OBBTree explizit im Hauptthread bauen!
    # Der Zugriff ist nur thread-safe, wenn er fertig gebaut ist.
    _ = mesh.obbTree 
    obb_obj = mesh.obbTree # Das zugrundeliegende VTK-Objekt

    # Face-Normals vorab berechnen und als Numpy-Array bereitstellen
    # (vermeidet VTK-Calls in Threads)
    face_normals = np.asarray(mesh.face_normals, dtype=float)
    fn_norms = np.linalg.norm(face_normals, axis=1, keepdims=True)
    fn_norms[fn_norms < 1e-12] = 1.0
    face_normals = face_normals / fn_norms

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
        return empty, pv.PolyData(), pv.PolyData(), pv.PolyData()

    # 2. Richtungen berechnen (Vektorisierte Numpy Ops)
    side_key = str(side or "").lower()
    base_dir = {
        "top":   np.array([0.0,  0.0, -1.0], dtype=float),
        "front": np.array([0.0,  1.0,  0.0], dtype=float),
        "back":  np.array([0.0, -1.0,  0.0], dtype=float),
        "left":  np.array([1.0,  0.0,  0.0], dtype=float),
        "right": np.array([-1.0, 0.0,  0.0], dtype=float),
    }.get(side_key, np.array([0.0, 0.0, -1.0], dtype=float))

    use_radial = ("spiral_cylinder" in (source or "").lower()) or ("helix" in (source or "").lower())
    
    if use_radial:
        b = mesh.bounds
        c = np.array([(b[0]+b[1])*0.5, (b[2]+b[3])*0.5])
        V = P[:, :2] - c
        nrm = np.linalg.norm(V, axis=1, keepdims=True)
        nrm[nrm < 1e-12] = 1.0
        # Nach innen gerichtet
        per_dirs = np.column_stack([-V / nrm, np.zeros((N, 1), float)])
        D = per_dirs 
    else:
        D = np.tile(_normalize(base_dir).reshape(1,3), (N, 1))

    if invert_dirs:
        D = -D

    # 3. Start/End Punkte berechnen
    if lock_xy:
        starts = P + (-D) * float(start_lift_mm)
    else:
        starts = P - D * float(start_lift_mm)

    ends = P + D * float(ray_len_mm)

    # 4. THREADING SETUP
    try:
        cpu_count = len(os.sched_getaffinity(0)) 
    except AttributeError:
        cpu_count = os.cpu_count() or 4
        
    chunk_size = math.ceil(N / cpu_count)
    
    # Resultat-Arrays vorallozieren
    final_valid = np.zeros((N,), dtype=bool)
    final_hits = np.zeros((N, 3), dtype=float)
    final_normals = np.zeros((N, 3), dtype=float)
    final_refl = np.zeros((N, 3), dtype=float)

    with concurrent.futures.ThreadPoolExecutor(max_workers=cpu_count) as executor:
        futures = []
        for i in range(0, N, chunk_size):
            # Slices für Input-Arrays
            sl = slice(i, min(i + chunk_size, N))
            # Indizes, um zu wissen, wo wir ins Ergebnis-Array schreiben müssen
            idx_range = np.arange(i, min(i + chunk_size, N))
            
            futures.append(executor.submit(
                _process_chunk,
                idx_range,
                starts[sl],
                ends[sl],
                D[sl],
                obb_obj,      # C++ OBBTree (Thread-Safe Read)
                face_normals, # Numpy Array Read
                1e-3          # Grazing min
            ))

        # Ergebnisse einsammeln
        for f in concurrent.futures.as_completed(futures):
            try:
                idxs, c_val, c_hit, c_norm, c_ref = f.result()
                final_valid[idxs] = c_val
                final_hits[idxs] = c_hit
                final_normals[idxs] = c_norm
                final_refl[idxs] = c_ref
            except Exception as e:
                # Fallback, falls Threads crashen (sollte nicht passieren)
                pass

    # 5. TCP und Visualisierung
    tcp = final_hits + final_normals * float(stand_off_mm)

    valid_mask = final_valid
    if np.any(valid_mask):
        rays_hit_poly = _polydata_from_segments(starts[valid_mask], final_hits[valid_mask])
        tcp_poly      = _polydata_from_segments(final_hits[valid_mask], tcp[valid_mask])
    else:
        rays_hit_poly = pv.PolyData()
        tcp_poly      = pv.PolyData()

    if np.any(~valid_mask):
        rays_miss_poly = _polydata_from_segments(starts[~valid_mask], ends[~valid_mask])
    else:
        rays_miss_poly = pv.PolyData()

    rc = RayProjectResult(
        valid=final_valid,
        hit_mm=final_hits,
        normal=final_normals,
        refl_dir=final_refl,
        tcp_mm=tcp,
    )
    return rc, rays_hit_poly, rays_miss_poly, tcp_poly

# Fallback: Lokaler Projektor für Tests ohne Weltkoordinaten (nutzt denselben Core wenn nötig, hier vereinfacht)
def project_path_via_rays_reflected(
    P_mm: np.ndarray,
    substrate_mesh: pv.PolyData,
    *,
    stand_off_mm: float,
    ray_dir_fn: Callable[[np.ndarray], np.ndarray],
    ray_len_mm: float = 10000.0,
    grazing_cos_min: float = 1e-3,
) -> RayProjectResult:
    # Wrapper um die neue Logik, wenn man es lokal braucht
    P = np.asarray(P_mm, dtype=float).reshape(-1, 3)
    D = np.asarray(ray_dir_fn(P), dtype=float).reshape(-1, 3)
    
    # Fake call to robust caster logic or simplify:
    # Da dies oft nur für kleine Tests genutzt wird, reicht die single-threaded Logik.
    # Der Vollständigkeit halber hier eine kompakte Single-Thread Version.
    if not isinstance(substrate_mesh, pv.PolyData):
        raise TypeError("substrate_mesh muss pv.PolyData sein")
    
    mesh = substrate_mesh.triangulate()
    _ = mesh.obbTree
    face_normals = np.asarray(mesh.face_normals)
    
    N = len(P)
    starts = P - D * (ray_len_mm * 0.5)
    ends   = P + D * (ray_len_mm * 0.5)
    
    # Nutzung des Chunks-Prozessors im Main-Thread
    _, valid, hits, normals, refl = _process_chunk(
        np.arange(N), starts, ends, _normalize(D), 
        mesh.obbTree, face_normals, grazing_cos_min
    )
    
    tcp = hits + normals * float(stand_off_mm)
    return RayProjectResult(valid, hits, normals, refl, tcp)