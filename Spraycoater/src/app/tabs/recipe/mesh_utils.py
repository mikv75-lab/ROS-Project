# Spraycoater/src/app/scene/mesh_utils.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from typing import Optional, Dict, Any, Tuple
import numpy as np
import pyvista as pv

# ---- RPY/Transforms (mm) ----
def rpy_deg_to_R(rpy_deg):
    import math
    r, p, y = [math.radians(float(a)) for a in rpy_deg]
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]], float)
    Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]], float)
    Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]], float)
    return Rz @ Ry @ Rx

def make_T_mm(xyz_mm=(0,0,0), rpy_deg=(0,0,0)):
    T = np.eye(4)
    T[:3,:3] = rpy_deg_to_R(rpy_deg)
    T[:3, 3] = np.array(xyz_mm, float)
    return T

def T_mm_to_m(Tmm: np.ndarray) -> np.ndarray:
    Tm = Tmm.copy()
    Tm[:3,3] = Tm[:3,3] * 0.001
    return Tm

def apply_T_mesh_mm(mesh: pv.PolyData, Tmm: np.ndarray) -> pv.PolyData:
    m = mesh.copy(deep=True)
    R, t = Tmm[:3,:3], Tmm[:3,3]
    m.points = (m.points @ R.T) + t
    return m

# ---- package:// Auflösung (keine ROS-Abhängigkeit) ----
def resolve_uri(path_or_uri: str, *, package_map: Optional[Dict[str,str]] = None) -> str:
    if not str(path_or_uri).startswith("package://"):
        return path_or_uri
    pkg, rel = path_or_uri.replace("package://", "", 1).split("/", 1)
    if not package_map or pkg not in package_map:
        raise FileNotFoundError(f"Unbekanntes ROS-Package '{pkg}' für {path_or_uri}")
    return os.path.join(package_map[pkg], rel)

# ---- Mesh IO ----
def load_mesh(path_or_uri: str, *, package_map: Optional[Dict[str,str]] = None) -> pv.PolyData:
    fp = resolve_uri(path_or_uri, package_map=package_map)
    mesh = pv.read(fp)
    if not isinstance(mesh, pv.PolyData):
        mesh = mesh.extract_geometry()
    return mesh  # Einheiten: wir interpretieren Koordinaten als mm

# ---- Szene: Mount & Substrat (mm) ----
def mount_mesh_with_scene_offset_mm(mount_cfg: Dict[str,Any], *, package_map=None) -> Tuple[pv.PolyData, np.ndarray, np.ndarray]:
    """
    Returns:
      mount_mesh_mm          : pv.PolyData (in mm)
      T_world_scene_mm (4x4) : Scene-Frame relativ Welt (mm)
      T_world_scene_m  (4x4) : dasselbe in Metern (für Traj-Builder/ROS)
    """
    mesh = load_mesh(mount_cfg["mesh"], package_map=package_map)
    off = mount_cfg.get("scene_offset", {}) or {}
    xyz_mm  = off.get("xyz", [0,0,0])      # <-- mm!
    rpy_deg = off.get("rpy_deg", [0,0,0])

    T_world_scene_mm = make_T_mm(xyz_mm, rpy_deg)
    T_world_scene_m  = T_mm_to_m(T_world_scene_mm)

    # Mount-Mesh selbst bleibt am Weltursprung (so wie STL modelliert wurde).
    return mesh, T_world_scene_mm, T_world_scene_m

def place_substrate_in_scene_mm(sub_cfg: Dict[str,Any], *, package_map=None) -> pv.PolyData:
    """
    Optional kann das Substrat eine 'scene_pose' in mm besitzen.
    """
    mesh = load_mesh(sub_cfg["mesh"], package_map=package_map)
    pose = sub_cfg.get("scene_pose") or {}
    Tmm = make_T_mm(pose.get("xyz",[0,0,0]), pose.get("rpy_deg",[0,0,0]))
    return apply_T_mesh_mm(mesh, Tmm)
