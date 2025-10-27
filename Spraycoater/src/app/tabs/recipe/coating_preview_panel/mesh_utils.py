# mesh_utils.py
from __future__ import annotations
import os
import math
from typing import Tuple, Optional, Dict

import numpy as np
import pyvista as pv


# ---------- Kontextbasierte Pfad-Helfer ----------
def _project_root(ctx) -> str:
    if ctx and getattr(ctx, "project_root", None):
        return os.path.abspath(ctx.project_root)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

def _resource_root(ctx) -> str:
    if ctx and getattr(ctx, "resource_root", None):
        return os.path.abspath(ctx.resource_root)
    return os.path.join(_project_root(ctx), "resource")


# ---------- Pfad-Auflösung ----------
def _resolve_mesh_path(uri_or_path: str, ctx=None) -> str:
    """
    Unterstützt:
      - absolute Pfade
      - package://<pkg>/sub/path
      - resource/...
      - relativ zum Projekt-Root
      - relativ zum CWD
    """
    p = (uri_or_path or "").strip()
    if not p:
        raise FileNotFoundError("Leerer Mesh-Pfad")

    if os.path.isabs(p) and os.path.exists(p):
        return p

    if p.startswith("package://"):
        rest = p[len("package://"):]
        pkg, _, sub = rest.partition("/")
        if not pkg or not sub:
            raise FileNotFoundError(f"Ungültige package-URI: {p}")
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(pkg)
        cand = os.path.join(share, sub)
        if os.path.exists(cand):
            return cand
        raise FileNotFoundError(f"package-URI nicht gefunden: {p}")

    if p.startswith("resource/"):
        cand = os.path.join(_project_root(ctx), p)
        if os.path.exists(cand):
            return cand
        cand2 = os.path.join(_resource_root(ctx), p[len("resource/"):])
        if os.path.exists(cand2):
            return cand2
        raise FileNotFoundError(f"resource-Pfad nicht gefunden: {p}")

    cand = os.path.join(_project_root(ctx), p)
    if os.path.exists(cand):
        return cand

    if os.path.exists(p):
        return os.path.abspath(p)

    raise FileNotFoundError(f"Mesh nicht gefunden/auflösbar: {uri_or_path}")


# ---------- Geometrie-Utils ----------
def _rpy_deg_to_matrix(rpy_deg: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = [math.radians(v) for v in rpy_deg]
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])
    return Rz @ Ry @ Rx

def apply_transform(mesh: pv.PolyData,
                    *,
                    translate_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> pv.PolyData:
    out = mesh.copy(deep=True)
    R = _rpy_deg_to_matrix(rpy_deg)
    out.points[:] = (out.points @ R.T) + np.asarray(translate_mm)[None, :]
    return out


# ---------- Loader ----------
def load_mesh(uri_or_path: str, ctx=None) -> pv.PolyData:
    path = _resolve_mesh_path(uri_or_path, ctx)
    try:
        mesh = pv.read(path)
    except Exception as e:
        raise RuntimeError(f"Mesh-Datei gefunden, aber Laden schlug fehl: {path} ({e})")
    if not isinstance(mesh, pv.PolyData):
        mesh = mesh.extract_surface().triangulate()
    return mesh


# ---------- Mounts: strikt aus substrate_mounts.yaml ----------
def _get_mounts_yaml(ctx) -> Dict:
    my = getattr(ctx, "mounts_yaml", None)
    if not isinstance(my, dict):
        raise FileNotFoundError("ctx.mounts_yaml fehlt oder ist ungültig.")
    mounts = my.get("mounts")
    if not isinstance(mounts, dict) or not mounts:
        raise FileNotFoundError("ctx.mounts_yaml['mounts'] fehlt oder ist leer.")
    return mounts

def _get_mount_entry_strict(ctx, mount_key: str) -> Dict:
    key = (mount_key or "").strip()
    if not key:
        raise FileNotFoundError("Mount-Key ist leer.")
    mounts = _get_mounts_yaml(ctx)
    entry = mounts.get(key)
    if not entry:
        raise FileNotFoundError(f"Mount '{key}' nicht in mounts_yaml['mounts'] gefunden.")
    return entry

def load_mount_mesh_from_key(ctx, mount_key: str) -> pv.PolyData:
    entry = _get_mount_entry_strict(ctx, mount_key)
    mesh_uri = entry.get("mesh")
    if not mesh_uri:
        raise FileNotFoundError(f"Mount '{mount_key}' hat kein 'mesh'-Feld.")
    return load_mesh(mesh_uri, ctx)

def get_mount_scene_offset_from_key(ctx, mount_key: str) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    entry = _get_mount_entry_strict(ctx, mount_key)
    so = entry.get("scene_offset")
    if not isinstance(so, dict):
        # kein stilles Raten – explizit fordern
        raise FileNotFoundError(f"Mount '{mount_key}' hat keinen gültigen scene_offset-Eintrag.")
    xyz = tuple((so.get("xyz") or [0.0, 0.0, 0.0])[:3])
    rpy = tuple((so.get("rpy_deg") or [0.0, 0.0, 0.0])[:3])
    return xyz, rpy


# ---------- Substrate Loader (datei-basiert) ----------
def load_substrate_mesh_from_key(ctx, substrate_key: str) -> pv.PolyData:
    key = (substrate_key or "").strip()
    if not key:
        raise FileNotFoundError("Leerer Substrate-Key.")

    # 1) Direkter Pfad / URI?
    if key.startswith("package://") or key.startswith("resource/") or os.path.isabs(key) or os.path.exists(key):
        return load_mesh(key, ctx)

    # 2) Standard-Ort im Projekt
    cand = os.path.join(_project_root(ctx), "resource", "substrates", f"{key}.stl")
    if os.path.exists(cand):
        return load_mesh(cand, ctx)

    # 3) Install-Share (mecademic_bringup)
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("mecademic_bringup")
    cand_pkg = os.path.join(share, "resource", "substrates", f"{key}.stl")
    if os.path.exists(cand_pkg):
        return load_mesh(cand_pkg, ctx)

    raise FileNotFoundError(f"Substrate-Mesh nicht gefunden: {substrate_key}")


# ---------- Platzierung (strikt über mount_key) ----------
def place_substrate_on_mount(ctx, substrate_mesh: pv.PolyData, *, mount_key: Optional[str]) -> pv.PolyData:
    """
    Richtet das Substrat zuerst mit z_min → 0 aus und wendet DANN strikt den
    scene_offset des angegebenen mount_key aus substrate_mounts.yaml an.
    """
    if not mount_key:
        raise FileNotFoundError("place_substrate_on_mount: mount_key ist erforderlich.")

    # Offset strikt aus YAML holen (wirft Fehler, wenn nicht vorhanden)
    xyz_mm, rpy_deg = get_mount_scene_offset_from_key(ctx, mount_key)

    sub = substrate_mesh.copy(deep=True)
    zmin = float(sub.bounds[4])
    sub.translate((0.0, 0.0, -zmin), inplace=True)
    sub = apply_transform(sub, translate_mm=xyz_mm, rpy_deg=rpy_deg)
    return sub
