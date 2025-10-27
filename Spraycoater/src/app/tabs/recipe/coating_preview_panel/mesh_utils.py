# mesh_utils.py
from __future__ import annotations
import os
import re
import math
from typing import Tuple, Optional, Dict

import numpy as np
import pyvista as pv


# ---------- Kontextbasierte Pfad-Helfer ----------
def _project_root(ctx) -> str:
    # bevorzugt ctx.project_root
    if ctx and getattr(ctx, "project_root", None):
        return os.path.abspath(ctx.project_root)
    # Fallback: relativ zu diesem Modul (…/src/app/tabs/recipe/…)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..", ".."))

def _resource_root(ctx) -> str:
    if ctx and getattr(ctx, "resource_root", None):
        return os.path.abspath(ctx.resource_root)
    return os.path.join(_project_root(ctx), "resource")


# ---------- Pfad-Auflösung ----------
def _resolve_mesh_path(uri_or_path: str, ctx=None) -> str:
    """
    Löst Mesh-Pfade robust auf:
      - absolute Pfade
      - package://<pkg>/sub/path  (über ament_index_python)
      - resource/...   (relativ zu Projekt-/Resource-Root)
      - relativ zum Projekt-Root
      - relativ zum CWD
    """
    p = (uri_or_path or "").strip()
    if not p:
        raise FileNotFoundError("Leerer Mesh-Pfad")

    # 1) Absolut?
    if os.path.isabs(p) and os.path.exists(p):
        return p

    # 2) package://<pkg>/sub/path via ament_index_python
    if p.startswith("package://"):
        rest = p[len("package://"):]
        pkg, _, sub = rest.partition("/")
        if not pkg or not sub:
            raise FileNotFoundError(f"Ungültige package-URI: {p}")
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory(pkg)
            cand = os.path.join(share, sub)
            if os.path.exists(cand):
                return cand
        except Exception:
            # weiter unten mit anderen Strategien weitermachen
            pass

    # 3) resource/... → relativ zu project/resource root
    if p.startswith("resource/"):
        cand = os.path.join(_project_root(ctx), p)                 # /proj/resource/...
        if os.path.exists(cand):
            return cand
        cand2 = os.path.join(_resource_root(ctx), p[9:])           # /proj/resource/<rest>
        if os.path.exists(cand2):
            return cand2

    # 4) relativ zu project root
    cand = os.path.join(_project_root(ctx), p)
    if os.path.exists(cand):
        return cand

    # 5) relativ zum CWD
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
    """Rotation (RPY°) + Translation (mm). Gibt eine Kopie zurück."""
    out = mesh.copy(deep=True)
    R = _rpy_deg_to_matrix(rpy_deg)
    out.points[:] = (out.points @ R.T) + np.asarray(translate_mm)[None, :]
    return out


# ---------- Loader ----------
def load_mesh(uri_or_path: str, ctx=None) -> pv.PolyData:
    path = _resolve_mesh_path(uri_or_path, ctx)  # wirft FileNotFoundError bei Auflösungsfehlern
    try:
        mesh = pv.read(path)
    except Exception as e:
        # Datei gefunden, aber Laden schlug fehl.
        raise RuntimeError(f"Mesh-Datei gefunden, aber Laden schlug fehl: {path} ({e})")
    if not isinstance(mesh, pv.PolyData):
        mesh = mesh.extract_surface().triangulate()
    return mesh


# ---------- Mounts ----------
def _def_scene_offset() -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

def _norm_mount_key(key: str) -> str:
    return str(key or "").strip()

def _lookup_mount(ctx, mount_key: Optional[str]) -> Optional[Dict]:
    """Sucht Mount-Eintrag in ctx.mounts_yaml['mounts'] mit tolerantem Abgleich (z. B. _h50-Suffix ignorieren)."""
    my = getattr(ctx, "mounts_yaml", None)
    if not isinstance(my, dict):
        return None
    mounts = my.get("mounts") or {}
    if not isinstance(mounts, dict):
        return None

    # direkter Treffer?
    if mount_key and mount_key in mounts:
        return mounts[mount_key]

    # Toleranter Abgleich: entferne einfache Suffixe wie _h50
    if mount_key:
        base = re.sub(r"_h\d+(\.\d+)?$", "", mount_key)
        if base in mounts:
            return mounts[base]
    return None

def load_active_mount_mesh(ctx) -> pv.PolyData:
    # bevorzugt: ctx.active_mount_key
    key = getattr(ctx, "active_mount_key", None)
    if not key:
        # Fallback: mounts_yaml.active_mount
        my = getattr(ctx, "mounts_yaml", None) or {}
        key = my.get("active_mount")
    if not key:
        raise FileNotFoundError("Kein aktiver Mount gesetzt (ctx.active_mount_key oder mounts_yaml.active_mount).")
    return load_mount_mesh_from_key(ctx, key)

def load_mount_mesh_from_key(ctx, mount_key: str) -> pv.PolyData:
    entry = _lookup_mount(ctx, _norm_mount_key(mount_key))
    if not entry:
        # letzter Versuch: vielleicht ist mount_key direkt eine Datei/URI
        if isinstance(mount_key, str) and (
            mount_key.startswith("package://") or
            os.path.exists(mount_key) or
            mount_key.startswith("resource/")
        ):
            return load_mesh(mount_key, ctx)
        raise FileNotFoundError(f"Mount '{mount_key}' nicht in ctx.mounts_yaml gefunden.")

    mesh_uri = entry.get("mesh")
    if not mesh_uri:
        raise FileNotFoundError(f"Mount '{mount_key}' hat kein 'mesh'-Feld.")
    return load_mesh(mesh_uri, ctx)

def get_mount_scene_offset_from_key(ctx, mount_key: str):
    """
    Liefert (xyz_mm, rpy_deg) für einen bestimmten Mount-Key.
    Nutzt denselben toleranten Lookup wie load_mount_mesh_from_key.
    """
    entry = _lookup_mount(ctx, _norm_mount_key(mount_key))
    if entry and isinstance(entry.get("scene_offset"), dict):
        so = entry["scene_offset"]
        xyz = tuple((so.get("xyz") or [0.0, 0.0, 0.0])[:3])
        rpy = tuple((so.get("rpy_deg") or [0.0, 0.0, 0.0])[:3])
        return xyz, rpy
    # Fallback: globale TF im ctx
    tf = getattr(ctx, "tf_world_to_meca_mount", None)
    if tf and hasattr(tf, "xyz") and hasattr(tf, "rpy_deg"):
        return tuple(tf.xyz), tuple(tf.rpy_deg)
    # Defaults
    return _def_scene_offset()

def get_active_mount_scene_offset(ctx):
    # 1) Versuch: aus mounts_yaml anhand des aktiven Mounts
    my = getattr(ctx, "mounts_yaml", None) or {}
    active = getattr(ctx, "active_mount_key", None) or my.get("active_mount")
    if active:
        return get_mount_scene_offset_from_key(ctx, active)

    # 2) Optional: globaler TF aus ctx
    tf = getattr(ctx, "tf_world_to_meca_mount", None)
    if tf and hasattr(tf, "xyz") and hasattr(tf, "rpy_deg"):
        return tuple(tf.xyz), tuple(tf.rpy_deg)

    # 3) Defaults
    return _def_scene_offset()


# ---------- Substrate Loader (Datei-basiert, kein Parametrik) ----------
def load_substrate_mesh_from_key(ctx, substrate_key: str) -> pv.PolyData:
    """
    Läd ein Substrat-Mesh über folgende Reihenfolge:
      1) Direkt als Pfad/URI (package://, resource/, absolut/relativ)
      2) <PROJECT_ROOT>/resource/substrates/<key>.stl
      3) <share>/mecademic_bringup/resource/substrates/<key>.stl  (install-Share)
    """
    key = (substrate_key or "").strip()
    if not key:
        raise FileNotFoundError("Leerer Substrate-Key.")

    # 1) Direkter Pfad / URI?
    if key.startswith("package://") or key.startswith("resource/") or os.path.isabs(key) or os.path.exists(key):
        return load_mesh(key, ctx)

    # 2) Standard-Ort im Projekt: resource/substrates/<key>.stl
    cand = os.path.join(_project_root(ctx), "resource", "substrates", f"{key}.stl")
    if os.path.exists(cand):
        return load_mesh(cand, ctx)

    # 3) Install-Share des ROS-Pakets 'mecademic_bringup'
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory("mecademic_bringup")
        cand_pkg = os.path.join(share, "resource", "substrates", f"{key}.stl")
        if os.path.exists(cand_pkg):
            return load_mesh(cand_pkg, ctx)
    except Exception:
        pass

    raise FileNotFoundError(f"Mesh nicht gefunden/auflösbar: {substrate_key}")


# ---------- Platzierung ----------
def place_substrate_on_mount(ctx, substrate_mesh: pv.PolyData) -> pv.PolyData:
    """
    Legt das Substrat auf die Mount-Oberfläche:
      - Substrat-Geometrien werden so verschoben, dass ihr z-min auf 0 liegt (Boden auf 0).
      - Danach wird die Mount-Scene-Offset (xyz,rpy) angewandt (z.B. +50 mm).
    """
    sub = substrate_mesh.copy(deep=True)
    # Höhe (axis aligned bbox)
    zmin, zmax = float(sub.bounds[4]), float(sub.bounds[5])

    # 1) Boden auf z=0
    sub.translate((0.0, 0.0, -zmin), inplace=True)

    # 2) Mount-Offset anwenden
    xyz_mm, rpy_deg = get_active_mount_scene_offset(ctx)
    sub = apply_transform(sub, translate_mm=xyz_mm, rpy_deg=rpy_deg)
    return sub
