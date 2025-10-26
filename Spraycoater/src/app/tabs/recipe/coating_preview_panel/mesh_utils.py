# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from typing import Optional, Tuple

import numpy as np
import pyvista as pv

# Wichtig: nicht "src.config.startup", sondern direkt "config.startup".
# Der SRC_ROOT wird in main_gui.py in sys.path eingetragen.
from config.startup import resolve_package_uri as _resolve_pkg


# ---------- Pfad/URI Auflösung ----------

def resolve_uri_from_ctx(uri_or_path: str) -> Optional[str]:
    """
    Auflösung von Mesh-Pfaden:
      - 'package://...'  -> via strict resolver (config.startup.resolve_package_uri)
      - absoluter Pfad   -> wenn existiert
      - relativ          -> nicht erlaubt (liefert None)
    """
    if not uri_or_path:
        return None
    s = str(uri_or_path)

    if s.startswith("package://"):
        try:
            abs_path = _resolve_pkg(s)
            return abs_path if os.path.exists(abs_path) else None
        except Exception:
            return None

    if os.path.isabs(s) and os.path.exists(s):
        return s

    return None  # bewusst strikt


# Rückwärtskompatibler Alias (früher genutzt)
def resolve_pkg_or_abs(uri_or_path: str, *, base_dirs=()) -> Optional[str]:  # noqa: ARG002
    return resolve_uri_from_ctx(uri_or_path)


def load_mesh(uri_or_path: str) -> pv.PolyData:
    """
    Lädt ein STL/OBJ/VTP... via PyVista.
    Erwartet einen bereits aufgelösten absoluten Pfad oder 'package://...'.
    """
    p = resolve_uri_from_ctx(uri_or_path)
    if not p:
        raise FileNotFoundError(f"Mesh nicht gefunden/auflösbar: {uri_or_path}")
    return pv.read(p)


# ---------- Transformationen ----------

def _rpy_deg_to_rot(rpy_deg: Tuple[float, float, float]) -> np.ndarray:
    r, p, y = [np.deg2rad(float(a)) for a in rpy_deg]
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # Rz * Ry * Rx


def apply_transform(mesh: pv.PolyData,
                    translate_mm: Tuple[float, float, float] = (0.0, 0.0, 0.0),
                    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> pv.PolyData:
    """
    Kopie mit Rotation (Rz*Ry*Rx) und anschließender Translation (mm).
    """
    M = mesh.copy(deep=True)
    R = _rpy_deg_to_rot(rpy_deg)
    pts = (M.points @ R.T) + np.asarray(translate_mm, dtype=float)[None, :]
    M.points = pts
    return M


# Rückwärtskompatibler Alias (früher genutzt)
def apply_translation_deg(mesh: pv.PolyData,
                          *,
                          translate_mm: Tuple[float, float, float],
                          rpy_deg: Tuple[float, float, float]) -> pv.PolyData:
    return apply_transform(mesh, translate_mm=translate_mm, rpy_deg=rpy_deg)


# ---------- Mount/Substrate Platzierung ----------

def load_active_mount_mesh(ctx) -> pv.PolyData:
    """
    Nimmt die aktive Mount-Konfiguration aus ctx.mounts_yaml:
      ctx.mounts_yaml['active_mount']
      ctx.mounts_yaml['mounts'][<key>]['mesh'] (package://...)
    """
    if not hasattr(ctx, "mounts_yaml"):
        raise RuntimeError("ctx.mounts_yaml fehlt (bitte startup.yaml + Loader prüfen).")
    active = ctx.mounts_yaml.get("active_mount")
    mounts = ctx.mounts_yaml.get("mounts", {})
    if not active or active not in mounts:
        raise KeyError("Aktiver Mount nicht in mounts_yaml gefunden.")
    mesh_uri = mounts[active].get("mesh")
    if not mesh_uri:
        raise KeyError("Mount-Eintrag hat kein 'mesh'-Feld.")
    return load_mesh(mesh_uri)


def get_active_mount_scene_offset(ctx) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    """
    Gibt (xyz_mm, rpy_deg) aus ctx.mounts_yaml['mounts'][active]['scene_offset'] zurück.
    """
    active = ctx.mounts_yaml.get("active_mount")
    mounts = ctx.mounts_yaml.get("mounts", {})
    entry = mounts.get(active, {})
    off = entry.get("scene_offset", {})
    xyz = tuple(off.get("xyz", (0.0, 0.0, 0.0)))
    rpy = tuple(off.get("rpy_deg", (0.0, 0.0, 0.0)))
    return xyz, rpy


def load_substrate_mesh_from_key(ctx, substrate_key: str) -> pv.PolyData:
    """
    Falls du später eine substrates.yaml hast:
      - substrate_key -> 'mesh' (package://...) dort auflösen
    Bis dahin: wir nehmen den Schlüssel direkt als Pfad/URI an.
    """
    if not substrate_key:
        raise ValueError("substrate_key ist leer.")
    return load_mesh(substrate_key)


def place_substrate_on_mount(ctx, substrate_mesh: pv.PolyData) -> pv.PolyData:
    """
    Positioniert das Substrat basierend auf dem scene_offset der aktiven Mount.
    """
    xyz, rpy = get_active_mount_scene_offset(ctx)
    return apply_transform(substrate_mesh, translate_mm=xyz, rpy_deg=rpy)
