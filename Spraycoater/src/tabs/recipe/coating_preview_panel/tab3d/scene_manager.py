# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/views_3d/scene_manager.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, List

import pyvista as pv

from . import mesh_utils

_LOG = logging.getLogger("tabs.recipe.preview.scene_manager")

Bounds = Tuple[float, float, float, float, float, float]


@dataclass
class PreviewScene:
    """
    Container für die fertig geladene und platzierte Szene.
    Alles hier ist bereits im World-Frame (mm).
    """
    bounds: Optional[Bounds]
    substrate_mesh: Optional[pv.PolyData]
    mount_mesh: Optional[pv.PolyData]
    cage_mesh: Optional[pv.PolyData]
    extra_meshes: Dict[str, pv.PolyData]
    meta: Dict[str, Any]


def _get(obj: Any, key: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _union_bounds(bounds_list: List[Bounds]) -> Optional[Bounds]:
    if not bounds_list:
        return None
    xs = [b[0] for b in bounds_list] + [b[1] for b in bounds_list]
    ys = [b[2] for b in bounds_list] + [b[3] for b in bounds_list]
    zs = [b[4] for b in bounds_list] + [b[5] for b in bounds_list]
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))


class SceneManager:
    """
    Sorgt dafür, dass die Strings aus dem Rezept (z.B. 'cube') 
    in echte 3D-Objekte verwandelt werden, unter Verwendung von mesh_utils.
    """

    def __init__(self) -> None:
        pass

    def build_scene(self, recipe: Any, ctx: Any = None) -> Optional[PreviewScene]:
        """
        Lädt Meshes basierend auf Recipe-Keys via mesh_utils.
        
        :param recipe: Das Recipe-Objekt.
        :param ctx: Application Context (wichtig für mesh_utils Pfad-Auflösung).
        """
        if recipe is None:
            return None

        # 1. Keys auslesen (Strings aus dem Rezept)
        tool_key = str(_get(recipe, "tool") or "").strip()
        sub_key = str(_get(recipe, "substrate") or "").strip()
        mount_key = str(_get(recipe, "substrate_mount") or "").strip()

        _LOG.debug(f"Build Scene: Tool='{tool_key}', Sub='{sub_key}', Mount='{mount_key}'")

        cage_mesh: Optional[pv.PolyData] = None
        mount_mesh: Optional[pv.PolyData] = None
        substrate_mesh: Optional[pv.PolyData] = None
        extras: Dict[str, pv.PolyData] = {}

        # 2. Mount laden (falls vorhanden)
        if mount_key and mount_key.lower() != "none":
            try:
                # Lädt Mount und normalisiert ihn auf Z=0 (Floor) und XY-Center
                mount_mesh = mesh_utils.load_mount_mesh_from_key(ctx, mount_key)
            except Exception as e:
                _LOG.warning(f"Konnte Mount '{mount_key}' nicht laden: {e}")

        # 3. Substrat laden & platzieren
        if sub_key and sub_key.lower() != "none":
            try:
                # Roh-Mesh laden
                raw_sub = mesh_utils.load_substrate_mesh_from_key(ctx, sub_key)
                
                if mount_mesh is not None and mount_key:
                    # FALL A: Mount ist da -> Substrat relativ zum Mount platzieren
                    # (Liest scene_offset aus substrate_mounts.yaml via mesh_utils)
                    substrate_mesh = mesh_utils.place_substrate_on_mount(ctx, raw_sub, mount_key=mount_key)
                else:
                    # FALL B: Kein Mount -> Substrat einfach auf Z=0 legen (Origin)
                    substrate_mesh = mesh_utils.place_substrate_on_mount_origin(raw_sub, offset_z=0.0)
            
            except Exception as e:
                _LOG.warning(f"Konnte Substrat '{sub_key}' nicht laden/platzieren: {e}")

        # 4. Tool / Cage (Optional)
        # Wenn nötig, hier später Tool-Meshes laden.
        
        # 5. Bounds berechnen
        bounds_list = []
        if substrate_mesh: bounds_list.append(substrate_mesh.bounds)
        if mount_mesh: bounds_list.append(mount_mesh.bounds)
        
        scene_bounds = _union_bounds(bounds_list)

        # Warnung, wenn Szene leer ist
        if substrate_mesh is None and mount_mesh is None:
            _LOG.warning("Szene ist leer (Kein Substrat, kein Mount geladen).")

        return PreviewScene(
            bounds=scene_bounds,
            substrate_mesh=substrate_mesh,
            mount_mesh=mount_mesh,
            cage_mesh=cage_mesh,
            extra_meshes=extras,
            meta={
                "tool": tool_key,
                "substrate": sub_key,
                "mount": mount_key
            }
        )