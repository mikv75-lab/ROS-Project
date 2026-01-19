# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/views_3d/scene_manager.py
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv

# Spraypaths: Geometrie-Generierung & Raycasting
from model.spray_paths.path_builder import PathBuilder
from model.spray_paths.raycast_projector import cast_rays_for_side

# Recipe SSoT: Neue Datenmodelle
from model.spray_paths.draft import Draft, PathSide, PoseQuat
from model.recipe.recipe import Recipe

from .overlays import OverlayOut, OverlayRenderer
from . import mesh_utils

_LOG = logging.getLogger("tabs.recipe.preview.scene_manager")

Bounds = Tuple[float, float, float, float, float, float]
_ALLOWED_SIDES = ("top", "front", "back", "left", "right", "polyhelix", "helix")
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

# ------------------------------------------------------------
# Visueller Stil (Single Source of Truth)
# ------------------------------------------------------------
_LAYER_STYLE: Dict[str, Dict[str, Any]] = {
    "cage": {"color": "#9aa0a6", "opacity": 0.25, "smooth_shading": True, "specular": 0.05},
    "ground": {"color": "#000000", "opacity": 1.00, "smooth_shading": False, "lighting": True, "specular": 0.0},
    "mount": {"color": "#6b6f75", "opacity": 1.00, "smooth_shading": True, "specular": 0.05},
    "substrate": {"color": "#d9d9d9", "opacity": 1.00, "smooth_shading": True, "specular": 0.10},
    "mask": {"color": "#3498db", "line_width": 2.4, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "path": {"color": "#2ecc71", "line_width": 2.8, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "hits": {"color": "#a569bd", "line_width": 1.6, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "misses": {"color": "#e74c3c", "line_width": 1.6, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "tcp_line": {"color": "#5dade2", "line_width": 2.0, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "tcp_x": {"color": "#e74c3c", "line_width": 1.6, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "tcp_y": {"color": "#2ecc71", "line_width": 1.6, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "tcp_z": {"color": "#3498db", "line_width": 1.6, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "normals": {"color": "#a569bd", "line_width": 1.5, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
}

# ------------------------------------------------------------
# Daten-Container
# ------------------------------------------------------------

@dataclass
class PreviewScene:
    bounds: Optional[Bounds]
    substrate_mesh: Optional[pv.PolyData]
    mount_mesh: Optional[pv.PolyData]
    cage_mesh: Optional[pv.PolyData]
    extra_meshes: Dict[str, pv.PolyData]
    meta: Dict[str, Any]


@dataclass
class Renderable:
    layer: str
    name: str
    poly: pv.PolyData
    render_kwargs: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PreviewResult:
    recipe: Optional[Recipe]
    valid: bool
    invalid_reason: Optional[str]
    scene: Optional[PreviewScene]
    substrate_mesh: Optional[pv.PolyData]
    path_xyz_mm: Optional[np.ndarray]
    final_tcp_world_mm: Optional[np.ndarray]
    renderables: List[Renderable]
    bounds: Bounds
    substrate_bounds: Optional[Bounds]
    meta: Dict[str, Any]


# ------------------------------------------------------------
# Interne Helfer (Styling & Geometrie)
# ------------------------------------------------------------

def _render_kwargs_for(layer: str, poly: Any) -> Dict[str, Any]:
    style = dict(_LAYER_STYLE.get(str(layer), {}))

    # Bestimmung der Geometrieart (Fläche, Linie oder Punkt)
    kind = "surface"
    try:
        if hasattr(poly, "lines") and np.asarray(poly.lines).size > 0:
            kind = "lines"
        elif getattr(poly, "n_cells", 0) == 0 and getattr(poly, "n_faces", 0) == 0:
            kind = "points"
    except Exception:
        kind = "surface"

    kwargs: Dict[str, Any] = {
        "color": style.get("color"),
        "opacity": style.get("opacity"),
        "lighting": style.get("lighting", True),
    }

    if kind == "lines":
        kwargs.update(
            {
                "line_width": style.get("line_width"),
                "render_lines_as_tubes": style.get("render_lines_as_tubes"),
            }
        )
    elif kind == "points":
        kwargs.update(
            {
                "point_size": style.get("point_size"),
                "render_points_as_spheres": style.get("render_points_as_spheres"),
            }
        )
    else:
        kwargs.update(
            {
                "smooth_shading": style.get("smooth_shading"),
                "specular": style.get("specular"),
            }
        )

    return {k: v for k, v in kwargs.items() if v is not None}


def _rotation_matrix_from_vectors(x_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    """Erzeugt eine 3x3 Rotationsmatrix aus Tangente (X) und Normale (Z)."""

    def unit(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        return v / n if n > 1e-12 else np.array([1.0, 0.0, 0.0], dtype=float)

    z = unit(np.asarray(z_axis, dtype=float))
    x_raw = np.asarray(x_axis, dtype=float)
    x = unit(x_raw - float(np.dot(x_raw, z)) * z)  # Orthogonalisierung
    y = np.cross(z, x)

    R = np.identity(3, dtype=float)
    R[:, 0], R[:, 1], R[:, 2] = x, y, z
    return R


def _compute_poses(points: np.ndarray, normals: np.ndarray) -> List[PoseQuat]:
    """Konvertiert Punkte und Normalen in eine Liste von PoseQuat Objekten."""
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    N = np.asarray(normals, dtype=float).reshape(-1, 3)
    n = int(min(len(P), len(N)))
    if n < 2:
        return []

    poses: List[PoseQuat] = []
    for i in range(n):
        tangent = (P[i + 1] - P[i]) if i < n - 1 else (P[i] - P[i - 1])
        R = _rotation_matrix_from_vectors(tangent, N[i])

        # Matrix -> Quaternion (robust enough for preview; replace if you have a canonical util)
        tr = float(np.trace(R))
        if tr > 0.0:
            S = math.sqrt(tr + 1.0) * 2.0
            qw = 0.25 * S
            qx = float((R[2, 1] - R[1, 2]) / S)
            qy = float((R[0, 2] - R[2, 0]) / S)
            qz = float((R[1, 0] - R[0, 1]) / S)
        else:
            # Fallback: identity orientation
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        poses.append(
            PoseQuat(
                x=float(P[i, 0]),
                y=float(P[i, 1]),
                z=float(P[i, 2]),
                qx=float(qx),
                qy=float(qy),
                qz=float(qz),
                qw=float(qw),
            )
        )
    return poses


# ------------------------------------------------------------
# SceneManager Klasse
# ------------------------------------------------------------

class SceneManager:
    """Verantwortlich für die Preview-Pipeline: Mesh-Setup, Raycasting und Post-Processing."""

    # OverlayOut-Attribute -> UI-Layer
    _ATTR_TO_LAYER: Dict[str, str] = {
        "path_poly": "path",
        "mask_poly": "mask",
        "rays_hit_poly": "hits",
        "rays_miss_poly": "misses",
        "normals_poly": "normals",
        "tcp_poly": "tcp_line",
        "tcp_x_poly": "tcp_x",
        "tcp_y_poly": "tcp_y",
        "tcp_z_poly": "tcp_z",
    }

    def __init__(self) -> None:
        self._overlay = OverlayRenderer()

    def build_scene(self, recipe: Recipe, ctx: Any = None) -> Optional[PreviewScene]:
        """Lädt und platziert alle statischen Meshes (Halterung, Substrat, Käfig)."""
        if not recipe:
            return None

        mount_mesh: Optional[pv.PolyData] = None
        if getattr(recipe, "substrate_mount", None):
            mount_mesh = mesh_utils.load_mount_mesh_from_key(ctx, recipe.substrate_mount)

        substrate_mesh: Optional[pv.PolyData] = None
        if getattr(recipe, "substrate", None):
            raw_sub = mesh_utils.load_substrate_mesh_from_key(ctx, recipe.substrate)
            if mount_mesh is not None:
                substrate_mesh = mesh_utils.place_substrate_on_mount(ctx, raw_sub, mount_key=recipe.substrate_mount)
            else:
                substrate_mesh = mesh_utils.place_substrate_on_mount_origin(raw_sub)

        cage_mesh: Optional[pv.PolyData] = None
        try:
            cage_mesh = mesh_utils.load_cage_mesh(ctx)
        except Exception as e:
            _LOG.debug("Cage mesh load failed (ignored): %s", e)

        bounds_list: List[Bounds] = []
        for m in (cage_mesh, mount_mesh, substrate_mesh):
            if m is not None:
                try:
                    bounds_list.append(tuple(m.bounds))  # type: ignore[arg-type]
                except Exception:
                    pass

        scene_bounds: Bounds = mesh_utils._union_bounds(bounds_list) if bounds_list else _DEFAULT_BOUNDS

        return PreviewScene(
            bounds=scene_bounds,
            substrate_mesh=substrate_mesh,
            mount_mesh=mount_mesh,
            cage_mesh=cage_mesh,
            extra_meshes={},
            meta={"tool": getattr(recipe, "tool", None), "substrate": getattr(recipe, "substrate", None)},
        )

    def build_preview(self, *, recipe: Recipe, ctx: Any, overlay_cfg: Dict[str, Any]) -> PreviewResult:
        """Haupt-Pipeline: Generiert den Sprühpfad und projiziert ihn auf das Substrat."""
        renderables: List[Renderable] = []

        # 1) Szene vorbereiten
        scene = self.build_scene(recipe, ctx)
        if not scene:
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason="scene_fail",
                scene=None,
                substrate_mesh=None,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                renderables=[],
                bounds=_DEFAULT_BOUNDS,
                substrate_bounds=None,
                meta={},
            )

        # Statische Meshes zu Renderables konvertieren
        for layer_name in ("cage", "mount", "substrate"):
            mesh = getattr(scene, f"{layer_name}_mesh", None)
            if mesh is not None:
                renderables.append(Renderable(layer=layer_name, name=layer_name, poly=mesh, render_kwargs=_render_kwargs_for(layer_name, mesh)))

        # 2) Pfad-Generierung (Lokal)
        side = str(getattr(recipe, "parameters", {}).get("active_side", "top")).lower().strip()
        if side not in _ALLOWED_SIDES:
            side = "top"

        params = getattr(recipe, "parameters", {}) or {}
        pd_mask = PathBuilder.from_side(
            recipe,
            side=side,
            globals_params=params,
            sample_step_mm=float(params.get("sample_step_mm", 1.0)),
            max_points=int(params.get("max_points", 1000)),
        )

        # Pfad auf Start-Höhe heben
        mask_pts = np.asarray(pd_mask.points_mm, dtype=float).reshape(-1, 3).copy()
        z_base = float(scene.substrate_mesh.bounds[5]) if scene.substrate_mesh is not None else 0.0
        z_offset = z_base + float(params.get("stand_off_mm", 50.0))
        if mask_pts.size:
            mask_pts[:, 2] += z_offset

        # 3) Raycast
        if scene.substrate_mesh is None:
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason="no_substrate",
                scene=scene,
                substrate_mesh=None,
                path_xyz_mm=mask_pts,
                final_tcp_world_mm=None,
                renderables=renderables,
                bounds=scene.bounds or _DEFAULT_BOUNDS,
                substrate_bounds=None,
                meta={"side": side},
            )

        rc, hit_p, miss_p, tcp_p = cast_rays_for_side(
            P_world_start=mask_pts,
            sub_mesh_world=scene.substrate_mesh,
            side=side,
            stand_off_mm=float(params.get("stand_off_mm", 50.0)),
        )

        # 4) Validierung & Post-Processing
        valid_mask = getattr(rc, "valid", None)
        if valid_mask is None:
            valid_mask = np.array([], dtype=bool)
        valid_mask = np.asarray(valid_mask, dtype=bool).reshape(-1)

        miss_n = int(len(valid_mask) - int(np.sum(valid_mask))) if len(valid_mask) else 0
        preview_valid = (miss_n == 0)

        final_tcp: Optional[np.ndarray] = None
        final_norms: Optional[np.ndarray] = None

        if preview_valid:
            # Trim & Air-Moves anwenden
            p_side = (getattr(recipe, "paths_by_side", {}) or {}).get(side, {}) or {}
            final_tcp, final_norms = mesh_utils._postprocess_compiled_path_strict(
                getattr(rc, "tcp_mm", None),
                getattr(rc, "normal", None),
                side_path_params=p_side,
            )

            # Ergebnis in Rezept-Draft speichern (FIX: Methode korrekt referenzieren)
            recipe.draft = self._merge_draft_side(
                getattr(recipe, "draft", None),
                side=side,
                poses=_compute_poses(final_tcp, final_norms) if final_tcp is not None and final_norms is not None else [],
            )

        # 5) Visuelle Overlays hinzufügen
        out = self._overlay.render_for_side(
            side=side,
            scene=scene,
            points_local_mm=mask_pts,
            raycast_result=rc,
            hit_poly=hit_p,
            miss_poly=miss_p,
            tcp_poly=tcp_p,
            overlay_cfg=overlay_cfg,
        )

        # OverlayOut -> Renderables (FIX: Layer-Mapping korrekt)
        for attr, layer in self._ATTR_TO_LAYER.items():
            poly = getattr(out, attr, None)
            if poly is None:
                continue
            renderables.append(Renderable(layer=layer, name=attr, poly=poly, render_kwargs=_render_kwargs_for(layer, poly)))

        return PreviewResult(
            recipe=recipe,
            valid=preview_valid,
            invalid_reason=None if preview_valid else "raycast_miss",
            scene=scene,
            substrate_mesh=scene.substrate_mesh,
            path_xyz_mm=mask_pts,
            final_tcp_world_mm=final_tcp,
            renderables=renderables,
            bounds=scene.bounds or _DEFAULT_BOUNDS,
            substrate_bounds=tuple(scene.substrate_mesh.bounds) if scene.substrate_mesh is not None else None,
            meta={"miss_n": miss_n, "side": side},
        )

    @staticmethod
    def _merge_draft_side(
        draft: Optional[Draft],
        *,
        side: str,
        poses: List[PoseQuat],
    ) -> Draft:
        """
        STRICT Draft merge:
        - Draft.version is always 1
        - Only writes draft.sides[side] = PathSide(poses_quat=poses)
        - Leaves other sides untouched
        - If poses is empty -> removes the side (keeps draft if other sides exist)
        """
        s = str(side or "").strip()
        if not s:
            # no-op, but ensure a draft exists
            return draft if isinstance(draft, Draft) else Draft(version=1, sides={})

        # Normalize existing draft
        if not isinstance(draft, Draft):
            cur_sides: Dict[str, PathSide] = {}
            draft_v = Draft(version=1, sides=cur_sides)
        else:
            # copy sides to avoid in-place aliasing issues if Draft is reused elsewhere
            cur = getattr(draft, "sides", None)
            cur_sides = dict(cur) if isinstance(cur, dict) else {}
            draft_v = Draft(version=int(getattr(draft, "version", 1) or 1), sides=cur_sides)

        # Enforce version=1 (your v1 schema)
        draft_v.version = 1

        if not poses:
            # remove side if empty
            if s in draft_v.sides:
                try:
                    del draft_v.sides[s]
                except Exception:
                    draft_v.sides.pop(s, None)
            return draft_v

        draft_v.sides[s] = PathSide(poses_quat=list(poses))
        return draft_v
