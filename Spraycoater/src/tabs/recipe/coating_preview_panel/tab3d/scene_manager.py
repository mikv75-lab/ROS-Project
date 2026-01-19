# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/views_3d/scene_manager.py
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv

from model.spray_paths.path_builder import PathBuilder
from model.spray_paths.raycast_projector import cast_rays_for_side

from model.spray_paths.draft import Draft, PathSide, PoseQuat
from model.recipe.recipe import Recipe

from .overlays import OverlayRenderer
from . import mesh_utils

_LOG = logging.getLogger("tabs.recipe.preview.scene_manager")

Bounds = Tuple[float, float, float, float, float, float]
_ALLOWED_SIDES = ("top", "front", "back", "left", "right", "polyhelix", "helix")
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

_LAYER_STYLE: Dict[str, Dict[str, Any]] = {
    "cage": {"color": "#9aa0a6", "opacity": 0.25, "smooth_shading": True, "specular": 0.05},
    "ground": {"color": "#000000", "opacity": 1.00, "smooth_shading": False, "lighting": True, "specular": 0.0},
    "mount": {"color": "#6b6f75", "opacity": 1.00, "smooth_shading": True, "specular": 0.05},
    "substrate": {"color": "#d9d9d9", "opacity": 1.00, "smooth_shading": True, "specular": 0.10},

    # overlays
    "mask": {"color": "#3498db", "line_width": 2.4, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "path": {"color": "#2ecc71", "line_width": 2.8, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},

    # IMPORTANT: hits/misses are POINT clouds -> need point_size
    "hits": {"color": "#a569bd", "opacity": 1.0, "lighting": False, "point_size": 10.0, "render_points_as_spheres": True},
    "misses": {"color": "#e74c3c", "opacity": 1.0, "lighting": False, "point_size": 10.0, "render_points_as_spheres": True},

    "normals": {"color": "#a569bd", "line_width": 1.5, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},

    # TCP local frame axes at points (no tcp_line: Path draws the line)
    "tcp_x": {"color": "#d62728", "line_width": 1.4, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "tcp_y": {"color": "#2ca02c", "line_width": 1.4, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "tcp_z": {"color": "#1f77b4", "line_width": 1.6, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
}


@dataclass
class PreviewScene:
    bounds: Optional[Bounds]
    substrate_mesh: Optional[pv.PolyData]
    mount_mesh: Optional[pv.PolyData]
    cage_mesh: Optional[pv.PolyData]
    ground_mesh: Optional[pv.PolyData]
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

    # For 2D: substrate + FINAL path in mount-top-frame (KS at mount top)
    substrate_mesh: Optional[pv.PolyData]          # mount-top-frame mesh (for 2D)
    path_xyz_mm: Optional[np.ndarray]              # final path (mount-top-frame)
    final_tcp_world_mm: Optional[np.ndarray]       # kept name for API; here: mount-top-frame final TCP/path

    # For 3D rendering: WORLD renderables
    renderables: List[Renderable]

    # Camera framing in WORLD
    bounds: Bounds
    substrate_bounds: Optional[Bounds]             # mount-top-frame bounds (2D extents)
    meta: Dict[str, Any]


def _render_kwargs_for(layer: str, poly: Any) -> Dict[str, Any]:
    style = dict(_LAYER_STYLE.get(str(layer), {}))

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
                "point_size": style.get("point_size", 8.0),
                "render_points_as_spheres": style.get("render_points_as_spheres", True),
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


def _unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(-1)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < eps:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


def _rotation_matrix_from_vectors(x_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    z = _unit(z_axis)
    x_raw = np.asarray(x_axis, dtype=float).reshape(3)
    x = _unit(x_raw - float(np.dot(x_raw, z)) * z)  # orthogonalize
    y = np.cross(z, x)
    R = np.identity(3, dtype=float)
    R[:, 0], R[:, 1], R[:, 2] = x, y, z
    return R


def _quat_from_rotmat(R: np.ndarray) -> Tuple[float, float, float, float]:
    m = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(m[0, 0] + m[1, 1] + m[2, 2])

    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    else:
        if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
            S = math.sqrt(max(1e-12, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) * 2.0
            qw = (m[2, 1] - m[1, 2]) / S
            qx = 0.25 * S
            qy = (m[0, 1] + m[1, 0]) / S
            qz = (m[0, 2] + m[2, 0]) / S
        elif m[1, 1] > m[2, 2]:
            S = math.sqrt(max(1e-12, 1.0 + m[1, 1] - m[0, 0] - m[2, 2])) * 2.0
            qw = (m[0, 2] - m[2, 0]) / S
            qx = (m[0, 1] + m[1, 0]) / S
            qy = 0.25 * S
            qz = (m[1, 2] + m[2, 1]) / S
        else:
            S = math.sqrt(max(1e-12, 1.0 + m[2, 2] - m[0, 0] - m[1, 1])) * 2.0
            qw = (m[1, 0] - m[0, 1]) / S
            qx = (m[0, 2] + m[2, 0]) / S
            qy = (m[1, 2] + m[2, 1]) / S
            qz = 0.25 * S

    q = np.array([qx, qy, qz, qw], dtype=float)
    qn = float(np.linalg.norm(q))
    if not np.isfinite(qn) or qn < 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    q /= qn
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def _compute_poses(points: np.ndarray, normals: np.ndarray) -> List[PoseQuat]:
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    N = np.asarray(normals, dtype=float).reshape(-1, 3)
    n = int(min(len(P), len(N)))
    if n < 2:
        return []

    poses: List[PoseQuat] = []
    for i in range(n):
        tangent = (P[i + 1] - P[i]) if i < n - 1 else (P[i] - P[i - 1])
        R = _rotation_matrix_from_vectors(tangent, N[i])
        qx, qy, qz, qw = _quat_from_rotmat(R)

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


def _shift_points(P: Optional[np.ndarray], off: Tuple[float, float, float]) -> Optional[np.ndarray]:
    if P is None:
        return None
    A = np.asarray(P, dtype=float).reshape(-1, 3)
    if A.size == 0:
        return A.copy()
    dx, dy, dz = float(off[0]), float(off[1]), float(off[2])
    B = A.copy()
    B[:, 0] -= dx
    B[:, 1] -= dy
    B[:, 2] -= dz
    return B


def _add_points_offset(P_local: np.ndarray, off_world: Tuple[float, float, float]) -> np.ndarray:
    A = np.asarray(P_local, dtype=float).reshape(-1, 3)
    if A.size == 0:
        return A.copy()
    dx, dy, dz = float(off_world[0]), float(off_world[1]), float(off_world[2])
    B = A.copy()
    B[:, 0] += dx
    B[:, 1] += dy
    B[:, 2] += dz
    return B


def _polyline(P: np.ndarray) -> Optional[pv.PolyData]:
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    if P.shape[0] < 2:
        return None
    try:
        return pv.lines_from_points(P, close=False)
    except Exception:
        _LOG.exception("pv.lines_from_points failed")
        return None


class SceneManager:
    """
    Preview pipeline (STRICT):

    WORLD:
      - ground plane
      - mount mesh
      - substrate mesh (placed on mount)
      - cage mesh (optional)

    Compute-frame (mount-top KS):
      - origin = (mount center XY, mount top Z) in WORLD
      - (0,0,0) is mount-top plane
      - 2D receives substrate + final path in this compute-frame
    """

    _ATTR_TO_LAYER: Dict[str, str] = {
        # raycaster debug
        "rays_hit_poly": "hits",
        "rays_miss_poly": "misses",
        "normals_poly": "normals",

        # tcp local frame axes (no tcp_line: Path draws the line)
        "tcp_x_poly": "tcp_x",
        "tcp_y_poly": "tcp_y",
        "tcp_z_poly": "tcp_z",
    }

    def __init__(self) -> None:
        self._overlay = OverlayRenderer()

    def build_scene(self, recipe: Recipe, ctx: Any = None) -> Optional[PreviewScene]:
        if not recipe:
            return None

        mount_mesh: Optional[pv.PolyData] = None
        if getattr(recipe, "substrate_mount", None):
            mount_mesh = mesh_utils.load_mount_mesh_from_key(ctx, recipe.substrate_mount)

        substrate_mesh: Optional[pv.PolyData] = None
        if getattr(recipe, "substrate", None):
            raw_sub = mesh_utils.load_substrate_mesh_from_key(ctx, recipe.substrate)
            if mount_mesh is not None and getattr(recipe, "substrate_mount", None):
                substrate_mesh = mesh_utils.place_substrate_on_mount(ctx, raw_sub, mount_key=recipe.substrate_mount)
            else:
                substrate_mesh = mesh_utils.place_substrate_on_mount_origin(raw_sub)

        cage_mesh: Optional[pv.PolyData] = None
        try:
            cage_mesh = mesh_utils.load_cage_mesh(ctx)
        except Exception as e:
            _LOG.debug("Cage mesh load failed (ignored): %s", e)

        ground_mesh: Optional[pv.PolyData] = None
        if mount_mesh is not None:
            b = mount_mesh.bounds
            xmid = 0.5 * (float(b[0]) + float(b[1]))
            ymid = 0.5 * (float(b[2]) + float(b[3]))
            dx = float(b[1] - b[0])
            dy = float(b[3] - b[2])
            size = max(dx, dy) * 2.2 if max(dx, dy) > 1e-6 else 400.0
            ground_mesh = pv.Plane(
                center=(xmid, ymid, 0.0),
                direction=(0.0, 0.0, 1.0),
                i_size=float(size),
                j_size=float(size),
                i_resolution=1,
                j_resolution=1,
            )

        bounds_list: List[Bounds] = []
        for m in (cage_mesh, ground_mesh, mount_mesh, substrate_mesh):
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
            ground_mesh=ground_mesh,
            extra_meshes={},
            meta={
                "tool": getattr(recipe, "tool", None),
                "substrate": getattr(recipe, "substrate", None),
                "mount": getattr(recipe, "substrate_mount", None),
            },
        )

    def build_preview(self, *, recipe: Recipe, ctx: Any, overlay_cfg: Dict[str, Any]) -> PreviewResult:
        renderables: List[Renderable] = []

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

        # Compute-frame origin = mount center XY, mount top Z in WORLD
        origin_world = (0.0, 0.0, 0.0)
        if scene.mount_mesh is not None:
            b = scene.mount_mesh.bounds
            origin_world = (
                0.5 * (float(b[0]) + float(b[1])),
                0.5 * (float(b[2]) + float(b[3])),
                float(b[5]),
            )

        # Static WORLD renderables
        for layer_name, mesh in (
            ("ground", scene.ground_mesh),
            ("cage", scene.cage_mesh),
            ("mount", scene.mount_mesh),
            ("substrate", scene.substrate_mesh),
        ):
            if mesh is not None:
                renderables.append(
                    Renderable(
                        layer=layer_name,
                        name=layer_name,
                        poly=mesh,
                        render_kwargs=_render_kwargs_for(layer_name, mesh),
                    )
                )

        # Side + params
        side = str(getattr(recipe, "parameters", {}).get("active_side", "top")).lower().strip()
        if side not in _ALLOWED_SIDES:
            side = "top"
        params = getattr(recipe, "parameters", {}) or {}

        # 1) Mask in compute-frame (mount-top KS)
        pd_mask = PathBuilder.from_side(
            recipe,
            side=side,
            globals_params=params,
            sample_step_mm=float(params.get("sample_step_mm", 1.0)),
            max_points=int(params.get("max_points", 1000)),
        )
        mask_local = np.asarray(pd_mask.points_mm, dtype=float).reshape(-1, 3).copy()

        if scene.substrate_mesh is None:
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason="no_substrate",
                scene=scene,
                substrate_mesh=None,
                path_xyz_mm=mask_local,
                final_tcp_world_mm=None,
                renderables=renderables,
                bounds=scene.bounds or _DEFAULT_BOUNDS,
                substrate_bounds=None,
                meta={"side": side, "frame": "mount_top", "origin_world_mm": origin_world},
            )

        # 2) Mask -> WORLD (ray start points). Anchor to substrate top + stand_off.
        sub_top_world_z = float(scene.substrate_mesh.bounds[5])
        stand_off = float(params.get("stand_off_mm", 50.0))

        mask_world = _add_points_offset(mask_local, origin_world)
        if mask_world.size:
            mask_world[:, 2] = sub_top_world_z + stand_off + mask_local[:, 2]

        # Render mask (WORLD)
        mask_poly = _polyline(mask_world)
        if mask_poly is not None:
            renderables.append(
                Renderable(
                    layer="mask",
                    name="mask_poly",
                    poly=mask_poly,
                    render_kwargs=_render_kwargs_for("mask", mask_poly),
                )
            )

        # 3) Raycast (WORLD)
        rc, hit_p, miss_p, _tcp_p_from_projector = cast_rays_for_side(
            P_world_start=mask_world,
            sub_mesh_world=scene.substrate_mesh,
            side=side,
            stand_off_mm=stand_off,
        )

        # STRICT fix: no "or" with numpy arrays
        v_raw = getattr(rc, "valid", None)
        if v_raw is None:
            valid_mask = np.zeros((0,), dtype=bool)
        else:
            valid_mask = np.asarray(v_raw, dtype=bool).reshape(-1)

        miss_n = int(len(valid_mask) - int(np.sum(valid_mask))) if len(valid_mask) else 0
        preview_valid = (miss_n == 0)

        final_tcp_world: Optional[np.ndarray] = None
        final_norms_world: Optional[np.ndarray] = None

        final_tcp_local: Optional[np.ndarray] = None
        substrate_local_mesh: Optional[pv.PolyData] = None
        substrate_bounds_local: Optional[Bounds] = None

        # 4) Substrate for 2D (mount-top KS): copy + shift by origin_world
        try:
            substrate_local_mesh = scene.substrate_mesh.copy(deep=True) if scene.substrate_mesh is not None else None
            if substrate_local_mesh is not None:
                substrate_local_mesh.translate((-origin_world[0], -origin_world[1], -origin_world[2]), inplace=True)
                substrate_bounds_local = tuple(substrate_local_mesh.bounds)  # type: ignore[arg-type]
        except Exception:
            substrate_local_mesh = None
            substrate_bounds_local = None

        if preview_valid:
            p_side = (getattr(recipe, "paths_by_side", {}) or {}).get(side, {}) or {}
            final_tcp_world, final_norms_world = mesh_utils._postprocess_compiled_path_strict(
                getattr(rc, "tcp_mm", None),
                getattr(rc, "normal", None),
                side_path_params=p_side,
            )

            # Render final path (WORLD)
            path_poly = _polyline(final_tcp_world) if final_tcp_world is not None else None
            if path_poly is not None:
                renderables.append(
                    Renderable(
                        layer="path",
                        name="final_path_poly",
                        poly=path_poly,
                        render_kwargs=_render_kwargs_for("path", path_poly),
                    )
                )

            # Store in mount-top KS for 2D + draft
            final_tcp_local = _shift_points(final_tcp_world, origin_world) if final_tcp_world is not None else None

            if final_tcp_local is not None and final_norms_world is not None:
                recipe.draft = self._merge_draft_side(
                    getattr(recipe, "draft", None),
                    side=side,
                    poses=_compute_poses(final_tcp_local, final_norms_world),
                )

        # 5) Overlays (WORLD): ALWAYS build all overlay geometries.
        # UI toggles only set visibility; no rebuild needed.
        cfg_all = dict(overlay_cfg or {})
        cfg_all["visibility"] = {
            "mask": True,
            "path": True,
            "tcp": True,
            "hits": True,
            "misses": True,
            "normals": True,
            "tcp_axes": True,
        }

        empty3 = np.zeros((0, 3), dtype=float)
        out = self._overlay.render_for_side(
            side=side,
            raycast_result=rc,
            hit_poly=hit_p,
            miss_poly=miss_p,
            overlay_cfg=cfg_all,
            mask_points_world_mm=np.asarray(mask_world, dtype=float).reshape(-1, 3) if mask_world is not None else empty3,
            path_points_world_mm=np.asarray(final_tcp_world, dtype=float).reshape(-1, 3) if final_tcp_world is not None else empty3,
            tcp_points_world_mm=np.asarray(final_tcp_world, dtype=float).reshape(-1, 3) if final_tcp_world is not None else empty3,
        )

        for attr, layer in self._ATTR_TO_LAYER.items():
            poly = getattr(out, attr, None)
            if poly is None:
                continue
            renderables.append(
                Renderable(
                    layer=layer,
                    name=attr,
                    poly=poly,
                    render_kwargs=_render_kwargs_for(layer, poly),
                )
            )

        return PreviewResult(
            recipe=recipe,
            valid=preview_valid,
            invalid_reason=None if preview_valid else "raycast_miss",
            scene=scene,
            substrate_mesh=substrate_local_mesh,      # 2D substrate in mount-top-frame
            path_xyz_mm=final_tcp_local,              # 2D final path in mount-top-frame
            final_tcp_world_mm=final_tcp_local,       # kept name, but contract: mount-top-frame
            renderables=renderables,                  # WORLD renderables for 3D
            bounds=scene.bounds or _DEFAULT_BOUNDS,   # WORLD bounds
            substrate_bounds=substrate_bounds_local,  # mount-top-frame bounds for 2D scaling
            meta={
                "miss_n": miss_n,
                "side": side,
                "frame": "mount_top",
                "origin_world_mm": origin_world,
            },
        )

    @staticmethod
    def _merge_draft_side(
        draft: Optional[Draft],
        *,
        side: str,
        poses: List[PoseQuat],
    ) -> Draft:
        s = str(side or "").strip()
        if not s:
            cur_sides: Dict[str, PathSide] = {}
            if isinstance(draft, Draft) and isinstance(getattr(draft, "sides", None), dict):
                cur_sides = dict(getattr(draft, "sides", {}) or {})
            return Draft(version=1, sides=cur_sides)

        cur_sides: Dict[str, PathSide] = {}
        if isinstance(draft, Draft):
            cur = getattr(draft, "sides", None)
            if isinstance(cur, dict):
                cur_sides = dict(cur)

        if not poses:
            cur_sides.pop(s, None)
        else:
            cur_sides[s] = PathSide(poses_quat=list(poses))

        return Draft(version=1, sides=cur_sides)
