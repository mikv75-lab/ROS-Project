# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/views_3d/scene_manager.py
from __future__ import annotations

import logging
import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pyvista as pv

# Spraypaths (new folder): geometry generation + raycast
from model.spraypaths.path_builder import PathBuilder
from model.spraypaths.raycast_projector import cast_rays_for_side

# Recipe SSoT (new recipe folder structure):
# Draft is alias to Path; PoseQuat + PathSide defined here.
from model.spraypaths.draft import Draft, PathSide, PoseQuat

from .overlays import OverlayOut, OverlayRenderer
from . import mesh_utils

_LOG = logging.getLogger("tabs.recipe.preview.scene_manager")

Bounds = Tuple[float, float, float, float, float, float]
_ALLOWED_SIDES = ("top", "front", "back", "left", "right", "polyhelix", "helix")
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

# ------------------------------------------------------------
# Visual style (SSoT)
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


def _layer_style(layer: str) -> Dict[str, Any]:
    return dict(_LAYER_STYLE.get(str(layer), {}))


def _poly_kind(poly: Any) -> str:
    """Best-effort classification for render kwargs."""
    try:
        if hasattr(poly, "lines"):
            arr = np.asarray(getattr(poly, "lines"), dtype=np.int64).ravel()
            if arr.size > 0:
                return "lines"
    except Exception:
        pass
    try:
        n_cells = int(getattr(poly, "n_cells", 0) or 0)
        n_faces = int(getattr(poly, "n_faces", 0) or 0)
        if n_cells == 0 and n_faces == 0:
            return "points"
    except Exception:
        pass
    return "surface"


def _render_kwargs_for(layer: str, poly: Any) -> Dict[str, Any]:
    style = _layer_style(layer)
    kind = _poly_kind(poly)
    kwargs: Dict[str, Any] = {}

    if "color" in style and style.get("color") is not None:
        kwargs["color"] = str(style["color"])
    if "opacity" in style and style.get("opacity") is not None:
        kwargs["opacity"] = float(style["opacity"])
    if "lighting" in style and style.get("lighting") is not None:
        kwargs["lighting"] = bool(style["lighting"])

    if kind == "lines":
        if "line_width" in style and style.get("line_width") is not None:
            kwargs["line_width"] = float(style["line_width"])
        if "render_lines_as_tubes" in style and style.get("render_lines_as_tubes") is not None:
            kwargs["render_lines_as_tubes"] = bool(style["render_lines_as_tubes"])
    elif kind == "points":
        if "point_size" in style and style.get("point_size") is not None:
            kwargs["point_size"] = float(style["point_size"])
        if "render_points_as_spheres" in style and style.get("render_points_as_spheres") is not None:
            kwargs["render_points_as_spheres"] = bool(style["render_points_as_spheres"])
    else:
        if "smooth_shading" in style and style.get("smooth_shading") is not None:
            kwargs["smooth_shading"] = bool(style["smooth_shading"])
        if "specular" in style and style.get("specular") is not None:
            kwargs["specular"] = float(style["specular"])
    return kwargs


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
    recipe: Any
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


def _get(obj: Any, key: str, default: Any = None) -> Any:
    if isinstance(obj, dict):
        return obj.get(key, default)
    return getattr(obj, key, default)


def _set(obj: Any, key: str, value: Any) -> None:
    if isinstance(obj, dict):
        obj[key] = value
    else:
        setattr(obj, key, value)


def _union_bounds(bounds_list: List[Bounds]) -> Optional[Bounds]:
    if not bounds_list:
        return None
    xs = [b[0] for b in bounds_list] + [b[1] for b in bounds_list]
    ys = [b[2] for b in bounds_list] + [b[3] for b in bounds_list]
    zs = [b[4] for b in bounds_list] + [b[5] for b in bounds_list]
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _clamp_nonneg(v: Any) -> float:
    try:
        return max(0.0, float(v))
    except Exception:
        return 0.0


def _make_ground_box(bounds: Bounds, *, z_under: float, pad_factor: float = 3.0, thickness_mm: float = 0.6) -> pv.PolyData:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    dx = max(10.0, xmax - xmin)
    dy = max(10.0, ymax - ymin)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    hx = 0.5 * dx * float(pad_factor)
    hy = 0.5 * dy * float(pad_factor)
    t = max(0.1, float(thickness_mm))
    return pv.Box(bounds=(cx - hx, cx + hx, cy - hy, cy + hy, float(z_under) - t, float(z_under)))


def _safe_unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < eps:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


def _poly_segments(starts: np.ndarray, dirs: np.ndarray, length: float) -> Optional[pv.PolyData]:
    S = np.asarray(starts, dtype=float).reshape(-1, 3)
    D = np.asarray(dirs, dtype=float).reshape(-1, 3)
    n = min(S.shape[0], D.shape[0])
    if n <= 0:
        return None
    S = S[:n]
    D = D[:n]
    Dn = np.empty_like(D)
    for i in range(n):
        Dn[i] = _safe_unit(D[i])
    E = S + Dn * float(length)
    pts = np.vstack([S, E])
    lines = np.empty((n, 3), dtype=np.int64)
    lines[:, 0] = 2
    lines[:, 1] = np.arange(0, n, dtype=np.int64)
    lines[:, 2] = np.arange(n, 2 * n, dtype=np.int64)
    poly = pv.PolyData(pts)
    poly.lines = lines.reshape(-1)
    return poly


# ------------------------------------------------------------
# Math helpers: Pose & Rotation
# ------------------------------------------------------------
def _rotation_matrix_from_vectors(x_axis: np.ndarray, z_axis: np.ndarray) -> np.ndarray:
    """Construct a 3x3 rotation matrix from X (tangent) and Z (normal)."""
    z = _safe_unit(z_axis)
    x = _safe_unit(x_axis)

    # Orthonormalize x against z
    x = x - np.dot(x, z) * z
    if float(np.linalg.norm(x)) < 1e-6:
        # Singularity: x parallel to z. Choose arbitrary basis.
        y_temp = np.array([0.0, 1.0, 0.0], dtype=float) if abs(float(z[1])) < 0.9 else np.array([1.0, 0.0, 0.0], dtype=float)
        x = _safe_unit(np.cross(y_temp, z))
    else:
        x = _safe_unit(x)

    y = np.cross(z, x)

    # Rotation matrix [col0, col1, col2] = [X, Y, Z]
    R = np.identity(3)
    R[:, 0] = x
    R[:, 1] = y
    R[:, 2] = z
    return R


def _rot_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to quaternion (qx, qy, qz, qw)."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    tr = float(R[0, 0] + R[1, 1] + R[2, 2])

    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S

    return (float(qx), float(qy), float(qz), float(qw))


def _compute_poses(points: np.ndarray, normals: np.ndarray) -> List[PoseQuat]:
    """Generate PoseQuat list from points and normals."""
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    N = np.asarray(normals, dtype=float).reshape(-1, 3)
    n = min(P.shape[0], N.shape[0])
    if n < 2:
        return []

    P = P[:n]
    N = N[:n]

    poses: List[PoseQuat] = []
    for i in range(n):
        p = P[i]
        norm = N[i]
        if i < n - 1:
            tangent = P[i + 1] - P[i]
        else:
            tangent = P[i] - P[i - 1]

        R = _rotation_matrix_from_vectors(tangent, norm)
        qx, qy, qz, qw = _rot_to_quat(R)
        poses.append(
            PoseQuat(
                x=float(p[0]),
                y=float(p[1]),
                z=float(p[2]),
                qx=float(qx),
                qy=float(qy),
                qz=float(qz),
                qw=float(qw),
            )
        )

    return poses


def _tcp_frames_from_tangent_normal(
    tcp_mm: np.ndarray,
    normals: np.ndarray,
    *,
    max_frames: int = 160,
    scale_mm: float = 8.0,
) -> Tuple[Optional[pv.PolyData], Optional[pv.PolyData], Optional[pv.PolyData]]:
    """Build axis line segments for visualization."""
    P = np.asarray(tcp_mm, dtype=float).reshape(-1, 3)
    N = np.asarray(normals, dtype=float).reshape(-1, 3)
    n = min(P.shape[0], N.shape[0])
    if n < 2:
        return None, None, None

    P = P[:n]
    N = N[:n]

    if max_frames <= 0 or n <= max_frames:
        idx = np.arange(n, dtype=int)
    else:
        idx = np.linspace(0, n - 1, num=int(max_frames), dtype=int)

    origins = P[idx]
    tangents = np.zeros_like(origins)
    normals_sel = np.zeros_like(origins)

    for k, i in enumerate(idx.tolist()):
        i0 = max(0, i - 1)
        i1 = min(n - 1, i + 1)
        tangents[k] = _safe_unit(P[i1] - P[i0])
        normals_sel[k] = _safe_unit(N[i])

    z_dirs = normals_sel
    x_dirs = tangents
    y_dirs = np.cross(z_dirs, x_dirs)

    # Re-orthogonalize frames
    for i in range(y_dirs.shape[0]):
        if float(np.linalg.norm(y_dirs[i])) < 1e-9:
            y_dirs[i] = _safe_unit(np.cross(z_dirs[i], np.array([0.0, 1.0, 0.0], dtype=float)))
        else:
            y_dirs[i] = _safe_unit(y_dirs[i])
        x_dirs[i] = _safe_unit(np.cross(y_dirs[i], z_dirs[i]))

    px = _poly_segments(origins, x_dirs, float(scale_mm))
    py = _poly_segments(origins, y_dirs, float(scale_mm))
    pz = _poly_segments(origins, z_dirs, float(scale_mm))
    return px, py, pz


def _airmove_offset_mm(side_path_params: Any, *, kind: str) -> float:
    """
    Normalize airmove offsets across schema variants.
    """
    if not isinstance(side_path_params, dict):
        return 0.0

    k = str(kind).strip().lower()
    if k not in ("predispense", "retreat"):
        return 0.0

    legacy_key = f"{k}_offset_mm"
    try:
        v = side_path_params.get(legacy_key, None)
        if v is not None:
            return _clamp_nonneg(v)
    except Exception:
        pass

    try:
        blk = side_path_params.get(k, None)
        if isinstance(blk, dict):
            if "offset_mm" in blk:
                return _clamp_nonneg(blk.get("offset_mm"))
            if "extend_mm" in blk:
                return _clamp_nonneg(blk.get("extend_mm"))
    except Exception:
        pass

    return 0.0


def _postprocess_compiled_path_strict(
    P: np.ndarray,
    N: np.ndarray,
    *,
    side_path_params: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray]:
    """Apply trim + airmoves AFTER raycast, operating on points and normals."""
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    N = np.asarray(N, dtype=float).reshape(-1, 3)

    if P.shape[0] < 2 or P.shape[0] != N.shape[0]:
        return P, N

    # 1) Trim (simple index filtering on cumulative arclength)
    start_off = _clamp_nonneg(side_path_params.get("start_offset_mm", 0.0))
    end_off = _clamp_nonneg(side_path_params.get("end_offset_mm", 0.0))

    if start_off > 0.0 or end_off > 0.0:
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total = float(s[-1]) if s.size else 0.0

        if total > 1e-9:
            a = float(start_off)
            b = max(a, total - float(end_off))
            if b - a > 1e-9:
                mask = (s >= a) & (s <= b)
                if int(np.sum(mask)) >= 2:
                    P = P[mask]
                    N = N[mask]

    if P.shape[0] < 2:
        return P, N

    # 2) Extend (Air Moves) — schema normalized
    pre_mm = _airmove_offset_mm(side_path_params, kind="predispense")
    post_mm = _airmove_offset_mm(side_path_params, kind="retreat")

    out_P = P
    out_N = N

    if pre_mm > 0.0:
        v0 = out_P[0] - out_P[1]
        n0 = float(np.linalg.norm(v0))
        if n0 > 1e-9:
            p_pre = out_P[0] + (v0 / n0) * float(pre_mm)
            out_P = np.vstack([p_pre[None, :], out_P])
            out_N = np.vstack([out_N[0][None, :], out_N])

    if post_mm > 0.0:
        v1 = out_P[-1] - out_P[-2]
        n1 = float(np.linalg.norm(v1))
        if n1 > 1e-9:
            p_post = out_P[-1] + (v1 / n1) * float(post_mm)
            out_P = np.vstack([out_P, p_post[None, :]])
            out_N = np.vstack([out_N, out_N[-1][None, :]])

    return out_P, out_N


def _force_all_overlay_cfg(cfg: Any) -> Dict[str, Any]:
    base = dict(cfg) if isinstance(cfg, dict) else {}
    for k in ("mask", "path", "tcp", "hits", "misses", "normals"):
        base[k] = True
    return base


def _merge_draft_side(existing: Any, *, side: str, poses: List[PoseQuat]) -> Draft:
    """
    Update/merge Draft.sides[side] without discarding other sides.

    Recipe SSoT:
      - Draft == Path (alias) from model.recipe.recipe
      - Draft.from_yaml_dict() exists (strict v1)
    """
    s = str(side)

    # 1) Draft instance
    if isinstance(existing, Draft):
        sides = dict(existing.sides or {})
        sides[s] = PathSide(poses_quat=list(poses))
        return Draft(version=int(existing.version or 1), sides=sides)

    # 2) YAML dict -> Draft
    if isinstance(existing, dict):
        try:
            ex = Draft.from_yaml_dict(existing, name="draft")  # type: ignore[arg-type]
            sides = dict(ex.sides or {})
            sides[s] = PathSide(poses_quat=list(poses))
            return Draft(version=int(ex.version or 1), sides=sides)
        except Exception:
            # fall through to minimal reconstruction
            pass

    # 3) New Draft
    return Draft(version=1, sides={s: PathSide(poses_quat=list(poses))})


class SceneManager:
    """Preview pipeline owner (strict). Responsible for geometry generation and draft updates."""

    def __init__(self) -> None:
        self._overlay = OverlayRenderer()

    # ------------------------------------------------------------
    # Scene build (meshes + placement)
    # ------------------------------------------------------------
    def build_scene(self, recipe: Any, ctx: Any = None) -> Optional[PreviewScene]:
        if recipe is None:
            return None

        tool_key = str(_get(recipe, "tool") or "").strip()
        sub_key = str(_get(recipe, "substrate") or "").strip()
        mount_key = str(_get(recipe, "substrate_mount") or "").strip()

        cage_mesh: Optional[pv.PolyData] = None
        mount_mesh: Optional[pv.PolyData] = None
        substrate_mesh: Optional[pv.PolyData] = None
        extras: Dict[str, pv.PolyData] = {}

        try:
            fn = getattr(mesh_utils, "load_cage_mesh", None)
            if callable(fn):
                cage_mesh = fn(ctx)
        except Exception as e:
            _LOG.debug("cage mesh load skipped/failed: %s", e)

        if mount_key and mount_key.lower() != "none":
            try:
                mount_mesh = mesh_utils.load_mount_mesh_from_key(ctx, mount_key)
            except Exception as e:
                _LOG.warning("Konnte Mount '%s' nicht laden: %s", mount_key, e)

        if sub_key and sub_key.lower() != "none":
            try:
                raw_sub = mesh_utils.load_substrate_mesh_from_key(ctx, sub_key)
                if mount_mesh is not None and mount_key:
                    substrate_mesh = mesh_utils.place_substrate_on_mount(ctx, raw_sub, mount_key=mount_key)
                else:
                    substrate_mesh = mesh_utils.place_substrate_on_mount_origin(raw_sub, offset_z=0.0)
            except Exception as e:
                _LOG.warning("Konnte Substrat '%s' nicht laden/platzieren: %s", sub_key, e)

        bounds_list: List[Bounds] = []
        if cage_mesh is not None:
            bounds_list.append(tuple(cage_mesh.bounds))
        if mount_mesh is not None:
            bounds_list.append(tuple(mount_mesh.bounds))
        if substrate_mesh is not None:
            bounds_list.append(tuple(substrate_mesh.bounds))

        scene_bounds = _union_bounds(bounds_list)

        return PreviewScene(
            bounds=scene_bounds,
            substrate_mesh=substrate_mesh,
            mount_mesh=mount_mesh,
            cage_mesh=cage_mesh,
            extra_meshes=extras,
            meta={"tool": tool_key, "substrate": sub_key, "mount": mount_key},
        )

    # ------------------------------------------------------------
    # Preview pipeline (ORDER: Raycast -> Offset -> Pose -> Visualization)
    # ------------------------------------------------------------
    def build_preview(
        self,
        *,
        recipe: Any,
        ctx: Any,
        overlay_cfg: Dict[str, Any],
    ) -> PreviewResult:
        renderables: List[Renderable] = []
        miss_n = 0

        if recipe is None:
            return PreviewResult(
                recipe=None,
                valid=False,
                invalid_reason="no_preview",
                scene=None,
                substrate_mesh=None,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                renderables=[],
                bounds=_DEFAULT_BOUNDS,
                substrate_bounds=None,
                meta={},
            )

        overlay_cfg_all = _force_all_overlay_cfg(overlay_cfg)

        # 1) Build scene
        try:
            scene = self.build_scene(recipe, ctx=ctx)
        except Exception as e:
            _LOG.exception("Scene build failed")
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=f"scene_build_failed: {e}",
                scene=None,
                substrate_mesh=None,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                renderables=[],
                bounds=_DEFAULT_BOUNDS,
                substrate_bounds=None,
                meta={},
            )

        substrate_mesh = scene.substrate_mesh if scene else None

        # 1b) Base meshes -> renderables
        try:
            if scene and scene.cage_mesh is not None:
                poly = scene.cage_mesh
                renderables.append(Renderable("cage", "cage", poly, _render_kwargs_for("cage", poly)))

            if scene and scene.mount_mesh is not None:
                poly = scene.mount_mesh
                renderables.append(Renderable("mount", "mount", poly, _render_kwargs_for("mount", poly)))
                try:
                    b = scene.mount_mesh.bounds
                    ground = _make_ground_box(
                        (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), float(b[5])),
                        z_under=float(b[4]) - 0.30,
                        pad_factor=3.2,
                    )
                    renderables.append(Renderable("ground", "ground", ground, _render_kwargs_for("ground", ground)))
                except Exception:
                    pass

            if substrate_mesh is not None:
                poly = substrate_mesh
                renderables.append(Renderable("substrate", "substrate", poly, _render_kwargs_for("substrate", poly)))
        except Exception:
            _LOG.exception("base renderables failed")

        # 2) Params
        side = str(_get(recipe, "active_side", None) or "top").lower()
        if side not in _ALLOWED_SIDES:
            side = "top"

        globals_params = dict(_get(recipe, "parameters", {}) or {})
        sample_step_mm = _as_float(globals_params.get("sample_step_mm", 1.0), 1.0)
        max_points = int(globals_params.get("max_points", 1000) or 1000)
        stand_off_mm = _as_float(globals_params.get("stand_off_mm", 50.0), 50.0)

        # 3) Mask path generation (local path -> lifted to stand-off plane)
        try:
            pbs = _get(recipe, "paths_by_side", {}) or {}
            p_side = dict((pbs.get(side) if isinstance(pbs, dict) else {}) or {})

            pd_mask = PathBuilder.from_side(
                recipe,
                side=side,
                globals_params=globals_params,
                sample_step_mm=float(sample_step_mm),
                max_points=int(max_points),
                include_airmoves=False,  # strict: post-raycast only
            )
            mask_points_mm = np.asarray(pd_mask.points_mm, dtype=float).reshape(-1, 3)

            base_z = float(substrate_mesh.bounds[5]) if substrate_mesh is not None else 0.0
            if mask_points_mm.shape[0] > 0:
                mask_points_mm = mask_points_mm.copy()
                mask_points_mm[:, 2] += base_z + float(stand_off_mm)
                mask_points_mm[:, 2] += _as_float(p_side.get("z_mm", 0.0))
        except Exception as e:
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=f"path_build_failed: {e}",
                scene=scene,
                substrate_mesh=substrate_mesh,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                renderables=renderables,
                bounds=(scene.bounds if scene and scene.bounds else _DEFAULT_BOUNDS),
                substrate_bounds=(tuple(substrate_mesh.bounds) if substrate_mesh is not None else None),
                meta={"side": side},
            )

        # Render mask
        if mask_points_mm.shape[0] >= 2:
            poly = pv.lines_from_points(mask_points_mm, close=False)
            renderables.append(Renderable("mask", "mask", poly, _render_kwargs_for("mask", poly)))

        # 4) Raycast (Calculation Phase I)
        if substrate_mesh is None:
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason="no_substrate_mesh",
                scene=scene,
                substrate_mesh=None,
                path_xyz_mm=mask_points_mm,
                final_tcp_world_mm=None,
                renderables=renderables,
                bounds=(scene.bounds if scene and scene.bounds else _DEFAULT_BOUNDS),
                substrate_bounds=None,
                meta={"side": side, "miss_n": 0},
            )

        try:
            rc, hit_poly, miss_poly, tcp_poly = cast_rays_for_side(
                P_world_start=mask_points_mm,
                sub_mesh_world=substrate_mesh,
                side=side,
                source="",
                stand_off_mm=float(stand_off_mm),
                invert_dirs=bool(globals_params.get("invert_dirs", False)),
            )
        except Exception as e:
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=f"raycast_failed: {e}",
                scene=scene,
                substrate_mesh=substrate_mesh,
                path_xyz_mm=mask_points_mm,
                final_tcp_world_mm=None,
                renderables=renderables,
                bounds=(scene.bounds if scene and scene.bounds else _DEFAULT_BOUNDS),
                substrate_bounds=(tuple(substrate_mesh.bounds) if substrate_mesh is not None else None),
                meta={"side": side},
            )

        valid_mask = np.asarray(getattr(rc, "valid", []), dtype=bool).reshape(-1)
        miss_n = int(valid_mask.size - int(np.sum(valid_mask)))
        preview_valid = (miss_n == 0)
        invalid_reason = None if preview_valid else f"raycast_misses(count={miss_n})"

        # 5) Postprocess & Poses (Calculation Phase II)
        final_tcp_world_mm: Optional[np.ndarray] = None
        final_norms: Optional[np.ndarray] = None

        if preview_valid:
            try:
                tcp_mm = np.asarray(getattr(rc, "tcp_mm", []), dtype=float).reshape(-1, 3)
                norm_mm = np.asarray(getattr(rc, "normal", []), dtype=float).reshape(-1, 3)

                final_tcp, final_norms_arr = _postprocess_compiled_path_strict(tcp_mm, norm_mm, side_path_params=p_side)
                final_tcp_world_mm = final_tcp
                final_norms = final_norms_arr

                poses = _compute_poses(final_tcp, final_norms_arr)
                if poses:
                    try:
                        existing = _get(recipe, "draft", None)
                        _set(recipe, "draft", _merge_draft_side(existing, side=side, poses=poses))
                    except Exception:
                        _LOG.debug("draft merge failed", exc_info=True)

            except Exception as e:
                _LOG.exception("Post-processing failed")
                preview_valid = False
                invalid_reason = f"postprocess_failed: {e}"
                final_tcp_world_mm = None
                final_norms = None

        # 6) Overlays
        try:
            out: OverlayOut = self._overlay.render_for_side(
                side=side,
                scene=scene,
                points_local_mm=mask_points_mm,
                raycast_result=rc,
                hit_poly=hit_poly,
                miss_poly=miss_poly,
                tcp_poly=tcp_poly,
                overlay_cfg=overlay_cfg_all,
            )
            if out.path_poly is not None:
                renderables.append(Renderable("path", "path", out.path_poly, _render_kwargs_for("path", out.path_poly)))
            if out.rays_hit_poly is not None:
                renderables.append(Renderable("hits", "hits", out.rays_hit_poly, _render_kwargs_for("hits", out.rays_hit_poly)))
            if out.rays_miss_poly is not None:
                renderables.append(Renderable("misses", "misses", out.rays_miss_poly, _render_kwargs_for("misses", out.rays_miss_poly)))
            if out.normals_poly is not None:
                renderables.append(Renderable("normals", "normals", out.normals_poly, _render_kwargs_for("normals", out.normals_poly)))
        except Exception:
            pass

        # 7) Final TCP visuals
        if final_tcp_world_mm is not None and final_norms is not None and preview_valid and final_tcp_world_mm.shape[0] >= 2:
            poly = pv.lines_from_points(final_tcp_world_mm, close=False)
            renderables.append(Renderable("tcp_line", "tcp_line", poly, _render_kwargs_for("tcp_line", poly)))

            px, py, pz = _tcp_frames_from_tangent_normal(final_tcp_world_mm, final_norms, max_frames=160, scale_mm=8.0)
            if px is not None:
                renderables.append(Renderable("tcp_x", "tcp_x", px, _render_kwargs_for("tcp_x", px)))
            if py is not None:
                renderables.append(Renderable("tcp_y", "tcp_y", py, _render_kwargs_for("tcp_y", py)))
            if pz is not None:
                renderables.append(Renderable("tcp_z", "tcp_z", pz, _render_kwargs_for("tcp_z", pz)))

        path_xyz_mm = final_tcp_world_mm if (final_tcp_world_mm is not None) else mask_points_mm

        return PreviewResult(
            recipe=recipe,
            valid=bool(preview_valid),
            invalid_reason=invalid_reason,
            scene=scene,
            substrate_mesh=substrate_mesh,
            path_xyz_mm=path_xyz_mm,
            final_tcp_world_mm=final_tcp_world_mm,
            renderables=renderables,
            bounds=(scene.bounds if scene and scene.bounds else _DEFAULT_BOUNDS),
            substrate_bounds=(tuple(substrate_mesh.bounds) if substrate_mesh is not None else None),
            meta={"side": side, "miss_n": miss_n},
        )
