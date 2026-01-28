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

# Fixed yaw correction around TCP local Z (optional)
_TCP_YAW_DEG: float = 0.0

_LAYER_STYLE: Dict[str, Dict[str, Any]] = {
    # matte solids (no lighting/shading tricks)
    "cage": {"color": "#6b6f75", "opacity": 0.25, "smooth_shading": False, "lighting": False, "specular": 0.0},

    # floor: darker than mount
    "ground": {"color": "#2b2f33", "opacity": 1.00, "smooth_shading": False, "lighting": False, "specular": 0.0},

    # mount: dark gray
    "mount": {"color": "#3f454c", "opacity": 1.00, "smooth_shading": False, "lighting": False, "specular": 0.0},

    # substrate: light gray
    "substrate": {"color": "#d0d6df", "opacity": 1.00, "smooth_shading": False, "lighting": False, "specular": 0.0},

    # overlays
    "mask": {"color": "#3498db", "line_width": 2.4, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "path": {"color": "#2ecc71", "line_width": 2.8, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
    "hits": {"color": "#a569bd", "opacity": 1.0, "lighting": False, "point_size": 10.0, "render_points_as_spheres": False},
    "misses": {"color": "#e74c3c", "opacity": 1.0, "lighting": False, "point_size": 10.0, "render_points_as_spheres": False},
    "normals": {"color": "#a569bd", "line_width": 1.5, "render_lines_as_tubes": True, "opacity": 1.0, "lighting": False},
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

    substrate_mesh: Optional[pv.PolyData]          # mount-top-frame mesh (for 2D)
    path_xyz_mm: Optional[np.ndarray]              # display path (mount-top-frame; incl pre/ret)
    final_tcp_world_mm: Optional[np.ndarray]       # kept name for API; here: mount-top-frame display path

    renderables: List[Renderable]                  # WORLD renderables for 3D

    bounds: Bounds                                 # WORLD bounds
    substrate_bounds: Optional[Bounds]             # mount-top-frame bounds (2D extents)
    meta: Dict[str, Any]


# ============================================================
# Local post-processing hook (moved from mesh_utils)
# ============================================================

def _postprocess_compiled_path_strict(
    tcp_mm: Any,
    normal: Any,
    *,
    side_path_params: Dict[str, Any],
) -> Tuple[np.ndarray, np.ndarray]:
    """
    STRICT postprocess hook used by SceneManager.

    Minimal safe behavior:
      - converts tcp_mm and normal to Nx3 float arrays
      - ensures same length by truncation
      - optional trim support (start/end indices)
      - returns (tcp, normals)
    """
    P = np.asarray(tcp_mm, dtype=float).reshape(-1, 3) if tcp_mm is not None else np.zeros((0, 3), dtype=float)
    N = np.asarray(normal, dtype=float).reshape(-1, 3) if normal is not None else np.zeros((0, 3), dtype=float)

    n = int(min(len(P), len(N)))
    if n <= 0:
        return np.zeros((0, 3), dtype=float), np.zeros((0, 3), dtype=float)

    P = P[:n].copy()
    N = N[:n].copy()

    # Optional: basic trim config
    try:
        trim = side_path_params.get("trim", None)
        if isinstance(trim, dict):
            a = int(trim.get("start", 0) or 0)
            b = trim.get("end", None)
            b_i = int(b) if b is not None else n
            if b_i < 0:
                b_i = max(0, n + b_i)
            a = max(0, min(n, a))
            b_i = max(a, min(n, b_i))
            P = P[a:b_i]
            N = N[a:b_i]
    except Exception:
        pass

    return P, N


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
        kwargs.update({"line_width": style.get("line_width"), "render_lines_as_tubes": style.get("render_lines_as_tubes")})
    elif kind == "points":
        kwargs.update({"point_size": style.get("point_size", 8.0), "render_points_as_spheres": style.get("render_points_as_spheres", True)})
    else:
        kwargs.update({"smooth_shading": style.get("smooth_shading"), "specular": style.get("specular")})

    return {k: v for k, v in kwargs.items() if v is not None}


def _unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(-1)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < eps:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


def _rot_z_deg(deg: float) -> np.ndarray:
    a = math.radians(float(deg))
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0],
                     [s,  c, 0.0],
                     [0.0, 0.0, 1.0]], dtype=float)


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


def _fixed_frame_from_z(z_axis: np.ndarray, *, yaw_deg_about_z: float = 0.0) -> np.ndarray:
    z = _unit(z_axis).reshape(3)

    ref = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(ref, z))) > 0.95:
        ref = np.array([0.0, 1.0, 0.0], dtype=float)

    x = ref - float(np.dot(ref, z)) * z
    x = _unit(x)
    y = _unit(np.cross(z, x))

    R = np.identity(3, dtype=float)
    R[:, 0], R[:, 1], R[:, 2] = x, y, z

    if abs(float(yaw_deg_about_z)) > 1e-9:
        R = R @ _rot_z_deg(float(yaw_deg_about_z))
        x2 = _unit(R[:, 0])
        z2 = _unit(R[:, 2])
        y2 = _unit(np.cross(z2, x2))
        x2 = _unit(np.cross(y2, z2))
        R[:, 0], R[:, 1], R[:, 2] = x2, y2, z2

    return R


def _compute_main_poses(points: np.ndarray, normals_world: np.ndarray) -> List[PoseQuat]:
    """
    Main (on-surface) poses from points+normals.
    """
    P = np.asarray(points, dtype=float).reshape(-1, 3)
    N = np.asarray(normals_world, dtype=float).reshape(-1, 3)
    n = int(min(len(P), len(N)))
    if n < 1:
        return []

    out: List[PoseQuat] = []
    for i in range(n):
        R = _fixed_frame_from_z(N[i], yaw_deg_about_z=_TCP_YAW_DEG)
        qx, qy, qz, qw = _quat_from_rotmat(R)
        out.append(PoseQuat(x=float(P[i, 0]), y=float(P[i, 1]), z=float(P[i, 2]),
                            qx=float(qx), qy=float(qy), qz=float(qz), qw=float(qw)))
    return out


def _extend_pre_ret_points_local(P_local: np.ndarray, *, pre_mm: float, ret_mm: float) -> Tuple[np.ndarray, bool, bool]:
    """
    Create pre/ret points by extending along the path tangents in LOCAL frame.
    Returns (P_ext, has_pre, has_ret).
    """
    P = np.asarray(P_local, dtype=float).reshape(-1, 3)
    if P.shape[0] < 2:
        return P.copy(), False, False

    has_pre = float(pre_mm) > 0.0
    has_ret = float(ret_mm) > 0.0

    t0 = P[1] - P[0]
    t1 = P[-1] - P[-2]
    t0n = t0 / (np.linalg.norm(t0) + 1e-12)
    t1n = t1 / (np.linalg.norm(t1) + 1e-12)

    pts: List[np.ndarray] = []
    if has_pre:
        pts.append((P[0] - float(pre_mm) * t0n).reshape(1, 3))
    pts.append(P)
    if has_ret:
        pts.append((P[-1] + float(ret_mm) * t1n).reshape(1, 3))

    return np.vstack(pts), has_pre, has_ret


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


def _draft_display_points_local(draft: Optional[Draft], side: str) -> Optional[np.ndarray]:
    if draft is None or not isinstance(draft, Draft):
        return None
    s = str(side or "").strip()
    if not s:
        return None

    ps = (draft.sides or {}).get(s)
    if ps is None or not isinstance(ps, PathSide):
        return None

    pts: List[List[float]] = []
    if ps.predispense:
        p0 = ps.predispense[0]
        pts.append([float(p0.x), float(p0.y), float(p0.z)])
    for p in (ps.poses_quat or []):
        pts.append([float(p.x), float(p.y), float(p.z)])
    if ps.retreat:
        p1 = ps.retreat[0]
        pts.append([float(p1.x), float(p1.y), float(p1.z)])

    if len(pts) < 2:
        return None
    return np.asarray(pts, dtype=float).reshape(-1, 3)


class SceneManager:
    _ATTR_TO_LAYER: Dict[str, str] = {
        "rays_hit_poly": "hits",
        "rays_miss_poly": "misses",
        "normals_poly": "normals",
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
            ground_mesh = pv.Plane(center=(xmid, ymid, 0.0),
                                   direction=(0.0, 0.0, 1.0),
                                   i_size=float(size), j_size=float(size),
                                   i_resolution=1, j_resolution=1)

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
            return PreviewResult(recipe=recipe, valid=False, invalid_reason="scene_fail",
                                 scene=None, substrate_mesh=None, path_xyz_mm=None, final_tcp_world_mm=None,
                                 renderables=[], bounds=_DEFAULT_BOUNDS, substrate_bounds=None, meta={})

        # Compute-frame origin = mount center XY, mount top Z in WORLD
        origin_world = (0.0, 0.0, 0.0)
        if scene.mount_mesh is not None:
            b = scene.mount_mesh.bounds
            origin_world = (0.5 * (float(b[0]) + float(b[1])),
                            0.5 * (float(b[2]) + float(b[3])),
                            float(b[5]))

        # Static WORLD renderables
        for layer_name, mesh in (("ground", scene.ground_mesh),
                                 ("cage", scene.cage_mesh),
                                 ("mount", scene.mount_mesh),
                                 ("substrate", scene.substrate_mesh)):
            if mesh is not None:
                renderables.append(Renderable(layer=layer_name, name=layer_name, poly=mesh,
                                              render_kwargs=_render_kwargs_for(layer_name, mesh)))

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
            return PreviewResult(recipe=recipe, valid=False, invalid_reason="no_substrate",
                                 scene=scene, substrate_mesh=None, path_xyz_mm=mask_local, final_tcp_world_mm=None,
                                 renderables=renderables, bounds=scene.bounds or _DEFAULT_BOUNDS,
                                 substrate_bounds=None, meta={"side": side, "frame": "mount_top", "origin_world_mm": origin_world})

        # 2) Mask -> WORLD (ray start points). Anchor to substrate top + stand_off.
        sub_top_world_z = float(scene.substrate_mesh.bounds[5])
        stand_off = float(params.get("stand_off_mm", 50.0))

        mask_world = _add_points_offset(mask_local, origin_world)
        if mask_world.size:
            mask_world[:, 2] = sub_top_world_z + stand_off + mask_local[:, 2]

        # Render mask (WORLD)
        mask_poly = _polyline(mask_world)
        if mask_poly is not None:
            renderables.append(Renderable(layer="mask", name="mask_poly", poly=mask_poly,
                                          render_kwargs=_render_kwargs_for("mask", mask_poly)))

        # 3) Raycast (WORLD) -> produces on-surface tcp + normals for valid points
        rc, hit_p, miss_p, _tcp_p_from_projector = cast_rays_for_side(
            P_world_start=mask_world,
            sub_mesh_world=scene.substrate_mesh,
            side=side,
            stand_off_mm=stand_off,
            # NEW: 0..45 deg nozzle-tilt constraint (0 disables filter)
            max_nozzle_tilt_deg=float(params.get("max_nozzle_tilt_deg", 0.0) or 0.0),
        )

        v_raw = getattr(rc, "valid", None)
        valid_mask = np.zeros((0,), dtype=bool) if v_raw is None else np.asarray(v_raw, dtype=bool).reshape(-1)
        miss_n = int(len(valid_mask) - int(np.sum(valid_mask))) if len(valid_mask) else 0
        preview_valid = (miss_n == 0)

        # 4) Substrate for 2D (mount-top KS): copy + shift by origin_world
        substrate_local_mesh: Optional[pv.PolyData] = None
        substrate_bounds_local: Optional[Bounds] = None
        try:
            substrate_local_mesh = scene.substrate_mesh.copy(deep=True) if scene.substrate_mesh is not None else None
            if substrate_local_mesh is not None:
                substrate_local_mesh.translate((-origin_world[0], -origin_world[1], -origin_world[2]), inplace=True)
                substrate_bounds_local = tuple(substrate_local_mesh.bounds)  # type: ignore[arg-type]
        except Exception:
            substrate_local_mesh = None
            substrate_bounds_local = None

        display_path_local: Optional[np.ndarray] = None

        # For overlays:
        # - path_points_world_mm: display polyline (may include off-surface offsets!)
        # - tcp_points_world_mm : ONLY main on-surface TCP points (must match rc.valid length semantics)
        path_world_for_display: Optional[np.ndarray] = None
        tcp_world_for_overlay: Optional[np.ndarray] = None

        if preview_valid:
            p_side = (getattr(recipe, "paths_by_side", {}) or {}).get(side, {}) or {}

            main_tcp_world, main_norms_world = _postprocess_compiled_path_strict(
                getattr(rc, "tcp_mm", None),
                getattr(rc, "normal", None),
                side_path_params=p_side,
            )

            tcp_world_for_overlay = main_tcp_world  # IMPORTANT: overlays expect on-surface here

            # Convert to local (mount-top KS) positions only (translation)
            main_tcp_local = _shift_points(main_tcp_world, origin_world) if main_tcp_world is not None else None

            if main_tcp_local is not None and main_norms_world is not None:
                try:
                    pre_mm = float((p_side or {}).get("predispense_offset_mm", 0.0) or 0.0)
                except Exception:
                    pre_mm = 0.0
                try:
                    ret_mm = float((p_side or {}).get("retreat_offset_mm", 0.0) or 0.0)
                except Exception:
                    ret_mm = 0.0
                pre_mm = max(0.0, float(pre_mm))
                ret_mm = max(0.0, float(ret_mm))

                Pm = np.asarray(main_tcp_local, dtype=float).reshape(-1, 3)
                NmW = np.asarray(main_norms_world, dtype=float).reshape(-1, 3)

                n = int(min(Pm.shape[0], NmW.shape[0]))
                if n >= 2:
                    Pm = Pm[:n]
                    NmW = NmW[:n]

                    # 1) Build MAIN poses (on-surface) from normals
                    main_poses = _compute_main_poses(Pm, NmW)
                    if len(main_poses) >= 2:
                        # 2) Extend points (pre/ret) AFTER raycast (can be off-surface)
                        P_ext, has_pre, has_ret = _extend_pre_ret_points_local(Pm, pre_mm=pre_mm, ret_mm=ret_mm)

                        # 3) Create EXTENDED poses by copying orientation from neighbor MAIN pose
                        poses_ext: List[PoseQuat] = []
                        if has_pre:
                            ppre = P_ext[0]
                            qref = main_poses[0]
                            poses_ext.append(PoseQuat(x=float(ppre[0]), y=float(ppre[1]), z=float(ppre[2]),
                                                      qx=float(qref.qx), qy=float(qref.qy), qz=float(qref.qz), qw=float(qref.qw)))

                        # main poses as-is (positions already match Pm)
                        poses_ext.extend(main_poses)

                        if has_ret:
                            pret = P_ext[-1]
                            qref = main_poses[-1]
                            poses_ext.append(PoseQuat(x=float(pret[0]), y=float(pret[1]), z=float(pret[2]),
                                                      qx=float(qref.qx), qy=float(qref.qy), qz=float(qref.qz), qw=float(qref.qw)))

                        # Merge into Draft (this is what should be persisted elsewhere)
                        recipe.draft = self._merge_draft_side(
                            getattr(recipe, "draft", None),
                            side=side,
                            poses=poses_ext,
                            pre_mm=pre_mm,
                            ret_mm=ret_mm,
                        )

                        display_path_local = _draft_display_points_local(recipe.draft, side)

            # Render final path (WORLD) using display path (incl pre/ret)
            if display_path_local is not None:
                path_world_for_display = _add_points_offset(display_path_local, origin_world)
            else:
                path_world_for_display = main_tcp_world

            path_poly = _polyline(path_world_for_display) if path_world_for_display is not None else None
            if path_poly is not None:
                renderables.append(Renderable(layer="path", name="final_path_poly", poly=path_poly,
                                              render_kwargs=_render_kwargs_for("path", path_poly)))

        # 5) Overlays (WORLD): ALWAYS build all overlay geometries.
        cfg_all = dict(overlay_cfg or {})
        cfg_all["visibility"] = {
            "mask": True,
            "path": True,
            "tcp": True,
            "hits": True,
            "misses": True,
            "normals": True,
            "tcp_axes": False,
        }

        empty3 = np.zeros((0, 3), dtype=float)
        out = self._overlay.render_for_side(
            side=side,
            raycast_result=rc,
            hit_poly=hit_p,
            miss_poly=miss_p,
            overlay_cfg=cfg_all,
            mask_points_world_mm=np.asarray(mask_world, dtype=float).reshape(-1, 3) if mask_world is not None else empty3,

            # IMPORTANT FIX:
            # - path_points can include off-surface offsets (pure display)
            # - tcp_points MUST be on-surface (to avoid "offsets marked wrong")
            path_points_world_mm=np.asarray(path_world_for_display, dtype=float).reshape(-1, 3) if path_world_for_display is not None else empty3,
            tcp_points_world_mm=np.asarray(tcp_world_for_overlay, dtype=float).reshape(-1, 3) if tcp_world_for_overlay is not None else empty3,
        )

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
            substrate_mesh=substrate_local_mesh,
            path_xyz_mm=display_path_local,
            final_tcp_world_mm=display_path_local,  # contract: mount-top-frame display path
            renderables=renderables,
            bounds=scene.bounds or _DEFAULT_BOUNDS,
            substrate_bounds=substrate_bounds_local,
            meta={
                "miss_n": miss_n,
                "side": side,
                "frame": "mount_top",
                "origin_world_mm": origin_world,
                "tcp_yaw_deg": float(_TCP_YAW_DEG),
            },
        )

    @staticmethod
    def _merge_draft_side(
        draft: Optional[Draft],
        *,
        side: str,
        poses: List[PoseQuat],
        pre_mm: float,
        ret_mm: float,
    ) -> Draft:
        s = str(side or "").strip()

        cur_sides: Dict[str, PathSide] = {}
        if isinstance(draft, Draft) and isinstance(getattr(draft, "sides", None), dict):
            cur_sides = dict(getattr(draft, "sides", {}) or {})

        if not s:
            return Draft(version=1, sides=cur_sides)

        if not poses:
            cur_sides.pop(s, None)
            return Draft(version=1, sides=cur_sides)

        if len(poses) < 1:
            cur_sides.pop(s, None)
            return Draft(version=1, sides=cur_sides)

        # If you generated explicit pre/ret, keep split, else treat everything as main.
        # Here: convention = first is pre, last is ret, middle is main, BUT only if len>=3.
        if len(poses) >= 3:
            pre = poses[0]
            ret = poses[-1]
            main = poses[1:-1]
            cur_sides[s] = PathSide(predispense=[pre], retreat=[ret], poses_quat=list(main))
        else:
            # fallback: just store as main poses
            cur_sides[s] = PathSide(predispense=[], retreat=[], poses_quat=list(poses))

        return Draft(version=1, sides=cur_sides)
