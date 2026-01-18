# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/tab3d.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Any, Dict, Tuple

import numpy as np
import pyvista as pv

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

# --- Models/Logic (moved here; panel stays UI-only) ---
from model.recipe.path_builder import PathBuilder
from model.recipe.recipe import Recipe

from .overlays import OverlayOut, OverlayRenderer
from .raycast_projector import cast_rays_for_side
from .scene_manager import PreviewScene, SceneManager

# --- UI sub-widgets ---
from .overlays_groupbox import OverlaysGroupBox
from .views_3d_box import Views3DBox


_LOG = logging.getLogger("tabs.recipe.preview.tab3d")

Bounds = Tuple[float, float, float, float, float, float]
_ALLOWED_SIDES = ("top", "front", "back", "left", "right", "polyhelix", "helix")
_DEFAULT_BOUNDS: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)


def _safe_unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < eps:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


def _poly_segments(starts: np.ndarray, dirs: np.ndarray, length: float) -> Optional[pv.PolyData]:
    """Create line segments from start points and direction vectors."""
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


def _make_ground_box(bounds: Bounds, *, z_under: float, pad_factor: float = 3.0, thickness_mm: float = 0.6) -> pv.PolyData:
    """Create a thin black ground box under the mount (large XY extent)."""
    xmin, xmax, ymin, ymax, zmin, zmax = (float(bounds[0]), float(bounds[1]), float(bounds[2]), float(bounds[3]), float(bounds[4]), float(bounds[5]))
    dx = max(10.0, xmax - xmin)
    dy = max(10.0, ymax - ymin)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)

    hx = 0.5 * dx * float(pad_factor)
    hy = 0.5 * dy * float(pad_factor)
    x0, x1 = (cx - hx, cx + hx)
    y0, y1 = (cy - hy, cy + hy)
    t = max(0.1, float(thickness_mm))
    z1 = float(z_under)
    z0 = z1 - t
    return pv.Box(bounds=(x0, x1, y0, y1, z0, z1))


def _tcp_frames_from_tangent_normal(
    tcp_mm: np.ndarray,
    normals: np.ndarray,
    *,
    max_frames: int = 160,
    scale_mm: float = 8.0,
) -> Tuple[Optional[pv.PolyData], Optional[pv.PolyData], Optional[pv.PolyData]]:
    """Build small coordinate frames along the TCP path.

    Orientation heuristic (strict, deterministic):
      - z axis = surface normal
      - x axis = path tangent
      - y axis = z × x
    """
    P = np.asarray(tcp_mm, dtype=float).reshape(-1, 3)
    N = np.asarray(normals, dtype=float).reshape(-1, 3)
    n = min(P.shape[0], N.shape[0])
    if n < 2:
        return None, None, None
    P = P[:n]
    N = N[:n]

    # choose frame origins (decimate)
    if max_frames <= 0 or n <= max_frames:
        idx = np.arange(n, dtype=int)
    else:
        idx = np.linspace(0, n - 1, num=int(max_frames), dtype=int)

    origins = P[idx]

    # tangent per selected idx
    tangents = np.zeros_like(origins)
    normals_sel = np.zeros_like(origins)
    for k, i in enumerate(idx.tolist()):
        i0 = max(0, i - 1)
        i1 = min(n - 1, i + 1)
        t = P[i1] - P[i0]
        tangents[k] = _safe_unit(t)
        normals_sel[k] = _safe_unit(N[i])

    z_dirs = normals_sel
    x_dirs = tangents
    y_dirs = np.cross(z_dirs, x_dirs)
    for i in range(y_dirs.shape[0]):
        if np.linalg.norm(y_dirs[i]) < 1e-9:
            # if tangent is parallel to normal -> pick a stable perpendicular
            y_dirs[i] = _safe_unit(np.cross(z_dirs[i], np.array([0.0, 1.0, 0.0], dtype=float)))
        else:
            y_dirs[i] = _safe_unit(y_dirs[i])
        x_dirs[i] = _safe_unit(np.cross(y_dirs[i], z_dirs[i]))

    px = _poly_segments(origins, x_dirs, float(scale_mm))
    py = _poly_segments(origins, y_dirs, float(scale_mm))
    pz = _poly_segments(origins, z_dirs, float(scale_mm))
    return px, py, pz


# ------------------------------------------------------------
# Visual style (SSoT for 3D colors)
# ------------------------------------------------------------
# NOTE: keep these broadly aligned with Matplot2DView._style.
_LAYER_STYLE: Dict[str, Dict[str, Any]] = {
    # Base meshes
    "cage": {
        "color": "#9aa0a6",
        "opacity": 0.25,
        "smooth_shading": True,
        "specular": 0.05,
    },
    "ground": {
        "color": "#000000",
        "opacity": 1.00,
        "smooth_shading": False,
        "lighting": True,
        "specular": 0.0,
    },
    "mount": {
        "color": "#6b6f75",  # dark gray
        "opacity": 1.00,
        "smooth_shading": True,
        "specular": 0.05,
    },
    "substrate": {
        "color": "#d9d9d9",  # light gray
        "opacity": 1.00,
        "smooth_shading": True,
        "specular": 0.10,
    },

    # Authoring/preview layers
    "mask": {
        "color": "#3498db",  # blue
        "line_width": 2.4,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "path": {
        "color": "#2ecc71",  # matches 2D path_color
        "line_width": 2.8,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    # Markers are intentionally not used by default (continuous polyline preferred).

    # Raycast debug
    "hits": {
        "color": "#a569bd",  # purple
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "misses": {
        "color": "#e74c3c",
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "tcp_line": {
        "color": "#5dade2",
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "tcp_x": {
        "color": "#e74c3c",
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "tcp_y": {
        "color": "#2ecc71",
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "tcp_z": {
        "color": "#3498db",
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "tcp": {
        "color": "#5dade2",
        "line_width": 1.6,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
    "normals": {
        "color": "#a569bd",
        "line_width": 1.5,
        "render_lines_as_tubes": True,
        "opacity": 1.0,
        "lighting": False,
    },
}


def _layer_style(layer: str) -> Dict[str, Any]:
    """Return resolved style for layer (never None)."""
    s = _LAYER_STYLE.get(str(layer), {})
    return dict(s)


def _poly_kind(poly: Any) -> str:
    """Heuristic classification for PyVista PolyData."""
    try:
        # Lines: .lines array present and non-empty
        if hasattr(poly, "lines"):
            arr = np.asarray(getattr(poly, "lines"), dtype=np.int64).ravel()
            if arr.size > 0:
                return "lines"
    except Exception:
        pass

    try:
        # Points-only polydata often has 0 cells
        n_cells = int(getattr(poly, "n_cells", 0) or 0)
        n_faces = int(getattr(poly, "n_faces", 0) or 0)
        if n_cells == 0 and n_faces == 0:
            return "points"
    except Exception:
        pass

    return "surface"


# ------------------------------------------------------------
# Helpers (strict, UI-neutral)
# ------------------------------------------------------------

def _clamp_nonneg(v: Any) -> float:
    try:
        return max(0.0, float(v))
    except Exception:
        return 0.0


def _as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return float(default)


def _postprocess_compiled_tcp_mm_strict(P: np.ndarray, *, side_path_params: Dict[str, Any]) -> np.ndarray:
    """STRICT postprocess applied AFTER raycast (compiled TCP).

    - Trim by start/end offset along arc-length.
    - Add predispense/retreat air moves by extending along first/last segment direction.

    NOTE: We intentionally avoid any silent fallbacks besides a small inline trim fallback,
    because the PathBuilder method may not be exposed in every deployment.
    """
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    if P.shape[0] < 2:
        return P

    # 1) Trim (Start/End Offset)
    start_off = _clamp_nonneg(side_path_params.get("start_offset_mm", 0.0))
    end_off = _clamp_nonneg(side_path_params.get("end_offset_mm", 0.0))

    if start_off > 0.0 or end_off > 0.0:
        trim_fn = getattr(PathBuilder, "_trim_by_arclength", None)
        if callable(trim_fn):
            P = np.asarray(trim_fn(P, float(start_off), float(end_off)), dtype=float).reshape(-1, 3)
        else:
            # Inline fallback logic if method not exposed
            d = np.linalg.norm(P[1:] - P[:-1], axis=1)
            s = np.concatenate([[0.0], np.cumsum(d)])
            total = float(s[-1]) if s.size else 0.0
            if total > 1e-9:
                a = float(start_off)
                b = max(a, total - float(end_off))
                if b - a > 1e-9:
                    def sample_at(t: float) -> np.ndarray:
                        t = float(np.clip(t, 0.0, total))
                        i = int(np.searchsorted(s, t, side="right") - 1)
                        i = max(0, min(i, len(d) - 1))
                        seg_len = float(d[i])
                        if seg_len <= 1e-12:
                            return P[i].copy()
                        u = (t - float(s[i])) / seg_len
                        return (1.0 - u) * P[i] + u * P[i + 1]

                    pts = [sample_at(a)]
                    for i in range(1, len(P) - 1):
                        if a <= float(s[i]) <= b:
                            pts.append(P[i].copy())
                    pts.append(sample_at(b))
                    P = np.asarray(pts, dtype=float).reshape(-1, 3)

        if P.shape[0] < 2:
            return P

    # 2) Extend (Air Moves)
    pre_mm = _clamp_nonneg(side_path_params.get("predispense_offset_mm", 0.0))
    post_mm = _clamp_nonneg(side_path_params.get("retreat_offset_mm", 0.0))

    out = P
    if pre_mm > 0.0 and out.shape[0] >= 2:
        v0 = out[0] - out[1]
        n0 = float(np.linalg.norm(v0))
        if n0 > 1e-9:
            p_pre = out[0] + (v0 / n0) * float(pre_mm)
            out = np.vstack([p_pre[None, :], out])

    if post_mm > 0.0 and out.shape[0] >= 2:
        v1 = out[-1] - out[-2]
        n1 = float(np.linalg.norm(v1))
        if n1 > 1e-9:
            p_post = out[-1] + (v1 / n1) * float(post_mm)
            out = np.vstack([out, p_post[None, :]])

    return np.asarray(out, dtype=float).reshape(-1, 3)


# ------------------------------------------------------------
# Data contracts
# ------------------------------------------------------------

@dataclass
class PreviewResult:
    recipe: Optional[Recipe]

    valid: bool
    invalid_reason: Optional[str]

    scene: Optional[PreviewScene]
    substrate_mesh: Optional[pv.PolyData]

    # Path for consumer use (2D view + info). Depending on validity this is
    # either the final TCP (postprocessed) or the local mask.
    path_xyz_mm: Optional[np.ndarray]

    # Final post-processed TCP (only when valid)
    final_tcp_world_mm: Optional[np.ndarray]

    # Bounds used for camera.
    bounds: Bounds
    substrate_bounds: Optional[Bounds]


@dataclass
class _LayerActors:
    actors: list[Any]


class Tab3D(QWidget):
    """Kapselt die 3D-Ansicht.

    IMPORTANT ARCHITECTURE:
      - Panel builds UI and debounces update.
      - Tab3D owns the entire 3D preview *logic* (scene, mask build, raycast, overlays).
      - Tab3D exposes bounds + preview status/results for other tabs.
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        render_callable: Any = None,
        get_bounds_callable: Any = None,  # kept for compatibility; ignored
        get_substrate_bounds_callable: Any = None,  # kept for compatibility; ignored
        on_overlay_changed: Any = None,
    ) -> None:
        super().__init__(parent)

        self._render_callable = render_callable
        self._on_overlay_changed = on_overlay_changed

        # Preview state (SSoT for other widgets)
        self._scene_mgr = SceneManager()
        self._overlay = OverlayRenderer()
        self._scene: Optional[PreviewScene] = None
        self._preview_valid: bool = False
        self._preview_invalid_reason: Optional[str] = "no_preview"
        self._final_tcp_world_mm: Optional[np.ndarray] = None
        self._path_xyz_mm: Optional[np.ndarray] = None

        self._layers: Dict[str, _LayerActors] = {}

        # ---------------- UI ----------------
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(4, 4, 4, 4)
        self._layout.setSpacing(6)

        # 1) Overlays (Top)
        self.overlays_box = OverlaysGroupBox(parent=self)
        self._set_policy(self.overlays_box, v=QSizePolicy.Policy.Preferred)
        self._layout.addWidget(self.overlays_box, 0)

        if callable(self._on_overlay_changed):
            self.overlays_box.sig_changed.connect(self._on_overlay_changed)

        # 2) PV Host Setup
        self.pv_host = QWidget(self)
        self._host_layout = QVBoxLayout(self.pv_host)
        self._host_layout.setContentsMargins(0, 0, 0, 0)
        self._host_layout.setSpacing(0)

        self.pv_plot = QtInteractor(self.pv_host)
        self._host_layout.addWidget(self.pv_plot, 1)

        # Visual defaults
        try:
            self.pv_plot.set_background("white")
        except Exception:
            pass
        try:
            # Keep the view clean; users have dedicated view buttons.
            self.pv_plot.hide_axes()
        except Exception:
            pass

        # 3) View Controls (use Tab3D as bounds authority)
        self.views_box = Views3DBox(
            interactor_getter=lambda: self.pv_plot,
            render_callable=self._render,
            bounds_getter=self.get_bounds,
            substrate_bounds_getter=self.get_substrate_bounds,
            cam_pad=1.1,
            iso_extra_zoom=1.30,
            parent=self,
        )
        self._set_policy(self.views_box, v=QSizePolicy.Policy.Preferred)
        self._layout.addWidget(self.views_box, 0)

        # 4) Plotter (Bottom)
        self._set_policy(self.pv_host, v=QSizePolicy.Policy.Expanding)
        self._layout.addWidget(self.pv_host, 1)

    # ------------------------------------------------------------
    # Public API (UI)
    # ------------------------------------------------------------

    def get_pv_host(self) -> QWidget:
        return self.pv_host

    def get_overlay_config(self) -> Dict[str, Any]:
        return self.overlays_box.get_config()

    def render(self) -> None:
        self._render()

    def _render(self) -> None:
        if self.pv_plot:
            try:
                self.pv_plot.render()
            except Exception:
                pass

    # ------------------------------------------------------------
    # Public API (preview state)
    # ------------------------------------------------------------

    def preview_is_valid(self) -> bool:
        return bool(self._preview_valid)

    def preview_invalid_reason(self) -> Optional[str]:
        return self._preview_invalid_reason

    def final_tcp_world_mm(self) -> Optional[np.ndarray]:
        if self._final_tcp_world_mm is None:
            return None
        return np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)

    def path_xyz_mm(self) -> Optional[np.ndarray]:
        if self._path_xyz_mm is None:
            return None
        return np.asarray(self._path_xyz_mm, dtype=float).reshape(-1, 3)

    # ------------------------------------------------------------
    # Bounds (SSoT for camera)
    # ------------------------------------------------------------

    def get_bounds(self) -> Bounds:
        if self._scene and getattr(self._scene, "bounds", None):
            try:
                return tuple(self._scene.bounds)  # type: ignore[return-value]
            except Exception:
                return _DEFAULT_BOUNDS
        return _DEFAULT_BOUNDS

    def get_substrate_bounds(self) -> Optional[Bounds]:
        if self._scene and getattr(self._scene, "substrate_mesh", None) is not None:
            try:
                b = self._scene.substrate_mesh.bounds  # type: ignore[attr-defined]
                return (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), float(b[5]))
            except Exception:
                return None
        return None

    # ------------------------------------------------------------
    # Layer management
    # ------------------------------------------------------------

    def clear_layers(self) -> None:
        """Clears all managed actor layers."""
        if self.pv_plot is None:
            return

        for _name, layer_obj in self._layers.items():
            for a in list(layer_obj.actors):
                try:
                    self.pv_plot.remove_actor(a)
                except Exception:
                    pass
            layer_obj.actors.clear()

    def add_polydata(
        self,
        layer: str,
        poly: Any,
        *,
        name: str = "",
        opacity: Optional[float] = None,
        color: Optional[str] = None,
        line_width: Optional[float] = None,
        point_size: Optional[float] = None,
        render_lines_as_tubes: Optional[bool] = None,
        render_points_as_spheres: Optional[bool] = None,
        smooth_shading: Optional[bool] = None,
        lighting: Optional[bool] = None,
        specular: Optional[float] = None,
    ) -> None:
        """Adds a mesh/polydata to a specific layer.

        Style is resolved from _LAYER_STYLE by default and can be overridden per call.
        """
        if self.pv_plot is None or poly is None:
            return

        # Avoid empty mesh crashes
        if hasattr(poly, "n_points") and getattr(poly, "n_points") == 0:
            return

        if layer not in self._layers:
            self._layers[layer] = _LayerActors(actors=[])

        try:
            style = _layer_style(layer)
            kind = _poly_kind(poly)

            # Resolve style + overrides
            if color is None:
                color = style.get("color")
            if opacity is None:
                opacity = style.get("opacity", 1.0)
            if line_width is None:
                line_width = style.get("line_width")
            if point_size is None:
                point_size = style.get("point_size")
            if render_lines_as_tubes is None:
                render_lines_as_tubes = style.get("render_lines_as_tubes")
            if render_points_as_spheres is None:
                render_points_as_spheres = style.get("render_points_as_spheres")
            if smooth_shading is None:
                smooth_shading = style.get("smooth_shading")
            if lighting is None:
                lighting = style.get("lighting")
            if specular is None:
                specular = style.get("specular")

            kwargs: Dict[str, Any] = {
                "name": str(name or layer),
                "pickable": False,
                "opacity": float(opacity),
            }
            if color is not None:
                kwargs["color"] = str(color)

            # Geometry-type specific knobs
            if kind == "lines":
                if line_width is not None:
                    kwargs["line_width"] = float(line_width)
                if render_lines_as_tubes is not None:
                    kwargs["render_lines_as_tubes"] = bool(render_lines_as_tubes)
            elif kind == "points":
                if point_size is not None:
                    kwargs["point_size"] = float(point_size)
                if render_points_as_spheres is not None:
                    kwargs["render_points_as_spheres"] = bool(render_points_as_spheres)
            else:
                if smooth_shading is not None:
                    kwargs["smooth_shading"] = bool(smooth_shading)
                if specular is not None:
                    kwargs["specular"] = float(specular)

            if lighting is not None:
                kwargs["lighting"] = bool(lighting)

            a = self.pv_plot.add_mesh(poly, **kwargs)
            self._layers[layer].actors.append(a)
        except Exception:
            pass

    # ------------------------------------------------------------
    # Preview pipeline (moved from panel)
    # ------------------------------------------------------------

    def update_preview(self, *, recipe: Optional[Recipe], ctx: Any) -> PreviewResult:
        """Compute + render the current 3D preview.

        This function owns all logic previously in CoatingPreviewPanel.
        It is safe to call repeatedly.

        Returns a PreviewResult to allow the panel (and 2D view) to update.
        """
        # Reset state
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._final_tcp_world_mm = None
        self._path_xyz_mm = None
        self._scene = None

        self.clear_layers()

        if recipe is None:
            return PreviewResult(
                recipe=None,
                valid=False,
                invalid_reason="no_preview",
                scene=None,
                substrate_mesh=None,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                bounds=self.get_bounds(),
                substrate_bounds=None,
            )

        # 1) Build Scene (must pass ctx for path resolution)
        try:
            self._scene = self._scene_mgr.build_scene(recipe, ctx=ctx)
        except Exception as e:
            _LOG.exception("Scene build failed")
            self._scene = None
            self._preview_valid = False
            self._preview_invalid_reason = f"scene_build_failed: {e}"
            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=self._preview_invalid_reason,
                scene=None,
                substrate_mesh=None,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                bounds=_DEFAULT_BOUNDS,
                substrate_bounds=None,
            )

        substrate_mesh = self._scene.substrate_mesh if self._scene else None

        # 1b) Draw base meshes
        try:
            if self._scene and self._scene.cage_mesh is not None:
                self.add_polydata("cage", self._scene.cage_mesh, name="cage")
            if self._scene and self._scene.mount_mesh is not None:
                self.add_polydata("mount", self._scene.mount_mesh, name="mount")
                # Ground plane: thin, large, black, directly under the mount
                try:
                    b = self._scene.mount_mesh.bounds
                    z_under = float(b[4]) - 0.30
                    ground = _make_ground_box(
                        (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), float(b[5])),
                        z_under=z_under,
                        pad_factor=3.2,
                        thickness_mm=0.6,
                    )
                    self.add_polydata("ground", ground, name="ground")
                except Exception:
                    pass
            if substrate_mesh is not None:
                self.add_polydata("substrate", substrate_mesh, name="substrate")
        except Exception:
            _LOG.exception("draw base meshes failed")

        # 2) Params
        side = str(getattr(recipe, "active_side", None) or "top").lower()
        if side not in _ALLOWED_SIDES:
            side = "top"
        globals_params = dict(getattr(recipe, "parameters", {}) or {})
        sample_step_mm = _as_float(globals_params.get("sample_step_mm", 1.0), 1.0)
        max_points = int(globals_params.get("max_points", 1000) or 1000)
        stand_off_mm = _as_float(globals_params.get("stand_off_mm", 50.0), 50.0)

        # 3) Mask path generation
        p_side: Dict[str, Any] = {}
        try:
            pbs = getattr(recipe, "paths_by_side", {}) or {}
            p_side = dict(pbs.get(side) or {})

            pd_mask = PathBuilder._from_path_dict(
                p_side,
                sample_step_mm=float(sample_step_mm),
                max_points=int(max_points),
            )
            pd_mask = PathBuilder._with_globals_meta(pd_mask, globals_params)
            mask_points_mm = np.asarray(pd_mask.points_mm, dtype=float).reshape(-1, 3)
            meta = dict(pd_mask.meta or {})
            path_source = str(meta.get("source", ""))

            # Lift mask above substrate top + stand_off (plus optional per-side z_mm)
            base_z = 0.0
            if substrate_mesh is not None:
                base_z = float(substrate_mesh.bounds[5])
            if mask_points_mm.shape[0] > 0:
                mask_points_mm[:, 2] += (base_z + float(stand_off_mm))
                mask_points_mm[:, 2] += _as_float(p_side.get("z_mm", 0.0))

        except Exception as e:
            _LOG.exception("Path generation failed for side '%s'", side)
            self._preview_valid = False
            self._preview_invalid_reason = f"path_build_failed: {e}"

            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=self._preview_invalid_reason,
                scene=self._scene,
                substrate_mesh=substrate_mesh,
                path_xyz_mm=None,
                final_tcp_world_mm=None,
                bounds=self.get_bounds(),
                substrate_bounds=self.get_substrate_bounds(),
            )

        if mask_points_mm.shape[0] < 2:
            self._preview_valid = False
            self._preview_invalid_reason = "mask_too_short"
            self._path_xyz_mm = np.asarray(mask_points_mm, dtype=float).reshape(-1, 3)

            # Optional: draw mask anyway if enabled
            cfg = self.get_overlay_config()
            if cfg.get("mask", True):
                try:
                    self.add_polydata("mask", pv.lines_from_points(mask_points_mm, close=False), name="mask")
                except Exception:
                    _LOG.exception("draw mask failed")

            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=self._preview_invalid_reason,
                scene=self._scene,
                substrate_mesh=substrate_mesh,
                path_xyz_mm=self._path_xyz_mm,
                final_tcp_world_mm=None,
                bounds=self.get_bounds(),
                substrate_bounds=self.get_substrate_bounds(),
            )

        cfg = self.get_overlay_config()

        # Show mask (3D)
        if cfg.get("mask", True):
            try:
                self.add_polydata("mask", pv.lines_from_points(mask_points_mm, close=False), name="mask")
            except Exception:
                _LOG.exception("draw mask failed")

        # 4) Raycast
        if substrate_mesh is None:
            self._preview_valid = False
            self._preview_invalid_reason = "no_substrate_mesh"
            self._path_xyz_mm = np.asarray(mask_points_mm, dtype=float).reshape(-1, 3)

            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=self._preview_invalid_reason,
                scene=self._scene,
                substrate_mesh=None,
                path_xyz_mm=self._path_xyz_mm,
                final_tcp_world_mm=None,
                bounds=self.get_bounds(),
                substrate_bounds=None,
            )

        try:
            rc, hit_poly, miss_poly, tcp_poly = cast_rays_for_side(
                P_world_start=mask_points_mm,
                sub_mesh_world=substrate_mesh,
                side=side,
                source=path_source,
                stand_off_mm=float(stand_off_mm),
                invert_dirs=bool(globals_params.get("invert_dirs", False)),
            )
        except Exception as e:
            _LOG.exception("Raycast calculation failed")
            self._preview_valid = False
            self._preview_invalid_reason = f"raycast_failed: {e}"
            self._path_xyz_mm = np.asarray(mask_points_mm, dtype=float).reshape(-1, 3)

            return PreviewResult(
                recipe=recipe,
                valid=False,
                invalid_reason=self._preview_invalid_reason,
                scene=self._scene,
                substrate_mesh=substrate_mesh,
                path_xyz_mm=self._path_xyz_mm,
                final_tcp_world_mm=None,
                bounds=self.get_bounds(),
                substrate_bounds=self.get_substrate_bounds(),
            )

        # Misses check
        try:
            valid = np.asarray(rc.valid, dtype=bool).reshape(-1)
        except Exception:
            valid = np.zeros((0,), dtype=bool)

        total_n = int(mask_points_mm.shape[0])
        n = min(total_n, int(valid.shape[0])) if valid.size else 0
        hit_n = int(np.sum(valid[:n])) if n > 0 else 0
        miss_n = int(n - hit_n) + int(max(0, total_n - n))

        self._preview_valid = (miss_n == 0)
        self._preview_invalid_reason = None if self._preview_valid else f"raycast_misses(count={miss_n})"

        # 5) Postprocess final TCP
        self._final_tcp_world_mm = None
        if self._preview_valid:
            try:
                tcp_mm = np.asarray(rc.tcp_mm, dtype=float).reshape(-1, 3)
                if n > 0:
                    tcp_mm = tcp_mm[:n]
                if tcp_mm.shape[0] < 2:
                    raise ValueError("compiled tcp has <2 points")
                if not np.isfinite(tcp_mm).all():
                    raise ValueError("compiled tcp contains NaN/Inf")

                final_tcp = _postprocess_compiled_tcp_mm_strict(tcp_mm, side_path_params=p_side)
                final_tcp = np.asarray(final_tcp, dtype=float).reshape(-1, 3)
                if final_tcp.shape[0] < 2:
                    raise ValueError("final tcp too short after postprocess")
                self._final_tcp_world_mm = final_tcp
            except Exception as e:
                _LOG.exception("Post-processing TCP failed")
                self._preview_valid = False
                self._preview_invalid_reason = f"postprocess_failed: {e}"
                self._final_tcp_world_mm = None

        # 6) Overlays (3D)
        try:
            out: OverlayOut = self._overlay.render_for_side(
                side=side,
                scene=self._scene,
                points_local_mm=mask_points_mm,
                raycast_result=rc,
                hit_poly=hit_poly,
                miss_poly=miss_poly,
                tcp_poly=tcp_poly,
                overlay_cfg=cfg,
            )

            if out.path_poly is not None and cfg.get("path", True):
                self.add_polydata("path", out.path_poly, name="path")
            if out.rays_hit_poly is not None and cfg.get("hits", False):
                self.add_polydata("hits", out.rays_hit_poly, name="hits")
            if out.rays_miss_poly is not None and cfg.get("misses", False):
                self.add_polydata("misses", out.rays_miss_poly, name="misses")
            if out.normals_poly is not None and cfg.get("normals", False):
                self.add_polydata("normals", out.normals_poly, name="normals")

            # TCP: show a continuous TCP line + small coordinate frames (KS) along the TCP path.
            if cfg.get("tcp", True):
                try:
                    # continuous TCP centerline (prefer final postprocessed TCP if available)
                    tcp_line_pts = None
                    if self._final_tcp_world_mm is not None and self._preview_valid:
                        tcp_line_pts = np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)
                    else:
                        vmask = np.asarray(getattr(rc, "valid", None), dtype=bool).reshape(-1)
                        tcp_all = np.asarray(getattr(rc, "tcp_mm", None), dtype=float).reshape(-1, 3)
                        if vmask.size and tcp_all.shape[0]:
                            m = min(vmask.shape[0], tcp_all.shape[0])
                            tcp_line_pts = tcp_all[:m][vmask[:m]]
                        else:
                            tcp_line_pts = tcp_all

                    if tcp_line_pts is not None and tcp_line_pts.shape[0] >= 2:
                        self.add_polydata("tcp_line", pv.lines_from_points(tcp_line_pts, close=False), name="tcp_line")

                    # frames from raycast TCP+normals (only on valid hit points)
                    vmask = np.asarray(getattr(rc, "valid", None), dtype=bool).reshape(-1)
                    tcp_all = np.asarray(getattr(rc, "tcp_mm", None), dtype=float).reshape(-1, 3)
                    n_all = np.asarray(getattr(rc, "normal", None), dtype=float).reshape(-1, 3)
                    if vmask.size and tcp_all.shape[0] and n_all.shape[0]:
                        m = min(vmask.shape[0], tcp_all.shape[0], n_all.shape[0])
                        tcp_v = tcp_all[:m][vmask[:m]]
                        n_v = n_all[:m][vmask[:m]]
                        px, py, pz = _tcp_frames_from_tangent_normal(tcp_v, n_v, max_frames=160, scale_mm=8.0)
                        if px is not None:
                            self.add_polydata("tcp_x", px, name="tcp_x")
                        if py is not None:
                            self.add_polydata("tcp_y", py, name="tcp_y")
                        if pz is not None:
                            self.add_polydata("tcp_z", pz, name="tcp_z")
                except Exception:
                    _LOG.exception("TCP overlay failed")
        except Exception:
            _LOG.exception("Overlay rendering failed")

        # 7) Path for consumer display (2D + info)
        if self._final_tcp_world_mm is not None and self._preview_valid:
            self._path_xyz_mm = np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)
        else:
            self._path_xyz_mm = np.asarray(mask_points_mm, dtype=float).reshape(-1, 3)

        return PreviewResult(
            recipe=recipe,
            valid=bool(self._preview_valid),
            invalid_reason=self._preview_invalid_reason,
            scene=self._scene,
            substrate_mesh=substrate_mesh,
            path_xyz_mm=self._path_xyz_mm,
            final_tcp_world_mm=self._final_tcp_world_mm,
            bounds=self.get_bounds(),
            substrate_bounds=self.get_substrate_bounds(),
        )

    # ------------------------------------------------------------

    @staticmethod
    def _set_policy(
        w: QWidget,
        *,
        h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
        v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
    ) -> None:
        sp = w.sizePolicy()
        sp.setHorizontalPolicy(h)
        sp.setVerticalPolicy(v)
        w.setSizePolicy(sp)
