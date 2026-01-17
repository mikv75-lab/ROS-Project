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

    def add_polydata(self, layer: str, poly: Any, *, name: str = "", opacity: float = 1.0) -> None:
        """Adds a mesh/polydata to a specific layer."""
        if self.pv_plot is None or poly is None:
            return

        # Avoid empty mesh crashes
        if hasattr(poly, "n_points") and getattr(poly, "n_points") == 0:
            return

        if layer not in self._layers:
            self._layers[layer] = _LayerActors(actors=[])

        try:
            a = self.pv_plot.add_mesh(
                poly,
                name=str(name or layer),
                opacity=float(opacity),
                pickable=False,
            )
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
                self.add_polydata("cage", self._scene.cage_mesh, name="cage", opacity=0.20)
            if self._scene and self._scene.mount_mesh is not None:
                self.add_polydata("mount", self._scene.mount_mesh, name="mount", opacity=0.35)
            if substrate_mesh is not None:
                self.add_polydata("substrate", substrate_mesh, name="substrate", opacity=0.55)
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
                    self.add_polydata("mask", pv.lines_from_points(mask_points_mm, close=False), name="mask", opacity=1.0)
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
                self.add_polydata("mask", pv.lines_from_points(mask_points_mm, close=False), name="mask", opacity=1.0)
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
                self.add_polydata("path", out.path_poly, name="path", opacity=1.0)
            if out.rays_hit_poly is not None and cfg.get("hits", False):
                self.add_polydata("hits", out.rays_hit_poly, name="hits", opacity=1.0)
            if out.rays_miss_poly is not None and cfg.get("misses", False):
                self.add_polydata("misses", out.rays_miss_poly, name="misses", opacity=1.0)
            if out.normals_poly is not None and cfg.get("normals", False):
                self.add_polydata("normals", out.normals_poly, name="normals", opacity=1.0)
            if out.tcp_poly is not None and cfg.get("tcp", True):
                self.add_polydata("tcp", out.tcp_poly, name="tcp", opacity=1.0)
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
