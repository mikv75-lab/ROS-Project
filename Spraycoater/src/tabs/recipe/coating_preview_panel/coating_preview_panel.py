# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations

import logging
from typing import Any, Dict, Optional, Tuple
import numpy as np

from PyQt6.QtCore import QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QVBoxLayout,
    QWidget,
    QTabWidget,
    QSizePolicy,
)
from PyQt6.sip import isdeleted

# --- Models & Widgets ---
from model.recipe.path_builder import PathBuilder
from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from widgets.info_groupbox import InfoGroupBox

# --- UI Sub-Components (Tabs) ---
from .tab2d.tab2d import Tab2D
from .tab3d.tab3d import Tab3D

# --- Logic Components (located in tab3d folder) ---
from .tab3d.overlays import OverlayOut, OverlayRenderer
from .tab3d.raycast_projector import cast_rays_for_side
from .tab3d.scene_manager import PreviewScene, SceneManager

import pyvista as pv

_LOG = logging.getLogger("tabs.recipe.preview.panel")

Bounds = Tuple[float, float, float, float, float, float]
_ALLOWED_SIDES = ("top", "front", "back", "left", "right", "polyhelix", "helix")


# --- Helpers ---

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

def _set_policy(w: QWidget, *, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)

def _postprocess_compiled_tcp_mm_strict(P: np.ndarray, *, side_path_params: Dict[str, Any]) -> np.ndarray:
    """
    STRICT postprocess applied AFTER raycast (compiled TCP).
    Trims path based on parameters and adds approach/retreat moves.
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
                        if seg_len <= 1e-12: return P[i].copy()
                        u = (t - float(s[i])) / seg_len
                        return (1.0 - u) * P[i] + u * P[i + 1]
                    pts = [sample_at(a)]
                    for i in range(1, len(P) - 1):
                        if a <= float(s[i]) <= b: pts.append(P[i].copy())
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


class CoatingPreviewPanel(QWidget):
    """
    Panel zur Vorschau der Beschichtungsbahn.
    Layout:
      - Oben: InfoGroupBox
      - Unten: QTabWidget (2D View / 3D View)
    """
    sig_request_update = pyqtSignal()
    sig_preview_updated = pyqtSignal()

    def __init__(
        self,
        *,
        ctx: Any,
        store: RecipeStore,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self.ctx = ctx
        if store is None or not isinstance(store, RecipeStore):
            raise TypeError(f"CoatingPreviewPanel: store invalid: {type(store)}")
        self.store: RecipeStore = store

        self._recipe: Optional[Recipe] = None
        self._busy = False

        # Status Flags
        self._preview_valid: bool = False
        self._preview_invalid_reason: Optional[str] = "no_preview"
        self._final_tcp_world_mm: Optional[np.ndarray] = None

        # Logic Helpers
        self._scene_mgr = SceneManager()
        self._scene: Optional[PreviewScene] = None
        self._overlay = OverlayRenderer()

        # Update Timer (Debounce)
        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.timeout.connect(self._on_update_timer)

        # UI Construction
        self._build_ui()
        self._wire_signals()
        
        # Initial Reset
        self._update_info(None, None)

    # ---------------- Public API ----------------

    def get_pv_host(self) -> QWidget:
        # Delegated to the inner Tab3D
        return self._tab3d.get_pv_host()

    def update_preview(self, model: Optional[Recipe]) -> None:
        self._recipe = model
        self.request_update()

    def request_update(self) -> None:
        if isdeleted(self):
            return
        self._timer.start(30)

    def render(self) -> None:
        # Delegate render to Tab3D (which holds the plotter)
        self._tab3d.render()

    def preview_is_valid(self) -> bool:
        return bool(self._preview_valid)

    def preview_invalid_reason(self) -> Optional[str]:
        return self._preview_invalid_reason

    def final_tcp_world_mm(self) -> Optional[np.ndarray]:
        if self._final_tcp_world_mm is None:
            return None
        return np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)

    # ---------------- Bounds Getters ----------------
    
    def get_bounds(self) -> Bounds:
        """Liefert Szene-Bounds oder Default."""
        if self._scene and getattr(self._scene, "bounds", None):
            return tuple(self._scene.bounds) # type: ignore
        return (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

    def get_substrate_bounds(self) -> Optional[Bounds]:
        if self._scene and getattr(self._scene, "substrate_mesh", None) is not None:
            try:
                b = self._scene.substrate_mesh.bounds # type: ignore
                return (float(b[0]), float(b[1]), float(b[2]), float(b[3]), float(b[4]), float(b[5]))
            except Exception: return None
        return None

    # ---------------- Info box ----------------

    def _update_info(self, recipe: Optional[Recipe], points_mm: Optional[np.ndarray]) -> None:
        try:
            self._info_box.update_from_recipe(recipe, points_mm)
        except Exception:
            _LOG.exception("InfoGroupBox.update_from_recipe failed")

    # ---------------- UI Construction ----------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # 1. Info Header
        self._info_box = InfoGroupBox(parent=self, title="Preview")
        _set_policy(self._info_box, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        root.addWidget(self._info_box, 0)

        # 2. Tab Widget
        self._tabs = QTabWidget(self)
        root.addWidget(self._tabs, 1)

        # Tab 1: 2D
        self._tab2d = Tab2D(
            parent=self,
            refresh_callable=self.render,
            get_bounds_callable=self.get_bounds
        )
        self._tabs.addTab(self._tab2d, "2D View")

        # Tab 2: 3D
        self._tab3d = Tab3D(
            parent=self,
            render_callable=self.render,
            get_bounds_callable=self.get_bounds,
            get_substrate_bounds_callable=self.get_substrate_bounds,
            on_overlay_changed=self.request_update # Trigger update on check change
        )
        self._tabs.addTab(self._tab3d, "3D View")

        # Set default tab
        self._tabs.setCurrentIndex(1) # Default to 3D View usually better

    def _wire_signals(self) -> None:
        self.sig_request_update.connect(self.request_update)
        # Tab3D overlay changes are connected via init callback

    # ---------------- ValidSave flag ----------------

    def _set_recipe_valid_save(self, recipe: Recipe, ok: bool, reason: Optional[str]) -> None:
        try:
            setattr(recipe, "validSave", bool(ok))
            setattr(recipe, "validSaveReason", (None if ok else str(reason or "invalid")))
            meta = getattr(recipe, "meta", None)
            if isinstance(meta, dict):
                meta["valid_save"] = bool(ok)
                meta["valid_save_reason"] = (None if ok else str(reason or "invalid"))
        except Exception: pass

    # ---------------- Update pipeline ----------------

    def _on_update_timer(self) -> None:
        if self._busy: return
        self._busy = True
        try:
            self._update_preview()
        except Exception:
            _LOG.exception("update_preview failed")
        finally:
            self._busy = False

    def _update_preview(self) -> None:
        recipe = self._recipe
        
        # Reset Status
        self._preview_valid = False
        self._preview_invalid_reason = "no_preview"
        self._final_tcp_world_mm = None
        
        # Reset Views
        self._tab2d.update_scene(substrate_mesh=None, path_xyz=None, bounds=self.get_bounds())
        self._tab3d.clear_layers()

        if recipe is None:
            self._update_info(None, None)
            self.render()
            return

        # 1) Build Scene (Using SceneManager which uses mesh_utils)
        try:
            # We MUST pass ctx here so SceneManager can resolve paths (package:// etc.)
            self._scene = self._scene_mgr.build_scene(recipe, ctx=self.ctx)
        except Exception as e:
            _LOG.exception("Scene build failed")
            self._scene = None
            self._update_info(recipe, None)
            self._set_recipe_valid_save(recipe, False, f"scene_build_failed: {e}")
            self.render()
            return

        # Add Scene Meshes to 3D Tab
        substrate_mesh = self._scene.substrate_mesh if self._scene else None
        try:
            if self._scene and self._scene.cage_mesh:
                self._tab3d.add_polydata("cage", self._scene.cage_mesh, name="cage", opacity=0.20)
            if self._scene and self._scene.mount_mesh:
                self._tab3d.add_polydata("mount", self._scene.mount_mesh, name="mount", opacity=0.35)
            if substrate_mesh:
                self._tab3d.add_polydata("substrate", substrate_mesh, name="substrate", opacity=0.55)
        except Exception:
            _LOG.exception("draw base meshes failed")

        # 2) Params
        side = str(getattr(recipe, "active_side", None) or "top").lower()
        if side not in _ALLOWED_SIDES: side = "top"
        globals_params = dict(getattr(recipe, "parameters", {}) or {})
        sample_step_mm = _as_float(globals_params.get("sample_step_mm", 1.0), 1.0)
        max_points = int(globals_params.get("max_points", 1000) or 1000)
        stand_off_mm = _as_float(globals_params.get("stand_off_mm", 50.0), 50.0)

        # 3) Mask Path Generation
        p_side = {}
        try:
            pbs = getattr(recipe, "paths_by_side", {}) or {}
            p_side = dict(pbs.get(side) or {})
            
            # Create Path (typically at Z=0)
            pd_mask = PathBuilder._from_path_dict(
                p_side,
                sample_step_mm=float(sample_step_mm),
                max_points=int(max_points),
            )
            pd_mask = PathBuilder._with_globals_meta(pd_mask, globals_params)
            mask_points_mm = np.asarray(pd_mask.points_mm, dtype=float).reshape(-1, 3)
            meta = dict(pd_mask.meta or {})
            path_source = str(meta.get("source", ""))

            # FIX: Lift Mask above Substrate
            # Find top of substrate (max Z) and add stand_off
            base_z = 0.0
            if substrate_mesh is not None:
                base_z = substrate_mesh.bounds[5] 
            
            if mask_points_mm.shape[0] > 0:
                mask_points_mm[:, 2] += (base_z + float(stand_off_mm))
                mask_points_mm[:, 2] += _as_float(p_side.get("z_mm", 0.0))

        except Exception as e:
            _LOG.exception(f"Path generation failed for side '{side}'")
            self._update_info(recipe, None)
            self._set_recipe_valid_save(recipe, False, f"path_build_failed: {e}")
            self.render()
            return

        if mask_points_mm.shape[0] < 2:
            self._update_info(recipe, mask_points_mm)
            self._set_recipe_valid_save(recipe, False, "mask_too_short")
            self.render()
            return

        # Show Mask (3D)
        cfg = self._tab3d.get_overlay_config()
        if cfg.get("mask", True):
            try:
                self._tab3d.add_polydata("mask", pv.lines_from_points(mask_points_mm, close=False), name="mask", opacity=1.0)
            except Exception: _LOG.exception("draw mask failed")

        # 4) Raycast
        if substrate_mesh is None:
            self._update_info(recipe, mask_points_mm)
            self._preview_valid = False
            self._preview_invalid_reason = "no_substrate_mesh"
            self._set_recipe_valid_save(recipe, False, self._preview_invalid_reason)
            # Update 2D with mask only
            self._tab2d.update_scene(substrate_mesh=None, path_xyz=mask_points_mm, bounds=self.get_bounds())
            self.render()
            return

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
            self._update_info(recipe, mask_points_mm)
            self._set_recipe_valid_save(recipe, False, self._preview_invalid_reason)
            self._tab2d.update_scene(substrate_mesh=substrate_mesh, path_xyz=mask_points_mm, bounds=self.get_bounds())
            self.render()
            return

        # Misses check
        try:
            valid = np.asarray(rc.valid, dtype=bool).reshape(-1)
        except Exception: valid = np.zeros((0,), dtype=bool)
        
        total_n = int(mask_points_mm.shape[0])
        n = min(total_n, int(valid.shape[0])) if valid.size else 0
        hit_n = int(np.sum(valid[:n])) if n > 0 else 0
        miss_n = int(n - hit_n) + int(max(0, total_n - n))

        self._preview_valid = (miss_n == 0)
        self._preview_invalid_reason = None if self._preview_valid else f"raycast_misses(count={miss_n})"

        # 5) Postprocess
        self._final_tcp_world_mm = None
        if self._preview_valid:
            try:
                tcp_mm = np.asarray(rc.tcp_mm, dtype=float).reshape(-1, 3)
                if n > 0: tcp_mm = tcp_mm[:n]
                if tcp_mm.shape[0] < 2: raise ValueError("compiled tcp has <2 points")
                if not np.isfinite(tcp_mm).all(): raise ValueError("compiled tcp contains NaN/Inf")

                final_tcp = _postprocess_compiled_tcp_mm_strict(tcp_mm, side_path_params=p_side)
                final_tcp = np.asarray(final_tcp, dtype=float).reshape(-1, 3)
                if final_tcp.shape[0] < 2: raise ValueError("final tcp too short after postprocess")
                self._final_tcp_world_mm = final_tcp
            except Exception as e:
                _LOG.exception("Post-processing TCP failed")
                self._preview_valid = False
                self._preview_invalid_reason = f"postprocess_failed: {e}"
                self._final_tcp_world_mm = None

        # 6) Overlays (3D)
        try:
            out: OverlayOut = self._overlay.render_for_side(
                side=side, scene=self._scene, points_local_mm=mask_points_mm,
                raycast_result=rc, hit_poly=hit_poly, miss_poly=miss_poly, tcp_poly=tcp_poly,
                overlay_cfg=cfg,
            )
            if out.path_poly is not None and cfg.get("path", True):
                self._tab3d.add_polydata("path", out.path_poly, name="path", opacity=1.0)
            if out.rays_hit_poly is not None and cfg.get("hits", False):
                self._tab3d.add_polydata("hits", out.rays_hit_poly, name="hits", opacity=1.0)
            if out.rays_miss_poly is not None and cfg.get("misses", False):
                self._tab3d.add_polydata("misses", out.rays_miss_poly, name="misses", opacity=1.0)
            if out.normals_poly is not None and cfg.get("normals", False):
                self._tab3d.add_polydata("normals", out.normals_poly, name="normals", opacity=1.0)
            if out.tcp_poly is not None and cfg.get("tcp", True):
                self._tab3d.add_polydata("tcp", out.tcp_poly, name="tcp", opacity=1.0)
        except Exception:
            _LOG.exception("Overlay rendering failed")

        # 7) 2D Update
        path_for_display = mask_points_mm
        if self._final_tcp_world_mm is not None and self._preview_valid:
            path_for_display = np.asarray(self._final_tcp_world_mm, dtype=float).reshape(-1, 3)
        
        self._tab2d.update_scene(
            substrate_mesh=substrate_mesh,
            path_xyz=path_for_display,
            bounds=self.get_bounds()
        )

        # 8) Finalize
        self._update_info(recipe, path_for_display)
        self._set_recipe_valid_save(recipe, self._preview_valid, self._preview_invalid_reason)
        self.render()
        self.sig_preview_updated.emit()