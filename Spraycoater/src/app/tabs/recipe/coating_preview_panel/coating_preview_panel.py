# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Dict, Tuple

import numpy as np
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QWidget, QFrame, QStackedWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSizePolicy
)

from .scene_manager import SceneManager
from .matplot2d import Matplot2DView
from .overlays_groupbox import OverlaysGroupBox
from .views_groupbox import ViewsGroupBox
from app.widgets.planner_groupbox import PlannerGroupBox

_LOG = logging.getLogger("app.tabs.recipe.preview.panel")


class CoatingPreviewPanel(QWidget):
    sprayPathSetRequested = pyqtSignal(object)

    DEFAULT_GROUND_COLOR    = "#3a3a3a"
    DEFAULT_MOUNT_COLOR     = "#5d5d5d"
    DEFAULT_SUBSTRATE_COLOR = "#d0d6dd"

    def __init__(self, *, ctx, store=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        self._stack = QStackedWidget(self)
        self._stack.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        # 3D-Seite
        self._page3d = QWidget(self)
        v3d = QVBoxLayout(self._page3d); v3d.setContentsMargins(0, 0, 0, 0); v3d.setSpacing(0)
        self._host3d = QFrame(self._page3d)
        vhost = QVBoxLayout(self._host3d); vhost.setContentsMargins(0, 0, 0, 0); vhost.setSpacing(0)
        v3d.addWidget(self._host3d)
        self._stack.addWidget(self._page3d)

        # 2D-Seite
        self._page2d = QWidget(self)
        v2d = QVBoxLayout(self._page2d); v2d.setContentsMargins(0, 0, 0, 0); v2d.setSpacing(0)
        self._mat2d = Matplot2DView(parent=None)
        v2d.addWidget(self._mat2d)
        self._stack.addWidget(self._page2d)

        # SceneManager — parent_widget + container_widget explizit
        self.scene = SceneManager(parent_widget=self._page3d, container_widget=self._host3d)

        # Welt-Bounds (mm)
        self._bounds: Tuple[float, float, float, float, float, float] = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        # Overlays
        self.grpOverlays = OverlaysGroupBox(
            self,
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.add_path_polyline,
            show_poly_fn=self.show_poly,
            show_frames_at_fn=self.show_frames_at,
            set_layer_visible_fn=lambda layer, vis, render=True: self.scene.set_layer_visible(layer, vis, render=render),
            update_2d_scene_fn=lambda mesh, path_xyz, mask_poly: self.update_2d_scene(
                substrate_mesh=mesh, path_xyz=path_xyz, mask_poly=mask_poly
            ),
            layers={
                "ground": "ground",
                "mount": "mount",
                "substrate": "substrate",
                "mask": "mask",
                "mask_mrk": "mask_markers",
                "path": "path",
                "path_mrk": "path_markers",
                "rays_hit": "rays_hit",
                "rays_miss": "rays_miss",
                "normals": "normals",
                "frames_x": "frames_x",
                "frames_y": "frames_y",
                "frames_z": "frames_z",
                "frames_labels": "frames_labels",
            },
            get_bounds=self.get_bounds,
        )
        self.grpOverlays.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        root.addWidget(self.grpOverlays, 0)

        # Views: nur Callbacks zum Umschalten (kein direkter Scene-Zugriff hier)
        self.grpViews = ViewsGroupBox(
            self,
            interactor_getter=lambda: self.scene._ia(),
            render_callable=self.render,
            bounds_getter=self.get_bounds,
            cam_pad=1.8,
            activate_3d=self._switch_to_3d,
            switch_2d=self._switch_2d_plane,
        )
        self.grpViews.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        root.addWidget(self.grpViews, 0)

        root.addWidget(self._stack, 1)

        self.plannerBox = PlannerGroupBox(store=self.store, parent=self)
        self.plannerBox.setSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Maximum)
        root.addWidget(self.plannerBox, 0)

        row = QHBoxLayout(); row.setContentsMargins(0, 0, 0, 0); row.setSpacing(8)
        self.btnValidate = QPushButton("Validate", self)
        self.btnOptimize = QPushButton("Optimize", self)
        for b in (self.btnValidate, self.btnOptimize):
            b.setAutoDefault(False)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            row.addWidget(b, 1)
        root.addLayout(row)

        self.validateRequested = self.btnValidate.clicked
        self.optimizeRequested = self.btnOptimize.clicked

        # Interactor vorbereiten & 3D aktivieren
        self.ensure_interactor()
        self._switch_to_3d()
        QTimer.singleShot(0, self._push_initial_visibility)

    # --- sorgt dafür, dass der Interactor erst NACH dem Einhängen existiert ---
    def showEvent(self, ev):
        super().showEvent(ev)
        try:
            self.ensure_interactor()
        except Exception:
            _LOG.exception("showEvent.ensure_interactor failed")

    # =================== API vom RecipeTab ===================
    def handle_update_preview(self, model: object) -> None:
        """Neuer Render-Flow: Build -> Draw -> Overlays."""
        if not self.ensure_interactor():
            _LOG.error("handle_update_preview: no interactor")
            return

        view_is_3d = self.is_3d_active()
        cam_snap = self.snapshot_camera() if view_is_3d else None

        # Panels/Layers leeren
        self.clear()

        # Scene bauen & Grenzen setzen
        try:
            scene = self.scene.build_scene(self.ctx, model)
        except Exception:
            _LOG.exception("build_scene failed")
            return
        self.set_bounds(scene.bounds)

        # Grundszene zeichnen
        try:
            self.scene.draw_scene(scene, visibility=None)
        except Exception:
            _LOG.exception("draw_scene failed")

        # compile_poses -> Overlays rendern
        stand_off = 10.0
        try:
            recipe_params = getattr(model, "parameters", {}) or {}
            stand_off = float(recipe_params.get("stand_off_mm", 10.0))
        except Exception:
            pass

        compiled = None
        try:
            if hasattr(model, "compile_poses") and callable(getattr(model, "compile_poses")):
                b = scene.substrate_mesh.bounds if scene.substrate_mesh is not None else scene.bounds
                compiled = model.compile_poses(
                    bounds=b,
                    sides=None,
                    stand_off_mm=stand_off,
                    tool_frame=None,
                )
        except Exception:
            _LOG.exception("compile_poses failed")

        if compiled:
            try:
                self.grpOverlays.render_compiled(
                    substrate_mesh=scene.substrate_mesh or scene.ground_mesh,
                    compiled=compiled,
                    visibility=self._visibility_from_ui(),
                    mask_lift_mm=50.0,
                    default_stand_off_mm=stand_off,
                    ray_len_mm=1000.0,
                    sides=None,
                    only_selected=True,
                )
            except Exception:
                _LOG.exception("Overlays render failed")

        # Kamera zurück + Refresh
        try:
            if view_is_3d and cam_snap:
                self.restore_camera(cam_snap)
            self.refresh_all_views(hard_reset=not view_is_3d)
        except Exception:
            _LOG.exception("finalize view failed")

    # =================== Helpers ===================
    def _visibility_from_ui(self) -> dict:
        gb = self.grpOverlays
        return {
            "mask":   gb.chkShowMask.isChecked(),
            "path":   gb.chkShowPath.isChecked(),
            "hits":   gb.chkShowHits.isChecked(),
            "misses": gb.chkShowMisses.isChecked(),
            "normals":gb.chkShowNormals.isChecked(),
            "frames": gb.chkShowLocalFrames.isChecked(),
        }

    def _push_initial_visibility(self) -> None:
        try:
            self.grpOverlays.overlays.apply_visibility(self._visibility_from_ui())
        except Exception:
            _LOG.exception("Initial visibility push failed")

    def _emit_spraypath(self, marker_array: object) -> None:
        try:
            self.sprayPathSetRequested.emit(marker_array)
        except Exception:
            _LOG.exception("emit sprayPathSetRequested failed")

    # -------- Scene passthrough --------
    def ensure_interactor(self) -> bool:
        try:
            return bool(self.scene.ensure_interactor())
        except Exception:
            _LOG.exception("ensure_interactor failed")
            return False

    def is_3d_active(self) -> bool:
        return self._stack.currentIndex() == 0

    def _ia(self):
        return self.scene._ia()

    def snapshot_camera(self) -> Dict[str, Any] | None:
        ia = self._ia()
        if ia is None or not hasattr(ia, "camera"):
            return None
        cam = ia.camera
        try:
            return {
                "position": tuple(cam.position),
                "focal_point": tuple(cam.focal_point),
                "view_up": tuple(cam.up),
                "clipping_range": tuple(cam.clipping_range),
            }
        except Exception:
            return None

    def restore_camera(self, snap: Dict[str, Any] | None) -> None:
        if not snap:
            return
        ia = self._ia()
        if ia is None or not hasattr(ia, "camera"):
            return
        try:
            cam = ia.camera
            cam.position = snap.get("position", cam.position)
            cam.focal_point = snap.get("focal_point", cam.focal_point)
            cam.up = snap.get("view_up", cam.up)
            cr = snap.get("clipping_range", None)
            if cr:
                cam.clipping_range = cr
        except Exception:
            _LOG.exception("restore_camera failed")

    def refresh_all_views(self, *, hard_reset: bool = False) -> None:
        ia = self._ia()
        try:
            if ia is not None:
                if hard_reset:
                    ia.reset_camera(bounds=self._bounds)
                if hasattr(ia, "reset_camera_clipping_range"):
                    ia.reset_camera_clipping_range()
                ia.render()
            self._mat2d.refresh()
            if self.is_3d_active():
                if ia is not None:
                    ia.render()
            else:
                self._mat2d.refresh()
        except Exception:
            _LOG.exception("refresh_all_views failed")

    # --- Wird von ViewsGroupBox als render_callable aufgerufen ---
    def render(self, *, reset_camera: bool = False, view: str | None = None) -> None:
        ia = self._ia()
        if ia is None:
            return
        try:
            if view == "iso":
                ia.view_isometric()
            elif view == "top3d":
                ia.view_xy()
            elif view == "front3d":
                ia.view_yz()
            elif view == "back3d":
                ia.view_yz(); ia.camera.azimuth(180)
            elif view == "left3d":
                ia.view_xz()
            elif view == "right3d":
                ia.view_xz(); ia.camera.azimuth(180)

            if reset_camera:
                ia.reset_camera(bounds=self._bounds)
            if hasattr(ia, "reset_camera_clipping_range"):
                ia.reset_camera_clipping_range()
            ia.render()
        except Exception:
            _LOG.exception("render() failed")

    # -------- Stack-Steuerung --------
    def _switch_to_3d(self):
        try:
            self.ensure_interactor()
            if self._stack.currentIndex() != 0:
                self._stack.setCurrentIndex(0)
            ia = self._ia()
            if ia is not None:
                ia.render()
        except Exception:
            _LOG.exception("_switch_to_3d failed")

    def _switch_2d_plane(self, plane: str) -> None:
        try:
            self._mat2d.set_plane(plane)
            if self._stack.currentIndex() != 1:
                self._stack.setCurrentIndex(1)
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    # -------- Bounds --------
    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]) -> None:
        self._bounds = tuple(map(float, bounds))
        try:
            self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("set_bounds -> 2D failed")

    def get_bounds(self) -> Tuple[float, float, float, float, float, float]:
        return self._bounds

    def set_bounds_from_mesh(self, mesh, *, use_contact_plane=True, span_xy=240.0, span_z=240.0) -> None:
        if not hasattr(mesh, "bounds"):
            return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        z0 = float(zmin if use_contact_plane else 0.5 * (zmin + zmax))
        self.set_bounds( (cx - span_xy*0.5, cx + span_xy*0.5,
                          cy - span_xy*0.5, cy + span_xy*0.5,
                          z0, z0 + span_z) )

    # -------- 2D Scene Feeder --------
    def update_2d_scene(self, *, substrate_mesh=None, path_xyz: np.ndarray | None, mask_poly=None):
        try:
            self._mat2d.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=None if path_xyz is None else np.asarray(path_xyz, float).reshape(-1, 3),
                bounds=self._bounds,
                mask_poly=mask_poly,
            )
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("update_2d_scene failed")

    # -------- Scene passthrough --------
    def clear(self) -> None:
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    def show_poly(self, poly, *, layer: str, color: str = "royalblue", line_width: float = 2.0, lighting: bool = False):
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=lighting, layer=layer)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_frames_at(self, **kwargs) -> None:
        layer = kwargs.pop("layer", None)
        layer_prefix = layer if layer else "frames"
        for lyr in (f"{layer_prefix}_x", f"{layer_prefix}_y", f"{layer_prefix}_z", f"{layer_prefix}_labels"):
            try:
                self.scene.clear_layer(lyr)
            except Exception:
                pass
        self.scene.add_frames(layer_prefix=layer_prefix, **kwargs)
