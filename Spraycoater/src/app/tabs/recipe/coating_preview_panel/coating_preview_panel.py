# -*- coding: utf-8 -*-
# File: tabs/recipe/coating_preview_panel.py
from __future__ import annotations
import logging
from typing import Optional, Any, Dict

import numpy as np
import pyvista as pv
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QWidget, QFrame, QStackedWidget, QVBoxLayout, QHBoxLayout, QPushButton,
    QSizePolicy
)

from .interactor_host import InteractorHost
from .scene_manager import SceneManager
from .matplot2d import Matplot2DView

from .overlays_groupbox import OverlaysGroupBox
from .views_groupbox import ViewsGroupBox
from .info_groupbox import InfoGroupBox
from app.widgets.planner_groupbox import PlannerGroupBox
from app.model.recipe.recipe_store import RecipeStore

_LOG = logging.getLogger("app.tabs.recipe")


class CoatingPreviewPanel(QWidget):
    """
    Kompakter Aufbau:
      [ OverlaysGroupBox ]  (min. vertikal) -> instanziert OverlayRenderer intern
      [ ViewsGroupBox    ]  (min. vertikal) -> erzeugt/verwaltet ViewController
      [ InfoGroupBox     ]  (min. vertikal)
      [ QStackedWidget   ]  (füllt den Rest; 3D/2D)
      [ PlannerGroupBox  ]  (min. vertikal)
      [ Validate | Optimize ] (min. vertikal)
    """

    pathReady = pyqtSignal(object)       # np.ndarray | None
    previewYamlReady = pyqtSignal(str)   # YAML Text

    validateRequested = pyqtSignal()
    optimizeRequested = pyqtSignal()

    DEFAULT_GROUND_COLOR    = "#3a3a3a"
    DEFAULT_MOUNT_COLOR     = "#5d5d5d"
    DEFAULT_SUBSTRATE_COLOR = "#d0d6dd"

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        # ---------- Root Layout ----------
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ---------- Interactor + Scene ----------
        # (früh, damit Callbacks auf self.scene verfügbar sind)
        self._hoster = InteractorHost(self, None)  # Host-Frame wird gleich gesetzt
        self.scene = SceneManager(lambda: self._hoster.ia)

        self._bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)
        self._grid_step = 10.0

        # ---------- Overlays (GroupBox instanziert OverlayRenderer intern) ----------
        self.grpOverlays = OverlaysGroupBox(
            self,
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.scene.add_path_polyline,
            show_poly_fn=self.scene.show_poly,
            show_frames_at_fn=self.scene.add_frames,
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
            get_bounds=lambda: self._bounds,
            yaml_out_fn=lambda text: self.previewYamlReady.emit(text),
        )
        self.grpOverlays.set_defaults(
            mask=False, path=True, hits=False, misses=False, normals=False, local_frames=False
        )
        sp = self.grpOverlays.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Maximum)
        self.grpOverlays.setSizePolicy(sp)
        root.addWidget(self.grpOverlays, 0)

        # ---------- Stacked (3D/2D) ----------
        self._stack = QStackedWidget(self)
        self._stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # ---- 3D-Seite
        self._page3d = QWidget(self)
        v3d = QVBoxLayout(self._page3d)
        v3d.setContentsMargins(0, 0, 0, 0)
        v3d.setSpacing(0)
        self._host3d = QFrame(self._page3d)
        vhost = QVBoxLayout(self._host3d)
        vhost.setContentsMargins(0, 0, 0, 0)
        vhost.setSpacing(0)
        self._host3d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        v3d.addWidget(self._host3d)
        self._stack.addWidget(self._page3d)

        # ---- 2D-Seite
        self._page2d = QWidget(self)
        v2d = QVBoxLayout(self._page2d)
        v2d.setContentsMargins(0, 0, 0, 0)
        v2d.setSpacing(0)
        self._host2d = QWidget(self._page2d)
        vmat = QVBoxLayout(self._host2d)
        vmat.setContentsMargins(0, 0, 0, 0)
        vmat.setSpacing(0)
        self._host2d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        v2d.addWidget(self._host2d)
        self._stack.addWidget(self._page2d)

        # ---------- Interactor-Host an echten 3D-Host binden ----------
        self._hoster.parent = self
        self._hoster.container = self._host3d  # Ziel-Frame

        # ---------- 2D-View ----------
        self._mat2d = Matplot2DView(parent=None)
        self._mat2d.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._host2d.layout().addWidget(self._mat2d)

        # ---------- Views (jetzt im GroupBox) ----------
        self._current_view: str = "3d"     # "3d" | "2d"
        self._current_plane: str = "top"   # top/front/back/left/right

        self.grpViews = ViewsGroupBox(
            parent=self,
            interactor_getter=self._get_ia,
            render_callable=self.render,
            bounds_getter=lambda: self._bounds,
            cam_pad=1.8,
            activate_3d=self._switch_to_3d,
            switch_2d=self._switch_2d_plane,
        )
        sp = self.grpViews.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Maximum)
        self.grpViews.setSizePolicy(sp)
        root.addWidget(self.grpViews, 0)

        # ---------- Info ----------
        self.infoBox = InfoGroupBox(self)
        sp = self.infoBox.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Maximum)
        self.infoBox.setSizePolicy(sp)
        root.addWidget(self.infoBox, 0)

        # Stapel erst nach Views/Info einfügen, damit Layout schön ist
        root.addWidget(self._stack, 1)

        QTimer.singleShot(0, self._push_initial_visibility)

        # ---------- Planner ----------
        self.plannerBox = PlannerGroupBox(parent=self, store=self.store)
        sp = self.plannerBox.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Maximum)
        self.plannerBox.setSizePolicy(sp)
        root.addWidget(self.plannerBox, 0)

        # ---------- Validate/Optimize ----------
        row = QHBoxLayout()
        row.setContentsMargins(0, 0, 0, 0)
        row.setSpacing(8)
        self.btnValidate = QPushButton("Validate", self)
        self.btnOptimize = QPushButton("Optimize", self)
        for b in (self.btnValidate, self.btnOptimize):
            b.setAutoDefault(False)
            bsp = b.sizePolicy()
            bsp.setHorizontalPolicy(QSizePolicy.Expanding)
            bsp.setVerticalPolicy(QSizePolicy.Maximum)
            b.setSizePolicy(bsp)
            row.addWidget(b)
        root.addLayout(row, 0)

        # ---------- Initial: 3D Iso ----------
        self._switch_to_3d()

    # =================== Internals / helpers ===================

    def _get_ia(self):
        ia = self._hoster.ia
        if ia is None:
            try:
                self._hoster.ensure()
            except Exception:
                _LOG.exception("Interactor ensure() failed")
            ia = self._hoster.ia
        return ia

    def ensure_interactor(self) -> bool:
        ia = self._get_ia()
        return ia is not None

    def is_3d_active(self) -> bool:
        try:
            return self._current_view == "3d" and self._stack.currentIndex() == 0
        except Exception:
            return False

    def is_2d_active(self) -> bool:
        try:
            return self._current_view == "2d" and self._stack.currentIndex() == 1
        except Exception:
            return False

    def current_plane(self) -> str:
        return self._current_plane

    def _switch_to_3d(self):
        self._current_view = "3d"
        if self._stack.currentIndex() != 0:
            self._stack.setCurrentIndex(0)
        # Kamera-View: entscheidet ViewsGroupBox via Buttons; initial iso:
        try:
            self.grpViews.views.view_isometric()
        except Exception:
            _LOG.exception("default iso view failed")

    def _switch_2d_plane(self, plane: str) -> None:
        try:
            self._current_view = "2d"
            self._current_plane = str(plane or "top")
            self._mat2d.set_plane(self._current_plane)
            if self._stack.currentIndex() != 1:
                self._stack.setCurrentIndex(1)
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    def _push_initial_visibility(self) -> None:
        try:
            vis = {
                "mask":   self.grpOverlays.chkShowMask.isChecked(),
                "path":   self.grpOverlays.chkShowPath.isChecked(),
                "hits":   self.grpOverlays.chkShowHits.isChecked(),
                "misses": self.grpOverlays.chkShowMisses.isChecked(),
                "normals":self.grpOverlays.chkShowNormals.isChecked(),
                "frames": self.grpOverlays.chkShowLocalFrames.isChecked(),
            }
            # Sichtbarkeiten an GroupBox (die ihren Renderer kennt)
            self.grpOverlays.apply_visibility(vis)
        except Exception:
            _LOG.exception("Initial visibility push failed")

    # =================== Public API (Hosts) ===================

    def preview_host(self) -> QFrame:
        return self._host3d

    def attach_interactor(self, interactor: Any) -> None:
        self._hoster.attach(interactor)

    # --- optional: direkter Zugriff auf den OverlayRenderer, falls benötigt ---
    def overlays(self):
        """Convenience: direkten Renderer liefern (lebt in der GroupBox)."""
        return getattr(self.grpOverlays, "overlays", None)

    # --- ✨ NEW: public setter for info box ---
    def set_runtime_info(self, info: Dict[str, Any]) -> None:
        try:
            if hasattr(self, "infoBox") and self.infoBox is not None:
                self.infoBox.set_values(info or {})
        except Exception:
            _LOG.exception("InfoGroupBox update failed")

    # -------- Camera snapshot helpers (unverändert) --------
    def snapshot_camera(self) -> Optional[dict]:
        ia = self._get_ia()
        if ia is None:
            return None
        try:
            cam = ia.camera
            return {
                "position": tuple(getattr(cam, "position", (0.0, 0.0, 1.0))),
                "focal_point": tuple(getattr(cam, "focal_point", (0.0, 0.0, 0.0))),
                "view_up": tuple(getattr(cam, "up", (0.0, 1.0, 0.0))),
                "parallel_projection": bool(getattr(cam, "parallel_projection", False)),
                "parallel_scale": float(getattr(cam, "parallel_scale", 1.0)),
                "clipping_range": tuple(getattr(cam, "clipping_range", (0.1, 10000.0))),
                "view_angle": float(getattr(cam, "view_angle", 30.0)),
            }
        except Exception:
            _LOG.exception("snapshot_camera failed")
            return None

    def restore_camera(self, snap: Optional[dict]) -> None:
        return
        # (wie vorher; derzeit nicht genutzt)

    # -------- 2D Scene Feeder --------
    def update_2d_scene(
        self,
        *,
        substrate_mesh: pv.PolyData | None,
        path_xyz: np.ndarray | None,
        mask_poly: pv.PolyData | None = None
    ):
        self._mat2d.set_scene(
            substrate_mesh=substrate_mesh,
            path_xyz=path_xyz,
            bounds=self._bounds,
            mask_poly=mask_poly,
        )
        try:
            self.pathReady.emit(path_xyz if path_xyz is not None else None)
        except Exception:
            _LOG.exception("pathReady emit failed")

    # -------- Render/Bounds/Scene (unverändert) --------
    def refresh_current_view(self, *, hard_reset: bool = False) -> None:
        try:
            if self.is_3d_active():
                self.render(reset_camera=bool(hard_reset))
            else:
                self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("refresh_current_view failed")

    def render(self, *, reset_camera: bool = True) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            xmin, xmax, ymin, ymax, zmin, _ = self._bounds
            target = np.array([(xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin], dtype=float)
            ia.camera.focal_point = tuple(target.tolist())
            if reset_camera:
                ia.reset_camera(bounds=self._bounds)
            ia.render()
        except Exception:
            _LOG.exception("render() failed")

    def set_grid_step(self, step_mm: float) -> None:
        self._grid_step = max(1.0, float(step_mm))
        self._apply_bounds()

    def set_world_bounds_at(self, *, center_xy=(0.0, 0.0), z0=0.0, span_xy=240.0, span_z=240.0) -> None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        half = float(span_xy) * 0.5
        z0, span_z = float(z0), float(span_z)
        self._bounds = (cx - half, cx + half, cy - half, cy + half, z0, z0 + span_z)
        self._apply_bounds()

    def set_bounds_from_mesh(self, mesh, *, use_contact_plane=True, span_xy=240.0, span_z=240.0) -> None:
        if not hasattr(mesh, "bounds"):
            return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        cx = 0.5 * (xmin + xmax); cy = 0.5 * (ymin + ymax)
        z0 = float(zmin if use_contact_plane else 0.5 * (zmin + zmax))
        self.set_world_bounds_at(center_xy=(cx, cy), z0=z0, span_xy=span_xy, span_z=span_z)

    def _apply_bounds(self) -> None:
        try:
            if self.ensure_interactor():
                self.scene.refresh_floor(bounds=self._bounds, step=self._grid_step)
            self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("apply_bounds: refresh_floor failed")

    def clear(self) -> None:
        if not self.ensure_interactor():
            return
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        if not self.ensure_interactor():
            return
        _ = self._get_ia()
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        if not self.ensure_interactor():
            return
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        if not self.ensure_interactor():
            return
        self.scene.add_path_polyline(*a, **kw)
