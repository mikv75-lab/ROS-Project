# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any

import numpy as np
import pyvista as pv
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import (
    QWidget, QFrame, QStackedWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSizePolicy
)

from .interactor_host import InteractorHost
from .scene_manager import SceneManager
from .views import ViewController
from .matplot2d import Matplot2DView
from .overlays import OverlayRenderer

from .overlays_groupbox import OverlaysGroupBox
from .views_groupbox import ViewsGroupBox
from app.widgets.planner_groupbox import PlannerGroupBox

_LOG = logging.getLogger("app.tabs.recipe")


class CoatingPreviewPanel(QWidget):
    """
    Layout (reine Programmierung):
      [ OverlaysGroupBox (eine Zeile, horizontal) ]
      [ ViewsGroupBox  ("Kamera", 3D & 2D)       ]
      [ QStackedWidget (3D-PyVista / 2D-Matplot) ]  <- bekommt am meisten Platz
      [ PlannerGroupBox (kompakt, vertikal gequetscht) ]
      [  Validate | Optimize  ]  (HBox, Buttons gestreckt)
    """

    # Ausgänge
    pathReady = pyqtSignal(object)       # np.ndarray | None
    previewYamlReady = pyqtSignal(str)   # YAML Text

    # Aktionen
    validateRequested = pyqtSignal()
    optimizeRequested = pyqtSignal()

    # Farben (falls du später im SceneManager einfärben willst)
    DEFAULT_GROUND_COLOR    = "#3a3a3a"
    DEFAULT_MOUNT_COLOR     = "#5d5d5d"
    DEFAULT_SUBSTRATE_COLOR = "#d0d6dd"

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # ---------- Root Layout ----------
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ---------- Overlays ----------
        self.grpOverlays = OverlaysGroupBox(self)
        self.grpOverlays.set_defaults(
            mask=False, path=True, hits=False, misses=False, normals=False, local_frames=False
        )
        root.addWidget(self.grpOverlays, 0)

        # ---------- Views ----------
        self.grpViews = ViewsGroupBox(self)
        root.addWidget(self.grpViews, 0)

        # ---------- Stacked (3D/2D) ----------
        self._stack = QStackedWidget(self)
        root.addWidget(self._stack, 1)

        # ---- 3D-Seite
        self._page3d = QWidget(self)
        v3d = QVBoxLayout(self._page3d); v3d.setContentsMargins(0, 0, 0, 0); v3d.setSpacing(0)
        self._host3d = QFrame(self._page3d)
        vhost = QVBoxLayout(self._host3d); vhost.setContentsMargins(0, 0, 0, 0); vhost.setSpacing(0)
        v3d.addWidget(self._host3d)
        self._stack.addWidget(self._page3d)

        # ---- 2D-Seite
        self._page2d = QWidget(self)
        v2d = QVBoxLayout(self._page2d); v2d.setContentsMargins(0, 0, 0, 0); v2d.setSpacing(0)
        self._host2d = QWidget(self._page2d)
        vmat = QVBoxLayout(self._host2d); vmat.setContentsMargins(0, 0, 0, 0); vmat.setSpacing(0)
        v2d.addWidget(self._host2d)
        self._stack.addWidget(self._page2d)

        # ---------- Interactor + Scene ----------
        self._hoster = InteractorHost(self, self._host3d)
        self.scene = SceneManager(lambda: self._hoster.ia)

        # Welt-/View-Bounds (mm): xmin, xmax, ymin, ymax, zmin, zmax
        self._bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)
        self._grid_step = 10.0

        # View-Controller
        self.views = ViewController(
            interactor_getter=self._get_ia,
            render_callable=self.render,
            bounds_getter=lambda: self._bounds,
            cam_pad=1.8,
        )

        # ---------- 2D-View ----------
        self._mat2d = Matplot2DView(parent=None)
        self._host2d.layout().addWidget(self._mat2d)

        # ---------- OverlayRenderer ----------
        layers = {
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
        }
        self.overlays = OverlayRenderer(
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.scene.add_path_polyline,
            show_poly_fn=self.scene.show_poly,
            show_frames_at_fn=self.scene.add_frames,  # Frames jetzt im SceneManager
            set_layer_visible_fn=lambda layer, vis, render=True: self.scene.set_layer_visible(layer, vis, render=render),
            update_2d_scene_fn=lambda mesh, path_xyz, mask_poly: self.update_2d_scene(
                substrate_mesh=mesh, path_xyz=path_xyz, mask_poly=mask_poly
            ),
            layers=layers,
            get_bounds=lambda: self._bounds,
            yaml_out_fn=lambda text: self.previewYamlReady.emit(text),
        )

        # ---------- Overlay-Toggles ----------
        self.grpOverlays.maskToggled.connect(lambda v: self.overlays.set_mask_visible(bool(v)))
        self.grpOverlays.pathToggled.connect(lambda v: self.overlays.set_path_visible(bool(v)))
        self.grpOverlays.hitsToggled.connect(lambda v: self.overlays.set_hits_visible(bool(v)))
        self.grpOverlays.missesToggled.connect(lambda v: self.overlays.set_misses_visible(bool(v)))
        self.grpOverlays.normalsToggled.connect(lambda v: self.overlays.set_normals_visible(bool(v)))
        self.grpOverlays.localFramesToggled.connect(lambda v: self.overlays.set_frames_visible(bool(v)))

        # ---------- View-Buttons (Wiring direkt im Widget) ----------
        self.grpViews.set_handlers(
            on3d=lambda name: (
                self._switch_to_3d(),
                {
                    "iso":   self.view_isometric,
                    "top":   self.view_top,
                    "front": self.view_front,
                    "back":  self.view_back,
                    "left":  self.view_left,
                    "right": self.view_right,
                }[name]()
            ),
            on2d=self._switch_2d_plane,
        )

        # Initiale Sichtbarkeit pushen (erzwingt Render, wenn path=True)
        QTimer.singleShot(0, self._push_initial_visibility)

        # ---------- Planner ----------
        self.plannerBox = PlannerGroupBox(parent=self)
        self.plannerBox.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Maximum)
        self.plannerBox.setMaximumHeight(220)
        if self.plannerBox.layout() is not None:
            self.plannerBox.layout().setContentsMargins(6, 6, 6, 6)
        root.addWidget(self.plannerBox, 0)

        # ---------- Validate/Optimize ----------
        row = QHBoxLayout(); row.setContentsMargins(0, 0, 0, 0); row.setSpacing(8)
        self.btnValidate = QPushButton("Validate", self)
        self.btnOptimize = QPushButton("Optimize", self)
        for b in (self.btnValidate, self.btnOptimize):
            b.setAutoDefault(False)
            b.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
            row.addWidget(b, 1)
        root.addLayout(row)

        self.btnValidate.clicked.connect(self.validateRequested.emit)
        self.btnOptimize.clicked.connect(self.optimizeRequested.emit)

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

    def _switch_to_3d(self):
        if self._stack.currentIndex() != 0:
            self._stack.setCurrentIndex(0)
        self.view_isometric()

    def _switch_2d_plane(self, plane: str) -> None:
        try:
            self._mat2d.set_plane(plane)
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
            self.overlays.apply_visibility(vis)
        except Exception:
            _LOG.exception("Initial visibility push failed")

    # =================== Public API (Hosts) ===================

    def preview_host(self) -> QFrame:
        return self._host3d

    def attach_interactor(self, interactor: Any) -> None:
        self._hoster.attach(interactor)

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

    # -------- Views (Delegates) --------
    def view_isometric(self): self.views.view_isometric()
    def view_top(self):       self.views.view_top()
    def view_front(self):     self.views.view_front()
    def view_back(self):      self.views.view_back()
    def view_left(self):      self.views.view_left()
    def view_right(self):     self.views.view_right()

    def render(self, *, reset_camera: bool = True) -> None:
        ia = self._get_ia()
        if ia is None:
            return
        try:
            # Kamera-Fokus auf Bodenmitte
            xmin, xmax, ymin, ymax, zmin, _ = self._bounds
            target = np.array([(xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin], dtype=float)
            ia.camera.focal_point = tuple(target.tolist())
            if reset_camera:
                ia.reset_camera(bounds=self._bounds)
            ia.render()
        except Exception:
            _LOG.exception("render() failed")

    # -------- Grid/Bounds --------
    def set_grid_step(self, step_mm: float) -> None:
        self._grid_step = max(1.0, float(step_mm))
        self._apply_bounds()

    def set_world_bounds_at(
        self, *, center_xy=(0.0, 0.0), z0=0.0, span_xy=240.0, span_z=240.0
    ) -> None:
        cx, cy = float(center_xy[0]), float(center_xy[1])
        half = float(span_xy) * 0.5
        z0, span_z = float(z0), float(span_z)
        self._bounds = (cx - half, cx + half, cy - half, cy + half, z0, z0 + span_z)
        self._apply_bounds()

    def set_bounds_from_mesh(
        self, mesh, *, use_contact_plane=True, span_xy=240.0, span_z=240.0
    ) -> None:
        if not hasattr(mesh, "bounds"):
            return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        cx = 0.5 * (xmin + xmax); cy = 0.5 * (ymin + ymax)
        z0 = float(zmin if use_contact_plane else 0.5 * (zmin + zmax))
        self.set_world_bounds_at(center_xy=(cx, cy), z0=z0, span_xy=span_xy, span_z=span_z)

    def _apply_bounds(self) -> None:
        try:
            # komplette Deko an SceneManager delegiert
            self.scene.refresh_floor(bounds=self._bounds, step=self._grid_step)
        except Exception:
            _LOG.exception("apply_bounds: refresh_floor failed")

    # -------- Scene passthrough --------
    def clear(self) -> None:
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        _ = self._get_ia()
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)
