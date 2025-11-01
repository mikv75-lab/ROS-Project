# -*- coding: utf-8 -*-
from __future__ import annotations
import os, logging
from typing import Optional, Tuple, Any

import numpy as np
import pyvista as pv
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame

from .interactor_host import InteractorHost
from .grid_manager import GridManager
from .scene_manager import SceneManager
from .views import ViewController

_LOG = logging.getLogger("app.tabs.recipe")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class CoatingPreviewPanel(QWidget):
    DEFAULT_MOUNT_COLOR = "lightgray"
    DEFAULT_SUBSTRATE_COLOR = "#3498db"

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # UI-Refs
        self.chkShowMask: QCheckBox = self.findChild(QCheckBox, "chkShowMask")
        self.chkShowRays: QCheckBox = self.findChild(QCheckBox, "chkShowRays")
        self.chkShowLocalFrames: QCheckBox = self.findChild(QCheckBox, "chkShowLocalFrames")
        self.btnCamIso:   QPushButton = self.findChild(QPushButton, "btnCamIso")
        self.btnCamTop:   QPushButton = self.findChild(QPushButton, "btnCamTop")
        self.btnCamFront: QPushButton = self.findChild(QPushButton, "btnCamFront")
        self.btnCamBack:  QPushButton = self.findChild(QPushButton, "btnCamBack")
        self.btnCamLeft:  QPushButton = self.findChild(QPushButton, "btnCamLeft")
        self.btnCamRight: QPushButton = self.findChild(QPushButton, "btnCamRight")
        self.btnValidate: QPushButton = self.findChild(QPushButton, "btnValidate")
        self._host: QFrame = self.findChild(QFrame, "previewHost")
        if self._host is None:
            raise RuntimeError("coating_preview_panel.ui braucht QFrame 'previewHost'.")

        # Composition
        self._hoster = InteractorHost(self, self._host)
        self.grid = GridManager(lambda: self._hoster.ia)

        # SceneManager NICHT mehr veranlassen, bei clear() ein Grid aufzubauen.
        # (SceneManager.clear() ist bereits so angepasst, dass es nur leert.)
        self.scene = SceneManager(lambda: self._hoster.ia, self.grid.build_init_scene)
        self.views = ViewController(lambda: self._hoster.ia, self.render)

        # Kamera-Buttons
        self.views.wire_buttons(
            btn_iso=self.btnCamIso, btn_top=self.btnCamTop, btn_front=self.btnCamFront,
            btn_left=self.btnCamLeft, btn_right=self.btnCamRight, btn_back=self.btnCamBack
        )

    # --- Hosting API (MainWindow ruft das) ---
    def preview_host(self) -> QFrame:
        return self._host

    def attach_interactor(self, interactor: Any) -> None:
        self._hoster.attach(interactor)

    # --- Public API (vom RecipeTab/Controller verwendet) ---
    def build_init_scene_mainstyle(self, grid_step: float = 10.0):
        # nicht mehr automatisch aufrufen – bleibt aber aufrufbar über deinen Init-Button
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("build_init_scene_mainstyle(): kein Interactor")
            return
        self.grid.step = grid_step
        self.grid.build_init_scene_mainstyle()

    def build_init_scene(self, bounds=None, grid_step: float = 10.0):
        # dito: nur auf expliziten Aufruf
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("build_init_scene(): kein Interactor")
            return
        if bounds is not None:
            self.grid.bounds = bounds
        self.grid.step = grid_step
        self.grid.build_init_scene()

    def clear(self) -> None:
        # leert wirklich alles, baut KEIN Grid neu auf
        if self._hoster.ia is None and not self._hoster.ensure():
            return
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("add_mesh(): kein Interactor")
            return
        self.scene.add_mesh(mesh, **kwargs)

    # Layer-Shortcuts
    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    # Pfad-Shortcuts
    def add_path_polyline(self, *a, **kw):
        self.scene.add_path_polyline(*a, **kw)

    def add_path_markers(self, *a, **kw):
        self.scene.add_path_markers(*a, **kw)

    # --- generische Poly + Normalen-Linien ----------------------
    def show_poly(self, poly: pv.PolyData, *, layer: str,
                  color: str = "#f39c12", line_width: float = 1.0, lighting: bool = False):
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=lighting)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_normals_from_hits(self, points_mm: np.ndarray, normals: np.ndarray,
                               *, layer: str = "normals", length_mm: float = 8.0,
                               color: str = "#27ae60", line_width: float = 1.0):
        try:
            P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
            N = np.asarray(normals,   dtype=float).reshape(-1, 3)
            M = min(len(P), len(N))
            if M == 0:
                self.clear_layer(layer)
                return

            A = P[:M]
            B = P[:M] + N[:M] * float(length_mm)

            pts = np.vstack([A, B])
            lines = np.empty((M, 3), dtype=np.int64)
            lines[:, 0] = 2
            lines[:, 1] = np.arange(M, dtype=np.int64)
            lines[:, 2] = np.arange(M, dtype=np.int64) + M
            poly = pv.PolyData(pts)
            poly.lines = lines.reshape(-1)

            self.clear_layer(layer)
            self.add_mesh(poly, color=color, render=False, reset_camera=False,
                          line_width=float(line_width), lighting=False)
        except Exception:
            _LOG.exception("show_normals_from_hits() failed")

    # --- Grid-Zentrierung (Explizit, nach Substrat-Spawn aufrufen) -----
    def center_grid_on_point(self, cx: float, cy: float, cz: float,
                             *, span_xy: float = 240.0, span_z: float = 240.0,
                             step: float | None = None):
        if step is not None:
            self.grid.step = float(step)
        half_xy = float(span_xy) * 0.5
        half_z  = float(span_z)  * 0.5
        bounds = (cx - half_xy, cx + half_xy, cy - half_xy, cy + half_xy, cz - half_z, cz + half_z)
        self.build_init_scene(bounds=bounds, grid_step=self.grid.step)

    def center_grid_on_mesh(self, mesh: pv.PolyData, *,
                            on_contact_plane: bool = True,
                            margin_xy: float = 20.0,
                            margin_z_top: float = 60.0,
                            margin_z_bottom: float = 20.0,
                            min_span_xy: float = 200.0,
                            min_span_z: float = 200.0,
                            step: float | None = None):
        if not hasattr(mesh, "bounds"):
            return
        xmin, xmax, ymin, ymax, zmin, zmax = mesh.bounds
        width  = (xmax - xmin)
        height = (ymax - ymin)
        depth  = (zmax - zmin)

        span_xy = max(width, height) + 2.0 * margin_xy
        span_xy = max(span_xy, float(min_span_xy))

        if on_contact_plane:
            cz = zmin
            span_z = depth + margin_z_top + margin_z_bottom
        else:
            cz = 0.5 * (zmin + zmax)
            span_z = depth + margin_z_top + margin_z_bottom

        span_z = max(span_z, float(min_span_z))

        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)

        self.center_grid_on_point(cx, cy, cz, span_xy=span_xy, span_z=span_z, step=step)

    # Views
    def view_isometric(self): self.views.view_isometric()
    def view_top(self):       self.views.view_top()
    def view_front(self):     self.views.view_front()
    def view_left(self):      self.views.view_left()
    def view_right(self):     self.views.view_right()
    def view_back(self):      self.views.view_back()

    # Render
    def render(self, *, reset_camera: bool = True) -> None:
        ia = self._hoster.ia if self._hoster.ia is not None else (self._hoster.ensure() and self._hoster.ia)
        if ia is None:
            return
        try:
            if reset_camera:
                b = getattr(self.grid, "bounds", None)
                # Nur reset mit gültigen Bounds – sonst generischer Reset (verhindert leere Ansicht)
                if b is not None and np.all(np.isfinite(b)):
                    ia.reset_camera(bounds=b)
                else:
                    ia.reset_camera()
            ia.render()
        except Exception:
            _LOG.exception("render() failed")
