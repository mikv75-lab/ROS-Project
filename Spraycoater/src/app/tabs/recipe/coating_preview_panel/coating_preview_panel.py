# -*- coding: utf-8 -*-
from __future__ import annotations
import os, logging
from typing import Optional, Tuple, Any

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame

from .interactor_host import InteractorHost
from .grid_manager import GridManager
from .scene_manager import SceneManager
from .views import ViewController

_LOG = logging.getLogger("app.tabs.recipe.preview")

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

    # --- Public API (vom RecipeTab verwendet) ---
    def build_init_scene_mainstyle(self, grid_step: float = 10.0):
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("build_init_scene_mainstyle(): kein Interactor")
            return
        self.grid.step = grid_step
        self.grid.build_init_scene_mainstyle()

    def build_init_scene(self, bounds=None, grid_step: float = 10.0):
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("build_init_scene(): kein Interactor")
            return
        if bounds is not None:
            self.grid.bounds = bounds
        self.grid.step = grid_step
        self.grid.build_init_scene()

    def clear(self) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            return
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        if self._hoster.ia is None and not self._hoster.ensure():
            _LOG.warning("add_mesh(): kein Interactor")
            return
        self.scene.add_mesh(mesh, **kwargs)

    # Layer-Shortcuts
    def clear_layer(self, layer: str) -> None: self.scene.clear_layer(layer)

    # Pfad-Shortcuts
    def add_path_polyline(self, *a, **kw): self.scene.add_path_polyline(*a, **kw)
    def add_path_markers(self, *a, **kw): self.scene.add_path_markers(*a, **kw)

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
                ia.reset_camera(bounds=self.grid.bounds)
            ia.render()
        except Exception:
            _LOG.exception("render() failed")
