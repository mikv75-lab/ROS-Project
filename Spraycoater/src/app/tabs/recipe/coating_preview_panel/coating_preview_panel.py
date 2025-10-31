from __future__ import annotations
import os
import logging
from typing import Optional

from PyQt6 import uic
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QPushButton

_LOG = logging.getLogger("app.tabs.recipe.preview")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    # .../src/app/tabs/recipe/coating_preview_panel/ -> bis Projektwurzel
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class CoatingPreviewPanel(QWidget):
    """
    Reines UI-Panel:
      - extrahiert Keys aus dem Model
      - emittiert renderRequested(mount_key, substrate_key)
      - emittiert Kamera-Signale bei Button-Klicks
    """

    # ---- Signale zu MainWindow ----
    renderRequested   = pyqtSignal(str, str)  # (mount_key, substrate_key)
    cameraIsoRequested   = pyqtSignal()
    cameraTopRequested   = pyqtSignal()
    cameraFrontRequested = pyqtSignal()
    cameraLeftRequested  = pyqtSignal()
    cameraRightRequested = pyqtSignal()
    cameraBackRequested  = pyqtSignal()

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # UI laden (ohne eingebetteten QtInteractor!)
        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # Kamera-Buttons → Signale
        self._wire_camera_buttons()

    # Wird vom RecipeTab bei Model-Änderung aufgerufen
    def render_from_model(self, model: object) -> None:
        mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt 'substrate_mount'.")
        substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt 'substrate'.")
        _LOG.info("Preview: emit renderRequested(%s, %s)", mount_key, substrate_key)
        self.renderRequested.emit(mount_key, substrate_key)

    # ---- Helpers ----
    @staticmethod
    def _get_required_str(model: object, key: str, err: str) -> str:
        if hasattr(model, key):
            val = getattr(model, key)
        elif isinstance(model, dict):
            val = model.get(key)
        else:
            val = None
        if not isinstance(val, str) or not val.strip():
            raise ValueError(err)
        return val.strip()

    def _btn(self, name: str) -> Optional[QPushButton]:
        return getattr(self, name, None)

    def _wire_camera_buttons(self):
        if self._btn("btnCamIso"):   self._btn("btnCamIso").clicked.connect(self.cameraIsoRequested)
        if self._btn("btnCamTop"):   self._btn("btnCamTop").clicked.connect(self.cameraTopRequested)
        if self._btn("btnCamFront"): self._btn("btnCamFront").clicked.connect(self.cameraFrontRequested)
        if self._btn("btnCamLeft"):  self._btn("btnCamLeft").clicked.connect(self.cameraLeftRequested)
        if self._btn("btnCamRight"): self._btn("btnCamRight").clicked.connect(self.cameraRightRequested)
        if self._btn("btnCamBack"):  self._btn("btnCamBack").clicked.connect(self.cameraBackRequested)
