from __future__ import annotations
import os
import logging
from typing import Optional, Callable

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame

_LOG = logging.getLogger("app.tabs.recipe.preview")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    # .../src/app/tabs/recipe/coating_preview_panel/ → bis Projektwurzel
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    # erwartet: resource/ui/tabs/recipe/<filename>
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class CoatingPreviewPanel(QWidget):
    """
    Lädt die UI aus .ui, stellt Controls + einen Host (previewHost) bereit.
    Der QtInteractor wird im MainWindow erzeugt und in previewHost eingehängt.
    """

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        uic.loadUi(_ui_path("coating_preview_panel.ui"), self)

        # Controls aus der UI
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

        # Host für QtInteractor
        self._host: QFrame = self.findChild(QFrame, "previewHost")
        if self._host is None:
            raise RuntimeError("coating_preview_panel.ui muss ein QFrame namens 'previewHost' enthalten.")

        # Kamera-Hooks (werden vom Tab gesetzt)
        self._hooks: dict[str, Optional[Callable[[], None]]] = {
            "iso": None, "top": None, "front": None, "left": None, "right": None, "back": None, "after": None
        }
        self._wire_buttons()

    # ---- API für RecipeTab / MainWindow ----
    def preview_host(self) -> QFrame:
        """Host-Widget, in das MainWindow den QtInteractor einhängt."""
        return self._host

    def set_camera_hooks(
        self,
        *,
        on_iso: Optional[Callable[[], None]] = None,
        on_top: Optional[Callable[[], None]] = None,
        on_front: Optional[Callable[[], None]] = None,
        on_left: Optional[Callable[[], None]] = None,
        on_right: Optional[Callable[[], None]] = None,
        on_back: Optional[Callable[[], None]] = None,
        on_after: Optional[Callable[[], None]] = None,
    ) -> None:
        self._hooks.update({
            "iso": on_iso, "top": on_top, "front": on_front,
            "left": on_left, "right": on_right, "back": on_back,
            "after": on_after,
        })

    # ---- intern ----
    def _wire_buttons(self):
        if self.btnCamIso:   self.btnCamIso.clicked.connect(lambda: self._call_cam("iso"))
        if self.btnCamTop:   self.btnCamTop.clicked.connect(lambda: self._call_cam("top"))
        if self.btnCamFront: self.btnCamFront.clicked.connect(lambda: self._call_cam("front"))
        if self.btnCamLeft:  self.btnCamLeft.clicked.connect(lambda: self._call_cam("left"))
        if self.btnCamRight: self.btnCamRight.clicked.connect(lambda: self._call_cam("right"))
        if self.btnCamBack:  self.btnCamBack.clicked.connect(lambda: self._call_cam("back"))

    def _call_cam(self, key: str):
        fn = self._hooks.get(key)
        if callable(fn):
            try:
                fn()
            except Exception:
                _LOG.exception("Kamera-Button '%s' fehlgeschlagen", key)
        after = self._hooks.get("after")
        if callable(after):
            try:
                after()
            except Exception:
                _LOG.exception("Kamera after-hook fehlgeschlagen")
