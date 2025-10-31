# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional, Callable, Tuple, Any

from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QPushButton, QCheckBox, QFrame, QVBoxLayout

import pyvista as pv

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

    NEU: Bringt eine Preview-API mit, die einen EXTERN ERZEUGTEN QtInteractor
         (aus dem MainWindow) nutzen kann:
         - attach_interactor(interactor)
         - build_init_scene(bounds, grid_step)
         - clear(), add_mesh(), view_*(), render()

    -> Der Interactor wird NICHT hier instanziert. Er lebt im MainWindow.
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

        # Der spätere (externe) QtInteractor
        self._ia: Optional[Any] = None  # pyvistaqt.QtInteractor

        # Letzte Grid-/Bounds-Configs
        self._bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self._grid_step: float = 10.0

        # Kamera-Hooks (können vom Tab/MainWindow überschrieben werden)
        # Default: auf interne Methoden mappen (falls niemand überschreibt)
        self._hooks: dict[str, Optional[Callable[[], None]]] = {
            "iso": self.view_isometric,
            "top": self.view_top,
            "front": self.view_front,
            "left": self.view_left,
            "right": self.view_right,
            "back": self.view_back,
            "after": lambda: self.render(reset_camera=False),
        }
        self._wire_buttons()

    # -------------------------------------------------------------------------
    # Public API – Hosting
    # -------------------------------------------------------------------------
    def preview_host(self) -> QFrame:
        """Host-Widget, in das MainWindow den QtInteractor einhängt (per setParent/addWidget)."""
        return self._host

    def attach_interactor(self, interactor: Any) -> None:
        """
        Interactor-Instanz (aus MainWindow) hier "andocken":
         - Parent umhängen
         - in unser Host-Layout einfügen
         - sichtbar/renderbar machen
        """
        if interactor is None:
            raise ValueError("attach_interactor: interactor is None")

        try:
            self._ia = interactor
            ly = self._host.layout()
            if ly is None:
                ly = QVBoxLayout(self._host)
                ly.setContentsMargins(0, 0, 0, 0)
                ly.setSpacing(0)

            self._ia.setParent(self._host)
            try:
                ly.addWidget(self._ia)
            except Exception:
                pass  # Duplikat-Hinzufügen ignorieren

            self._ia.setEnabled(True)
            self._ia.show()
            self._ia.update()
            if hasattr(self._ia, "render"):
                self._ia.render()
        except Exception:
            _LOG.exception("attach_interactor() failed")

    # -------------------------------------------------------------------------
    # Public API – Kamera-Hooks (optional überschreibbar von außen)
    # -------------------------------------------------------------------------
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
        """
        Überschreibe optional die Kamera-Callbacks.
        Fallback bleibt die interne View-API dieser Klasse.
        """
        if on_iso   is not None: self._hooks["iso"]   = on_iso
        if on_top   is not None: self._hooks["top"]   = on_top
        if on_front is not None: self._hooks["front"] = on_front
        if on_left  is not None: self._hooks["left"]  = on_left
        if on_right is not None: self._hooks["right"] = on_right
        if on_back  is not None: self._hooks["back"]  = on_back
        if on_after is not None: self._hooks["after"] = on_after

    # -------------------------------------------------------------------------
    # Public API – Szene / Grid
    # -------------------------------------------------------------------------
    def build_init_scene(
        self,
        bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240),
        grid_step: float = 10.0,
    ) -> None:
        """
        Leere Szene + Gitternetz + Achsenbeschriftung, ohne Box-Kanten.
        Nutzt den angehängten Interactor (_ia).
        """
        if self._ia is None:
            _LOG.warning("build_init_scene(): kein Interactor angehängt")
            return

        self._bounds = bounds
        self._grid_step = grid_step

        p = self._ia
        p.clear()

        axes = p.show_grid(
            bounds=bounds,
            xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
            show_xaxis=True, show_yaxis=True, show_zaxis=True,
            show_xlabels=True, show_ylabels=True, show_zlabels=True,
            n_xlabels=max(2, int((bounds[1] - bounds[0]) // (2 * grid_step))),
            n_ylabels=max(2, int((bounds[3] - bounds[2]) // (2 * grid_step))),
            n_zlabels=max(2, int((bounds[5] - bounds[4]) // (grid_step * 4))),
            ticks='both',
            grid='back',
            render=False,
        )
        try:
            axes.SetShowEdges(False)
            axes.SetDrawXGridlines(True); axes.SetDrawYGridlines(True); axes.SetDrawZGridlines(True)
            axes.SetDrawXInnerGridlines(True); axes.SetDrawYInnerGridlines(True); axes.SetDrawZInnerGridlines(True)
            axes.SetUseTextActor3D(1)
        except Exception:
            pass

        try:
            p.view_isometric()
            p.reset_camera(bounds=bounds)
        except Exception:
            pass

        p.render()

    def clear(self) -> None:
        """Szene leeren und Grid neu aufbauen."""
        if self._ia is None:
            return
        try:
            self._ia.clear()
            self.build_init_scene(self._bounds, self._grid_step)
        except Exception:
            _LOG.exception("clear() failed")

    def add_mesh(self, mesh, **kwargs) -> None:
        """Mesh hinzufügen (robust triangulieren, nicht sofort rendern)."""
        if self._ia is None:
            _LOG.warning("add_mesh(): kein Interactor angehängt")
            return
        try:
            fn = getattr(mesh, "is_all_triangles", None)
            if callable(fn) and not fn():
                mesh = mesh.triangulate()
        except Exception:
            pass
        try:
            self._ia.add_mesh(mesh, reset_camera=False, render=False, **kwargs)
        except Exception:
            _LOG.exception("add_mesh() failed")

    # -------------------------------------------------------------------------
    # Public API – Views
    # -------------------------------------------------------------------------
    def view_isometric(self) -> None:
        if self._ia is None:
            return
        try:
            self._ia.view_isometric()
        except Exception:
            pass

    def view_top(self) -> None:
        if self._ia is None:
            return
        try:
            self._ia.view_xy()
        except Exception:
            pass

    def view_front(self) -> None:
        if self._ia is None:
            return
        try:
            self._ia.view_yz()
        except Exception:
            pass

    def view_left(self) -> None:
        if self._ia is None:
            return
        try:
            self._ia.view_xz()
        except Exception:
            pass

    def view_right(self) -> None:
        if self._ia is None:
            return
        try:
            self._ia.view_xz()
            try:
                self._ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            pass

    def view_back(self) -> None:
        if self._ia is None:
            return
        try:
            self._ia.view_yz()
            try:
                self._ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            pass

    # -------------------------------------------------------------------------
    # Public API – Render
    # -------------------------------------------------------------------------
    def render(self, *, reset_camera: bool = True) -> None:
        if self._ia is None:
            return
        try:
            if reset_camera:
                self._ia.reset_camera(bounds=self._bounds)
            self._ia.render()
        except Exception:
            _LOG.exception("render() failed")

    # -------------------------------------------------------------------------
    # intern
    # -------------------------------------------------------------------------
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
