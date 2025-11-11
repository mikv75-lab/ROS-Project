# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Tuple
import logging

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QGridLayout, QLabel, QPushButton, QSizePolicy
)

_LOG = logging.getLogger("app.tabs.recipe.preview.views")


class ViewsGroupBox(QGroupBox):
    """
    Kamera/Views-Steuerung (3D + 2D).

      Kamera: [Iso] [Top] [Front] [Back] [Left] [Right]
      2D:           [Top] [Front] [Back] [Left] [Right]

    Host liefert:
      - activate_3d():  Stack auf 3D schalten
      - switch_2d(plane): "top|front|back|left|right" -> 2D-Seite
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], Tuple[float, float, float, float, float, float]]] = None,
        cam_pad: float = 1.6,
        activate_3d: Callable[[], None],
        switch_2d: Callable[[str], None],
    ):
        super().__init__("Kamera", parent)
        self._activate_3d = activate_3d
        self._switch_2d = switch_2d

        self.views = ViewController(
            interactor_getter=interactor_getter,
            render_callable=render_callable,
            bounds_getter=bounds_getter,
            cam_pad=cam_pad,
        )

        self._build_ui()
        self._wire_buttons()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        lay = QGridLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(6)

        row = 0
        lay.addWidget(self._bold("Kamera"), row, 0, alignment=Qt.AlignmentFlag.AlignRight)
        self.btnCamIso   = QPushButton("Iso", self)
        self.btnCamTop   = QPushButton("Top", self)
        self.btnCamFront = QPushButton("Front", self)
        self.btnCamBack  = QPushButton("Back", self)
        self.btnCamLeft  = QPushButton("Left", self)
        self.btnCamRight = QPushButton("Right", self)
        col = 1
        for b in (self.btnCamIso, self.btnCamTop, self.btnCamFront, self.btnCamBack, self.btnCamLeft, self.btnCamRight):
            lay.addWidget(b, row, col); col += 1

        row += 1
        lay.addWidget(self._bold("2D"), row, 0, alignment=Qt.AlignmentFlag.AlignRight)
        self.btn2DTop   = QPushButton("Top", self)
        self.btn2DFront = QPushButton("Front", self)
        self.btn2DBack  = QPushButton("Back", self)
        self.btn2DLeft  = QPushButton("Left", self)
        self.btn2DRight = QPushButton("Right", self)
        col = 2
        for b in (self.btn2DTop, self.btn2DFront, self.btn2DBack, self.btn2DLeft, self.btn2DRight):
            lay.addWidget(b, row, col); col += 1

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _wire_buttons(self) -> None:
        def on_3d():
            try:
                self._activate_3d()
            except Exception:
                pass

        self.btnCamIso.clicked.connect(  lambda: (on_3d(), self.views.view_isometric()))
        self.btnCamTop.clicked.connect(  lambda: (on_3d(), self.views.view_top()))
        self.btnCamFront.clicked.connect(lambda: (on_3d(), self.views.view_front()))
        self.btnCamBack.clicked.connect( lambda: (on_3d(), self.views.view_back()))
        self.btnCamLeft.clicked.connect( lambda: (on_3d(), self.views.view_left()))
        self.btnCamRight.clicked.connect(lambda: (on_3d(), self.views.view_right()))

        self.btn2DTop.clicked.connect(  lambda: self._switch_2d("top"))
        self.btn2DFront.clicked.connect(lambda: self._switch_2d("front"))
        self.btn2DBack.clicked.connect( lambda: self._switch_2d("back"))
        self.btn2DLeft.clicked.connect( lambda: self._switch_2d("left"))
        self.btn2DRight.clicked.connect(lambda: self._switch_2d("right"))

    def _bold(self, txt: str) -> QLabel:
        lab = QLabel(txt, self)
        lab.setStyleSheet("font-weight:600;")
        return lab


class ViewController:
    """Kamera-Views + Low-level Kamera-Calls mit robustem Camera-Snap."""

    def __init__(
        self,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], Tuple[float, float, float, float, float, float]]] = None,
        cam_pad: float = 1.6,
    ):
        self._get_ia = interactor_getter
        self._render_after = render_callable
        self._get_bounds = bounds_getter
        self._cam_pad = float(cam_pad)

    def _camera_snap(self, *, reset_cam: bool = False):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            if hasattr(ia, "reset_camera_clipping_range"):
                ia.reset_camera_clipping_range()
            if reset_cam and self._get_bounds is not None:
                b = self._get_bounds()
                if b is not None:
                    try:
                        ia.reset_camera(bounds=b)
                    except Exception:
                        ia.reset_camera()
            ia.render()
        except Exception:
            _LOG.exception("_camera_snap failed")

    def view_isometric(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_isometric()
        except Exception:
            _LOG.exception("view_isometric failed")
        self._camera_snap(reset_cam=False)

    def view_top(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xy()
        except Exception:
            _LOG.exception("view_top failed")
        self._camera_snap(reset_cam=False)

    def view_front(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_yz()
        except Exception:
            _LOG.exception("view_front failed")
        self._camera_snap(reset_cam=False)

    def view_left(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xz()
        except Exception:
            _LOG.exception("view_left failed")
        self._camera_snap(reset_cam=False)

    def view_right(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_xz()
            try:
                ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            _LOG.exception("view_right failed")
        self._camera_snap(reset_cam=False)

    def view_back(self):
        ia = self._get_ia()
        if ia is None:
            return
        try:
            ia.view_yz()
            try:
                ia.camera.azimuth(180)
            except Exception:
                pass
        except Exception:
            _LOG.exception("view_back failed")
        self._camera_snap(reset_cam=False)
