# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QGridLayout, QLabel, QPushButton, QSizePolicy
)

from .views import ViewController


class ViewsGroupBox(QGroupBox):
    """
    Kamera/Views-Steuerung (3D + 2D).

      Kamera: [Iso] [Top] [Front] [Back] [Left] [Right]
      2D:           [Top] [Front] [Back] [Left] [Right]

    Diese Klasse erzeugt und verwaltet intern den ViewController und verdrahtet die Buttons.
    Der Host (Panel) liefert nur noch:
      - activate_3d():  Stack auf 3D schalten
      - switch_2d(plane): "top|front|back|left|right" -> 2D-Seite wählen
    """

    def __init__(
        self,
        parent: Optional[QWidget] = None,
        *,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], tuple]] = None,
        cam_pad: float = 1.6,
        activate_3d: Callable[[], None],
        switch_2d: Callable[[str], None],
    ):
        super().__init__("Kamera", parent)
        self._activate_3d = activate_3d
        self._switch_2d = switch_2d

        # ViewController erzeugen
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

        # 3D row (Label "Kamera")
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

        # 2D row – um 1 Spalte nach rechts versetzt, damit "Top" unter "Top" steht
        row += 1
        lay.addWidget(self._bold("2D"), row, 0, alignment=Qt.AlignmentFlag.AlignRight)
        self.btn2DTop   = QPushButton("Top", self)
        self.btn2DFront = QPushButton("Front", self)
        self.btn2DBack  = QPushButton("Back", self)
        self.btn2DLeft  = QPushButton("Left", self)
        self.btn2DRight = QPushButton("Right", self)
        col = 2  # Start bei Spalte 2 (unter "Top")
        for b in (self.btn2DTop, self.btn2DFront, self.btn2DBack, self.btn2DLeft, self.btn2DRight):
            lay.addWidget(b, row, col); col += 1

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _wire_buttons(self) -> None:
        # 3D: zuerst Panel auf 3D umschalten, dann Kamera-View setzen
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

        # 2D: Panel-Seite/Plane wählen
        self.btn2DTop.clicked.connect(  lambda: self._switch_2d("top"))
        self.btn2DFront.clicked.connect(lambda: self._switch_2d("front"))
        self.btn2DBack.clicked.connect( lambda: self._switch_2d("back"))
        self.btn2DLeft.clicked.connect( lambda: self._switch_2d("left"))
        self.btn2DRight.clicked.connect(lambda: self._switch_2d("right"))

    # ---------- Helpers ----------
    def _bold(self, txt: str) -> QLabel:
        lab = QLabel(txt, self)
        lab.setStyleSheet("font-weight:600;")
        return lab
