# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QFormLayout, QHBoxLayout, QPushButton,
    QSizePolicy, QLabel
)

from .views_2d.view_controller_2d import ViewController2D

Bounds = Tuple[float, float, float, float, float, float]

def _set_policy(w: QWidget,
                *,
                h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
                v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)

class Views2DBox(QGroupBox):
    """
    2D-Controls (Matplotlib): Top / Front / Back / Left / Right
    Ruft switch_2d(plane) – zusätzlich steht `controller` für programmatic use bereit.
    """
    def __init__(
        self,
        *,
        switch_2d: Callable[[str], None],
        refresh_callable: Optional[Callable[[], None]] = None,
        get_bounds: Optional[Callable[[], Bounds]] = None,
        set_bounds: Optional[Callable[[Bounds], None]] = None,
        parent: Optional[QWidget] = None
    ):
        super().__init__("2D View", parent)
        self._switch_2d = switch_2d

        # Optionaler Controller (falls refresh/bounds übergeben)
        self.controller = ViewController2D(
            set_plane=self._switch_2d,
            refresh=(refresh_callable or (lambda: None)),
            get_bounds=get_bounds,
            set_bounds=set_bounds,
        )

        form = QFormLayout(self)
        form.setContentsMargins(8, 8, 8, 8)
        form.setHorizontalSpacing(8)
        form.setVerticalSpacing(4)
        form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFormAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.ExpandingFieldsGrow)

        row = QWidget(self)
        lay = QHBoxLayout(row)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self.btnTop   = QPushButton("Top", self)
        self.btnFront = QPushButton("Front", self)
        self.btnBack  = QPushButton("Back", self)
        self.btnLeft  = QPushButton("Left", self)
        self.btnRight = QPushButton("Right", self)

        for b in (self.btnTop, self.btnFront, self.btnBack, self.btnLeft, self.btnRight):
            b.setAutoDefault(False)
            lay.addWidget(b)

        form.addRow(QLabel("Plane", self), row)

        # Wiring – wenn Controller existiert, nutze ihn; sonst direkt switch_2d
        if refresh_callable:
            self.btnTop.clicked.connect(  self.controller.plane_top)
            self.btnFront.clicked.connect(self.controller.plane_front)
            self.btnBack.clicked.connect( self.controller.plane_back)
            self.btnLeft.clicked.connect( self.controller.plane_left)
            self.btnRight.clicked.connect(self.controller.plane_right)
        else:
            self.btnTop.clicked.connect(  lambda: self._switch_2d("top"))
            self.btnFront.clicked.connect(lambda: self._switch_2d("front"))
            self.btnBack.clicked.connect( lambda: self._switch_2d("back"))
            self.btnLeft.clicked.connect( lambda: self._switch_2d("left"))
            self.btnRight.clicked.connect(lambda: self._switch_2d("right"))
