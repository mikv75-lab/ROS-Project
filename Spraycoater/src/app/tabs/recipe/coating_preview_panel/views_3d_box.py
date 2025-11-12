# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Tuple

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QFormLayout, QHBoxLayout, QPushButton,
    QSizePolicy, QLabel
)

from .views_3d.view_controller_3d import ViewController3D

Bounds = Tuple[float, float, float, float, float, float]


def _set_policy(w: QWidget,
                *,
                h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
                v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


class Views3DBox(QGroupBox):
    """
    3D-Controls (PyVista): Iso / Top / Front / Back / Left / Right
    Nutzt ViewController3D intern. Keine Panel-Logik.
    """

    def __init__(
        self,
        *,
        interactor_getter: Callable[[], object],
        render_callable: Callable[..., None],
        bounds_getter: Optional[Callable[[], Bounds]] = None,
        substrate_bounds_getter: Optional[Callable[[], Optional[Bounds]]] = None,
        cam_pad: float = 1.6,
        iso_extra_zoom: float = 1.30,
        parent: Optional[QWidget] = None,
    ):
        super().__init__("3D View", parent)

        self.views = ViewController3D(
            interactor_getter=interactor_getter,
            render_callable=render_callable,
            bounds_getter=bounds_getter,
            cam_pad=cam_pad,
            substrate_bounds_getter=substrate_bounds_getter,
            zoom_after_reset=1.12,      # Basis-Zoom nach Reset
            iso_extra_zoom=iso_extra_zoom,
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

        self.btnIso   = QPushButton("Iso", self)
        self.btnTop   = QPushButton("Top", self)
        self.btnFront = QPushButton("Front", self)
        self.btnBack  = QPushButton("Back", self)
        self.btnLeft  = QPushButton("Left", self)
        self.btnRight = QPushButton("Right", self)

        for b in (self.btnIso, self.btnTop, self.btnFront, self.btnBack, self.btnLeft, self.btnRight):
            b.setAutoDefault(False)
            lay.addWidget(b)

        form.addRow(QLabel("Camera", self), row)

        # Wiring
        self.btnIso.clicked.connect(  self.views.view_isometric)
        self.btnTop.clicked.connect(  self.views.view_top)
        self.btnFront.clicked.connect(self.views.view_front)
        self.btnBack.clicked.connect( self.views.view_back)
        self.btnLeft.clicked.connect( self.views.view_left)
        self.btnRight.clicked.connect(self.views.view_right)

    # optional convenience setter, falls du den Substrat-Getter später erst setzen willst
    def set_substrate_bounds_getter(self, fn: Optional[Callable[[], Optional[Bounds]]]) -> None:
        try:
            self.views._get_sub_bounds = fn  # bewusst einfach gehalten
        except Exception:
            pass
