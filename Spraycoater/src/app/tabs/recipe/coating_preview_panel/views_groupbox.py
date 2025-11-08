# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QGridLayout, QLabel, QPushButton, QSizePolicy
)


class ViewsGroupBox(QGroupBox):
    """
    Kamera/Views-Steuerung (3D + 2D).

      Kamera: [Iso] [Top] [Front] [Back] [Left] [Right]
      2D:           [Top] [Front] [Back] [Left] [Right]   (eine Spalte nach rechts eingerückt)
    """

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Kamera", parent)  # Gruppen-Titel: "Kamera"
        self._build_ui()

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

        # size policy
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _bold(self, txt: str) -> QLabel:
        lab = QLabel(txt, self)
        lab.setStyleSheet("font-weight:600;")
        return lab
