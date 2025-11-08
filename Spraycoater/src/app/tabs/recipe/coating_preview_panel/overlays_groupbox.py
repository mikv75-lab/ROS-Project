# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QHBoxLayout, QCheckBox, QLabel, QSizePolicy
)


class OverlaysGroupBox(QGroupBox):
    """
    Overlays – alles in EINER horizontalen Zeile:
      Mask [ ]   Path [x]   Hits [ ]   Misses [ ]   Normals [ ]   Lokale KS [ ]
    """

    maskToggled = pyqtSignal(bool)
    pathToggled = pyqtSignal(bool)
    hitsToggled = pyqtSignal(bool)
    missesToggled = pyqtSignal(bool)
    normalsToggled = pyqtSignal(bool)
    localFramesToggled = pyqtSignal(bool)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Overlays", parent)
        self._build_ui()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add(lbl: str, checked: bool = False) -> QCheckBox:
            lab = QLabel(lbl, self)
            lab.setStyleSheet("font-weight:600;")
            chk = QCheckBox(self)
            chk.setChecked(checked)
            lay.addWidget(lab)
            lay.addWidget(chk)
            return chk

        self.chkShowMask        = add("Mask", False)
        self.chkShowPath        = add("Path", True)
        self.chkShowHits        = add("Hits", False)
        self.chkShowMisses      = add("Misses", False)
        self.chkShowNormals     = add("Normals", False)
        self.chkShowLocalFrames = add("Lokale KS", False)

        lay.addStretch(1)

        # size policy
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

        # wiring
        self.chkShowMask.toggled.connect(self.maskToggled)
        self.chkShowPath.toggled.connect(self.pathToggled)
        self.chkShowHits.toggled.connect(self.hitsToggled)
        self.chkShowMisses.toggled.connect(self.missesToggled)
        self.chkShowNormals.toggled.connect(self.normalsToggled)
        self.chkShowLocalFrames.toggled.connect(self.localFramesToggled)

    # ---------- Convenience ----------
    def set_defaults(
        self, *, mask=False, path=True, hits=False, misses=False, normals=False, local_frames=False
    ) -> None:
        self.chkShowMask.setChecked(bool(mask))
        self.chkShowPath.setChecked(bool(path))
        self.chkShowHits.setChecked(bool(hits))
        self.chkShowMisses.setChecked(bool(misses))
        self.chkShowNormals.setChecked(bool(normals))
        self.chkShowLocalFrames.setChecked(bool(local_frames))
