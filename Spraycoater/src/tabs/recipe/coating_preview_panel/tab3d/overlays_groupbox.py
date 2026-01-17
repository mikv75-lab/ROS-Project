# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/overlays_groupbox.py
from __future__ import annotations

from typing import Any, Dict, Optional

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QCheckBox, QGroupBox, QHBoxLayout, QLabel, QSizePolicy, QWidget


class OverlaysGroupBox(QGroupBox):
    """UI-only overlay toggles.

    IMPORTANT: This widget intentionally contains *no* rendering logic.

    get_config() returns a stable dict consumed by Tab3D.update_preview():
      {
        'mask': bool,
        'path': bool,
        'hits': bool,
        'misses': bool,
        'normals': bool,
        'tcp': bool,
      }
    """

    sig_changed = pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__("Overlays", parent)
        self._build_ui()
        self._wire()

    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add_toggle(key: str, text: str, checked: bool) -> QCheckBox:
            lab = QLabel(text, self)
            lab.setStyleSheet("font-weight:600;")
            chk = QCheckBox(self)
            chk.setChecked(bool(checked))
            lay.addWidget(lab)
            lay.addWidget(chk)
            chk.setProperty("overlay_key", key)
            return chk

        # Defaults: what you typically want to see during recipe authoring
        self.chk_mask = add_toggle("mask", "Mask", True)
        self.chk_path = add_toggle("path", "Path", True)
        self.chk_tcp = add_toggle("tcp", "TCP", True)

        # Debug layers default off
        self.chk_hits = add_toggle("hits", "Hits", False)
        self.chk_misses = add_toggle("misses", "Misses", False)
        self.chk_normals = add_toggle("normals", "Normals", False)

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _wire(self) -> None:
        for chk in (self.chk_mask, self.chk_path, self.chk_tcp, self.chk_hits, self.chk_misses, self.chk_normals):
            chk.toggled.connect(self.sig_changed.emit)

    def get_config(self) -> Dict[str, Any]:
        return {
            "mask": bool(self.chk_mask.isChecked()),
            "path": bool(self.chk_path.isChecked()),
            "tcp": bool(self.chk_tcp.isChecked()),
            "hits": bool(self.chk_hits.isChecked()),
            "misses": bool(self.chk_misses.isChecked()),
            "normals": bool(self.chk_normals.isChecked()),
        }
