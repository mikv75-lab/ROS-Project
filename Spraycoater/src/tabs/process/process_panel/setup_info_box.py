# -*- coding: utf-8 -*-
# File: widgets/setup_info_box.py
from __future__ import annotations

from typing import Optional
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QLabel, QSizePolicy,
    QVBoxLayout, QHBoxLayout
)


class SetupInfoBox(QGroupBox):
    """
    Kompakte Anzeige für das aktive Hardware-Setup (3 Zeilen).
    """

    # UI-Defaults (wenn ROS/Node nicht liefert)
    DEFAULT_TOOL: str = "spray_nozzle_01"

    def __init__(self, parent: Optional[QGroupBox] = None, title: str = "Active Setup"):
        super().__init__(title, parent)
        self._build_ui()
        self._apply_policies()

        # Apply UI defaults (may be overwritten by ROS signals later)
        self.set_tool(self.DEFAULT_TOOL)

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        def row(label_text: str, *, default_value: str = "-") -> QLabel:
            hb = QHBoxLayout()
            bold = QLabel(label_text, self)
            bold.setStyleSheet("font-weight:600;")
            bold.setMinimumWidth(100)

            val = QLabel(default_value, self)
            val.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
            val.setStyleSheet("color: #005cc5;")  # Blau für aktive ROS-Werte

            hb.addWidget(bold)
            hb.addWidget(val, 1)
            root.addLayout(hb)
            return val

        # Defaults: Tool = spray_nozzle_01, Rest unbekannt
        self.lblTool      = row("Tool", default_value=self.DEFAULT_TOOL)
        self.lblSubstrate = row("Substrate", default_value="-")
        self.lblMount     = row("Substrate Mount", default_value="-")

        root.addStretch(1)

    def _apply_policies(self) -> None:
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    # ---------- Public Setters ----------
    def set_tool(self, name: str) -> None:
        self.lblTool.setText(str(name).strip() if name and str(name).strip() else "-")

    def set_substrate(self, name: str) -> None:
        self.lblSubstrate.setText(str(name).strip() if name and str(name).strip() else "-")

    def set_mount(self, name: str) -> None:
        self.lblMount.setText(str(name).strip() if name and str(name).strip() else "-")
