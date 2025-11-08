# -*- coding: utf-8 -*-
# File: tabs/service/tool_box.py
from __future__ import annotations
from typing import Optional, List

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QGridLayout, QLabel, QComboBox, QPushButton,
    QSizePolicy
)


class ToolGroupBox(QGroupBox):
    """
    Tool-Auswahl + Status.

    UI:
      ┌──────────────────────────────────────────────┐
      │ Tool | [ComboBox.................] [Set] [-] │
      │ Curr | <current or ->                          │
      │ State| <state or ->                            │
      └──────────────────────────────────────────────┘

    Bridge:
      - erwartet bridge._tb.signals mit:
          * toolListChanged(list[str])
          * toolCurrentChanged(str)
          * toolStateChanged(str)
          * setToolRequested(str)
    """

    def __init__(self, bridge=None, parent: Optional[QWidget] = None):
        super().__init__("Tool", parent)
        self.bridge = bridge
        self._build_ui()
        self._wire_bridge()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        lay = QGridLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setHorizontalSpacing(8)
        lay.setVerticalSpacing(6)

        # Row 0: Auswahl + Set/Clear
        lay.addWidget(self._bold("Tool"), 0, 0, alignment=Qt.AlignmentFlag.AlignRight)

        self.cmbTool = QComboBox(self)
        self.cmbTool.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        lay.addWidget(self.cmbTool, 0, 1)

        self.btnSet = QPushButton("Set", self)
        self.btnClear = QPushButton("Clear", self)
        lay.addWidget(self.btnSet, 0, 2)
        lay.addWidget(self.btnClear, 0, 3)

        # Row 1: Current
        lay.addWidget(self._bold("Current"), 1, 0, alignment=Qt.AlignmentFlag.AlignRight)
        self.lblCurrent = QLabel("-", self)
        self.lblCurrent.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(self.lblCurrent, 1, 1, 1, 3)

        # Row 2: State
        lay.addWidget(self._bold("State"), 2, 0, alignment=Qt.AlignmentFlag.AlignRight)
        self.lblState = QLabel("-", self)
        self.lblState.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        lay.addWidget(self.lblState, 2, 1, 1, 3)

        # Wire buttons
        self.btnSet.clicked.connect(self._on_set_clicked)
        self.btnClear.clicked.connect(lambda: self._emit_set_request(""))

        # Sizing
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def _bold(self, txt: str) -> QLabel:
        l = QLabel(txt, self)
        l.setStyleSheet("font-weight:600;")
        return l

    # ---------- Bridge Verdrahtung ----------
    def _wire_bridge(self) -> None:
        tb = getattr(self.bridge, "_tb", None) if self.bridge else None
        sig = getattr(tb, "signals", None) if tb else None
        if not sig:
            return

        # Inbound (ROS -> UI)
        if hasattr(sig, "toolListChanged"):
            sig.toolListChanged.connect(self.set_tool_list)
        if hasattr(sig, "toolCurrentChanged"):
            sig.toolCurrentChanged.connect(self.set_current_tool)
        if hasattr(sig, "toolStateChanged"):
            sig.toolStateChanged.connect(self.set_tool_state)

        # Outbound (UI -> ROS)
        self._setToolRequested = getattr(sig, "setToolRequested", None)

    # ---------- Slots / Handlers ----------
    def _on_set_clicked(self) -> None:
        name = self.cmbTool.currentText().strip()
        self._emit_set_request(name)

    def _emit_set_request(self, name: str) -> None:
        # Forward only if bridge signal exists
        if callable(getattr(self, "_setToolRequested", None)):
            self._setToolRequested.emit(name)  # type: ignore[attr-defined]

    # ---------- Public API (für externe Nutzung/Tests) ----------
    def set_tool_list(self, items: List[str]) -> None:
        self.cmbTool.blockSignals(True)
        try:
            self.cmbTool.clear()
            self.cmbTool.addItems(items or [])
        finally:
            self.cmbTool.blockSignals(False)

    def set_current_tool(self, name: str) -> None:
        name = (name or "").strip()
        self.lblCurrent.setText(name if name else "-")
        if name:
            idx = self.cmbTool.findText(name)
            if idx >= 0:
                self.cmbTool.setCurrentIndex(idx)

    def set_tool_state(self, state: str) -> None:
        state = (state or "").strip()
        self.lblState.setText(state if state else "-")

    def get_selected_tool(self) -> str:
        return self.cmbTool.currentText().strip()

    def clear_selection(self) -> None:
        self.cmbTool.setCurrentIndex(-1)
        self.lblCurrent.setText("-")
