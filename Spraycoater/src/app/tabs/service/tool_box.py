# -*- coding: utf-8 -*-
# File: tabs/service/tool_box.py
from __future__ import annotations
from typing import Optional, List
import json

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QGridLayout, QLabel, QComboBox, QPushButton,
)

class ToolGroupBox(QGroupBox):
    """
    Tool-Auswahl.

    ROS Topics:
      - subscribe (UI -> ROS): /spraycoater/tool/select (std_msgs/String, Name)
      - publish  (ROS -> UI): /spraycoater/current_tool (std_msgs/String, Name)
      - publish  (ROS -> UI): /spraycoater/tool/list   (std_msgs/String, Liste)

    Bridge erwartet in bridge._tb.signals (mind. eins der Paare):
      Inbound:
        * toolListChanged(list[str])                # bevorzugt
        * oder: toolListStrChanged(str)             # Roh-String (JSON/CSV/…)
        * currentToolChanged(str)
      Outbound:
        * selectToolRequested(str)                  # UI->ROS (Name)
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

        # Wire buttons
        self.btnSet.clicked.connect(self._on_set_clicked)
        self.btnClear.clicked.connect(lambda: self._emit_select_request(""))

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
            sig.toolListChanged.connect(self.set_tool_list)  # list[str]
        # Alternativ: Roh-String (vom /tool/list Publisher)
        for cand in ("toolListStrChanged", "toolListUpdated", "toolListMsg"):
            if hasattr(sig, cand):
                getattr(sig, cand).connect(self.set_tool_list_from_string)
        if hasattr(sig, "currentToolChanged"):
            sig.currentToolChanged.connect(self.set_current_tool)

        # Outbound (UI -> ROS)
        self._selectToolRequested = getattr(sig, "selectToolRequested", None)

    # ---------- Slots / Handlers ----------
    def _on_set_clicked(self) -> None:
        name = self.cmbTool.currentText().strip()
        self._emit_select_request(name)

    def _emit_select_request(self, name: str) -> None:
        if callable(getattr(self, "_selectToolRequested", None)):
            self._selectToolRequested.emit(name)  # type: ignore[attr-defined]

    # ---------- Tool-List Parsing ----------
    def set_tool_list_from_string(self, payload: str) -> None:
        """
        Akzeptiert:
          - JSON-Array:   '["toolA","toolB"]'
          - CSV/SSV:      'toolA, toolB' oder 'toolA;toolB'
          - Zeilen:       'toolA\\ntoolB'
        """
        if payload is None:
            self.set_tool_list([])
            return
        s = str(payload).strip()
        items: List[str] = []
        # JSON?
        if s.startswith("[") and s.endswith("]"):
            try:
                arr = json.loads(s)
                if isinstance(arr, list):
                    items = [str(x).strip() for x in arr if str(x).strip()]
            except Exception:
                items = []
        # Fallback: splitten
        if not items:
            for sep in ("\n", ",", ";", "|"):
                if sep in s:
                    items = [p.strip() for p in s.split(sep)]
                    break
            else:
                items = [s] if s else []

        # deduplizieren, Reihenfolge beibehalten
        seen = set()
        uniq = []
        for x in items:
            if x and x not in seen:
                seen.add(x)
                uniq.append(x)
        self.set_tool_list(uniq)

    # ---------- Public API ----------
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

    def get_selected_tool(self) -> str:
        return self.cmbTool.currentText().strip()

    def clear_selection(self) -> None:
        self.cmbTool.setCurrentIndex(-1)
        self.lblCurrent.setText("-")
