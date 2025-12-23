# -*- coding: utf-8 -*-
# File: tabs/service/tool_box.py
from __future__ import annotations
from typing import Optional, List, Any
import json

from PyQt6 import QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QGridLayout, QLabel, QComboBox, QPushButton,
)


class ToolGroupBox(QGroupBox):
    """
    Tool-Auswahl.

    Bridge erwartet (ToolBridge.signals) mindestens:
      Inbound (ROS -> UI):
        - toolListChanged(list[str])           (bevorzugt)
        - optional: toolListStrChanged(str)    (JSON/CSV/…)
        - currentToolChanged(str)
      Outbound (UI -> ROS):
        - selectToolRequested(str)
    """

    # Outbound (Widget -> außen/Bridge)
    selectToolRequested = QtCore.pyqtSignal(str)

    def __init__(self, ros=None, parent: Optional[QWidget] = None, title: str = "Tool"):
        super().__init__(title, parent)
        self.ros = ros
        self._build_ui()
        self._wire_bridge_inbound()
        self._wire_outbound()

    # ------------------------------------------------------------------ Bridge lookup
    def _find_tool_bridge(self):
        """
        Findet den Tool-Bridge/Node auf der neuen RosBridge.

        Erwartet (bevorzugt):
          - ros.tool (Node) mit .signals
        Optional (legacy):
          - ros.tool_bridge
          - ros._tb
        """
        if self.ros is None:
            return None

        # optional ensure_connected (falls jemand es anbietet)
        ensure = getattr(self.ros, "ensure_connected", None)
        if callable(ensure):
            try:
                ensure()
            except Exception:
                pass

        # 1) neue API
        for attr in ("tool", "tool_bridge"):
            if hasattr(self.ros, attr):
                try:
                    b = getattr(self.ros, attr)
                    if b is not None and getattr(b, "signals", None) is not None:
                        return b
                except Exception:
                    pass

        # 2) fallback
        b = getattr(self.ros, "_tb", None)
        if b is not None and getattr(b, "signals", None) is not None:
            return b

        return None


    # ------------------------------------------------------------------ UI
    def _build_ui(self) -> None:
        g = QGridLayout(self)

        def _bold(txt: str) -> QLabel:
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            return l

        # Header – wie bei Scene
        g.addWidget(_bold("Type"),    0, 0)
        g.addWidget(_bold("Select"),  0, 1)
        g.addWidget(_bold("Set"),     0, 2)
        g.addWidget(_bold("Current"), 0, 3)
        g.addWidget(_bold("Clear"),   0, 4)

        # Einzige Datenzeile: Tool
        g.addWidget(QLabel("Tool", self), 1, 0)

        self.cmbTool = QComboBox(self)
        self.cmbTool.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        g.addWidget(self.cmbTool, 1, 1)

        self.btnSet = QPushButton("Set", self)
        g.addWidget(self.btnSet, 1, 2)

        self.lblCurrent = QLabel("-", self)
        self.lblCurrent.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        g.addWidget(self.lblCurrent, 1, 3)

        self.btnClear = QPushButton("Clear", self)
        g.addWidget(self.btnClear, 1, 4)

        # Buttons -> Widget-Signal
        self.btnSet.clicked.connect(self._on_set_clicked)
        self.btnClear.clicked.connect(lambda: self.selectToolRequested.emit(""))

    # ------------------------------------------------------------------ Bridge INBOUND (ROS -> UI)
    def _wire_bridge_inbound(self) -> None:
        tb = self._find_tool_bridge()
        sig = getattr(tb, "signals", None) if tb else None
        if not sig:
            return

        if hasattr(sig, "toolListChanged"):
            sig.toolListChanged.connect(self.set_tool_list)  # list[str]

        # optional: Roh-String
        for cand in ("toolListStrChanged", "toolListUpdated", "toolListMsg"):
            if hasattr(sig, cand):
                getattr(sig, cand).connect(self.set_tool_list_from_string)

        if hasattr(sig, "currentToolChanged"):
            sig.currentToolChanged.connect(self.set_current_tool)

    # ------------------------------------------------------------------ Outbound (UI -> Bridge)
    def _wire_outbound(self) -> None:
        tb = self._find_tool_bridge()
        sig = getattr(tb, "signals", None) if tb else None
        if not sig:
            return

        # Widget-Signal -> Bridge-Signal
        if hasattr(sig, "selectToolRequested"):
            self.selectToolRequested.connect(sig.selectToolRequested.emit)

    # ------------------------------------------------------------------ Slots / Handlers
    def _on_set_clicked(self) -> None:
        name = self.cmbTool.currentText().strip()
        self.selectToolRequested.emit(name)

    # ------------------------------------------------------------------ Tool-List Parsing
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
        uniq: List[str] = []
        for x in items:
            if x and x not in seen:
                seen.add(x)
                uniq.append(x)

        self.set_tool_list(uniq)

    # ------------------------------------------------------------------ Public API
    def set_tool_list(self, items: List[str]) -> None:
        cur = self.cmbTool.currentText().strip()
        self.cmbTool.blockSignals(True)
        try:
            self.cmbTool.clear()
            self.cmbTool.addItems(list(items or []))
            # best-effort: current selection beibehalten
            if cur:
                ix = self.cmbTool.findText(cur)
                if ix >= 0:
                    self.cmbTool.setCurrentIndex(ix)
        finally:
            self.cmbTool.blockSignals(False)

    def set_current_tool(self, name: str) -> None:
        name = (name or "").strip()
        self.lblCurrent.setText(name if name else "-")
        if name:
            ix = self.cmbTool.findText(name)
            if ix >= 0:
                self.cmbTool.blockSignals(True)
                try:
                    self.cmbTool.setCurrentIndex(ix)
                finally:
                    self.cmbTool.blockSignals(False)

    def get_selected_tool(self) -> str:
        return self.cmbTool.currentText().strip()

    def clear_selection(self) -> None:
        self.cmbTool.blockSignals(True)
        try:
            self.cmbTool.setCurrentIndex(-1)
        finally:
            self.cmbTool.blockSignals(False)
        self.lblCurrent.setText("-")
