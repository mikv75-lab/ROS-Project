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

    # ---- UI-Fallback defaults (wenn Node nicht läuft) ----
    _DEFAULT_TOOL_LIST: List[str] = ["spray_nozzle_01", "spray_nozzle_02"]
    _DEFAULT_CURRENT_TOOL: str = "spray_nozzle_01"

    # Outbound (Widget -> außen/Bridge)
    selectToolRequested = QtCore.pyqtSignal(str)

    def __init__(self, ros=None, parent: Optional[QWidget] = None, title: str = "Tool"):
        super().__init__(title, parent)
        self.ros = ros

        # Track whether we ever got a real inbound update (optional, but useful)
        self._got_inbound_list: bool = False
        self._got_inbound_current: bool = False

        self._build_ui()

        # Apply UI defaults first (so UI is usable even without ROS)
        self._apply_ui_defaults()

        # Then wire bridge (inbound may override defaults)
        self._wire_bridge_inbound()
        self._wire_outbound()

    # ------------------------------------------------------------------ defaults
    def _apply_ui_defaults(self) -> None:
        """
        UI-only defaults when tool node is not available.
        Inbound signals (if they ever arrive) will override these.
        """
        self.set_tool_list(list(self._DEFAULT_TOOL_LIST))
        self.set_current_tool(str(self._DEFAULT_CURRENT_TOOL))

        # also select the current in combobox (set_current_tool already tries)
        # if list didn't contain current (shouldn't happen), ensure first item
        if self.cmbTool.count() > 0 and self.cmbTool.currentIndex() < 0:
            self.cmbTool.blockSignals(True)
            try:
                self.cmbTool.setCurrentIndex(0)
            finally:
                self.cmbTool.blockSignals(False)

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
            def _on_list(items: list):
                self._got_inbound_list = True
                self.set_tool_list(list(items or []))
            sig.toolListChanged.connect(_on_list)  # list[str]

        # optional: Roh-String
        for cand in ("toolListStrChanged", "toolListUpdated", "toolListMsg"):
            if hasattr(sig, cand):
                def _on_str(s: str):
                    self._got_inbound_list = True
                    self.set_tool_list_from_string(s)
                getattr(sig, cand).connect(_on_str)

        if hasattr(sig, "currentToolChanged"):
            def _on_cur(name: str):
                self._got_inbound_current = True
                self.set_current_tool(name)
            sig.currentToolChanged.connect(_on_cur)

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
        items = list(items or [])

        cur = self.cmbTool.currentText().strip()
        self.cmbTool.blockSignals(True)
        try:
            self.cmbTool.clear()
            self.cmbTool.addItems(items)
            # best-effort: current selection beibehalten
            if cur:
                ix = self.cmbTool.findText(cur)
                if ix >= 0:
                    self.cmbTool.setCurrentIndex(ix)
        finally:
            self.cmbTool.blockSignals(False)

        # If no inbound current arrived yet, keep default current consistent with list
        if not self._got_inbound_current:
            if self._DEFAULT_CURRENT_TOOL in items:
                self.set_current_tool(self._DEFAULT_CURRENT_TOOL)
            elif items:
                self.set_current_tool(items[0])

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
