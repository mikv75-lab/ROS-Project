# -*- coding: utf-8 -*-
# File: src/tabs/process/spray_path_box.py
from __future__ import annotations

import logging
from typing import Optional

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QGroupBox,
    QGridLayout,
    QLabel,
    QCheckBox,
    QWidget,
)

_LOG = logging.getLogger("tabs.process.spray_path_box")


class SprayPathBox(QGroupBox):
    """
    UI für SprayPath Layer Toggles (Compiled / Planned / Executed).
    Direkt verdrahtet mit ROS Bridge Signals.
    """

    showCompiledToggled = QtCore.pyqtSignal(bool)
    showPlannedToggled = QtCore.pyqtSignal(bool)
    showExecutedToggled = QtCore.pyqtSignal(bool)

    # Bridge Outbound
    showCompiledRequested = QtCore.pyqtSignal(bool)
    showPlannedRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)

    # Legacy alias
    showTrajToggled = QtCore.pyqtSignal(bool)
    showTrajRequested = QtCore.pyqtSignal(bool)

    def __init__(self, ros, parent: Optional[QWidget] = None, title: Optional[str] = "Spray Paths"):
        super().__init__(title or "", parent)

        self.ros = ros
        self._sb = getattr(self.ros, "spray", None) if self.ros else None
        if self._sb is None:
            raise RuntimeError("SprayPathBox: ros.spray is None")
        self._sig = self._sb.signals

        self._block: bool = False
        self._avail_compiled: Optional[bool] = None
        self._avail_planned: Optional[bool] = None
        self._avail_executed: Optional[bool] = None

        self._build_ui()
        self._wire_inbound_optional()
        self._wire_outbound()

        self.set_defaults(compiled=True, planned=True, executed=True)

    def _build_ui(self) -> None:
        g = QGridLayout(self)
        g.setContentsMargins(8, 8, 8, 8)

        def _bold(txt: str) -> QLabel:
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            return l

        g.addWidget(_bold("Layer"), 0, 0)
        g.addWidget(_bold("Show"), 0, 1)
        g.addWidget(_bold("Available"), 0, 2)

        # Zeilen
        self.chkCompiled = QCheckBox("", self)
        self.lblCompiledAvail = QLabel("–", self)
        g.addWidget(QLabel("Compiled", self), 1, 0)
        g.addWidget(self.chkCompiled, 1, 1)
        g.addWidget(self.lblCompiledAvail, 1, 2)

        self.chkPlanned = QCheckBox("", self)
        self.lblPlannedAvail = QLabel("–", self)
        g.addWidget(QLabel("Planned", self), 2, 0)
        g.addWidget(self.chkPlanned, 2, 1)
        g.addWidget(self.lblPlannedAvail, 2, 2)

        self.chkExecuted = QCheckBox("", self)
        self.lblExecutedAvail = QLabel("–", self)
        g.addWidget(QLabel("Executed", self), 3, 0)
        g.addWidget(self.chkExecuted, 3, 1)
        g.addWidget(self.lblExecutedAvail, 3, 2)

    def _wire_inbound_optional(self) -> None:
        try:
            if hasattr(self._sig, "compiledAvailableChanged"):
                self._sig.compiledAvailableChanged.connect(lambda v: self._set_avail("compiled", v))
            if hasattr(self._sig, "plannedAvailableChanged"):
                self._sig.plannedAvailableChanged.connect(lambda v: self._set_avail("planned", v))
            if hasattr(self._sig, "executedAvailableChanged"):
                self._sig.executedAvailableChanged.connect(lambda v: self._set_avail("executed", v))
        except Exception:
            pass

    def _set_avail(self, which: str, val: bool) -> None:
        txt = "yes" if val else "no"
        if which == "compiled": self.lblCompiledAvail.setText(txt)
        elif which == "planned": self.lblPlannedAvail.setText(txt)
        elif which == "executed": self.lblExecutedAvail.setText(txt)

    def _wire_outbound(self) -> None:
        self.chkCompiled.toggled.connect(self._on_toggled_compiled)
        self.chkPlanned.toggled.connect(self._on_toggled_planned)
        self.chkExecuted.toggled.connect(self._on_toggled_executed)

        self.showCompiledRequested.connect(self._sig.showCompiledRequested.emit)
        self.showPlannedRequested.connect(self._sig.showPlannedRequested.emit)
        self.showExecutedRequested.connect(self._sig.showExecutedRequested.emit)

    def _on_toggled_compiled(self, v: bool) -> None:
        if self._block: return
        self.showCompiledToggled.emit(v)
        self.showCompiledRequested.emit(v)

    def _on_toggled_planned(self, v: bool) -> None:
        if self._block: return
        self.showPlannedToggled.emit(v)
        self.showPlannedRequested.emit(v)
        # Legacy
        self.showTrajToggled.emit(v)
        self.showTrajRequested.emit(v)

    def _on_toggled_executed(self, v: bool) -> None:
        if self._block: return
        self.showExecutedToggled.emit(v)
        self.showExecutedRequested.emit(v)

    def set_defaults(self, *, compiled=True, planned=True, executed=True) -> None:
        self._block = True
        self.chkCompiled.setChecked(compiled)
        self.chkPlanned.setChecked(planned)
        self.chkExecuted.setChecked(executed)
        self._block = False
        self.publish_current()

    def publish_current(self) -> None:
        try:
            self._sig.showCompiledRequested.emit(self.chkCompiled.isChecked())
            self._sig.showPlannedRequested.emit(self.chkPlanned.isChecked())
            self._sig.showExecutedRequested.emit(self.chkExecuted.isChecked())
        except Exception: pass