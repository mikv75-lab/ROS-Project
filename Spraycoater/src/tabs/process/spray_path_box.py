# -*- coding: utf-8 -*-
# File: tabs/process/spray_path_box.py
#
# SprayPathBox – besitzt Bridge, verdrahtet sich selbst, hat eigene Qt-Signals
# Ziel: GUI sendet NUR Bool-Toggles (show_compiled/show_traj/show_executed) an die SprayPathBridge.
# Default: alle drei True.

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
    UI für SprayPath Layer Toggles (Compiled / Traj / Executed).

    - Verdrahtet direkt zur SprayPathBridge (ros.spray.signals).
    - Checkboxen senden ausschließlich True/False (std_msgs/Bool) via Bridge.
    - Default: alle drei True.

    Erwartete Bridge-Signale (Outbound, UI->ROS):
      - signals.showCompiledRequested(bool)
      - signals.showTrajRequested(bool)
      - signals.showExecutedRequested(bool)

    Optional (Inbound, ROS->UI):
      - signals.compiledAvailableChanged(bool)
      - signals.trajAvailableChanged(bool)
      - signals.executedAvailableChanged(bool)
    """

    # --- Public UI signals (ProcessTab erwartet *Toggled*) ---
    showCompiledToggled = QtCore.pyqtSignal(bool)
    showTrajToggled = QtCore.pyqtSignal(bool)
    showExecutedToggled = QtCore.pyqtSignal(bool)

    # --- Backward/alias: "Requested" (falls an anderen Stellen genutzt) ---
    showCompiledRequested = QtCore.pyqtSignal(bool)
    showTrajRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)

    def __init__(self, ros, parent: Optional[QWidget] = None, title: str = "Spray Paths"):
        super().__init__(title, parent)

        self.ros = ros
        self._sb = getattr(self.ros, "spray", None) if self.ros is not None else None
        if self._sb is None:
            raise RuntimeError("SprayPathBox: ros.spray is None (SprayPathBridge not started?)")
        self._sig = self._sb.signals

        self._block: bool = False

        # optional availability state
        self._avail_compiled: Optional[bool] = None
        self._avail_traj: Optional[bool] = None
        self._avail_executed: Optional[bool] = None

        self._build_ui()
        self._wire_inbound_optional()
        self._wire_outbound()

        # Default: always all True, and publish once
        self.set_defaults(compiled=True, traj=True, executed=True)

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        g = QGridLayout(self)

        def _bold(txt: str) -> QLabel:
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            return l

        g.addWidget(_bold("Layer"), 0, 0)
        g.addWidget(_bold("Show"), 0, 1)
        g.addWidget(_bold("Available"), 0, 2)

        g.addWidget(QLabel("Compiled (recipe path)", self), 1, 0)
        self.chkCompiled = QCheckBox("", self)
        g.addWidget(self.chkCompiled, 1, 1)
        self.lblCompiledAvail = QLabel("–", self)
        g.addWidget(self.lblCompiledAvail, 1, 2)

        g.addWidget(QLabel("Traj (saved/validate/optimize)", self), 2, 0)
        self.chkTraj = QCheckBox("", self)
        g.addWidget(self.chkTraj, 2, 1)
        self.lblTrajAvail = QLabel("–", self)
        g.addWidget(self.lblTrajAvail, 2, 2)

        g.addWidget(QLabel("Executed (last executed)", self), 3, 0)
        self.chkExecuted = QCheckBox("", self)
        g.addWidget(self.chkExecuted, 3, 1)
        self.lblExecutedAvail = QLabel("–", self)
        g.addWidget(self.lblExecutedAvail, 3, 2)

    # ------------------------------------------------------------------ Bridge inbound (optional)

    def _wire_inbound_optional(self) -> None:
        try:
            if hasattr(self._sig, "compiledAvailableChanged"):
                self._sig.compiledAvailableChanged.connect(self._on_compiled_avail)
            if hasattr(self._sig, "trajAvailableChanged"):
                self._sig.trajAvailableChanged.connect(self._on_traj_avail)
            if hasattr(self._sig, "executedAvailableChanged"):
                self._sig.executedAvailableChanged.connect(self._on_executed_avail)
        except Exception:
            _LOG.exception("SprayPathBox: optional inbound wiring failed")

    @QtCore.pyqtSlot(bool)
    def _on_compiled_avail(self, v: bool) -> None:
        self._avail_compiled = bool(v)
        self._refresh_avail_labels()

    @QtCore.pyqtSlot(bool)
    def _on_traj_avail(self, v: bool) -> None:
        self._avail_traj = bool(v)
        self._refresh_avail_labels()

    @QtCore.pyqtSlot(bool)
    def _on_executed_avail(self, v: bool) -> None:
        self._avail_executed = bool(v)
        self._refresh_avail_labels()

    def _refresh_avail_labels(self) -> None:
        def _fmt(x: Optional[bool]) -> str:
            if x is None:
                return "–"
            return "yes" if x else "no"

        self.lblCompiledAvail.setText(_fmt(self._avail_compiled))
        self.lblTrajAvail.setText(_fmt(self._avail_traj))
        self.lblExecutedAvail.setText(_fmt(self._avail_executed))

    # ------------------------------------------------------------------ Outbound (Widget -> Bridge)

    def _wire_outbound(self) -> None:
        # Checkbox toggles -> internal handler (guarded)
        self.chkCompiled.toggled.connect(self._on_chk_compiled_toggled)
        self.chkTraj.toggled.connect(self._on_chk_traj_toggled)
        self.chkExecuted.toggled.connect(self._on_chk_executed_toggled)

        # UI signals -> Bridge signals (robust via .emit)
        # Bridge MUST provide these three.
        self.showCompiledRequested.connect(self._sig.showCompiledRequested.emit)
        self.showTrajRequested.connect(self._sig.showTrajRequested.emit)
        self.showExecutedRequested.connect(self._sig.showExecutedRequested.emit)

    @QtCore.pyqtSlot(bool)
    def _on_chk_compiled_toggled(self, v: bool) -> None:
        if self._block:
            return
        self._emit_compiled(bool(v))

    @QtCore.pyqtSlot(bool)
    def _on_chk_traj_toggled(self, v: bool) -> None:
        if self._block:
            return
        self._emit_traj(bool(v))

    @QtCore.pyqtSlot(bool)
    def _on_chk_executed_toggled(self, v: bool) -> None:
        if self._block:
            return
        self._emit_executed(bool(v))

    # ------------------------------------------------------------------ Emit helpers

    def _emit_compiled(self, v: bool) -> None:
        # Public + alias
        self.showCompiledToggled.emit(v)
        self.showCompiledRequested.emit(v)

    def _emit_traj(self, v: bool) -> None:
        self.showTrajToggled.emit(v)
        self.showTrajRequested.emit(v)

    def _emit_executed(self, v: bool) -> None:
        self.showExecutedToggled.emit(v)
        self.showExecutedRequested.emit(v)

    # ------------------------------------------------------------------ Public API

    def set_defaults(self, *, compiled: bool = True, traj: bool = True, executed: bool = True) -> None:
        """
        Default ist IMMER alle True. Diese Methode setzt die Checkboxen (ohne Toggle-Sturm)
        und publiziert anschließend die Zustände einmal in ROS (latched).
        """
        self._block = True
        try:
            self.chkCompiled.setChecked(bool(compiled))
            self.chkTraj.setChecked(bool(traj))
            self.chkExecuted.setChecked(bool(executed))
        finally:
            self._block = False
        self.publish_current()

    def reset_defaults(self) -> None:
        """Alias für ältere Call-Sites."""
        self.set_defaults(compiled=True, traj=True, executed=True)

    def publish_current(self) -> None:
        """Publiziert die aktuellen Checkbox-Zustände (auch nach recipe load / republish)."""
        try:
            self._sig.showCompiledRequested.emit(bool(self.chkCompiled.isChecked()))
            self._sig.showTrajRequested.emit(bool(self.chkTraj.isChecked()))
            self._sig.showExecutedRequested.emit(bool(self.chkExecuted.isChecked()))
        except Exception:
            _LOG.exception("SprayPathBox: publish_current failed")
