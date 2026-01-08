# -*- coding: utf-8 -*-
# File: tabs/process/spray_path_box.py
#
# SprayPathBox – besitzt Bridge, verdrahtet sich selbst, hat eigene Qt-Signals
# Ziel: GUI sendet NUR Bool-Toggles (show_compiled/show_planned/show_executed) an die SprayPathBridge.
# Default: alle drei True.
#
# Backward-Compat:
# - ProcessTab erwartet historisch showTrajToggled/showTrajRequested (== planned).
# - Wir exposen daher "planned" als Primary und halten "traj" als Alias.

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

    - Verdrahtet direkt zur SprayPathBridge (ros.spray.signals).
    - Checkboxen senden ausschließlich True/False (std_msgs/Bool) via Bridge.
    - Default: alle drei True.

    Erwartete Bridge-Signale (Outbound, UI->ROS):
      - signals.showCompiledRequested(bool)
      - signals.showPlannedRequested(bool)
      - signals.showExecutedRequested(bool)

    Optional (Inbound, ROS->UI):
      - signals.compiledAvailableChanged(bool)
      - signals.plannedAvailableChanged(bool)
      - signals.executedAvailableChanged(bool)

    Backward-Compat (optional auf Bridge-Seite):
      - signals.showTrajRequested(bool)          # alias für planned
      - signals.trajAvailableChanged(bool)      # alias für plannedAvailableChanged
    """

    # --- Public UI signals (Primary: Planned) ---
    showCompiledToggled = QtCore.pyqtSignal(bool)
    showPlannedToggled = QtCore.pyqtSignal(bool)
    showExecutedToggled = QtCore.pyqtSignal(bool)

    showCompiledRequested = QtCore.pyqtSignal(bool)
    showPlannedRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)

    # --- Backward-Compat for ProcessTab (Traj == Planned) ---
    showTrajToggled = QtCore.pyqtSignal(bool)
    showTrajRequested = QtCore.pyqtSignal(bool)

    def __init__(self, ros, parent: Optional[QWidget] = None, title: str = "Spray Paths"):
        super().__init__(title, parent)

        self.ros = ros
        self._sb = getattr(self.ros, "spray", None) if self.ros is not None else None
        if self._sb is None:
            raise RuntimeError("SprayPathBox: ros.spray is None (SprayPathBridge not started?)")
        self._sig = self._sb.signals

        # Hard requirement: planned/executed naming
        for name in ("showCompiledRequested", "showPlannedRequested", "showExecutedRequested"):
            if not hasattr(self._sig, name):
                raise RuntimeError(f"SprayPathBox: Bridge missing signals.{name}")

        self._block: bool = False

        # optional availability state
        self._avail_compiled: Optional[bool] = None
        self._avail_planned: Optional[bool] = None
        self._avail_executed: Optional[bool] = None

        self._build_ui()
        self._wire_inbound_optional()
        self._wire_outbound()

        # Default: always all True, and publish once
        self.set_defaults(compiled=True, planned=True, executed=True)

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

        g.addWidget(QLabel("Planned (validate/optimize)", self), 2, 0)
        self.chkPlanned = QCheckBox("", self)
        g.addWidget(self.chkPlanned, 2, 1)
        self.lblPlannedAvail = QLabel("–", self)
        g.addWidget(self.lblPlannedAvail, 2, 2)

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

            # preferred
            if hasattr(self._sig, "plannedAvailableChanged"):
                self._sig.plannedAvailableChanged.connect(self._on_planned_avail)
            # legacy alias
            elif hasattr(self._sig, "trajAvailableChanged"):
                self._sig.trajAvailableChanged.connect(self._on_planned_avail)

            if hasattr(self._sig, "executedAvailableChanged"):
                self._sig.executedAvailableChanged.connect(self._on_executed_avail)
        except Exception:
            _LOG.exception("SprayPathBox: optional inbound wiring failed")

    @QtCore.pyqtSlot(bool)
    def _on_compiled_avail(self, v: bool) -> None:
        self._avail_compiled = bool(v)
        self._refresh_avail_labels()

    @QtCore.pyqtSlot(bool)
    def _on_planned_avail(self, v: bool) -> None:
        self._avail_planned = bool(v)
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
        self.lblPlannedAvail.setText(_fmt(self._avail_planned))
        self.lblExecutedAvail.setText(_fmt(self._avail_executed))

    # ------------------------------------------------------------------ Outbound (Widget -> Bridge)

    def _wire_outbound(self) -> None:
        self.chkCompiled.toggled.connect(self._on_chk_compiled_toggled)
        self.chkPlanned.toggled.connect(self._on_chk_planned_toggled)
        self.chkExecuted.toggled.connect(self._on_chk_executed_toggled)

        # UI signals -> Bridge signals (publish Bool toggles)
        self.showCompiledRequested.connect(self._sig.showCompiledRequested.emit)
        self.showPlannedRequested.connect(self._sig.showPlannedRequested.emit)
        self.showExecutedRequested.connect(self._sig.showExecutedRequested.emit)

        # IMPORTANT:
        # Do NOT connect showTrajRequested back into the bridge, otherwise planned gets published twice.
        # Legacy listeners can still connect to showTrajRequested externally.

    @QtCore.pyqtSlot(bool)
    def _on_chk_compiled_toggled(self, v: bool) -> None:
        if self._block:
            return
        v = bool(v)
        self.showCompiledToggled.emit(v)
        self.showCompiledRequested.emit(v)

    @QtCore.pyqtSlot(bool)
    def _on_chk_planned_toggled(self, v: bool) -> None:
        if self._block:
            return
        v = bool(v)

        # Primary planned (this is the ONLY one that must hit the bridge)
        self.showPlannedToggled.emit(v)
        self.showPlannedRequested.emit(v)

        # Backward-Compat (ProcessTab expects "Traj") - emitted for legacy listeners only
        self.showTrajToggled.emit(v)
        self.showTrajRequested.emit(v)

    @QtCore.pyqtSlot(bool)
    def _on_chk_executed_toggled(self, v: bool) -> None:
        if self._block:
            return
        v = bool(v)
        self.showExecutedToggled.emit(v)
        self.showExecutedRequested.emit(v)

    # ------------------------------------------------------------------ Public API

    def set_defaults(self, *, compiled: bool = True, planned: bool = True, executed: bool = True) -> None:
        """
        Setzt Checkboxen (ohne Toggle-Sturm) und publiziert anschließend die Zustände einmal in ROS.
        """
        self._block = True
        try:
            self.chkCompiled.setChecked(bool(compiled))
            self.chkPlanned.setChecked(bool(planned))
            self.chkExecuted.setChecked(bool(executed))
        finally:
            self._block = False
        self.publish_current()

    # Legacy API name kept for callers that still pass traj=
    def set_defaults_legacy(self, *, compiled: bool = True, traj: bool = True, executed: bool = True) -> None:
        self.set_defaults(compiled=compiled, planned=traj, executed=executed)

    def reset_defaults(self) -> None:
        self.set_defaults(compiled=True, planned=True, executed=True)

    def publish_current(self) -> None:
        try:
            self._sig.showCompiledRequested.emit(bool(self.chkCompiled.isChecked()))
            self._sig.showPlannedRequested.emit(bool(self.chkPlanned.isChecked()))
            self._sig.showExecutedRequested.emit(bool(self.chkExecuted.isChecked()))
        except Exception:
            _LOG.exception("SprayPathBox: publish_current failed")
