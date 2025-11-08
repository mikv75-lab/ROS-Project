# -*- coding: utf-8 -*-
# File: widgets/motion_box.py
from __future__ import annotations
from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QDoubleSpinBox, QWidget
)

# Planner-Widget (keine Signals, nur Getter/Setter)
from widgets.planner_groupbox import PlannerGroupBox, PLANNER_CONFIG


class MotionWidget(QWidget):
    """
    Motion-Commands + Planner + eigene Motion-Geschwindigkeit (mm/s).

    UI:
      - Speed (mm/s): eigener Regler NUR für Motion
      - Buttons: Move to Home / Move to Service
      - PlannerGroupBox (unten)
      - (kein UI-File)

    Signals (Widget -> außen/Bridge):
      - motionSpeedChanged(float mm_s)
      - moveHomeRequested()
      - moveServiceRequested()
      - moveHomeRequestedWithSpeed(float mm_s)
      - moveServiceRequestedWithSpeed(float mm_s)

    Planner-Forwarder:
      get_planner(), get_params(), set_planner(...), set_params(...), reset_defaults()
    """

    motionSpeedChanged = QtCore.pyqtSignal(float)
    moveHomeRequested = QtCore.pyqtSignal()
    moveServiceRequested = QtCore.pyqtSignal()
    moveHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    def __init__(self, bridge=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.bridge = bridge
        self._build_ui()
        self._wire_outbound_to_bridge_if_present()

    def _build_ui(self):
        root = QVBoxLayout(self)

        # --- Motion Speed (mm/s) ---
        frm = QFormLayout()
        self.spinSpeed = QDoubleSpinBox(self)
        self.spinSpeed.setRange(1.0, 2000.0)     # anpassbar
        self.spinSpeed.setSingleStep(1.0)
        self.spinSpeed.setDecimals(1)
        self.spinSpeed.setValue(100.0)           # Default
        self.spinSpeed.setSuffix(" mm/s")
        frm.addRow("Speed (mm/s)", self.spinSpeed)
        root.addLayout(frm)

        self.spinSpeed.valueChanged.connect(lambda v: self.motionSpeedChanged.emit(float(v)))

        # --- Buttons ---
        row = QHBoxLayout()
        self.btnHome = QPushButton("Move to Home", self)
        self.btnService = QPushButton("Move to Service", self)
        for b in (self.btnHome, self.btnService):
            b.setMinimumHeight(32)
        row.addWidget(self.btnHome)
        row.addWidget(self.btnService)
        row.addStretch(1)
        root.addLayout(row)

        # Buttons -> Signals (ohne und mit Speed)
        self.btnHome.clicked.connect(self.moveHomeRequested.emit)
        self.btnService.clicked.connect(self.moveServiceRequested.emit)
        self.btnHome.clicked.connect(lambda: self.moveHomeRequestedWithSpeed.emit(self.get_motion_speed_mm_s()))
        self.btnService.clicked.connect(lambda: self.moveServiceRequestedWithSpeed.emit(self.get_motion_speed_mm_s()))

        # --- Planner unten ---
        self.planner = PlannerGroupBox(self)
        self.planner.set_params(PLANNER_CONFIG)
        root.addWidget(self.planner)

    def _wire_outbound_to_bridge_if_present(self):
        # bevorzugt dedizierte Motion-Bridge, sonst Poses-Bridge
        targets = []
        if self.bridge is not None:
            for attr in ("_motion", "_pb"):
                obj = getattr(self.bridge, attr, None)
                if obj and getattr(obj, "signals", None):
                    targets.append(obj.signals)

        for t in targets:
            if hasattr(t, "motionSpeedChanged"):
                self.motionSpeedChanged.connect(t.motionSpeedChanged.emit)

            # Varianten MIT Speed
            if hasattr(t, "moveToHomeRequestedWithSpeed"):
                self.moveHomeRequestedWithSpeed.connect(t.moveToHomeRequestedWithSpeed.emit)
            if hasattr(t, "moveToServiceRequestedWithSpeed"):
                self.moveServiceRequestedWithSpeed.connect(t.moveToServiceRequestedWithSpeed.emit)

            # Fallback: alte Varianten OHNE Speed
            if hasattr(t, "moveToHomeRequested"):
                self.moveHomeRequested.connect(t.moveToHomeRequested.emit)
            if hasattr(t, "moveToServiceRequested"):
                self.moveServiceRequested.connect(t.moveToServiceRequested.emit)

    # ---------- Utilities ----------
    def set_busy(self, busy: bool):
        self.btnHome.setEnabled(not busy)  # Python hat kein ! -> wir korrigieren unten
        self.btnService.setEnabled(not busy)
        self.spinSpeed.setEnabled(not busy)

    # ---------- Motion Speed API ----------
    def get_motion_speed_mm_s(self) -> float:
        return float(self.spinSpeed.value())

    def set_motion_speed_mm_s(self, v: float) -> None:
        self.spinSpeed.setValue(float(v))

    # ---------- Planner Forwarders ----------
    def get_planner(self) -> str:
        return self.planner.get_planner()

    def get_params(self) -> Dict[str, Any]:
        return self.planner.get_params()

    def set_planner(self, pipeline: str):
        self.planner.set_planner(pipeline)

    def set_params(self, cfg: Dict[str, Any]):
        self.planner.set_params(cfg)

    def reset_defaults(self):
        self.planner.reset_defaults()
