# -*- coding: utf-8 -*-
# File: widgets/motion_box.py
from __future__ import annotations
from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QDoubleSpinBox
)

from app.model.recipe.recipe_store import RecipeStore
from widgets.planner_groupbox import PlannerGroupBox


class MotionWidget(QWidget):
    """
    Motion-Commands + Planner + eigene Motion-Geschwindigkeit (mm/s).

    REIHENFOLGE:
      1) PlannerGroupBox (Common, Pipeline, Planner ID, params)
      2) Speed (mm/s)
      3) Buttons: Move to Home / Move to Service

    Signals (Widget -> außen/Bridge):
      - motionSpeedChanged(float mm_s)
      - moveHomeRequested()
      - moveServiceRequested()
      - moveHomeRequestedWithSpeed(float mm_s)
      - moveServiceRequestedWithSpeed(float mm_s)

    Öffentliche API:
      - set_store(RecipeStore)
      - apply_planner_model(dict|None)
      - collect_planner() -> dict
      - get_motion_speed_mm_s() / set_motion_speed_mm_s()
      - get_planner() / set_planner(pipeline)
      - set_params(dict) / get_params()
      - reset_defaults()  # lädt Defaults strikt aus RecipeStore
    """

    motionSpeedChanged = QtCore.pyqtSignal(float)
    moveHomeRequested = QtCore.pyqtSignal()
    moveServiceRequested = QtCore.pyqtSignal()
    moveHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    def __init__(
        self,
        *,
        store: RecipeStore,
        bridge=None,
        parent: Optional[QWidget] = None
    ):
        if store is None:
            raise ValueError("MotionWidget: RecipeStore ist Pflicht (store=None).")
        super().__init__(parent)
        self.bridge = bridge
        self.store: RecipeStore = store
        self._build_ui()
        self._wire_outbound_to_bridge_if_present()

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- 1) Planner oben (strict storage-driven) ---
        self.planner = PlannerGroupBox(self, title="Planner", store=self.store)
        root.addWidget(self.planner)

        # --- 2) Motion Speed (mm/s) ---
        frm = QFormLayout()
        self.spinSpeed = QDoubleSpinBox(self)
        self.spinSpeed.setRange(1.0, 2000.0)
        self.spinSpeed.setSingleStep(1.0)
        self.spinSpeed.setDecimals(1)
        self.spinSpeed.setValue(100.0)
        self.spinSpeed.setSuffix(" mm/s")
        frm.addRow("Speed (mm/s)", self.spinSpeed)
        root.addLayout(frm)

        self.spinSpeed.valueChanged.connect(lambda v: self.motionSpeedChanged.emit(float(v)))

        # --- 3) Buttons ---
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

            # Legacy-Varianten OHNE Speed
            if hasattr(t, "moveToHomeRequested"):
                self.moveHomeRequested.connect(t.moveToHomeRequested.emit)
            if hasattr(t, "moveToServiceRequested"):
                self.moveServiceRequested.connect(t.moveToServiceRequested.emit)

    # ---------- Store-Handling ----------
    def set_store(self, store: RecipeStore):
        """Store wechseln (z. B. nach Reload). Lädt sofort die Store-Defaults in den Planner."""
        if store is None:
            raise ValueError("MotionWidget.set_store: store=None ist nicht erlaubt.")
        self.store = store
        self.planner.set_store(store)
        self.planner.apply_store_defaults()

    # ---------- Utilities ----------
    def set_busy(self, busy: bool):
        self.btnHome.setEnabled(not busy)  # type: ignore[attr-defined]
        self.btnService.setEnabled(not busy)  # type: ignore[attr-defined]
        self.spinSpeed.setEnabled(not busy)  # type: ignore[attr-defined]

    # ---------- Motion Speed API ----------
    def get_motion_speed_mm_s(self) -> float:
        return float(self.spinSpeed.value())

    def set_motion_speed_mm_s(self, v: float) -> None:
        self.spinSpeed.setValue(float(v))

    # ---------- Planner passthroughs ----------
    # Für Recipe <-> UI
    def apply_planner_model(self, planner_cfg: Dict[str, Any] | None):
        """Recipe.planner → UI (strikt, keine Fallbacks)."""
        self.planner.apply_planner_model(planner_cfg)

    def collect_planner(self) -> Dict[str, Any]:
        """UI → Dict (für Recipe.planner)."""
        return self.planner.collect_planner()

    # Für externe Nutzung
    def get_planner(self) -> str:
        return self.planner.get_planner()

    def get_params(self) -> Dict[str, Any]:
        return self.planner.get_params()

    def set_planner(self, pipeline: str):
        self.planner.set_planner(pipeline)

    def set_params(self, cfg: Dict[str, Any]):
        self.planner.set_params(cfg)

    def reset_defaults(self):
        """Lädt Defaults *nur* aus RecipeStore."""
        self.planner.apply_store_defaults()
