# -*- coding: utf-8 -*-
# File: app/widgets/motion_box.py
from __future__ import annotations
from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout,
    QPushButton, QDoubleSpinBox
)

from app.model.recipe.recipe_store import RecipeStore
from app.widgets.planner_groupbox import PlannerGroupBox


class MotionWidget(QWidget):
    """
    Service-Motion-Widget:

      1) PlannerGroupBox (role="service")
      2) Motion-Geschwindigkeit (mm/s)
      3) Buttons: Move to Home / Move to Service

    Signals (Widget → außen/Bridge):
      - motionSpeedChanged(float mm_s)
      - moveHomeRequestedWithSpeed(float mm_s)
      - moveServiceRequestedWithSpeed(float mm_s)
      - plannerCfgChanged(object)  # Dict mit pipeline/planner_id/params
    """

    motionSpeedChanged = QtCore.pyqtSignal(float)
    moveHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveServiceRequestedWithSpeed = QtCore.pyqtSignal(float)
    plannerCfgChanged = QtCore.pyqtSignal(object)  # z.B. Dict[str, Any]

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

    # ------------------------------------------------------------------ UI
    def _build_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- 1) Planner oben (role="service") ---
        self.planner = PlannerGroupBox(
            parent=self,
            title="Service planner",
            role="service",
            store=self.store,
        )
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

        self.spinSpeed.valueChanged.connect(
            lambda v: self.motionSpeedChanged.emit(float(v))
        )

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

        # Buttons → immer mit aktueller Speed
        self.btnHome.clicked.connect(
            lambda: self.moveHomeRequestedWithSpeed.emit(self.get_motion_speed_mm_s())
        )
        self.btnService.clicked.connect(
            lambda: self.moveServiceRequestedWithSpeed.emit(self.get_motion_speed_mm_s())
        )

    # ------------------------------------------------------------------ Bridge-Wiring
    def _wire_outbound_to_bridge_if_present(self):
        """
        Verdrahtet die Widget-Signale direkt mit der MotionBridge,
        wenn eine UIBridge mit MotionBridge (_motion) übergeben wurde.
        """
        if self.bridge is None:
            return

        motion_bridge = getattr(self.bridge, "_motion", None)
        if motion_bridge is None:
            return

        sig = getattr(motion_bridge, "signals", None)
        if sig is None:
            return

        # Speed-Änderung
        if hasattr(sig, "motionSpeedChanged"):
            self.motionSpeedChanged.connect(sig.motionSpeedChanged)

        # Buttons -> Varianten MIT Speed
        if hasattr(sig, "moveToHomeRequestedWithSpeed"):
            self.moveHomeRequestedWithSpeed.connect(sig.moveToHomeRequestedWithSpeed)
        if hasattr(sig, "moveToServiceRequestedWithSpeed"):
            self.moveServiceRequestedWithSpeed.connect(sig.moveToServiceRequestedWithSpeed)

        # Planner-Konfiguration (als Dict) → MotionBridge
        if hasattr(sig, "plannerCfgChanged"):
            self.plannerCfgChanged.connect(sig.plannerCfgChanged)

        # Beim ersten Verbinden direkt aktuelle Planner-Config einmal pushen
        self._emit_planner_cfg()

    # ------------------------------------------------------------------ Store-Handling
    def set_store(self, store: RecipeStore):
        """Store wechseln (z. B. nach Reload). Lädt sofort die Store-Defaults in den Planner."""
        if store is None:
            raise ValueError("MotionWidget.set_store: store=None ist nicht erlaubt.")
        self.store = store
        self.planner.set_store(store)
        self.planner.apply_store_defaults()
        self._emit_planner_cfg()

    # ------------------------------------------------------------------ Utilities
    def set_busy(self, busy: bool):
        self.btnHome.setEnabled(not busy)
        self.btnService.setEnabled(not busy)
        self.spinSpeed.setEnabled(not busy)

    # ------------------------------------------------------------------ Motion Speed API
    def get_motion_speed_mm_s(self) -> float:
        return float(self.spinSpeed.value())

    def set_motion_speed_mm_s(self, v: float) -> None:
        self.spinSpeed.setValue(float(v))

    # ------------------------------------------------------------------ Planner passthroughs (Service-Rolle)
    def apply_planner_model(self, planner_cfg: Dict[str, Any] | None):
        """Recipe.planner['service'] → UI."""
        self.planner.apply_planner_model(planner_cfg)
        self._emit_planner_cfg()

    def collect_planner(self) -> Dict[str, Any]:
        """UI → Dict (für Recipe.planner['service'])."""
        return self.planner.collect_planner()

    # Für ServiceTab-Forwarder
    def get_planner(self) -> str:
        return self.planner.get_planner()

    def get_params(self) -> Dict[str, Any]:
        return self.planner.get_params()

    def set_planner(self, pipeline: str):
        self.planner.set_planner(pipeline)
        self._emit_planner_cfg()

    def set_params(self, cfg: Dict[str, Any]):
        self.planner.set_params(cfg)
        self._emit_planner_cfg()

    def reset_defaults(self):
        """Lädt Defaults *nur* aus RecipeStore (service-Rolle bleibt)."""
        self.planner.apply_store_defaults()
        self._emit_planner_cfg()

    # ------------------------------------------------------------------ Planner → Bridge Helper
    def _emit_planner_cfg(self):
        """
        Baut die Planner-Konfiguration (Dict) aus der UI
        und emittiert plannerCfgChanged.
        """
        cfg = self.collect_planner()
        self.plannerCfgChanged.emit(cfg)
