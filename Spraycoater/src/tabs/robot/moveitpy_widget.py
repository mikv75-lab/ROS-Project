# -*- coding: utf-8 -*-
# File: app/widgets/moveitpy_widget.py
from __future__ import annotations

from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtCore import QEvent, QObject, QTimer
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QPushButton,
    QDoubleSpinBox,
)

from model.recipe.recipe_store import RecipeStore
from widgets.planner_groupbox import PlannerGroupBox


class MoveItPyWidget(QWidget):
    """
    Service-MoveItPy-Widget:

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
    plannerCfgChanged = QtCore.pyqtSignal(object)  # Dict[str, Any]

    def __init__(
        self,
        *,
        store: RecipeStore,
        ros=None,
        parent: Optional[QWidget] = None,
    ):
        if store is None:
            raise ValueError("MoveItPyWidget: RecipeStore ist Pflicht (store=None).")
        super().__init__(parent)

        self.store: RecipeStore = store
        self.ros = None

        # Debounce für Planner-Config (UI-Änderungen im PlannerGroupBox)
        self._planner_emit_timer = QTimer(self)
        self._planner_emit_timer.setSingleShot(True)
        self._planner_emit_timer.setInterval(60)
        self._planner_emit_timer.timeout.connect(self._emit_planner_cfg)

        self._build_ui()
        self._install_planner_change_watchdog()

        if ros is not None:
            self.set_ros(ros)

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # 1) Planner (role="service")
        self.planner = PlannerGroupBox(
            parent=self,
            title="Service planner",
            role="service",
            store=self.store,
        )
        root.addWidget(self.planner)

        # 2) Motion speed (mm/s)
        frm = QFormLayout()
        self.spinSpeed = QDoubleSpinBox(self)
        self.spinSpeed.setRange(1.0, 2000.0)
        self.spinSpeed.setSingleStep(1.0)
        self.spinSpeed.setDecimals(1)
        self.spinSpeed.setValue(100.0)
        self.spinSpeed.setSuffix(" mm/s")
        frm.addRow("Speed (mm/s)", self.spinSpeed)
        root.addLayout(frm)

        self.spinSpeed.valueChanged.connect(self._on_speed_changed)

        # 3) Buttons
        row = QHBoxLayout()
        self.btnHome = QPushButton("Move to Home", self)
        self.btnService = QPushButton("Move to Service", self)
        for b in (self.btnHome, self.btnService):
            b.setMinimumHeight(32)
        row.addWidget(self.btnHome)
        row.addWidget(self.btnService)
        row.addStretch(1)
        root.addLayout(row)

        # >>> WICHTIG: Restplatz nach unten drücken
        root.addStretch(1)

        # Optional (empfohlen): Planner darf wachsen, Rest bleibt oben kompakt
        root.setStretchFactor(self.planner, 1)

        self.btnHome.clicked.connect(self._on_move_home)
        self.btnService.clicked.connect(self._on_move_service)

    def _on_speed_changed(self, v: float) -> None:
        self.motionSpeedChanged.emit(float(v))

    def _on_move_home(self) -> None:
        self.moveHomeRequestedWithSpeed.emit(self.get_motion_speed_mm_s())

    def _on_move_service(self) -> None:
        self.moveServiceRequestedWithSpeed.emit(self.get_motion_speed_mm_s())

    # ------------------------------------------------------------------ Bridge

    def set_ros(self, ros) -> None:
        """
        Setzt/tauscht die RosBridge und verdrahtet die Widget-Signale nach außen.
        Erwartung: ros.moveitpy.signals ist vorhanden.
        """
        self.ros = ros
        self._wire_outbound_to_ros()

        # Direkt nach dem Verdrahten einmal den aktuellen Zustand pushen
        self.motionSpeedChanged.emit(self.get_motion_speed_mm_s())
        self._emit_planner_cfg()

    def _wire_outbound_to_ros(self) -> None:
        if self.ros is None:
            return

        moveitpy_bridge = getattr(self.ros, "moveitpy", None)
        if moveitpy_bridge is None:
            return

        sig = getattr(moveitpy_bridge, "signals", None)
        if sig is None:
            return

        # Speed
        if hasattr(sig, "motionSpeedChanged"):
            self.motionSpeedChanged.connect(sig.motionSpeedChanged)

        # Moves (mit Speed)
        if hasattr(sig, "moveToHomeRequestedWithSpeed"):
            self.moveHomeRequestedWithSpeed.connect(sig.moveToHomeRequestedWithSpeed)
        if hasattr(sig, "moveToServiceRequestedWithSpeed"):
            self.moveServiceRequestedWithSpeed.connect(sig.moveToServiceRequestedWithSpeed)

        # Planner-Cfg (Dict)
        if hasattr(sig, "plannerCfgChanged"):
            self.plannerCfgChanged.connect(sig.plannerCfgChanged)

    # ------------------------------------------------------------------ Store

    def set_store(self, store: RecipeStore) -> None:
        """Store wechseln (z.B. nach Reload). Lädt sofort Store-Defaults in den Planner."""
        if store is None:
            raise ValueError("MoveItPyWidget.set_store: store=None ist nicht erlaubt.")
        self.store = store
        self.planner.set_store(store)
        self.planner.apply_store_defaults()
        self._emit_planner_cfg()

    # ------------------------------------------------------------------ Busy

    def set_busy(self, busy: bool) -> None:
        en = not bool(busy)
        self.btnHome.setEnabled(en)
        self.btnService.setEnabled(en)
        self.spinSpeed.setEnabled(en)
        self.planner.setEnabled(en)

    # ------------------------------------------------------------------ Motion speed API

    def get_motion_speed_mm_s(self) -> float:
        return float(self.spinSpeed.value())

    def set_motion_speed_mm_s(self, v: float) -> None:
        self.spinSpeed.setValue(float(v))

    # ------------------------------------------------------------------ Planner passthroughs

    def apply_planner_model(self, planner_cfg: Dict[str, Any] | None) -> None:
        """Recipe.planner['service'] → UI."""
        self.planner.apply_planner_model(planner_cfg or {})
        self._emit_planner_cfg()

    def collect_planner(self) -> Dict[str, Any]:
        """UI → Dict (für Recipe.planner['service'])."""
        return dict(self.planner.collect_planner() or {})

    def get_planner(self) -> str:
        return self.planner.get_planner()

    def get_params(self) -> Dict[str, Any]:
        return dict(self.planner.get_params() or {})

    def set_planner(self, pipeline: str) -> None:
        self.planner.set_planner(pipeline)
        self._emit_planner_cfg()

    def set_params(self, cfg: Dict[str, Any]) -> None:
        self.planner.set_params(cfg)
        self._emit_planner_cfg()

    def reset_defaults(self) -> None:
        """Lädt Defaults nur aus dem RecipeStore (service-Rolle bleibt)."""
        self.planner.apply_store_defaults()
        self._emit_planner_cfg()

    # ------------------------------------------------------------------ Planner change detection

    def _install_planner_change_watchdog(self) -> None:
        """
        PlannerGroupBox liefert nicht zwingend ein einheitliches 'changed'-Signal.
        Stattdessen beobachten wir UI-Events (Key/Mouse/Focus) innerhalb des Planner-Bereichs
        und emittieren die Planner-Konfiguration debounced.
        """
        self.planner.installEventFilter(self)
        for w in self.planner.findChildren(QWidget):
            w.installEventFilter(self)

    def eventFilter(self, obj: QObject, ev: QEvent) -> bool:  # noqa: N802 (Qt API)
        t = ev.type()
        if t in (
            QEvent.Type.MouseButtonRelease,
            QEvent.Type.KeyRelease,
            QEvent.Type.FocusOut,
            QEvent.Type.Wheel,
        ):
            self._schedule_emit_planner_cfg()
        return super().eventFilter(obj, ev)

    def _schedule_emit_planner_cfg(self) -> None:
        # Debounce: mehrere UI-Events bündeln
        self._planner_emit_timer.start()

    # ------------------------------------------------------------------ Planner → Signal

    def _emit_planner_cfg(self) -> None:
        """Baut die Planner-Konfiguration (Dict) aus der UI und emittiert plannerCfgChanged."""
        self.plannerCfgChanged.emit(self.collect_planner())
