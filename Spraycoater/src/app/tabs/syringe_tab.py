# -*- coding: utf-8 -*-
# src/app/tabs/service/syringe_tab.py
from __future__ import annotations

import logging
from typing import Any, Optional

from PyQt6 import QtCore
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QWidget,
    QGridLayout,
    QGroupBox,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QCheckBox,
    QPushButton,
    QDoubleSpinBox,
)

from plc.plc_client import PlcClientBase

LOG = logging.getLogger(__name__)


class SyringeTab(QWidget):
    """
    Service-Tab für Kartusche / Dispenser.

    WICHTIG:
      PLC-Callbacks können aus einem Worker-Thread kommen.
      UI-Updates müssen dann in den Qt-GUI-Thread gehoppt werden -> QTimer.singleShot(0, ...).
    """

    # ---- Commands → nach außen signalisieren ----
    start_syringe_requested = pyqtSignal()
    stop_syringe_requested = pyqtSignal()
    purge_syringe_requested = pyqtSignal()
    set_speed_requested = pyqtSignal(float)  # z.B. mm/s

    def __init__(
        self,
        *,
        ctx: Any,
        store: Any,
        bridge: Any,
        plc: Optional[PlcClientBase],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bridge = bridge
        self._plc: Optional[PlcClientBase] = plc

        self._dying: bool = False
        self.destroyed.connect(lambda *_: setattr(self, "_dying", True))

        self._chk_running: QCheckBox | None = None
        self._lbl_alarm: QLabel | None = None
        self._speed_spin: QDoubleSpinBox | None = None

        self._build_ui()
        self._init_plc_callbacks()

    # ------------------------------------------------------------------
    # Thread-safe UI update helper
    # ------------------------------------------------------------------
    def _ui(self, fn) -> None:
        """Hoppt sicher in den GUI-Thread (und ignoriert Updates beim Zerstören)."""
        if self._dying:
            return
        QtCore.QTimer.singleShot(0, lambda: (None if self._dying else fn()))

    # ------------------------------------------------------------------
    # PLC-Callback Registrierung
    # ------------------------------------------------------------------
    def _init_plc_callbacks(self) -> None:
        plc = self._plc
        if plc is None or not plc.is_connected:
            LOG.info("SyringeTab: PLC nicht vorhanden oder nicht verbunden.")
            self._set_running(False)
            self._set_alarm_text("")
            return

        # Nur Callbacks registrieren – initiale Werte bleiben auf Default (False/0)
        try:
            plc.register_bool_callback("syringe_running", self._cb_syringe_running)
        except Exception as e:
            LOG.error("SyringeTab: register_bool_callback('syringe_running', ...) failed: %s", e)

        try:
            plc.register_int_callback("syringe_alarm_code", self._cb_syringe_alarm_code)
        except Exception as e:
            LOG.error("SyringeTab: register_int_callback('syringe_alarm_code', ...) failed: %s", e)

    # ------------------------------------------------------------------
    # UI-Aufbau
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        grid = QGridLayout(self)

        # 1) Status-Box
        status_box = QGroupBox("Status", self)
        status_layout = QVBoxLayout(status_box)

        # "läuft" Checkbox
        row_running = QHBoxLayout()
        lbl_running = QLabel("Kartusche läuft:", self)
        chk_running = QCheckBox(self)
        chk_running.setEnabled(False)  # reine Anzeige
        row_running.addWidget(lbl_running)
        row_running.addStretch(1)
        row_running.addWidget(chk_running)
        status_layout.addLayout(row_running)
        self._chk_running = chk_running

        # Alarm-Label
        lbl_alarm_title = QLabel("Alarm:", self)
        lbl_alarm = QLabel("-", self)
        status_layout.addWidget(lbl_alarm_title)
        status_layout.addWidget(lbl_alarm)
        self._lbl_alarm = lbl_alarm

        grid.addWidget(status_box, 0, 0)

        # 2) Parameter-Box (Speed)
        param_box = QGroupBox("Parameter", self)
        param_layout = QVBoxLayout(param_box)

        row_speed = QHBoxLayout()
        lbl_speed = QLabel("Geschwindigkeit [mm/s]:", self)
        speed_spin = QDoubleSpinBox(self)
        speed_spin.setRange(0.1, 100.0)
        speed_spin.setSingleStep(0.1)
        speed_spin.setValue(10.0)
        speed_spin.setDecimals(2)
        row_speed.addWidget(lbl_speed)
        row_speed.addStretch(1)
        row_speed.addWidget(speed_spin)
        param_layout.addLayout(row_speed)
        self._speed_spin = speed_spin

        grid.addWidget(param_box, 0, 1)

        # 3) Steuer-Box (Start/Stop/Purge)
        ctrl_box = QGroupBox("Steuerung", self)
        ctrl_layout = QVBoxLayout(ctrl_box)

        row_buttons = QHBoxLayout()
        btn_start = QPushButton("Start", self)
        btn_stop = QPushButton("Stop", self)
        btn_purge = QPushButton("Spülen", self)

        row_buttons.addWidget(btn_start)
        row_buttons.addWidget(btn_stop)
        row_buttons.addWidget(btn_purge)
        ctrl_layout.addLayout(row_buttons)

        # Button-Signale → eigene Qt-Signale
        btn_start.clicked.connect(self._on_start_clicked)
        btn_stop.clicked.connect(self._on_stop_clicked)
        btn_purge.clicked.connect(self._on_purge_clicked)

        # Speed-Änderung → set_speed_requested
        speed_spin.valueChanged.connect(self._on_speed_changed)

        grid.addWidget(ctrl_box, 1, 0, 1, 2)  # unten über beide Spalten

    # ------------------------------------------------------------------
    # UI-Helfer
    # ------------------------------------------------------------------
    def _set_running(self, running: bool) -> None:
        if self._chk_running is not None:
            self._chk_running.setChecked(bool(running))

    def _set_alarm_text(self, text: str) -> None:
        if self._lbl_alarm is not None:
            self._lbl_alarm.setText(text if text else "-")

    # ------------------------------------------------------------------
    # PLC-Callbacks (thread-safe via _ui)
    # ------------------------------------------------------------------
    def _cb_syringe_running(self, value: bool) -> None:
        """PLC → UI: syringe_running"""
        self._ui(lambda: self._set_running(bool(value)))

    def _cb_syringe_alarm_code(self, code: int) -> None:
        """PLC → UI: syringe_alarm_code"""
        c = int(code) if code else 0
        text = f"Alarmcode: {c}" if c else ""
        self._ui(lambda: self._set_alarm_text(text))

    # ------------------------------------------------------------------
    # Button-Handler → Qt-Signale nach außen
    # ------------------------------------------------------------------
    def _on_start_clicked(self) -> None:
        LOG.info("SyringeTab: Start angefordert")
        self.start_syringe_requested.emit()

    def _on_stop_clicked(self) -> None:
        LOG.info("SyringeTab: Stop angefordert")
        self.stop_syringe_requested.emit()

    def _on_purge_clicked(self) -> None:
        LOG.info("SyringeTab: Spülen angefordert")
        self.purge_syringe_requested.emit()

    def _on_speed_changed(self, value: float) -> None:
        v = float(value)
        LOG.info("SyringeTab: Geschwindigkeit gesetzt: %.3f mm/s", v)
        self.set_speed_requested.emit(v)
