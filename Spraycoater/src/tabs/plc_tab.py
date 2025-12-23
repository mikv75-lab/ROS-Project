# -*- coding: utf-8 -*-
# src/app/tabs/service/plc_tab.py
from __future__ import annotations

import logging
from typing import Any, Optional, Dict, Callable

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
    QSpacerItem,
    QSizePolicy,
)

from plc.plc_client import PlcClientBase, PlcStatus

LOG = logging.getLogger(__name__)


class PlcTab(QWidget):
    """
    Kombinierter Tab: Signals + Syringe in einem Widget.

    Layout:
      [ Signals-GroupBox ] [ Syringe-GroupBox ]  (nebeneinander)
      Beide Bereiche bleiben kompakt (Spacers fressen freien Platz).
    """

    # ---- Commands → nach außen signalisieren ----
    start_syringe_requested = pyqtSignal()
    stop_syringe_requested = pyqtSignal()
    purge_syringe_requested = pyqtSignal()
    set_speed_requested = pyqtSignal(float)  # mm/s

    def __init__(
        self,
        *,
        ctx: Any,
        store: Any,
        # NEU: ros=... akzeptieren
        ros: Any = None,
        # ALT: bridge=... weiterhin akzeptieren
        bridge: Any = None,
        # optional weitere Legacy-Namen, falls irgendwo verwendet
        ui_bridge: Any = None,
        ros_bridge: Any = None,
        plc: Optional[PlcClientBase],
        parent: Optional[QWidget] = None,
        **_ignored: Any,
    ) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        # unify -> self.bridge (damit dein restlicher Code unverändert bleibt)
        self.bridge = (
            ros
            if ros is not None
            else (bridge if bridge is not None else (ui_bridge if ui_bridge is not None else ros_bridge))
        )

        self._plc: Optional[PlcClientBase] = plc

        self._dying: bool = False
        self.destroyed.connect(lambda *_: setattr(self, "_dying", True))

        # ---------------- Signals UI State ----------------
        self._inputs: Dict[str, QCheckBox] = {}

        # ---------------- Syringe UI State ----------------
        self._chk_running: QCheckBox | None = None
        self._lbl_alarm: QLabel | None = None
        self._speed_spin: QDoubleSpinBox | None = None

        self._build_ui()
        self._init_plc()
        self._add_callbacks()

    # ------------------------------------------------------------------
    # Thread-safe UI update helper
    # ------------------------------------------------------------------
    def _ui(self, fn: Callable[[], None]) -> None:
        """Hoppt sicher in den GUI-Thread (und ignoriert Updates beim Zerstören)."""
        if self._dying:
            return

        if QtCore.QThread.currentThread() == self.thread():
            fn()
            return

        QtCore.QTimer.singleShot(0, lambda: (None if self._dying else fn()))

    # ------------------------------------------------------------------
    # UI-Aufbau (kompakt + Spacer)
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        root = QGridLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)
        root.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignTop | QtCore.Qt.AlignmentFlag.AlignLeft
        )

        # ===== Signals GroupBox (links) =====
        gb_signals = QGroupBox("Signals", self)
        gb_signals.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        signals_grid = QGridLayout(gb_signals)
        signals_grid.setContentsMargins(8, 8, 8, 8)
        signals_grid.setSpacing(8)
        signals_grid.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignTop | QtCore.Qt.AlignmentFlag.AlignLeft
        )

        # Subbox 1: Türen
        left_box = QGroupBox("Schutzeinrichtungen / Türen", gb_signals)
        left_box.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        left_layout = QVBoxLayout(left_box)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(4)

        for key, text in [
            ("estop", "Notaus"),
            ("cabinet_temp_ok", "Temperatur im Schaltschrank OK"),
            ("exhaust_ok", "Absaugung vorhanden"),
            ("door_left_closed", "Schutztür links geschlossen"),
            ("door_left_locked", "Schutztür links verriegelt"),
            ("door_left_front_closed", "Schutztür links vorn geschlossen"),
            ("door_right_closed", "Schutztür rechts geschlossen"),
            ("door_right_locked", "Schutztür rechts verriegelt"),
            ("door_right_front_closed", "Schutztür rechts vorn geschlossen"),
        ]:
            cb = self._make_status_row(left_layout, text)
            self._inputs[key] = cb

        signals_grid.addWidget(left_box, 0, 0)

        # Subbox 2: Ampel
        mid_box = QGroupBox("Ampel / Allgemeine Signale", gb_signals)
        mid_box.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        mid_layout = QVBoxLayout(mid_box)
        mid_layout.setContentsMargins(8, 8, 8, 8)
        mid_layout.setSpacing(4)

        for key, text in [
            ("lamp_red", "Ampel Rot"),
            ("lamp_yellow", "Ampel Gelb"),
            ("lamp_green", "Ampel Grün"),
            ("lamp_blue", "Ampel Blau"),
            ("lamp_white", "Ampel Weiss"),
            ("horn", "Horn"),
        ]:
            cb = self._make_status_row(mid_layout, text)
            self._inputs[key] = cb

        signals_grid.addWidget(mid_box, 0, 1)

        # Subbox 3: Prozess/Vakuum
        right_box = QGroupBox("Ladeschublade / Vakuum / Prozess", gb_signals)
        right_box.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        right_layout = QVBoxLayout(right_box)
        right_layout.setContentsMargins(8, 8, 8, 8)
        right_layout.setSpacing(4)

        for key, text in [
            ("tray_locked", "Ladeschublade verriegelt"),
            ("tray_unlocked", "Ladeschublade entriegelt"),
            ("tray_closed", "Ladeschublade geschlossen"),
            ("tray_open", "Ladeschublade geöffnet"),
            ("tray_safe", "Ladeschublade sicher"),
            ("vacuum_present", "Haltevakuum vorhanden"),
            ("vacuum_on", "Haltevakuum ein"),
            ("n2_pressure_ok", "N2 Druck erreicht"),
            ("n2_nozzle_active", "Stickstoff Sprühdüse"),
            ("ultrasonic_error", "Ultraschall Generatorfehler"),
            ("ultrasonic_on", "Ultraschall an"),
            ("syringe_running", "Kartusche Start"),
        ]:
            cb = self._make_status_row(right_layout, text)
            self._inputs[key] = cb

        signals_grid.addWidget(right_box, 0, 2)

        # Signals-Spacer: rechts + unten
        signals_grid.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum),
            0, 3
        )
        signals_grid.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding),
            1, 0, 1, 4
        )
        signals_grid.setColumnStretch(0, 0)
        signals_grid.setColumnStretch(1, 0)
        signals_grid.setColumnStretch(2, 0)
        signals_grid.setColumnStretch(3, 1)
        signals_grid.setRowStretch(0, 0)
        signals_grid.setRowStretch(1, 1)

        # ===== Syringe GroupBox (rechts) =====
        gb_syringe = QGroupBox("Syringe", self)
        gb_syringe.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        syringe_grid = QGridLayout(gb_syringe)
        syringe_grid.setContentsMargins(8, 8, 8, 8)
        syringe_grid.setSpacing(8)
        syringe_grid.setAlignment(
            QtCore.Qt.AlignmentFlag.AlignTop | QtCore.Qt.AlignmentFlag.AlignLeft
        )

        # Status box
        status_box = QGroupBox("Status", gb_syringe)
        status_box.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        status_layout = QVBoxLayout(status_box)
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setSpacing(6)

        row_running = QHBoxLayout()
        row_running.setSpacing(6)
        lbl_running = QLabel("Kartusche läuft:", gb_syringe)
        chk_running = QCheckBox(gb_syringe)
        chk_running.setEnabled(False)
        row_running.addWidget(lbl_running)
        row_running.addStretch(1)
        row_running.addWidget(chk_running)
        status_layout.addLayout(row_running)
        self._chk_running = chk_running

        lbl_alarm_title = QLabel("Alarm:", gb_syringe)
        lbl_alarm = QLabel("-", gb_syringe)
        status_layout.addWidget(lbl_alarm_title)
        status_layout.addWidget(lbl_alarm)
        self._lbl_alarm = lbl_alarm

        syringe_grid.addWidget(status_box, 0, 0)

        # Parameter box
        param_box = QGroupBox("Parameter", gb_syringe)
        param_box.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        param_layout = QVBoxLayout(param_box)
        param_layout.setContentsMargins(8, 8, 8, 8)
        param_layout.setSpacing(6)

        row_speed = QHBoxLayout()
        row_speed.setSpacing(6)
        lbl_speed = QLabel("Geschwindigkeit [mm/s]:", gb_syringe)
        speed_spin = QDoubleSpinBox(gb_syringe)
        speed_spin.setRange(0.1, 100.0)
        speed_spin.setSingleStep(0.1)
        speed_spin.setDecimals(2)
        speed_spin.setValue(10.0)
        row_speed.addWidget(lbl_speed)
        row_speed.addStretch(1)
        row_speed.addWidget(speed_spin)
        param_layout.addLayout(row_speed)

        self._speed_spin = speed_spin
        syringe_grid.addWidget(param_box, 0, 1)

        # Steuerung box
        ctrl_box = QGroupBox("Steuerung", gb_syringe)
        ctrl_box.setSizePolicy(QSizePolicy.Policy.Maximum, QSizePolicy.Policy.Maximum)
        ctrl_layout = QVBoxLayout(ctrl_box)
        ctrl_layout.setContentsMargins(8, 8, 8, 8)
        ctrl_layout.setSpacing(6)

        row_buttons = QHBoxLayout()
        row_buttons.setSpacing(6)
        btn_start = QPushButton("Start", gb_syringe)
        btn_stop = QPushButton("Stop", gb_syringe)
        btn_purge = QPushButton("Spülen", gb_syringe)
        row_buttons.addWidget(btn_start)
        row_buttons.addWidget(btn_stop)
        row_buttons.addWidget(btn_purge)
        ctrl_layout.addLayout(row_buttons)

        syringe_grid.addWidget(ctrl_box, 1, 0, 1, 2)

        # Syringe-Spacer: rechts + unten
        syringe_grid.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum),
            0, 2
        )
        syringe_grid.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding),
            2, 0, 1, 3
        )
        syringe_grid.setColumnStretch(0, 0)
        syringe_grid.setColumnStretch(1, 0)
        syringe_grid.setColumnStretch(2, 1)
        syringe_grid.setRowStretch(0, 0)
        syringe_grid.setRowStretch(1, 0)
        syringe_grid.setRowStretch(2, 1)

        # Root: nebeneinander + Root-Spacer
        root.addWidget(gb_signals, 0, 0)
        root.addWidget(gb_syringe, 0, 1)

        # Root spacer rechts / unten (falls MainWindow riesig wird)
        root.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum),
            0, 2
        )
        root.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding),
            1, 0, 1, 3
        )

        root.setColumnStretch(0, 0)
        root.setColumnStretch(1, 0)
        root.setColumnStretch(2, 1)
        root.setRowStretch(0, 0)
        root.setRowStretch(1, 1)

        # UI signals
        btn_start.clicked.connect(self._on_start_clicked)
        btn_stop.clicked.connect(self._on_stop_clicked)
        btn_purge.clicked.connect(self._on_purge_clicked)
        speed_spin.valueChanged.connect(self._on_speed_changed)

    def _make_status_row(self, parent_layout: QVBoxLayout, text: str) -> QCheckBox:
        row = QHBoxLayout()
        row.setSpacing(6)

        lbl = QLabel(text, self)
        cb = QCheckBox(self)
        cb.setEnabled(False)

        row.addWidget(lbl)
        row.addStretch(1)
        row.addWidget(cb)
        parent_layout.addLayout(row)
        return cb

    # ------------------------------------------------------------------
    # PLC init (one-time read)
    # ------------------------------------------------------------------
    def _init_plc(self) -> None:
        plc = self._plc
        if plc is None or not getattr(plc, "is_connected", False):
            LOG.info("PlcTab: PLC nicht vorhanden oder nicht verbunden.")
            self._apply_status_to_ui(PlcStatus(connected=False))
            self._ui(lambda: (self._set_syringe_running(False), self._set_syringe_alarm_text("")))
            return

        # Signals: Status snapshot
        try:
            st = plc.read_status()
        except Exception as e:
            LOG.error("PlcTab: read_status() failed: %s", e)
            st = PlcStatus(connected=False)

        self._apply_status_to_ui(st)

        # Syringe: optional snapshot (falls im Status enthalten)
        def _maybe_syringe_from_status() -> None:
            running = bool(getattr(st, "syringe_running", False))
            alarm = int(getattr(st, "syringe_alarm_code", 0) or 0)
            self._set_syringe_running(running)
            self._set_syringe_alarm_text(f"Alarmcode: {alarm}" if alarm else "")

        self._ui(_maybe_syringe_from_status)

    # ------------------------------------------------------------------
    # PLC callback registration (signals + syringe)
    # ------------------------------------------------------------------
    def _add_callbacks(self) -> None:
        plc = self._plc
        if plc is None or not getattr(plc, "is_connected", False):
            return

        try:
            # Signals
            plc.register_bool_callback("process_active", self._cb_process_active)
            plc.register_bool_callback("process_done", self._cb_process_done)
            plc.register_bool_callback("process_error", self._cb_process_error)
            plc.register_bool_callback("interlock_ok", self._cb_interlock_ok)
            plc.register_bool_callback("vacuum_ok", self._cb_vacuum_ok)
            plc.register_int_callback("alarm_code", self._cb_alarm_code)

            # Syringe
            plc.register_bool_callback("syringe_running", self._cb_syringe_running)
            plc.register_int_callback("syringe_alarm_code", self._cb_syringe_alarm_code)

        except Exception as e:
            LOG.error("PlcTab: register_*_callback failed: %s", e)

    # ------------------------------------------------------------------
    # Signals: PlcStatus -> UI
    # ------------------------------------------------------------------
    def _apply_status_to_ui(self, st: PlcStatus) -> None:
        def _do() -> None:
            # 1) Best effort: gleichnamige Felder übernehmen
            for key, cb in (self._inputs or {}).items():
                try:
                    if hasattr(st, key):
                        cb.setChecked(bool(getattr(st, key)))
                except Exception:
                    pass

            # 2) Grobe Mapping-Logik
            self._set_checked("vacuum_present", bool(getattr(st, "vacuum_ok", False)))
            self._set_checked("vacuum_on", bool(getattr(st, "vacuum_ok", False)))
            self._set_checked("tray_safe", bool(getattr(st, "interlock_ok", False)))

            proc_active = bool(getattr(st, "process_active", False))
            proc_done = bool(getattr(st, "process_done", False))
            proc_error = bool(getattr(st, "process_error", False))
            connected = bool(getattr(st, "connected", False))

            self._set_checked("lamp_green", proc_active and not proc_error)
            self._set_checked("lamp_red", proc_error)
            self._set_checked("lamp_yellow", proc_done)
            self._set_checked("lamp_white", connected)

            alarm_code = int(getattr(st, "alarm_code", 0) or 0)
            txt = f"Alarmcode: {alarm_code}" if alarm_code else ""
            for key in ("lamp_red", "horn"):
                cb = self._inputs.get(key)
                if cb:
                    cb.setToolTip(txt)

        self._ui(_do)

    def _set_checked(self, key: str, value: bool) -> None:
        cb = self._inputs.get(key)
        if cb is not None:
            cb.setChecked(bool(value))

    # ------------------------------------------------------------------
    # Signals callbacks
    # ------------------------------------------------------------------
    def _cb_vacuum_ok(self, value: bool) -> None:
        self._ui(lambda: (
            self._set_checked("vacuum_present", bool(value)),
            self._set_checked("vacuum_on", bool(value)),
        ))

    def _cb_interlock_ok(self, value: bool) -> None:
        self._ui(lambda: self._set_checked("tray_safe", bool(value)))

    def _cb_process_active(self, value: bool) -> None:
        self._ui(lambda: self._set_checked("lamp_green", bool(value)))

    def _cb_process_done(self, value: bool) -> None:
        self._ui(lambda: self._set_checked("lamp_yellow", bool(value)))

    def _cb_process_error(self, value: bool) -> None:
        def _do() -> None:
            self._set_checked("lamp_red", bool(value))
            if value:
                self._set_checked("horn", True)
        self._ui(_do)

    def _cb_alarm_code(self, code: int) -> None:
        txt = f"Alarmcode: {int(code)}" if code else ""
        def _do() -> None:
            for k in ("lamp_red", "horn"):
                cb = self._inputs.get(k)
                if cb is not None:
                    cb.setToolTip(txt)
        self._ui(_do)

    # ------------------------------------------------------------------
    # Syringe UI helpers
    # ------------------------------------------------------------------
    def _set_syringe_running(self, running: bool) -> None:
        if self._chk_running is not None:
            self._chk_running.setChecked(bool(running))

    def _set_syringe_alarm_text(self, text: str) -> None:
        if self._lbl_alarm is not None:
            self._lbl_alarm.setText(text if text else "-")

    # ------------------------------------------------------------------
    # Syringe callbacks
    # ------------------------------------------------------------------
    def _cb_syringe_running(self, value: bool) -> None:
        self._ui(lambda: self._set_syringe_running(bool(value)))

    def _cb_syringe_alarm_code(self, code: int) -> None:
        c = int(code) if code else 0
        text = f"Alarmcode: {c}" if c else ""
        self._ui(lambda: self._set_syringe_alarm_text(text))

    # ------------------------------------------------------------------
    # Buttons -> outward signals
    # ------------------------------------------------------------------
    def _on_start_clicked(self) -> None:
        LOG.info("PlcTab: Syringe Start angefordert")
        self.start_syringe_requested.emit()

    def _on_stop_clicked(self) -> None:
        LOG.info("PlcTab: Syringe Stop angefordert")
        self.stop_syringe_requested.emit()

    def _on_purge_clicked(self) -> None:
        LOG.info("PlcTab: Syringe Spülen angefordert")
        self.purge_syringe_requested.emit()

    def _on_speed_changed(self, value: float) -> None:
        v = float(value)
        LOG.info("PlcTab: Syringe Speed gesetzt: %.3f mm/s", v)
        self.set_speed_requested.emit(v)
