# -*- coding: utf-8 -*-
# src/app/tabs/service/service_signals_tab.py
from __future__ import annotations

import logging
from typing import Any, Optional, Dict

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QGridLayout, QGroupBox, QVBoxLayout, QHBoxLayout,
    QLabel, QCheckBox,
)

from plc.plc_client import PlcClientBase, PlcStatus

LOG = logging.getLogger(__name__)


class ServiceSignalsTab(QWidget):
    """
    Service-Tab für SPS-Signale / Interlocks.

    WICHTIG:
      PLC-Callbacks kommen je nach Implementierung ggf. aus einem Worker-Thread.
      UI-Updates müssen dann in den Qt-GUI-Thread gehoppt werden -> QTimer.singleShot(0, ...).
    """

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

        # lifecycle guard
        self._dying: bool = False
        self.destroyed.connect(lambda *_: setattr(self, "_dying", True))

        # key -> Checkbox (nur Anzeige)
        self._inputs: Dict[str, QCheckBox] = {}

        self._build_ui()
        self._init_plc()
        self._add_callbacks()

    # ------------------------------------------------------------------
    # PLC-Initialisierung (einmaliger Read)
    # ------------------------------------------------------------------
    def _init_plc(self) -> None:
        plc = self._plc
        if plc is None or not getattr(plc, "is_connected", False):
            LOG.info("ServiceSignalsTab: PLC nicht vorhanden oder nicht verbunden.")
            self._apply_status_to_ui(PlcStatus(connected=False))
            return

        try:
            st = plc.read_status()
        except Exception as e:
            LOG.error("ServiceSignalsTab: read_status() failed: %s", e)
            st = PlcStatus(connected=False)

        self._apply_status_to_ui(st)

    # ------------------------------------------------------------------
    # Callback-Registrierung
    # ------------------------------------------------------------------
    def _add_callbacks(self) -> None:
        plc = self._plc
        if plc is None or not getattr(plc, "is_connected", False):
            return

        try:
            plc.register_bool_callback("process_active", self._cb_process_active)
            plc.register_bool_callback("process_done", self._cb_process_done)
            plc.register_bool_callback("process_error", self._cb_process_error)
            plc.register_bool_callback("interlock_ok", self._cb_interlock_ok)
            plc.register_bool_callback("vacuum_ok", self._cb_vacuum_ok)
            plc.register_int_callback("alarm_code", self._cb_alarm_code)
        except Exception as e:
            LOG.error("ServiceSignalsTab: register_*_callback failed: %s", e)

    # ------------------------------------------------------------------
    # UI-Aufbau
    # ------------------------------------------------------------------
    def _build_ui(self) -> None:
        grid = QGridLayout(self)

        # ---------------- LINKS: Schutzeinrichtungen / Türen -------------
        left_box = QGroupBox("Schutzeinrichtungen / Türen", self)
        left_layout = QVBoxLayout(left_box)

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

        grid.addWidget(left_box, 0, 0)

        # ---------------- MITTE: Ampel / Allgemeine Signale --------------
        mid_box = QGroupBox("Ampel / Allgemeine Signale", self)
        mid_layout = QVBoxLayout(mid_box)

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

        grid.addWidget(mid_box, 0, 1)

        # -------------- RECHTS: Ladeschublade / Vakuum / Prozess ---------
        right_box = QGroupBox("Ladeschublade / Vakuum / Prozess", self)
        right_layout = QVBoxLayout(right_box)

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

        grid.addWidget(right_box, 0, 2)

    def _make_status_row(self, parent_layout: QVBoxLayout, text: str) -> QCheckBox:
        row = QHBoxLayout()
        lbl = QLabel(text, self)
        cb = QCheckBox(self)
        cb.setEnabled(False)  # reine Anzeige
        row.addWidget(lbl)
        row.addStretch(1)
        row.addWidget(cb)
        parent_layout.addLayout(row)
        return cb

    # ------------------------------------------------------------------
    # Thread-safe UI update helper
    # ------------------------------------------------------------------
    def _ui(self, fn) -> None:
        """Hoppt sicher in den GUI-Thread (und ignoriert Updates beim Zerstören)."""
        if self._dying:
            return
        QtCore.QTimer.singleShot(0, lambda: (None if self._dying else fn()))

    # ------------------------------------------------------------------
    # PlcStatus → UI (initial)
    # ------------------------------------------------------------------
    def _apply_status_to_ui(self, st: PlcStatus) -> None:
        def _do():
            # 1) Best effort: alle gleichnamigen Felder übernehmen (wenn PlcStatus sie hat)
            for key, cb in (self._inputs or {}).items():
                try:
                    if hasattr(st, key):
                        cb.setChecked(bool(getattr(st, key)))
                except Exception:
                    pass

            # 2) Grobe Mapping-Logik (wie vorher)
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
    # Callbacks vom PLC (pyads-Notifications → Tab-Callbacks)
    # ------------------------------------------------------------------
    def _cb_vacuum_ok(self, value: bool) -> None:
        self._ui(lambda: (
            self._set_checked("vacuum_present", value),
            self._set_checked("vacuum_on", value),
        ))

    def _cb_interlock_ok(self, value: bool) -> None:
        self._ui(lambda: self._set_checked("tray_safe", value))

    def _cb_process_active(self, value: bool) -> None:
        self._ui(lambda: self._set_checked("lamp_green", value))

    def _cb_process_done(self, value: bool) -> None:
        self._ui(lambda: self._set_checked("lamp_yellow", value))

    def _cb_process_error(self, value: bool) -> None:
        def _do():
            self._set_checked("lamp_red", value)
            if value:
                self._set_checked("horn", True)
        self._ui(_do)

    def _cb_alarm_code(self, code: int) -> None:
        txt = f"Alarmcode: {int(code)}" if code else ""
        self._ui(lambda: [self._inputs[k].setToolTip(txt) for k in ("lamp_red", "horn") if k in self._inputs])
