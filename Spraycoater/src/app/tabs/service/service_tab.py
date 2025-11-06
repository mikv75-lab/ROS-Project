# -*- coding: utf-8 -*-
# ServiceTab (signal-basiert) – nutzt bridge.scene.* Signals und setzt via set*Requested
from __future__ import annotations
import os
import logging
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QLabel, QComboBox, QPushButton

_LOG = logging.getLogger("app.tabs.service")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "service", filename)


def _rviz_cfg_path() -> str:
    return os.path.join(_project_root(), "resource", "rviz", "live.rviz")


class ServiceTab(QWidget):
    """
    Service-Tab mit 'Scene'-Gruppe (signal-basiert):
      - Combos für Cage/Mount/Substrate (gefüllt über *_ListChanged)
      - Labels für jeweils *_CurrentChanged (ACK)
      - Set-Buttons emittieren set*Cage/Mount/SubstrateRequested (UI -> Bridge -> ROS)
    """

    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("service_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("ServiceTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)

        # Widgets holen
        self.lblRvizHint: QLabel = self.findChild(QLabel, "lblRvizHint")

        self.cmbCage: QComboBox = self.findChild(QComboBox, "cmbCage")
        self.cmbMount: QComboBox = self.findChild(QComboBox, "cmbMount")
        self.cmbSubstrate: QComboBox = self.findChild(QComboBox, "cmbSubstrate")

        self.lblCageCurrent: QLabel = self.findChild(QLabel, "lblCageCurrent")
        self.lblMountCurrent: QLabel = self.findChild(QLabel, "lblMountCurrent")
        self.lblSubstrateCurrent: QLabel = self.findChild(QLabel, "lblSubstrateCurrent")

        self.btnSetCage: QPushButton = self.findChild(QPushButton, "btnSetCage")
        self.btnSetMount: QPushButton = self.findChild(QPushButton, "btnSetMount")
        self.btnSetSubstrate: QPushButton = self.findChild(QPushButton, "btnSetSubstrate")

        # RViz-Hinweis
        self._cfg = _rviz_cfg_path()
        self._show_config_in_container()

        # Scene-Bindings (reines Signal-Wiring)
        self._wire_scene_signals()

    # ---------- RViz-Hinweis ----------

    def _show_config_in_container(self) -> None:
        label: QLabel = self.lblRvizHint
        if label is None:
            _LOG.warning("lblRvizHint nicht gefunden – nichts anzuzeigen.")
            return

        if os.path.exists(self._cfg):
            label.setText(f"RViz config:\n{self._cfg}")
            label.setStyleSheet("")
        else:
            label.setText(f"RViz config fehlt:\n{self._cfg}")
            label.setStyleSheet("color: #b00020;")

    # ---------- Scene: Signal-Wiring ----------

    def _wire_scene_signals(self) -> None:
        if not self.bridge or not hasattr(self.bridge, "scene"):
            _LOG.error("Bridge hat kein 'scene' Signal-Objekt – bitte ui_bridge prüfen.")
            return

        sig = self.bridge.scene  # = BridgeSignals (Alias)

        # ROS -> UI: Listen füllen
        sig.cageListChanged.connect(lambda items: self._fill_combo(self.cmbCage, items))
        sig.mountListChanged.connect(lambda items: self._fill_combo(self.cmbMount, items))
        sig.substrateListChanged.connect(lambda items: self._fill_combo(self.cmbSubstrate, items))

        # ROS -> UI: Currents anzeigen + Auswahl synchronisieren
        sig.cageCurrentChanged.connect(lambda v: self._set_current(self.lblCageCurrent, self.cmbCage, v))
        sig.mountCurrentChanged.connect(lambda v: self._set_current(self.lblMountCurrent, self.cmbMount, v))
        sig.substrateCurrentChanged.connect(lambda v: self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, v))

        # UI -> ROS: Set-Requests (nur emitten; Bridge published intern)
        if self.btnSetCage:
            self.btnSetCage.clicked.connect(
                lambda: sig.setCageRequested.emit(self.cmbCage.currentText().strip())
            )
        if self.btnSetMount:
            self.btnSetMount.clicked.connect(
                lambda: sig.setMountRequested.emit(self.cmbMount.currentText().strip())
            )
        if self.btnSetSubstrate:
            self.btnSetSubstrate.clicked.connect(
                lambda: sig.setSubstrateRequested.emit(self.cmbSubstrate.currentText().strip())
            )

    # ---------- UI-Helfer ----------

    @staticmethod
    def _fill_combo(combo: QComboBox, items: list[str]) -> None:
        try:
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(items or [])
        finally:
            combo.blockSignals(False)

    @staticmethod
    def _set_current(label: QLabel, combo: QComboBox, value: str) -> None:
        v = (value or "").strip()
        label.setText(v if v else "-")
        if v:
            idx = combo.findText(v)
            if idx >= 0:
                combo.setCurrentIndex(idx)
