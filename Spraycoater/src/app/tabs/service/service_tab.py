# -*- coding: utf-8 -*-
# ServiceTab – nur echte Bridge-Signale (keine Fallbacks, keine Cache-Pulls)
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


class ServiceTab(QWidget):
    """
    Service-Tab mit 'Scene'-Gruppe:
      - bindet ausschließlich an bridge._sb.signals (SceneBridge.signals)
      - keine Fallbacks, keine Initial-Befüllung aus Cache
      - Combos/Labels reagieren nur auf eingehende Signals
      - Set/Clear-Buttons emittieren set*Requested
    """

    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("service_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("ServiceTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)

        # Widgets
        self.cmbCage: QComboBox = self.findChild(QComboBox, "cmbCage")
        self.cmbMount: QComboBox = self.findChild(QComboBox, "cmbMount")
        self.cmbSubstrate: QComboBox = self.findChild(QComboBox, "cmbSubstrate")

        self.lblCageCurrent: QLabel = self.findChild(QLabel, "lblCageCurrent")
        self.lblMountCurrent: QLabel = self.findChild(QLabel, "lblMountCurrent")
        self.lblSubstrateCurrent: QLabel = self.findChild(QLabel, "lblSubstrateCurrent")

        self.btnSetCage: QPushButton = self.findChild(QPushButton, "btnSetCage")
        self.btnSetMount: QPushButton = self.findChild(QPushButton, "btnSetMount")
        self.btnSetSubstrate: QPushButton = self.findChild(QPushButton, "btnSetSubstrate")

        self.btnClearCage: QPushButton = self.findChild(QPushButton, "btnClearCage")
        self.btnClearMount: QPushButton = self.findChild(QPushButton, "btnClearMount")
        self.btnClearSubstrate: QPushButton = self.findChild(QPushButton, "btnClearSubstrate")

        self._wire_scene_signals()

    # ---------- Scene: Signal-Wiring (nur echte Signals) ----------
    def _wire_scene_signals(self) -> None:
        # NUR echte SceneBridge.signals benutzen
        sb = getattr(self.bridge, "_sb", None)
        if sb is None or not hasattr(sb, "signals") or sb.signals is None:
            _LOG.error("SceneBridge-Signale nicht verfügbar (bridge._sb.signals fehlt).")
            return
        sig = sb.signals  # SceneSignals aus SceneBridge

        # Erwartete Signal-Namen (hart)
        # Inbound (ROS -> UI)
        sig.cageListChanged.connect(lambda items: self._fill_combo(self.cmbCage, items))
        sig.mountListChanged.connect(lambda items: self._fill_combo(self.cmbMount, items))
        sig.substrateListChanged.connect(lambda items: self._fill_combo(self.cmbSubstrate, items))

        sig.cageCurrentChanged.connect(lambda v: self._set_current(self.lblCageCurrent, self.cmbCage, v))
        sig.mountCurrentChanged.connect(lambda v: self._set_current(self.lblMountCurrent, self.cmbMount, v))
        sig.substrateCurrentChanged.connect(lambda v: self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, v))

        # Outbound (UI -> ROS)
        if self.btnSetCage:
            self.btnSetCage.clicked.connect(lambda: sig.setCageRequested.emit(self.cmbCage.currentText().strip()))
        if self.btnSetMount:
            self.btnSetMount.clicked.connect(lambda: sig.setMountRequested.emit(self.cmbMount.currentText().strip()))
        if self.btnSetSubstrate:
            self.btnSetSubstrate.clicked.connect(lambda: sig.setSubstrateRequested.emit(self.cmbSubstrate.currentText().strip()))

        if self.btnClearCage:
            self.btnClearCage.clicked.connect(lambda: sig.setCageRequested.emit(""))
        if self.btnClearMount:
            self.btnClearMount.clicked.connect(lambda: sig.setMountRequested.emit(""))
        if self.btnClearSubstrate:
            self.btnClearSubstrate.clicked.connect(lambda: sig.setSubstrateRequested.emit(""))

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
