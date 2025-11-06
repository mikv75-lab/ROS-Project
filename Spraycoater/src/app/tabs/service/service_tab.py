# -*- coding: utf-8 -*-
# ServiceTab (signal-basiert) – robustes Signal-Finding (scene.signals | _sb.signals | scene_signals)
from __future__ import annotations
import os
import logging
from typing import Any, Optional
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
    Service-Tab mit 'Scene'-Gruppe (signal-basiert):
      - Combos für Cage/Mount/Substrate (gefüllt über *_ListChanged)
      - Labels für *_CurrentChanged (ACK)
      - Set/Clear-Buttons emittieren set*Requested (UI -> Bridge -> ROS)
      - Fallback: Wenn keine Signale gefunden werden, initialer Pull aus SceneState
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

        # Scene-Bindings
        self._wire_scene_signals()

    # ---------- Scene: Signal-Wiring ----------
    def _wire_scene_signals(self) -> None:
        sig = self._find_scene_signals()

        if sig is None:
            # Kein Crash mehr: initialen Zustand aus SceneState ziehen (read-only Anzeige)
            _LOG.error("Keine Scene-Signale gefunden – nutze SceneState-Fallback (UI bleibt read-only).")
            state = getattr(self.bridge, "scene", None)  # SceneState
            if state is not None:
                try:
                    self._fill_combo(self.cmbCage, state.cage_list())
                    self._fill_combo(self.cmbMount, state.mount_list())
                    self._fill_combo(self.cmbSubstrate, state.substrate_list())

                    self._set_current(self.lblCageCurrent, self.cmbCage, state.cage_current())
                    self._set_current(self.lblMountCurrent, self.cmbMount, state.mount_current())
                    self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, state.substrate_current())
                except Exception as e:
                    _LOG.exception("SceneState-Fallback fehlgeschlagen: %s", e)
            # Buttons deaktivieren, weil kein set*Requested möglich
            for b in (self.btnSetCage, self.btnSetMount, self.btnSetSubstrate,
                      self.btnClearCage, self.btnClearMount, self.btnClearSubstrate):
                if b:
                    b.setEnabled(False)
            return

        # --- Prüfen, ob die erwarteten Signale existieren ---
        required_inbound = [
            "cageListChanged", "mountListChanged", "substrateListChanged",
            "cageCurrentChanged", "mountCurrentChanged", "substrateCurrentChanged",
        ]
        required_outbound = ["setCageRequested", "setMountRequested", "setSubstrateRequested"]

        missing = [name for name in required_inbound + required_outbound if not hasattr(sig, name)]
        if missing:
            _LOG.error("Scene-Signale fehlen: %s", ", ".join(missing))
            # wie oben: read-only Fallback
            state = getattr(self.bridge, "scene", None)
            if state is not None:
                try:
                    self._fill_combo(self.cmbCage, state.cage_list())
                    self._fill_combo(self.cmbMount, state.mount_list())
                    self._fill_combo(self.cmbSubstrate, state.substrate_list())

                    self._set_current(self.lblCageCurrent, self.cmbCage, state.cage_current())
                    self._set_current(self.lblMountCurrent, self.cmbMount, state.mount_current())
                    self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, state.substrate_current())
                except Exception:
                    _LOG.exception("SceneState-Fallback fehlgeschlagen")
            for b in (self.btnSetCage, self.btnSetMount, self.btnSetSubstrate,
                      self.btnClearCage, self.btnClearMount, self.btnClearSubstrate):
                if b:
                    b.setEnabled(False)
            return

        # --- ROS -> UI: Listen + Currents ---
        sig.cageListChanged.connect(lambda items: self._fill_combo(self.cmbCage, items))
        sig.mountListChanged.connect(lambda items: self._fill_combo(self.cmbMount, items))
        sig.substrateListChanged.connect(lambda items: self._fill_combo(self.cmbSubstrate, items))

        sig.cageCurrentChanged.connect(lambda v: self._set_current(self.lblCageCurrent, self.cmbCage, v))
        sig.mountCurrentChanged.connect(lambda v: self._set_current(self.lblMountCurrent, self.cmbMount, v))
        sig.substrateCurrentChanged.connect(lambda v: self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, v))

        # --- UI -> ROS: Set/Clear ---
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

        # --- Initial Pull: vorhandene Cache-Werte der Signals (oder SceneState) ziehen ---
        # Viele Bridge-Implementierungen legen die letzten Werte als Attribute auf 'signals' ab.
        # Das nutzt du ja bereits in SceneBridge.__init__().
        try:
            if hasattr(sig, "cage_list"):       self._fill_combo(self.cmbCage, getattr(sig, "cage_list"))
            if hasattr(sig, "mount_list"):      self._fill_combo(self.cmbMount, getattr(sig, "mount_list"))
            if hasattr(sig, "substrate_list"):  self._fill_combo(self.cmbSubstrate, getattr(sig, "substrate_list"))

            if hasattr(sig, "cage_current"):       self._set_current(self.lblCageCurrent, self.cmbCage, getattr(sig, "cage_current"))
            if hasattr(sig, "mount_current"):      self._set_current(self.lblMountCurrent, self.cmbMount, getattr(sig, "mount_current"))
            if hasattr(sig, "substrate_current"):  self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, getattr(sig, "substrate_current"))
        except Exception as e:
            _LOG.warning("Initial Pull aus signals fehlgeschlagen: %s", e)

        # Falls die Signals noch leer sind, versuche SceneState
        try:
            state = getattr(self.bridge, "scene", None)
            if state:
                if self.cmbCage.count() == 0:        self._fill_combo(self.cmbCage, state.cage_list())
                if self.cmbMount.count() == 0:       self._fill_combo(self.cmbMount, state.mount_list())
                if self.cmbSubstrate.count() == 0:   self._fill_combo(self.cmbSubstrate, state.substrate_list())

                if not self.lblCageCurrent.text() or self.lblCageCurrent.text() == "-":
                    self._set_current(self.lblCageCurrent, self.cmbCage, state.cage_current())
                if not self.lblMountCurrent.text() or self.lblMountCurrent.text() == "-":
                    self._set_current(self.lblMountCurrent, self.cmbMount, state.mount_current())
                if not self.lblSubstrateCurrent.text() or self.lblSubstrateCurrent.text() == "-":
                    self._set_current(self.lblSubstrateCurrent, self.cmbSubstrate, state.substrate_current())
        except Exception:
            _LOG.debug("SceneState-Nachzug nicht nötig oder fehlgeschlagen.", exc_info=True)

    def _find_scene_signals(self) -> Optional[Any]:
        """
        Sucht die Scene-Signale an bekannten Stellen:
          1) bridge.scene.signals
          2) bridge._sb.signals
          3) bridge.scene_signals (falls du das später als Property anbietest)
        """
        # 1) scene.signals
        scene = getattr(self.bridge, "scene", None)
        if scene is not None:
            sig = getattr(scene, "signals", None)
            if sig is not None:
                return sig

        # 2) _sb.signals (direkter Zugriff auf die SceneBridge)
        sb = getattr(self.bridge, "_sb", None)
        if sb is not None:
            sig = getattr(sb, "signals", None)
            if sig is not None:
                return sig

        # 3) optionale Property
        sig = getattr(self.bridge, "scene_signals", None)
        if sig is not None:
            return sig

        return None

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
