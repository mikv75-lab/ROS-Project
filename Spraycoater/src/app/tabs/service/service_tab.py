# -*- coding: utf-8 -*-
# ServiceTab – nur echte Bridge-Signale (keine Fallbacks, keine Cache-Pulls)
from __future__ import annotations
import os
import logging
from typing import Iterable, Tuple
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QLabel, QComboBox, QPushButton

_LOG = logging.getLogger("app.tabs.service")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "service", filename)


def _fmt_pose6(pose: Iterable[float] | Tuple[float, float, float, float, float, float]) -> str:
    """
    Formatiert eine 6D-Pose (x,y,z,rx,ry,rz) knapp für die Anzeige.
    Erwartet SI-Einheiten (m, rad) oder bereits mm/deg – wir formatieren nur numerisch.
    """
    try:
        x, y, z, rx, ry, rz = pose  # type: ignore[misc]
        return f"{x:.3f}  {y:.3f}  {z:.3f}   {rx:.3f}  {ry:.3f}  {rz:.3f}"
    except Exception:
        return "-"  # wenn leer/ungültig, strikt leer anzeigen


class ServiceTab(QWidget):
    """
    Service-Tab mit 'Scene' + 'Poses'-Gruppe:
      - bindet ausschließlich an bridge._sb.signals (SceneBridge.signals)
        und bridge._pb.signals (PosesBridge.signals)
      - keine Fallbacks, keine Initial-Befüllung aus Cache
      - UI reagiert nur auf eingehende Signals
      - Set-Buttons emittieren die Outbound-Signale der Bridges
    """

    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("service_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("ServiceTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)

        # ---------- Scene-Widgets ----------
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

        # ---------- Poses-Widgets ----------
        self.lblHomePose: QLabel = self.findChild(QLabel, "lblHomePose")
        self.lblServicePose: QLabel = self.findChild(QLabel, "lblServicePose")
        self.btnSetHome: QPushButton = self.findChild(QPushButton, "btnSetHome")
        self.btnSetService: QPushButton = self.findChild(QPushButton, "btnSetService")

        # Wiring
        self._wire_scene_signals()
        self._wire_poses_signals()

    # ---------- Scene: Signal-Wiring (nur echte Signals) ----------
    def _wire_scene_signals(self) -> None:
        sb = getattr(self.bridge, "_sb", None)
        if sb is None or not hasattr(sb, "signals") or sb.signals is None:
            _LOG.error("SceneBridge-Signale nicht verfügbar (bridge._sb.signals fehlt).")
            return
        sig = sb.signals  # SceneSignals

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

    # ---------- Poses: Signal-Wiring (nur echte Signals) ----------
    def _wire_poses_signals(self) -> None:
        pb = getattr(self.bridge, "_pb", None)
        if pb is None or not hasattr(pb, "signals") or pb.signals is None:
            _LOG.error("PosesBridge-Signale nicht verfügbar (bridge._pb.signals fehlt).")
            return
        psig = pb.signals  # PosesSignals

        # Inbound (ROS -> UI): Anzeigen als kompakte 6D-Pose
        psig.homePoseChanged.connect(lambda pose: self._set_pose_label(self.lblHomePose, pose))
        psig.servicePoseChanged.connect(lambda pose: self._set_pose_label(self.lblServicePose, pose))

        # Outbound (UI -> ROS): nur der Name wird übertragen; Node nimmt aktuelle Roboterpose
        if self.btnSetHome:
            self.btnSetHome.clicked.connect(lambda: psig.poseSetRequested.emit("home"))
        if self.btnSetService:
            self.btnSetService.clicked.connect(lambda: psig.poseSetRequested.emit("service"))

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

    @staticmethod
    def _set_pose_label(label: QLabel, pose: Iterable[float] | Tuple[float, float, float, float, float, float]) -> None:
        label.setText(_fmt_pose6(pose))
