# -*- coding: utf-8 -*-
# ServiceTab – nur echte Bridge-Signale (keine Fallbacks, keine Cache-Pulls)
from __future__ import annotations
import os
import logging
from typing import Iterable, Tuple
from math import atan2, asin
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget, QLabel, QComboBox, QPushButton
from geometry_msgs.msg import PoseStamped

_LOG = logging.getLogger("app.tabs.service")

# -------- Helpers: paths --------

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "service", filename)

# -------- Helpers: pose formatting --------

def _quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Quaternion -> RPY (rad) (ZYX-Konvention)."""
    # Roll (x)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)
    # Pitch (y)
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        from math import pi
        pitch = (sinp / abs(sinp)) * (pi / 2.0)
    else:
        pitch = asin(sinp)
    # Yaw (z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def _pose_stamped_to_pose6(ps: PoseStamped) -> Tuple[float, float, float, float, float, float]:
    p = ps.pose.position
    q = ps.pose.orientation
    r, pch, y = _quat_to_rpy(q.x, q.y, q.z, q.w)
    return (float(p.x), float(p.y), float(p.z), float(r), float(pch), float(y))

def _fmt_pose6(pose: Iterable[float] | Tuple[float, float, float, float, float, float]) -> str:
    """Formatiert 6D-Pose (x,y,z,rx,ry,rz) knapp für die Anzeige."""
    try:
        x, y, z, rx, ry, rz = pose  # type: ignore[misc]
        return f"{x:.3f}  {y:.3f}  {z:.3f}   {rx:.3f}  {ry:.3f}  {rz:.3f}"
    except Exception:
        return "-"

# -------- UI --------

class ServiceTab(QWidget):
    """
    Service-Tab mit 'Scene' + 'Poses'-Gruppe:
      - bindet ausschließlich an bridge._sb.signals (SceneBridge.signals)
        und bridge._pb.signals (PosesBridge.signals)
      - UI reagiert nur auf eingehende Signals
      - Set-Buttons emittieren die Outbound-Signale (setHomeRequested/setServiceRequested)
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

    # ---------- Scene ----------
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

    # ---------- Poses ----------
    def _wire_poses_signals(self) -> None:
        pb = getattr(self.bridge, "_pb", None)
        if pb is None or not hasattr(pb, "signals") or pb.signals is None:
            _LOG.error("PosesBridge-Signale nicht verfügbar (bridge._pb.signals fehlt).")
            return
        psig = pb.signals  # PosesSignals

        # Inbound (ROS -> UI): kompakte 6D-Anzeige
        psig.homePoseChanged.connect(lambda ps: self._set_pose_label_from_ps(self.lblHomePose, ps))
        psig.servicePoseChanged.connect(lambda ps: self._set_pose_label_from_ps(self.lblServicePose, ps))

        # Outbound (UI -> ROS): getrennte Buttons
        if self.btnSetHome:
            self.btnSetHome.clicked.connect(psig.setHomeRequested.emit)
        if self.btnSetService:
            self.btnSetService.clicked.connect(psig.setServiceRequested.emit)

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
    def _set_pose_label_from_ps(label: QLabel, ps: PoseStamped) -> None:
        try:
            pose6 = _pose_stamped_to_pose6(ps)
            label.setText(_fmt_pose6(pose6))
        except Exception as e:
            _LOG.error("PoseStamped -> label format failed: %s", e)
            label.setText("-")
