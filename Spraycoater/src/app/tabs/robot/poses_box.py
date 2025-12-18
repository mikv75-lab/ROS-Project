# -*- coding: utf-8 -*-
# File: app/widgets/poses_groupbox.py
# PosesGroupBox – besitzt Bridge, verdrahtet sich selbst, hat eigene Qt-Signals
from __future__ import annotations

import logging
from typing import Optional, Tuple

from PyQt6 import QtCore
from PyQt6.QtWidgets import QGroupBox, QGridLayout, QLabel, QPushButton, QWidget

from geometry_msgs.msg import PoseStamped

_LOG = logging.getLogger("app.widgets.poses_groupbox")


class PosesGroupBox(QGroupBox):
    """UI für Home/Service Pose Anzeige + Set-Buttons (verdrahtet direkt zur Bridge)."""

    # Outbound (Widget -> außen/Bridge)
    setHomeRequested = QtCore.pyqtSignal()
    setServiceRequested = QtCore.pyqtSignal()

    def __init__(self, bridge, parent: Optional[QWidget] = None, title: str = "Poses"):
        super().__init__(title, parent)

        # Strikt: Bridge muss ein 'poses_bridge' mit 'signals' liefern
        self.bridge = bridge
        self._pb = self.bridge.poses_bridge
        self._sig = self._pb.signals

        self._build_ui()
        self._wire_inbound()
        self._wire_outbound()

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        g = QGridLayout(self)

        def _bold(txt: str) -> QLabel:
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            return l

        # Header (Tabellenstil)
        g.addWidget(_bold("Component"),             0, 0)
        g.addWidget(_bold("Pose (X Y Z RX RY RZ)"), 0, 1, 1, 2)
        g.addWidget(_bold("Set"),                   0, 3)

        # Home
        g.addWidget(QLabel("Home", self), 1, 0)
        self.lblHome = QLabel("-", self)
        self.lblHome.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        g.addWidget(self.lblHome, 1, 1, 1, 2)
        self.btnHome = QPushButton("Set", self)
        g.addWidget(self.btnHome, 1, 3)

        # Service
        g.addWidget(QLabel("Service", self), 2, 0)
        self.lblService = QLabel("-", self)
        self.lblService.setTextInteractionFlags(QtCore.Qt.TextInteractionFlag.TextSelectableByMouse)
        g.addWidget(self.lblService, 2, 1, 1, 2)
        self.btnService = QPushButton("Set", self)
        g.addWidget(self.btnService, 2, 3)

    # ------------------------------------------------------------------ Bridge inbound (Bridge -> Widget)

    def _wire_inbound(self) -> None:
        # erwartet: signals.homePoseChanged(PoseStamped), signals.servicePoseChanged(PoseStamped)
        self._sig.homePoseChanged.connect(
            lambda ps: self._set_pose_label_from_ps(self.lblHome, ps)
        )
        self._sig.servicePoseChanged.connect(
            lambda ps: self._set_pose_label_from_ps(self.lblService, ps)
        )

    # ------------------------------------------------------------------ Outbound (Widget -> Bridge)

    def _wire_outbound(self) -> None:
        # Buttons -> Widget-Signale
        self.btnHome.clicked.connect(self.setHomeRequested.emit)
        self.btnService.clicked.connect(self.setServiceRequested.emit)

        # Widget-Signale -> Bridge-Signale (ohne .emit im connect!)
        # erwartet: signals.setHomeRequested, signals.setServiceRequested
        self.setHomeRequested.connect(self._sig.setHomeRequested)
        self.setServiceRequested.connect(self._sig.setServiceRequested)

    # ------------------------------------------------------------------ Helpers

    @staticmethod
    def _pose_stamped_to_pose6(ps: PoseStamped) -> Tuple[float, float, float, float, float, float]:
        """
        PoseStamped -> (x,y,z, roll,pitch,yaw) in Radiant.
        """
        p = ps.pose.position
        q = ps.pose.orientation

        # Quaternion -> RPY (Roll/Pitch/Yaw)
        from math import atan2, asin, pi

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1.0:
            pitch = (sinp / abs(sinp)) * (pi / 2.0)
        else:
            pitch = asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return float(p.x), float(p.y), float(p.z), float(roll), float(pitch), float(yaw)

    @staticmethod
    def _fmt_pose6(pose: Tuple[float, float, float, float, float, float]) -> str:
        x, y, z, rx, ry, rz = pose
        return f"{x:.3f}  {y:.3f}  {z:.3f}   {rx:.3f}  {ry:.3f}  {rz:.3f}"

    def _set_pose_label_from_ps(self, label: QLabel, ps: PoseStamped) -> None:
        try:
            label.setText(self._fmt_pose6(self._pose_stamped_to_pose6(ps)))
        except Exception:
            _LOG.exception("failed to render PoseStamped")
            label.setText("-")
