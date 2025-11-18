# -*- coding: utf-8 -*-
# File: widgets/robot_status_box.py
from __future__ import annotations
from typing import Optional, Iterable, Tuple, Dict, Any, Sequence

from math import atan2, asin, degrees

from PyQt6 import QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QSizePolicy
)


class RobotStatusInfoBox(QGroupBox):
    """
    Vertikal angeordnete Status-Anzeige:
      - jede Zeile: [Bold Label]  [Wert-Label]
      - Connection, Mode, Initialized, Moving, Servo, Power, E-Stop
      - TCP Pose (6D)
      - Joints (Grad)
      - Errors (einfacher String, word-wrapped)
    """
    def __init__(self, parent: Optional[QWidget] = None, title: str = "Robot Status"):
        super().__init__(title, parent)
        self._build_ui()
        self._apply_policies()

    # ---------- UI ----------
    def _build_ui(self) -> None:
        self._root = QVBoxLayout(self)
        self._root.setContentsMargins(8, 8, 8, 8)
        self._root.setSpacing(6)

        def row(label_text: str) -> QLabel:
            hb = QHBoxLayout()
            hb.setSpacing(8)
            bold = QLabel(label_text, self)
            bold.setStyleSheet("font-weight:600;")
            val = QLabel("-", self)
            val.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)

            hb.addWidget(bold)
            hb.addWidget(val, 1)
            self._root.addLayout(hb)
            return val

        self.lblConn   = row("Connection")
        self.lblMode   = row("Mode")
        self.lblInit   = row("Initialized")
        self.lblMoving = row("Moving")
        self.lblServo  = row("Servo")
        self.lblPower  = row("Power")
        self.lblEstop  = row("E-Stop")

        self.lblPose   = row("TCP Pose (X Y Z RX RY RZ)")
        self.lblJoints = row("Joints (°)")

        # Errors: eigener Block
        hb_err = QHBoxLayout()
        hb_err.setSpacing(8)
        lbl_err_bold = QLabel("Errors", self)
        lbl_err_bold.setStyleSheet("font-weight:600;")
        self.lblErrors = QLabel("-", self)
        self.lblErrors.setWordWrap(True)
        self.lblErrors.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        hb_err.addWidget(lbl_err_bold)
        hb_err.addWidget(self.lblErrors, 1)
        self._root.addLayout(hb_err)

        self._root.addStretch(1)

    def _apply_policies(self) -> None:
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    # ---------- Public setters ----------
    def set_connection(self, connected: bool | str):
        self.lblConn.setText(self._fmt_bool_or_text(connected))

    def set_mode(self, mode: str):
        self.lblMode.setText(mode or "-")

    def set_initialized(self, flag: bool):
        self._set_bool_label(self.lblInit, flag)

    def set_moving(self, flag: bool):
        self._set_bool_label(self.lblMoving, flag)

    def set_servo_enabled(self, flag: bool):
        self._set_bool_label(self.lblServo, flag)

    def set_power(self, flag: bool):
        self._set_bool_label(self.lblPower, flag)

    def set_estop(self, engaged: bool):
        self.lblEstop.setText("ENGAGED" if engaged else "OK")
        self.lblEstop.setStyleSheet("color:#b00020;" if engaged else "color:#22863a;")

    def set_errors(self, text: str | Sequence[str] | None):
        if text is None:
            self.lblErrors.setText("-")
        elif isinstance(text, (list, tuple)):
            self.lblErrors.setText("\n".join(map(str, text)))
        else:
            s = str(text).strip()
            self.lblErrors.setText(s if s else "-")

    def set_tcp_pose6(self, pose6: Iterable[float] | Tuple[float, float, float, float, float, float]):
        self.lblPose.setText(self._fmt_pose6(pose6))

    def set_tcp_from_ps(self, ps):
        if ps is None:
            self.lblPose.setText("-")
            return
        self.lblPose.setText(self._fmt_pose6(self._pose_stamped_to_pose6(ps)))

    def set_joints(self, joints: Sequence[float] | None):
        """Erwartet Joint-Werte in RAD und zeigt sie in GRAD an."""
        if not joints:
            self.lblJoints.setText("-")
            return
        self.lblJoints.setText("  ".join(f"{degrees(float(j)):.1f}" for j in joints))

    def set_status_dict(self, st: Dict[str, Any]):
        if "connected" in st: self.set_connection(st.get("connected"))
        if "mode" in st: self.set_mode(st.get("mode") or "-")
        if "initialized" in st: self.set_initialized(bool(st.get("initialized")))
        if "moving" in st: self.set_moving(bool(st.get("moving")))
        if "power" in st: self.set_power(bool(st.get("power")))
        if "servo_enabled" in st: self.set_servo_enabled(bool(st.get("servo_enabled")))
        if "estop" in st: self.set_estop(bool(st.get("estop")))
        if "errors" in st: self.set_errors(st.get("errors"))
        if "tcp_pose6" in st: self.set_tcp_pose6(st.get("tcp_pose6"))
        if "joints" in st: self.set_joints(st.get("joints"))

    # ---------- helpers ----------
    @staticmethod
    def _set_bool_label(label: QLabel, flag: bool):
        label.setText("Yes" if flag else "No")
        label.setStyleSheet("color:#22863a;" if flag else "color:#6a737d;")

    @staticmethod
    def _fmt_bool_or_text(v: bool | str) -> str:
        if isinstance(v, bool):
            return "Connected" if v else "Disconnected"
        return v or "-"

    @staticmethod
    def _pose_stamped_to_pose6(ps) -> Tuple[float, float, float, float, float, float]:
        p = ps.pose.position
        q = ps.pose.orientation  # x y z w

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if abs(sinp) >= 1:
            from math import pi
            pitch = (sinp / abs(sinp)) * (pi / 2.0)
        else:
            pitch = asin(sinp)

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = atan2(siny_cosp, cosy_cosp)

        return float(p.x), float(p.y), float(p.z), float(roll), float(pitch), float(yaw)

    @staticmethod
    def _fmt_pose6(pose: Iterable[float] | Tuple[float, float, float, float, float, float]) -> str:
        try:
            x, y, z, rx, ry, rz = pose  # type: ignore[misc]
            return f"{x:.3f}  {y:.3f}  {z:.3f}   {rx:.3f}  {ry:.3f}  {rz:.3f}"
        except Exception:
            return "-"
