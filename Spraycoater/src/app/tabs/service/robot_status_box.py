# -*- coding: utf-8 -*-
# File: widgets/robot_status_box.py
from __future__ import annotations
from typing import Optional, Iterable, Tuple, Dict, Any, Sequence
from math import atan2, asin
from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QGroupBox, QGridLayout, QLabel, QPushButton, QWidget, QHBoxLayout, QTextEdit
)
from geometry_msgs.msg import PoseStamped  # optional, wenn vorhanden

class RobotStatusGroupBox(QGroupBox):
    """
    Kompakter Roboter-Statusblock mit Bridge-Anbindung (Inbound + Outbound).
    Anzeige:
      - Connection, Mode
      - Initialized, Moving, Servo, Power, E-Stop
      - Errors (mehrzeilig)
      - TCP Pose (X Y Z RX RY RZ)
      - Joints (J1..Jn, kompakt)

    Outbound-Signals (Widget -> außen / Bridge):
      - initRequested()
      - stopRequested()
      - clearErrorRequested()
      - powerOnRequested()
      - powerOffRequested()
      - servoEnableRequested()
      - servoDisableRequested()

    Inbound (Bridge -> Widget): verdrahtet sich automatisch, wenn es passende
    Signals in der Bridge gibt. Unterstützte Kandidaten (falls vorhanden):
      - statusChanged(dict)                           # bevorzugt
      - connectionChanged(str/bool)
      - modeChanged(str)
      - initializedChanged(bool)
      - movingChanged(bool)
      - estopChanged(bool)
      - powerChanged(bool)
      - servoEnabledChanged(bool)
      - errorTextChanged(str)
      - tcpPoseChanged(PoseStamped)
      - jointsChanged(list/tuple of float)
    """

    # ---------- Outbound ----------
    initRequested         = QtCore.pyqtSignal()
    stopRequested         = QtCore.pyqtSignal()
    clearErrorRequested   = QtCore.pyqtSignal()
    powerOnRequested      = QtCore.pyqtSignal()
    powerOffRequested     = QtCore.pyqtSignal()
    servoEnableRequested  = QtCore.pyqtSignal()
    servoDisableRequested = QtCore.pyqtSignal()

    def __init__(self, bridge, parent: Optional[QWidget] = None, title: str = "Robot Status"):
        super().__init__(parent)
        self.setTitle(title)
        self.bridge = bridge
        self._build_ui()
        self._wire_outbound_to_bridge_if_present()
        self._wire_bridge_inbound()

    # ---------- UI ----------
    def _build_ui(self):
        g = QGridLayout(self)
        r = 0

        def _bold(lbl: str) -> QLabel:
            w = QLabel(lbl, self)
            w.setStyleSheet("font-weight:600;")
            return w

        # Zeile 1: Connection / Mode
        g.addWidget(_bold("Connection"), r, 0)
        self.lblConn = QLabel("-", self); g.addWidget(self.lblConn, r, 1)
        g.addWidget(_bold("Mode"), r, 2)
        self.lblMode = QLabel("-", self); g.addWidget(self.lblMode, r, 3)
        r += 1

        # Zeile 2: Flags
        g.addWidget(_bold("Initialized"), r, 0)
        self.lblInit = QLabel("-", self); g.addWidget(self.lblInit, r, 1)

        g.addWidget(_bold("Moving"), r, 2)
        self.lblMoving = QLabel("-", self); g.addWidget(self.lblMoving, r, 3)
        r += 1

        g.addWidget(_bold("Servo"), r, 0)
        self.lblServo = QLabel("-", self); g.addWidget(self.lblServo, r, 1)

        g.addWidget(_bold("Power"), r, 2)
        self.lblPower = QLabel("-", self); g.addWidget(self.lblPower, r, 3)
        r += 1

        g.addWidget(_bold("E-Stop"), r, 0)
        self.lblEstop = QLabel("-", self); g.addWidget(self.lblEstop, r, 1)
        r += 1

        # Zeile: TCP Pose
        g.addWidget(_bold("TCP Pose (X Y Z RX RY RZ)"), r, 0)
        self.lblPose = QLabel("-", self); g.addWidget(self.lblPose, r, 1, 1, 3)
        r += 1

        # Zeile: Joints
        g.addWidget(_bold("Joints (rad)"), r, 0)
        self.lblJoints = QLabel("-", self); self.lblJoints.setTextInteractionFlags(
            self.lblJoints.textInteractionFlags() | QtCore.Qt.TextInteractionFlag.TextSelectableByMouse
        )
        g.addWidget(self.lblJoints, r, 1, 1, 3)
        r += 1

        # Zeile: Errors
        g.addWidget(_bold("Errors"), r, 0)
        self.txtErrors = QTextEdit(self)
        self.txtErrors.setReadOnly(True)
        self.txtErrors.setFixedHeight(64)
        g.addWidget(self.txtErrors, r, 1, 1, 3)
        r += 1

        # Buttons (Aktionen)
        self.btnRow = QHBoxLayout()
        self.btnInit   = QPushButton("Initialize", self)
        self.btnStop   = QPushButton("Stop", self)
        self.btnClrErr = QPushButton("Clear Error", self)
        self.btnPwrOn  = QPushButton("Power ON", self)
        self.btnPwrOff = QPushButton("Power OFF", self)
        self.btnSrvOn  = QPushButton("Servo ENABLE", self)
        self.btnSrvOff = QPushButton("Servo DISABLE", self)

        for b in (self.btnInit, self.btnStop, self.btnClrErr, self.btnPwrOn, self.btnPwrOff, self.btnSrvOn, self.btnSrvOff):
            b.setMinimumHeight(28)
            self.btnRow.addWidget(b)
        self.btnRow.addStretch(1)
        g.addLayout(self.btnRow, r, 0, 1, 4)
        r += 1

        # Wire Buttons -> Signals
        self.btnInit.clicked.connect(self.initRequested.emit)
        self.btnStop.clicked.connect(self.stopRequested.emit)
        self.btnClrErr.clicked.connect(self.clearErrorRequested.emit)
        self.btnPwrOn.clicked.connect(self.powerOnRequested.emit)
        self.btnPwrOff.clicked.connect(self.powerOffRequested.emit)
        self.btnSrvOn.clicked.connect(self.servoEnableRequested.emit)
        self.btnSrvOff.clicked.connect(self.servoDisableRequested.emit)

    # ---------- Public API (Setter) ----------
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
        # E-Stop engaged = True => rot
        self.lblEstop.setText("ENGAGED" if engaged else "OK")
        self.lblEstop.setStyleSheet("color: #b00020;" if engaged else "color: #22863a;")

    def set_errors(self, text: str | Sequence[str] | None):
        if text is None:
            self.txtErrors.setPlainText("")
        elif isinstance(text, (list, tuple)):
            self.txtErrors.setPlainText("\n".join(map(str, text)))
        else:
            self.txtErrors.setPlainText(str(text))

    def set_tcp_pose6(self, pose6: Iterable[float] | Tuple[float, float, float, float, float, float]):
        self.lblPose.setText(self._fmt_pose6(pose6))

    def set_tcp_from_ps(self, ps: PoseStamped):
        self.lblPose.setText(self._fmt_pose6(self._pose_stamped_to_pose6(ps)))

    def set_joints(self, joints: Sequence[float] | None):
        if not joints:
            self.lblJoints.setText("-")
            return
        # kompakt, 3 Dezimalen
        self.lblJoints.setText("  ".join(f"{float(j):.3f}" for j in joints))

    def set_status_dict(self, st: Dict[str, Any]):
        """
        Erwartete Keys (optional): connected, mode, initialized, moving, estop,
        power, servo_enabled, errors, tcp_pose6, joints
        """
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

    # ---------- Bridge: wire inbound ----------
    def _wire_bridge_inbound(self):
        # Kandidaten für Status-Bridge
        cand = [getattr(self.bridge, "_rb", None),
                getattr(self.bridge, "_status", None),
                getattr(self.bridge, "_pb", None)]  # fallback: PosesBridge liefert u.U. tcpPose

        br = next((c for c in cand if c is not None), None)
        sig = getattr(br, "signals", None) if br else None
        if not sig:
            return

        # 1) Bevorzugt: statusChanged(dict)
        if hasattr(sig, "statusChanged"):
            sig.statusChanged.connect(self.set_status_dict)

        # 2) Einzel-Signale (optional)
        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(self.set_connection)
        if hasattr(sig, "modeChanged"):
            sig.modeChanged.connect(self.set_mode)
        if hasattr(sig, "initializedChanged"):
            sig.initializedChanged.connect(self.set_initialized)
        if hasattr(sig, "movingChanged"):
            sig.movingChanged.connect(self.set_moving)
        if hasattr(sig, "powerChanged"):
            sig.powerChanged.connect(self.set_power)
        if hasattr(sig, "servoEnabledChanged"):
            sig.servoEnabledChanged.connect(self.set_servo_enabled)
        if hasattr(sig, "estopChanged"):
            sig.estopChanged.connect(self.set_estop)
        if hasattr(sig, "errorTextChanged"):
            sig.errorTextChanged.connect(self.set_errors)
        if hasattr(sig, "tcpPoseChanged"):
            sig.tcpPoseChanged.connect(self.set_tcp_from_ps)
        if hasattr(sig, "jointsChanged"):
            sig.jointsChanged.connect(self.set_joints)

    # ---------- Bridge: wire outbound ----------
    def _wire_outbound_to_bridge_if_present(self):
        br = getattr(self.bridge, "_rb", None) or getattr(self.bridge, "_status", None)
        bsig = getattr(br, "signals", None) if br else None
        if not bsig:
            # keine Bridge-Durchleitung; Parent kann die Widget-Signals selbst nutzen
            return

        # Falls die Bridge die passenden Requests bereitstellt, direkt durchleiten:
        if hasattr(bsig, "initRequested"):
            self.initRequested.connect(bsig.initRequested.emit)
        if hasattr(bsig, "stopRequested"):
            self.stopRequested.connect(bsig.stopRequested.emit)
        if hasattr(bsig, "clearErrorRequested"):
            self.clearErrorRequested.connect(bsig.clearErrorRequested.emit)
        if hasattr(bsig, "powerOnRequested"):
            self.powerOnRequested.connect(bsig.powerOnRequested.emit)
        if hasattr(bsig, "powerOffRequested"):
            self.powerOffRequested.connect(bsig.powerOffRequested.emit)
        if hasattr(bsig, "servoEnableRequested"):
            self.servoEnableRequested.connect(bsig.servoEnableRequested.emit)
        if hasattr(bsig, "servoDisableRequested"):
            self.servoDisableRequested.connect(bsig.servoDisableRequested.emit)

    # ---------- Helpers ----------
    @staticmethod
    def _set_bool_label(label: QLabel, flag: bool):
        label.setText("Yes" if flag else "No")
        label.setStyleSheet("color: #22863a;" if flag else "color: #6a737d;")

    @staticmethod
    def _fmt_bool_or_text(v: bool | str):
        if isinstance(v, bool):
            return "Connected" if v else "Disconnected"
        return v or "-"

    @staticmethod
    def _pose_stamped_to_pose6(ps: PoseStamped) -> Tuple[float, float, float, float, float, float]:
        p = ps.pose.position
        q = ps.pose.orientation
        # ZYX
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
