# -*- coding: utf-8 -*-
# File: widgets/robot_command_status_box.py
from __future__ import annotations
from typing import Optional, Iterable, Tuple, Dict, Any, Sequence

from math import atan2, asin, degrees

from PyQt6 import QtCore
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout,
    QGroupBox, QLabel, QPushButton, QSizePolicy
)

try:
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState
except Exception:  # pragma: no cover
    PoseStamped = None  # type: ignore[assignment]
    JointState = None   # type: ignore[assignment]


# =============================================================================
# Rechte Spalte: Status-Anzeige (nur VerticalLayout, jede Zeile als HBox)
# =============================================================================
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

        # Reihen-Helper
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

        # Zeilen anlegen
        self.lblConn   = row("Connection")
        self.lblMode   = row("Mode")
        self.lblInit   = row("Initialized")
        self.lblMoving = row("Moving")
        self.lblServo  = row("Servo")
        self.lblPower  = row("Power")
        self.lblEstop  = row("E-Stop")

        self.lblPose   = row("TCP Pose (X Y Z RX RY RZ)")
        self.lblJoints = row("Joints (°)")

        # Errors: eigener Block (multi-line)
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
        """
        Erwartet Joint-Werte in RAD und zeigt sie in GRAD an.
        """
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
        # Erwartet geometry_msgs/PoseStamped
        p = ps.pose.position
        q = ps.pose.orientation  # x y z w
        # RPY (ZYX)
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


# =============================================================================
# Linke Spalte: Commands (vertikale Anordnung)
# =============================================================================
class RobotCommandButtonsBox(QGroupBox):
    """
    Vertikale Aktions-Buttons:
      Initialize | Stop | Clear Error | Power ON/OFF | Servo ENABLE/DISABLE
    """
    initRequested         = QtCore.pyqtSignal()
    stopRequested         = QtCore.pyqtSignal()
    clearErrorRequested   = QtCore.pyqtSignal()
    powerOnRequested      = QtCore.pyqtSignal()
    powerOffRequested     = QtCore.pyqtSignal()
    servoEnableRequested  = QtCore.pyqtSignal()
    servoDisableRequested = QtCore.pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None, title: str = "Commands"):
        super().__init__(title, parent)
        self._build_ui()
        self._apply_policies()

    def _build_ui(self) -> None:
        v = QVBoxLayout(self)
        v.setSpacing(6)
        v.setContentsMargins(8, 8, 8, 8)

        self.btnInit   = QPushButton("Initialize", self)
        self.btnStop   = QPushButton("Stop", self)
        self.btnClrErr = QPushButton("Clear Error", self)
        self.btnPwrOn  = QPushButton("Power ON", self)
        self.btnPwrOff = QPushButton("Power OFF", self)
        self.btnSrvOn  = QPushButton("Servo ENABLE", self)
        self.btnSrvOff = QPushButton("Servo DISABLE", self)

        for b in (self.btnInit, self.btnStop, self.btnClrErr,
                  self.btnPwrOn, self.btnPwrOff, self.btnSrvOn, self.btnSrvOff):
            b.setMinimumHeight(28)
            v.addWidget(b)

        v.addStretch(1)

        # Buttons -> Widget-Signale
        self.btnInit.clicked.connect(self.initRequested.emit)
        self.btnStop.clicked.connect(self.stopRequested.emit)
        self.btnClrErr.clicked.connect(self.clearErrorRequested.emit)
        self.btnPwrOn.clicked.connect(self.powerOnRequested.emit)
        self.btnPwrOff.clicked.connect(self.powerOffRequested.emit)
        self.btnSrvOn.clicked.connect(self.servoEnableRequested.emit)
        self.btnSrvOff.clicked.connect(self.servoDisableRequested.emit)

    def _apply_policies(self) -> None:
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)


# =============================================================================
# Composite: zwei GroupBoxen nebeneinander (Commands links | Status rechts)
# =============================================================================
class RobotCommandStatusWidget(QWidget):
    """
    Zusammengesetztes Widget:
      [ RobotCommandButtonsBox (links) ] | [ RobotStatusInfoBox (rechts) ]
    """
    def __init__(self, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.bridge = bridge

        # Referenzen auf RobotBridge + Signals merken
        self._rb = getattr(self.bridge, "_rb", None)
        self._sig = getattr(self._rb, "signals", None) if self._rb else None
        self._inbound_connected = False
        self._outbound_connected = False

        # UI
        h = QHBoxLayout(self)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(8)

        self.commandBox = RobotCommandButtonsBox(self)  # links
        self.statusBox  = RobotStatusInfoBox(self)      # rechts

        h.addWidget(self.commandBox, 1)
        h.addWidget(self.statusBox, 2)

        for w in (self.statusBox, self.commandBox):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        # Wiring
        self._wire_bridge_inbound()
        self._wire_outbound()

    # ---------- Bridge: inbound (ROS -> UI) ----------
    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        """Slot für jointsChanged (JointState oder kompatibel)."""
        if js is None or not hasattr(js, "position"):
            self.statusBox.set_joints(None)
        else:
            self.statusBox.set_joints(list(js.position or []))

    def _wire_bridge_inbound(self) -> None:
        if not self._sig or self._inbound_connected:
            return

        sig = self._sig
        sb = self.statusBox

        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(sb.set_connection)
        if hasattr(sig, "modeChanged"):
            sig.modeChanged.connect(sb.set_mode)
        if hasattr(sig, "initializedChanged"):
            sig.initializedChanged.connect(sb.set_initialized)
        if hasattr(sig, "movingChanged"):
            sig.movingChanged.connect(sb.set_moving)
        if hasattr(sig, "powerChanged"):
            sig.powerChanged.connect(sb.set_power)
        if hasattr(sig, "servoEnabledChanged"):
            sig.servoEnabledChanged.connect(sb.set_servo_enabled)
        if hasattr(sig, "estopChanged"):
            sig.estopChanged.connect(sb.set_estop)
        if hasattr(sig, "errorsChanged"):
            sig.errorsChanged.connect(sb.set_errors)
        if hasattr(sig, "tcpPoseChanged"):
            sig.tcpPoseChanged.connect(sb.set_tcp_from_ps)
        if hasattr(sig, "jointsChanged"):
            sig.jointsChanged.connect(self._on_joints)

        self._inbound_connected = True

    # ---------- Bridge: outbound (UI -> ROS) ----------
    def _wire_outbound(self) -> None:
        if not self._sig or self._outbound_connected:
            return

        sig = self._sig
        cb = self.commandBox

        cb.initRequested.connect(sig.initRequested.emit)
        cb.stopRequested.connect(sig.stopRequested.emit)
        cb.clearErrorRequested.connect(sig.clearErrorRequested.emit)
        cb.powerOnRequested.connect(sig.powerOnRequested.emit)
        cb.powerOffRequested.connect(sig.powerOffRequested.emit)
        cb.servoEnableRequested.connect(sig.servoEnableRequested.emit)
        cb.servoDisableRequested.connect(sig.servoDisableRequested.emit)

        self._outbound_connected = True

    # ---------- Disconnect-Helfer ----------
    def _disconnect_inbound(self) -> None:
        if not self._sig or not self._inbound_connected:
            return

        sig = self._sig
        sb = self.statusBox

        def safe_disc(signal, slot):
            try:
                signal.disconnect(slot)
            except TypeError:
                pass

        if hasattr(sig, "connectionChanged"):
            safe_disc(sig.connectionChanged, sb.set_connection)
        if hasattr(sig, "modeChanged"):
            safe_disc(sig.modeChanged, sb.set_mode)
        if hasattr(sig, "initializedChanged"):
            safe_disc(sig.initializedChanged, sb.set_initialized)
        if hasattr(sig, "movingChanged"):
            safe_disc(sig.movingChanged, sb.set_moving)
        if hasattr(sig, "powerChanged"):
            safe_disc(sig.powerChanged, sb.set_power)
        if hasattr(sig, "servoEnabledChanged"):
            safe_disc(sig.servoEnabledChanged, sb.set_servo_enabled)
        if hasattr(sig, "estopChanged"):
            safe_disc(sig.estopChanged, sb.set_estop)
        if hasattr(sig, "errorsChanged"):
            safe_disc(sig.errorsChanged, sb.set_errors)
        if hasattr(sig, "tcpPoseChanged"):
            safe_disc(sig.tcpPoseChanged, sb.set_tcp_from_ps)
        if hasattr(sig, "jointsChanged"):
            safe_disc(sig.jointsChanged, self._on_joints)

        self._inbound_connected = False

    def _disconnect_outbound(self) -> None:
        if not self._sig or not self._outbound_connected:
            return

        sig = self._sig
        cb = self.commandBox

        def safe_disc(signal, slot):
            try:
                signal.disconnect(slot)
            except TypeError:
                pass

        safe_disc(cb.initRequested, sig.initRequested.emit)
        safe_disc(cb.stopRequested, sig.stopRequested.emit)
        safe_disc(cb.clearErrorRequested, sig.clearErrorRequested.emit)
        safe_disc(cb.powerOnRequested, sig.powerOnRequested.emit)
        safe_disc(cb.powerOffRequested, sig.powerOffRequested.emit)
        safe_disc(cb.servoEnableRequested, sig.servoEnableRequested.emit)
        safe_disc(cb.servoDisableRequested, sig.servoDisableRequested.emit)

        self._outbound_connected = False

    # ---------- Convenience ----------
    def set_status_dict(self, st: Dict[str, Any]) -> None:
        self.statusBox.set_status_dict(st)
