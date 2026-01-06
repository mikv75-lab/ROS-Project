# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from typing import Optional, Any, Dict

from PyQt6 import QtCore
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from builtin_interfaces.msg import Duration as RosDuration
from std_msgs.msg import String as MsgString

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class ServoSignals(QtCore.QObject):
    """
    HARD CONTRACT to match tabs/service/servo_widgets.py

    JointJogWidget expects:
      - paramsChanged(dict)
      - jointJogRequested(str, float, float)          # joint_name, delta/sign, speed (pct)

    CartesianJogWidget expects:
      - frameChanged(str)                              # "world" / "tcp"
      - paramsChanged(dict)
      - cartesianJogRequested(str, float, float, str) # axis, delta/sign, speed, frame

    ServiceRobotTab expects:
      - modeChanged(str) (used as "emit(mode)")
    """

    # -------------------- UI -> Bridge (as widgets emit) --------------------

    paramsChanged = QtCore.pyqtSignal(dict)

    # JointJogWidget
    jointJogRequested = QtCore.pyqtSignal(str, float, float)  # joint_name, delta_or_sign, speed_pct

    # CartesianJogWidget
    frameChanged = QtCore.pyqtSignal(str)  # "world" / "tcp"
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)  # axis, delta_or_sign, speed, frame

    # expected by ServiceRobotTab
    modeChanged = QtCore.pyqtSignal(str)

    # -------------------- Optional legacy (do NOT collide) --------------------

    cartesianRequested = QtCore.pyqtSignal(object)        # TwistStamped direct
    jointJogObjectRequested = QtCore.pyqtSignal(object)   # JointJog direct

    # -------------------- Bridge -> UI (optional) --------------------

    twistOutChanged = QtCore.pyqtSignal(object)  # TwistStamped
    jointOutChanged = QtCore.pyqtSignal(object)  # JointJog

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._last_twist_out: Optional[TwistStamped] = None
        self._last_joint_out: Optional[JointJog] = None
        self._last_params: Dict[str, Any] = {}
        self._last_frame: str = "world"
        self._last_mode: str = ""

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        if self._last_params:
            self.paramsChanged.emit(dict(self._last_params))
        if self._last_frame:
            self.frameChanged.emit(self._last_frame)
        if self._last_mode:
            self.modeChanged.emit(self._last_mode)

        if self._last_twist_out is not None:
            self.twistOutChanged.emit(self._last_twist_out)
        if self._last_joint_out is not None:
            self.jointOutChanged.emit(self._last_joint_out)


class ServoBridge(BaseBridge):
    """
    Bridge for servo widgets -> ROS topics (SSoT IDs):
      publish:
        - cartesian_mm (TwistStamped)
        - joint_jog (JointJog)
        - set_mode (MsgString)
        - set_frame (MsgString)
      subscribe (optional, for UI monitoring):
        - twist_out (TwistStamped)
        - joint_out (JointJog)
    """

    GROUP = "servo"

    def __init__(self, content: AppContent, namespace: str = "") -> None:
        self.signals = ServoSignals()
        super().__init__("servo_bridge", content, namespace=namespace)

        # ---- internal "last known" widget params (for reasonable conversions) ----
        self._params: Dict[str, Any] = {
            # JointJogWidget
            "joint_step_deg": 2.0,
            "joint_speed_pct": 40.0,

            # CartesianJogWidget
            "cart_lin_step_mm": 1.0,
            "cart_ang_step_deg": 2.0,
            "cart_speed_mm_s": 50.0,
            "cart_speed_deg_s": 30.0,
            "frame": "world",
        }
        self._frame: str = "world"

        # ---------------- UI -> ROS ----------------

        # Params cache (widgets emit early/often)
        self.signals.paramsChanged.connect(self._on_params_changed)

        # Frame changes -> set_frame topic (and cache)
        self.signals.frameChanged.connect(self._on_frame_changed)

        # Mode changes -> set_mode topic (ServiceRobotTab emits this)
        self.signals.modeChanged.connect(self._on_mode_changed)

        # JointJogWidget -> JointJog msg
        self.signals.jointJogRequested.connect(self._on_joint_jog_widget)

        # CartesianJogWidget -> TwistStamped msg
        self.signals.cartesianJogRequested.connect(self._on_cartesian_jog_widget)

        # Optional legacy direct paths
        self.signals.cartesianRequested.connect(self.publish_cartesian)
        self.signals.jointJogObjectRequested.connect(self.publish_joint_jog)

    # ---------------- inbound (ROS -> UI) ----------------

    @sub_handler("servo", "twist_out")
    def _on_twist_out(self, msg: TwistStamped) -> None:
        self.signals._last_twist_out = msg
        self.signals.twistOutChanged.emit(msg)

    @sub_handler("servo", "joint_out")
    def _on_joint_out(self, msg: JointJog) -> None:
        self.signals._last_joint_out = msg
        self.signals.jointOutChanged.emit(msg)

    # ---------------- internal helpers ----------------

    def _set_duration(self, msg: JointJog, seconds: float) -> None:
        """
        Robustly set JointJog.duration across different Python IDL mappings.

        Some environments expose JointJog.duration incorrectly (e.g. float),
        so assignments like msg.duration.sec will crash. We handle:
          - builtin_interfaces/Duration message
          - assigning a new Duration
          - dict/tuple fallback
          - and finally "leave unset" without crashing
        """
        s = float(max(0.0, seconds))
        sec = int(s)
        nsec = int((s - sec) * 1e9)

        # preferred: duration already a Duration message
        try:
            d = getattr(msg, "duration", None)
            if isinstance(d, RosDuration):
                d.sec = sec
                d.nanosec = nsec
                return
        except Exception:
            pass

        # common: field exists but is not initialized as message
        try:
            msg.duration = RosDuration(sec=sec, nanosec=nsec)
            return
        except Exception:
            pass

        # alternative: some bindings accept dict/tuple
        try:
            msg.duration = {"sec": sec, "nanosec": nsec}
            return
        except Exception:
            pass

        try:
            msg.duration = (sec, nsec)
            return
        except Exception:
            pass

        # last resort: do nothing
        try:
            self.get_logger().debug(
                "[servo] JointJog.duration could not be set (mapping=%r)",
                type(getattr(msg, "duration", None)),
            )
        except Exception:
            pass

    # ---------------- UI event handlers ----------------

    def _on_params_changed(self, cfg: dict) -> None:
        try:
            if isinstance(cfg, dict):
                self._params.update(cfg)
                self.signals._last_params = dict(self._params)
        except Exception:
            # never crash on params
            pass

    def _on_frame_changed(self, frame: str) -> None:
        f = (frame or "").strip().lower()
        if not f:
            return
        if f not in ("world", "tcp", "trf", "wrf"):
            return
        if f == "trf":
            f = "tcp"
        if f == "wrf":
            f = "world"

        self._frame = f
        self._params["frame"] = f
        self.signals._last_frame = f
        self.set_frame(f)

    def _on_mode_changed(self, mode: str) -> None:
        m = (mode or "").strip()
        if not m:
            return
        self.signals._last_mode = m
        self.set_mode(m)

    def _on_joint_jog_widget(self, joint: str, delta_or_sign: float, speed_pct: float) -> None:
        """
        JointJogWidget contract:
          jointJogRequested(joint_name, delta_deg OR sign, speed_pct)

        We translate to JointJog (velocity command) and rely on widget streaming + stop.
        - If delta_or_sign is near +/-1 -> treat as direction sign.
        - Else treat as delta in deg; convert to sign and estimate duration from delta & speed.
        """
        j = (joint or "").strip()
        if not j:
            return

        val = float(delta_or_sign)
        sp = float(speed_pct)

        # stop command (widget sends (name, 0, 0))
        if abs(val) < 1e-9 or abs(sp) < 1e-9:
            msg = JointJog()
            msg.joint_names = [j]
            msg.velocities = [0.0]
            self._set_duration(msg, 0.0)
            self.publish_joint_jog(msg)
            return

        # clamp speed percent
        sp = abs(sp)
        sp = max(0.0, min(100.0, sp))

        # Heuristic: sign vs delta
        if abs(val) <= 1.5:
            sign = 1.0 if val >= 0.0 else -1.0
            delta_deg = None
        else:
            sign = 1.0 if val >= 0.0 else -1.0
            delta_deg = abs(val)

        # Convert speed_pct to a reasonable rad/s.
        # If you later want exact limits: add them to params and use them here.
        max_rad_s = 1.5  # conservative default
        vel = sign * (sp / 100.0) * max_rad_s

        msg = JointJog()
        msg.joint_names = [j]
        msg.velocities = [float(vel)]

        # Duration:
        # - streaming (hold) -> short duration; widget ticks at 50 Hz
        # - pulse (delta given) -> estimate duration from delta and speed
        if delta_deg is None:
            dur_s = 0.10
        else:
            vel_deg_s = abs(vel) * (180.0 / math.pi)
            if vel_deg_s < 1e-6:
                dur_s = 0.10
            else:
                dur_s = max(0.05, min(1.0, float(delta_deg) / vel_deg_s))

        self._set_duration(msg, dur_s)
        self.publish_joint_jog(msg)

    def _on_cartesian_jog_widget(self, axis: str, delta_or_sign: float, speed: float, frame: str) -> None:
        """
        CartesianJogWidget contract:
          cartesianJogRequested(axis, delta_or_sign, speed, frame)

        We translate to TwistStamped on 'cartesian_mm'.
        Your ServoWrapper already applies header.frame_id based on set_frame,
        but we also set it here to be explicit.
        """
        a = (axis or "").strip().lower()
        if not a:
            return

        v = float(delta_or_sign)
        sp = float(speed)

        # stop
        if abs(v) < 1e-9 or abs(sp) < 1e-9:
            msg = TwistStamped()
            msg.header.frame_id = (frame or self._frame or "world")
            self.publish_cartesian(msg)
            return

        # interpret sign vs delta
        if abs(v) <= 1.5:
            sign = 1.0 if v >= 0.0 else -1.0
        else:
            sign = 1.0 if v >= 0.0 else -1.0  # magnitude ignored; widget sends stop shortly after

        msg = TwistStamped()
        msg.header.frame_id = (frame or self._frame or "world")

        # Linear axes are in mm/s for your pipeline (topic id: cartesian_mm)
        # Rotational axes: widget uses deg/s; convert to rad/s.
        if a == "x":
            msg.twist.linear.x = sign * sp
        elif a == "y":
            msg.twist.linear.y = sign * sp
        elif a == "z":
            msg.twist.linear.z = sign * sp
        elif a == "rx":
            msg.twist.angular.x = sign * math.radians(sp)
        elif a == "ry":
            msg.twist.angular.y = sign * math.radians(sp)
        elif a == "rz":
            msg.twist.angular.z = sign * math.radians(sp)
        else:
            return

        # Keep internal frame cache + publish set_frame so ServoWrapper updates too
        f = (frame or "").strip().lower()
        if f:
            if f == "trf":
                f = "tcp"
            if f == "wrf":
                f = "world"
            if f in ("world", "tcp") and f != self._frame:
                self._frame = f
                self._params["frame"] = f
                self.signals._last_frame = f
                self.set_frame(f)

        self.publish_cartesian(msg)

    # ---------------- outbound (Bridge -> ROS) ----------------

    def publish_cartesian(self, msg: Any) -> None:
        if msg is None:
            return
        try:
            self.pub("cartesian_mm").publish(msg)
        except Exception as e:
            self.get_logger().error(f"[servo] publish cartesian_mm failed: {e}")

    def publish_joint_jog(self, msg: Any) -> None:
        if msg is None:
            return
        try:
            self.pub("joint_jog").publish(msg)
        except Exception as e:
            self.get_logger().error(f"[servo] publish joint_jog failed: {e}")

    def set_mode(self, mode: str) -> None:
        mode = (mode or "").strip()
        if not mode:
            return
        try:
            self.pub("set_mode").publish(MsgString(data=mode))
        except Exception as e:
            self.get_logger().error(f"[servo] set_mode failed: {e}")

    def set_frame(self, frame: str) -> None:
        frame = (frame or "").strip()
        if not frame:
            return
        try:
            self.pub("set_frame").publish(MsgString(data=frame))
        except Exception as e:
            self.get_logger().error(f"[servo] set_frame failed: {e}")
