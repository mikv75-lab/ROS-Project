# -*- coding: utf-8 -*-
# File: src/ros/bridge/servo_bridge.py
from __future__ import annotations

from typing import Optional, Dict, Any, Callable, Type
import math

from PyQt6 import QtCore

from std_msgs.msg import String as MsgString
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

from config.startup import AppContent, TopicSpec
from .base_bridge import BaseBridge


class ServoSignals(QtCore.QObject):
    """
    Qt-Signale (UI -> ServoBridge -> ROS Topics)

    topics.yaml (servo/subscribe)  => UI -> ROS-Servo-Wrapper-Node
      - cartesian_mm  TwistStamped
      - joint_jog     JointJog
      - set_mode      String
      - set_frame     String

    topics.yaml (servo/publish)    => ROS-Servo-Wrapper-Node -> UI (optional Debug)
      - twist_out     TwistStamped
      - joint_out     JointJog
    """

    paramsChanged = QtCore.pyqtSignal(dict)

    jointJogRequested = QtCore.pyqtSignal(str, float, float)
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)

    frameChanged = QtCore.pyqtSignal(str)
    modeChanged = QtCore.pyqtSignal(str)

    twistOutChanged = QtCore.pyqtSignal(object)  # TwistStamped
    jointOutChanged = QtCore.pyqtSignal(object)  # JointJog

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self._last_twist_out: Optional[TwistStamped] = None
        self._last_joint_out: Optional[JointJog] = None

    def _set_twist_out(self, msg: TwistStamped) -> None:
        self._last_twist_out = msg

    def _set_joint_out(self, msg: JointJog) -> None:
        self._last_joint_out = msg

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        if self._last_twist_out is not None:
            self.twistOutChanged.emit(self._last_twist_out)
        if self._last_joint_out is not None:
            self.jointOutChanged.emit(self._last_joint_out)


class ServoBridge(BaseBridge):
    """
    UI-ServoBridge (PyQt-Seite)

    WICHTIG:
      - Single Source of Truth: topics.yaml + qos.yaml (über AppContent)
      - KEIN manuelles Prefixing, KEIN Hardcode QoS
    """

    GROUP = "servo"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = ServoSignals()

        self._ui_to_node_pubs: Dict[str, Any] = {}
        self._node_to_ui_subs: Dict[str, Any] = {}

        # UI Params (Defaults)
        self._cart_lin_step_mm: float = 1.0
        self._cart_ang_step_deg: float = 2.0
        self._cart_speed_mm_s: float = 50.0
        self._cart_speed_deg_s: float = 30.0

        self._joint_step_deg: float = 2.0
        self._joint_speed_pct: float = 40.0

        self._frame_ui: str = "wrf"
        self._last_mode_txt: Optional[str] = None
        self._last_frame_txt: Optional[str] = None

        super().__init__("servo_bridge", content, namespace=namespace)

        s = self.signals
        s.paramsChanged.connect(self._on_params_changed)
        s.frameChanged.connect(self._on_frame_changed)
        s.modeChanged.connect(lambda m: self.set_command_type(m, force=False))
        s.jointJogRequested.connect(self._on_joint_jog_requested)
        s.cartesianJogRequested.connect(self._on_cart_jog_requested)

        # UI -> Node
        self._ensure_pub("cartesian_mm", TwistStamped)
        self._ensure_pub("joint_jog", JointJog)
        self._ensure_pub("set_mode", MsgString)
        self._ensure_pub("set_frame", MsgString)

        # Node -> UI (optional)
        self._ensure_sub("twist_out", TwistStamped, self._on_twist_out)
        self._ensure_sub("joint_out", JointJog, self._on_joint_out)

        self.get_logger().info(
            f"[servo] ServoBridge bereit (ns='{self.namespace or '/'}'): "
            "pubs: cartesian_mm, joint_jog, set_mode, set_frame | subs: twist_out, joint_out"
        )

    # ─────────────────────────────────────────────────────────────
    # SSoT Helpers: topics.yaml + qos.yaml (AppContent)
    # ─────────────────────────────────────────────────────────────

    def _spec(self, direction: str, topic_id: str) -> TopicSpec:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        if topic_id in self._ui_to_node_pubs:
            return

        spec = self._spec("subscribe", topic_id)  # Node subscribt => UI published
        qos = self._content.qos(spec.qos_key)
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, spec.name, qos)
        self.get_logger().info(f"[servo] PUB ui->node id={topic_id} topic={spec.name} qos={spec.qos_key}")

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb: Callable) -> None:
        if topic_id in self._node_to_ui_subs:
            return

        spec = self._spec("publish", topic_id)  # Node publisht => UI subscribt
        qos = self._content.qos(spec.qos_key)
        self._node_to_ui_subs[topic_id] = self.create_subscription(msg_type, spec.name, cb, qos)
        self.get_logger().info(f"[servo] SUB node->ui id={topic_id} topic={spec.name} qos={spec.qos_key}")

    def _pub(self, topic_id: str):
        p = self._ui_to_node_pubs.get(topic_id)
        if p is None:
            raise KeyError(f"[servo] Publisher '{topic_id}' fehlt (ensure_pub nicht gelaufen?)")
        return p

    # ─────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────

    def set_command_type(self, mode: str, *, force: bool = False) -> None:
        """
        publish auf set_mode (String).
        Akzeptiert: "joint"/"cart" oder direkt "JOINT_JOG"/"TWIST"/...
        """
        raw = (mode or "").strip().lower()
        if raw in ("joint", "j", "joint_jog"):
            txt = "JOINT_JOG"
        elif raw in ("cart", "cartesian", "twist", "t"):
            txt = "TWIST"
        else:
            txt = (mode or "").strip() or "JOINT_JOG"

        if (not force) and (txt == self._last_mode_txt):
            return
        self._last_mode_txt = txt

        self._pub("set_mode").publish(MsgString(data=txt))
        self.get_logger().info(f"[servo_bridge] PUB set_mode -> {txt} (force={force})")

    def set_frame_ui(self, frame_ui: str, *, force: bool = False) -> None:
        """
        frame_ui: "wrf"/"trf" (UI) oder "world"/"tcp"
        publish auf set_frame (String).
        """
        raw = (frame_ui or "").strip().lower() or "wrf"
        self._frame_ui = raw

        if raw in ("wrf", "world"):
            txt = "world"
        elif raw in ("trf", "tcp", "tool"):
            txt = "tcp"
        else:
            txt = raw

        if (not force) and (txt == self._last_frame_txt):
            return
        self._last_frame_txt = txt

        self._pub("set_frame").publish(MsgString(data=txt))
        self.get_logger().info(f"[servo_bridge] PUB set_frame -> {txt} (from {raw}, force={force})")

    # ─────────────────────────────────────────────────────────────
    # UI inbound (Signals aus Widgets)
    # ─────────────────────────────────────────────────────────────

    def _on_params_changed(self, cfg: dict) -> None:
        cfg = cfg or {}

        if "joint_step_deg" in cfg:
            self._joint_step_deg = float(cfg["joint_step_deg"])
        if "joint_speed_pct" in cfg:
            self._joint_speed_pct = float(cfg["joint_speed_pct"])

        if "cart_lin_step_mm" in cfg:
            self._cart_lin_step_mm = float(cfg["cart_lin_step_mm"])
        if "cart_ang_step_deg" in cfg:
            self._cart_ang_step_deg = float(cfg["cart_ang_step_deg"])
        if "cart_speed_mm_s" in cfg:
            self._cart_speed_mm_s = float(cfg["cart_speed_mm_s"])
        if "cart_speed_deg_s" in cfg:
            self._cart_speed_deg_s = float(cfg["cart_speed_deg_s"])

        if "frame" in cfg:
            self._frame_ui = str(cfg["frame"]).strip().lower() or self._frame_ui

    def _on_frame_changed(self, frame_ui: str) -> None:
        self.set_frame_ui(frame_ui, force=False)

    def _on_joint_jog_requested(self, joint_name: str, delta_deg: float, speed_pct: float) -> None:
        # ✅ For robustness: always publish mode on jog (wrapper may have restarted)
        self.set_command_type("joint", force=True)

        j = str(joint_name)
        ddeg = float(delta_deg)
        spct = float(speed_pct) if speed_pct is not None else self._joint_speed_pct

        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # Wrapper-Node setzt current_frame wenn leer
        msg.joint_names = [j]

        v = max(0.0, min(1.0, spct / 100.0))
        msg.velocities = [v]

        rad = math.radians(ddeg)
        msg.displacements = [rad]

        self._pub("joint_jog").publish(msg)
        self.get_logger().info(
            f"[servo_bridge] PUB joint_jog joint={j} delta_deg={ddeg:+.3f} rad={rad:+.6f} speed_pct={spct:.1f} v={v:.3f}"
        )

    def _on_cart_jog_requested(self, axis: str, delta: float, speed: float, frame_ui: str) -> None:
        # ✅ robust: always publish mode + frame on jog
        self.set_command_type("cart", force=True)
        self.set_frame_ui(frame_ui, force=True)

        a = (axis or "").strip().lower()
        d = float(delta)

        # NOTE:
        # - UI sends speed in mm/s for x/y/z
        # - UI sends speed in deg/s for rx/ry/rz
        s = float(speed) if speed is not None else (
            self._cart_speed_deg_s if a in ("rx", "ry", "rz") else self._cart_speed_mm_s
        )

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # Wrapper-Node setzt current_frame wenn leer

        if a in ("x", "y", "z"):
            lin_v = s / 1000.0  # mm/s -> m/s
            if a == "x":
                msg.twist.linear.x = math.copysign(lin_v, d)
            elif a == "y":
                msg.twist.linear.y = math.copysign(lin_v, d)
            else:
                msg.twist.linear.z = math.copysign(lin_v, d)

            self._pub("cartesian_mm").publish(msg)
            self.get_logger().info(
                f"[servo_bridge] PUB cartesian_mm axis={a} delta_mm={d:+.3f} speed_mm_s={s:.1f} "
                f"frame_ui={frame_ui} lin_v={lin_v:.4f}"
            )
            return

        if a in ("rx", "ry", "rz"):
            ang_v = math.radians(abs(s))  # deg/s -> rad/s
            if a == "rx":
                msg.twist.angular.x = math.copysign(ang_v, d)
            elif a == "ry":
                msg.twist.angular.y = math.copysign(ang_v, d)
            else:
                msg.twist.angular.z = math.copysign(ang_v, d)

            self._pub("cartesian_mm").publish(msg)
            self.get_logger().info(
                f"[servo_bridge] PUB cartesian_mm axis={a} delta_deg={d:+.3f} speed_deg_s={s:.1f} "
                f"frame_ui={frame_ui} ang_v={ang_v:.4f}"
            )
            return

        self.get_logger().warning(f"[servo_bridge] unknown cart axis: {axis!r}")

    # ─────────────────────────────────────────────────────────────
    # Inbound vom ROS-Servo-Wrapper (optional Debug)
    # ─────────────────────────────────────────────────────────────

    def _on_twist_out(self, msg: TwistStamped) -> None:
        self.signals._set_twist_out(msg)
        self.signals.twistOutChanged.emit(msg)

    def _on_joint_out(self, msg: JointJog) -> None:
        self.signals._set_joint_out(msg)
        self.signals.jointOutChanged.emit(msg)
