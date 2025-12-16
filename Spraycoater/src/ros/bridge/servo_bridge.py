# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, Callable, Type
import math

from PyQt6 import QtCore

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import String as MsgString
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

from config.startup import AppContent
from .base_bridge import BaseBridge


class ServoSignals(QtCore.QObject):
    """
    Qt-Signale (UI → ServoBridge)
    """
    # UI Widgets pushen Parameter in Bridge (optional)
    paramsChanged = QtCore.pyqtSignal(dict)

    # Joint jog: joint_name, delta_deg, speed_pct
    jointJogRequested = QtCore.pyqtSignal(str, float, float)

    # Cartesian jog: axis ("x","y","z","rx","ry","rz"), delta(mm|deg), speed(mm/s), frame_ui("wrf"/"trf")
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)

    # UI Frame selector: "wrf"/"trf"
    frameChanged = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        # Servo hat typischerweise keinen Cache, aber Contract muss existieren
        return


class ServoBridge(BaseBridge):
    """
    UI-ServoBridge:

    UI → Node Topics (Node subscribt):
      - cartesian_mm  (TwistStamped)
      - joint_jog     (JointJog)
      - set_mode      (String)   # z.B. "JOINT_JOG" / "TWIST"
      - set_frame     (String)   # z.B. "world" / "tcp" / "wrf"/"trf" (dein Servo-Node mappt das)

    WICHTIG:
      - NICHT twist_out/joint_out publishen! Das macht der ROS-Servo-Node selbst.
    """

    GROUP = "servo"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = ServoSignals()

        # eigene, korrekt gedrehte Verdrahtung
        self._ui_to_node_pubs: Dict[str, Any] = {}

        # Params (aus UI)
        self._cart_lin_step_mm: float = 1.0
        self._cart_ang_step_deg: float = 2.0
        self._cart_speed_mm_s: float = 50.0
        self._joint_step_deg: float = 2.0
        self._joint_speed_pct: float = 40.0

        # Frame (world/tool)
        self._frame_ui: str = "wrf"  # "wrf"|"trf"

        super().__init__("servo_bridge", content, namespace=namespace)

        s = self.signals
        s.paramsChanged.connect(self._on_params_changed)
        s.frameChanged.connect(self._on_frame_changed)
        s.jointJogRequested.connect(self._on_joint_jog_requested)
        s.cartesianJogRequested.connect(self._on_cart_jog_requested)

        # --- UI -> Node publishers (Node subscribt) ---
        self._ensure_pub("cartesian_mm", TwistStamped)
        self._ensure_pub("joint_jog", JointJog)
        self._ensure_pub("set_mode", MsgString)
        self._ensure_pub("set_frame", MsgString)

        self.get_logger().info(
            f"[servo] ServoBridge bereit (ns='{self.namespace or '/'}'): "
            "UI->Node pubs: cartesian_mm, joint_jog, set_mode, set_frame"
        )

    # ─────────────────────────────────────────────────────────────
    # Helpers: YAML spec → topic name + QoS
    # ─────────────────────────────────────────────────────────────

    def _resolve_ns_topic(self, name: str) -> str:
        name = (name or "").strip()
        if not name:
            return name
        if name.startswith("/"):
            return name

        ns = (self.namespace or "").strip().strip("/")
        if not ns:
            return "/" + name
        return f"/{ns}/{name}".replace("//", "/")

    def _qos_from_spec(self, spec: Any) -> QoSProfile:
        qos_id = getattr(spec, "qos", None) or getattr(spec, "qos_id", None) or "default"
        qos_id = str(qos_id).lower()

        if qos_id == "sensor_data":
            return qos_profile_sensor_data

        if qos_id == "latched":
            return QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _spec_from_content(self, direction: str, topic_id: str) -> Any:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _topic_name_from_content(self, direction: str, topic_id: str) -> str:
        spec = self._spec_from_content(direction, topic_id)
        raw_name = getattr(spec, "name", None) or getattr(spec, "topic", None) or ""
        return self._resolve_ns_topic(str(raw_name))

    def _ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        # UI -> Node: Publisher auf YAML 'subscribe'
        if topic_id in self._ui_to_node_pubs:
            return
        spec = self._spec_from_content("subscribe", topic_id)
        topic = self._topic_name_from_content("subscribe", topic_id)
        qos = self._qos_from_spec(spec)
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[servo] PUB ui->node id={topic_id} topic={topic}")

    def _pub(self, topic_id: str):
        p = self._ui_to_node_pubs.get(topic_id)
        if p is None:
            raise KeyError(f"[servo] Publisher '{topic_id}' fehlt (ensure_pub nicht gelaufen?)")
        return p

    # ─────────────────────────────────────────────────────────────
    # Public API (von UIBridge genutzt)
    # ─────────────────────────────────────────────────────────────

    def set_command_type(self, mode: str) -> None:
        """
        UIBridge.servo_set_command_type("joint"|"cart") ruft das.
        Wir publishen auf set_mode als String.
        """
        raw = (mode or "").strip().lower()
        if raw in ("joint", "j", "joint_jog"):
            txt = "JOINT_JOG"
        elif raw in ("cart", "cartesian", "twist", "t"):
            txt = "TWIST"
        else:
            txt = mode.strip() or "JOINT_JOG"

        self._pub("set_mode").publish(MsgString(data=txt))

    # ─────────────────────────────────────────────────────────────
    # UI inbound (Signals aus Widgets)
    # ─────────────────────────────────────────────────────────────

    def _on_params_changed(self, cfg: dict) -> None:
        cfg = cfg or {}

        # Joint params
        if "joint_step_deg" in cfg:
            self._joint_step_deg = float(cfg["joint_step_deg"])
        if "joint_speed_pct" in cfg:
            self._joint_speed_pct = float(cfg["joint_speed_pct"])

        # Cart params
        if "cart_lin_step_mm" in cfg:
            self._cart_lin_step_mm = float(cfg["cart_lin_step_mm"])
        if "cart_ang_step_deg" in cfg:
            self._cart_ang_step_deg = float(cfg["cart_ang_step_deg"])
        if "cart_speed_mm_s" in cfg:
            self._cart_speed_mm_s = float(cfg["cart_speed_mm_s"])

        # Frame
        if "frame" in cfg:
            self._frame_ui = str(cfg["frame"]).strip().lower() or self._frame_ui

    def _on_frame_changed(self, frame_ui: str) -> None:
        self._frame_ui = (frame_ui or "").strip().lower() or "wrf"

        # an den ROS-Servo Node: "world"/"tcp" (oder du lässt "wrf"/"trf" durch)
        # dein Servo-Node kann world/tcp – wir mappen:
        txt = "world" if self._frame_ui == "wrf" else "tcp"
        self._pub("set_frame").publish(MsgString(data=txt))

    def _on_joint_jog_requested(self, joint_name: str, delta_deg: float, speed_pct: float) -> None:
        # Mode auf JOINT_JOG setzen
        self.set_command_type("joint")

        # JointJog message
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [str(joint_name)]
        # velocity = speed_pct/100 (als einfache Normierung)
        v = max(0.0, min(1.0, float(speed_pct) / 100.0))
        msg.velocities = [v]
        # displacement in rad
        msg.displacements = [math.radians(float(delta_deg))]
        self._pub("joint_jog").publish(msg)

    def _on_cart_jog_requested(self, axis: str, delta: float, speed: float, frame_ui: str) -> None:
        # Mode auf TWIST setzen
        self.set_command_type("cart")

        # Frame setzen (wrf/trf)
        self._on_frame_changed(frame_ui)

        a = (axis or "").strip().lower()
        d = float(delta)
        s = float(speed)

        # TwistStamped in m/s bzw rad/s:
        # - Linear delta ist mm pro step -> wir senden "velocity-like" proportional zu speed
        # - Rot delta ist deg pro step -> wir senden angular proportional
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ""  # ServoNode setzt falls leer seinen current_frame

        # Linear: mm -> m
        lin_v = (s / 1000.0)  # mm/s -> m/s
        ang_v = math.radians(max(0.0, s))  # grob: speed als deg/s interpretiert

        if a == "x":
            msg.twist.linear.x = math.copysign(lin_v, d)
        elif a == "y":
            msg.twist.linear.y = math.copysign(lin_v, d)
        elif a == "z":
            msg.twist.linear.z = math.copysign(lin_v, d)
        elif a == "rx":
            msg.twist.angular.x = math.copysign(ang_v, d)
        elif a == "ry":
            msg.twist.angular.y = math.copysign(ang_v, d)
        elif a == "rz":
            msg.twist.angular.z = math.copysign(ang_v, d)
        else:
            self.get_logger().warning(f"[servo] unknown cart axis: {axis!r}")
            return

        self._pub("cartesian_mm").publish(msg)
