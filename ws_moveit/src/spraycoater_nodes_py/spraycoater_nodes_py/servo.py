#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, List

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from moveit_msgs.srv import ServoCommandType

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY = "servo"


@dataclass
class _Counters:
    rx_twist: int = 0
    rx_joint: int = 0
    rx_set_mode: int = 0
    rx_set_frame: int = 0
    tx_twist: int = 0
    tx_joint: int = 0
    last_rx_wall: float = 0.0


class Servo(Node):
    """
    ROS-Servo-Wrapper-Node (UI -> MoveIt Servo /<ns>/servo_node).

    UI in (subscribe):
      - servo/cartesian_mm (TwistStamped)
      - servo/joint_jog    (JointJog)
      - servo/set_mode     (String)         -> calls switch_command_type
      - servo/set_frame    (String)

    OUT (publish):
      - servo/delta_twist_cmds   (TwistStamped)
      - servo/delta_joint_cmds   (JointJog)

    IMPORTANT:
      MoveIt Servo MUST subscribe to these OUT topics (in same namespace),
      otherwise out_subs stays 0.
    """

    MODE_MAP = {
        "JOINT_JOG": ServoCommandType.Request.JOINT_JOG,
        "JOINT":     ServoCommandType.Request.JOINT_JOG,
        "J":         ServoCommandType.Request.JOINT_JOG,

        "TWIST":     ServoCommandType.Request.TWIST,
        "CARTESIAN": ServoCommandType.Request.TWIST,
        "T":         ServoCommandType.Request.TWIST,

        "POSE":      ServoCommandType.Request.POSE,
        "P":         ServoCommandType.Request.POSE,
    }

    def __init__(self) -> None:
        super().__init__("servo")

        # ---------------- Params
        self.declare_parameter("backend", "default")
        self.backend = self.get_parameter("backend").get_parameter_value().string_value or "default"

        # ✅ default: RELATIV (sauber in /shadow oder /live)
        # override: "auto" oder absolute "/shadow/servo_node/switch_command_type"
        self.declare_parameter("switch_service", "servo_node/switch_command_type")
        self.switch_service_param = (self.get_parameter("switch_service").value or "").strip() or "servo_node/switch_command_type"

        self.declare_parameter("ui_twist_is_mm_deg", True)
        self.ui_twist_is_mm_deg = bool(self.get_parameter("ui_twist_is_mm_deg").value)

        self.declare_parameter("default_mode", "JOINT_JOG")
        self.declare_parameter("auto_set_mode_on_rx", True)
        self.default_mode = (self.get_parameter("default_mode").value or "JOINT_JOG").strip().upper()
        self.auto_set_mode_on_rx = bool(self.get_parameter("auto_set_mode_on_rx").value)

        self.declare_parameter("log_steps", True)
        self.declare_parameter("log_every_n", 1)
        self.declare_parameter("warn_no_downstream", True)
        self.declare_parameter("status_rate_hz", 1.0)

        self.log_steps = bool(self.get_parameter("log_steps").value)
        self.log_every_n = int(self.get_parameter("log_every_n").value or 1)
        self.warn_no_downstream = bool(self.get_parameter("warn_no_downstream").value)
        self.status_rate_hz = float(self.get_parameter("status_rate_hz").value or 0.0)
        if self.log_every_n < 1:
            self.log_every_n = 1

        # Warn throttling
        self._last_switch_warn = 0.0
        self._warn_every_s = 3.0

        # ---------------- Config hub
        self.loader = topics()
        self.frames_cfg = frames()
        self._F = self.frames_cfg.resolve
        self.frame_tcp = self._F(self.frames_cfg.get("tcp", "tcp"))
        self.frame_world = self._F(self.frames_cfg.get("world", "world"))

        self.current_frame = self.frame_tcp
        self.last_ui_mode: str = "UNKNOWN"
        self.current_mode: Optional[int] = None

        # pending-mode retry
        self._pending_mode: Optional[int] = None
        self._pending_label: str = ""
        self._switch_inflight: bool = False

        # switch client
        self.switch_service: str = ""
        self.switch_client = None

        # --- topics ---
        self.t_ui_twist = self.loader.subscribe_topic(NODE_KEY, "cartesian_mm")
        q_ui_twist = self.loader.qos_by_id("subscribe", NODE_KEY, "cartesian_mm")

        self.t_ui_joint = self.loader.subscribe_topic(NODE_KEY, "joint_jog")
        q_ui_joint = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_jog")

        self.t_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        q_set_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        self.t_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        q_set_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        self.t_twist_out = self.loader.publish_topic(NODE_KEY, "twist_out")
        q_twist_out = self.loader.qos_by_id("publish", NODE_KEY, "twist_out")

        self.t_joint_out = self.loader.publish_topic(NODE_KEY, "joint_out")
        q_joint_out = self.loader.qos_by_id("publish", NODE_KEY, "joint_out")

        self.pub_twist = self.create_publisher(TwistStamped, self.t_twist_out, q_twist_out)
        self.pub_joint = self.create_publisher(JointJog, self.t_joint_out, q_joint_out)

        self.create_subscription(TwistStamped, self.t_ui_twist, self._on_ui_twist, q_ui_twist)
        self.create_subscription(JointJog, self.t_ui_joint, self._on_ui_joint, q_ui_joint)
        self.create_subscription(String, self.t_set_mode, self._on_set_mode, q_set_mode)
        self.create_subscription(String, self.t_set_frame, self._on_set_frame, q_set_frame)

        self._cnt = _Counters()

        # bind client (non-blocking)
        self._bind_switch_client(force=True)
        self.create_timer(0.5, self._bind_switch_client)

        # Retry only if pending exists
        self.create_timer(0.1, self._retry_pending_mode)

        # Default mode once (pending) – but no spam until service is ready
        if self.default_mode in self.MODE_MAP:
            self._pending_mode = self.MODE_MAP[self.default_mode]
            self._pending_label = self.default_mode

        self.get_logger().info(
            f"✅ Servo Wrapper aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}')\n"
            f"   UI in:  {self.t_ui_twist}, {self.t_ui_joint}, {self.t_set_mode}, {self.t_set_frame}\n"
            f"   OUT:    {self.t_twist_out}, {self.t_joint_out}\n"
            f"   switch_service(param)='{self.switch_service_param}' bound='{self.switch_service or '<pending>'}'"
        )

        if self.status_rate_hz and self.status_rate_hz > 0.0:
            self.create_timer(1.0 / self.status_rate_hz, self._status_tick)

    # -------------------------------------------------------------------------
    # Service binding
    # -------------------------------------------------------------------------

    def _resolve_service_name(self, raw_srv: str) -> str:
        raw_srv = (raw_srv or "").strip()
        if not raw_srv:
            return ""
        if raw_srv.startswith("/"):
            return raw_srv
        ns = (self.get_namespace() or "").rstrip("/")
        return f"{ns}/{raw_srv}" if ns else f"/{raw_srv}"

    def _find_switch_services(self) -> List[str]:
        out: List[str] = []
        try:
            for name, types in self.get_service_names_and_types():
                if not name.endswith("switch_command_type"):
                    continue
                if any(t == "moveit_msgs/srv/ServoCommandType" or t.endswith("/ServoCommandType") for t in types):
                    out.append(name)
        except Exception:
            pass
        return sorted(set(out))

    def _choose_best_service(self, candidates: List[str]) -> str:
        if not candidates:
            return ""
        ns = (self.get_namespace() or "").rstrip("/")
        preferred = f"{ns}/servo_node/switch_command_type" if ns else "/servo_node/switch_command_type"
        if preferred in candidates:
            return preferred
        if ns:
            in_ns = [c for c in candidates if c.startswith(ns + "/")]
            if in_ns:
                return in_ns[0]
        return candidates[0]

    def _bind_switch_client(self, force: bool = False) -> None:
        want = (self.switch_service_param or "").strip()
        want_auto = (want.lower() == "auto")

        if self.switch_client and self.switch_client.service_is_ready() and not force:
            return

        chosen = ""
        candidates = self._find_switch_services()

        if want_auto:
            chosen = self._choose_best_service(candidates)
        else:
            chosen = self._resolve_service_name(want)
            if candidates and chosen not in candidates:
                chosen = self._choose_best_service(candidates)

        if not chosen:
            chosen = self._resolve_service_name("servo_node/switch_command_type")

        if (not force) and chosen == self.switch_service and self.switch_client is not None:
            return

        self.switch_service = chosen
        try:
            self.switch_client = self.create_client(ServoCommandType, self.switch_service)
            self.get_logger().info(f"🔧 bind switch_command_type -> '{self.switch_service}'")
        except Exception as e:
            self.switch_client = None
            self.get_logger().warning(f"switch client create failed: {e}")

    # -------------------------------------------------------------------------
    # Helpers
    # -------------------------------------------------------------------------

    def _should_log_rx(self, n: int) -> bool:
        return self.log_steps and (n % self.log_every_n == 0)

    def _subs(self, pub) -> int:
        try:
            return int(pub.get_subscription_count())
        except Exception:
            return -1

    def _warn_if_no_downstream(self, pub, topic_name: str) -> None:
        if not self.warn_no_downstream:
            return
        if self._subs(pub) == 0:
            self.get_logger().warning(
                f"⚠️ downstream hat 0 Subscriber: {topic_name}\n"
                f"   -> Das heißt: MoveIt Servo subscribed NICHT auf diesem Topic.\n"
                f"      Prüfe moveit_servo cartesian_command_in_topic/joint_command_in_topic "
                f"im *gleichen Namespace*."
            )

    # -------------------------------------------------------------------------
    # switch_command_type (robust + throttled warnings)
    # -------------------------------------------------------------------------

    def _retry_pending_mode(self) -> None:
        if self._pending_mode is None:
            return
        self._request_mode(self._pending_mode, self._pending_label or "PENDING")

    def _request_mode(self, cmd_type: int, label: str) -> None:
        if self.current_mode == cmd_type:
            self._pending_mode = None
            self._pending_label = ""
            return
        if self._switch_inflight:
            return

        self._bind_switch_client()

        if not self.switch_client or not self.switch_client.service_is_ready():
            self._pending_mode = cmd_type
            self._pending_label = label

            now = time.time()
            if now - self._last_switch_warn > self._warn_every_s:
                self._last_switch_warn = now
                cands = self._find_switch_services()
                self.get_logger().warning(
                    f"switch_command_type Service nicht ready: '{self.switch_service}' (retrying)\n"
                    f"  gefunden: {cands if cands else '[]'}\n"
                    f"  HINWEIS: wenn du gleichzeitig /servo_node und /shadow/servo_node hast, "
                    f"ist der Stack doppelt gestartet."
                )
            return

        req = ServoCommandType.Request()
        req.command_type = cmd_type

        self._switch_inflight = True
        fut = self.switch_client.call_async(req)

        def _done(_f):
            self._switch_inflight = False
            try:
                resp = _f.result()
            except Exception as e:
                self.get_logger().warning(f"switch_command_type exception: {e} (retrying)")
                self._pending_mode = cmd_type
                self._pending_label = label
                return

            success = bool(getattr(resp, "success", False)) if resp is not None else False
            if success:
                self.current_mode = cmd_type
                self._pending_mode = None
                self._pending_label = ""
                self.get_logger().info(f"✅ switch_command_type OK -> {label} ({cmd_type})")
            else:
                self.get_logger().warning(f"❌ switch_command_type failed -> {label} ({cmd_type}) (retrying)")
                self._pending_mode = cmd_type
                self._pending_label = label

        fut.add_done_callback(_done)

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_ui_twist(self, msg: TwistStamped) -> None:
        self._cnt.rx_twist += 1
        self._cnt.last_rx_wall = time.time()

        if self.auto_set_mode_on_rx:
            self._request_mode(ServoCommandType.Request.TWIST, "TWIST")

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.twist = msg.twist

        if self.ui_twist_is_mm_deg:
            out.twist.linear.x *= 0.001
            out.twist.linear.y *= 0.001
            out.twist.linear.z *= 0.001
            out.twist.angular.x *= (math.pi / 180.0)
            out.twist.angular.y *= (math.pi / 180.0)
            out.twist.angular.z *= (math.pi / 180.0)

        self.pub_twist.publish(out)
        self._cnt.tx_twist += 1

        if self._should_log_rx(self._cnt.rx_twist):
            self.get_logger().info(
                f"[RX twist #{self._cnt.rx_twist}] -> '{self.t_twist_out}' (subs={self._subs(self.pub_twist)})"
            )

        self._warn_if_no_downstream(self.pub_twist, self.t_twist_out)

    def _on_ui_joint(self, msg: JointJog) -> None:
        self._cnt.rx_joint += 1
        self._cnt.last_rx_wall = time.time()

        if self.auto_set_mode_on_rx:
            self._request_mode(ServoCommandType.Request.JOINT_JOG, "JOINT_JOG")

        out = JointJog()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.joint_names = list(msg.joint_names)
        out.velocities = list(msg.velocities)
        out.displacements = list(msg.displacements)
        out.duration = msg.duration

        self.pub_joint.publish(out)
        self._cnt.tx_joint += 1

        if self._should_log_rx(self._cnt.rx_joint):
            self.get_logger().info(
                f"[RX joint #{self._cnt.rx_joint}] -> '{self.t_joint_out}' (subs={self._subs(self.pub_joint)})"
            )

        self._warn_if_no_downstream(self.pub_joint, self.t_joint_out)

    def _on_set_mode(self, msg: String) -> None:
        self._cnt.rx_set_mode += 1
        self._cnt.last_rx_wall = time.time()

        raw = (msg.data or "").strip().upper()
        if not raw:
            return

        if raw not in self.MODE_MAP:
            self.get_logger().warning(f"[set_mode] Unbekannt: {msg.data!r} (TWIST/JOINT_JOG/POSE)")
            return

        self._request_mode(self.MODE_MAP[raw], raw)

    def _on_set_frame(self, msg: String) -> None:
        self._cnt.rx_set_frame += 1
        self._cnt.last_rx_wall = time.time()

        raw = (msg.data or "").strip().lower()
        if raw == "world":
            self.current_frame = self.frame_world
            self.get_logger().info(f"[set_frame] world -> '{self.current_frame}'")
        elif raw == "tcp":
            self.current_frame = self.frame_tcp
            self.get_logger().info(f"[set_frame] tcp -> '{self.current_frame}'")
        else:
            self.get_logger().warning(f"[set_frame] Unbekannt: {msg.data!r} (world|tcp)")

    # -------------------------------------------------------------------------
    # Status
    # -------------------------------------------------------------------------

    def _status_tick(self) -> None:
        age = (time.time() - self._cnt.last_rx_wall) if self._cnt.last_rx_wall else -1.0
        subs_tw = self._subs(self.pub_twist)
        subs_j = self._subs(self.pub_joint)
        srv_ready = "ready" if (self.switch_client and self.switch_client.service_is_ready()) else "not_ready"
        self.get_logger().info(
            f"[status] out_subs(twist={subs_tw}, joint={subs_j}) last_rx_age={age:.2f}s "
            f"switch_srv={srv_ready} switch_name='{self.switch_service}' pending={self._pending_mode if self._pending_mode is not None else -1}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = Servo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
