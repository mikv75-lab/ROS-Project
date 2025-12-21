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
    ROS-Servo-Wrapper-Node (UI -> MoveIt Servo /servo_node):

      UI in (topics.yaml / NODE_KEY='servo' / subscribe):
        - servo/cartesian_mm   (TwistStamped)
        - servo/joint_jog      (JointJog)
        - servo/set_mode       (String)         -> calls switch_command_type
        - servo/set_frame      (String)

      OUT (topics.yaml / NODE_KEY='servo' / publish):
        - servo/delta_twist_cmds
        - servo/delta_joint_cmds

    IMPORTANT:
      - MoveIt Servo muss auf DIESE output-topics hören, sonst bleibt out_subs=0.
        In moveit_servo params:
          cartesian_command_in_topic := "servo/delta_twist_cmds"
          joint_command_in_topic     := "servo/delta_joint_cmds"
    """

    MODE_MAP = {
        # joint jog
        "JOINT_JOG": ServoCommandType.Request.JOINT_JOG,
        "JOINT":     ServoCommandType.Request.JOINT_JOG,
        "J":         ServoCommandType.Request.JOINT_JOG,

        # twist
        "TWIST":     ServoCommandType.Request.TWIST,
        "CARTESIAN": ServoCommandType.Request.TWIST,
        "T":         ServoCommandType.Request.TWIST,

        # pose
        "POSE":      ServoCommandType.Request.POSE,
        "P":         ServoCommandType.Request.POSE,
    }

    def __init__(self) -> None:
        super().__init__("servo")

        # ---------------- Params
        self.declare_parameter("backend", "default")
        self.backend = self.get_parameter("backend").get_parameter_value().string_value or "default"

        # Default: RELATIV und korrekt zur ComposableNode name="servo_node"
        # => /<ns>/servo_node/switch_command_type
        self.declare_parameter("switch_service", "servo_node/switch_command_type")
        raw_srv = (self.get_parameter("switch_service").get_parameter_value().string_value or "").strip()
        if not raw_srv:
            raw_srv = "servo_node/switch_command_type"

        # UI Units: cartesian_mm ist mm/s + deg/s -> konvertiere zu m/s + rad/s
        self.declare_parameter("ui_twist_is_mm_deg", True)
        self.ui_twist_is_mm_deg = bool(self.get_parameter("ui_twist_is_mm_deg").value)

        # Robustness: wenn UI set_mode nicht kommt -> Default + Auto
        self.declare_parameter("default_mode", "JOINT_JOG")   # JOINT_JOG | TWIST | POSE
        self.declare_parameter("auto_set_mode_on_rx", True)   # wenn Twist/Joint kommt, Mode passend setzen
        self.default_mode = (self.get_parameter("default_mode").value or "JOINT_JOG").strip().upper()
        self.auto_set_mode_on_rx = bool(self.get_parameter("auto_set_mode_on_rx").value)

        # Logging / Debug
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
        self._last_switch_warn: float = 0.0

        # switch service handling
        self._switch_service_param = raw_srv  # keep original for status
        self.switch_service: str = ""         # final resolved name
        self.switch_client = None

        # --- subscribe: UI topics ---
        self.t_ui_twist = self.loader.subscribe_topic(NODE_KEY, "cartesian_mm")
        q_ui_twist = self.loader.qos_by_id("subscribe", NODE_KEY, "cartesian_mm")

        self.t_ui_joint = self.loader.subscribe_topic(NODE_KEY, "joint_jog")
        q_ui_joint = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_jog")

        self.t_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        q_set_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        self.t_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        q_set_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        # --- publish: moveit_servo inputs (delta_*) ---
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

        # Resolve/create service client
        self._init_switch_client(raw_srv)

        # Retry Timer (zieht pending-mode nach, wenn Service ready wird)
        self.create_timer(0.1, self._retry_pending_mode)

        # Default-Mode beim Start setzen
        if self.default_mode in self.MODE_MAP:
            self._pending_mode = self.MODE_MAP[self.default_mode]
            self._pending_label = self.default_mode

        self.get_logger().info(
            f"✅ Servo Wrapper aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}')\n"
            f"   UI in:  {self.t_ui_twist}, {self.t_ui_joint}, {self.t_set_mode}, {self.t_set_frame}\n"
            f"   OUT:    {self.t_twist_out}, {self.t_joint_out}\n"
            f"   switch_service(param)='{self._switch_service_param}' resolved='{self.switch_service or '<pending>'}'\n"
            f"   ui_twist_is_mm_deg={self.ui_twist_is_mm_deg}\n"
            f"   default_mode={self.default_mode} auto_set_mode_on_rx={self.auto_set_mode_on_rx}\n"
            f"   log_steps={self.log_steps} log_every_n={self.log_every_n} warn_no_downstream={self.warn_no_downstream}"
        )

        if self.status_rate_hz and self.status_rate_hz > 0.0:
            self.create_timer(1.0 / self.status_rate_hz, self._status_tick)

    # -------------------------------------------------------------------------
    # Service discovery / client init
    # -------------------------------------------------------------------------

    def _resolve_service_name(self, raw_srv: str) -> str:
        """If raw_srv is relative, resolve it against the node namespace."""
        raw_srv = (raw_srv or "").strip()
        if not raw_srv:
            return ""
        if raw_srv.startswith("/"):
            return raw_srv
        ns = (self.get_namespace() or "").rstrip("/")
        return f"{ns}/{raw_srv}" if ns else f"/{raw_srv}"

    def _find_switch_services(self) -> List[str]:
        """Find ServoCommandType services ending with 'switch_command_type'."""
        matches: List[str] = []
        try:
            for name, types in self.get_service_names_and_types():
                if not name.endswith("switch_command_type"):
                    continue
                if "__unset__" in name:
                    continue
                ok = any(
                    t == "moveit_msgs/srv/ServoCommandType" or t.endswith("/ServoCommandType")
                    for t in types
                )
                if ok:
                    matches.append(name)
        except Exception:
            pass
        return matches

    def _choose_best_service(self, candidates: List[str]) -> str:
        """Prefer a candidate within our namespace first, otherwise shortest absolute."""
        if not candidates:
            return ""
        ns = (self.get_namespace() or "").rstrip("/")
        if ns:
            in_ns = [c for c in candidates if c.startswith(ns + "/")]
            if in_ns:
                return sorted(in_ns, key=len)[0]
        return sorted(candidates, key=len)[0]

    def _init_switch_client(self, raw_srv: str) -> None:
        raw = (raw_srv or "").strip()
        if raw.lower() == "auto":
            candidates = self._find_switch_services()
            chosen = self._choose_best_service(candidates)
            self.switch_service = chosen
        else:
            self.switch_service = self._resolve_service_name(raw)

        # Client erstellen (auch wenn noch nicht ready)
        if not self.switch_service:
            # fallback (sollte praktisch nicht mehr passieren)
            self.switch_service = self._resolve_service_name("servo_node/switch_command_type")

        self.switch_client = self.create_client(ServoCommandType, self.switch_service)

        # Auto- oder Not-ready: regelmäßig prüfen/rebind
        self.create_timer(0.5, self._maybe_rebind_switch_client)

    def _maybe_rebind_switch_client(self) -> None:
        """If 'auto' OR service not ready, try to discover/rebind."""
        want_auto = (self._switch_service_param or "").strip().lower() == "auto"

        # wenn nicht auto: nur warten bis ready
        if not want_auto:
            return

        if self.switch_client and self.switch_client.service_is_ready():
            return

        candidates = self._find_switch_services()
        chosen = self._choose_best_service(candidates)
        if not chosen:
            return
        if chosen == self.switch_service:
            return

        self.switch_service = chosen
        try:
            self.switch_client = self.create_client(ServoCommandType, self.switch_service)
            self.get_logger().info(f"🔎 auto switch_service gefunden -> '{self.switch_service}'")
        except Exception as e:
            self.get_logger().warning(f"auto rebind failed: {e}")

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
        subs = self._subs(pub)
        if subs == 0:
            self.get_logger().warning(
                f"⚠️ downstream hat 0 Subscriber: {topic_name}\n"
                f"   -> Check: moveit_servo cartesian_command_in_topic/joint_command_in_topic\n"
                f"      müssen auf '{self.t_twist_out}' und '{self.t_joint_out}' zeigen."
            )

    # -------------------------------------------------------------------------
    # switch_command_type (robust + retry)
    # -------------------------------------------------------------------------

    def _set_pending(self, cmd_type: int, label: str) -> None:
        self._pending_mode = cmd_type
        self._pending_label = label

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

        if not self.switch_client or not self.switch_client.service_is_ready():
            self._set_pending(cmd_type, label)
            now = time.time()
            if now - self._last_switch_warn > 2.0:
                self._last_switch_warn = now
                self.get_logger().warning(
                    f"switch_command_type Service nicht ready: '{self.switch_service or '<unknown>'}' (retrying)"
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
                self._set_pending(cmd_type, label)
                return

            success = bool(getattr(resp, "success", False)) if resp is not None else False
            if success:
                self.current_mode = cmd_type
                self._pending_mode = None
                self._pending_label = ""
                self.get_logger().info(f"✅ switch_command_type OK -> {label} ({cmd_type})")
            else:
                self.get_logger().warning(f"❌ switch_command_type failed -> {label} ({cmd_type}) (retrying)")
                self._set_pending(cmd_type, label)

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
            subs = self._subs(self.pub_twist)
            self.get_logger().info(
                f"[RX twist #{self._cnt.rx_twist}] in='{self.t_ui_twist}' "
                f"frame='{out.header.frame_id}' "
                f"lin(m/s)=({out.twist.linear.x:.4f},{out.twist.linear.y:.4f},{out.twist.linear.z:.4f}) "
                f"ang(rad/s)=({out.twist.angular.x:.4f},{out.twist.angular.y:.4f},{out.twist.angular.z:.4f}) "
                f"-> '{self.t_twist_out}' (subs={subs})"
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
            subs = self._subs(self.pub_joint)
            j = out.joint_names[0] if out.joint_names else "?"
            d = out.displacements[0] if out.displacements else 0.0
            v = out.velocities[0] if out.velocities else 0.0
            self.get_logger().info(
                f"[RX joint #{self._cnt.rx_joint}] in='{self.t_ui_joint}' joint='{j}' "
                f"disp(rad)={d:+.6f} vel(rad/s)={v:.3f} frame='{out.header.frame_id}' "
                f"-> '{self.t_joint_out}' (subs={subs})"
            )

        self._warn_if_no_downstream(self.pub_joint, self.t_joint_out)

    def _on_set_mode(self, msg: String) -> None:
        self._cnt.rx_set_mode += 1
        self._cnt.last_rx_wall = time.time()

        raw = (msg.data or "").strip().upper()
        if not raw:
            return
        self.last_ui_mode = raw

        if raw not in self.MODE_MAP:
            self.get_logger().warning(f"[set_mode] Unbekannt: {msg.data!r} (z.B. TWIST/JOINT_JOG/POSE)")
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
            self.get_logger().warning(f"[set_frame] Unbekannt: {msg.data!r} (expected 'world'|'tcp')")

    # -------------------------------------------------------------------------
    # Status ticker
    # -------------------------------------------------------------------------

    def _status_tick(self) -> None:
        age = (time.time() - self._cnt.last_rx_wall) if self._cnt.last_rx_wall else -1.0
        subs_tw = self._subs(self.pub_twist)
        subs_j = self._subs(self.pub_joint)
        srv_ready = "ready" if (self.switch_client and self.switch_client.service_is_ready()) else "not_ready"
        mode = self.current_mode if self.current_mode is not None else -1
        pending = self._pending_mode if self._pending_mode is not None else -1
        self.get_logger().info(
            f"[status] rx_twist={self._cnt.rx_twist} rx_joint={self._cnt.rx_joint} "
            f"tx_twist={self._cnt.tx_twist} tx_joint={self._cnt.tx_joint} "
            f"out_subs(twist={subs_tw}, joint={subs_j}) last_rx_age={age:.2f}s "
            f"switch_srv={srv_ready} switch_name='{self.switch_service}' "
            f"mode={mode} pending={pending} ui_mode={self.last_ui_mode}"
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
