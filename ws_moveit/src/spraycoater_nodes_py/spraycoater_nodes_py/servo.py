#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import time
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from std_srvs.srv import Trigger

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
    ROS-Servo-Wrapper-Node (UI -> MoveIt Servo):

      UI in:
        - servo/cartesian_mm   (geometry_msgs/TwistStamped)   -> forwarded to delta_twist_cmds
        - servo/joint_jog      (control_msgs/JointJog)        -> forwarded to delta_joint_cmds
        - servo/set_mode       (std_msgs/String)              -> nur Logging/Status (kein Switch-Service)
        - servo/set_frame      (std_msgs/String)              -> 'world'|'tcp' -> setzt header.frame_id Default

      OUT (MoveIt Servo inputs):
        - servo/delta_twist_cmds
        - servo/delta_joint_cmds

    MoveIt2 Humble:
      Servo wird typischerweise per Service gestartet:
        /<ns>/servo_node/start_servo   (std_srvs/Trigger)
      Daher: wir unterstützen optional auto-start über Trigger.
    """

    def __init__(self) -> None:
        super().__init__("servo")

        # ---------------- Params
        self.declare_parameter("backend", "default")
        self.backend = self.get_parameter("backend").get_parameter_value().string_value or "default"

        # MoveIt Servo start service (relativ, damit Namespace greift -> /shadow/servo_node/start_servo)
        self.declare_parameter("start_servo_service", "servo_node/start_servo")
        self.start_service_name = (
            self.get_parameter("start_servo_service").get_parameter_value().string_value
            or "servo_node/start_servo"
        )

        # auto-start Servo (Trigger) sobald Service da ist
        self.declare_parameter("auto_start_servo", True)
        self.auto_start_servo = bool(self.get_parameter("auto_start_servo").value)

        # Logging / Debug
        self.declare_parameter("log_steps", True)            # loggt jede RX/TX Aktion
        self.declare_parameter("log_every_n", 1)             # nur jede n-te RX Nachricht loggen
        self.declare_parameter("warn_no_downstream", True)   # warn wenn delta_* keine Subscriber hat
        self.declare_parameter("status_rate_hz", 1.0)        # 0 deaktiviert

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

        # Default frame (wenn UI keinen frame_id sendet)
        self.current_frame = self.frame_tcp

        # optional: nur für Status/Debug
        self.last_ui_mode: str = "UNKNOWN"

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

        # --- MoveIt Servo start service client (Trigger) ---
        self.start_client = self.create_client(Trigger, self.start_service_name)
        self._servo_started: bool = False
        self._start_inflight: bool = False
        self._last_start_warn: float = 0.0
        self._last_start_try: float = 0.0

        self._cnt = _Counters()

        self.get_logger().info(
            f"✅ Servo aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}')\n"
            f"   UI in:  {self.t_ui_twist}, {self.t_ui_joint}, {self.t_set_mode}, {self.t_set_frame}\n"
            f"   OUT:    {self.t_twist_out}, {self.t_joint_out}\n"
            f"   start_service='{self.start_service_name}' auto_start_servo={self.auto_start_servo}\n"
            f"   log_steps={self.log_steps} log_every_n={self.log_every_n} warn_no_downstream={self.warn_no_downstream}"
        )

        # Auto-start ticker (bis gestartet)
        if self.auto_start_servo:
            self.create_timer(0.5, self._auto_start_tick)

        # Status ticker
        if self.status_rate_hz and self.status_rate_hz > 0.0:
            period = 1.0 / self.status_rate_hz
            self.create_timer(period, self._status_tick)

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
                f"⚠️ downstream hat 0 Subscriber: {topic_name} "
                f"(MoveIt Servo hört evtl. auf einem anderen Topic!)"
            )

    # -------------------------------------------------------------------------
    # MoveIt Servo start (Trigger)
    # -------------------------------------------------------------------------

    def _auto_start_tick(self) -> None:
        """Periodischer Versuch, MoveIt Servo zu starten, bis success."""
        if self._servo_started:
            return
        self._try_start_servo()

    def _ensure_servo_started(self) -> None:
        """Wird bei eingehenden Commands aufgerufen (best-effort)."""
        if self._servo_started:
            return
        if not self.auto_start_servo:
            # auch wenn auto-start aus: einmal pro Sekunde best-effort versuchen,
            # falls Nutzer trotzdem will, dass wir automatisch starten sobald service auftaucht
            now = time.time()
            if now - self._last_start_try > 1.0:
                self._try_start_servo()
            return
        # auto-start tick übernimmt, aber wir dürfen hier auch anstoßen
        self._try_start_servo()

    def _try_start_servo(self) -> None:
        if self._servo_started or self._start_inflight:
            return

        self._last_start_try = time.time()

        if not self.start_client.service_is_ready():
            now = time.time()
            if now - self._last_start_warn > 2.0:
                self._last_start_warn = now
                self.get_logger().warning(
                    f"start_servo Service noch nicht ready: '{self.start_service_name}' "
                    f"(wenn MoveIt Servo läuft, sollte der Service existieren)"
                )
            return

        req = Trigger.Request()
        self._start_inflight = True
        fut = self.start_client.call_async(req)

        def _done(_f):
            self._start_inflight = False
            try:
                resp = _f.result()
            except Exception as e:
                self.get_logger().warning(f"start_servo exception: {e}")
                return

            if resp and getattr(resp, "success", False):
                self._servo_started = True
                self.get_logger().info(f"✅ start_servo OK: {getattr(resp, 'message', '')}")
            else:
                self.get_logger().warning(
                    f"❌ start_servo failed: {getattr(resp, 'message', '') if resp else 'no response'}"
                )

        fut.add_done_callback(_done)

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _on_ui_twist(self, msg: TwistStamped) -> None:
        self._cnt.rx_twist += 1
        self._cnt.last_rx_wall = time.time()

        self._ensure_servo_started()

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.twist = msg.twist

        self.pub_twist.publish(out)
        self._cnt.tx_twist += 1

        if self._should_log_rx(self._cnt.rx_twist):
            subs = self._subs(self.pub_twist)
            self.get_logger().info(
                f"[RX twist #{self._cnt.rx_twist}] in='{self.t_ui_twist}' "
                f"frame='{out.header.frame_id}' "
                f"lin=({out.twist.linear.x:.4f}, {out.twist.linear.y:.4f}, {out.twist.linear.z:.4f}) "
                f"ang=({out.twist.angular.x:.4f}, {out.twist.angular.y:.4f}, {out.twist.angular.z:.4f}) "
                f"-> forward '{self.t_twist_out}' (subs={subs})"
            )

        self._warn_if_no_downstream(self.pub_twist, self.t_twist_out)

    def _on_ui_joint(self, msg: JointJog) -> None:
        self._cnt.rx_joint += 1
        self._cnt.last_rx_wall = time.time()

        self._ensure_servo_started()

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
                f"disp(rad)={d:+.6f} vel={v:.3f} frame='{out.header.frame_id}' "
                f"-> forward '{self.t_joint_out}' (subs={subs})"
            )

        self._warn_if_no_downstream(self.pub_joint, self.t_joint_out)

    def _on_set_mode(self, msg: String) -> None:
        """UI-Mode bleibt als Status erhalten (kein switch_service in Humble nötig)."""
        self._cnt.rx_set_mode += 1
        self._cnt.last_rx_wall = time.time()

        raw = (msg.data or "").strip().upper()
        if not raw:
            return
        self.last_ui_mode = raw
        self.get_logger().info(f"[set_mode] RX '{raw}' (no-op; MoveIt Servo Humble braucht keinen switch_command_type)")

    def _on_set_frame(self, msg: String) -> None:
        self._cnt.rx_set_frame += 1
        self._cnt.last_rx_wall = time.time()

        raw = (msg.data or "").strip().lower()
        if raw == "world":
            self.current_frame = self.frame_world
            self.get_logger().info(f"[set_frame] RX 'world' -> current_frame='{self.current_frame}'")
        elif raw == "tcp":
            self.current_frame = self.frame_tcp
            self.get_logger().info(f"[set_frame] RX 'tcp' -> current_frame='{self.current_frame}'")
        else:
            self.get_logger().warning(f"[set_frame] Unbekannter Frame: {msg.data!r} (expected 'world'|'tcp')")

    # -------------------------------------------------------------------------
    # Status ticker (optional)
    # -------------------------------------------------------------------------

    def _status_tick(self) -> None:
        age = (time.time() - self._cnt.last_rx_wall) if self._cnt.last_rx_wall else -1.0
        subs_tw = self._subs(self.pub_twist)
        subs_j = self._subs(self.pub_joint)
        srv_ready = "ready" if self.start_client.service_is_ready() else "not_ready"
        started = "started" if self._servo_started else "not_started"
        self.get_logger().info(
            f"[status] rx_twist={self._cnt.rx_twist} rx_joint={self._cnt.rx_joint} "
            f"tx_twist={self._cnt.tx_twist} tx_joint={self._cnt.tx_joint} "
            f"out_subs(twist={subs_tw}, joint={subs_j}) last_rx_age={age:.2f}s "
            f"start_srv={srv_ready} servo={started} ui_mode={self.last_ui_mode}"
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
