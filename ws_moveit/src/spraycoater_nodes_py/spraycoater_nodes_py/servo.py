#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/servo.py
from __future__ import annotations

import math
from typing import Optional

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from moveit_msgs.srv import ServoCommandType

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY = "servo"


class Servo(Node):
    """
    ROS-Servo-Node (config_hub-only)

    Input (UI -> Node):
      - servo/delta_twist_cmds (TwistStamped)
      - servo/delta_joint_cmds (JointJog)
      - servo/set_mode         (String)   optional
      - servo/set_frame        (String)   optional

    Output (Node -> moveit_servo):
      - servo/cartesian_mm (TwistStamped)
      - servo/joint_jog    (JointJog)

    Optional Service (namespaced):
      - omron_servo_node/switch_command_type (ServoCommandType)
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

    AXIS_ORDER = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    def __init__(self) -> None:
        # ✅ NICHT "servo_bridge" nennen (das ist UI-seitig oft schon vergeben)
        super().__init__("servo")

        # ---------------- Parameters ----------------
        self.declare_parameter("backend", "default")
        self.backend: str = self.get_parameter("backend").value or "default"
        backend_lower = str(self.backend).lower()

        # "real" nur wenn explizit real
        self.is_real_backend = "real" in backend_lower

        # Service nur in SIM/default nutzen (du kannst das über backend steuern)
        self.use_servo_service = (not self.is_real_backend) and (
            ("sim" in backend_lower) or (backend_lower == "default") or ("omron_sim" in backend_lower)
        )

        # Defaults (falls REAL mal aktiv wird)
        self.declare_parameter("joint_speed_pct_default", 30.0)
        self.declare_parameter("cart_speed_mmps_default", 50.0)
        self.joint_speed_pct_default = float(self.get_parameter("joint_speed_pct_default").value)
        self.cart_speed_mmps_default = float(self.get_parameter("cart_speed_mmps_default").value)

        # Service-Name RELATIV lassen → Namespace greift automatisch
        # Bei dir existiert: /shadow/omron_servo_node/switch_command_type
        self.declare_parameter("servo_command_type_service", "omron_servo_node/switch_command_type")
        self.servo_service_name = str(self.get_parameter("servo_command_type_service").value or "omron_servo_node/switch_command_type")

        self.declare_parameter("auto_mode", True)
        self.auto_mode = bool(self.get_parameter("auto_mode").value)

        # ---------------- config_hub ----------------
        self.loader = topics()
        self.frames_cfg = frames()
        self._F = self.frames_cfg.resolve

        self.frame_world = self._F(self.frames_cfg.get("world", "world"))
        # du hast in poses.py "tcp" als Key – bleib konsistent:
        self.frame_tcp = self._F(self.frames_cfg.get("tcp", "tcp"))
        self.current_frame = self.frame_world

        # Topics + QoS
        topic_twist_in = self.loader.subscribe_topic(NODE_KEY, "twist_out")
        qos_twist_in   = self.loader.qos_by_id("subscribe", NODE_KEY, "twist_out")

        topic_joint_in = self.loader.subscribe_topic(NODE_KEY, "joint_out")
        qos_joint_in   = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_out")

        topic_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        qos_set_mode   = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        topic_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        qos_set_frame   = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        topic_cartesian_cmd = self.loader.publish_topic(NODE_KEY, "cartesian_mm")
        qos_cartesian_cmd   = self.loader.qos_by_id("publish", NODE_KEY, "cartesian_mm")

        topic_joint_cmd = self.loader.publish_topic(NODE_KEY, "joint_jog")
        qos_joint_cmd   = self.loader.qos_by_id("publish", NODE_KEY, "joint_jog")

        topic_omron_cmd = self.loader.publish_topic("omron", "command")
        qos_omron_cmd   = self.loader.qos_by_id("publish", "omron", "command")

        # ---------------- Publishers ----------------
        self.pub_cartesian = None
        self.pub_joint = None
        self.pub_omron_cmd = None

        if not self.is_real_backend:
            self.pub_cartesian = self.create_publisher(TwistStamped, topic_cartesian_cmd, qos_cartesian_cmd)
            self.pub_joint = self.create_publisher(JointJog, topic_joint_cmd, qos_joint_cmd)
        else:
            self.pub_omron_cmd = self.create_publisher(String, topic_omron_cmd, qos_omron_cmd)

        # ---------------- Subscribers ----------------
        self.create_subscription(TwistStamped, topic_twist_in, self._on_twist_cmd, qos_twist_in)
        self.create_subscription(JointJog, topic_joint_in, self._on_joint_cmd, qos_joint_in)
        self.create_subscription(String, topic_set_mode, self._on_set_mode, qos_set_mode)
        self.create_subscription(String, topic_set_frame, self._on_set_frame, qos_set_frame)

        # ---------------- Service Client (SIM) ----------------
        self.servo_client = None
        if self.use_servo_service:
            self.servo_client = self.create_client(ServoCommandType, self.servo_service_name)

        self.current_mode: Optional[int] = None
        self._desired_mode: Optional[int] = None
        self._desired_label: str = ""
        self._mode_switch_inflight: bool = False

        if self.use_servo_service and self.auto_mode:
            self._retry_timer = self.create_timer(0.5, self._retry_set_mode)
        else:
            self._retry_timer = None

        # Einmal-Flags: sofort sehen ob Inputs überhaupt ankommen
        self._seen_twist = False
        self._seen_joint = False

        self.get_logger().info(
            f"✅ Servo Node aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}') | "
            f"world='{self.frame_world}', tcp='{self.frame_tcp}', current_frame='{self.current_frame}' | "
            f"in: twist='{topic_twist_in}', joint='{topic_joint_in}' | "
            f"out(sim): cart='{topic_cartesian_cmd}', joint='{topic_joint_cmd}' | "
            f"service(sim): '{self.servo_service_name}' (use={self.use_servo_service})"
        )

    # ------------------------------------------------------------
    # SIM: Mode switching (non-blocking)
    # ------------------------------------------------------------
    def _ensure_mode(self, cmd_type: int, label: str) -> None:
        if not self.use_servo_service or not self.auto_mode:
            return
        if self.current_mode == cmd_type:
            return
        self._desired_mode = cmd_type
        self._desired_label = label
        self._retry_set_mode()

    def _retry_set_mode(self) -> None:
        if not self.use_servo_service or self.servo_client is None:
            return
        if self._desired_mode is None:
            return
        if self.current_mode == self._desired_mode:
            self._desired_mode = None
            self._desired_label = ""
            return
        if self._mode_switch_inflight:
            return
        if not self.servo_client.service_is_ready():
            return  # still, no spam

        req = ServoCommandType.Request()
        req.command_type = int(self._desired_mode)

        self._mode_switch_inflight = True
        future = self.servo_client.call_async(req)

        def _done(fut):
            self._mode_switch_inflight = False
            try:
                resp = fut.result()
            except Exception:
                return
            if resp and getattr(resp, "success", False):
                self.current_mode = int(req.command_type)
                self._desired_mode = None
                self._desired_label = ""

        future.add_done_callback(_done)

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def _on_twist_cmd(self, msg: TwistStamped) -> None:
        if not self._seen_twist:
            self._seen_twist = True
            self.get_logger().info("📥 First Twist input received on servo/delta_twist_cmds")

        if self.is_real_backend:
            cmd = self._make_jogc_cmd_from_twist(msg)
            if cmd and self.pub_omron_cmd:
                self.pub_omron_cmd.publish(String(data=cmd))
            return

        if self.pub_cartesian is None:
            return

        self._ensure_mode(ServoCommandType.Request.TWIST, "TWIST")

        out = TwistStamped()
        out.twist = msg.twist
        out.header.frame_id = self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_cartesian.publish(out)

    def _on_joint_cmd(self, msg: JointJog) -> None:
        if not self._seen_joint:
            self._seen_joint = True
            self.get_logger().info("📥 First Joint input received on servo/delta_joint_cmds")

        if self.is_real_backend:
            cmd = self._make_jogj_cmd_from_joint(msg)
            if cmd and self.pub_omron_cmd:
                self.pub_omron_cmd.publish(String(data=cmd))
            return

        if self.pub_joint is None:
            return

        self._ensure_mode(ServoCommandType.Request.JOINT_JOG, "JOINT_JOG")

        out = JointJog()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_names = list(msg.joint_names)
        out.velocities = list(msg.velocities)
        out.displacements = list(msg.displacements)
        out.duration = msg.duration
        self.pub_joint.publish(out)

    def _on_set_mode(self, msg: String) -> None:
        if not self.use_servo_service or self.servo_client is None:
            return
        raw = (msg.data or "").strip().upper()
        if raw not in self.MODE_MAP:
            return
        self._desired_mode = int(self.MODE_MAP[raw])
        self._desired_label = raw
        self._retry_set_mode()

    def _on_set_frame(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if raw == "tcp":
            self.current_frame = self.frame_tcp
        elif raw == "world":
            self.current_frame = self.frame_world

    # ------------------------------------------------------------
    # REAL helpers (falls später)
    # ------------------------------------------------------------
    def _make_jogj_cmd_from_joint(self, msg: JointJog) -> Optional[str]:
        if not msg.joint_names:
            return None

        jname = msg.joint_names[0].lower()
        try:
            axis = self.AXIS_ORDER.index(jname) + 1
        except ValueError:
            try:
                axis = int(jname[-1])
            except Exception:
                return None

        step_deg = msg.displacements[0] if msg.displacements else 0.0
        if abs(step_deg) < 1e-6:
            return None

        speed_pct = msg.velocities[0] if msg.velocities else self.joint_speed_pct_default
        return f"JOGJ {axis} {step_deg:.3f} {speed_pct:.1f}"

    def _make_jogc_cmd_from_twist(self, msg: TwistStamped) -> Optional[str]:
        dx = msg.twist.linear.x
        dy = msg.twist.linear.y
        dz = msg.twist.linear.z
        drx = msg.twist.angular.x
        dry = msg.twist.angular.y
        drz = msg.twist.angular.z

        if (
            abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dz) < 1e-6 and
            abs(drx) < 1e-6 and abs(dry) < 1e-6 and abs(drz) < 1e-6
        ):
            return None

        speed_mmps = self.cart_speed_mmps_default
        return (
            f"JOGC {speed_mmps:.1f} "
            f"{dx:.3f} {dy:.3f} {dz:.3f} "
            f"{drx:.3f} {dry:.3f} {drz:.3f}"
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
