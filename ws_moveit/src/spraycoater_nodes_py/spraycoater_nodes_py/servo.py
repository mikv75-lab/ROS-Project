#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

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
    ROS-Servo-Node (läuft im ROS graph):
      UI -> (servo/cartesian_mm, servo/joint_jog)
      -> moveit_servo input: (servo/delta_twist_cmds, servo/delta_joint_cmds)

    Zusätzlich: set_mode/set_frame + optional switch_command_type.
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

        self.declare_parameter("backend", "default")
        self.backend = self.get_parameter("backend").get_parameter_value().string_value or "default"

        # relativ, damit Namespace greift -> /shadow/omron_servo_node/switch_command_type
        self.declare_parameter("servo_command_type_service", "omron_servo_node/switch_command_type")
        self.servo_service_name = (
            self.get_parameter("servo_command_type_service").get_parameter_value().string_value
            or "omron_servo_node/switch_command_type"
        )

        self.declare_parameter("auto_mode", True)
        self.auto_mode = bool(self.get_parameter("auto_mode").get_parameter_value().bool_value)

        self.loader = topics()
        self.frames_cfg = frames()
        self._F = self.frames_cfg.resolve
        self.current_frame = self._F(self.frames_cfg.get("tcp", "tcp"))

        # --- subscribe: UI topics ---
        t_ui_twist = self.loader.subscribe_topic(NODE_KEY, "cartesian_mm")
        q_ui_twist = self.loader.qos_by_id("subscribe", NODE_KEY, "cartesian_mm")

        t_ui_joint = self.loader.subscribe_topic(NODE_KEY, "joint_jog")
        q_ui_joint = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_jog")

        t_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        q_set_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        t_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        q_set_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        # --- publish: moveit_servo inputs (delta_*) ---
        t_twist_out = self.loader.publish_topic(NODE_KEY, "twist_out")
        q_twist_out = self.loader.qos_by_id("publish", NODE_KEY, "twist_out")

        t_joint_out = self.loader.publish_topic(NODE_KEY, "joint_out")
        q_joint_out = self.loader.qos_by_id("publish", NODE_KEY, "joint_out")

        self.pub_twist = self.create_publisher(TwistStamped, t_twist_out, q_twist_out)
        self.pub_joint = self.create_publisher(JointJog, t_joint_out, q_joint_out)

        self.create_subscription(TwistStamped, t_ui_twist, self._on_ui_twist, q_ui_twist)
        self.create_subscription(JointJog, t_ui_joint, self._on_ui_joint, q_ui_joint)
        self.create_subscription(String, t_set_mode, self._on_set_mode, q_set_mode)
        self.create_subscription(String, t_set_frame, self._on_set_frame, q_set_frame)

        self.servo_client = self.create_client(ServoCommandType, self.servo_service_name)
        self.current_mode: int | None = None
        self._desired_mode: int | None = None
        self._inflight = False

        self.get_logger().info(
            f"✅ Servo aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}') "
            f"UI in: {t_ui_twist}, {t_ui_joint} -> moveit_servo in: {t_twist_out}, {t_joint_out} "
            f"switch_service='{self.servo_service_name}'"
        )

    def _ensure_mode(self, cmd_type: int) -> None:
        if not self.auto_mode:
            return
        if self.current_mode == cmd_type:
            return
        self._desired_mode = cmd_type
        self._try_switch_mode()

    def _try_switch_mode(self) -> None:
        if self._desired_mode is None or self._inflight:
            return
        if not self.servo_client.service_is_ready():
            return
        req = ServoCommandType.Request()
        req.command_type = int(self._desired_mode)

        self._inflight = True
        fut = self.servo_client.call_async(req)

        def _done(_f):
            self._inflight = False
            try:
                resp = _f.result()
            except Exception as e:
                self.get_logger().warning(f"switch_command_type exception: {e}")
                return
            if resp and getattr(resp, "success", False):
                self.current_mode = int(req.command_type)
                self._desired_mode = None
            else:
                self.get_logger().warning(f"switch_command_type failed (type={req.command_type})")

        fut.add_done_callback(_done)

    def _on_ui_twist(self, msg: TwistStamped) -> None:
        self._ensure_mode(ServoCommandType.Request.TWIST)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.twist = msg.twist
        self.pub_twist.publish(out)

    def _on_ui_joint(self, msg: JointJog) -> None:
        self._ensure_mode(ServoCommandType.Request.JOINT_JOG)

        out = JointJog()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.joint_names = list(msg.joint_names)
        out.velocities = list(msg.velocities)
        out.displacements = list(msg.displacements)
        out.duration = msg.duration
        self.pub_joint.publish(out)

    def _on_set_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip().upper()
        if raw not in self.MODE_MAP:
            self.get_logger().warning(f"Unbekannter Mode: {msg.data!r}")
            return
        self._desired_mode = int(self.MODE_MAP[raw])
        self._try_switch_mode()

    def _on_set_frame(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if raw in ("world", "tcp"):
            self.current_frame = self._F(self.frames_cfg.get(raw, raw))
            self.get_logger().info(f"Frame gesetzt: {raw} -> {self.current_frame}")


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
