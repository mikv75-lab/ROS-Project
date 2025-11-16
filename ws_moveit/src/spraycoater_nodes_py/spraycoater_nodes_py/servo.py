# -*- coding: utf-8 -*-
# spraycoater_nodes_py/servo.py
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import String
from moveit_msgs.srv import ServoCommandType

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY = "servo"


class ServoBridge(Node):
    """
    Bridge zwischen UI-ServiceTabs und moveit_servo.

    Topics (laut topics.yaml, NODE_KEY='servo'):

      subscribe (UI -> ServoBridge):
        - twist_out   (geometry_msgs/TwistStamped)   /servo/delta_twist_cmds
        - joint_out   (control_msgs/JointJog)        /servo/delta_joint_cmds
        - set_mode    (std_msgs/String)              /servo/set_mode
        - set_frame   (std_msgs/String)              /servo/set_frame

      publish (ServoBridge -> moveit_servo):
        - cartesian_mm (geometry_msgs/TwistStamped)  /servo/cartesian_mm
        - joint_jog    (control_msgs/JointJog)       /servo/joint_jog
    """

    # Mapping von String -> ServoCommandType
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
        super().__init__("servo_bridge")

        # Config-Loader / Frames
        self.loader = topics()
        self.frames_cfg = frames()
        self._F = self.frames_cfg.resolve

        # Default-Frame (world aus frames.yaml, sonst "world")
        self.current_frame = self._F(self.frames_cfg.get("world", "world"))
        self.get_logger().info(f"ServoBridge: Initial frame = '{self.current_frame}'")

        # ---- Topics aus config_hub laden ----
        topic_twist_in = self.loader.subscribe_topic(NODE_KEY, "twist_out")
        qos_twist_in = self.loader.qos_by_id("subscribe", NODE_KEY, "twist_out")

        topic_joint_in = self.loader.subscribe_topic(NODE_KEY, "joint_out")
        qos_joint_in = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_out")

        topic_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        qos_set_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        topic_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        qos_set_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        topic_cartesian_cmd = self.loader.publish_topic(NODE_KEY, "cartesian_mm")
        qos_cartesian_cmd = self.loader.qos_by_id("publish", NODE_KEY, "cartesian_mm")

        topic_joint_cmd = self.loader.publish_topic(NODE_KEY, "joint_jog")
        qos_joint_cmd = self.loader.qos_by_id("publish", NODE_KEY, "joint_jog")

        # ---- Publisher (an moveit_servo) ----
        self.pub_cartesian = self.create_publisher(
            TwistStamped, topic_cartesian_cmd, qos_cartesian_cmd
        )
        self.pub_joint = self.create_publisher(
            JointJog, topic_joint_cmd, qos_joint_cmd
        )

        # ---- Subscriber (von UI-Tabs) ----
        self.create_subscription(
            TwistStamped, topic_twist_in, self._on_twist_cmd, qos_twist_in
        )
        self.create_subscription(
            JointJog, topic_joint_in, self._on_joint_cmd, qos_joint_in
        )
        self.create_subscription(
            String, topic_set_mode, self._on_set_mode, qos_set_mode
        )
        self.create_subscription(
            String, topic_set_frame, self._on_set_frame, qos_set_frame
        )

        # ---- Service-Client für CommandType ----
        self.servo_client = self.create_client(
            ServoCommandType, "/omron_servo_node/switch_command_type"
        )
        self.current_mode: int | None = None

        self.get_logger().info(
            f"✅ ServoBridge aktiv – Publiziert auf '{topic_cartesian_cmd}' und '{topic_joint_cmd}'"
        )

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------

    def _on_twist_cmd(self, msg: TwistStamped) -> None:
        out = TwistStamped()
        out.twist = msg.twist  # nur Inhalt übernehmen
        out.header.frame_id = self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_cartesian.publish(out)
        self.get_logger().debug(
            f"Twist → cartesian_mm (frame={out.header.frame_id}, "
            f"lin=({out.twist.linear.x:.4f},"
            f"{out.twist.linear.y:.4f},"
            f"{out.twist.linear.z:.4f}), "
            f"ang=({out.twist.angular.x:.4f},"
            f"{out.twist.angular.y:.4f},"
            f"{out.twist.angular.z:.4f}))"
        )

    def _on_joint_cmd(self, msg: JointJog) -> None:
        out = JointJog()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_names = list(msg.joint_names)
        out.velocities = list(msg.velocities)
        out.displacements = list(msg.displacements)
        out.duration = msg.duration

        self.pub_joint.publish(out)
        self.get_logger().debug(
            f"JointJog → joint_jog (n={len(out.joint_names)}, duration={out.duration:.3f})"
        )

    def _on_set_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip().upper()
        if raw not in self.MODE_MAP:
            self.get_logger().warning(
                f"Unbekannter Servo-Mode '{msg.data}' – erwartet z.B. JOINT_JOG, TWIST oder POSE."
            )
            return

        cmd_type = self.MODE_MAP[raw]
        self._switch_mode(cmd_type, raw)

    def _switch_mode(self, cmd_type: int, label: str = "") -> None:
        """Interne Hilfsfunktion: CommandType-Service synchron aufrufen."""
        if self.current_mode == cmd_type:
            self.get_logger().info(
                f"Servo-Mode bereits '{label}' – kein Servicecall nötig."
            )
            return

        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                "Service /omron_servo_node/switch_command_type nicht verfügbar."
            )
            return

        req = ServoCommandType.Request()
        req.command_type = cmd_type

        self.get_logger().info(
            f"↪ switch_command_type: '{label}' → command_type={cmd_type}"
        )
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error(
                "switch_command_type: Keine Antwort vom Service."
            )
            return

        resp = future.result()
        if resp.success:
            # Response hat *nur* success, kein command_type-Feld
            self.current_mode = cmd_type
            self.get_logger().info(
                f"✅ Servo-Mode geändert auf '{label}' (command_type={cmd_type})"
            )
        else:
            self.get_logger().warning(
                f"❌ switch_command_type fehlgeschlagen (command_type={cmd_type})"
            )

    def _on_set_frame(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            self.get_logger().warning("set_frame: Leerstring ignoriert.")
            return

        resolved = self._F(self.frames_cfg.get(raw, raw))
        old = self.current_frame
        self.current_frame = resolved
        self.get_logger().info(
            f"Frame geändert: '{raw}' → '{resolved}' (alt='{old}')"
        )


# ------------------------------------------------------------------
# main: normaler Node-Start (kein Test mehr)
# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
