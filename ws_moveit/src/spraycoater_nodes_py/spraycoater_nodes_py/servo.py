#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/servo.py
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
    Bridge zwischen UI-ServiceTabs und moveit_servo (Fake-/SIM-Roboter)
    ODER Omron-Real (via TCP-Bridge).

    - Topics ausschließlich aus topics.yaml via config_hub
    - Namespace ausschließlich über den Node (Launchfile)
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
        super().__init__("servo_bridge")

        # ---------------- Backend-Parameter ----------------
        self.declare_parameter("backend", "default")
        self.backend: str = (
            self.get_parameter("backend").get_parameter_value().string_value or "default"
        )
        backend_lower = self.backend.lower()

        self.is_real_backend = "real" in backend_lower

        # moveit_servo-Service nur in SIM / default
        self.use_servo_service = (
            (not self.is_real_backend)
            and ("sim" in backend_lower or self.backend == "default")
        )

        # Defaults für REAL-Backend (Omron)
        self.declare_parameter("joint_speed_pct_default", 30.0)
        self.declare_parameter("cart_speed_mmps_default", 50.0)
        self.joint_speed_pct_default = (
            self.get_parameter("joint_speed_pct_default").get_parameter_value().double_value
        )
        self.cart_speed_mmps_default = (
            self.get_parameter("cart_speed_mmps_default").get_parameter_value().double_value
        )

        # Service-Name relativ halten, damit Namespace greifen kann
        self.declare_parameter(
            "servo_command_type_service",
            "omron_servo_node/switch_command_type",
        )
        self.servo_service_name = (
            self.get_parameter("servo_command_type_service").get_parameter_value().string_value
            or "omron_servo_node/switch_command_type"
        )
        if self.servo_service_name.startswith("/"):
            self.get_logger().warning(
                f"[{self.backend}] servo_command_type_service ist absolut ('{self.servo_service_name}'). "
                "Das ignoriert Namespace. Besser ohne führenden '/'."
            )

        # Config-Loader / Frames
        self.loader = topics()
        self.frames_cfg = frames()
        self._F = self.frames_cfg.resolve

        # Default-Frame (tcp aus frames.yaml, sonst "world")
        # (frames.yaml keys: world/tcp etc. -> resolve() mappt ggf. Prefix)
        self.current_frame = self._F(self.frames_cfg.get("tcp", "world"))

        # ---- Topics aus config_hub ----
        # SUB (UI → ServoBridge)
        topic_twist_in = self.loader.subscribe_topic(NODE_KEY, "twist_out")
        qos_twist_in = self.loader.qos_by_id("subscribe", NODE_KEY, "twist_out")

        topic_joint_in = self.loader.subscribe_topic(NODE_KEY, "joint_out")
        qos_joint_in = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_out")

        topic_set_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        qos_set_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        topic_set_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        qos_set_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        # PUB (ServoBridge → moveit_servo) [SIM]
        topic_cartesian_cmd = self.loader.publish_topic(NODE_KEY, "cartesian_mm")
        qos_cartesian_cmd = self.loader.qos_by_id("publish", NODE_KEY, "cartesian_mm")

        topic_joint_cmd = self.loader.publish_topic(NODE_KEY, "joint_jog")
        qos_joint_cmd = self.loader.qos_by_id("publish", NODE_KEY, "joint_jog")

        # PUB (ServoBridge → OmronTcpBridge) [REAL]
        # ✅ topics.yaml: group 'omron', publish-id ist 'command' (nicht 'command_out')
        topic_omron_cmd = self.loader.publish_topic("omron", "command")
        qos_omron_cmd = self.loader.qos_by_id("publish", "omron", "command")

        # ---- Publisher je Backend ----
        self.pub_cartesian = None
        self.pub_joint = None
        if not self.is_real_backend:
            self.pub_cartesian = self.create_publisher(
                TwistStamped, topic_cartesian_cmd, qos_cartesian_cmd
            )
            self.pub_joint = self.create_publisher(
                JointJog, topic_joint_cmd, qos_joint_cmd
            )

        self.pub_omron_cmd = None
        if self.is_real_backend:
            self.pub_omron_cmd = self.create_publisher(String, topic_omron_cmd, qos_omron_cmd)

        # ---- Subscriber (von UI-Tabs) ----
        self.create_subscription(TwistStamped, topic_twist_in, self._on_twist_cmd, qos_twist_in)
        self.create_subscription(JointJog, topic_joint_in, self._on_joint_cmd, qos_joint_in)
        self.create_subscription(String, topic_set_mode, self._on_set_mode, qos_set_mode)
        self.create_subscription(String, topic_set_frame, self._on_set_frame, qos_set_frame)

        # ---- Service-Client für CommandType (nur SIM/Fake) ----
        if self.use_servo_service:
            self.servo_client = self.create_client(ServoCommandType, self.servo_service_name)
            self.get_logger().info(
                f"[{self.backend}] ServoBridge: benutze CommandType-Service '{self.servo_service_name}'"
            )
        else:
            self.servo_client = None
            self.get_logger().info(f"[{self.backend}] ServoBridge: KEIN CommandType-Service aktiv")

        self.current_mode: int | None = None

        self.get_logger().info(
            f"✅ ServoBridge aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}') – "
            f"Frame='{self.current_frame}', "
            f"SIM publish: {topic_cartesian_cmd}, {topic_joint_cmd}, "
            f"REAL publish: {topic_omron_cmd}"
        )

    # ------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------
    def _on_twist_cmd(self, msg: TwistStamped) -> None:
        """
        Cartesian Jog:
          SIM  -> TwistStamped an moveit_servo (servo/cartesian_mm)
          REAL -> JOGC-String an OmronTcpBridge (spraycoater/omron/command)
        """
        if self.is_real_backend:
            cmd = self._make_jogc_cmd_from_twist(msg)
            if cmd is None or self.pub_omron_cmd is None:
                return
            self.pub_omron_cmd.publish(String(data=cmd))
            self.get_logger().debug(f"[{self.backend}] JOGC → '{cmd}'")
            return

        if self.pub_cartesian is None:
            return

        out = TwistStamped()
        out.twist = msg.twist
        out.header.frame_id = self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        self.pub_cartesian.publish(out)

    def _on_joint_cmd(self, msg: JointJog) -> None:
        """
        Joint Jog:
          SIM  -> JointJog an moveit_servo (servo/joint_jog)
          REAL -> JOGJ-String an OmronTcpBridge (spraycoater/omron/command)
        """
        if self.is_real_backend:
            cmd = self._make_jogj_cmd_from_joint(msg)
            if cmd is None or self.pub_omron_cmd is None:
                return
            self.pub_omron_cmd.publish(String(data=cmd))
            self.get_logger().debug(f"[{self.backend}] JOGJ → '{cmd}'")
            return

        if self.pub_joint is None:
            return

        out = JointJog()
        out.header.frame_id = msg.header.frame_id or self.current_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_names = list(msg.joint_names)
        out.velocities = list(msg.velocities)
        out.displacements = list(msg.displacements)
        out.duration = msg.duration
        self.pub_joint.publish(out)

    # ------------------------------------------------------------
    # Hilfsfunktionen REAL-Backend
    # ------------------------------------------------------------
    def _make_jogj_cmd_from_joint(self, msg: JointJog) -> str | None:
        if not msg.joint_names:
            self.get_logger().warning(f"[{self.backend}] JOGJ: joint_names leer.")
            return None

        jname = msg.joint_names[0].lower()
        try:
            axis = self.AXIS_ORDER.index(jname) + 1
        except ValueError:
            try:
                axis = int(jname[-1])
            except Exception:
                self.get_logger().warning(
                    f"[{self.backend}] JOGJ: kann Achse aus '{msg.joint_names[0]}' nicht bestimmen."
                )
                return None

        if axis < 1 or axis > 6:
            self.get_logger().warning(
                f"[{self.backend}] JOGJ: ungültige Achse {axis} aus '{msg.joint_names[0]}'"
            )
            return None

        step_deg = msg.displacements[0] if msg.displacements else 0.0
        if abs(step_deg) < 1e-6:
            return None

        speed_pct = msg.velocities[0] if msg.velocities else self.joint_speed_pct_default
        return f"JOGJ {axis} {step_deg:.3f} {speed_pct:.1f}"

    def _make_jogc_cmd_from_twist(self, msg: TwistStamped) -> str | None:
        dx = msg.twist.linear.x
        dy = msg.twist.linear.y
        dz = msg.twist.linear.z
        drx = msg.twist.angular.x
        dry = msg.twist.angular.y
        drz = msg.twist.angular.z

        if (
            abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dz) < 1e-6
            and abs(drx) < 1e-6 and abs(dry) < 1e-6 and abs(drz) < 1e-6
        ):
            return None

        speed_mmps = self.cart_speed_mmps_default
        return (
            f"JOGC {speed_mmps:.1f} "
            f"{dx:.3f} {dy:.3f} {dz:.3f} "
            f"{drx:.3f} {dry:.3f} {drz:.3f}"
        )

    # ------------------------------------------------------------
    # Mode / Frame
    # ------------------------------------------------------------
    def _on_set_mode(self, msg: String) -> None:
        raw = (msg.data or "").strip().upper()
        if raw not in self.MODE_MAP:
            self.get_logger().warning(
                f"[{self.backend}] Unbekannter Servo-Mode '{msg.data}' – erwartet JOINT_JOG, TWIST oder POSE."
            )
            return
        self._switch_mode(self.MODE_MAP[raw], raw)

    def _switch_mode(self, cmd_type: int, label: str = "") -> None:
        if not self.use_servo_service:
            self.current_mode = cmd_type
            self.get_logger().info(
                f"[{self.backend}] (real) Servo-Mode lokal auf '{label}' gesetzt – kein Service aktiv."
            )
            return

        if self.current_mode == cmd_type:
            return

        if self.servo_client is None:
            self.get_logger().warning(
                f"[{self.backend}] Servo-Client nicht initialisiert – kann Mode '{label}' nicht setzen."
            )
            return

        if not self.servo_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning(
                f"[{self.backend}] Service '{self.servo_service_name}' nicht verfügbar."
            )
            return

        req = ServoCommandType.Request()
        req.command_type = cmd_type
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        resp = future.result()
        if resp and resp.success:
            self.current_mode = cmd_type
            self.get_logger().info(
                f"[{self.backend}] ✅ Servo-Mode geändert auf '{label}' (command_type={cmd_type})"
            )
        else:
            self.get_logger().warning(
                f"[{self.backend}] ❌ switch_command_type fehlgeschlagen (command_type={cmd_type})"
            )

    def _on_set_frame(self, msg: String) -> None:
        raw = (msg.data or "").strip().lower()
        if not raw:
            return

        allowed = ("world", "tcp")
        if raw not in allowed:
            self.get_logger().warning(
                f"[{self.backend}] set_frame: Frame '{raw}' nicht erlaubt. "
                f"Erlaubt: {allowed}. (frames.yaml: {list(self.frames_cfg.as_dict().keys())})"
            )
            return

        resolved = self._F(self.frames_cfg.get(raw, raw))
        old = self.current_frame
        self.current_frame = resolved
        self.get_logger().info(
            f"[{self.backend}] Servo-Frame geändert: {old} → {self.current_frame} (gewählt: {raw})"
        )


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
