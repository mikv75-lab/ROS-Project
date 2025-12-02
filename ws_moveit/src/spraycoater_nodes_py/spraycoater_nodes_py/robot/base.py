# spraycoater_nodes_py/robot/base.py
from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from spraycoater_nodes_py.utils.config_hub import topics


class BaseRobot(Node):
    """
    Gemeinsame Basis für alle Robot-Nodes.

    Nutzt die Sektion 'robot' aus topics.yaml:
      - publish: connection, mode, initialized, moving, servo_enabled,
                 power, estop, errors, tcp_pose, joints
    """

    def __init__(self, node_name: str = "robot") -> None:
        super().__init__(node_name)

        self.loader = topics()
        self.node_key = "robot"

        # State
        self._connected: bool = False
        self._initialized: bool = False
        self._moving: bool = False
        self._servo_enabled: bool = False
        self._power: bool = False
        self._estop: bool = False
        self._mode: str = "DISCONNECTED"
        self._last_error: str = ""

        # Datencontainer
        self._tcp_pose = PoseStamped()
        self._joints = JointState()

        # Publisher (alle aus topics.yaml.robot.publish)
        self.pub_connection = self._make_pub(Bool, "connection")
        self.pub_mode = self._make_pub(String, "mode")
        self.pub_initialized = self._make_pub(Bool, "initialized")
        self.pub_moving = self._make_pub(Bool, "moving")
        self.pub_servo_enabled = self._make_pub(Bool, "servo_enabled")
        self.pub_power = self._make_pub(Bool, "power")
        self.pub_estop = self._make_pub(Bool, "estop")
        self.pub_errors = self._make_pub(String, "errors")

        self.pub_tcp_pose = self._make_pub(PoseStamped, "tcp_pose")
        self.pub_joints = self._make_pub(JointState, "joints")

        # 10 Hz State-Update
        self.state_timer = self.create_timer(0.1, self._on_state_timer)

    # ------------------------------------------------------------------
    # Helpers: Topics
    # ------------------------------------------------------------------
    def _make_pub(self, msg_type, topic_id: str):
        return self.create_publisher(
            msg_type,
            self.loader.publish_topic(self.node_key, topic_id),
            self.loader.qos_by_id("publish", self.node_key, topic_id),
        )

    def _make_sub(self, msg_type, topic_id: str, cb):
        return self.create_subscription(
            msg_type,
            self.loader.subscribe_topic(self.node_key, topic_id),
            cb,
            self.loader.qos_by_id("subscribe", self.node_key, topic_id),
        )

    # ------------------------------------------------------------------
    # State-Setter
    # ------------------------------------------------------------------
    def _set_mode(self, mode: str):
        mode = (mode or "").strip() or "UNKNOWN"
        if mode != self._mode:
            self._mode = mode
            self.get_logger().info(f"🔁 Robot-Mode -> {mode}")

    def _set_error(self, msg: str):
        msg = (msg or "").strip()
        if not msg:
            return
        self._last_error = msg
        self.get_logger().error(f"❌ Robot-Error: {msg}")
        self.pub_errors.publish(String(data=msg))

    # ------------------------------------------------------------------
    # Hooks für Subklassen
    # ------------------------------------------------------------------
    def _update_tcp_pose(self):
        """Von Subklassen überschreiben, um self._tcp_pose zu setzen."""
        return

    def _update_joints(self):
        """Von Subklassen überschreiben, falls nötig."""
        return

    # ------------------------------------------------------------------
    # Haupt-Timer
    # ------------------------------------------------------------------
    def _on_state_timer(self):
        now_msg = self.get_clock().now().to_msg()

        # States
        self.pub_connection.publish(Bool(data=self._connected))
        self.pub_initialized.publish(Bool(data=self._initialized))
        self.pub_moving.publish(Bool(data=self._moving))
        self.pub_servo_enabled.publish(Bool(data=self._servo_enabled))
        self.pub_power.publish(Bool(data=self._power))
        self.pub_estop.publish(Bool(data=self._estop))
        self.pub_mode.publish(String(data=self._mode))

        # Daten aktualisieren
        self._update_tcp_pose()
        self._update_joints()

        # TCP-Pose senden (wenn nicht 0)
        if (
            self._tcp_pose.header.stamp.sec != 0
            or self._tcp_pose.header.stamp.nanosec != 0
        ):
            self.pub_tcp_pose.publish(self._tcp_pose)

        # JointState senden (wenn sinnvoll)
        if self._joints.name and self._joints.position:
            self._joints.header.stamp = now_msg
            self.pub_joints.publish(self._joints)
