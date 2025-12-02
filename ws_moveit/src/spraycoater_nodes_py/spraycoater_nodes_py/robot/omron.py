# spraycoater_nodes_py/robot/omron.py
from __future__ import annotations

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from spraycoater_nodes_py.utils.config_hub import topics
from spraycoater_nodes_py.utils.omron_tcp_client import OmronTcpClient
from .base import BaseRobot


class OmronRobot(BaseRobot):
    """
    REAL-Node:

    - Commands: topics.yaml.omron.subscribe.command
      (/spraycoater/omron/command)
    - State-Publisher: topics.yaml.robot.publish.*
    """

    def __init__(self) -> None:
        super().__init__(node_name="robot")

        # Für Commands nutzen wir die Sektion "omron"
        self._topics = topics()

        self._joints = JointState()
        self._tcp_pose = PoseStamped()

        topic_cmd = self._topics.subscribe_topic("omron", "command")
        qos_cmd = self._topics.qos_by_id("subscribe", "omron", "command")

        self.sub_cmd = self.create_subscription(
            String,
            topic_cmd,
            self._on_command,
            qos_cmd,
        )

        host = "127.0.0.1"
        port = 5000
        self.get_logger().info(f"🔌 Verbinde zu Omron TCP-Server {host}:{port}")

        self._client = OmronTcpClient(host, port, timeout=3.0)
        self._connected = False

        self._ensure_connected()

    # ---------------------------------------------------
    # TCP Verbindung herstellen
    # ---------------------------------------------------
    def _ensure_connected(self) -> bool:
        if self._connected:
            return True
        try:
            self._client.connect()
            self._connected = True
            self._power = True
            self._servo_enabled = True
            self._initialized = True
            self._set_mode("READY")
            self.get_logger().info("✅ TCP-Verbindung aufgebaut.")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ TCP connect failed: {e}")
            self._connected = False
            self._power = False
            self._servo_enabled = False
            self._initialized = False
            self._set_mode("DISCONNECTED")
            return False

    # ---------------------------------------------------
    # Raw command vom GUI → Omron TCP
    # ---------------------------------------------------
    def _on_command(self, msg: String):
        cmd = (msg.data or "").strip()
        if not cmd:
            return

        if not self._ensure_connected():
            self.get_logger().error("❌ Not connected – command ignored")
            return

        self.get_logger().info(f"➡️ SEND: {cmd}")
        try:
            self._client._send_line(cmd)
        except Exception as e:
            self.get_logger().error(f"❌ Send error: {e}")
            self._connected = False
            self._power = False
            self._servo_enabled = False
            self._set_mode("DISCONNECTED")

    # ---------------------------------------------------
    # Hooks aus BaseRobot – Platzhalter
    # ---------------------------------------------------
    def _update_tcp_pose(self):
        # TODO: Pose-Feedback vom Controller → self._tcp_pose
        return

    def _update_joints(self):
        # TODO: joint-Feedback vom Controller → self._joints
        return


def main():
    import rclpy
    rclpy.init()
    node = OmronRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
