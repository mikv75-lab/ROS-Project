#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot.py

from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from spraycoater_nodes_py.utils.config_hub import topics
from spraycoater_nodes_py.utils.omron_tcp_client import OmronTcpClient


class OmronRobot(Node):
    """
    Minimaler Robot-Node (REAL ONLY):

      - verbindet zum Omron-TCP-Server
      - empfängt Raw-Kommandos über /robot/command
      - sendet sie 1:1 weiter
      - publiziert joints (später vom Controller)
      - publiziert tcp_pose (später per Query oder Eigenkinematik)

    Kein SIM, kein TF.
    """

    def __init__(self) -> None:
        super().__init__("robot")

        # --------------------------------------------
        # Topics-Loader
        # --------------------------------------------
        self.loader = topics()
        self.node_key = "robot"

        # --------------------------------------------
        # Publishes
        # --------------------------------------------
        self.pub_connected = self._make_pub(Bool, "connection")
        self.pub_joints = self._make_pub(JointState, "joints")
        self.pub_tcp_pose = self._make_pub(PoseStamped, "tcp_pose")

        # Internal buffers
        self._joints = JointState()
        self._tcp_pose = PoseStamped()

        # --------------------------------------------
        # Subscriber: Commands
        # --------------------------------------------
        topic_cmd = self.loader.subscribe_topic(self.node_key, "command")
        qos_cmd = self.loader.qos_by_id("subscribe", self.node_key, "command")

        self.sub_cmd = self.create_subscription(
            String,
            topic_cmd,
            self._on_command,
            qos_cmd,
        )

        # --------------------------------------------
        # TCP-Verbindung
        # --------------------------------------------
        host = "127.0.0.1"
        port = 5000

        self.get_logger().info(f"🔌 Verbinde zu Omron TCP-Server {host}:{port}")

        self._client = OmronTcpClient(host, port, timeout=3.0)
        self._connected = False

        # Verbindungsversuch direkt beim Start
        self._ensure_connected()

        # zyklischer Status
        self.timer = self.create_timer(0.1, self._on_timer)

    # ---------------------------------------------------
    # Helpers
    # ---------------------------------------------------
    def _make_pub(self, msg_type, topic_id: str):
        return self.create_publisher(
            msg_type,
            self.loader.publish_topic(self.node_key, topic_id),
            self.loader.qos_by_id("publish", self.node_key, topic_id),
        )

    # ---------------------------------------------------
    # TCP Verbindung herstellen
    # ---------------------------------------------------
    def _ensure_connected(self) -> bool:
        if self._connected:
            return True
        try:
            self._client.connect()
            self._connected = True
            self.get_logger().info("✅ TCP-Verbindung aufgebaut.")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ TCP connect failed: {e}")
            self._connected = False
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

    # ---------------------------------------------------
    # Timer: publish state
    # ---------------------------------------------------
    def _on_timer(self):
        # Verbindung
        self.pub_connected.publish(Bool(data=self._connected))

        # joints (leer bis Feedback implementiert ist)
        if self._joints.name:
            self.pub_joints.publish(self._joints)

        # tcp_pose (leer)
        if self._tcp_pose.header.frame_id:
            self.pub_tcp_pose.publish(self._tcp_pose)


def main():
    rclpy.init()
    node = OmronRobot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
