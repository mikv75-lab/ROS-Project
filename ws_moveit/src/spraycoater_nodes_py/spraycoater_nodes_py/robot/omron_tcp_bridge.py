#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot/omron_tcp_bridge.py

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from spraycoater_nodes_py.utils.config_hub import topics
from .omron_tcp_client import OmronTcpClient


class OmronTcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("omron_tcp_bridge")

        self.loader = topics()

        # --- Topics aus topics.yaml holen ---
        topic_cmd = self.loader.subscribe_topic("omron", "command_in")
        qos_cmd = self.loader.qos_by_id("subscribe", "omron", "command_in")

        topic_raw_tx = self.loader.publish_topic("omron", "raw_tx")
        qos_raw_tx = self.loader.qos_by_id("publish", "omron", "raw_tx")

        topic_raw_rx = self.loader.publish_topic("omron", "raw_rx")
        qos_raw_rx = self.loader.qos_by_id("publish", "omron", "raw_rx")

        topic_conn = self.loader.publish_topic("omron", "connection_status")
        qos_conn = self.loader.qos_by_id("publish", "omron", "connection_status")

        # Subscriber: GUI / TCP-Client-Widget -> Bridge
        self.sub_cmd = self.create_subscription(
            String,
            topic_cmd,
            self._on_command,
            qos_cmd,
        )

        # Publisher: Bridge -> Debug / UI
        self.pub_raw_tx = self.create_publisher(String, topic_raw_tx, qos_raw_tx)
        self.pub_raw_rx = self.create_publisher(String, topic_raw_rx, qos_raw_rx)
        self.pub_conn = self.create_publisher(Bool, topic_conn, qos_conn)

        self._client: OmronTcpClient | None = None
        self._connected: bool = False

        self._rx_timer = self.create_timer(0.05, self._poll_rx)

    def _set_connection(self, connected: bool) -> None:
        self._connected = bool(connected)
        self.pub_conn.publish(Bool(data=self._connected))

    def _ensure_connected(self) -> bool:
        if self._connected and self._client is not None:
            return True
        try:
            self._client = OmronTcpClient()
            self._client.connect()
            self._set_connection(True)
            self.get_logger().info("✅ OmronTcpClient verbunden.")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Connect failed: {e}")
            self._set_connection(False)
            self._client = None
            return False

    def _poll_rx(self) -> None:
        if not self._connected or self._client is None:
            return

        # pro Timer-Tick begrenzen, damit wir den Executor nicht blockieren
        max_lines_per_tick = 10

        for _ in range(max_lines_per_tick):
            try:
                line = self._client.recv_line_nowait()
            except Exception as e:
                self.get_logger().warning(f"⚠️ RX poll Fehler: {e}")
                self._set_connection(False)
                return

            # recv_line_nowait() liefert "" wenn keine vollständige Zeile da ist
            if not line:
                return

            text = line.strip()
            if not text:
                return

            self.get_logger().info(f"⬅️ RECV (poll): {text}")
            self.pub_raw_rx.publish(String(data=text))

    def _on_command(self, msg: String) -> None:
        cmd = (msg.data or "").strip()
        if not cmd:
            return
        if not self._ensure_connected():
            self.get_logger().error("❌ Nicht verbunden – Command verworfen")
            return

        try:
            self.get_logger().info(f"➡️ SEND: {cmd}")
            self.pub_raw_tx.publish(String(data=cmd))
            self._client._send_line(cmd)  # oder self._client.send_line(cmd)
        except Exception as e:
            self.get_logger().error(f"❌ Send error: {e}")
            self._set_connection(False)


def main():
    rclpy.init()
    node = OmronTcpBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
