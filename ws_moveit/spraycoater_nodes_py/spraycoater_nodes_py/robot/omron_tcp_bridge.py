#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot/omron_tcp_bridge.py
#
# Schlanke Bridge:
#   - SUB:  omron.command          (/.../omron/command)
#   - PUB:  omron.raw_tx           (/.../omron/raw_tx)
#           omron.raw_rx           (/.../omron/raw_rx)
#           omron.connectionStatus (/.../omron/connectionStatus)

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from spraycoater_nodes_py.utils.config_hub import topics
from .omron_tcp_client import OmronTcpClient   # <--- HIER: relativer Import!


class OmronTcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("omron_tcp_bridge")

        self.loader = topics()

        # --- Topics aus topics.yaml holen ---
        topic_cmd = self.loader.subscribe_topic("omron", "command")
        qos_cmd = self.loader.qos_by_id("subscribe", "omron", "command")

        topic_raw_tx = self.loader.publish_topic("omron", "raw_tx")
        qos_raw_tx = self.loader.qos_by_id("publish", "omron", "raw_tx")

        topic_raw_rx = self.loader.publish_topic("omron", "raw_rx")
        qos_raw_rx = self.loader.qos_by_id("publish", "omron", "raw_rx")

        topic_conn = self.loader.publish_topic("omron", "connectionStatus")
        qos_conn = self.loader.qos_by_id("publish", "omron", "connectionStatus")

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

        # Parameter (kannst du im Launchfile überschreiben)
        self._host = (
            self.declare_parameter("host", "host.docker.internal")
            .get_parameter_value()
            .string_value
        )
        self._port = (
            self.declare_parameter("port", 5000)
            .get_parameter_value()
            .integer_value
        )
        self._timeout = (
            self.declare_parameter("timeout", 3.0)
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(
            f"🔌 OmronTcpBridge verbindet zu {self._host}:{self._port} (timeout={self._timeout}s)"
        )
        self.get_logger().info(
            "Hinweis: host/port können via --ros-args -p host:=... -p port:=... überschrieben werden."
        )

        self._client = OmronTcpClient(self._host, self._port, timeout=self._timeout)
        self._connected = False

        # initial klarstellen: nicht verbunden
        self._set_connection(False)

        # Direkt beim Start einmalig Verbindungsversuch
        self.get_logger().info("🌐 Initialer Verbindungsversuch zum Omron-Server ...")
        self._ensure_connected()

        # 🔁 Periodischer Status-Tick
        self._conn_timer = self.create_timer(1.0, self._publish_connection_status)

        # 🔁 Periodischer RX-Poll
        self._rx_timer = self.create_timer(0.05, self._poll_rx)

    # ------------------------------------------------------------------
    # Verbindung / State
    # ------------------------------------------------------------------
    def _set_connection(self, connected: bool) -> None:
        connected = bool(connected)
        self._connected = connected

        try:
            msg = Bool()
            msg.data = connected
            self.pub_conn.publish(msg)
        except Exception as e:
            self.get_logger().error(f"❌ connectionStatus publish failed: {e}")

        if connected:
            self.get_logger().info("✅ connectionStatus = True (TCP verbunden)")
        else:
            self.get_logger().info("⚠️ connectionStatus = False (TCP getrennt)")

    def _publish_connection_status(self) -> None:
        try:
            msg = Bool()
            msg.data = self._connected
            self.pub_conn.publish(msg)
        except Exception as e:
            self.get_logger().error(f"❌ connectionStatus periodic publish failed: {e}")

    def _ensure_connected(self) -> bool:
        if self._connected:
            return True
        try:
            self.get_logger().info("🔄 TCP-Verbindungsversuch zum Omron-Server ...")
            self._client.connect()
            self._set_connection(True)
            self.get_logger().info("✅ TCP-Verbindung zum Omron-Server steht.")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ TCP connect failed: {e}")
            self._set_connection(False)
            return False

    # ------------------------------------------------------------------
    # Helper für OmronTcpClient
    # ------------------------------------------------------------------
    def _send_line(self, line: str) -> None:
        if hasattr(self._client, "send_line"):
            self._client.send_line(line)
        elif hasattr(self._client, "_send_line"):
            self._client._send_line(line)
        else:
            raise RuntimeError("OmronTcpClient hat weder send_line() noch _send_line().")

    def _recv_line(self) -> str | None:
        if hasattr(self._client, "recv_line"):
            return self._client.recv_line()
        if hasattr(self._client, "_recv_line"):
            return self._client._recv_line()
        raise RuntimeError("OmronTcpClient hat weder recv_line() noch _recv_line().")

    # ------------------------------------------------------------------
    # RX-Poll
    # ------------------------------------------------------------------
    def _poll_rx(self) -> None:
        if not self._connected:
            return

        max_lines_per_tick = 10
        count = 0

        while count < max_lines_per_tick:
            try:
                line = self._recv_line()
            except Exception as e:
                self.get_logger().warning(f"⚠️ RX poll Fehler: {e}")
                self._set_connection(False)
                break

            if line is None:
                break

            text = line.strip()
            if not text:
                break

            self.get_logger().info(f"⬅️ RECV (poll): {text}")
            self.pub_raw_rx.publish(String(data=text))
            count += 1

    # ------------------------------------------------------------------
    # Command von UI -> TCP
    # ------------------------------------------------------------------
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
            self._send_line(cmd)
        except Exception as e:
            self.get_logger().error(f"❌ Send error: {e}")
            self._set_connection(False)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def destroy_node(self):
        self._set_connection(False)
        try:
            self._client.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OmronTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
