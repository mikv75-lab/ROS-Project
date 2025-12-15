#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/omron_tcp_bridge.py
"""
Omron TCP Bridge (ROS 2 Node)

Ziel:
  - ROS Topics (via config_hub/topics.yaml) <-> TCP Socket zum Omron V+ Controller / ACE-Server
  - KEINE hardcodierten Topic-Namen (Namespace kommt über Node-namespace / Launchfile)
  - robustes Auto-Reconnect + latched connection_status
  - raw_tx/raw_rx Topics für Debugging (UI ServiceTab)

topics.yaml (group "omron") erwartet typischerweise:
  subscribe:
    - command_in            (std_msgs/String)  -> send to TCP
    - connection_status_in  (std_msgs/Bool)    (optional: externes Status-Feed)
  publish:
    - raw_tx                (std_msgs/String)
    - raw_rx                (std_msgs/String)
    - connection_status     (std_msgs/Bool)    (latched empfohlen)
    - command_out           (std_msgs/String)  (optional Echo / Bridge-Output)

Hinweis:
  Dieses Node implementiert bewusst nur die Transport-Schicht.
  Interpretation/Parsing von Responses (ACKs, Fehlercodes, etc.) kann in einem
  separaten Interpreter-Node erfolgen (oder in einer erweiterten Version hier).
"""
from __future__ import annotations

import socket
import threading
import queue
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String as MsgString, Bool as MsgBool

from spraycoater_nodes_py.utils.config_hub import topics

NODE_KEY = "omron"


class OmronTcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("omron_tcp_bridge")

        # ---------------- Parameters ----------------
        self.declare_parameter("backend", "default")
        self.backend: str = (
            self.get_parameter("backend").get_parameter_value().string_value or "default"
        )

        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 5000)
        self.declare_parameter("reconnect_s", 1.0)
        self.declare_parameter("socket_timeout_s", 0.2)
        self.declare_parameter("line_ending", "\n")
        self.declare_parameter("max_rx_line_len", 8192)

        self.host: str = self.get_parameter("host").value
        self.port: int = int(self.get_parameter("port").value)
        self.reconnect_s: float = float(self.get_parameter("reconnect_s").value)
        self.socket_timeout_s: float = float(self.get_parameter("socket_timeout_s").value)
        self.line_ending: str = str(self.get_parameter("line_ending").value or "\n")
        self.max_rx_line_len: int = int(self.get_parameter("max_rx_line_len").value)

        # ---------------- Topics (config_hub) ----------------
        self.cfg_topics = topics()

        # SUB: UI/Nodes -> Bridge
        topic_cmd_in = self.cfg_topics.subscribe_topic(NODE_KEY, "command_in")
        qos_cmd_in = self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "command_in")

        # Optional: externer Status-Input (wird durchgereicht)
        topic_conn_in = None
        qos_conn_in = None
        try:
            topic_conn_in = self.cfg_topics.subscribe_topic(NODE_KEY, "connection_status_in")
            qos_conn_in = self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "connection_status_in")
        except Exception:
            topic_conn_in = None

        # PUB: Bridge -> UI/Nodes
        topic_raw_tx = self.cfg_topics.publish_topic(NODE_KEY, "raw_tx")
        qos_raw_tx = self.cfg_topics.qos_by_id("publish", NODE_KEY, "raw_tx")

        topic_raw_rx = self.cfg_topics.publish_topic(NODE_KEY, "raw_rx")
        qos_raw_rx = self.cfg_topics.qos_by_id("publish", NODE_KEY, "raw_rx")

        topic_conn = self.cfg_topics.publish_topic(NODE_KEY, "connection_status")
        qos_conn = self.cfg_topics.qos_by_id("publish", NODE_KEY, "connection_status")

        # command_out ist optional (manche Setups wollen Echo / Broadcast)
        topic_cmd_out = None
        qos_cmd_out = None
        try:
            topic_cmd_out = self.cfg_topics.publish_topic(NODE_KEY, "command_out")
            qos_cmd_out = self.cfg_topics.qos_by_id("publish", NODE_KEY, "command_out")
        except Exception:
            topic_cmd_out = None

        # ---------------- Publishers ----------------
        self.pub_raw_tx = self.create_publisher(MsgString, topic_raw_tx, qos_raw_tx)
        self.pub_raw_rx = self.create_publisher(MsgString, topic_raw_rx, qos_raw_rx)
        self.pub_conn = self.create_publisher(MsgBool, topic_conn, qos_conn)

        self.pub_cmd_out = None
        if topic_cmd_out:
            self.pub_cmd_out = self.create_publisher(MsgString, topic_cmd_out, qos_cmd_out)

        # ---------------- Subscribers ----------------
        self.create_subscription(MsgString, topic_cmd_in, self._on_command_in, qos_cmd_in)
        if topic_conn_in and qos_conn_in:
            self.create_subscription(MsgBool, topic_conn_in, self._on_connection_in, qos_conn_in)

        # ---------------- TCP worker state ----------------
        self._sock: Optional[socket.socket] = None
        self._connected: bool = False

        self._tx_q: "queue.Queue[str]" = queue.Queue(maxsize=500)
        self._rx_q: "queue.Queue[str]" = queue.Queue(maxsize=500)

        self._stop_evt = threading.Event()
        self._thread = threading.Thread(target=self._worker, name="omron_tcp_bridge", daemon=True)
        self._thread.start()

        # Timer: RX flush + connection publish
        self._last_conn_pub: Optional[bool] = None
        self.create_timer(0.05, self._pump_ros)

        self.get_logger().info(
            f"✅ OmronTcpBridge aktiv (backend='{self.backend}', ns='{self.get_namespace() or '/'}') "
            f"host={self.host}:{self.port}, reconnect={self.reconnect_s:.2f}s"
        )

    # ---------------- ROS callbacks ----------------

    def _on_command_in(self, msg: MsgString) -> None:
        line = (msg.data or "").strip()
        if not line:
            return

        # Debug publish
        self.pub_raw_tx.publish(MsgString(data=line))
        if self.pub_cmd_out is not None:
            # optional echo/back-channel
            self.pub_cmd_out.publish(MsgString(data=line))

        try:
            self._tx_q.put_nowait(line)
        except queue.Full:
            self.get_logger().warning("TX queue full – command dropped.")

    def _on_connection_in(self, msg: MsgBool) -> None:
        # Externer Status-Input: publishen wir einfach weiter (UI kann das nutzen).
        self._publish_connection(bool(msg.data))

    # ---------------- Worker ----------------

    def _connect(self) -> Optional[socket.socket]:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(self.socket_timeout_s)
        s.connect((self.host, self.port))
        return s

    def _close_sock(self) -> None:
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                self._sock.close()
            except Exception:
                pass
        self._sock = None

    def _set_connected(self, val: bool) -> None:
        self._connected = val

    def _worker(self) -> None:
        buf = b""
        le = (self.line_ending or "\n").encode("utf-8", errors="ignore")

        while not self._stop_evt.is_set():
            # ensure connected
            if self._sock is None:
                try:
                    self._sock = self._connect()
                    self._set_connected(True)
                    buf = b""
                except Exception:
                    self._set_connected(False)
                    self._close_sock()
                    time.sleep(self.reconnect_s)
                    continue

            # --- send pending ---
            try:
                while True:
                    try:
                        line = self._tx_q.get_nowait()
                    except queue.Empty:
                        break
                    payload = (line + self.line_ending).encode("utf-8", errors="replace")
                    try:
                        self._sock.sendall(payload)  # type: ignore[union-attr]
                    except Exception:
                        # connection broke -> reconnect
                        self._set_connected(False)
                        self._close_sock()
                        break
            except Exception:
                # keep worker alive
                self._set_connected(False)
                self._close_sock()

            if self._sock is None:
                time.sleep(self.reconnect_s)
                continue

            # --- receive ---
            try:
                chunk = self._sock.recv(4096)  # type: ignore[union-attr]
                if not chunk:
                    # peer closed
                    self._set_connected(False)
                    self._close_sock()
                    time.sleep(self.reconnect_s)
                    continue
                buf += chunk
                # split lines
                while True:
                    idx = buf.find(le)
                    if idx < 0:
                        # avoid unbounded buffer if sender never sends newline
                        if len(buf) > self.max_rx_line_len:
                            buf = b""
                        break
                    raw = buf[:idx]
                    buf = buf[idx + len(le):]
                    try:
                        line = raw.decode("utf-8", errors="replace").strip()
                    except Exception:
                        line = ""
                    if not line:
                        continue
                    try:
                        self._rx_q.put_nowait(line)
                    except queue.Full:
                        # drop oldest by draining one
                        try:
                            _ = self._rx_q.get_nowait()
                            self._rx_q.put_nowait(line)
                        except Exception:
                            pass
            except socket.timeout:
                # normal
                pass
            except Exception:
                self._set_connected(False)
                self._close_sock()
                time.sleep(self.reconnect_s)

        # stop: cleanup
        self._set_connected(False)
        self._close_sock()

    # ---------------- ROS timer pump ----------------

    def _publish_connection(self, connected: bool) -> None:
        if self._last_conn_pub is connected:
            return
        self._last_conn_pub = connected
        self.pub_conn.publish(MsgBool(data=connected))

    def _pump_ros(self) -> None:
        # publish connection state on change
        self._publish_connection(self._connected)

        # drain rx queue
        n = 0
        while n < 50:
            try:
                line = self._rx_q.get_nowait()
            except queue.Empty:
                break
            self.pub_raw_rx.publish(MsgString(data=line))
            n += 1

    # ---------------- lifecycle ----------------

    def destroy_node(self):
        try:
            self._stop_evt.set()
            if self._thread.is_alive():
                self._thread.join(timeout=1.0)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
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
