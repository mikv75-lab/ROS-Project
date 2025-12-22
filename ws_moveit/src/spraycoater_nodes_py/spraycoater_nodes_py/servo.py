#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spraycoater_nodes_py/servo.py

Servo Wrapper Node (SSoT via config_hub topics/qos):
- UI -> Node (subscribe ids): cartesian_mm, joint_jog, set_mode, set_frame
- Node -> moveit_servo (publish ids): twist_out, joint_out   (WICHTIG: IDs aus topics.yaml)

Service (hardcoded, NICHT in topics.yaml):
  - <ns>/servo_node/switch_command_type
"""

from __future__ import annotations

import time
import logging
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String as MsgString
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType

from spraycoater_nodes_py.utils.config_hub import topics

_LOG = logging.getLogger(__name__)
NODE_KEY = "servo"

MODE_MAP = {
    "JOINT_JOG": ServoCommandType.Request.JOINT_JOG,
    "JOINT":     ServoCommandType.Request.JOINT_JOG,
    "J":         ServoCommandType.Request.JOINT_JOG,
    "TWIST":     ServoCommandType.Request.TWIST,
    "CART":      ServoCommandType.Request.TWIST,
    "CARTESIAN": ServoCommandType.Request.TWIST,
    "T":         ServoCommandType.Request.TWIST,
}


class ServoWrapper(Node):
    def __init__(self) -> None:
        super().__init__("servo")

        self.loader = topics()

        # -------------------------
        # IN: UI -> Node (subscribe)
        # -------------------------
        self._topic_in_twist = self.loader.subscribe_topic(NODE_KEY, "cartesian_mm")
        self._qos_in_twist = self.loader.qos_by_id("subscribe", NODE_KEY, "cartesian_mm")

        self._topic_in_joint = self.loader.subscribe_topic(NODE_KEY, "joint_jog")
        self._qos_in_joint = self.loader.qos_by_id("subscribe", NODE_KEY, "joint_jog")

        self._topic_in_mode = self.loader.subscribe_topic(NODE_KEY, "set_mode")
        self._qos_in_mode = self.loader.qos_by_id("subscribe", NODE_KEY, "set_mode")

        self._topic_in_frame = self.loader.subscribe_topic(NODE_KEY, "set_frame")
        self._qos_in_frame = self.loader.qos_by_id("subscribe", NODE_KEY, "set_frame")

        # -------------------------
        # OUT: Node -> moveit_servo (publish)
        # IDs MUST match topics.yaml (dein Log zeigt: twist_out / joint_out)
        # -------------------------
        self._topic_out_twist = self.loader.publish_topic(NODE_KEY, "twist_out")
        self._qos_out_twist = self.loader.qos_by_id("publish", NODE_KEY, "twist_out")

        self._topic_out_joint = self.loader.publish_topic(NODE_KEY, "joint_out")
        self._qos_out_joint = self.loader.qos_by_id("publish", NODE_KEY, "joint_out")

        # -------------------------
        # HARDCODED service (Namespace via Node)
        # -------------------------
        self._switch_srv_name = self._resolve_srv_name("servo_node/switch_command_type")
        self._switch_client = self.create_client(ServoCommandType, self._switch_srv_name)

        # pubs
        self._pub_twist = self.create_publisher(TwistStamped, self._topic_out_twist, self._qos_out_twist)
        self._pub_joint = self.create_publisher(JointJog, self._topic_out_joint, self._qos_out_joint)

        # subs
        self.create_subscription(TwistStamped, self._topic_in_twist, self._on_twist, self._qos_in_twist)
        self.create_subscription(JointJog, self._topic_in_joint, self._on_joint, self._qos_in_joint)
        self.create_subscription(MsgString, self._topic_in_mode, self._on_set_mode, self._qos_in_mode)
        self.create_subscription(MsgString, self._topic_in_frame, self._on_set_frame, self._qos_in_frame)

        self._pending_mode: int = -1
        self._current_mode: Optional[int] = None
        self._current_frame: str = "world"

        self._rx_twist_count = 0
        self._rx_joint_count = 0
        self._last_rx_time: Optional[float] = None

        self._timer_bind = self.create_timer(1.0, self._bind_timer)
        self._timer_status = self.create_timer(1.0, self._status_timer)

        _LOG.info("✅ ServoWrapper up (ns='%s')", self.get_namespace() or "/")
        _LOG.info("   IN : %s | %s | %s | %s", self._topic_in_twist, self._topic_in_joint, self._topic_in_mode, self._topic_in_frame)
        _LOG.info("   OUT: %s | %s", self._topic_out_twist, self._topic_out_joint)
        _LOG.info("   SRV(hardcoded): %s", self._switch_srv_name)

    def _resolve_srv_name(self, name: str) -> str:
        name = (name or "").strip()
        if not name:
            raise ValueError("empty service name")
        if name.startswith("/"):
            return name
        ns = (self.get_namespace() or "").rstrip("/")
        return f"{ns}/{name}" if ns else f"/{name}"

    def _bind_timer(self) -> None:
        if self._switch_client.service_is_ready():
            return
        if not self._switch_client.wait_for_service(timeout_sec=0.0):
            _LOG.warning("switch_command_type not ready: '%s' (retrying)", self._switch_srv_name)

    def _call_switch(self, mode_int: int) -> None:
        if not self._switch_client.service_is_ready():
            return
        if self._current_mode == mode_int or self._pending_mode == mode_int:
            return

        req = ServoCommandType.Request()
        req.command_type = int(mode_int)
        fut = self._switch_client.call_async(req)
        self._pending_mode = mode_int

        def _done_cb(f):
            self._pending_mode = -1
            try:
                resp = f.result()
                ok = bool(getattr(resp, "success", True))
                if ok:
                    self._current_mode = mode_int
                    _LOG.info("✅ switch_command_type -> %s (%d)", self._mode_str(mode_int), mode_int)
                else:
                    _LOG.error("❌ switch_command_type FAILED -> %s (%d)", self._mode_str(mode_int), mode_int)
            except Exception as e:
                _LOG.exception("❌ switch_command_type EXC: %s", e)

        fut.add_done_callback(_done_cb)

    @staticmethod
    def _mode_str(mode_int: int) -> str:
        return "JOINT_JOG" if mode_int == ServoCommandType.Request.JOINT_JOG else "TWIST"

    def _on_set_mode(self, msg: MsgString) -> None:
        mode = (msg.data or "").strip().upper()
        mode_int = MODE_MAP.get(mode)
        if mode_int is None:
            _LOG.warning("[set_mode] unknown '%s'", mode)
            return
        self._call_switch(mode_int)

    def _on_set_frame(self, msg: MsgString) -> None:
        frame = (msg.data or "").strip()
        if not frame:
            return
        self._current_frame = frame
        _LOG.info("[set_frame] -> '%s'", frame)

    def _on_twist(self, msg: TwistStamped) -> None:
        self._rx_twist_count += 1
        self._last_rx_time = time.time()
        self._pub_twist.publish(msg)

    def _on_joint(self, msg: JointJog) -> None:
        self._rx_joint_count += 1
        self._last_rx_time = time.time()
        self._pub_joint.publish(msg)

    def _status_timer(self) -> None:
        twist_subs = self.count_subscribers(self._topic_out_twist)
        joint_subs = self.count_subscribers(self._topic_out_joint)
        age = -1.0 if self._last_rx_time is None else (time.time() - self._last_rx_time)
        _LOG.info(
            "[status] out_subs(twist=%d, joint=%d) last_rx_age=%.2fs switch_srv=%s pending=%d",
            twist_subs, joint_subs, age,
            "ready" if self._switch_client.service_is_ready() else "no",
            self._pending_mode,
        )


def main() -> None:
    logging.basicConfig(level=logging.INFO)
    rclpy.init()
    node = ServoWrapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
