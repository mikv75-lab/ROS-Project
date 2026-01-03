#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spraycoater_nodes_py/servo.py

Servo Wrapper Node (SSoT via config_hub topics/qos):
- UI -> Node (subscribe ids): cartesian_mm, joint_jog, set_mode, set_frame
- Node -> moveit_servo (publish ids): twist_out, joint_out   (WICHTIG: IDs aus topics.yaml)

Service (hardcoded, NICHT in topics.yaml):
  - <ns>/servo_node/switch_command_type

Fixes vs "old working" version:
- Bootstrap default command_type after service becomes ready (prevents:
  "Command type has not been set, cannot accept input")
- Implicitly ensure correct command_type on first incoming Twist/JointJog (debounced)
- Apply set_frame by writing header.frame_id before forwarding
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
        # IDs MUST match topics.yaml
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

        # ---- internal state ----
        self._pending_mode: int = -1
        self._current_mode: Optional[int] = None
        self._desired_mode: Optional[int] = None  # last requested/needed
        self._last_switch_try_t: float = 0.0

        self._current_frame: str = "world"

        self._rx_twist_count = 0
        self._rx_joint_count = 0
        self._last_rx_time: Optional[float] = None

        # tuning
        self._switch_debounce_s = 0.25
        self._bootstrap_default_mode = ServoCommandType.Request.JOINT_JOG

        # timers
        self._timer_bind = self.create_timer(0.5, self._bind_timer)
        self._timer_status = self.create_timer(1.0, self._status_timer)
        self._timer_ensure = self.create_timer(0.1, self._ensure_timer)

        _LOG.info("✅ ServoWrapper up (ns='%s')", self.get_namespace() or "/")
        _LOG.info("   IN : %s | %s | %s | %s", self._topic_in_twist, self._topic_in_joint, self._topic_in_mode, self._topic_in_frame)
        _LOG.info("   OUT: %s | %s", self._topic_out_twist, self._topic_out_joint)
        _LOG.info("   SRV(hardcoded): %s", self._switch_srv_name)

    # -------------------------
    # helpers
    # -------------------------
    def _resolve_srv_name(self, name: str) -> str:
        name = (name or "").strip()
        if not name:
            raise ValueError("empty service name")
        if name.startswith("/"):
            return name
        ns = (self.get_namespace() or "").rstrip("/")
        return f"{ns}/{name}" if ns else f"/{name}"

    @staticmethod
    def _mode_str(mode_int: int) -> str:
        if mode_int == ServoCommandType.Request.JOINT_JOG:
            return "JOINT_JOG"
        if mode_int == ServoCommandType.Request.TWIST:
            return "TWIST"
        return f"UNKNOWN({mode_int})"

    def _maybe_call_switch(self, mode_int: int) -> None:
        """
        Debounced, idempotent async service call.
        Sets:
          - _desired_mode immediately
          - _current_mode only on successful response
        """
        self._desired_mode = int(mode_int)

        if not self._switch_client.service_is_ready():
            return

        # already in desired or currently pending that desired
        if self._current_mode == mode_int or self._pending_mode == mode_int:
            return

        now = time.time()
        if (now - self._last_switch_try_t) < self._switch_debounce_s:
            return
        self._last_switch_try_t = now

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

    # -------------------------
    # timers
    # -------------------------
    def _bind_timer(self) -> None:
        # keep connection state quiet, but detect readiness reliably
        if self._switch_client.service_is_ready():
            return
        if not self._switch_client.wait_for_service(timeout_sec=0.0):
            # only warn occasionally to avoid log spam
            _LOG.debug("switch_command_type not ready: '%s' (retrying)", self._switch_srv_name)

    def _ensure_timer(self) -> None:
        """
        Bootstrap: once service becomes ready, make sure ServoNode has a command type set.
        Also: if something asked for a mode while service wasn't ready, apply it now.
        """
        if not self._switch_client.service_is_ready():
            return

        # 1) bootstrap default if ServoNode hasn't been configured yet
        if self._current_mode is None and self._desired_mode is None:
            self._maybe_call_switch(self._bootstrap_default_mode)
            return

        # 2) apply queued desired mode
        if self._desired_mode is not None:
            self._maybe_call_switch(self._desired_mode)

    # -------------------------
    # callbacks (UI -> wrapper)
    # -------------------------
    def _on_set_mode(self, msg: MsgString) -> None:
        mode = (msg.data or "").strip().upper()
        mode_int = MODE_MAP.get(mode)
        if mode_int is None:
            _LOG.warning("[set_mode] unknown '%s'", mode)
            return
        self._maybe_call_switch(mode_int)

    def _on_set_frame(self, msg: MsgString) -> None:
        frame = (msg.data or "").strip()
        if not frame:
            return
        if frame == self._current_frame:
            return
        self._current_frame = frame
        _LOG.info("[set_frame] -> '%s'", frame)

    def _on_twist(self, msg: TwistStamped) -> None:
        self._rx_twist_count += 1
        self._last_rx_time = time.time()

        # Ensure ServoNode is in TWIST mode before forwarding
        self._maybe_call_switch(ServoCommandType.Request.TWIST)

        # Apply chosen frame (important!)
        try:
            msg.header.frame_id = self._current_frame
        except Exception:
            pass

        self._pub_twist.publish(msg)

    def _on_joint(self, msg: JointJog) -> None:
        self._rx_joint_count += 1
        self._last_rx_time = time.time()

        # Ensure ServoNode is in JOINT_JOG mode before forwarding
        self._maybe_call_switch(ServoCommandType.Request.JOINT_JOG)

        # Apply chosen frame (MoveIt Servo expects header.frame_id in some setups; harmless otherwise)
        try:
            msg.header.frame_id = self._current_frame
        except Exception:
            pass

        self._pub_joint.publish(msg)

    # -------------------------
    # status
    # -------------------------
    def _status_timer(self) -> None:
        twist_subs = self.count_subscribers(self._topic_out_twist)
        joint_subs = self.count_subscribers(self._topic_out_joint)
        age = -1.0 if self._last_rx_time is None else (time.time() - self._last_rx_time)

        _LOG.info(
            "[status] out_subs(twist=%d, joint=%d) last_rx_age=%.2fs switch_srv=%s cur=%s desired=%s pending=%s frame=%s",
            twist_subs,
            joint_subs,
            age,
            "ready" if self._switch_client.service_is_ready() else "no",
            "-" if self._current_mode is None else self._mode_str(self._current_mode),
            "-" if self._desired_mode is None else self._mode_str(self._desired_mode),
            "-" if self._pending_mode < 0 else self._mode_str(self._pending_mode),
            self._current_frame,
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
