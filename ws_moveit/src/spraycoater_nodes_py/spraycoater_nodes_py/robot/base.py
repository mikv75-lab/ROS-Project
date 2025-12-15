# spraycoater_nodes_py/robot/base.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from spraycoater_nodes_py.utils.config_hub import topics

NODE_KEY = "robot"


class BaseRobot(Node):
    """
    Gemeinsame Basis für:
      - SimRobot
      - OmronRobot (real)

    Aufgaben:
      - Statusverwaltung
      - Statuspublishing (topics.yaml!)
      - gemeinsame Helfer
    """

    def __init__(self, node_name: str = "robot") -> None:
        super().__init__(node_name)

        self.log = self.get_logger()
        self.cfg = topics()

        # --------------------------
        # interner Zustand
        # --------------------------
        self._connected = False
        self._initialized = False
        self._power = False
        self._servo_enabled = False
        self._moving = False
        self._estop = False
        self._mode = "UNKNOWN"
        self._last_error: str = ""

        self._tcp_pose = PoseStamped()
        self._joints = JointState()

        # --------------------------
        # Publisher (aus topics.yaml)
        # --------------------------
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

        # --------------------------
        # zyklisches Publishen
        # --------------------------
        self._timer = self.create_timer(0.05, self._publish_state)

        self.log.info("🤖 BaseRobot initialisiert")

    # ==========================================================
    # Helpers: Publisher / Subscriber
    # ==========================================================

    def _make_pub(self, msg_type, topic_id: str):
        return self.create_publisher(
            msg_type,
            self.cfg.publish_topic(NODE_KEY, topic_id),
            self.cfg.qos_by_id("publish", NODE_KEY, topic_id),
        )

    def _make_sub(self, msg_type, topic_id: str, cb):
        return self.create_subscription(
            msg_type,
            self.cfg.subscribe_topic(NODE_KEY, topic_id),
            cb,
            self.cfg.qos_by_id("subscribe", NODE_KEY, topic_id),
        )

    # ==========================================================
    # Status Setter
    # ==========================================================

    def _set_error(self, text: str):
        self._last_error = text
        self.log.error(text)

    def _set_mode(self, mode: str):
        self._mode = mode
        self.log.info(f"[robot] mode → {mode}")

    # ==========================================================
    # Publish Loop
    # ==========================================================

    def _publish_state(self):
        now = self.get_clock().now().to_msg()

        self.pub_connection.publish(Bool(data=self._connected))
        self.pub_initialized.publish(Bool(data=self._initialized))
        self.pub_power.publish(Bool(data=self._power))
        self.pub_servo_enabled.publish(Bool(data=self._servo_enabled))
        self.pub_moving.publish(Bool(data=self._moving))
        self.pub_estop.publish(Bool(data=self._estop))
        self.pub_mode.publish(String(data=self._mode))
        self.pub_errors.publish(String(data=self._last_error))

        # TCP Pose
        self._tcp_pose.header.stamp = now
        self.pub_tcp_pose.publish(self._tcp_pose)

        # Joints
        self.pub_joints.publish(self._joints)
