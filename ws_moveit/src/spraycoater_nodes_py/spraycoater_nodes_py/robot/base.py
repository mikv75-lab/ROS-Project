# spraycoater_nodes_py/robot/base.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from abc import ABC, abstractmethod

from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from spraycoater_nodes_py.utils.config_hub import topics

NODE_KEY = "robot"


class BaseRobot(Node, ABC):
    """
    Gemeinsame Basis für:
      - SimRobot
      - OmronRobot (real)

    Clean (kein Backwards/Fallback):
      - topics.yaml ist Source-of-Truth
      - keine self.loader, keine alten APIs
      - Sim/Real implementieren nur die Hooks
    """

    def __init__(self, node_name: str = "robot") -> None:
        super().__init__(node_name)

        self.log = self.get_logger()
        self.cfg = topics()

        # --------------------------
        # interner Zustand
        # --------------------------
        self._connected: bool = False
        self._initialized: bool = False
        self._power: bool = False
        self._servo_enabled: bool = False
        self._moving: bool = False
        self._estop: bool = False
        self._mode: str = "UNKNOWN"
        self._last_error: str = ""

        # Daten, die publisht werden
        self._tcp_pose: PoseStamped = PoseStamped()
        self._joints: JointState = JointState()

        # --------------------------
        # Publisher (topics.yaml)
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
        # zyklischer Tick (20 Hz)
        # --------------------------
        self._timer = self.create_timer(0.05, self._tick)

        self.log.info("🤖 BaseRobot initialisiert")

    # ==========================================================
    # Helpers: Pub/Sub via config_hub
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
    # Clean Hooks (Sim/Real)
    # ==========================================================

    @abstractmethod
    def on_tick(self, now: Time) -> None:
        """
        Wird alle 50ms aufgerufen, bevor publisht wird.
        Sim/Real soll hier:
          - Status updaten (_connected/_moving/...)
          - _tcp_pose und _joints setzen (falls neue Daten)
        """
        raise NotImplementedError

    # ==========================================================
    # Status Setter (nur intern benutzen)
    # ==========================================================

    def _set_error(self, text: str) -> None:
        self._last_error = str(text or "")
        if self._last_error:
            self.log.error(self._last_error)

    def _set_mode(self, mode: str) -> None:
        self._mode = str(mode or "UNKNOWN")

    # ==========================================================
    # Tick + Publish
    # ==========================================================

    def _tick(self) -> None:
        now = self.get_clock().now()

        # Hook: Sim/Real aktualisiert seinen Zustand
        try:
            self.on_tick(now)
        except Exception as e:
            # Backend-Fehler soll den Node nicht killen
            self._set_error(f"[robot] on_tick failed: {type(e).__name__}: {e}")

        # Status publishen
        self.pub_connection.publish(Bool(data=self._connected))
        self.pub_initialized.publish(Bool(data=self._initialized))
        self.pub_power.publish(Bool(data=self._power))
        self.pub_servo_enabled.publish(Bool(data=self._servo_enabled))
        self.pub_moving.publish(Bool(data=self._moving))
        self.pub_estop.publish(Bool(data=self._estop))
        self.pub_mode.publish(String(data=self._mode))
        self.pub_errors.publish(String(data=self._last_error))

        # TCP Pose publishen (Stamp wird IMMER hier gesetzt)
        self._tcp_pose.header.stamp = now.to_msg()
        self.pub_tcp_pose.publish(self._tcp_pose)

        # JointState publishen
        self.pub_joints.publish(self._joints)
