# spraycoater_nodes_py/robot/base.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from abc import ABC
from typing import Optional, Sequence

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

    ✅ Event-driven / onChange:
      - Kein Timer, kein Polling
      - Publish nur bei Änderung
      - tcp_pose/joints werden explizit über publish_*() publisht (on events)
    """

    # Toleranzen für "ändert sich wirklich?" (damit Joint/TCP nicht wegen Float-Rauschen spammt)
    JOINT_EPS: float = 1e-6
    POSE_POS_EPS: float = 1e-6
    POSE_QUAT_EPS: float = 1e-6

    def __init__(self, node_name: str = "robot") -> None:
        super().__init__(node_name)

        self.log = self.get_logger()
        self.cfg = topics()

        # --------------------------
        # interner Zustand (letzter publizierter Wert)
        # --------------------------
        self._connected: bool = False
        self._initialized: bool = False
        self._power: bool = False
        self._servo_enabled: bool = False
        self._moving: bool = False
        self._estop: bool = False
        self._mode: str = "UNKNOWN"
        self._last_error: str = ""

        # letzte publizierte Messages (für onChange)
        self._tcp_pose_last: Optional[PoseStamped] = None
        self._joints_last: Optional[JointState] = None

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

        self.log.info("🤖 BaseRobot initialisiert (onChange, no timer)")

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
    # OnChange Setter -> Publish
    # ==========================================================

    def set_connected(self, v: bool, *, force: bool = False) -> None:
        v = bool(v)
        if force or (v != self._connected):
            self._connected = v
            self.pub_connection.publish(Bool(data=v))

    def set_initialized(self, v: bool, *, force: bool = False) -> None:
        v = bool(v)
        if force or (v != self._initialized):
            self._initialized = v
            self.pub_initialized.publish(Bool(data=v))

    def set_power(self, v: bool, *, force: bool = False) -> None:
        v = bool(v)
        if force or (v != self._power):
            self._power = v
            self.pub_power.publish(Bool(data=v))

    def set_servo_enabled(self, v: bool, *, force: bool = False) -> None:
        v = bool(v)
        if force or (v != self._servo_enabled):
            self._servo_enabled = v
            self.pub_servo_enabled.publish(Bool(data=v))

    def set_moving(self, v: bool, *, force: bool = False) -> None:
        v = bool(v)
        if force or (v != self._moving):
            self._moving = v
            self.pub_moving.publish(Bool(data=v))

    def set_estop(self, v: bool, *, force: bool = False) -> None:
        v = bool(v)
        if force or (v != self._estop):
            self._estop = v
            self.pub_estop.publish(Bool(data=v))

    def set_mode(self, mode: str, *, force: bool = False) -> None:
        mode = (mode or "UNKNOWN").strip() or "UNKNOWN"
        if force or (mode != self._mode):
            self._mode = mode
            self.pub_mode.publish(String(data=mode))

    def set_error(self, text: str, *, force: bool = False, log_on_set: bool = True) -> None:
        """
        Setzt Fehlertext. Publisht nur bei Änderung.
        - text == "" -> cleared
        """
        text = str(text or "")
        if force or (text != self._last_error):
            self._last_error = text
            self.pub_errors.publish(String(data=text))
            if log_on_set and text:
                self.log.error(text)

    # ==========================================================
    # Publish helpers for Pose / Joints (onChange)
    # ==========================================================

    def publish_tcp_pose(self, msg: PoseStamped, *, now: Optional[Time] = None, force: bool = False) -> None:
        """
        Publisht tcp_pose nur bei Änderung (oder force=True).
        Stamp wird beim Publish gesetzt (now oder Clock.now()).
        """
        if msg is None:
            return

        if not force and self._tcp_pose_last is not None:
            if self._pose_equal(self._tcp_pose_last, msg):
                return

        # stamp setzen
        t = now or self.get_clock().now()
        msg.header.stamp = t.to_msg()

        self.pub_tcp_pose.publish(msg)
        self._tcp_pose_last = self._copy_pose(msg)

    def publish_joints(self, msg: JointState, *, now: Optional[Time] = None, force: bool = False) -> None:
        """
        Publisht JointState nur bei Änderung (oder force=True).
        Stamp wird übernommen/gesetzt, wenn leer.
        """
        if msg is None:
            return

        if not force and self._joints_last is not None:
            if self._joints_equal(self._joints_last, msg):
                return

        # stamp optional setzen (nur wenn nicht gesetzt)
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            t = now or self.get_clock().now()
            msg.header.stamp = t.to_msg()

        self.pub_joints.publish(msg)
        self._joints_last = self._copy_joints(msg)

    def publish_all_once(self) -> None:
        """
        Einmaliger Snapshot, z.B. beim Start.
        """
        self.set_connected(self._connected, force=True)
        self.set_initialized(self._initialized, force=True)
        self.set_power(self._power, force=True)
        self.set_servo_enabled(self._servo_enabled, force=True)
        self.set_moving(self._moving, force=True)
        self.set_estop(self._estop, force=True)
        self.set_mode(self._mode, force=True)
        self.set_error(self._last_error, force=True, log_on_set=False)

        # Pose/Joints nur wenn vorhanden
        if self._tcp_pose_last is not None:
            self.publish_tcp_pose(self._copy_pose(self._tcp_pose_last), force=True)
        if self._joints_last is not None:
            self.publish_joints(self._copy_joints(self._joints_last), force=True)

    # ==========================================================
    # Equality helpers
    # ==========================================================

    def _pose_equal(self, a: PoseStamped, b: PoseStamped) -> bool:
        try:
            ap, aq = a.pose.position, a.pose.orientation
            bp, bq = b.pose.position, b.pose.orientation
            if abs(ap.x - bp.x) > self.POSE_POS_EPS: return False
            if abs(ap.y - bp.y) > self.POSE_POS_EPS: return False
            if abs(ap.z - bp.z) > self.POSE_POS_EPS: return False
            if abs(aq.x - bq.x) > self.POSE_QUAT_EPS: return False
            if abs(aq.y - bq.y) > self.POSE_QUAT_EPS: return False
            if abs(aq.z - bq.z) > self.POSE_QUAT_EPS: return False
            if abs(aq.w - bq.w) > self.POSE_QUAT_EPS: return False
            if (a.header.frame_id or "") != (b.header.frame_id or ""): return False
            return True
        except Exception:
            return False

    def _joints_equal(self, a: JointState, b: JointState) -> bool:
        try:
            if list(a.name) != list(b.name):
                return False
            ap = list(a.position or [])
            bp = list(b.position or [])
            if len(ap) != len(bp):
                return False
            for x, y in zip(ap, bp):
                if abs(float(x) - float(y)) > self.JOINT_EPS:
                    return False
            return True
        except Exception:
            return False

    def _copy_pose(self, src: PoseStamped) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = src.header.frame_id
        out.header.stamp = src.header.stamp
        out.pose.position.x = src.pose.position.x
        out.pose.position.y = src.pose.position.y
        out.pose.position.z = src.pose.position.z
        out.pose.orientation.x = src.pose.orientation.x
        out.pose.orientation.y = src.pose.orientation.y
        out.pose.orientation.z = src.pose.orientation.z
        out.pose.orientation.w = src.pose.orientation.w
        return out

    def _copy_joints(self, src: JointState) -> JointState:
        out = JointState()
        out.header = src.header
        out.name = list(src.name or [])
        out.position = list(src.position or [])
        out.velocity = list(src.velocity or [])
        out.effort = list(src.effort or [])
        return out
