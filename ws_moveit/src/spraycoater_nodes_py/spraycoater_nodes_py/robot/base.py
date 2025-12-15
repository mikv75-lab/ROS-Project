# spraycoater_nodes_py/robot/base.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import importlib
from typing import Any, Callable, Dict, Optional, Type

from rclpy.node import Node

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from spraycoater_nodes_py.utils.config_hub import topics


def _resolve_msg_type(type_str: str) -> Type[Any]:
    """
    'std_msgs/msg/Bool' -> std_msgs.msg.Bool
    """
    if not type_str or "/msg/" not in type_str:
        raise ValueError(f"Ungültiger msg-Typ: {type_str!r}")
    pkg, rest = type_str.split("/", 1)          # std_msgs, msg/Bool
    _, cls = rest.split("/", 1)                 # msg, Bool
    mod = importlib.import_module(f"{pkg}.msg")
    return getattr(mod, cls)


class BaseRobot(Node):
    """
    Gemeinsame Basis für Robot-Status-Nodes (Sim + Real/Omron).

    - Publiziert alle Status-Topics aus topics.yaml -> group 'robot' / publish
    - Bietet _make_sub(...) zum Abonnieren der Kommandos aus group 'robot' / subscribe
    - Hält interne State-Variablen (_connected/_mode/...) und publisht sie periodisch
      (damit UI nach Connect sofort Daten sieht, auch wenn nichts 'changed' triggert).
    """

    GROUP = "robot"

    def __init__(self, node_name: str = "robot") -> None:
        super().__init__(node_name)

        self.loader = topics()

        # -----------------------------
        # interne Robot-State-Variablen
        # -----------------------------
        self._connected: bool = False
        self._mode: str = "DISCONNECTED"
        self._initialized: bool = False
        self._moving: bool = False
        self._servo_enabled: bool = False
        self._power: bool = False
        self._estop: bool = False
        self._last_error: str = ""

        self._tcp_pose: PoseStamped = PoseStamped()
        self._joints: JointState = JointState()

        # -----------------------------
        # Publisher aus topics.yaml
        # -----------------------------
        self._pubs: Dict[str, Any] = {}

        for spec in self.loader.publish_specs(self.GROUP):
            MsgT = _resolve_msg_type(spec.type)
            qos = self.loader.qos_by_id("publish", self.GROUP, spec.id)
            pub = self.create_publisher(MsgT, spec.name, qos)
            self._pubs[spec.id] = pub
            # als Attribut (z.B. self.pub_errors)
            setattr(self, f"pub_{spec.id}", pub)

        # Timer: Status regelmäßig publishen (UI kann jederzeit subscriben)
        self._state_timer = self.create_timer(0.2, self._publish_state_once)

        # initial publish
        self._publish_state_once()

    # -----------------------------
    # Subscribe helper (Kommandos)
    # -----------------------------
    def _make_sub(self, msg_type: Type[Any], topic_id: str, cb: Callable[[Any], None]):
        topic = self.loader.subscribe_topic(self.GROUP, topic_id)
        qos = self.loader.qos_by_id("subscribe", self.GROUP, topic_id)
        return self.create_subscription(msg_type, topic, cb, qos)

    # -----------------------------
    # State setter helpers
    # -----------------------------
    def _set_mode(self, mode: str) -> None:
        mode = (mode or "").strip() or "UNKNOWN"
        self._mode = mode
        if "pub_mode" in dir(self):
            self.pub_mode.publish(String(data=mode))

    def _set_connection(self, connected: bool) -> None:
        self._connected = bool(connected)
        if "pub_connection" in dir(self):
            self.pub_connection.publish(Bool(data=self._connected))

    def _set_error(self, text: str) -> None:
        self._last_error = (text or "").strip()
        if "pub_errors" in dir(self):
            self.pub_errors.publish(String(data=self._last_error))

    # -----------------------------
    # Publish all current state
    # -----------------------------
    def _publish_state_once(self) -> None:
        # bool/status topics
        if "pub_connection" in dir(self):
            self.pub_connection.publish(Bool(data=self._connected))
        if "pub_mode" in dir(self):
            self.pub_mode.publish(String(data=self._mode))
        if "pub_initialized" in dir(self):
            self.pub_initialized.publish(Bool(data=self._initialized))
        if "pub_moving" in dir(self):
            self.pub_moving.publish(Bool(data=self._moving))
        if "pub_servo_enabled" in dir(self):
            self.pub_servo_enabled.publish(Bool(data=self._servo_enabled))
        if "pub_power" in dir(self):
            self.pub_power.publish(Bool(data=self._power))
        if "pub_estop" in dir(self):
            self.pub_estop.publish(Bool(data=self._estop))
        if "pub_errors" in dir(self):
            self.pub_errors.publish(String(data=self._last_error))

        # pose/joints (nur wenn gesetzt)
        if "pub_tcp_pose" in dir(self) and self._tcp_pose is not None:
            self.pub_tcp_pose.publish(self._tcp_pose)
        if "pub_joints" in dir(self) and self._joints is not None:
            self.pub_joints.publish(self._joints)

    # -----------------------------
    # Convenience update for derived classes
    # -----------------------------
    def update_tcp_pose(self, msg: PoseStamped) -> None:
        self._tcp_pose = msg
        if "pub_tcp_pose" in dir(self):
            self.pub_tcp_pose.publish(msg)

    def update_joints(self, msg: JointState) -> None:
        self._joints = msg
        if "pub_joints" in dir(self):
            self.pub_joints.publish(msg)
