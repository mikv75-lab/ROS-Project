#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot/omron.py

from __future__ import annotations

import math
from typing import List

import rclpy
from rclpy.time import Time
from std_msgs.msg import String

from .base import BaseRobot
from spraycoater_nodes_py.utils.joint_mapping import omron_to_ros_positions
from .omron_interpreter import OmronInterpreter


class OmronRobot(BaseRobot):
    """
    Robot-Node für den echten Omron-Viper:

    - Liest Textstatus vom OmronTcpBridge als raw_rx.
    - Parsen via OmronInterpreter.
    - Mappt Omron-Reihenfolge (J1..J6) -> ROS/URDF Joint-Reihenfolge.
    - Publiziert JointStates + Zustände über BaseRobot.

    topics.yaml muss enthalten:
      topics:
        omron:
          subscribe:
            - id: raw_rx
              name: spraycoater/omron/raw_rx
              type: std_msgs/msg/String
              qos: default
    """

    def __init__(self) -> None:
        # gleicher Node-Name wie SimRobot (Namespace trennt shadow/live)
        super().__init__(node_name="robot")

        self._interp = OmronInterpreter()

        # Diese Liste MUSS zu deinen URDF/MoveIt Joint-Namen passen!
        self._joint_order_ros_names: List[str] = [
            "Adept_Viper_s650_Link1",  # J1
            "Adept_Viper_s650_Link2",  # J2
            "Adept_Viper_s650_Link3",  # J3
            "Adept_Viper_s650_Link4",  # J4
            "Adept_Viper_s650_Link5",  # J5
            "Adept_Viper_s650_Link6",  # J6
        ]

        self._joints.name = list(self._joint_order_ros_names)
        self._joints.position = [0.0] * len(self._joints.name)

        # Grundzustand
        self._connected = False
        self._initialized = False
        self._power = False
        self._estop = False
        self._servo_enabled = False
        self._moving = False
        self._mode = "DISCONNECTED"
        self._last_error = ""

        # raw_rx abonnieren (aus config_hub)
        topic_raw_rx = self.cfg.subscribe_topic("omron", "raw_rx")
        qos_raw_rx = self.cfg.qos_by_id("subscribe", "omron", "raw_rx")

        self._sub_raw_rx = self.create_subscription(
            String,
            topic_raw_rx,
            self._on_raw_rx,
            qos_raw_rx,
        )

        self.get_logger().info(
            f"🤖 OmronRobot gestartet – subscribe raw_rx='{topic_raw_rx}' (via config_hub)"
        )

    # ----------------------------------------------------------
    # Hook (BaseRobot)
    # ----------------------------------------------------------

    def on_tick(self, now: Time) -> None:
        # Real-Robot wird eventbasiert über raw_rx aktualisiert.
        # Hier nichts tun (aber Hook muss existieren).
        return

    # ------------------------------------------------------------------
    # RX-Handler (Text vom OmronTcpBridge)
    # ------------------------------------------------------------------
    def _on_raw_rx(self, msg: String) -> None:
        line = (msg.data or "").strip()
        if not line:
            return

        ev = self._interp.parse_line(line)
        if not ev:
            return

        etype = ev.get("type")

        if etype == "status_joints":
            joints = ev.get("joints", [])
            if len(joints) != 6:
                return

            joints_rad = [float(j) * math.pi / 180.0 for j in joints]

            try:
                positions_ros = omron_to_ros_positions(
                    omron_positions=joints_rad,
                    ros_joint_names=self._joints.name,
                    joint_order_ros_names=self._joint_order_ros_names,
                )
            except ValueError as exc:
                self._set_error(f"Joint-Mapping-Fehler: {exc}")
                return

            self._joints.position = positions_ros

            self._connected = True
            self._power = True
            self._initialized = True
            self._mode = "READY"

        elif etype == "ack":
            cmd = (ev.get("cmd", "") or "").upper()
            if cmd == "PING":
                self._connected = True
                if self._mode == "DISCONNECTED":
                    self._mode = "IDLE"

        elif etype == "nack":
            cmd = ev.get("cmd", "")
            rest = ev.get("rest", "")
            self._set_error(f"NACK {cmd}: {rest}")


def main(args=None):
    rclpy.init(args=args)
    node = OmronRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
