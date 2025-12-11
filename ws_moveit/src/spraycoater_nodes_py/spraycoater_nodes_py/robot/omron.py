#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from typing import List

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from .base import BaseRobot
from .joint_mapping import omron_to_ros_positions
from .omron_interpreter import OmronInterpreter


class OmronRobot(BaseRobot):
    """
    Robot-Node für den echten Omron-Viper:

    - Liest Textstatus vom OmronTcpBridge (Topic: omron.raw_rx).
    - Nutzt OmronInterpreter zum Parsen von STATUS J j1..j6.
    - Mappt Omron-Reihenfolge (J1..J6) in URDF/ROS-Jointreihenfolge.
    - Publiziert JointStates + Zustände über BaseRobot.
    """

    def __init__(self) -> None:
        super().__init__(node_name="omron_robot")

        # Interpreter für Textzeilen
        self._interp = OmronInterpreter()

        # ROS-Jointnamen in Omron-Reihenfolge (joint_order_ros_names)
        # → Diese Namen müssen den URDF/MoveIt-Jointnamen entsprechen!
        self._joint_order_ros_names: List[str] = [
            "Adept_Viper_s650_Link1",  # J1
            "Adept_Viper_s650_Link2",  # J2
            "Adept_Viper_s650_Link3",  # J3
            "Adept_Viper_s650_Link4",  # J4
            "Adept_Viper_s650_Link5",  # J5
            "Adept_Viper_s650_Link6",  # J6
        ]

        # Zielreihenfolge für JointState (Standard: URDF-Reihenfolge)
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

        # Topic-Namen + QoS für raw_rx aus der Omron-Section
        topic_raw_rx = self.loader.publish_topic("omron", "raw_rx")
        qos_raw_rx = self.loader.qos_by_id("publish", "omron", "raw_rx")

        self._sub_raw_rx = self.create_subscription(
            String,
            topic_raw_rx,
            self._on_raw_rx,
            qos_raw_rx,
        )

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

        # STATUS J j1..j6
        if etype == "status_joints":
            joints = ev.get("joints", [])
            if len(joints) != 6:
                return

            # Falls der Controller in Grad liefert → in rad umwandeln
            # Wenn du schon in rad sendest, diesen Block auskommentieren.
            joints_rad = [j * math.pi / 180.0 for j in joints]

            try:
                positions_ros = omron_to_ros_positions(
                    self._joint_order_ros_names,
                    self._joints.name,
                    joints_rad,
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
            # ACK PING, ACK TRAJSTART, ...
            cmd = ev.get("cmd", "")
            if cmd.upper() == "PING":
                self._connected = True
                if self._mode == "DISCONNECTED":
                    self._mode = "IDLE"

        elif etype == "nack":
            cmd = ev.get("cmd", "")
            rest = ev.get("rest", "")
            self._set_error(f"NACK {cmd}: {rest}")

        # andere Types (other/status_unknown) ignorieren wir erst mal

    # Optional könntest du hier _update_tcp_pose überschreiben, wenn du
    # später eine TCP-Pose aus STATUS T ... ableitest.
    # def _update_tcp_pose(self):
    #     ...


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
