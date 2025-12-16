#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot/omron.py

from __future__ import annotations

import math
from typing import List

import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

from .base import BaseRobot
from spraycoater_nodes_py.utils.joint_mapping import omron_to_ros_positions
from .omron_interpreter import OmronInterpreter


class OmronRobot(BaseRobot):
    """
    Robot-Node für den echten Omron-Viper (event-driven):

    - Liest Textstatus vom OmronTcpBridge als raw_rx.
    - Parsen via OmronInterpreter.
    - Mappt Omron-Reihenfolge (J1..J6) -> ROS/URDF Joint-Reihenfolge.
    - Publiziert JointStates + Zustände onChange (kein Timer/Polling).
    """

    def __init__(self) -> None:
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

        # Startzustand (einmalig publishen)
        self.set_connected(False, force=True)
        self.set_initialized(False, force=True)
        self.set_power(False, force=True)
        self.set_estop(False, force=True)
        self.set_servo_enabled(False, force=True)
        self.set_moving(False, force=True)
        self.set_mode("DISCONNECTED", force=True)
        self.set_error("", force=True, log_on_set=False)

        # raw_rx abonnieren
        topic_raw_rx = self.cfg.subscribe_topic("omron", "raw_rx")
        qos_raw_rx = self.cfg.qos_by_id("subscribe", "omron", "raw_rx")

        self._sub_raw_rx = self.create_subscription(
            String,
            topic_raw_rx,
            self._on_raw_rx,
            qos_raw_rx,
        )

        self.get_logger().info(
            f"🤖 OmronRobot gestartet – subscribe raw_rx='{topic_raw_rx}' (via config_hub, onChange)"
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

        if etype == "status_joints":
            joints = ev.get("joints", [])
            if len(joints) != 6:
                return

            joints_rad = [float(j) * math.pi / 180.0 for j in joints]

            # JointState bauen
            js = JointState()
            js.name = list(self._joint_order_ros_names)

            try:
                positions_ros = omron_to_ros_positions(
                    omron_positions=joints_rad,
                    ros_joint_names=js.name,
                    joint_order_ros_names=self._joint_order_ros_names,
                )
            except ValueError as exc:
                self.set_error(f"Joint-Mapping-Fehler: {exc}")
                return

            js.position = list(positions_ros)

            # ✅ joints publish nur bei Änderung
            self.publish_joints(js)

            # ✅ Status onChange setzen
            self.set_connected(True)
            # Achtung: echtes Power/Init/Servo besser aus echten Statusbits ableiten.
            # Für jetzt: wie vorher "READY", aber nur onChange.
            self.set_power(True)
            self.set_initialized(True)
            self.set_mode("READY")

        elif etype == "ack":
            cmd = (ev.get("cmd", "") or "").upper()
            if cmd == "PING":
                self.set_connected(True)
                if self._mode == "DISCONNECTED":
                    self.set_mode("IDLE")

        elif etype == "nack":
            cmd = ev.get("cmd", "")
            rest = ev.get("rest", "")
            self.set_error(f"NACK {cmd}: {rest}")


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
