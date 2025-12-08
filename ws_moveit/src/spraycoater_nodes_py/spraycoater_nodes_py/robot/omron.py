#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/robot/omron.py
#
# REAL-Omron-Roboter:
#   - sendet periodisch "STATUS" über /spraycoater/omron/command
#   - hört /spraycoater/omron/raw_rx (vom OmronTcpBridge)
#   - interpretiert STATUS-Antworten und published:
#       /joint_states                  (für MoveIt/RViz)
#       /spraycoater/robot/joint_states
#       /spraycoater/robot/tcp_pose
#       /spraycoater/robot/* (connection, mode, initialized, moving, servo,
#                             power, estop, errors)
#   - hört JointJog von /servo/delta_joint_cmds und macht daraus JOGJ
#     (Speed-% aus joint_limits.yaml, rad/s → %)
#   - hört Speed-/Setup-Topics und schickt passende V+-Kommandos
#   - hört Robot-Command-Topics (Initialize, Stop, Power ON/OFF,
#     Servo ENABLE/DISABLE, Clear Error) und schickt passende Kommandos.
#
# Wichtig:
#   OmronTcpBridge muss laufen und mit ACE/V+ verbunden sein.

from __future__ import annotations

import math
import os
from typing import List, Dict

import rclpy
from rclpy.node import Node

from std_msgs.msg import (
    String as MsgString,
    Bool as MsgBool,
    Empty as MsgEmpty,
    Float64 as MsgFloat64,
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from control_msgs.msg import JointJog

from ament_index_python.packages import get_package_share_directory
import yaml

from spraycoater_nodes_py.robot.omron_interpreter import OmronInterpreter
from spraycoater_nodes_py.utils.config_hub import topics


class OmronRobot(Node):
    """
    ROS-Node für den *echten* Omron-Roboter.

    Architektur (Realbetrieb):

      Publisher:
        - /spraycoater/omron/command        (std_msgs/String)       → OmronTcpBridge
        - /joint_states                     (sensor_msgs/JointState)→ MoveIt/RViz
        - /spraycoater/robot/joint_states   (sensor_msgs/JointState)
        - /spraycoater/robot/tcp_pose       (geometry_msgs/PoseStamped)
        - /spraycoater/robot/connection     (std_msgs/Bool)
        - /spraycoater/robot/mode           (std_msgs/String)
        - /spraycoater/robot/initialized    (std_msgs/Bool)
        - /spraycoater/robot/moving         (std_msgs/Bool)
        - /spraycoater/robot/servo_enabled  (std_msgs/Bool)
        - /spraycoater/robot/power          (std_msgs/Bool)
        - /spraycoater/robot/estop          (std_msgs/Bool)
        - /spraycoater/robot/errors         (std_msgs/String)

      Subscriber:
        - /spraycoater/omron/raw_rx         (std_msgs/String)       ← OmronTcpBridge
        - /spraycoater/omron/connection_status (std_msgs/Bool)      ← OmronTcpBridge
        - /servo/delta_joint_cmds           (control_msgs/JointJog) ← Service-UI (Joint Jog)
        - /spraycoater/omron/set_*          (std_msgs/Float64)      ← Service-UI (Speed/Setup)
        - /spraycoater/robot/init, stop, clear_error,
          power_on/off, servo_enable/disable (std_msgs/Empty)       ← Service-UI Commands

      Timer:
        - alle 0.1 s: sendet "STATUS" an den TCP-Server
    """

    def __init__(self) -> None:
        super().__init__("omron_robot")

        self._topics = topics()
        self._interp = OmronInterpreter()

        # ------------------------------------------------------
        # Joint-Limits aus omron_moveit_config/config/joint_limits.yaml
        # → für rad/s → % bei JOGJ
        # ------------------------------------------------------
        self._joint_vmax_by_index: Dict[int, float] = {}
        self._init_joint_limits_from_yaml()

        # ------------------------------------------------------
        # Publisher: Commands an OmronTcpBridge
        # ------------------------------------------------------
        try:
            cmd_topic = self._topics.publish_topic("omron", "command")
            cmd_qos = self._topics.qos_by_id("publish", "omron", "command")
        except Exception:
            cmd_topic = "/spraycoater/omron/command"
            cmd_qos = 10

        self._pub_cmd = self.create_publisher(MsgString, cmd_topic, cmd_qos)
        self.get_logger().info(f"📤 OmronRobot sendet Commands an: {cmd_topic}")

        # ------------------------------------------------------
        # Subscriber: raw_rx vom OmronTcpBridge
        # ------------------------------------------------------
        raw_rx_topic = self._topics.subscribe_topic("omron", "raw_rx")
        raw_rx_qos = self._topics.qos_by_id("subscribe", "omron", "raw_rx")

        self.create_subscription(
            MsgString,
            raw_rx_topic,
            self._on_raw_rx,
            raw_rx_qos,
        )
        self.get_logger().info(f"📥 OmronRobot empfängt Antworten von: {raw_rx_topic}")

        # ------------------------------------------------------
        # Verbindungstatus vom TcpBridge → robot.connection
        # ------------------------------------------------------
        try:
            conn_topic = self._topics.subscribe_topic("omron", "connectionStatus")
            conn_qos = self._topics.qos_by_id("subscribe", "omron", "connectionStatus")
        except Exception:
            conn_topic = "/spraycoater/omron/connection_status"
            conn_qos = 10

        self.create_subscription(
            MsgBool,
            conn_topic,
            self._on_omron_connection,
            conn_qos,
        )

        self._pub_robot_connection = self._make_robot_pub("connection", MsgBool)
        self._robot_connected = False

        # ------------------------------------------------------
        # Robot-Status Publisher (für ServiceTab)
        # ------------------------------------------------------
        self._pub_mode = self._make_robot_pub("mode", MsgString)
        self._pub_initialized = self._make_robot_pub("initialized", MsgBool)
        self._pub_moving = self._make_robot_pub("moving", MsgBool)
        self._pub_servo_enabled = self._make_robot_pub("servo_enabled", MsgBool)
        self._pub_power = self._make_robot_pub("power", MsgBool)
        self._pub_estop = self._make_robot_pub("estop", MsgBool)
        self._pub_errors = self._make_robot_pub("errors", MsgString)
        self._pub_tcp_pose = self._make_robot_pub("tcp_pose", PoseStamped)
        self._pub_robot_joints = self._make_robot_pub("joints", JointState)

        # interne Statusflags (werden von STATUS-Parser überschrieben)
        self._state_initialized = False
        self._state_moving = False
        self._state_servo = False
        self._state_power = False
        self._state_estop = False
        self._state_mode = "OFFLINE"
        self._state_errors = ""

        # ------------------------------------------------------
        # JointState-Publisher (für MoveIt/RViz)
        # ------------------------------------------------------
        self._pub_js_global = self.create_publisher(JointState, "/joint_states", 10)

        # Joint-Namen müssen exakt zu deinem URDF/MoveIt passen:
        self._joint_names: List[str] = [
            "Adept_Viper_s650_Link2",
            "Adept_Viper_s650_Link3",
            "Adept_Viper_s650_Link4",
            "Adept_Viper_s650_Link5",
            "Adept_Viper_s650_Link6",
            "Adept_Viper_650_Interface_Plate",
        ]

        self._last_joint_state = JointState()
        self._last_joint_state.name = list(self._joint_names)

        # ------------------------------------------------------
        # JointJog-Subscriber: Service-UI → OmronRobot
        # ------------------------------------------------------
        try:
            joint_cmd_topic = self._topics.subscribe_topic("servo", "joint_out")
            joint_cmd_qos = self._topics.qos_by_id("subscribe", "servo", "joint_out")
            self.create_subscription(
                JointJog,
                joint_cmd_topic,
                self._on_joint_jog,
                joint_cmd_qos,
            )
            self.get_logger().info(
                f"🎮 OmronRobot empfängt JointJog von: {joint_cmd_topic}"
            )
        except Exception as e:
            self.get_logger().warn(
                f"⚠️ Konnte JointJog-Subscription nicht anlegen: {e}"
            )

        # ------------------------------------------------------
        # Setup-Parameter (Speed, Accel, Override, JogSpeed ...)
        # ------------------------------------------------------
        try:
            t = self._topics

            def sub_float(id_: str, cb):
                topic = t.subscribe_topic("omron", id_)
                qos = t.qos_by_id("subscribe", "omron", id_)
                self.create_subscription(MsgFloat64, topic, cb, qos)
                self.get_logger().info(f"⚙️ OmronRobot hört {id_} auf: {topic}")

            sub_float("set_speed", self._on_set_speed)
            sub_float("set_accel", self._on_set_accel)
            sub_float("set_decel", self._on_set_decel)
            sub_float("set_override", self._on_set_override)
            sub_float("set_jog_speed", self._on_set_jog_speed)
            sub_float("set_jog_accel", self._on_set_jog_accel)
            sub_float("set_jog_decel", self._on_set_jog_decel)

        except Exception as e:
            self.get_logger().error(f"Setup-Subscriptions fehlgeschlagen: {e}")

        # ------------------------------------------------------
        # Robot-Commandos (Initialize, Stop, Clear Error, Power, Servo)
        # ------------------------------------------------------
        try:
            t = self._topics

            def sub_empty(id_: str, cb):
                topic = t.subscribe_topic("robot", id_)
                qos = t.qos_by_id("subscribe", "robot", id_)
                self.create_subscription(MsgEmpty, topic, cb, qos)
                self.get_logger().info(f"🕹️ OmronRobot hört Command {id_} auf: {topic}")

            sub_empty("init", self._on_cmd_init)
            sub_empty("stop", self._on_cmd_stop)
            sub_empty("clear_error", self._on_cmd_clear_error)
            sub_empty("power_on", self._on_cmd_power_on)
            sub_empty("power_off", self._on_cmd_power_off)
            sub_empty("servo_on", self._on_cmd_servo_on)
            sub_empty("servo_off", self._on_cmd_servo_off)

        except Exception as e:
            self.get_logger().error(f"Robot-Command-Subscriptions fehlgeschlagen: {e}")

        # ------------------------------------------------------
        # Timer: STATUS-Poll
        # ------------------------------------------------------
        self._status_period = 0.1
        self._status_timer = self.create_timer(self._status_period, self._tick_status)

        self.get_logger().info(
            "✅ OmronRobot initialisiert – STATUS, JointJog, Setup & Commands aktiv."
        )

    # ------------------------------------------------------------------
    # Hilfsfunktionen
    # ------------------------------------------------------------------
    def _make_robot_pub(self, id_: str, msg_type):
        try:
            topic = self._topics.publish_topic("robot", id_)
            qos = self._topics.qos_by_id("publish", "robot", id_)
            return self.create_publisher(msg_type, topic, qos)
        except Exception:
            default_name = f"/spraycoater/robot/{id_}"
            self.get_logger().warn(
                f"[OmronRobot] Kein publish-Eintrag für robot.{id_} – Fallback {default_name}"
            )
            return self.create_publisher(msg_type, default_name, 10)

    def _send_cmd(self, cmd: str):
        """Hilfsfunktion: sendet beliebiges V+-Command über command-topic."""
        msg = MsgString()
        msg.data = cmd
        self._pub_cmd.publish(msg)
        self.get_logger().info(f"➡️ CMD: {cmd}")

    # ------------------------------------------------------------------
    # Joint-Limits laden
    # ------------------------------------------------------------------
    def _init_joint_limits_from_yaml(self) -> None:
        """
        Lädt omron_moveit_config/config/joint_limits.yaml und füllt
        self._joint_vmax_by_index: {1: vmax1, ..., 6: vmax6} in rad/s.
        """
        try:
            cfg_pkg = get_package_share_directory("omron_moveit_config")
        except Exception as e:
            self.get_logger().warn(
                f"[OmronRobot] joint_limits.yaml: Paket 'omron_moveit_config' nicht gefunden: {e}"
            )
            return

        path = os.path.join(cfg_pkg, "config", "joint_limits.yaml")
        if not os.path.isfile(path):
            self.get_logger().warn(
                f"[OmronRobot] joint_limits.yaml nicht gefunden: {path}"
            )
            return

        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f)
        except Exception as e:
            self.get_logger().warn(
                f"[OmronRobot] joint_limits.yaml konnte nicht gelesen werden: {e}"
            )
            return

        jl = data.get("joint_limits", {})
        if not isinstance(jl, dict):
            self.get_logger().warn(
                f"[OmronRobot] joint_limits.yaml: 'joint_limits' fehlt oder ist kein Dict."
            )
            return

        for name, cfg in jl.items():
            if not isinstance(cfg, dict):
                continue
            if not cfg.get("has_velocity_limits", False):
                continue

            vmax = cfg.get("max_velocity", None)
            if vmax is None or vmax <= 0.0:
                continue

            if name.startswith("joint_"):
                try:
                    idx = int(name.split("_", 1)[1])
                except Exception:
                    continue
                self._joint_vmax_by_index[idx] = float(vmax)

        if not self._joint_vmax_by_index:
            self.get_logger().warn(
                f"[OmronRobot] joint_limits.yaml: keine gültigen max_velocity-Werte gefunden."
            )
        else:
            self.get_logger().info(
                f"[OmronRobot] Joint-Limits (rad/s) pro Index: {self._joint_vmax_by_index}"
            )

    # ------------------------------------------------------------------
    # Verbindungstatus vom TcpBridge
    # ------------------------------------------------------------------
    def _on_omron_connection(self, msg: MsgBool) -> None:
        self._robot_connected = bool(msg.data)
        out = MsgBool()
        out.data = self._robot_connected
        self._pub_robot_connection.publish(out)

        self._state_mode = "ONLINE" if self._robot_connected else "OFFLINE"
        self._publish_mode()

    # ------------------------------------------------------------------
    # Periodisches STATUS-Polling
    # ------------------------------------------------------------------
    def _tick_status(self) -> None:
        msg = MsgString()
        msg.data = "STATUS"
        self._pub_cmd.publish(msg)

    # ------------------------------------------------------------------
    # raw_rx → Interpreter → Status
    # ------------------------------------------------------------------
    def _on_raw_rx(self, msg: MsgString) -> None:
        line = (msg.data or "").strip()
        if not line:
            return

        parsed = self._interp.parse_line(line)
        if not parsed:
            return

        ptype = parsed.get("type")

        if ptype == "status_joints":
            joints_deg = parsed.get("joints", [])
            if len(joints_deg) != 6:
                self.get_logger().warn(f"STATUS J hat unerwartete Länge: {joints_deg}")
                return

            joints_rad = [math.radians(v) for v in joints_deg]

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(self._joint_names)
            js.position = joints_rad

            self._last_joint_state = js
            self._pub_js_global.publish(js)
            self._pub_robot_joints.publish(js)

        elif ptype == "status_pose":
            xyz = parsed.get("xyz", None)
            rxyz = parsed.get("rxyz", None)
            if xyz and len(xyz) == 3:
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = "world"
                pose.pose.position.x = xyz[0]
                pose.pose.position.y = xyz[1]
                pose.pose.position.z = xyz[2]
                # Orientation kannst du später aus rxyz bauen; hier identity.
                self._pub_tcp_pose.publish(pose)

        elif ptype == "status_flags":
            self._state_initialized = bool(parsed.get("initialized", self._state_initialized))
            self._state_moving = bool(parsed.get("moving", self._state_moving))
            self._state_servo = bool(parsed.get("servo", self._state_servo))
            self._state_power = bool(parsed.get("power", self._state_power))
            self._state_estop = bool(parsed.get("estop", self._state_estop))
            mode = parsed.get("mode", None)
            if isinstance(mode, str) and mode:
                self._state_mode = mode
            self._publish_status_flags()

        elif ptype == "error":
            txt = parsed.get("text", parsed.get("raw", line))
            self._state_errors = str(txt)
            self._publish_errors()

        elif ptype in ("ack", "nack"):
            cmd = parsed.get("cmd", "")
            if ptype == "nack":
                self._state_errors = f"NACK {cmd} {parsed.get('rest', '')}"
                self._publish_errors()

        else:
            self.get_logger().debug(f"OmronRobot OTHER: {parsed.get('raw', line)}")

    # ------------------------------------------------------------------
    # Status-Publisher
    # ------------------------------------------------------------------
    def _publish_mode(self):
        msg = MsgString()
        msg.data = self._state_mode
        self._pub_mode.publish(msg)

    def _publish_status_flags(self):
        b = MsgBool()

        b.data = self._state_initialized
        self._pub_initialized.publish(b)

        b = MsgBool()
        b.data = self._state_moving
        self._pub_moving.publish(b)

        b = MsgBool()
        b.data = self._state_servo
        self._pub_servo_enabled.publish(b)

        b = MsgBool()
        b.data = self._state_power
        self._pub_power.publish(b)

        b = MsgBool()
        b.data = self._state_estop
        self._pub_estop.publish(b)

        self._publish_mode()

    def _publish_errors(self):
        msg = MsgString()
        msg.data = self._state_errors
        self._pub_errors.publish(msg)

    # ------------------------------------------------------------------
    # JointJog → JOGJ
    # ------------------------------------------------------------------
    def _on_joint_jog(self, msg: JointJog) -> None:
        if not msg.joint_names or not msg.velocities:
            return

        name = msg.joint_names[0]
        vel_rad_s = msg.velocities[0]

        joint_idx = self._joint_index_from_name(name)
        if joint_idx is None:
            self.get_logger().warning(
                f"[OmronRobot] JointJog: Unbekannter Joint-Name '{name}' – ignoriere."
            )
            return

        vmax_rad_s = self._joint_vmax_by_index.get(joint_idx, None)
        if vmax_rad_s is None or vmax_rad_s <= 0.0:
            vmax_rad_s = 1.0
            self.get_logger().debug(
                f"[OmronRobot] Kein vmax für Joint {joint_idx} – Fallback 1.0 rad/s."
            )

        if abs(vel_rad_s) < 1e-6:
            speed_percent = 0.0
        else:
            speed_percent = (vel_rad_s / vmax_rad_s) * 100.0

        if speed_percent > 100.0:
            speed_percent = 100.0
        elif speed_percent < -100.0:
            speed_percent = -100.0

        cmd = f"JOGJ {joint_idx} {speed_percent:.2f}"
        self._send_cmd(cmd)

    def _joint_index_from_name(self, name: str) -> int | None:
        name = name or ""

        if name.startswith("joint_"):
            try:
                return int(name.split("_", 1)[1])
            except Exception:
                pass

        mapping = {
            "Adept_Viper_s650_Link2": 1,
            "Adept_Viper_s650_Link3": 2,
            "Adept_Viper_s650_Link4": 3,
            "Adept_Viper_s650_Link5": 4,
            "Adept_Viper_s650_Link6": 5,
            "Adept_Viper_650_Interface_Plate": 6,
        }
        return mapping.get(name, None)

    # ------------------------------------------------------------------
    # Setup Commands (Speed, Accel, Override, JogSpeed, ...)
    # ------------------------------------------------------------------
    def _on_set_speed(self, msg: MsgFloat64):
        self._send_cmd(f"SPEED {msg.data:.2f}")

    def _on_set_accel(self, msg: MsgFloat64):
        self._send_cmd(f"ACCEL {msg.data:.2f}")

    def _on_set_decel(self, msg: MsgFloat64):
        self._send_cmd(f"DECEL {msg.data:.2f}")

    def _on_set_override(self, msg: MsgFloat64):
        self._send_cmd(f"OVERRIDE {msg.data:.2f}")

    def _on_set_jog_speed(self, msg: MsgFloat64):
        self._send_cmd(f"JOGSPEED {msg.data:.2f}")

    def _on_set_jog_accel(self, msg: MsgFloat64):
        self._send_cmd(f"JOGACCEL {msg.data:.2f}")

    def _on_set_jog_decel(self, msg: MsgFloat64):
        self._send_cmd(f"JOGDECEL {msg.data:.2f}")

    # ------------------------------------------------------------------
    # Robot-Commands (Init, Stop, Power, Servo, Clear Error)
    # ------------------------------------------------------------------
    def _on_cmd_init(self, _: MsgEmpty):
        # genaue Strings kannst du in deinem V+-Server anpassen
        self._send_cmd("INIT")

    def _on_cmd_stop(self, _: MsgEmpty):
        self._send_cmd("STOP")

    def _on_cmd_clear_error(self, _: MsgEmpty):
        self._send_cmd("CLEARERR")

    def _on_cmd_power_on(self, _: MsgEmpty):
        self._send_cmd("POWER ON")

    def _on_cmd_power_off(self, _: MsgEmpty):
        self._send_cmd("POWER OFF")

    def _on_cmd_servo_on(self, _: MsgEmpty):
        self._send_cmd("SERVO ON")

    def _on_cmd_servo_off(self, _: MsgEmpty):
        self._send_cmd("SERVO OFF")


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
