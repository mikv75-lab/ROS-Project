# src/ros/bridge/servo_bridge.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Dict, Any

import math
import threading

from PyQt6 import QtCore

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType  # Service für CommandType

from config.startup import AppContent
from .base_bridge import BaseBridge


class ServoSignals(QtCore.QObject):
    paramsChanged = QtCore.pyqtSignal(dict)
    jointJogRequested = QtCore.pyqtSignal(str, float, float)
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)
    frameChanged = QtCore.pyqtSignal(str)


class ServoBridge(BaseBridge):
    """
    UI <-> ServoBridge <-> moveit_servo

    Publisht:
      - /servo/joint_jog    (JointJog)
      - /servo/cartesian_mm (TwistStamped)

    Schaltet CommandType via Service:
      - omron_servo_node/switch_command_type  (namespaced)
    """

    GROUP = "servo"

    _COMMAND_TYPE_MAP = {
        "joint": 0,        # JOINT_JOG
        "joint_jog": 0,
        "jog": 0,
        "joints": 0,
        "twist": 1,        # TWIST
        "cart": 1,
        "cartesian": 1,
        "tcp": 1,
        "pose": 2,         # POSE
    }

    def __init__(self, content: AppContent, namespace: str = ""):
        self.signals = ServoSignals()

        # Frames aus AppContent
        frames_cfg = getattr(content, "frames", None)
        if frames_cfg is not None:
            try:
                self.world_frame = frames_cfg.resolve(frames_cfg.get("world", "world"))
                self.tcp_frame = frames_cfg.resolve(frames_cfg.get("tool_mount", "tool_mount"))
            except Exception:
                self.world_frame = "world"
                self.tcp_frame = "tool_mount"
        else:
            self.world_frame = "world"
            self.tcp_frame = "tool_mount"

        self.current_frame: str = self.world_frame

        # CommandType-State
        self._cmd_type_client = None
        self._cmd_type_lock = threading.Lock()
        self._current_command_type: Optional[int] = None  # 0=JOINT_JOG,1=TWIST,2=POSE
        self._desired_command_type: Optional[int] = None
        self._cmd_type_inflight: bool = False

        # Node (ggf. im Namespace)
        super().__init__("servo_bridge", content, namespace=namespace)

        # -----------------------------
        # Publisher
        # -----------------------------
        try:
            spec_cart = self.spec("publish", "cartesian_mm")
            MsgCart = spec_cart.resolve_type()
            qos_cart = getattr(spec_cart, "qos_profile", None)
            self.pub_cartesian = self.create_publisher(
                MsgCart, spec_cart.name, qos_cart if qos_cart is not None else 10
            )
        except Exception as e:
            self.get_logger().error(f"[servo] cartesian_mm publisher init failed: {e}")
            self.pub_cartesian = None

        try:
            spec_joint = self.spec("publish", "joint_jog")
            MsgJoint = spec_joint.resolve_type()
            qos_joint = getattr(spec_joint, "qos_profile", None)
            self.pub_joint = self.create_publisher(
                MsgJoint, spec_joint.name, qos_joint if qos_joint is not None else 10
            )
        except Exception as e:
            self.get_logger().error(f"[servo] joint_jog publisher init failed: {e}")
            self.pub_joint = None

        # -----------------------------
        # Service-Client: CommandType
        # -----------------------------
        self._cmd_type_service_name = "omron_servo_node/switch_command_type"  # RELATIV lassen!
        try:
            self._cmd_type_client = self.create_client(ServoCommandType, self._cmd_type_service_name)
        except Exception as e:
            self.get_logger().error(f"[servo] CommandType client init failed: {e}")
            self._cmd_type_client = None

        # Timer: stiller Retry für gewünschten CommandType
        self._cmd_type_timer = self.create_timer(0.2, self._cmd_type_retry_tick)

        # -----------------------------
        # Qt-Signale ↔ Methoden
        # -----------------------------
        self.signals.jointJogRequested.connect(self._on_joint_jog_requested)
        self.signals.cartesianJogRequested.connect(self._on_cartesian_jog_requested)
        self.signals.frameChanged.connect(self._on_frame_changed)
        self.signals.paramsChanged.connect(self._on_params_changed)

        self.get_logger().info(
            f"[servo] ServoBridge aktiv, ns='{self.get_namespace() or '/'}', "
            f"world_frame='{self.world_frame}', tcp_frame='{self.tcp_frame}', current='{self.current_frame}', "
            f"cmd_type_srv='{self._cmd_type_service_name}'"
        )

    # ======================================================
    # CommandType Switching (robust, non-blocking)
    # ======================================================

    def set_command_type(self, mode: str) -> None:
        """
        Non-blocking: merkt sich gewünschten Mode und versucht im Hintergrund zu setzen.
        """
        if self._cmd_type_client is None:
            return

        key = (mode or "").strip().lower()
        cmd_val = self._COMMAND_TYPE_MAP.get(key)
        if cmd_val is None:
            return

        with self._cmd_type_lock:
            if self._current_command_type == cmd_val:
                return
            self._desired_command_type = cmd_val

        # Sofort versuchen (falls ready)
        self._try_set_command_type_once()

    def _cmd_type_retry_tick(self) -> None:
        # Timer tick: wenn Wunsch offen ist, versuchen
        with self._cmd_type_lock:
            pending = self._desired_command_type
        if pending is None:
            return
        self._try_set_command_type_once()

    def _try_set_command_type_once(self) -> None:
        if self._cmd_type_client is None:
            return
        if self._cmd_type_inflight:
            return

        with self._cmd_type_lock:
            desired = self._desired_command_type

        if desired is None:
            return

        # Service noch nicht da → später erneut (kein wait_for_service, kein block)
        if not self._cmd_type_client.service_is_ready():
            return

        req = ServoCommandType.Request()
        req.command_type = int(desired)

        self._cmd_type_inflight = True
        future = self._cmd_type_client.call_async(req)

        def _done(fut):
            self._cmd_type_inflight = False
            try:
                resp = fut.result()
            except Exception:
                return  # still retry later

            if not resp or not getattr(resp, "success", False):
                return  # still retry later

            with self._cmd_type_lock:
                self._current_command_type = int(req.command_type)
                # nur löschen, wenn es noch derselbe Wunsch war
                if self._desired_command_type == self._current_command_type:
                    self._desired_command_type = None

        future.add_done_callback(_done)

    # ======================================================
    # UI → JointJog (JOINT_JOG)
    # ======================================================

    def _on_joint_jog_requested(self, joint_name: str, delta_deg: float, speed_pct: float) -> None:
        if self.pub_joint is None:
            return
        if not joint_name:
            return

        # ✅ WICHTIG: CommandType passend setzen
        self.set_command_type("joint")

        dt = 0.5
        delta_rad = math.radians(delta_deg)
        scale = max(0.0, min(1.0, float(speed_pct) / 100.0))
        vel = (delta_rad / dt) * scale

        msg = JointJog()
        msg.header.frame_id = self.current_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [joint_name]
        msg.velocities = [vel]
        msg.displacements = []
        msg.duration = dt

        self.pub_joint.publish(msg)

    # ======================================================
    # UI → Cartesian Jog (TWIST)
    # ======================================================

    def _on_cartesian_jog_requested(
        self,
        axis: str,
        delta: float,
        speed_mm_s: float,
        frame_ui: str,
    ) -> None:
        if self.pub_cartesian is None:
            return

        axis = (axis or "").lower()
        if axis not in ("x", "y", "z", "rx", "ry", "rz"):
            return

        # Frame entsprechend UI umschalten
        self._set_frame_from_ui(frame_ui)

        # ✅ WICHTIG: CommandType passend setzen
        self.set_command_type("twist")

        msg = TwistStamped()
        msg.header.frame_id = self.current_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        speed_m_s = float(speed_mm_s) / 1000.0
        sign = 1.0 if float(delta) >= 0.0 else -1.0

        if axis in ("x", "y", "z"):
            v = sign * speed_m_s
            if axis == "x":
                msg.twist.linear.x = v
            elif axis == "y":
                msg.twist.linear.y = v
            else:
                msg.twist.linear.z = v
        else:
            dt = 0.5
            omega = math.radians(float(delta)) / dt
            if axis == "rx":
                msg.twist.angular.x = omega
            elif axis == "ry":
                msg.twist.angular.y = omega
            else:
                msg.twist.angular.z = omega

        self.pub_cartesian.publish(msg)

    # ======================================================
    # Frame / Params
    # ======================================================

    def _on_frame_changed(self, frame_ui: str) -> None:
        self._set_frame_from_ui(frame_ui)

    def _set_frame_from_ui(self, frame_ui: str) -> None:
        f = (frame_ui or "").strip().lower()
        if f == "trf":
            self.current_frame = self.tcp_frame
        else:
            self.current_frame = self.world_frame

    def _on_params_changed(self, cfg: Dict[str, Any]) -> None:
        # params aktuell nicht benutzt – ok
        pass
