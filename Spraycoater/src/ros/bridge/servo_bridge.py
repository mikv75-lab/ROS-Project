#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Dict, Any

import math

from PyQt6 import QtCore

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

from config.startup import AppContent
from .base_bridge import BaseBridge


class ServoSignals(QtCore.QObject):
    """
    Qt-Signale für Servo-Jogging.

    Werden von den Widgets in tabs/service/servo_widgets.py erwartet:

      - paramsChanged(dict)
      - jointJogRequested(str, float, float)        # joint_name, delta_deg, speed_pct
      - cartesianJogRequested(str, float, float, str)
            axis: 'x','y','z','rx','ry','rz'
            delta: mm (linear) oder deg (rot.)
            speed: mm/s (für linear; bei rot nur mitgegeben)
            frame: 'wrf' / 'trf'
      - frameChanged(str)                           # 'wrf' / 'trf'
    """
    paramsChanged = QtCore.pyqtSignal(dict)
    jointJogRequested = QtCore.pyqtSignal(str, float, float)
    cartesianJogRequested = QtCore.pyqtSignal(str, float, float, str)
    frameChanged = QtCore.pyqtSignal(str)


class ServoBridge(BaseBridge):
    """
    Bridge für Servo-Jogging:

      UI <-> ServoBridge <-> moveit_servo

    - UI-Seite:
        * angebunden über ServoSignals (s.o.)
        * die Widgets rufen über ihre Signals diese Bridge-Callbacks auf

    - ROS-Seite:
        * publisht direkt auf:
            - /servo/joint_jog      (JointJog)
            - /servo/cartesian_mm   (TwistStamped)

    Die Topic-Namen werden über AppContent / topics.yaml geholt.
    """

    GROUP = "servo"

    def __init__(self, content: AppContent):
        # Qt-Signale
        self.signals = ServoSignals()

        # Frame-Konfiguration aus AppContent.frames (falls vorhanden)
        frames_cfg = getattr(content, "frames", None)
        if frames_cfg is not None:
            try:
                # frames.yaml z.B.: world: base_link, tcp: tool0, ...
                self.world_frame = frames_cfg.resolve(frames_cfg.get("world", "world"))
                self.tcp_frame = frames_cfg.resolve(frames_cfg.get("tcp", "tcp"))
            except Exception:
                self.world_frame = "world"
                self.tcp_frame = "tcp"
        else:
            self.world_frame = "world"
            self.tcp_frame = "tcp"

        self.current_frame: str = self.world_frame

        super().__init__("servo_bridge", content)

        # -----------------------------
        # Publisher (→ moveit_servo)
        # -----------------------------
        # /servo/cartesian_mm
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

        # /servo/joint_jog
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
        # Qt-Signale ↔ Methoden
        # -----------------------------
        self.signals.jointJogRequested.connect(self._on_joint_jog_requested)
        self.signals.cartesianJogRequested.connect(self._on_cartesian_jog_requested)
        self.signals.frameChanged.connect(self._on_frame_changed)
        self.signals.paramsChanged.connect(self._on_params_changed)

        self.get_logger().info(
            f"[servo] ServoBridge aktiv, world_frame='{self.world_frame}', tcp_frame='{self.tcp_frame}', current='{self.current_frame}'"
        )

    # ======================================================
    # UI → JointJog (JOINT_JOG)
    # ======================================================

    def _on_joint_jog_requested(self, joint_name: str, delta_deg: float, speed_pct: float) -> None:
        """
        Wird vom JointJogWidget ausgelöst.

        Interpretation:
          - delta_deg: Schrittweite in Grad (±)
          - speed_pct: 0..100 (Skalierung einer Basisgeschwindigkeit)

        Aktuelle, einfache Abbildung:
          - dt = 0.5 s, delta_rad = deg→rad
          - v = (delta_rad / dt) * (speed_pct / 100)

        → Das ist bewusst simpel, kannst du später mit echten Limits verfeinern.
        """
        if self.pub_joint is None:
            self.get_logger().error("[servo] joint_jog publish failed: publisher not initialized")
            return

        if not joint_name:
            self.get_logger().warning("[servo] joint_jog: empty joint_name")
            return

        dt = 0.5
        delta_rad = math.radians(delta_deg)
        scale = max(0.0, min(1.0, speed_pct / 100.0))
        vel = (delta_rad / dt) * scale

        msg = JointJog()
        msg.header.frame_id = self.current_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [joint_name]
        msg.velocities = [vel]
        msg.displacements = []   # bei command_in_type = speed_units unbenutzt
        msg.duration = dt

        self.pub_joint.publish(msg)
        self.get_logger().debug(
            f"[servo] JointJog: joint={joint_name}, delta_deg={delta_deg:.2f}, "
            f"speed_pct={speed_pct:.1f}, vel_rad_s={vel:.4f}, dt={dt:.2f}, frame={self.current_frame}"
        )

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
        """
        Wird vom CartesianJogWidget ausgelöst.

        Interpretation:
          - axis: 'x','y','z'  -> linear (mm)
          - axis: 'rx','ry','rz' -> rot. (deg)
          - delta: Schritt (mm bzw. Grad) – Vorzeichen gibt Richtung an
          - speed_mm_s: Translationsgeschwindigkeit (mm/s) für Linearachsen
          - frame_ui: 'wrf' / 'trf' (Widget-Sicht)

        Aktuelle Abbildung:
          - Linear: Twist.linear = ± speed_mm_s (in m/s)
          - Rot:    Twist.angular = delta_rad / dt   (dt=0.5 s)
        """
        if self.pub_cartesian is None:
            self.get_logger().error("[servo] cartesian_mm publish failed: publisher not initialized")
            return

        axis = (axis or "").lower()
        if axis not in ("x", "y", "z", "rx", "ry", "rz"):
            self.get_logger().warning(f"[servo] cartesian_jog: unknown axis '{axis}'")
            return

        # Frame entsprechend UI umschalten
        self._set_frame_from_ui(frame_ui)

        msg = TwistStamped()
        msg.header.frame_id = self.current_frame
        msg.header.stamp = self.get_clock().now().to_msg()

        # Linear: mm/s → m/s
        speed_m_s = float(speed_mm_s) / 1000.0
        sign = 1.0 if delta >= 0.0 else -1.0

        if axis in ("x", "y", "z"):
            v = sign * speed_m_s
            if axis == "x":
                msg.twist.linear.x = v
            elif axis == "y":
                msg.twist.linear.y = v
            else:
                msg.twist.linear.z = v
        else:
            # Rotationsgeschwindigkeit grob aus delta (deg) abgeleitet
            dt = 0.5
            delta_rad = math.radians(delta)
            omega = delta_rad / dt
            if axis == "rx":
                msg.twist.angular.x = omega
            elif axis == "ry":
                msg.twist.angular.y = omega
            else:
                msg.twist.angular.z = omega

        self.pub_cartesian.publish(msg)
        self.get_logger().debug(
            f"[servo] Twist: axis={axis}, delta={delta:.2f}, speed_mm_s={speed_mm_s:.1f}, "
            f"frame={self.current_frame}"
        )

    # ======================================================
    # Frame / Params
    # ======================================================

    def _on_frame_changed(self, frame_ui: str) -> None:
        """
        Wird vom Widget gefeuert, wenn WRF/TRF umgeschaltet wird.
        """
        self._set_frame_from_ui(frame_ui)

    def _set_frame_from_ui(self, frame_ui: str) -> None:
        f = (frame_ui or "").strip().lower()
        old = self.current_frame

        if f == "trf":
            self.current_frame = self.tcp_frame
        else:
            self.current_frame = self.world_frame

        if self.current_frame != old:
            self.get_logger().info(f"[servo] frame changed: {old} -> {self.current_frame}")

    def _on_params_changed(self, cfg: Dict[str, Any]) -> None:
        """
        Optional: Widget-Parameter-Änderungen (Step, Speed, ...) landen hier.
        Aktuell nur Logging.
        """
        self.get_logger().debug(f"[servo] paramsChanged: {cfg!r}")
