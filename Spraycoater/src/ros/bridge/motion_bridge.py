# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore

from std_msgs.msg import String as MsgString
from geometry_msgs.msg import PoseStamped

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class MotionSignals(QtCore.QObject):
    """
    Qt-Signale für Motion:

    Vom UI kommend:
      - motionSpeedChanged(float)
      - moveToHomeRequested()
      - moveToServiceRequested()
      - moveToHomeRequestedWithSpeed(float)
      - moveToServiceRequestedWithSpeed(float)

    Zusätzlich (optional):
      - motionResultChanged(str): Texte aus /spraycoater/motion/result
    """

    motionSpeedChanged = QtCore.pyqtSignal(float)

    moveToHomeRequested = QtCore.pyqtSignal()
    moveToServiceRequested = QtCore.pyqtSignal()

    moveToHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveToServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    motionResultChanged = QtCore.pyqtSignal(str)


class MotionBridge(BaseBridge):
    """
    Bridge für 'motion'.

    Home/Service:
      - nutzt plan_named + execute:
          plan_named: "home" oder "service"
          execute: Bool(True)
      → Motion-Node macht TF-Lookup world<-home / world<-service
        und plant dorthin.

    Recipe / freie Posen:
      - Hilfsfunktion move_to_pose(pose: PoseStamped)
        → target_pose + execute

    Zusätzlich:
      - setzt Speed via cmd: "speed:<mm/s>"
      - hört auf motion_result für Text-Feedback
    """

    GROUP = "motion"

    def __init__(self, content: AppContent):
        # Signale VOR super().__init__, analog SceneBridge
        self.signals = MotionSignals()

        # Zuletzt gesetzte Motion-Geschwindigkeit (mm/s)
        self._last_speed_mm_s: float = 100.0

        super().__init__("motion_bridge", content)

        # UI-Signale verdrahten
        s = self.signals
        s.motionSpeedChanged.connect(self._on_speed_changed)

        s.moveToHomeRequested.connect(self._on_move_home)
        s.moveToServiceRequested.connect(self._on_move_service)

        s.moveToHomeRequestedWithSpeed.connect(self._on_move_home_with_speed)
        s.moveToServiceRequestedWithSpeed.connect(self._on_move_service_with_speed)

        self.get_logger().info("[motion] MotionBridge initialisiert (named home/service via TF).")

    # ------------------------------------------------------------------
    # Inbound vom Motion-Node (publish in topics.yaml)
    # ------------------------------------------------------------------

    @sub_handler("motion", "motion_result")
    def _on_motion_result(self, msg: MsgString):
        text = (getattr(msg, "data", "") or "").strip()
        self.get_logger().info(f"[motion] result: {text or '-'}")
        self.signals.motionResultChanged.emit(text)

    # ------------------------------------------------------------------
    # UI → Bridge: Speed-Handling
    # ------------------------------------------------------------------

    def _on_speed_changed(self, speed_mm_s: float):
        self._last_speed_mm_s = float(speed_mm_s)
        self._publish_speed(speed_mm_s)

    def _publish_speed(self, speed_mm_s: float):
        """
        Setzt über /spraycoater/motion/cmd den Speed:
          'speed:<mm/s>'
        """
        try:
            msg_type = self.spec("subscribe", "cmd").resolve_type()
            pub = self.pub("cmd")
            msg = msg_type()
            if hasattr(msg, "data"):
                msg.data = f"speed:{float(speed_mm_s):.3f}"
            pub.publish(msg)
            self.get_logger().info(f"[motion] speed set to {speed_mm_s:.3f} mm/s")
        except Exception as e:
            self.get_logger().error(f"[motion] publish speed failed: {e}")

    # ------------------------------------------------------------------
    # UI → Bridge: Move-Requests (Home/Service = named TF)
    # ------------------------------------------------------------------

    def _on_move_home(self):
        self._do_named("home")

    def _on_move_service(self):
        self._do_named("service")

    def _on_move_home_with_speed(self, speed_mm_s: float):
        self._on_speed_changed(speed_mm_s)
        self._do_named("home")

    def _on_move_service_with_speed(self, speed_mm_s: float):
        self._on_speed_changed(speed_mm_s)
        self._do_named("service")

    # ------------------------------------------------------------------
    # Kern-Move-Logik: Named Frames (home/service)
    # ------------------------------------------------------------------

    def _do_named(self, name: str):
        """
        Schickt:
          - plan_named (String: 'home'/'service')
          - execute (Bool=True)
        zum Motion-Node.

        Der Motion-Node macht dann TF-Lookup world<-name und plant dorthin.
        """
        name = (name or "").strip()
        if not name:
            self.get_logger().warning("[motion] named move ohne Name angefordert.")
            return

        try:
            # 1) plan_named publizieren
            spec_named = self.spec("subscribe", "plan_named")
            MsgNamed = spec_named.resolve_type()
            pub_named = self.pub("plan_named")

            msg_named = MsgNamed()
            if hasattr(msg_named, "data"):
                msg_named.data = name
            pub_named.publish(msg_named)
            self.get_logger().info(f"[motion] plan_named -> '{name}' published.")

            # 2) execute=True publizieren
            self._publish_execute(True, label=name)
        except Exception as e:
            self.get_logger().error(f"[motion] move_named('{name}') failed: {e}")

    # ------------------------------------------------------------------
    # Helper: Execute-Flag setzen
    # ------------------------------------------------------------------

    def _publish_execute(self, flag: bool, label: str = ""):
        try:
            exec_spec = self.spec("subscribe", "execute")
            ExecType = exec_spec.resolve_type()
            pub_exec = self.pub("execute")

            msg_exec = ExecType()
            if hasattr(msg_exec, "data"):
                msg_exec.data = bool(flag)

            pub_exec.publish(msg_exec)
            self.get_logger().info(f"[motion] execute -> {label or '?'} requested ({flag}).")
        except Exception as e:
            self.get_logger().error(f"[motion] publish execute failed: {e}")

    # ------------------------------------------------------------------
    # Recipe / freie Pose: Hilfs-Funktion
    # ------------------------------------------------------------------

    def move_to_pose(self, pose: PoseStamped, set_speed: Optional[float] = None):
        """
        Öffentliche Hilfsfunktion für andere Teile der App (z.B. Recipe-Executor):

          - optional Speed setzen
          - target_pose = PoseStamped publizieren
          - execute(True) setzen

        Motion-Node behandelt das wie _on_target_pose + _on_execute.
        """
        if pose is None:
            self.get_logger().warning("[motion] move_to_pose: pose is None.")
            return

        try:
            if set_speed is not None:
                self._on_speed_changed(float(set_speed))

            # target_pose publizieren
            spec_pose = self.spec("subscribe", "target_pose")
            PoseType = spec_pose.resolve_type()
            pub_pose = self.pub("target_pose")

            msg_pose = PoseType()
            msg_pose.header = pose.header
            msg_pose.pose = pose.pose

            pub_pose.publish(msg_pose)
            self.get_logger().info("[motion] target_pose (recipe) published.")

            # execute=True
            self._publish_execute(True, label="recipe_pose")
        except Exception as e:
            self.get_logger().error(f"[motion] move_to_pose failed: {e}")
