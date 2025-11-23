# src/ros/bridge/motion_bridge.py
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

    Vom UI / anderen Threads kommend:
      - motionSpeedChanged(float)
      - moveToHomeRequested()
      - moveToServiceRequested()
      - moveToHomeRequestedWithSpeed(float)
      - moveToServiceRequestedWithSpeed(float)

      - moveToPoseRequested(PoseStamped)
      - moveToPoseWithSpeedRequested(PoseStamped, float)

    Zusätzlich (Inbound aus ROS):
      - motionResultChanged(str): Texte aus /spraycoater/motion/result

    Plus:
      - reemit_cached(): emittiert den letzten bekannten motionResult-Text erneut.
    """

    motionSpeedChanged = QtCore.pyqtSignal(float)

    moveToHomeRequested = QtCore.pyqtSignal()
    moveToServiceRequested = QtCore.pyqtSignal()

    moveToHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveToServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    # freie Pose (z.B. aus Recipe / ProcessThread)
    moveToPoseRequested = QtCore.pyqtSignal(object)              # PoseStamped
    moveToPoseWithSpeedRequested = QtCore.pyqtSignal(object, float)

    motionResultChanged = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        # Cache für Re-Emit
        self.last_result: str = ""

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        """
        Erneutes Aussenden des letzten Motion-Result-Strings.
        Wird von UIBridge._try_reemit_cached() verwendet.
        """
        if self.last_result:
            self.motionResultChanged.emit(self.last_result)


class MotionBridge(BaseBridge):
    """
    Bridge für 'motion'.

    Home/Service:
      - nutzt plan_named + execute:
          plan_named: "home" oder "service"
          execute: Bool(True)
      → ABER: execute wird jetzt erst nach "PLANNED:OK named='...'" geschickt,
        um Rennbedingungen zu vermeiden.

    Recipe / freie Posen:
      - moveToPoseRequested-Signal → _move_to_pose_impl
        → target_pose publizieren
        → execute(True) wird erst nach "PLANNED:OK pose" automatisch gesetzt.

    Hinweis:
      Die Pose darf in beliebigem Frame kommen (z.B. 'scene').
      Der Motion-Node transformiert sie nun intern nach 'world'.
    """

    GROUP = "motion"

    def __init__(self, content: AppContent):
        # Signale VOR super().__init__, analog SceneBridge
        self.signals = MotionSignals()

        # Zuletzt gesetzte Motion-Geschwindigkeit (mm/s)
        self._last_speed_mm_s: float = 100.0

        # pending named-move (home/service) für Auto-Execute
        self._pending_named: Optional[str] = None

        # pending für freie Pose (Recipe)
        self._pending_pose: bool = False

        super().__init__("motion_bridge", content)

        # UI-/Thread-Signale verdrahten
        s = self.signals
        s.motionSpeedChanged.connect(self._on_speed_changed)

        s.moveToHomeRequested.connect(self._on_move_home)
        s.moveToServiceRequested.connect(self._on_move_service)

        s.moveToHomeRequestedWithSpeed.connect(self._on_move_home_with_speed)
        s.moveToServiceRequestedWithSpeed.connect(self._on_move_service_with_speed)

        # freie Pose über Signale
        s.moveToPoseRequested.connect(self._on_move_to_pose)
        s.moveToPoseWithSpeedRequested.connect(self._on_move_to_pose_with_speed)

        self.get_logger().info("[motion] MotionBridge initialisiert (named home/service via TF + Auto-Execute).")

    # ------------------------------------------------------------------
    # Inbound vom Motion-Node (publish in topics.yaml)
    # ------------------------------------------------------------------

    @sub_handler("motion", "motion_result")
    def _on_motion_result(self, msg: MsgString):
        text = (getattr(msg, "data", "") or "").strip()
        self.get_logger().info(f"[motion] result: {text or '-'}")

        # Cache aktualisieren (für reemit_cached)
        self.signals.last_result = text

        # --- Auto-Execute-Logik für named (home/service) ---
        if self._pending_named:
            name = self._pending_named
            if text.startswith("PLANNED:OK") and f"named='{name}'" in text:
                # Plan für das erwartete named-Frame ist fertig -> jetzt execute(True)
                self.get_logger().info(f"[motion] auto-execute for named '{name}'")
                self._pending_named = None
                self._publish_execute(True, label=name)
            elif text.startswith("ERROR:"):
                # Fehler beim Planen -> pending zurücksetzen
                self.get_logger().warning(f"[motion] planning for named '{name}' failed: {text}")
                self._pending_named = None

        # --- Auto-Execute-Logik für freie Pose (Recipe) ---
        if self._pending_pose:
            if text.startswith("PLANNED:OK pose"):
                self.get_logger().info("[motion] auto-execute for recipe_pose")
                self._pending_pose = False
                self._publish_execute(True, label="recipe_pose")
            elif text.startswith("ERROR:"):
                self.get_logger().warning(f"[motion] planning for recipe_pose failed: {text}")
                self._pending_pose = False

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
        (Motion-Node interpretiert das aktuell als Faktor 0.05..1.0.)
        """
        try:
            msg_type = self.spec("subscribe", "cmd").resolve_type()
            pub = self.pub("cmd")
            msg = msg_type()
            if hasattr(msg, "data"):
                msg.data = f"speed:{float(speed_mm_s):.3f}"
            pub.publish(msg)
            self.get_logger().info(f"[motion] speed set to {speed_mm_s:.3f} (raw)")
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
    # UI/Threads → Bridge: freie Pose per Signal
    # ------------------------------------------------------------------

    def _on_move_to_pose(self, pose: PoseStamped):
        """Slot für moveToPoseRequested-Signal (ohne Geschwindigkeitsänderung)."""
        if pose is None:
            self.get_logger().warning("[motion] _on_move_to_pose: pose is None")
            return
        self._move_to_pose_impl(pose, set_speed=None)

    def _on_move_to_pose_with_speed(self, pose: PoseStamped, speed_mm_s: float):
        """Slot für moveToPoseWithSpeedRequested-Signal (mit expliziter Speed-Vorgabe)."""
        if pose is None:
            self.get_logger().warning("[motion] _on_move_to_pose_with_speed: pose is None")
            return
        self._move_to_pose_impl(pose, set_speed=float(speed_mm_s))

    # ------------------------------------------------------------------
    # Kern-Move-Logik: Named Frames (home/service)
    # ------------------------------------------------------------------

    def _do_named(self, name: str):
        """
        Schickt:
          - plan_named (String: 'home'/'service')
        Execute(True) wird AUTOMATISCH ausgelöst,
        sobald der Motion-Node "PLANNED:OK named='<name>'" meldet.
        """
        name = (name or "").strip()
        if not name:
            self.get_logger().warning("[motion] named move ohne Name angefordert.")
            return

        try:
            # Merken, dass wir für dieses named einen Auto-Execute wollen
            self._pending_named = name

            # 1) plan_named publizieren
            spec_named = self.spec("subscribe", "plan_named")
            MsgNamed = spec_named.resolve_type()
            pub_named = self.pub("plan_named")

            msg_named = MsgNamed()
            if hasattr(msg_named, "data"):
                msg_named.data = name
            pub_named.publish(msg_named)
            self.get_logger().info(f"[motion] plan_named -> '{name}' published (auto-exec pending).")

            # KEIN direktes execute mehr hier!
        except Exception as e:
            self._pending_named = None
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
    # Recipe / freie Pose: Hilfs-Funktion (API + Impl)
    # ------------------------------------------------------------------

    def move_to_pose(self, pose: PoseStamped, set_speed: Optional[float] = None):
        """
        Öffentliche Hilfsfunktion für andere Teile der App (Backward-Compat):

          - optional Speed setzen
          - target_pose = PoseStamped publizieren
          - execute(True) wird NICHT mehr direkt gesetzt,
            sondern nach "PLANNED:OK pose" automatisch.

        Motion-Node behandelt das wie _on_target_pose + _on_execute.
        """
        if pose is None:
            self.get_logger().warning("[motion] move_to_pose: pose is None")
            return
        self._move_to_pose_impl(pose, set_speed=set_speed)

    def _move_to_pose_impl(self, pose: PoseStamped, set_speed: Optional[float] = None):
        """
        Zentrale Implementierung für freie Posen.
        Wird von:
          - _on_move_to_pose
          - _on_move_to_pose_with_speed
          - move_to_pose (API) verwendet.
        """
        if pose is None:
            self.get_logger().warning("[motion] move_to_pose_impl: pose is None.")
            return

        try:
            if set_speed is not None:
                self._on_speed_changed(float(set_speed))

            # Wir erwarten jetzt einen gültigen Plan für genau diese Pose
            self._pending_pose = True

            # target_pose publizieren
            spec_pose = self.spec("subscribe", "target_pose")
            PoseType = spec_pose.resolve_type()
            pub_pose = self.pub("target_pose")

            msg_pose = PoseType()
            msg_pose.header = pose.header
            msg_pose.pose = pose.pose

            self.get_logger().info(
                f"[motion] target_pose publish: frame_id='{msg_pose.header.frame_id}', "
                f"pos=({msg_pose.pose.position.x:.3f}, "
                f"{msg_pose.pose.position.y:.3f}, "
                f"{msg_pose.pose.position.z:.3f})"
            )

            pub_pose.publish(msg_pose)
            self.get_logger().info(
                "[motion] target_pose (recipe) published, warte auf PLANNED:OK pose."
            )
            # KEIN execute(True) mehr hier!
        except Exception as e:
            self._pending_pose = False
            self.get_logger().error(f"[motion] move_to_pose_impl failed: {e}")
