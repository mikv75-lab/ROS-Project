# src/ros/bridge/motion_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Dict, Any
import json

from PyQt6 import QtCore

from std_msgs.msg import String as MsgString
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

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
      - plannerCfgChanged(object)  # z.B. dict mit pipeline/planner_id/params

    Zusätzlich (Inbound aus ROS):
      - motionResultChanged(str): Texte aus /spraycoater/motion/result
      - plannedTrajectoryChanged(RobotTrajectoryMsg)
      - executedTrajectoryChanged(RobotTrajectoryMsg)

    Plus:
      - reemit_cached(): emittiert den letzten bekannten motionResult-Text erneut.
    """

    # Config aus UI
    motionSpeedChanged = QtCore.pyqtSignal(float)
    plannerCfgChanged = QtCore.pyqtSignal(object)

    # Named-Moves ohne Speed
    moveToHomeRequested = QtCore.pyqtSignal()
    moveToServiceRequested = QtCore.pyqtSignal()

    # Named-Moves mit Speed (ServiceTab/MotionWidget)
    moveToHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveToServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    # freie Pose (z.B. aus Recipe / ProcessThread)
    moveToPoseRequested = QtCore.pyqtSignal(object)  # PoseStamped

    # Ergebnis-Text
    motionResultChanged = QtCore.pyqtSignal(str)

    # MoveIt-Trajektorien als Qt-Signale
    plannedTrajectoryChanged = QtCore.pyqtSignal(object)   # RobotTrajectoryMsg
    executedTrajectoryChanged = QtCore.pyqtSignal(object)  # RobotTrajectoryMsg

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

    Protokoll (Themen):

      Subscribe:
        - plan_named       (std_msgs/String)           → /spraycoater/motion/plan_named
        - plan_pose        (geometry_msgs/PoseStamped) → /spraycoater/motion/plan_pose
        - execute          (std_msgs/Bool)             → /spraycoater/motion/execute
        - stop             (std_msgs/Empty)            → /spraycoater/motion/stop
        - set_speed_mm_s   (std_msgs/Float64)          → /spraycoater/motion/set_speed_mm_s
        - set_planner_cfg  (std_msgs/String)           → /spraycoater/motion/set_planner_cfg

      Publish:
        - motion_result         (std_msgs/String)
            z.B. "PLANNED:OK pose", "PLANNED:OK named='home'", "EXECUTED:OK", "ERROR:..."
        - planned_trajectory_rt  (moveit_msgs/RobotTrajectory, latched)
        - executed_trajectory_rt (moveit_msgs/RobotTrajectory, latched)

    Auto-Execute-Logik:
      - Home/Service:
          UI → moveToHomeRequested / moveToServiceRequested
          → plan_named('home'/'service')
          → bei "PLANNED:OK named='...'" wird automatisch execute(True) gesetzt.

      - Recipe / freie Pose:
          UI/ProcessThread → moveToPoseRequested(pose)
          → plan_pose(pose)
          → bei "PLANNED:OK pose" wird automatisch execute(True) gesetzt.
    """

    GROUP = "motion"

    def __init__(self, content: AppContent):
        # Signale VOR super().__init__, analog SceneBridge
        self.signals = MotionSignals()

        # pending named-move (home/service) für Auto-Execute
        self._pending_named: Optional[str] = None

        # pending für freie Pose (Recipe)
        self._pending_pose: bool = False

        # Cache für Trajektorien (MoveIt RobotTrajectoryMsg)
        self._last_planned_traj: Optional[RobotTrajectoryMsg] = None
        self._last_executed_traj: Optional[RobotTrajectoryMsg] = None

        super().__init__("motion_bridge", content)

        # UI-/Thread-Signale verdrahten
        s = self.signals

        # Named-Moves ohne Speed
        s.moveToHomeRequested.connect(self._on_move_home)
        s.moveToServiceRequested.connect(self._on_move_service)

        # Named-Moves mit Speed
        s.moveToHomeRequestedWithSpeed.connect(self._on_move_home_with_speed)
        s.moveToServiceRequestedWithSpeed.connect(self._on_move_service_with_speed)

        # freie Pose über Signal
        s.moveToPoseRequested.connect(self._on_move_to_pose)

        # Config: Speed + PlannerCfg
        s.motionSpeedChanged.connect(self._on_motion_speed_changed)
        s.plannerCfgChanged.connect(self._on_planner_cfg_changed)

        self.get_logger().info(
            "[motion] MotionBridge initialisiert "
            "(plan_named/plan_pose + Auto-Execute + Speed/Planner-Topics)."
        )

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
                self.get_logger().info("[motion] auto-execute for pose")
                self._pending_pose = False
                self._publish_execute(True, label="pose")
            elif text.startswith("ERROR:"):
                self.get_logger().warning(f"[motion] planning for pose failed: {text}")
                self._pending_pose = False

        self.signals.motionResultChanged.emit(text)

    # geplante Trajektorie vom Motion-Node (latched Topic)
    @sub_handler("motion", "planned_trajectory_rt")
    def _on_planned_traj(self, msg: RobotTrajectoryMsg):
        """
        Wird aufgerufen, wenn der Motion-Node eine geplante Trajektorie publiziert.
        """
        self._last_planned_traj = msg
        try:
            self.signals.plannedTrajectoryChanged.emit(msg)
        except Exception as e:
            self.get_logger().error(f"[motion] plannedTrajectoryChanged emit failed: {e}")

    # ausgeführte Trajektorie vom Motion-Node (latched Topic)
    @sub_handler("motion", "executed_trajectory_rt")
    def _on_executed_traj(self, msg: RobotTrajectoryMsg):
        """
        Wird aufgerufen, wenn der Motion-Node eine ausgeführte Trajektorie publiziert.
        """
        self._last_executed_traj = msg
        try:
            self.signals.executedTrajectoryChanged.emit(msg)
        except Exception as e:
            self.get_logger().error(f"[motion] executedTrajectoryChanged emit failed: {e}")

    # ------------------------------------------------------------------
    # UI → Bridge: Move-Requests (Home/Service = named TF)
    # ------------------------------------------------------------------

    def _on_move_home(self):
        self._do_named("home")

    def _on_move_service(self):
        self._do_named("service")

    def _on_move_home_with_speed(self, speed: float):
        """
        Variante mit Speed: zuerst Speed publizieren, dann named-Plan.
        """
        self._publish_speed_mm_s(speed)
        self._do_named("home")

    def _on_move_service_with_speed(self, speed: float):
        """
        Variante mit Speed: zuerst Speed publizieren, dann named-Plan.
        """
        self._publish_speed_mm_s(speed)
        self._do_named("service")

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

            # plan_named publizieren
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
    # Config: Speed + PlannerCfg (nur Topic-Forwarding)
    # ------------------------------------------------------------------

    def _publish_speed_mm_s(self, speed: float):
        """
        Hilfsfunktion: Speed als std_msgs/Float64 auf set_speed_mm_s publizieren.
        """
        try:
            spec = self.spec("subscribe", "set_speed_mm_s")
            MsgType = spec.resolve_type()
            pub = self.pub("set_speed_mm_s")

            msg = MsgType()
            if hasattr(msg, "data"):
                msg.data = float(speed)

            pub.publish(msg)
            self.get_logger().info(f"[motion] set_speed_mm_s -> {float(speed):.1f} mm/s")
        except Exception as e:
            self.get_logger().error(f"[motion] set_speed_mm_s publish failed: {e}")

    def _on_motion_speed_changed(self, speed: float):
        """
        UI-Event: Motion-Speed geändert.
        Wird 1:1 auf das Topic set_speed_mm_s weitergeleitet.
        """
        self._publish_speed_mm_s(speed)

    def _on_planner_cfg_changed(self, cfg: object):
        """
        UI-Event: Planner-Konfiguration geändert.
        Erwartet dict-ähnlich; wird als JSON-String auf set_planner_cfg publiziert.
        """
        try:
            if cfg is None:
                self.get_logger().warning("[motion] plannerCfgChanged: cfg is None")
                return

            raw: str
            if isinstance(cfg, str):
                raw = cfg
            else:
                # dict / Mapping o.ä. -> JSON
                raw = json.dumps(cfg, ensure_ascii=False)

            spec = self.spec("subscribe", "set_planner_cfg")
            MsgType = spec.resolve_type()
            pub = self.pub("set_planner_cfg")

            msg = MsgType()
            if hasattr(msg, "data"):
                msg.data = raw

            pub.publish(msg)
            self.get_logger().info(f"[motion] set_planner_cfg -> {raw}")
        except Exception as e:
            self.get_logger().error(f"[motion] set_planner_cfg publish failed: {e}")

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

          - target (Plan) setzen via plan_pose
          - execute(True) wird NICHT direkt gesetzt,
            sondern nach "PLANNED:OK pose" automatisch.
        """
        if pose is None:
            self.get_logger().warning("[motion] move_to_pose: pose is None")
            return
        self._move_to_pose_impl(pose)

    def _on_move_to_pose(self, pose: PoseStamped):
        """Slot für moveToPoseRequested-Signal."""
        if pose is None:
            self.get_logger().warning("[motion] _on_move_to_pose: pose is None")
            return
        self._move_to_pose_impl(pose)

    def _move_to_pose_impl(self, pose: PoseStamped):
        """
        Zentrale Implementierung für freie Posen.
        Wird von:
          - _on_move_to_pose
          - move_to_pose (API) verwendet.
        """
        if pose is None:
            self.get_logger().warning("[motion] move_to_pose_impl: pose is None.")
            return

        try:
            # Wir erwarten jetzt einen gültigen Plan für genau diese Pose
            self._pending_pose = True

            # plan_pose publizieren
            spec_pose = self.spec("subscribe", "plan_pose")
            PoseType = spec_pose.resolve_type()
            pub_pose = self.pub("plan_pose")

            msg_pose = PoseType()
            msg_pose.header = pose.header
            msg_pose.pose = pose.pose

            self.get_logger().info(
                f"[motion] plan_pose publish: frame_id='{msg_pose.header.frame_id}', "
                f"pos=({msg_pose.pose.position.x:.3f}, "
                f"{msg_pose.pose.position.y:.3f}, "
                f"{msg_pose.pose.position.z:.3f})"
            )

            pub_pose.publish(msg_pose)
            self.get_logger().info(
                "[motion] plan_pose (recipe) published, warte auf PLANNED:OK pose."
            )
            # KEIN execute(True) mehr hier – das macht _on_motion_result nach PLANNED:OK pose.
        except Exception as e:
            self._pending_pose = False
            self.get_logger().error(f"[motion] move_to_pose_impl failed: {e}")

    # ------------------------------------------------------------------
    # Getter für geplante/ausgeführte Trajektorien (optional)
    # ------------------------------------------------------------------

    def last_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_planned_traj

    def last_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_executed_traj
