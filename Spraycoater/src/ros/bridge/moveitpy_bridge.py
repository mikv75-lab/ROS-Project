# -*- coding: utf-8 -*-
# File: src/ros/bridge/moveitpy_bridge.py
from __future__ import annotations

from typing import Optional, Dict, Any, Callable, Type
import json

from PyQt6 import QtCore

from std_msgs.msg import String as MsgString, Bool as MsgBool, Float64 as MsgFloat64, Empty as MsgEmpty
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from config.startup import AppContent, TopicSpec
from .base_bridge import BaseBridge


class MoveItPySignals(QtCore.QObject):
    """
    Qt-Signale für MoveItPy Planning / Execution.

    SSoT: Node <-> UI über topics.yaml

    Minimal Contract (no fallbacks):
      Node -> UI:
        - motion_result
        - planned_trajectory_rt
        - executed_trajectory_rt
        - robot_description (latched)
        - robot_description_semantic (latched)

      UI -> Node:
        - plan_pose, plan_named, execute, stop, set_speed_mm_s, set_planner_cfg

    EXTENDED (Replay/Optimize without extra node):
      UI -> Node:
        - execute_trajectory (RobotTrajectoryMsg)   # execute given joint trajectory
        - set_segment (String)                     # optional tagging for motion_result
    """

    # ---------------- UI -> Node ----------------
    motionSpeedChanged = QtCore.pyqtSignal(float)
    plannerCfgChanged = QtCore.pyqtSignal(object)

    moveToHomeRequested = QtCore.pyqtSignal()
    moveToServiceRequested = QtCore.pyqtSignal()

    moveToHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveToServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    moveToPoseRequested = QtCore.pyqtSignal(object)  # PoseStamped
    stopRequested = QtCore.pyqtSignal()              # -> publish stop topic

    # NEW: replay/optimize path (joint-space)
    executeTrajectoryRequested = QtCore.pyqtSignal(object)  # RobotTrajectoryMsg
    segmentChanged = QtCore.pyqtSignal(str)                 # "MOVE_RECIPE" etc. (optional)

    # ---------------- Node -> UI ----------------
    motionResultChanged = QtCore.pyqtSignal(str)
    plannedTrajectoryChanged = QtCore.pyqtSignal(object)   # RobotTrajectoryMsg
    executedTrajectoryChanged = QtCore.pyqtSignal(object)  # RobotTrajectoryMsg
    robotDescriptionChanged = QtCore.pyqtSignal(str)       # URDF string (latched)
    robotDescriptionSemanticChanged = QtCore.pyqtSignal(str)  # SRDF string (latched)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # cached values (für reemit_cached)
        self.last_result: str = ""
        self._last_planned: Optional[RobotTrajectoryMsg] = None
        self._last_executed: Optional[RobotTrajectoryMsg] = None
        self._last_urdf: str = ""
        self._last_srdf: str = ""

    def _set_last_planned(self, msg: RobotTrajectoryMsg) -> None:
        self._last_planned = msg

    def _set_last_executed(self, msg: RobotTrajectoryMsg) -> None:
        self._last_executed = msg

    def _set_last_urdf(self, text: str) -> None:
        self._last_urdf = text or ""

    def _set_last_srdf(self, text: str) -> None:
        self._last_srdf = text or ""

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        if self.last_result:
            self.motionResultChanged.emit(self.last_result)
        if self._last_planned is not None:
            self.plannedTrajectoryChanged.emit(self._last_planned)
        if self._last_executed is not None:
            self.executedTrajectoryChanged.emit(self._last_executed)
        if self._last_urdf:
            self.robotDescriptionChanged.emit(self._last_urdf)
        if self._last_srdf:
            self.robotDescriptionSemanticChanged.emit(self._last_srdf)


class MoveItPyBridge(BaseBridge):
    """
    Bridge zwischen UI und MoveItPy Node.

    SINGLE SOURCE OF TRUTH:
      - Topic-Namen kommen direkt aus topics.yaml
      - QoS kommt direkt aus qos.yaml
      - KEIN manuelles Namespace-Prefixing hier!

    No fallback topics. If YAML says a topic exists, we bind it.
    """

    GROUP = "moveit_py"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = MoveItPySignals()

        self._pending_named: Optional[str] = None
        self._pending_pose: bool = False

        # synchronous caches (read by RosBridge getters)
        self._last_planned_traj: Optional[RobotTrajectoryMsg] = None
        self._last_executed_traj: Optional[RobotTrajectoryMsg] = None
        self._last_urdf: str = ""
        self._last_srdf: str = ""

        self._ui_to_node_pubs: Dict[str, Any] = {}
        self._node_to_ui_subs: Dict[str, Any] = {}

        super().__init__("moveitpy_bridge", content, namespace=namespace)

        s = self.signals
        s.moveToHomeRequested.connect(self._on_move_home)
        s.moveToServiceRequested.connect(self._on_move_service)
        s.moveToHomeRequestedWithSpeed.connect(self._on_move_home_with_speed)
        s.moveToServiceRequestedWithSpeed.connect(self._on_move_service_with_speed)
        s.moveToPoseRequested.connect(self._on_move_to_pose)
        s.stopRequested.connect(self._on_stop_requested)

        s.motionSpeedChanged.connect(self._on_motion_speed_changed)
        s.plannerCfgChanged.connect(self._on_planner_cfg_changed)

        # NEW
        s.executeTrajectoryRequested.connect(self._on_execute_trajectory_requested)
        s.segmentChanged.connect(self._on_segment_changed)

        # ---------------- UI -> Node (Node subscribes) ----------------
        self._ensure_pub("plan_named", MsgString)
        self._ensure_pub("plan_pose", PoseStamped)
        self._ensure_pub("execute", MsgBool)
        self._ensure_pub("stop", MsgEmpty)
        self._ensure_pub("set_speed_mm_s", MsgFloat64)
        self._ensure_pub("set_planner_cfg", MsgString)

        # NEW: replay/optimize (only if present in topics.yaml)
        self._maybe_ensure_pub("execute_trajectory", RobotTrajectoryMsg)
        self._maybe_ensure_pub("set_segment", MsgString)

        # ---------------- Node -> UI (Node publishes) ----------------
        self._ensure_sub("motion_result", MsgString, self._on_motion_result)
        self._ensure_sub("planned_trajectory_rt", RobotTrajectoryMsg, self._on_planned_traj)
        self._ensure_sub("executed_trajectory_rt", RobotTrajectoryMsg, self._on_executed_traj)
        self._ensure_sub("robot_description", MsgString, self._on_robot_description)
        self._ensure_sub("robot_description_semantic", MsgString, self._on_robot_description_semantic)

        self.get_logger().info(
            "[moveitpy] MoveItPyBridge initialisiert "
            f"(namespace='{self.namespace or 'default'}') – SSoT topics/qos active."
        )

    # ─────────────────────────────────────────────────────────────
    # Helpers: TopicSpec aus AppContent
    # ─────────────────────────────────────────────────────────────

    def _spec(self, direction: str, topic_id: str) -> TopicSpec:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _topic_and_qos(self, direction: str, topic_id: str) -> tuple[str, Any]:
        spec = self._spec(direction, topic_id)
        topic = str(spec.name)
        if topic.startswith("/"):
            raise ValueError(
                f"[moveitpy] topics.yaml Fehler: '{topic}' beginnt mit '/', "
                "damit würde ROS den Node-Namespace ignorieren."
            )
        qos = self._content.qos(str(spec.qos_key))
        return topic, qos

    def _ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        if topic_id in self._ui_to_node_pubs:
            return
        topic, qos = self._topic_and_qos("subscribe", topic_id)
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[moveitpy] PUB ui->node id={topic_id} topic={self._resolve_full(topic)}")

    def _maybe_ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        """
        Optional topic binding: bind ONLY if topic exists in topics.yaml.
        This keeps the bridge usable with older minimal topics.yaml.
        """
        if topic_id in self._ui_to_node_pubs:
            return
        try:
            topic, qos = self._topic_and_qos("subscribe", topic_id)
        except Exception as e:
            self.get_logger().warning(f"[moveitpy] optional PUB missing id={topic_id} ({e})")
            return
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[moveitpy] PUB ui->node id={topic_id} topic={self._resolve_full(topic)}")

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb: Callable) -> None:
        if topic_id in self._node_to_ui_subs:
            return
        topic, qos = self._topic_and_qos("publish", topic_id)
        self._node_to_ui_subs[topic_id] = self.create_subscription(msg_type, topic, cb, qos)
        self.get_logger().info(f"[moveitpy] SUB node->ui id={topic_id} topic={self._resolve_full(topic)}")

    def _pub_ui_to_node(self, topic_id: str):
        return self._ui_to_node_pubs[topic_id]

    def _has_pub(self, topic_id: str) -> bool:
        return topic_id in self._ui_to_node_pubs

    def _resolve_full(self, rel: str) -> str:
        ns = (self.namespace or "").strip().strip("/")
        return f"/{ns}/{rel}".replace("//", "/") if ns else f"/{rel}"

    # ─────────────────────────────────────────────────────────────
    # Inbound vom MoveItPy Node (Node -> UI)
    # ─────────────────────────────────────────────────────────────

    def _on_motion_result(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self.signals.last_result = text

        # Auto-Execute nach Planning (wie bisher)
        if self._pending_named:
            if text.startswith("PLANNED:OK") and f"named='{self._pending_named}'" in text:
                self._publish_execute(True)
                self._pending_named = None
            elif text.startswith("ERROR"):
                self._pending_named = None

        if self._pending_pose:
            if text.startswith("PLANNED:OK pose"):
                self._publish_execute(True)
                self._pending_pose = False
            elif text.startswith("ERROR"):
                self._pending_pose = False

        self.signals.motionResultChanged.emit(text)

    def _on_planned_traj(self, msg: RobotTrajectoryMsg) -> None:
        self._last_planned_traj = msg
        self.signals._set_last_planned(msg)
        self.signals.plannedTrajectoryChanged.emit(msg)

    def _on_executed_traj(self, msg: RobotTrajectoryMsg) -> None:
        self._last_executed_traj = msg
        self.signals._set_last_executed(msg)
        self.signals.executedTrajectoryChanged.emit(msg)

    def _on_robot_description(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self._last_urdf = text
        self.signals._set_last_urdf(text)
        self.signals.robotDescriptionChanged.emit(text)

    def _on_robot_description_semantic(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self._last_srdf = text
        self.signals._set_last_srdf(text)
        self.signals.robotDescriptionSemanticChanged.emit(text)

    # ─────────────────────────────────────────────────────────────
    # UI → MoveItPy (UI -> Node)
    # ─────────────────────────────────────────────────────────────

    def _do_named(self, name: str) -> None:
        name = (name or "").strip()
        if not name:
            return
        self._pending_named = name
        self._pub_ui_to_node("plan_named").publish(MsgString(data=name))

    def _on_move_home(self) -> None:
        self._do_named("home")

    def _on_move_service(self) -> None:
        self._do_named("service")

    def _on_move_home_with_speed(self, speed: float) -> None:
        self._publish_speed_mm_s(speed)
        self._do_named("home")

    def _on_move_service_with_speed(self, speed: float) -> None:
        self._publish_speed_mm_s(speed)
        self._do_named("service")

    def _on_move_to_pose(self, pose: PoseStamped) -> None:
        if pose is None:
            return
        self._pending_pose = True
        out = PoseStamped()
        out.header = pose.header
        out.pose = pose.pose
        self._pub_ui_to_node("plan_pose").publish(out)

    def _on_stop_requested(self) -> None:
        self.publish_stop()

    def _publish_speed_mm_s(self, speed: float) -> None:
        self._pub_ui_to_node("set_speed_mm_s").publish(MsgFloat64(data=float(speed)))

    def _on_motion_speed_changed(self, speed: float) -> None:
        self._publish_speed_mm_s(speed)

    def _on_planner_cfg_changed(self, cfg: object) -> None:
        raw = cfg if isinstance(cfg, str) else json.dumps(cfg, ensure_ascii=False)
        self._pub_ui_to_node("set_planner_cfg").publish(MsgString(data=str(raw)))

    def _publish_execute(self, flag: bool) -> None:
        self._pub_ui_to_node("execute").publish(MsgBool(data=bool(flag)))

    # ---------------- NEW: execute trajectory / segment tagging ----------------

    def _on_execute_trajectory_requested(self, traj: object) -> None:
        """
        Expects RobotTrajectoryMsg. Publishes to moveit_py/execute_trajectory.
        If topics.yaml does not provide execute_trajectory, we raise a clear error.
        """
        if traj is None:
            return
        if not isinstance(traj, RobotTrajectoryMsg):
            raise TypeError(f"executeTrajectoryRequested expects RobotTrajectoryMsg, got {type(traj)!r}")
        if not self._has_pub("execute_trajectory"):
            raise RuntimeError(
                "MoveItPyBridge: topic 'execute_trajectory' nicht gebunden. "
                "Prüfe topics.yaml (moveit_py.subscribe id=execute_trajectory)."
            )
        self._pub_ui_to_node("execute_trajectory").publish(traj)

    def _on_segment_changed(self, seg: str) -> None:
        """
        Optional. Allows statemachine to tag current segment so node emits:
          EXECUTED:OK seg=...
        If topic is missing in topics.yaml, we silently ignore (best-effort).
        """
        if not self._has_pub("set_segment"):
            return
        seg = (seg or "").strip()
        self._pub_ui_to_node("set_segment").publish(MsgString(data=seg))

    # ---------------- Public API ----------------

    def publish_stop(self) -> None:
        self._pub_ui_to_node("stop").publish(MsgEmpty())

    def publish_execute_trajectory(self, traj: RobotTrajectoryMsg, *, segment: str = "") -> None:
        """
        Convenience helper for statemachines:
          - optionally set segment tag
          - publish execute_trajectory
        """
        if segment:
            self._on_segment_changed(segment)
        self._on_execute_trajectory_requested(traj)

    # ─────────────────────────────────────────────────────────────
    # Public getters (used by RosBridge accessors)
    # ─────────────────────────────────────────────────────────────

    def last_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_planned_traj

    def last_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_executed_traj

    def last_robot_description(self) -> str:
        return self._last_urdf

    def last_robot_description_semantic(self) -> str:
        return self._last_srdf
