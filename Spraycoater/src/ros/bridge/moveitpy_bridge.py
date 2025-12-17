# -*- coding: utf-8 -*-
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
    Qt-Signale für MoveItPy Planning / Execution
    """

    motionSpeedChanged = QtCore.pyqtSignal(float)
    plannerCfgChanged = QtCore.pyqtSignal(object)

    moveToHomeRequested = QtCore.pyqtSignal()
    moveToServiceRequested = QtCore.pyqtSignal()

    moveToHomeRequestedWithSpeed = QtCore.pyqtSignal(float)
    moveToServiceRequestedWithSpeed = QtCore.pyqtSignal(float)

    moveToPoseRequested = QtCore.pyqtSignal(object)  # PoseStamped

    motionResultChanged = QtCore.pyqtSignal(str)

    plannedTrajectoryChanged = QtCore.pyqtSignal(object)   # RobotTrajectoryMsg
    executedTrajectoryChanged = QtCore.pyqtSignal(object)  # RobotTrajectoryMsg

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self.last_result: str = ""
        self._last_planned: Optional[RobotTrajectoryMsg] = None
        self._last_executed: Optional[RobotTrajectoryMsg] = None

    def _set_last_planned(self, msg: RobotTrajectoryMsg) -> None:
        self._last_planned = msg

    def _set_last_executed(self, msg: RobotTrajectoryMsg) -> None:
        self._last_executed = msg

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        if self.last_result:
            self.motionResultChanged.emit(self.last_result)
        if self._last_planned is not None:
            self.plannedTrajectoryChanged.emit(self._last_planned)
        if self._last_executed is not None:
            self.executedTrajectoryChanged.emit(self._last_executed)


class MoveItPyBridge(BaseBridge):
    """
    Bridge zwischen UI und MoveItPy Node.

    SINGLE SOURCE OF TRUTH:
      - Topic-Namen kommen direkt aus topics.yaml (AppContent.topic_by_id)
      - QoS kommt direkt aus qos.yaml (AppContent.qos)
      - KEIN manuelles Namespace-Prefixing hier!
        (Node-Namespace kommt ausschließlich über rclpy Node namespace / RosBridge)
    """

    GROUP = "moveit_py"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = MoveItPySignals()

        self._pending_named: Optional[str] = None
        self._pending_pose: bool = False

        self._last_planned_traj: Optional[RobotTrajectoryMsg] = None
        self._last_executed_traj: Optional[RobotTrajectoryMsg] = None

        self._ui_to_node_pubs: Dict[str, Any] = {}
        self._node_to_ui_subs: Dict[str, Any] = {}

        super().__init__("moveitpy_bridge", content, namespace=namespace)

        s = self.signals
        s.moveToHomeRequested.connect(self._on_move_home)
        s.moveToServiceRequested.connect(self._on_move_service)
        s.moveToHomeRequestedWithSpeed.connect(self._on_move_home_with_speed)
        s.moveToServiceRequestedWithSpeed.connect(self._on_move_service_with_speed)
        s.moveToPoseRequested.connect(self._on_move_to_pose)
        s.motionSpeedChanged.connect(self._on_motion_speed_changed)
        s.plannerCfgChanged.connect(self._on_planner_cfg_changed)

        # UI -> Node (Node subscribt)
        self._ensure_pub("plan_named", MsgString)
        self._ensure_pub("plan_pose", PoseStamped)
        self._ensure_pub("execute", MsgBool)
        self._ensure_pub("stop", MsgEmpty)  # optional
        self._ensure_pub("set_speed_mm_s", MsgFloat64)
        self._ensure_pub("set_planner_cfg", MsgString)

        # Node -> UI (Node publisht)
        self._ensure_sub("motion_result", MsgString, self._on_motion_result)
        self._ensure_sub("planned_trajectory_rt", RobotTrajectoryMsg, self._on_planned_traj)
        self._ensure_sub("executed_trajectory_rt", RobotTrajectoryMsg, self._on_executed_traj)

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

        topic = str(spec.name)  # MUSS relativ sein (kein führendes "/")
        if topic.startswith("/"):
            raise ValueError(
                f"[moveitpy] topics.yaml Fehler: '{topic}' beginnt mit '/', "
                "damit würde ROS den Node-Namespace ignorieren."
            )

        qos = self._content.qos(str(spec.qos_key))
        return topic, qos

    def _ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        # UI -> Node: Publisher auf YAML 'subscribe'
        if topic_id in self._ui_to_node_pubs:
            return

        topic, qos = self._topic_and_qos("subscribe", topic_id)
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[moveitpy] PUB ui->node id={topic_id} topic={self._resolve_full(topic)} qos={self._qos_name(qos)}")

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb: Callable) -> None:
        # Node -> UI: Subscription auf YAML 'publish'
        if topic_id in self._node_to_ui_subs:
            return

        topic, qos = self._topic_and_qos("publish", topic_id)
        self._node_to_ui_subs[topic_id] = self.create_subscription(msg_type, topic, cb, qos)
        self.get_logger().info(f"[moveitpy] SUB node->ui id={topic_id} topic={self._resolve_full(topic)} qos={self._qos_name(qos)}")

    def _pub_ui_to_node(self, topic_id: str):
        p = self._ui_to_node_pubs.get(topic_id)
        if p is None:
            raise KeyError(f"ui->node publisher '{topic_id}' fehlt (ensure_pub nicht gelaufen?)")
        return p

    # ─────────────────────────────────────────────────────────────
    # kleine Log-Helfer (optional, aber praktisch)
    # ─────────────────────────────────────────────────────────────

    def _resolve_full(self, rel: str) -> str:
        ns = (self.namespace or "").strip().strip("/")
        return f"/{ns}/{rel}".replace("//", "/") if ns else f"/{rel}"

    @staticmethod
    def _qos_name(_qos: Any) -> str:
        # nur fürs Logging – kein Hardcoding von Policies
        try:
            rel = getattr(_qos, "reliability", None)
            dur = getattr(_qos, "durability", None)
            dep = getattr(_qos, "depth", None)
            return f"reliability={rel} durability={dur} depth={dep}"
        except Exception:
            return "qos=?"

    # ─────────────────────────────────────────────────────────────
    # Inbound vom MoveItPy Node (Node -> UI)
    # ─────────────────────────────────────────────────────────────

    def _on_motion_result(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self.signals.last_result = text

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

    def _publish_speed_mm_s(self, speed: float) -> None:
        self._pub_ui_to_node("set_speed_mm_s").publish(MsgFloat64(data=float(speed)))

    def _on_motion_speed_changed(self, speed: float) -> None:
        self._publish_speed_mm_s(speed)

    def _on_planner_cfg_changed(self, cfg: object) -> None:
        raw = cfg if isinstance(cfg, str) else json.dumps(cfg, ensure_ascii=False)
        self._pub_ui_to_node("set_planner_cfg").publish(MsgString(data=str(raw)))

    def _publish_execute(self, flag: bool) -> None:
        self._pub_ui_to_node("execute").publish(MsgBool(data=bool(flag)))

    def publish_stop(self) -> None:
        self._pub_ui_to_node("stop").publish(MsgEmpty())

    # ─────────────────────────────────────────────────────────────
    # Public getters
    # ─────────────────────────────────────────────────────────────

    def last_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_planned_traj

    def last_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_executed_traj
