# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, Callable, Type
import json

from PyQt6 import QtCore

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import String as MsgString, Bool as MsgBool, Float64 as MsgFloat64, Empty as MsgEmpty
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from config.startup import AppContent
from .base_bridge import BaseBridge


class MoveItPySignals(QtCore.QObject):
    """
    Qt-Signale für MoveItPy Motion Planning / Execution
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
        # Ergebnistext
        if self.last_result:
            self.motionResultChanged.emit(self.last_result)

        # Trajektorien
        if self._last_planned is not None:
            self.plannedTrajectoryChanged.emit(self._last_planned)
        if self._last_executed is not None:
            self.executedTrajectoryChanged.emit(self._last_executed)


class MoveItPyBridge(BaseBridge):
    """
    Bridge zwischen UI und MoveItPy Motion Node

    WICHTIG:
      - YAML ist "node-sicht":
          subscribe = Node subscribt (UI -> Node)
          publish   = Node publisht (Node -> UI)

      - Falls BaseBridge intern "spiegelt", ignorieren wir das hier komplett
        und verdrahten sauber per _ensure_pub/_ensure_sub.
    """

    GROUP = "moveit_py"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = MoveItPySignals()

        self._pending_named: Optional[str] = None
        self._pending_pose: bool = False

        self._last_planned_traj: Optional[RobotTrajectoryMsg] = None
        self._last_executed_traj: Optional[RobotTrajectoryMsg] = None

        # eigene Verdrahtung
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

        # ------------------------
        # UI -> Node (Node subscribt)
        # ------------------------
        self._ensure_pub("plan_named", MsgString)
        self._ensure_pub("plan_pose", PoseStamped)
        self._ensure_pub("execute", MsgBool)
        self._ensure_pub("stop", MsgEmpty)  # optional
        self._ensure_pub("set_speed_mm_s", MsgFloat64)
        self._ensure_pub("set_planner_cfg", MsgString)

        # ------------------------
        # Node -> UI (Node publisht)
        # ------------------------
        self._ensure_sub("motion_result", MsgString, self._on_motion_result)
        self._ensure_sub("planned_trajectory_rt", RobotTrajectoryMsg, self._on_planned_traj)
        self._ensure_sub("executed_trajectory_rt", RobotTrajectoryMsg, self._on_executed_traj)

        self.get_logger().info(
            "[moveitpy] MoveItPyBridge initialisiert "
            f"(namespace='{self.namespace or 'default'}') – FIX wiring active."
        )

    # ─────────────────────────────────────────────────────────────
    # Helpers: YAML spec → topic name + QoS
    # ─────────────────────────────────────────────────────────────

    def _resolve_ns_topic(self, name: str) -> str:
        name = (name or "").strip()
        if not name:
            return name
        if name.startswith("/"):
            return name

        ns = (self.namespace or "").strip().strip("/")
        if not ns:
            return "/" + name
        return f"/{ns}/{name}".replace("//", "/")

    def _qos_from_spec(self, spec: Any) -> QoSProfile:
        qos_id = getattr(spec, "qos", None) or getattr(spec, "qos_id", None) or "default"
        qos_id = str(qos_id).lower()

        if qos_id == "sensor_data":
            return qos_profile_sensor_data

        if qos_id == "latched":
            return QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _spec_from_content(self, direction: str, topic_id: str) -> Any:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _topic_name_from_content(self, direction: str, topic_id: str) -> str:
        spec = self._spec_from_content(direction, topic_id)
        raw_name = getattr(spec, "name", None) or getattr(spec, "topic", None) or ""
        return self._resolve_ns_topic(str(raw_name))

    def _ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        # UI -> Node: Publisher auf YAML 'subscribe'
        if topic_id in self._ui_to_node_pubs:
            return
        spec = self._spec_from_content("subscribe", topic_id)
        topic = self._topic_name_from_content("subscribe", topic_id)
        qos = self._qos_from_spec(spec)

        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[moveitpy] PUB ui->node id={topic_id} topic={topic}")

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb: Callable) -> None:
        # Node -> UI: Subscription auf YAML 'publish'
        if topic_id in self._node_to_ui_subs:
            return
        spec = self._spec_from_content("publish", topic_id)
        topic = self._topic_name_from_content("publish", topic_id)
        qos = self._qos_from_spec(spec)

        self._node_to_ui_subs[topic_id] = self.create_subscription(msg_type, topic, cb, qos)
        self.get_logger().info(f"[moveitpy] SUB node->ui id={topic_id} topic={topic}")

    def _pub_ui_to_node(self, topic_id: str):
        p = self._ui_to_node_pubs.get(topic_id)
        if p is None:
            raise KeyError(f"ui->node publisher '{topic_id}' fehlt (ensure_pub nicht gelaufen?)")
        return p

    # ─────────────────────────────────────────────────────────────
    # Inbound vom Motion Node (Node -> UI)
    # ─────────────────────────────────────────────────────────────

    def _on_motion_result(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self.signals.last_result = text

        # Auto-execute bei plan_named/plan_pose
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
        # optional: MotionNode muss "stop" subscribe haben
        self._pub_ui_to_node("stop").publish(MsgEmpty())

    # ─────────────────────────────────────────────────────────────
    # Public getters
    # ─────────────────────────────────────────────────────────────

    def last_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_planned_traj

    def last_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_executed_traj
