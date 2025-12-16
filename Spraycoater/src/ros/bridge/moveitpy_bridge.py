# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, Callable, Type
import json

from PyQt6 import QtCore

import rclpy
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from std_msgs.msg import String as MsgString, Bool as MsgBool, Float64 as MsgFloat64
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


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

    plannedTrajectoryChanged = QtCore.pyqtSignal(object)
    executedTrajectoryChanged = QtCore.pyqtSignal(object)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self.last_result: str = ""

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        if self.last_result:
            self.motionResultChanged.emit(self.last_result)


class MoveItPyBridge(BaseBridge):
    """
    Bridge zwischen UI und MoveItPy Motion Node

    WICHTIG:
      - YAML ist "node-sicht": node.subscribe = UI->Node, node.publish = Node->UI
      - BaseBridge ist aktuell bei dir offenbar gespiegelt.
      - Diese Bridge verdrahtet die relevanten Topics deshalb selbst korrekt
        (Publisher auf node.subscribe, Subscriptions auf node.publish).
    """

    GROUP = "moveit_py"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = MoveItPySignals()

        self._pending_named: Optional[str] = None
        self._pending_pose: bool = False

        self._last_planned_traj: Optional[RobotTrajectoryMsg] = None
        self._last_executed_traj: Optional[RobotTrajectoryMsg] = None

        # eigene, korrekt gedrehte Verdrahtung
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

        # ---- FIX: korrekte Verdrahtung herstellen (unabhängig von BaseBridge) ----
        # UI -> Node (Node subscribt diese Topics)
        self._ensure_pub("plan_named", MsgString)
        self._ensure_pub("plan_pose", PoseStamped)
        self._ensure_pub("execute", MsgBool)
        self._ensure_pub("stop", None)  # optional, falls du stop später brauchst
        self._ensure_pub("set_speed_mm_s", MsgFloat64)
        self._ensure_pub("set_planner_cfg", MsgString)

        # Node -> UI (Node publisht diese Topics)
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
        """
        YAML-names sind bei dir relativ (z.B. 'spraycoater/motion/plan_named').
        Wir machen daraus '/<ns>/<name>' (z.B. '/shadow/spraycoater/...').
        """
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
        """
        Minimaler QoS-Mapper passend zu deinem YAML:
          - sensor_data -> qos_profile_sensor_data
          - latched     -> transient_local + reliable
          - default     -> reliable + volatile, depth=10
        """
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

        # default
        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _topic_name_from_content(self, direction: str, topic_id: str) -> str:
        """
        direction ist in YAML node-sicht:
          - 'subscribe' = node subscribt (UI->Node)
          - 'publish'   = node publisht (Node->UI)
        """
        spec = self._content.topic_by_id(self.GROUP, direction, topic_id)
        raw_name = getattr(spec, "name", None) or getattr(spec, "topic", None) or ""
        return self._resolve_ns_topic(str(raw_name))

    def _spec_from_content(self, direction: str, topic_id: str) -> Any:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _ensure_pub(self, topic_id: str, msg_type: Optional[Type]) -> None:
        """
        UI -> Node: Publisher auf YAML 'subscribe' (weil Node subscribt).
        """
        if topic_id in self._ui_to_node_pubs:
            return

        # stop ist optional/Empty – wenn msg_type None, skip (du nutzt stop aktuell nicht)
        if msg_type is None:
            return

        spec = self._spec_from_content("subscribe", topic_id)
        topic = self._topic_name_from_content("subscribe", topic_id)
        qos = self._qos_from_spec(spec)
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[moveitpy] PUB ui->node id={topic_id} topic={topic}")

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb: Callable) -> None:
        """
        Node -> UI: Subscription auf YAML 'publish' (weil Node publisht).
        """
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
            # falls yaml geändert wurde / spät geladen
            # -> nochmal versuchen
            # (msg_type muss dann aber bekannt sein; hier nur die IDs, die wir oben ensured haben)
            raise KeyError(f"ui->node publisher '{topic_id}' fehlt (ensure_pub nicht gelaufen?)")
        return p

    # ─────────────────────────────────────────────────────────────
    # Inbound vom Motion Node (Node -> UI)
    # ─────────────────────────────────────────────────────────────

    @sub_handler("moveit_py", "motion_result")
    def _on_motion_result(self, msg: MsgString):
        # NOTE: decorator ist egal; diese Methode wird von _ensure_sub direkt genutzt.
        text = (msg.data or "").strip()
        self.signals.last_result = text

        if self._pending_named:
            if text.startswith("PLANNED:OK") and f"named='{self._pending_named}'" in text:
                self._publish_execute(True, label=self._pending_named)
                self._pending_named = None
            elif text.startswith("ERROR"):
                self._pending_named = None

        if self._pending_pose:
            if text.startswith("PLANNED:OK pose"):
                self._publish_execute(True, label="pose")
                self._pending_pose = False
            elif text.startswith("ERROR"):
                self._pending_pose = False

        self.signals.motionResultChanged.emit(text)

    @sub_handler("moveit_py", "planned_trajectory_rt")
    def _on_planned_traj(self, msg: RobotTrajectoryMsg):
        self._last_planned_traj = msg
        self.signals.plannedTrajectoryChanged.emit(msg)

    @sub_handler("moveit_py", "executed_trajectory_rt")
    def _on_executed_traj(self, msg: RobotTrajectoryMsg):
        self._last_executed_traj = msg
        self.signals.executedTrajectoryChanged.emit(msg)

    # ─────────────────────────────────────────────────────────────
    # UI → MoveItPy (UI -> Node)
    # ─────────────────────────────────────────────────────────────

    def _do_named(self, name: str):
        name = (name or "").strip()
        if not name:
            return

        self._pending_named = name
        pub = self._pub_ui_to_node("plan_named")
        pub.publish(MsgString(data=name))

    def _on_move_home(self):
        self._do_named("home")

    def _on_move_service(self):
        self._do_named("service")

    def _on_move_home_with_speed(self, speed: float):
        self._publish_speed_mm_s(speed)
        self._do_named("home")

    def _on_move_service_with_speed(self, speed: float):
        self._publish_speed_mm_s(speed)
        self._do_named("service")

    def _on_move_to_pose(self, pose: PoseStamped):
        if pose is None:
            return
        self._pending_pose = True
        pub = self._pub_ui_to_node("plan_pose")
        msg = PoseStamped()
        msg.header = pose.header
        msg.pose = pose.pose
        pub.publish(msg)

    def _publish_speed_mm_s(self, speed: float):
        pub = self._pub_ui_to_node("set_speed_mm_s")
        pub.publish(MsgFloat64(data=float(speed)))

    def _on_motion_speed_changed(self, speed: float):
        self._publish_speed_mm_s(speed)

    def _on_planner_cfg_changed(self, cfg: object):
        raw = cfg if isinstance(cfg, str) else json.dumps(cfg, ensure_ascii=False)
        pub = self._pub_ui_to_node("set_planner_cfg")
        pub.publish(MsgString(data=raw))

    def _publish_execute(self, flag: bool, label: str = ""):
        pub = self._pub_ui_to_node("execute")
        pub.publish(MsgBool(data=bool(flag)))

    def last_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_planned_traj

    def last_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_executed_traj
