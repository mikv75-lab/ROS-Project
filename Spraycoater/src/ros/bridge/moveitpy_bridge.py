# -*- coding: utf-8 -*-
# File: src/ros/bridge/moveitpy_bridge.py
from __future__ import annotations

import json
from typing import Optional, Dict, Any, Callable, Type, Tuple

from PyQt6 import QtCore

from std_msgs.msg import (
    String as MsgString,
    Float64 as MsgFloat64,
    Empty as MsgEmpty,
    Bool as MsgBool,
)
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from config.startup import AppContent, TopicSpec
from .base_bridge import BaseBridge


# ============================================================
# Qt Signals (TOPIC-ONLY)
# ============================================================

class MoveItPySignals(QtCore.QObject):
    """
    Qt-Signale für MoveItPy (TOPIC-ONLY).

    STRICT:
      - Kein UI-only Context (kein segmentChanged, kein cache, keine "convenience").
      - Alle Aktionen gehen über Topics (plan_request/stop/set_* / execute_trajectory/optimize_trajectory).
      - Keys (run/id/seg/op) werden von außen geliefert (SM/Panel) – Bridge ist Transport-only.

    BUSY:
      - busy == "Node ist beschäftigt" (plan/execute/optimize/cancel in-flight)
      - Bridge setzt busy=True beim Publish einer Aktion.
      - Bridge setzt busy=False bei terminal motion_result Status (oder bei STOP).
      - Kein eigener ROS topic dafür (TOPIC-ONLY bleibt: busy wird aus Transport+Result abgeleitet).

    READY:
      - tray_exec_ready == "Controller/Action ready" (Capability, NICHT occupancy)
      - kommt als std_msgs/Bool vom Node.
      - GUI kann dann z.B. can_execute = tray_exec_ready && !busy rechnen.
    """

    # ---------------- UI -> Node (topics) ----------------
    planRequestRequested = QtCore.pyqtSignal(object)          # dict: {"key":{...},"payload":{...}}
    stopRequested = QtCore.pyqtSignal()
    motionSpeedChanged = QtCore.pyqtSignal(float)
    plannerCfgChanged = QtCore.pyqtSignal(object)            # STRICT: dict only

    executeTrajectoryRequested = QtCore.pyqtSignal(object)   # RobotTrajectoryMsg (key in jt.header.frame_id JSON)
    optimizeTrajectoryRequested = QtCore.pyqtSignal(object)  # RobotTrajectoryMsg (key in jt.header.frame_id JSON)

    # ---------------- Node -> UI (topics) ----------------
    motionResultChanged = QtCore.pyqtSignal(str)
    plannedTrajectoryChanged = QtCore.pyqtSignal(object)     # RobotTrajectoryMsg
    executedTrajectoryChanged = QtCore.pyqtSignal(object)    # RobotTrajectoryMsg
    optimizedTrajectoryChanged = QtCore.pyqtSignal(object)   # RobotTrajectoryMsg
    trajCacheClearChanged = QtCore.pyqtSignal()

    robotDescriptionChanged = QtCore.pyqtSignal(str)         # URDF
    robotDescriptionSemanticChanged = QtCore.pyqtSignal(str) # SRDF

    trayExecReadyChanged = QtCore.pyqtSignal(bool)           # tray_exec_ready topic

    # ---------------- Derived state (UI convenience, from topics) ----------------
    busyChanged = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)
        self.last_result: str = ""
        self._busy: bool = False
        self._tray_exec_ready: bool = False

    @property
    def busy(self) -> bool:
        return bool(self._busy)

    def _set_busy(self, v: bool) -> None:
        v = bool(v)
        if self._busy == v:
            return
        self._busy = v
        self.busyChanged.emit(v)

    @property
    def tray_exec_ready(self) -> bool:
        return bool(self._tray_exec_ready)

    def _set_tray_exec_ready(self, v: bool) -> None:
        v = bool(v)
        if self._tray_exec_ready == v:
            return
        self._tray_exec_ready = v
        self.trayExecReadyChanged.emit(v)


# ============================================================
# Bridge (TOPIC-ONLY)
# ============================================================

class MoveItPyBridge(BaseBridge):
    """
    Bridge zwischen UI und MoveItPy Node (TOPIC-ONLY).

    Topics (per topics.yaml moveit_py.*):
      subscribe (UI->Node):
        - plan_request (std_msgs/String JSON)
        - stop (std_msgs/Empty)
        - set_speed_mm_s (std_msgs/Float64)
        - set_planner_cfg (std_msgs/String)
        - execute_trajectory (moveit_msgs/RobotTrajectory)
        - optimize_trajectory (moveit_msgs/RobotTrajectory)

      publish (Node->UI):
        - motion_result (std_msgs/String)
        - planned_trajectory_rt (RobotTrajectory) [latched]
        - executed_trajectory_rt (RobotTrajectory) [latched]
        - optimized_trajectory_rt (RobotTrajectory) [latched]
        - traj_cache_clear (std_msgs/Empty)
        - robot_description (std_msgs/String) [latched]
        - robot_description_semantic (std_msgs/String) [latched]
        - tray_exec_ready (std_msgs/Bool)

    STRICT:
      - REQUIRED topic ids müssen existieren, sonst Fehler (keine Fallbacks).
      - Bridge ist Transport-only: keine ID-Erzeugung, keine Defaults, keine UI-only state.
      - plan_request Envelope wird strikt validiert (key/payload).
      - execute/optimize trajectory verlangen Key-JSON in jt.header.frame_id (strict).

    BUSY (abgeleitet, kein eigener ROS topic):
      - busy=True bei Publish einer "Aktion" (plan_request, execute_trajectory, optimize_trajectory)
      - busy=False bei terminal motion_result Status oder bei stop()

    READY (vom Node):
      - tray_exec_ready==True bedeutet: MoveItPy init ok UND FollowJT action server ready
      - hat nichts mit busy zu tun (Capability vs Occupancy).
    """

    GROUP = "moveit_py"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = MoveItPySignals()

        self._ui_to_node_pubs: Dict[str, Any] = {}
        self._node_to_ui_subs: Dict[str, Any] = {}

        # minimal caches for RosBridge getters (optional)
        self._last_planned_traj: Optional[RobotTrajectoryMsg] = None
        self._last_executed_traj: Optional[RobotTrajectoryMsg] = None
        self._last_optimized_traj: Optional[RobotTrajectoryMsg] = None
        self._last_urdf: str = ""
        self._last_srdf: str = ""

        # readiness cache
        self._last_tray_exec_ready: bool = False

        super().__init__("moveitpy_bridge", content, namespace=namespace)

        s = self.signals

        # UI -> Node (topics)
        s.planRequestRequested.connect(self.publish_plan_request)
        s.stopRequested.connect(self.publish_stop)
        s.motionSpeedChanged.connect(self.publish_set_speed_mm_s)
        s.plannerCfgChanged.connect(self.publish_set_planner_cfg)
        s.executeTrajectoryRequested.connect(self.publish_execute_trajectory)
        s.optimizeTrajectoryRequested.connect(self.publish_optimize_trajectory)

        # Publishers (UI->Node; Node subscribes)
        self._ensure_pub("plan_request", MsgString)
        self._ensure_pub("stop", MsgEmpty)
        self._ensure_pub("set_speed_mm_s", MsgFloat64)
        self._ensure_pub("set_planner_cfg", MsgString)
        self._ensure_pub("execute_trajectory", RobotTrajectoryMsg)
        self._ensure_pub("optimize_trajectory", RobotTrajectoryMsg)

        # Subscribers (Node->UI; Node publishes)
        self._ensure_sub("motion_result", MsgString, self._on_motion_result)
        self._ensure_sub("planned_trajectory_rt", RobotTrajectoryMsg, self._on_planned_traj)
        self._ensure_sub("executed_trajectory_rt", RobotTrajectoryMsg, self._on_executed_traj)
        self._ensure_sub("optimized_trajectory_rt", RobotTrajectoryMsg, self._on_optimized_traj)
        self._ensure_sub("traj_cache_clear", MsgEmpty, self._on_traj_cache_clear)
        self._ensure_sub("robot_description", MsgString, self._on_robot_description)
        self._ensure_sub("robot_description_semantic", MsgString, self._on_robot_description_semantic)

        # tray_exec_ready
        self._ensure_sub("tray_exec_ready", MsgBool, self._on_tray_exec_ready)

        self.get_logger().info(
            "[moveitpy] MoveItPyBridge (TOPIC-ONLY) initialisiert "
            f"(namespace='{self.namespace or 'default'}')"
        )

    # ─────────────────────────────────────────────────────────────
    # Helpers: TopicSpec aus AppContent
    # ─────────────────────────────────────────────────────────────

    def _spec(self, direction: str, topic_id: str) -> TopicSpec:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _topic_and_qos(self, direction: str, topic_id: str) -> Tuple[str, Any]:
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

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb: Callable) -> None:
        if topic_id in self._node_to_ui_subs:
            return
        topic, qos = self._topic_and_qos("publish", topic_id)
        self._node_to_ui_subs[topic_id] = self.create_subscription(msg_type, topic, cb, qos)
        self.get_logger().info(f"[moveitpy] SUB node->ui id={topic_id} topic={self._resolve_full(topic)}")

    def _pub_ui_to_node(self, topic_id: str):
        return self._ui_to_node_pubs[topic_id]

    def _resolve_full(self, rel: str) -> str:
        ns = (self.namespace or "").strip().strip("/")
        return f"/{ns}/{rel}".replace("//", "/") if ns else f"/{rel}"

    # ─────────────────────────────────────────────────────────────
    # STRICT JSON helpers (plan_request envelope)
    # ─────────────────────────────────────────────────────────────

    @staticmethod
    def _require_key_dict(key: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(key, dict) or not key:
            raise TypeError("plan_request.key must be non-empty dict")
        for req in ("run", "id", "seg", "op"):
            if req not in key:
                raise ValueError(f"plan_request.key missing '{req}'")
        run = str(key.get("run") or "").strip()
        op = str(key.get("op") or "").strip()
        seg = str(key.get("seg") or "").strip()
        rid = key.get("id")
        if not run:
            raise ValueError("plan_request.key.run must be non-empty string")
        if not isinstance(rid, int):
            raise ValueError(f"plan_request.key.id must be int, got {type(rid).__name__}")
        if not seg:
            raise ValueError("plan_request.key.seg must be non-empty string (STRICT)")
        if not op:
            raise ValueError("plan_request.key.op must be non-empty string")
        return {"run": run, "id": int(rid), "seg": seg, "op": op}

    def _publish_plan_request(self, *, key: Dict[str, Any], payload: Dict[str, Any]) -> None:
        if not isinstance(payload, dict):
            raise TypeError("plan_request.payload must be dict")
        k = self._require_key_dict(key)
        env: Dict[str, Any] = {"key": k, "payload": dict(payload)}
        raw = json.dumps(env, ensure_ascii=False, separators=(",", ":"))
        self._pub_ui_to_node("plan_request").publish(MsgString(data=raw))

    # ─────────────────────────────────────────────────────────────
    # BUSY: derive terminal statuses from motion_result
    # ─────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_motion_result_status(text: str) -> str:
        """
        motion_result is either:
          - JSON: {"key":{...},"status":"...","extra":{...}}
          - or plain string (legacy/errors)
        """
        t = (text or "").strip()
        if not t:
            return ""
        if t.startswith("{") and t.endswith("}"):
            try:
                obj = json.loads(t)
                if isinstance(obj, dict):
                    return str(obj.get("status") or "").strip()
            except Exception:
                pass
        return t

    @staticmethod
    def _is_terminal_status(status: str) -> bool:
        """
        Terminal means: request completed (success/error/stop).
        NOTE: STOP:REQ is *not* a completion from node; it's an acknowledgement.
              We clear busy already on publish_stop().
        """
        s = (status or "").strip()
        if not s:
            return False
        if s == "STOP:REQ":
            return False
        prefixes = (
            "PLANNED:",
            "EXECUTED:",
            "OPTIMIZED:",
            "ERROR:",
            "STOPPED:",
        )
        return s.startswith(prefixes)

    # ─────────────────────────────────────────────────────────────
    # Inbound vom MoveItPy Node (Node -> UI)
    # ─────────────────────────────────────────────────────────────

    @staticmethod
    def _is_empty_robot_trajectory(msg: RobotTrajectoryMsg) -> bool:
        try:
            jt = getattr(msg, "joint_trajectory", None)
            if jt is None:
                return True
            names = list(getattr(jt, "joint_names", []) or [])
            pts = list(getattr(jt, "points", []) or [])
            return (len(names) == 0) and (len(pts) == 0)
        except Exception:
            return False

    def _on_traj_cache_clear(self, _msg: MsgEmpty) -> None:
        self._last_planned_traj = None
        self._last_executed_traj = None
        self._last_optimized_traj = None
        self.signals.trajCacheClearChanged.emit()

    def _on_motion_result(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self.signals.last_result = text
        self.signals.motionResultChanged.emit(text)

        # BUSY: clear on terminal results
        st = self._parse_motion_result_status(text)
        if self._is_terminal_status(st):
            self.signals._set_busy(False)

    def _on_planned_traj(self, msg: RobotTrajectoryMsg) -> None:
        if self._is_empty_robot_trajectory(msg):
            self._last_planned_traj = None
            self._last_executed_traj = None
            self._last_optimized_traj = None
            self.signals.trajCacheClearChanged.emit()
            return
        self._last_planned_traj = msg
        self.signals.plannedTrajectoryChanged.emit(msg)

    def _on_executed_traj(self, msg: RobotTrajectoryMsg) -> None:
        if self._is_empty_robot_trajectory(msg):
            self._last_planned_traj = None
            self._last_executed_traj = None
            self._last_optimized_traj = None
            self.signals.trajCacheClearChanged.emit()
            return
        self._last_executed_traj = msg
        self.signals.executedTrajectoryChanged.emit(msg)

    def _on_optimized_traj(self, msg: RobotTrajectoryMsg) -> None:
        if self._is_empty_robot_trajectory(msg):
            self._last_planned_traj = None
            self._last_executed_traj = None
            self._last_optimized_traj = None
            self.signals.trajCacheClearChanged.emit()
            return
        self._last_optimized_traj = msg
        self.signals.optimizedTrajectoryChanged.emit(msg)

    def _on_robot_description(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self._last_urdf = text
        self.signals.robotDescriptionChanged.emit(text)

    def _on_robot_description_semantic(self, msg: MsgString) -> None:
        text = (msg.data or "").strip()
        self._last_srdf = text
        self.signals.robotDescriptionSemanticChanged.emit(text)

    def _on_tray_exec_ready(self, msg: MsgBool) -> None:
        v = bool(getattr(msg, "data", False))
        self._last_tray_exec_ready = v
        self.signals._set_tray_exec_ready(v)

    # ─────────────────────────────────────────────────────────────
    # Public transport API (TOPIC-ONLY)
    # ─────────────────────────────────────────────────────────────

    def publish_stop(self) -> None:
        # BUSY: stop clears busy immediately (UI can proceed)
        self.signals._set_busy(False)
        self._pub_ui_to_node("stop").publish(MsgEmpty())

    def publish_set_speed_mm_s(self, speed: float) -> None:
        self._pub_ui_to_node("set_speed_mm_s").publish(MsgFloat64(data=float(speed)))

    def publish_set_planner_cfg(self, cfg: object) -> None:
        """
        STRICT: cfg must be dict-like (will be JSON encoded for std_msgs/String).
        """
        if not isinstance(cfg, dict):
            raise TypeError(f"publish_set_planner_cfg expects dict, got {type(cfg)!r}")
        raw = json.dumps(cfg, ensure_ascii=False, separators=(",", ":"))
        self._pub_ui_to_node("set_planner_cfg").publish(MsgString(data=str(raw)))

    def publish_plan_request(self, req: object) -> None:
        """
        Accepts dict:
          {"key":{run,id,seg,op}, "payload":{...}}
        and publishes it to plan_request.
        """
        if not isinstance(req, dict):
            raise TypeError(f"publish_plan_request expects dict, got {type(req)!r}")
        key = req.get("key")
        payload = req.get("payload")
        if not isinstance(key, dict):
            raise TypeError("publish_plan_request: req['key'] must be dict")
        if not isinstance(payload, dict):
            raise TypeError("publish_plan_request: req['payload'] must be dict")

        # BUSY: any plan_request is an action (plan/execute)
        self.signals._set_busy(True)
        self._publish_plan_request(key=key, payload=payload)

    @staticmethod
    def _require_keyed_traj(msg: RobotTrajectoryMsg) -> None:
        jt = getattr(msg, "joint_trajectory", None)
        if jt is None:
            raise ValueError("RobotTrajectoryMsg missing joint_trajectory")
        hdr = getattr(jt, "header", None)
        if hdr is None:
            raise ValueError("RobotTrajectoryMsg.joint_trajectory missing header")
        key_json = str(getattr(hdr, "frame_id", "") or "").strip()
        if not key_json:
            raise ValueError("RobotTrajectoryMsg.joint_trajectory.header.frame_id empty (missing key JSON)")
        try:
            obj = json.loads(key_json)
        except Exception as e:
            raise ValueError(f"trajectory key JSON invalid: {e!r}")
        if not isinstance(obj, dict):
            raise ValueError("trajectory key JSON must be object/dict")

    def publish_execute_trajectory(self, traj: object) -> None:
        if traj is None:
            return
        if not isinstance(traj, RobotTrajectoryMsg):
            raise TypeError(f"publish_execute_trajectory expects RobotTrajectoryMsg, got {type(traj)!r}")
        self._require_keyed_traj(traj)

        # BUSY: execution is an action
        self.signals._set_busy(True)
        self._pub_ui_to_node("execute_trajectory").publish(traj)

    def publish_optimize_trajectory(self, traj: object) -> None:
        if traj is None:
            return
        if not isinstance(traj, RobotTrajectoryMsg):
            raise TypeError(f"publish_optimize_trajectory expects RobotTrajectoryMsg, got {type(traj)!r}")
        self._require_keyed_traj(traj)

        # BUSY: optimize is an action
        self.signals._set_busy(True)
        self._pub_ui_to_node("optimize_trajectory").publish(traj)

    # getters (optional; used by RosBridge)
    def last_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_planned_traj

    def last_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_executed_traj

    def last_optimized_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self._last_optimized_traj

    def last_robot_description(self) -> str:
        return self._last_urdf

    def last_robot_description_semantic(self) -> str:
        return self._last_srdf

    def last_tray_exec_ready(self) -> bool:
        return bool(self._last_tray_exec_ready)
