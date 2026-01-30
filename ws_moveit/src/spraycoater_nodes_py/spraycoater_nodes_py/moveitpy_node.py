#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spraycoater_nodes_py/moveit_py_node.py

MoveItPy wrapper node (MINIMAL TOPICS for GUI Offline-FK) - KEYED / OUT-OF-ORDER SAFE
"""

from __future__ import annotations

import json
import time
from threading import Thread, Event, Lock
from typing import Optional, Dict, Any, List, Tuple, Set

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from rclpy.action import ActionClient

from std_msgs.msg import (
    String as MsgString,
    Empty as MsgEmpty,
    Float64 as MsgFloat64,
    Bool as MsgBool,
)
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState

from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg, PlanningScene
from moveit_msgs.srv import GetPlanningScene

# MoveItPy Imports
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from controller_manager_msgs.srv import SwitchController
from control_msgs.action import FollowJointTrajectory 
from trajectory_msgs.msg import JointTrajectoryPoint

# Config helper (muss im Projekt existieren)
from spraycoater_nodes_py.utils.config_hub import topics, frames


# ----------------------------
# Constants / Defaults
# ----------------------------
NODE_KEY = "moveit_py"
GROUP_NAME = "omron_arm_group"
EE_LINK = "tcp"
WORLD_FRAME = "world"

DEFAULT_PLANNER_CFG: Dict[str, Any] = {
    "pipeline": "ompl",
    "planner_id": "RRTConnectkConfigDefault",
    "planning_time": 10.0,
    "planning_attempts": 100,
    "max_velocity_scaling_factor": 1.00,
    "max_acceleration_scaling_factor": 1.00,
}

DEFAULT_TRAJ_CONTROLLER = "omron_arm_controller"


# ----------------------------
# Retiming bindings (STRICT)
# ----------------------------
_HAVE_TRAJ_PROC = False
try:
    from moveit.core.robot_state import RobotState
    from moveit.core.trajectory_processing import (
        TimeOptimalTrajectoryGeneration,
        IterativeParabolicTimeParameterization,
    )
    _HAVE_TRAJ_PROC = True
except Exception:
    _HAVE_TRAJ_PROC = False


# ----------------------------
# Controller Mode Manager
# ----------------------------
class _ModeManager:
    """
    Minimal controller mode arbiter (embedded).
    """
    def __init__(
        self,
        node: Node,
        *,
        traj_controller: str = DEFAULT_TRAJ_CONTROLLER,
        debounce_s: float = 0.25,
    ) -> None:
        self.node = node
        self.traj_controller = traj_controller
        self.debounce_s = float(debounce_s)

        ns = (node.get_namespace() or "").rstrip("/")
        cm_base = f"{ns}/controller_manager" if ns else "/controller_manager"
        self._srv_name = f"{cm_base}/switch_controller"
        self._cli = node.create_client(SwitchController, self._srv_name)

        self._current_mode: Optional[str] = None
        self._pending: Optional[str] = None
        self._last_req_t = 0.0

    def _set_asap_and_timeout(self, req: SwitchController.Request, *, timeout_s: float) -> None:
        if hasattr(req, "activate_asap"):
            setattr(req, "activate_asap", True)
        elif hasattr(req, "start_asap"):
            setattr(req, "start_asap", True)

        if hasattr(req, "timeout"):
            try:
                setattr(req, "timeout", Duration(seconds=float(timeout_s)).to_msg())
            except Exception:
                try:
                    setattr(req, "timeout", float(timeout_s))
                except Exception:
                    pass

    def ensure_mode(self, mode: str) -> None:
        mode = (mode or "").strip().upper()
        if mode not in ("JOG", "TRAJ"):
            self.node.get_logger().warning(f"[ModeManager] unknown mode '{mode}'")
            return

        now = time.time()
        if self._pending == mode or self._current_mode == mode:
            return
        if (now - self._last_req_t) < self.debounce_s:
            return

        if not self._cli.service_is_ready():
            self._cli.wait_for_service(timeout_sec=0.0)
            if not self._cli.service_is_ready():
                self.node.get_logger().warning(f"[ModeManager] switch_controller not ready: {self._srv_name}")
                return

        activate: List[str] = []
        deactivate: List[str] = []
        if mode == "JOG":
            deactivate = [self.traj_controller]
        else:
            activate = [self.traj_controller]

        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.BEST_EFFORT
        self._set_asap_and_timeout(req, timeout_s=2.0)

        self._pending = mode
        self._last_req_t = now
        fut = self._cli.call_async(req)

        def _done(_fut):
            self._pending = None
            try:
                resp = _fut.result()
                ok = bool(getattr(resp, "ok", False))
                if ok:
                    self._current_mode = mode
                    self.node.get_logger().info(
                        f"[ModeManager] mode={mode} ok (activate={activate}, deactivate={deactivate})"
                    )
                else:
                    self.node.get_logger().error(f"[ModeManager] switch failed (mode={mode})")
            except Exception as e:
                self.node.get_logger().error(f"[ModeManager] switch exc: {e!r}")

        fut.add_done_callback(_done)


# ----------------------------
# Executed trajectory recorder (STRICT: from joint_states)
# ----------------------------
class _ExecutedRecorder:
    """
    Records executed joint motion from JointState while a trajectory is executing.
    """

    _MIN_SAMPLE_DT_NS = 5_000_000  # 5ms
    _STAMP_MAX_SKEW_NS = 5_000_000_000  # 5s

    def __init__(self, node: Node) -> None:
        self._node = node
        self._lock = Lock()
        self._active: bool = False
        self._start_ns: int = 0
        self._jn: List[str] = []
        self._samples: List[Tuple[int, List[float]]] = []
        self._last_t_ns: int = -1

        self._latest_names: List[str] = []
        self._latest_pos: List[float] = []
        self._latest_stamp_ns: int = 0

    @staticmethod
    def _norm_name(n: str) -> str:
        return str(n or "").strip().lstrip("/")

    @staticmethod
    def _stamp_to_ns(msg: JointState) -> int:
        try:
            st = getattr(getattr(msg, "header", None), "stamp", None)
            if st is not None:
                return int(getattr(st, "sec", 0)) * 1_000_000_000 + int(getattr(st, "nanosec", 0))
        except Exception:
            pass
        return 0

    def _now_ns(self) -> int:
        return int(self._node.get_clock().now().nanoseconds)

    def _select_stamp_ns(self, msg_stamp_ns: int) -> int:
        now_ns = self._now_ns()
        if msg_stamp_ns <= 0:
            return now_ns
        if now_ns > 0 and abs(msg_stamp_ns - now_ns) > self._STAMP_MAX_SKEW_NS:
            return now_ns
        return msg_stamp_ns

    def _vec_for_jn(self, names: List[str], pos: List[float], jn: List[str]) -> Optional[List[float]]:
        if not names or not pos or len(names) != len(pos):
            return None

        name_to_i: Dict[str, int] = {}
        for i, n in enumerate(names):
            nn = self._norm_name(n)
            if nn and nn not in name_to_i:
                name_to_i[nn] = i

        req = [self._norm_name(n) for n in (jn or [])]
        if any((not r) or (r not in name_to_i) for r in req):
            return None
        return [float(pos[name_to_i[r]]) for r in req]

    def feed_joint_state(self, msg: JointState) -> None:
        try:
            names = list(msg.name or [])
            pos = list(msg.position or [])
        except Exception:
            return
        if not names or not pos or len(names) != len(pos):
            return

        msg_stamp_ns = self._stamp_to_ns(msg)
        stamp_ns = self._select_stamp_ns(msg_stamp_ns)

        with self._lock:
            self._latest_names = names
            self._latest_pos = pos
            self._latest_stamp_ns = stamp_ns

            if not self._active or not self._jn:
                return

            vec = self._vec_for_jn(names, pos, self._jn)
            if vec is None:
                return

            t_ns = stamp_ns - self._start_ns
            if t_ns < 0:
                t_ns = self._last_t_ns + self._MIN_SAMPLE_DT_NS if self._last_t_ns >= 0 else 0

            if self._last_t_ns >= 0:
                if t_ns <= self._last_t_ns:
                    t_ns = self._last_t_ns + self._MIN_SAMPLE_DT_NS
                elif (t_ns - self._last_t_ns) < self._MIN_SAMPLE_DT_NS:
                    t_ns = self._last_t_ns + self._MIN_SAMPLE_DT_NS

            self._last_t_ns = t_ns
            self._samples.append((t_ns, vec))

    def start(self, joint_names: List[str]) -> None:
        jn = [self._norm_name(x) for x in (joint_names or [])]
        jn = [x for x in jn if x]
        if not jn:
            raise ValueError("ExecutedRecorder.start: joint_names empty")

        with self._lock:
            self._active = True
            self._jn = jn
            self._samples = []
            self._last_t_ns = -1
            self._start_ns = self._now_ns()

            vec0 = self._vec_for_jn(self._latest_names, self._latest_pos, self._jn)
            if vec0 is not None:
                self._samples.append((0, vec0))
                self._last_t_ns = 0

    def stop_and_build(self) -> RobotTrajectoryMsg:
        with self._lock:
            self._active = False
            jn = list(self._jn)
            samples = list(self._samples)

            latest_names = list(self._latest_names)
            latest_pos = list(self._latest_pos)
            latest_stamp_ns = int(self._latest_stamp_ns)
            start_ns = int(self._start_ns)
            last_t = int(self._last_t_ns)

            self._jn = []
            self._samples = []
            self._last_t_ns = -1
            self._start_ns = 0

        if not jn:
            raise RuntimeError("ExecutedRecorder: no joint_names")

        if len(samples) < 2:
            vec_last = self._vec_for_jn(latest_names, latest_pos, jn)
            if vec_last is None:
                raise RuntimeError(
                    "ExecutedRecorder: insufficient samples (<2) and latest joint_state lacks required joints"
                )
            t_ns = (
                latest_stamp_ns - start_ns
                if (latest_stamp_ns > 0 and start_ns > 0)
                else (last_t + self._MIN_SAMPLE_DT_NS)
            )
            if t_ns <= last_t:
                t_ns = last_t + self._MIN_SAMPLE_DT_NS
            if t_ns <= 0:
                t_ns = self._MIN_SAMPLE_DT_NS
            samples.append((t_ns, vec_last))

        if len(samples) < 2:
            raise RuntimeError("ExecutedRecorder: insufficient samples (<2)")

        out = RobotTrajectoryMsg()
        out.joint_trajectory.joint_names = list(jn)

        for t_ns, vec in samples:
            if len(vec) != len(jn):
                continue
            pt = JointTrajectoryPoint()
            pt.positions = list(vec)
            pt.time_from_start.sec = int(t_ns // 1_000_000_000)
            pt.time_from_start.nanosec = int(t_ns % 1_000_000_000)
            out.joint_trajectory.points.append(pt)

        if len(out.joint_trajectory.points) < 2:
            raise RuntimeError("ExecutedRecorder: insufficient valid points after build")

        return out

    def cancel(self) -> None:
        with self._lock:
            self._active = False
            self._jn = []
            self._samples = []
            self._last_t_ns = -1
            self._start_ns = 0


# ----------------------------
# MoveItPy Node (KEYED)
# ----------------------------
class MoveItPyNode(Node):
    """
    MoveItPy wrapper node (minimal topics, keyed).
    """

    def __init__(self) -> None:
        super().__init__(
            "moveit_py",
            automatically_declare_parameters_from_overrides=True,
        )
        self.log = self.get_logger()

        self.cfg_topics = topics()
        self.cfg_frames = frames()
        self.frame_world = self.cfg_frames.resolve(self.cfg_frames.get("world", WORLD_FRAME))

        if not self.has_parameter("backend"):
            self.declare_parameter("backend", "default")
        self.backend = self.get_parameter("backend").value or "default"

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        if not self.has_parameter("max_tcp_speed_mm_s"):
            self.declare_parameter("max_tcp_speed_mm_s", 300.0)
        self.max_tcp_speed_mm_s = float(self.get_parameter("max_tcp_speed_mm_s").value)

        if not self.has_parameter("plan_request_preset"):
            self.declare_parameter("plan_request_preset", "default_ompl")
        self.plan_request_preset = (
            str(self.get_parameter("plan_request_preset").value or "default_ompl").strip() or "default_ompl"
        )

        if not self.has_parameter("robot_description"):
            self.declare_parameter("robot_description", "")
        if not self.has_parameter("robot_description_semantic"):
            self.declare_parameter("robot_description_semantic", "")

        if not self.has_parameter("traj_controller"):
            self.declare_parameter("traj_controller", DEFAULT_TRAJ_CONTROLLER)
        self.traj_controller = str(self.get_parameter("traj_controller").value or DEFAULT_TRAJ_CONTROLLER).strip()
        if not self.traj_controller:
            self.traj_controller = DEFAULT_TRAJ_CONTROLLER

        ns = self.get_namespace() or "/"
        self.log.info(f"MoveItPyNode starting (backend='{self.backend}', ns='{ns}')")

        self._mode_mgr = _ModeManager(self, traj_controller=self.traj_controller, debounce_s=0.25)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._speed_mm_s: float = 100.0
        self._planner_cfg: Dict[str, Any] = DEFAULT_PLANNER_CFG.copy()
        self._plan_params: Optional[PlanRequestParameters] = None

        # BUSY state (node occupied: planning/executing/optimizing/canceling)
        self._busy: bool = False

        # Execution token model (avoid global cancel races):
        self._exec_token_ctr: int = 0
        self._active_exec_token: int = 0
        self._canceled_tokens: Set[int] = set()

        # --- active execution tracking (for keyed stop) ---
        self._external_active_goal = None
        self._external_active_traj: Optional[RobotTrajectoryMsg] = None
        self._active_key: Optional[Dict[str, Any]] = None
        self._active_key_json: str = ""

        self._robot_model_lock = Lock()
        self._robot_model_published: bool = False

        self._exec_rec = _ExecutedRecorder(self)

        # last planned (for op=execute in plan_request)
        self._last_planned_core: Optional[RobotTrajectoryCore] = None
        self._last_planned_key_json: Optional[str] = None

        # tray_exec_ready (NEW): publish change-only
        self._last_traj_exec_ready: Optional[bool] = None

        # publishers
        self.pub_planned = self.create_publisher(
            RobotTrajectoryMsg,
            self.cfg_topics.publish_topic(NODE_KEY, "planned_trajectory_rt"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "planned_trajectory_rt"),
        )
        self.pub_executed = self.create_publisher(
            RobotTrajectoryMsg,
            self.cfg_topics.publish_topic(NODE_KEY, "executed_trajectory_rt"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "executed_trajectory_rt"),
        )
        self.pub_optimized = self.create_publisher(
            RobotTrajectoryMsg,
            self.cfg_topics.publish_topic(NODE_KEY, "optimized_trajectory_rt"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "optimized_trajectory_rt"),
        )

        self.pub_traj_cache_clear = self.create_publisher(
            MsgEmpty,
            self.cfg_topics.publish_topic(NODE_KEY, "traj_cache_clear"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "traj_cache_clear"),
        )

        self.pub_robot_description = self.create_publisher(
            MsgString,
            self.cfg_topics.publish_topic(NODE_KEY, "robot_description"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "robot_description"),
        )
        self.pub_robot_description_semantic = self.create_publisher(
            MsgString,
            self.cfg_topics.publish_topic(NODE_KEY, "robot_description_semantic"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "robot_description_semantic"),
        )

        self.pub_result = self.create_publisher(
            MsgString,
            self.cfg_topics.publish_topic(NODE_KEY, "motion_result"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "motion_result"),
        )

        # NEW: tray_exec_ready publisher
        self.pub_traj_exec_ready = self.create_publisher(
            MsgBool,
            self.cfg_topics.publish_topic(NODE_KEY, "tray_exec_ready"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "tray_exec_ready"),
        )

        # FollowJT action
        self._followjt_clients: List[Tuple[str, ActionClient]] = []
        for action_name in self._followjt_action_candidates(self.traj_controller):
            self._followjt_clients.append((action_name, ActionClient(self, FollowJointTrajectory, action_name)))
        if self._followjt_clients:
            self.log.info("[followjt] action candidates: " + ", ".join(n for n, _ in self._followjt_clients))

        # joint_states subscriptions (STRICT: include namespace!)
        for jt_topic in self._joint_state_candidates():
            self.create_subscription(JointState, jt_topic, self._on_joint_states, 10)
            self.log.info(f"[joint_states] subscribed: {jt_topic}")

        # ----------------------------
        # subscriptions (KEYED)
        # ----------------------------
        self.create_subscription(
            MsgString,
            self.cfg_topics.subscribe_topic(NODE_KEY, "plan_request"),
            self._on_plan_request,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "plan_request"),
        )
        self.log.info("[topics] subscribed: plan_request")

        self.create_subscription(
            MsgEmpty,
            self.cfg_topics.subscribe_topic(NODE_KEY, "stop"),
            self._on_stop,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "stop"),
        )
        self.create_subscription(
            MsgFloat64,
            self.cfg_topics.subscribe_topic(NODE_KEY, "set_speed_mm_s"),
            self._on_set_speed_mm_s,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "set_speed_mm_s"),
        )
        self.create_subscription(
            MsgString,
            self.cfg_topics.subscribe_topic(NODE_KEY, "set_planner_cfg"),
            self._on_set_planner_cfg,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "set_planner_cfg"),
        )

        self.create_subscription(
            RobotTrajectoryMsg,
            self.cfg_topics.subscribe_topic(NODE_KEY, "execute_trajectory"),
            self._on_execute_trajectory,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "execute_trajectory"),
        )
        self.log.info("[topics] subscribed: execute_trajectory")

        self.create_subscription(
            RobotTrajectoryMsg,
            self.cfg_topics.subscribe_topic(NODE_KEY, "optimize_trajectory"),
            self._on_optimize_trajectory,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "optimize_trajectory"),
        )
        self.log.info("[topics] subscribed: optimize_trajectory")

        # planning scene cache service (optional)
        self._gps_srv = None
        self._gps_sub = None
        self._last_scene: Optional[PlanningScene] = None
        self._warned_no_scene = False
        self._ensure_get_planning_scene_cache_service()

        # MoveItPy init async
        self.moveit: Optional[MoveItPy] = None
        self.arm = None
        self.robot_model = None

        self._moveit_ready = Event()
        self._moveit_failed = Event()
        self._moveit_err: Optional[str] = None
        self._moveit_lock = Lock()

        self._name_space = "" if ns == "/" else ns.strip("/")
        self.log.info(f"[moveitpy] init async: node_ns='{ns}', name_space='{self._name_space}'")
        Thread(target=self._init_moveitpy_background, daemon=True).start()

        # NEW: timer to publish tray_exec_ready (change-only)
        self._ready_timer = self.create_timer(0.10, self._on_ready_timer)  # 10 Hz
        # publish initial state immediately
        self._publish_traj_exec_ready(self._calc_traj_exec_ready())

        self.log.info("MoveItPyNode init done (MoveItPy is initializing in background).")

    # ------------------------------------------------------------
    # BUSY helpers
    # ------------------------------------------------------------
    def _set_busy(self, v: bool) -> None:
        self._busy = bool(v)

    def _is_busy(self) -> bool:
        return bool(self._busy)

    # ------------------------------------------------------------
    # tray_exec_ready (NEW)
    # ------------------------------------------------------------
    def _calc_traj_exec_ready(self) -> bool:
        if self._moveit_failed.is_set():
            return False
        if not self._moveit_ready.is_set():
            return False
        return self._pick_ready_followjt() is not None

    def _publish_traj_exec_ready(self, v: bool) -> None:
        vv = bool(v)
        if self._last_traj_exec_ready is not None and self._last_traj_exec_ready == vv:
            return
        self._last_traj_exec_ready = vv
        try:
            self.pub_traj_exec_ready.publish(MsgBool(data=vv))
        except Exception as e:
            self.log.warning(f"[tray_exec_ready] publish failed: {e!r}")
            return
        self.log.info(f"[tray_exec_ready] {vv}")

    def _on_ready_timer(self) -> None:
        try:
            self._publish_traj_exec_ready(self._calc_traj_exec_ready())
        except Exception:
            # STRICT: never throw inside timer callback
            pass

    # ------------------------------------------------------------
    # joint_states candidates (namespace + root)
    # ------------------------------------------------------------
    def _joint_state_candidates(self) -> List[str]:
        ns = (self.get_namespace() or "/").strip().strip("/")
        cands: List[str] = []
        if ns:
            cands.append(f"/{ns}/joint_states")
        cands.append("/joint_states")
        out: List[str] = []
        seen = set()
        for t in cands:
            if t not in seen:
                seen.add(t)
                out.append(t)
        return out

    def _on_joint_states(self, msg: JointState) -> None:
        self._exec_rec.feed_joint_state(msg)

    # ------------------------------------------------------------
    # Key / JSON helpers (STRICT)
    # ------------------------------------------------------------
    @staticmethod
    def _json_load_strict(s: str, *, what: str) -> Dict[str, Any]:
        try:
            v = json.loads(s)
        except Exception as e:
            raise ValueError(f"{what}: invalid JSON ({e!r})")
        if not isinstance(v, dict):
            raise ValueError(f"{what}: JSON must be object/dict")
        return v

    @staticmethod
    def _require_key_obj(obj: Dict[str, Any]) -> Dict[str, Any]:
        if "key" not in obj or not isinstance(obj["key"], dict):
            raise ValueError("request: missing 'key' dict")
        k = obj["key"]
        for req in ("run", "id", "seg", "op"):
            if req not in k:
                raise ValueError(f"key: missing '{req}'")
        if not isinstance(k["run"], str) or not k["run"].strip():
            raise ValueError("key.run must be non-empty string")
        if not isinstance(k["id"], int):
            raise ValueError("key.id must be int")
        if not isinstance(k["seg"], str) or not k["seg"].strip():
            raise ValueError("key.seg must be non-empty string")
        if not isinstance(k["op"], str) or not k["op"].strip():
            raise ValueError("key.op must be non-empty string")
        return {"run": k["run"].strip(), "id": int(k["id"]), "seg": k["seg"].strip(), "op": k["op"].strip()}

    @staticmethod
    def _key_to_json(k: Dict[str, Any]) -> str:
        return json.dumps(
            {"run": k["run"], "id": int(k["id"]), "seg": k["seg"], "op": k["op"]},
            separators=(",", ":"),
        )

    @staticmethod
    def _key_short(k: Dict[str, Any]) -> str:
        return f"{k['op']}:{k['run']}:{int(k['id'])}:{k['seg']}"

    def _emit_keyed(self, key: Dict[str, Any], status: str, *, extra: Optional[Dict[str, Any]] = None) -> None:
        msg: Dict[str, Any] = {"key": dict(key), "status": str(status)}
        if extra:
            msg["extra"] = dict(extra)
        self.pub_result.publish(MsgString(data=json.dumps(msg, separators=(",", ":"))))
        self.log.info(f"[result] {status} ({self._key_short(key)})")

    def _tag_traj_msg_with_key(self, msg: RobotTrajectoryMsg, key_json: str) -> None:
        jt = getattr(msg, "joint_trajectory", None)
        if jt is None:
            raise RuntimeError("RobotTrajectoryMsg has no joint_trajectory")
        hdr = getattr(jt, "header", None)
        if hdr is None:
            raise RuntimeError("RobotTrajectoryMsg.joint_trajectory has no header")
        hdr.frame_id = key_json

    def _key_from_traj_msg_strict(self, msg: RobotTrajectoryMsg) -> Tuple[Dict[str, Any], str]:
        jt = getattr(msg, "joint_trajectory", None)
        if jt is None:
            raise ValueError("traj: missing joint_trajectory")
        hdr = getattr(jt, "header", None)
        if hdr is None:
            raise ValueError("traj: missing header")
        key_json = str(getattr(hdr, "frame_id", "") or "").strip()
        if not key_json:
            raise ValueError("traj: header.frame_id empty (missing key JSON)")
        key_obj = self._json_load_strict(key_json, what="traj.key")
        if "key" in key_obj and isinstance(key_obj["key"], dict):
            key_obj = key_obj["key"]
        for req in ("run", "id", "seg", "op"):
            if req not in key_obj:
                raise ValueError(f"traj.key: missing '{req}'")
        if not isinstance(key_obj["run"], str) or not key_obj["run"].strip():
            raise ValueError("traj.key.run must be non-empty string")
        if not isinstance(key_obj["id"], int):
            raise ValueError("traj.key.id must be int")
        if not isinstance(key_obj["seg"], str) or not key_obj["seg"].strip():
            raise ValueError("traj.key.seg must be non-empty string")
        if not isinstance(key_obj["op"], str) or not key_obj["op"].strip():
            raise ValueError("traj.key.op must be non-empty string")
        key = {
            "run": key_obj["run"].strip(),
            "id": int(key_obj["id"]),
            "seg": key_obj["seg"].strip(),
            "op": key_obj["op"].strip(),
        }
        return key, self._key_to_json(key)

    # ------------------------------------------------------------
    # Trajectory helpers (STRICT postprocess preconditions)
    # ------------------------------------------------------------
    def _make_empty_traj_msg(self) -> RobotTrajectoryMsg:
        return RobotTrajectoryMsg()

    def _publish_empty_latched_trajs(self, *, reason: str) -> None:
        try:
            empty = self._make_empty_traj_msg()
            self.pub_planned.publish(empty)
            self.pub_executed.publish(empty)
            self.pub_optimized.publish(empty)
            self.log.info(f"[traj] cleared latched planned/executed/optimized (reason='{reason}')")
        except Exception as e:
            self.log.warning(f"[traj] clear latched publish failed: {e!r}")

    @staticmethod
    def _tfs_to_ns(p) -> int:
        sec = int(getattr(p.time_from_start, "sec", 0))
        nsec = int(getattr(p.time_from_start, "nanosec", 0))
        return sec * 1_000_000_000 + nsec

    @staticmethod
    def _ns_to_tfs(ns: int):
        sec = int(ns // 1_000_000_000)
        nsec = int(ns % 1_000_000_000)
        return sec, nsec

    def _normalize_joint_traj_time_zero(self, jt) -> bool:
        pts = list(getattr(jt, "points", []) or [])
        if len(pts) < 1:
            return False

        t0 = self._tfs_to_ns(pts[0])
        changed = False

        if t0 != 0:
            for p in pts:
                ns = self._tfs_to_ns(p) - t0
                if ns < 0:
                    ns = 0
                sec, nsec = self._ns_to_tfs(ns)
                p.time_from_start.sec = sec
                p.time_from_start.nanosec = nsec
            changed = True

        last = -1
        for p in pts:
            ns = self._tfs_to_ns(p)
            if ns <= last:
                ns = last + 1_000_000  # +1ms
                sec, nsec = self._ns_to_tfs(ns)
                p.time_from_start.sec = sec
                p.time_from_start.nanosec = nsec
                changed = True
            last = ns

        return changed

    def _ensure_min_2_points(self, jt) -> bool:
        pts = list(getattr(jt, "points", []) or [])
        if len(pts) >= 2:
            return False
        if len(pts) == 0:
            raise RuntimeError("joint_trajectory has 0 points (invalid)")

        p0 = pts[0]
        p1 = JointTrajectoryPoint()
        p1.positions = list(getattr(p0, "positions", []) or [])

        try:
            if getattr(p0, "velocities", None):
                p1.velocities = list(p0.velocities)
        except Exception:
            pass
        try:
            if getattr(p0, "accelerations", None):
                p1.accelerations = list(p0.accelerations)
        except Exception:
            pass
        try:
            if getattr(p0, "effort", None):
                p1.effort = list(p0.effort)
        except Exception:
            pass

        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        p1.time_from_start.sec = 0
        p1.time_from_start.nanosec = 1_000_000

        jt.points.append(p1)
        return True

    def _sanitize_traj_msg_inplace(self, msg: RobotTrajectoryMsg, *, context: str) -> None:
        jt = getattr(msg, "joint_trajectory", None)
        if jt is None:
            raise RuntimeError(f"{context}: no joint_trajectory in RobotTrajectoryMsg")
        if not getattr(jt, "joint_names", None):
            raise RuntimeError(f"{context}: joint_names empty")
        if not getattr(jt, "points", None):
            raise RuntimeError(f"{context}: points empty")

        self._ensure_min_2_points(jt)
        self._normalize_joint_traj_time_zero(jt)

        if len(jt.points) < 2:
            raise RuntimeError(f"{context}: sanitize failed (points<2)")

    def _clear_traj_cache(self, *, reason: str) -> None:
        self._publish_empty_latched_trajs(reason=reason)
        self.pub_traj_cache_clear.publish(MsgEmpty())
        self.log.info(f"[traj] cache clear signaled (reason='{reason}')")

    # ------------------------------------------------------------
    # FollowJT helpers
    # ------------------------------------------------------------
    def _ns_prefix(self) -> str:
        return (self.get_namespace() or "/").strip().strip("/")

    def _followjt_action_candidates(self, controller_name: str) -> List[str]:
        c = (controller_name or "").strip().strip("/")
        if not c:
            c = DEFAULT_TRAJ_CONTROLLER

        ns = self._ns_prefix()
        out: List[str] = []
        if ns:
            out.append(f"/{ns}/{c}/follow_joint_trajectory")
        out.append(f"/{c}/follow_joint_trajectory")

        seen = set()
        uniq: List[str] = []
        for x in out:
            if x not in seen:
                seen.add(x)
                uniq.append(x)
        return uniq

    def _pick_ready_followjt(self) -> Optional[Tuple[str, ActionClient]]:
        for name, cli in self._followjt_clients:
            try:
                if cli.server_is_ready():
                    return name, cli
            except Exception:
                continue
        return None

    # ------------------------------------------------------------
    # Robot model publication
    # ------------------------------------------------------------
    def _publish_robot_model_strings_once(self) -> None:
        with self._robot_model_lock:
            if self._robot_model_published:
                return

            urdf = str(self.get_parameter("robot_description").value or "").strip()
            srdf = str(self.get_parameter("robot_description_semantic").value or "").strip()

            if not urdf:
                self.log.error("[robot_model] robot_description parameter ist leer.")
                return

            self.pub_robot_description.publish(MsgString(data=urdf))
            if srdf:
                self.pub_robot_description_semantic.publish(MsgString(data=srdf))
            else:
                self.log.warning("[robot_model] robot_description_semantic ist leer (SRDF fehlt/optional).")

            self._robot_model_published = True
            self.log.info(
                f"[robot_model] published robot_description ({len(urdf)} chars) + semantic ({len(srdf)} chars) [latched]"
            )

    # ------------------------------------------------------------
    # Init / Params
    # ------------------------------------------------------------
    def _default_pipeline_id(self) -> str:
        try:
            v = (self._planner_cfg.get("pipeline") or "").strip()
            if v:
                return v
        except Exception:
            pass
        for key in ("planning_pipelines.default_planning_pipeline", "default_planning_pipeline"):
            try:
                if self.has_parameter(key):
                    v = self.get_parameter(key).value
                    if isinstance(v, str) and v.strip():
                        return v.strip()
            except Exception:
                pass
        return "ompl"

    def _init_moveitpy_background(self) -> None:
        try:
            with self._moveit_lock:
                self.moveit = MoveItPy(
                    node_name=self.get_name(),
                    name_space=self._name_space,
                    provide_planning_service=True,
                )
                self.arm = self.moveit.get_planning_component(GROUP_NAME)
                self.robot_model = self.moveit.get_robot_model()

                preset = self.plan_request_preset
                self._plan_params = PlanRequestParameters(self.moveit, preset)
                self._apply_planner_cfg_to_params()
                self.log.info(f"[config] PlanRequestParameters loaded preset='{preset}'")

            self._publish_robot_model_strings_once()

            self._moveit_ready.set()
            self.log.info(f"MoveItPy ready (group={GROUP_NAME}, ns='{self.get_namespace() or '/'}')")
            self.log.info("MoveItPyNode online.")

            # NEW: publish readiness again after init
            self._publish_traj_exec_ready(self._calc_traj_exec_ready())

        except Exception as e:
            self._moveit_err = repr(e)
            self._moveit_failed.set()
            self.log.error(f"[moveitpy] INIT FAILED: {e!r}")
            # NEW: readiness becomes false
            self._publish_traj_exec_ready(False)

    def _require_moveit_ready(self, *, key: Optional[Dict[str, Any]] = None, key_json: str = "") -> bool:
        if self._moveit_ready.is_set():
            return True
        if self._moveit_failed.is_set():
            if key is not None:
                self._emit_keyed(key, "ERROR:MOVEIT_INIT_FAILED", extra={"err": self._moveit_err or ""})
            else:
                self.pub_result.publish(MsgString(data="ERROR:MOVEIT_INIT_FAILED"))
            return False
        if key is not None:
            self._emit_keyed(key, "ERROR:MOVEIT_NOT_READY", extra={"key_json": key_json})
        else:
            self.pub_result.publish(MsgString(data="ERROR:MOVEIT_NOT_READY"))
        return False

    # ------------------------------------------------------------
    # Planning scene cache service (optional)
    # ------------------------------------------------------------
    def _ensure_get_planning_scene_cache_service(self) -> None:
        if self._gps_srv is not None:
            return

        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        qos_scene = QoSProfile(depth=1)
        qos_scene.history = HistoryPolicy.KEEP_LAST
        qos_scene.reliability = ReliabilityPolicy.RELIABLE
        qos_scene.durability = DurabilityPolicy.VOLATILE

        self._gps_sub = self.create_subscription(
            PlanningScene,
            "monitored_planning_scene",
            self._on_monitored_planning_scene,
            qos_scene,
        )
        self._gps_srv = self.create_service(
            GetPlanningScene,
            "get_planning_scene",
            self._on_get_planning_scene,
        )
        self.log.info("[psm] Cache get_planning_scene Service aktiv: get_planning_scene")

    def _on_monitored_planning_scene(self, msg: PlanningScene) -> None:
        self._last_scene = msg

    def _on_get_planning_scene(
        self,
        _req: GetPlanningScene.Request,
        res: GetPlanningScene.Response,
    ) -> GetPlanningScene.Response:
        if self._last_scene is not None:
            res.scene = self._last_scene
            res.scene.is_diff = False
            return res
        if not self._warned_no_scene:
            self._warned_no_scene = True
            self.log.warning("[psm] get_planning_scene requested, aber noch kein monitored_planning_scene empfangen.")
        res.scene = PlanningScene()
        res.scene.is_diff = False
        return res

    # ------------------------------------------------------------
    # Planner parameters
    # ------------------------------------------------------------
    def _apply_planner_cfg_to_params(self) -> None:
        if self._plan_params is None:
            raise RuntimeError("PlanRequestParameters not available")

        cfg = self._planner_cfg
        p = self._plan_params

        pipeline_id = (str(cfg.get("pipeline") or "").strip()) or "ompl"
        planner_id = (str(cfg.get("planner_id") or "").strip()) or "RRTConnectkConfigDefault"

        p.planning_pipeline = pipeline_id  # type: ignore[attr-defined]
        p.planner_id = planner_id  # type: ignore[attr-defined]
        p.planning_time = float(cfg.get("planning_time", DEFAULT_PLANNER_CFG["planning_time"]))
        p.planning_attempts = int(cfg.get("planning_attempts", DEFAULT_PLANNER_CFG["planning_attempts"]))
        p.max_velocity_scaling_factor = float(cfg.get("max_velocity_scaling_factor", 1.0))
        p.max_acceleration_scaling_factor = float(cfg.get("max_acceleration_scaling_factor", 1.0))

        self.log.info(
            f"[config] Planner applied: planning_pipeline='{pipeline_id}', planner_id='{planner_id}', "
            f"time={p.planning_time:.2f}s, attempts={p.planning_attempts}, "
            f"vel_scale={p.max_velocity_scaling_factor:.2f}, acc_scale={p.max_acceleration_scaling_factor:.2f}"
        )

    def _on_set_speed_mm_s(self, msg: MsgFloat64) -> None:
        v = float(msg.data)
        if v <= 0.0:
            self.log.warning(f"[config] set_speed_mm_s ignoriert (<= 0): {v}")
            return
        self._speed_mm_s = v
        self.log.info(f"[config] Motion speed set to {self._speed_mm_s:.1f} mm/s")

    def _on_set_planner_cfg(self, msg: MsgString) -> None:
        raw = msg.data or ""
        incoming = json.loads(raw)
        if not isinstance(incoming, dict):
            raise ValueError("planner_cfg JSON ist kein dict.")

        if "pipeline" in incoming and not str(incoming.get("pipeline") or "").strip():
            incoming.pop("pipeline", None)
        if "planner_id" in incoming and not str(incoming.get("planner_id") or "").strip():
            incoming.pop("planner_id", None)

        self._planner_cfg.update(incoming)
        if not str(self._planner_cfg.get("pipeline") or "").strip():
            self._planner_cfg["pipeline"] = self._default_pipeline_id()

        if self._moveit_ready.is_set():
            self._apply_planner_cfg_to_params()

    # ------------------------------------------------------------
    # TF helpers
    # ------------------------------------------------------------
    def _lookup_tf(self, target: str, source: str):
        if not self.tf_buffer.can_transform(target, source, RclpyTime(), Duration(seconds=1.0)):
            raise RuntimeError(f"TF not available {target} <- {source}")
        return self.tf_buffer.lookup_transform(target, source, RclpyTime())

    def _transform_pose_to_world(self, pose: Pose, frame_id: str) -> PoseStamped:
        frame_id = (frame_id or "").strip()
        if not frame_id:
            raise ValueError("pose.frame empty")

        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = pose

        if frame_id == self.frame_world:
            out = PoseStamped()
            out.header.frame_id = self.frame_world
            out.header.stamp = self.get_clock().now().to_msg()
            out.pose = pose
            return out

        try:
            out = self.tf_buffer.transform(ps, self.frame_world, timeout=Duration(seconds=1.0))
            out.header.frame_id = self.frame_world
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        except Exception as e_primary:
            self.log.warning(f"[tf] buffer.transform failed ({frame_id} -> {self.frame_world}): {e_primary!r}")

        tf = self._lookup_tf(self.frame_world, frame_id)
        p = do_transform_pose(ps.pose, tf)
        out = PoseStamped()
        out.header.frame_id = self.frame_world
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose = p
        return out

    # ------------------------------------------------------------
    # Plan request parsing helpers (STRICT)
    # ------------------------------------------------------------
    @staticmethod
    def _pose_from_dict_strict(d: Dict[str, Any]) -> Pose:
        if not isinstance(d, dict):
            raise ValueError("pose must be dict")
        pos = d.get("position")
        ori = d.get("orientation")
        if not isinstance(pos, dict) or not isinstance(ori, dict):
            raise ValueError("pose.position and pose.orientation must be dict")
        for k in ("x", "y", "z"):
            if k not in pos:
                raise ValueError(f"pose.position missing '{k}'")
        for k in ("x", "y", "z", "w"):
            if k not in ori:
                raise ValueError(f"pose.orientation missing '{k}'")
        p = Pose()
        p.position.x = float(pos["x"])
        p.position.y = float(pos["y"])
        p.position.z = float(pos["z"])
        p.orientation.x = float(ori["x"])
        p.orientation.y = float(ori["y"])
        p.orientation.z = float(ori["z"])
        p.orientation.w = float(ori["w"])
        return p

    # ------------------------------------------------------------
    # Execution helper (async FollowJT; STOP works)
    # ------------------------------------------------------------
    def _next_exec_token(self) -> int:
        self._exec_token_ctr += 1
        if self._exec_token_ctr <= 0:
            self._exec_token_ctr = 1
        return int(self._exec_token_ctr)

    def _is_token_canceled(self, token: int) -> bool:
        return int(token) in self._canceled_tokens

    def _execute_traj_via_followjt_async(
        self,
        *,
        key: Dict[str, Any],
        key_json: str,
        msg: RobotTrajectoryMsg,
        source: str,
    ) -> None:
        """
        Start async FollowJT execution for a sanitized+keyed RobotTrajectoryMsg.

        STRICT:
          - publishes planned before result
          - records executed from joint_states
          - Stop works via cancel_goal_async()
        """
        if self._is_busy():
            self._emit_keyed(key, "ERROR:BUSY", extra={"src": source})
            return

        try:
            self._sanitize_traj_msg_inplace(msg, context=f"{source}:planned_in")
            self._tag_traj_msg_with_key(msg, key_json)
        except Exception as e:
            self._emit_keyed(key, "ERROR:TRAJ_INVALID", extra={"err": repr(e), "src": source})
            return

        self._mode_mgr.ensure_mode("TRAJ")

        # new execution token (cancellation is token-scoped)
        token = self._next_exec_token()
        self._active_exec_token = token

        self._set_busy(True)

        self._active_key = dict(key)
        self._active_key_json = str(key_json or "")

        self._external_active_goal = None
        self._external_active_traj = msg

        # publish planned BEFORE any result line
        self.pub_planned.publish(msg)

        # recorder start
        try:
            self._exec_rec.start(list(msg.joint_trajectory.joint_names))
        except Exception as e:
            self._exec_rec.cancel()
            self._external_active_goal = None
            self._external_active_traj = None
            self._active_key = None
            self._active_key_json = ""
            self._active_exec_token = 0
            self._set_busy(False)
            self._emit_keyed(key, "ERROR:RECORDER_START", extra={"err": repr(e), "src": source})
            return

        picked = self._pick_ready_followjt()
        if picked is None:
            for _name, cli in self._followjt_clients:
                try:
                    cli.wait_for_server(timeout_sec=0.0)
                except Exception:
                    pass
            picked = self._pick_ready_followjt()

        if picked is None:
            self._exec_rec.cancel()
            self._external_active_goal = None
            self._external_active_traj = None
            self._active_key = None
            self._active_key_json = ""
            self._active_exec_token = 0
            self._set_busy(False)
            self._emit_keyed(key, "ERROR:NO_FOLLOWJT_SERVER", extra={"src": source})
            return

        action_name, client = picked
        self.log.info(f"[followjt] using server: {action_name} ({self._key_short(key)})")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg.joint_trajectory

        send_fut = client.send_goal_async(goal)

        def _cleanup_after_done() -> None:
            self._external_active_goal = None
            self._external_active_traj = None
            self._active_key = None
            self._active_key_json = ""
            if self._active_exec_token == token:
                self._active_exec_token = 0
            self._set_busy(False)
            try:
                self._mode_mgr.ensure_mode("JOG")
            except Exception:
                pass

        def _on_goal_sent(fut):
            try:
                gh = fut.result()
            except Exception as e:
                self._exec_rec.cancel()
                self._emit_keyed(key, "ERROR:FOLLOWJT_SEND", extra={"err": repr(e), "src": source})
                _cleanup_after_done()
                return

            if gh is None or not getattr(gh, "accepted", False):
                self._exec_rec.cancel()
                self._emit_keyed(key, "ERROR:GOAL_REJECTED", extra={"src": source})
                _cleanup_after_done()
                return

            self._external_active_goal = gh

            if self._is_token_canceled(token):
                try:
                    gh.cancel_goal_async()
                except Exception:
                    pass

            res_fut = gh.get_result_async()

            def _on_result_done(rf):
                try:
                    wrapped = rf.result()
                    result = getattr(wrapped, "result", None)
                    status = getattr(wrapped, "status", None)
                    code = int(getattr(result, "error_code", -1)) if result is not None else -1

                    if self._is_token_canceled(token):
                        self._exec_rec.cancel()
                        self._emit_keyed(key, "STOPPED:USER", extra={"src": source})
                        return

                    if code == 0:
                        executed_msg = self._exec_rec.stop_and_build()
                        self._sanitize_traj_msg_inplace(executed_msg, context=f"{source}:executed")
                        self._tag_traj_msg_with_key(executed_msg, key_json)
                        self.pub_executed.publish(executed_msg)
                        self._emit_keyed(key, "EXECUTED:OK", extra={"src": source})
                    else:
                        self._exec_rec.cancel()
                        self._emit_keyed(key, "ERROR:EXEC", extra={"code": code, "status": status, "src": source})

                except Exception as e:
                    self._exec_rec.cancel()
                    self._emit_keyed(key, "ERROR:FOLLOWJT_RESULT", extra={"err": repr(e), "src": source})

                finally:
                    _cleanup_after_done()

            res_fut.add_done_callback(_on_result_done)

        send_fut.add_done_callback(_on_goal_sent)

    # ------------------------------------------------------------
    # plan_request dispatcher (KEYED)
    # ------------------------------------------------------------
    def _on_plan_request(self, msg: MsgString) -> None:
        if self._is_busy():
            try:
                obj = self._json_load_strict(msg.data or "", what="plan_request")
                key = self._require_key_obj(obj)
                self._emit_keyed(key, "ERROR:BUSY")
            except Exception:
                self.pub_result.publish(MsgString(data="ERROR:BUSY"))
            return

        try:
            obj = self._json_load_strict(msg.data or "", what="plan_request")
            key = self._require_key_obj(obj)
            key_json = self._key_to_json(key)
            payload = obj.get("payload")
            if not isinstance(payload, dict):
                raise ValueError("request: missing 'payload' dict")
        except Exception as e:
            self.pub_result.publish(MsgString(data=f"ERROR:REQUEST_INVALID {e}"))
            self.log.error(f"[plan_request] invalid: {e!r}")
            return

        if not self._require_moveit_ready(key=key, key_json=key_json):
            return

        op = key["op"]
        reason = self._key_short(key)

        # clear caches for ALL plan_request ops (including execute)
        self._clear_traj_cache(reason=f"plan_request:{reason}")
        self._publish_robot_model_strings_once()

        # planning ops are synchronous -> mark busy for the duration
        if op in ("plan_pose", "plan_pose_array"):
            self._set_busy(True)
            try:
                if op == "plan_pose":
                    self._handle_plan_pose(key, key_json, payload)
                    return
                if op == "plan_pose_array":
                    self._handle_plan_pose_array(key, key_json, payload)
                    return
            except Exception as e:
                self._emit_keyed(key, "ERROR:EX", extra={"err": repr(e)})
                return
            finally:
                self._set_busy(False)

        # execute op triggers async -> helper manages busy lifecycle
        if op == "execute":
            try:
                self._handle_execute_last_planned(key, key_json, payload)
            except Exception as e:
                self._emit_keyed(key, "ERROR:EX", extra={"err": repr(e)})
            return

        self._emit_keyed(key, "ERROR:REQUEST_INVALID", extra={"err": f"unsupported op='{op}'"})

    def _handle_plan_pose(self, key: Dict[str, Any], key_json: str, payload: Dict[str, Any]) -> None:
        frame = payload.get("frame")
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError("payload.frame must be non-empty string")
        pose_d = payload.get("pose")
        if not isinstance(pose_d, dict):
            raise ValueError("payload.pose must be dict")

        pose = self._pose_from_dict_strict(pose_d)
        goal = self._transform_pose_to_world(pose, frame.strip())

        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)

        if self._plan_params is None:
            raise RuntimeError("PlanRequestParameters not available")
        self._apply_planner_cfg_to_params()

        result = self.arm.plan(single_plan_parameters=self._plan_params)
        core = getattr(result, "trajectory", None)
        if core is None:
            self._emit_keyed(key, "ERROR:NO_TRAJ", extra={"what": "plan_pose", "frame": frame.strip()})
            return

        msg_traj = core.get_robot_trajectory_msg()
        self._sanitize_traj_msg_inplace(msg_traj, context="plan_pose:planned")
        self._tag_traj_msg_with_key(msg_traj, key_json)

        self.pub_planned.publish(msg_traj)

        self._last_planned_core = core
        self._last_planned_key_json = key_json

        self._emit_keyed(key, "PLANNED:OK", extra={"op": "plan_pose", "frame": frame.strip()})

    def _handle_plan_pose_array(self, key: Dict[str, Any], key_json: str, payload: Dict[str, Any]) -> None:
        frame = payload.get("frame")
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError("payload.frame must be non-empty string")
        poses_list = payload.get("poses")
        if not isinstance(poses_list, list) or len(poses_list) < 2:
            raise ValueError("payload.poses must be list with len>=2")

        poses_world: List[Pose] = []
        for i, pd in enumerate(poses_list):
            if not isinstance(pd, dict):
                raise ValueError(f"payload.poses[{i}] must be dict")
            p = self._pose_from_dict_strict(pd)
            ps_w = self._transform_pose_to_world(p, frame.strip())
            poses_world.append(ps_w.pose)

        fn = getattr(self.arm, "compute_cartesian_path", None)
        if not callable(fn):
            raise RuntimeError("compute_cartesian_path not available on planning component")

        self.arm.set_start_state_to_current_state()

        max_step = 0.005
        jump_threshold = 0.0

        out = fn(poses_world, max_step, jump_threshold)

        traj_core = None
        fraction = None
        if isinstance(out, tuple) and len(out) >= 1:
            traj_core = out[0]
            if len(out) >= 2:
                fraction = out[1]
        else:
            traj_core = getattr(out, "trajectory", None)
            fraction = getattr(out, "fraction", None)

        if traj_core is None or not hasattr(traj_core, "get_robot_trajectory_msg"):
            self._emit_keyed(key, "ERROR:NO_TRAJ", extra={"what": "plan_pose_array"})
            return

        msg_traj = traj_core.get_robot_trajectory_msg()
        self._sanitize_traj_msg_inplace(msg_traj, context="plan_pose_array:planned")
        self._tag_traj_msg_with_key(msg_traj, key_json)

        self.pub_planned.publish(msg_traj)

        self._last_planned_core = traj_core if isinstance(traj_core, RobotTrajectoryCore) else None
        self._last_planned_key_json = key_json if self._last_planned_core is not None else None

        extra = {"op": "plan_pose_array"}
        if fraction is not None:
            extra["fraction"] = float(fraction)
        self._emit_keyed(key, "PLANNED:OK", extra=extra)

    def _handle_execute_last_planned(self, key: Dict[str, Any], key_json: str, payload: Dict[str, Any]) -> None:
        if payload:
            raise ValueError("payload for op='execute' must be empty dict")

        if self._last_planned_core is None or not self._last_planned_key_json:
            self._emit_keyed(key, "ERROR:NO_PLAN", extra={"what": "execute"})
            return

        msg_traj = self._last_planned_core.get_robot_trajectory_msg()

        # execute via async FollowJT (STOP works reliably)
        self._execute_traj_via_followjt_async(
            key=key,
            key_json=key_json,
            msg=msg_traj,
            source="plan_request:execute",
        )

    # ------------------------------------------------------------
    # execute_trajectory (STRICT: key from jt.header.frame_id JSON)
    # ------------------------------------------------------------
    def _on_execute_trajectory(self, msg: RobotTrajectoryMsg) -> None:
        if self._is_busy():
            self.pub_result.publish(MsgString(data="ERROR:BUSY"))
            return

        try:
            key, key_json = self._key_from_traj_msg_strict(msg)
        except Exception as e:
            self.pub_result.publish(MsgString(data=f"ERROR:TRAJ_KEY_INVALID {e}"))
            return

        if not self._require_moveit_ready(key=key, key_json=key_json):
            return

        self._clear_traj_cache(reason=f"execute_trajectory:{self._key_short(key)}")
        self._publish_robot_model_strings_once()

        jt = getattr(msg, "joint_trajectory", None)
        if jt is None or not getattr(jt, "joint_names", None) or not getattr(jt, "points", None):
            self._emit_keyed(key, "ERROR:EMPTY_TRAJ")
            return

        # Execute via async helper (STOP works reliably)
        self._execute_traj_via_followjt_async(
            key=key,
            key_json=key_json,
            msg=msg,
            source="execute_trajectory",
        )

    # ------------------------------------------------------------
    # optimize_trajectory (STRICT: key from jt.header.frame_id JSON)
    # ------------------------------------------------------------
    def _on_optimize_trajectory(self, msg: RobotTrajectoryMsg) -> None:
        if self._is_busy():
            self.pub_result.publish(MsgString(data="ERROR:BUSY"))
            return

        try:
            key, key_json = self._key_from_traj_msg_strict(msg)
        except Exception as e:
            self.pub_result.publish(MsgString(data=f"ERROR:TRAJ_KEY_INVALID {e}"))
            return

        if not self._require_moveit_ready(key=key, key_json=key_json):
            return

        if not _HAVE_TRAJ_PROC:
            self._emit_keyed(key, "ERROR:OPTIMIZE_BINDINGS_MISSING")
            return

        self._clear_traj_cache(reason=f"optimize_trajectory:{self._key_short(key)}")
        self._publish_robot_model_strings_once()

        jt = getattr(msg, "joint_trajectory", None)
        if jt is None or not getattr(jt, "joint_names", None) or not getattr(jt, "points", None):
            self._emit_keyed(key, "ERROR:EMPTY_TRAJ")
            return

        try:
            self._sanitize_traj_msg_inplace(msg, context="optimize_trajectory:input")
            self._tag_traj_msg_with_key(msg, key_json)  # normalized
        except Exception as e:
            self._emit_keyed(key, "ERROR:TRAJ_INVALID", extra={"err": repr(e)})
            return

        self._set_busy(True)
        try:
            planner_id = str(self._planner_cfg.get("planner_id") or "").strip()
            out_msg = self._retime_robot_trajectory(msg, retimer_name=planner_id)

            self._sanitize_traj_msg_inplace(out_msg, context="optimize_trajectory:output")
            self._tag_traj_msg_with_key(out_msg, key_json)

            self.pub_optimized.publish(out_msg)
            self._emit_keyed(key, "OPTIMIZED:OK")

        except Exception as e:
            self._emit_keyed(key, "ERROR:OPTIMIZE", extra={"err": repr(e)})
        finally:
            self._set_busy(False)

    def _retime_robot_trajectory(self, msg: RobotTrajectoryMsg, *, retimer_name: str) -> RobotTrajectoryMsg:
        if self.robot_model is None:
            raise RuntimeError("robot_model not available (MoveItPy not ready)")
        if not _HAVE_TRAJ_PROC:
            raise RuntimeError("trajectory_processing bindings missing")

        jt = msg.joint_trajectory
        if jt is None or not jt.joint_names or not jt.points or len(jt.points) < 2:
            raise RuntimeError("input trajectory invalid/too short")

        state = RobotState(self.robot_model)
        p0 = jt.points[0]
        if len(p0.positions) != len(jt.joint_names):
            raise RuntimeError("first point positions size != joint_names size")
        for name, val in zip(list(jt.joint_names), list(p0.positions)):
            state.set_variable_position(str(name).lstrip("/"), float(val))  # type: ignore[attr-defined]

        traj = RobotTrajectoryCore(self.robot_model, GROUP_NAME)
        if not hasattr(traj, "set_robot_trajectory_msg"):
            raise RuntimeError("RobotTrajectoryCore.set_robot_trajectory_msg missing in bindings")
        traj.set_robot_trajectory_msg(state, msg)  # type: ignore[attr-defined]

        r = (retimer_name or "").strip().lower()
        use_iptp = ("parabolic" in r) or ("iptp" in r) or ("iterative" in r)
        retimer = IterativeParabolicTimeParameterization() if use_iptp else TimeOptimalTrajectoryGeneration()

        vel_scale = float(self._planner_cfg.get("max_velocity_scaling_factor", 1.0))
        acc_scale = float(self._planner_cfg.get("max_acceleration_scaling_factor", 1.0))
        vel_scale = max(0.01, min(1.00, vel_scale))
        acc_scale = max(0.01, min(1.00, acc_scale))

        fn = getattr(retimer, "compute_time_stamps", None)
        if not callable(fn):
            raise RuntimeError("retimer.compute_time_stamps missing in bindings")

        ok = bool(fn(traj, vel_scale, acc_scale))
        if not ok:
            raise RuntimeError(
                f"retime failed (retimer={'IPTP' if use_iptp else 'TOTG'}, "
                f"vel_scale={vel_scale:.3f}, acc_scale={acc_scale:.3f})"
            )

        if not hasattr(traj, "get_robot_trajectory_msg"):
            raise RuntimeError("RobotTrajectoryCore.get_robot_trajectory_msg missing in bindings")

        out = traj.get_robot_trajectory_msg()  # type: ignore[attr-defined]
        return out

    # ------------------------------------------------------------
    # stop (KEYED, async-safe, clears busy immediately)
    # ------------------------------------------------------------
    def _on_stop(self, _msg: MsgEmpty) -> None:
        # cancel token-scoped (no global cancel flag!)
        tok = int(self._active_exec_token)
        if tok > 0:
            self._canceled_tokens.add(tok)

        # stop recorder now (STRICT)
        try:
            self._exec_rec.cancel()
        except Exception:
            pass

        gh = self._external_active_goal
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self.log.warning(f"[stop] cancel_goal_async failed: {e!r}")

        # keyed stop notification if we have an active key
        if self._active_key is not None:
            try:
                self._emit_keyed(dict(self._active_key), "STOP:REQ")
            except Exception:
                pass
        else:
            self.pub_result.publish(MsgString(data="STOP:REQ"))

        # IMPORTANT: clear busy immediately (GUI can proceed),
        # cancellation completion is handled by execution token in callbacks.
        self._set_busy(False)

        # also clear "active pointers" (callbacks use captured key)
        self._external_active_goal = None
        self._external_active_traj = None
        self._active_key = None
        self._active_key_json = ""

        self.log.info("STOP:REQ")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItPyNode()
    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()