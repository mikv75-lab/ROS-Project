#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spraycoater_nodes_py/moveit_py_node.py

MoveItPy wrapper node (MINIMAL TOPICS for GUI Offline-FK) - KEYED / OUT-OF-ORDER SAFE

PATCHES (2026-01):
✅ Default controller mode = TRAJ (trajectory controller active).
✅ Do NOT auto-switch back to JOG after execute; keep TRAJ as normal default.
✅ STOP without goal handle no longer forces JOG (keeps default TRAJ).
✅ Wire optimize_trajectory to OptimizeRequestHandler (handler-only). Node must provide optimize_traj_msg().
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

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from controller_manager_msgs.srv import SwitchController, ListControllers
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from spraycoater_nodes_py.utils.config_hub import topics, frames

from spraycoater_nodes_py.moveit_ops.plan_requests import PlanRequestHandler
from spraycoater_nodes_py.moveit_ops.execute_requests import ExecuteRequestHandler
from spraycoater_nodes_py.moveit_ops.optimize_requests import OptimizeRequestHandler
from spraycoater_nodes_py.moveit_ops.executed_recorder import ExecutedRecorder


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


class _ModeManager:
    """
    Minimal controller mode arbiter (embedded).

    - Mode "TRAJ": ensure trajectory controller active (FollowJT)
    - Mode "JOG": deactivate trajectory controller (so servo/jog can take over)
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


class MoveItPyNode(Node):
    def __init__(self) -> None:
        super().__init__("moveit_py", automatically_declare_parameters_from_overrides=True)
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

        # ----------------------------
        # State
        # ----------------------------
        self._mode_mgr = _ModeManager(self, traj_controller=self.traj_controller, debounce_s=0.25)

        # controller_manager list_controllers
        ns_clean = (self.get_namespace() or "").rstrip("/")
        cm_base = f"{ns_clean}/controller_manager" if ns_clean else "/controller_manager"
        self._list_ctrl_srv_name = f"{cm_base}/list_controllers"
        self._list_ctrl_cli = self.create_client(ListControllers, self._list_ctrl_srv_name)

        self._ctrl_state_lock = Lock()
        self._ctrl_active_cache: Dict[str, bool] = {}
        self._ctrl_cache_t = 0.0
        self._ctrl_query_inflight = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self._speed_mm_s: float = 100.0
        self._planner_cfg: Dict[str, Any] = DEFAULT_PLANNER_CFG.copy()
        self._plan_params: Optional[PlanRequestParameters] = None

        self._busy: bool = False

        self._exec_token_ctr: int = 0
        self._active_exec_token: int = 0
        self._canceled_tokens: Set[int] = set()

        self._external_active_goal = None
        self._external_active_traj: Optional[RobotTrajectoryMsg] = None
        self._active_key: Optional[Dict[str, Any]] = None
        self._active_key_json: str = ""

        self._robot_model_lock = Lock()
        self._robot_model_published: bool = False

        self._exec_rec = ExecutedRecorder(self)

        self._last_planned_core: Optional[RobotTrajectoryCore] = None
        self._last_planned_key_json: Optional[str] = None

        self._plan_req = PlanRequestHandler(api=self)
        self._exec_req = ExecuteRequestHandler(api=self)
        self._opt_req = OptimizeRequestHandler(api=self)

        # tray_exec_ready
        self._last_traj_exec_ready: Optional[bool] = None

        # ✅ NEW: tray_exec_ready stability / hysteresis + reason
        self._ready_true_ctr: int = 0
        self._ready_false_ctr: int = 0
        self._ready_reason: str = ""
        self._last_ready_reason: str = ""

        # ----------------------------
        # Publishers
        # ----------------------------
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
        self.pub_traj_exec_ready = self.create_publisher(
            MsgBool,
            self.cfg_topics.publish_topic(NODE_KEY, "tray_exec_ready"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "tray_exec_ready"),
        )

        # ----------------------------
        # FollowJT action clients
        # ----------------------------
        self._followjt_clients: List[Tuple[str, ActionClient]] = []
        for action_name in self._followjt_action_candidates(self.traj_controller):
            self._followjt_clients.append((action_name, ActionClient(self, FollowJointTrajectory, action_name)))
        if self._followjt_clients:
            self.log.info("[followjt] action candidates: " + ", ".join(n for n, _ in self._followjt_clients))

        # ----------------------------
        # joint_states
        # ----------------------------
        for jt_topic in self._joint_state_candidates():
            self.create_subscription(JointState, jt_topic, self._on_joint_states, 10)
            self.log.info(f"[joint_states] subscribed: {jt_topic}")

        # ----------------------------
        # Subscriptions
        # ----------------------------
        self.create_subscription(
            MsgString,
            self.cfg_topics.subscribe_topic(NODE_KEY, "plan_request"),
            self._on_plan_request_router,
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

        # planning scene cache
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

        # tray_exec_ready timer
        self._ready_timer = self.create_timer(0.10, self._on_ready_timer)

        # ✅ NEW: publish initial state with reason
        try:
            v, reason = self._calc_traj_exec_ready_reason()
            self._publish_traj_exec_ready(v, reason=reason)
        except Exception:
            # fallback (no reason)
            self._publish_traj_exec_ready(False, reason="init_exception")

        # ✅ Default to TRAJ (best-effort; not spammed every 100ms anymore)
        self._mode_mgr.ensure_mode("TRAJ")

        self.log.info("MoveItPyNode init done (MoveItPy is initializing in background).")

    # ============================================================
    # Duck-typed API for handlers
    # ============================================================
    @property
    def plan_params(self) -> Optional[PlanRequestParameters]:
        return self._plan_params

    @property
    def ee_link(self) -> str:
        return EE_LINK

    def is_busy(self) -> bool:
        return bool(self._busy)

    def set_busy(self, v: bool) -> None:
        self._busy = bool(v)

    def require_moveit_ready(self, *, key: Optional[Dict[str, Any]] = None, key_json: str = "") -> bool:
        return self._require_moveit_ready(key=key, key_json=key_json)

    def clear_traj_cache(self, *, reason: str) -> None:
        self._clear_traj_cache(reason=reason)

    def publish_robot_model_strings_once(self) -> None:
        self._publish_robot_model_strings_once()

    def apply_planner_cfg_to_params(self) -> None:
        self._apply_planner_cfg_to_params()

    def transform_pose_to_world(self, pose: Pose, frame_id: str) -> PoseStamped:
        return self._transform_pose_to_world(pose, frame_id)

    def sanitize_traj_msg_inplace(self, msg: RobotTrajectoryMsg, context: str) -> None:
        self._sanitize_traj_msg_inplace(msg, context=context)

    def tag_traj_msg_with_key(self, msg: RobotTrajectoryMsg, key_json: str) -> None:
        self._tag_traj_msg_with_key(msg, key_json)

    def emit_keyed(self, key: Dict[str, Any], status: str, extra: Optional[Dict[str, Any]] = None) -> None:
        self._emit_keyed(key, status, extra=extra)

    def pub_planned_publish(self, msg: RobotTrajectoryMsg) -> None:
        self.pub_planned.publish(msg)

    def pub_optimized_publish(self, msg: RobotTrajectoryMsg) -> None:
        self.pub_optimized.publish(msg)

    def get_last_planned(self) -> Tuple[Optional[RobotTrajectoryCore], Optional[str]]:
        return self._last_planned_core, self._last_planned_key_json

    def set_last_planned(self, core: Optional[RobotTrajectoryCore], key_json: Optional[str]) -> None:
        self._last_planned_core = core
        self._last_planned_key_json = key_json

    def execute_traj_via_followjt_async(
        self, *, key: Dict[str, Any], key_json: str, msg: RobotTrajectoryMsg, source: str
    ) -> None:
        self._execute_traj_via_followjt_async(key=key, key_json=key_json, msg=msg, source=source)

    def publish_raw_result(self, s: str) -> None:
        self.pub_result.publish(MsgString(data=str(s)))

    def log_error(self, s: str) -> None:
        self.log.error(str(s))

    # ------------------------------------------------------------
    # REQUIRED by OptimizeRequestHandler (implement your optimizer here)
    # ------------------------------------------------------------
    def optimize_traj_msg(
        self, *, key: Dict[str, Any], key_json: str, msg: RobotTrajectoryMsg, source: str
    ) -> RobotTrajectoryMsg:
        # If you already extracted optimization logic elsewhere: call it here.
        raise NotImplementedError("optimize_traj_msg() not implemented in MoveItPyNode")

    # ------------------------------------------------------------
    # Controller active check (cached)
    # ------------------------------------------------------------
    def _is_traj_controller_active(self) -> bool:
        name = (self.traj_controller or "").strip() or DEFAULT_TRAJ_CONTROLLER
        now = time.time()

        with self._ctrl_state_lock:
            last_known = bool(self._ctrl_active_cache.get(name, False))
            cache_age = now - float(self._ctrl_cache_t or 0.0)
            if cache_age < 0.5:
                return last_known
            if self._ctrl_query_inflight:
                return last_known
            self._ctrl_query_inflight = True

        cli = self._list_ctrl_cli
        try:
            if not cli.service_is_ready():
                cli.wait_for_service(timeout_sec=0.0)
            if not cli.service_is_ready():
                with self._ctrl_state_lock:
                    self._ctrl_query_inflight = False
                    self._ctrl_cache_t = now
                return last_known
        except Exception:
            with self._ctrl_state_lock:
                self._ctrl_query_inflight = False
                self._ctrl_cache_t = now
            return last_known

        fut = cli.call_async(ListControllers.Request())

        def _done_cb(_f):
            got_value = False
            active = False
            try:
                resp = _f.result()
                for c in list(getattr(resp, "controller", []) or []):
                    if str(getattr(c, "name", "") or "") == name:
                        active = (str(getattr(c, "state", "") or "").lower() == "active")
                        got_value = True
                        break
            except Exception:
                got_value = False
            finally:
                with self._ctrl_state_lock:
                    # NUR wenn wir wirklich einen Wert haben → Cache ändern
                    if got_value:
                        self._ctrl_active_cache[name] = bool(active)

                    # Timestamp trotzdem aktualisieren,
                    # aber Wert nicht auf False zwingen
                    self._ctrl_cache_t = time.time()
                    self._ctrl_query_inflight = False

        fut.add_done_callback(_done_cb)
        return last_known

    def _calc_traj_exec_ready(self) -> bool:
        v, _reason = self._calc_traj_exec_ready_reason()
        return v


    def _calc_traj_exec_ready_reason(self) -> Tuple[bool, str]:
        if self._moveit_failed.is_set():
            return False, "moveit_failed"

        if not self._moveit_ready.is_set():
            return False, "moveit_not_ready"

        if self._pick_ready_followjt() is None:
            return False, "no_followjt"

        if not self._is_traj_controller_active():
            return False, "controller_inactive_or_unknown"

        return True, "ok"


    def _publish_traj_exec_ready(self, v: bool, *, reason: str = "") -> None:
        vv = bool(v)

        # --- Hysterese ---
        # True schnell, False langsam
        if vv:
            self._ready_true_ctr += 1
            self._ready_false_ctr = 0
            # mind. 2 True Samples bevor auf True gewechselt wird
            if self._last_traj_exec_ready is not True and self._ready_true_ctr < 2:
                return
        else:
            self._ready_false_ctr += 1
            self._ready_true_ctr = 0
            # mind. 5 False Samples bevor auf False gewechselt wird
            if self._last_traj_exec_ready is not False and self._ready_false_ctr < 5:
                return

        # Kein State Change → nur Reason updaten
        if self._last_traj_exec_ready is not None and self._last_traj_exec_ready == vv:
            if reason:
                self._ready_reason = reason
            return

        self._last_traj_exec_ready = vv
        if reason:
            self._ready_reason = reason

        try:
            self.pub_traj_exec_ready.publish(MsgBool(data=vv))
        except Exception as e:
            self.log.warning(f"[tray_exec_ready] publish failed: {e!r}")
            return

        self.log.info(f"[tray_exec_ready] {vv} (reason='{self._ready_reason}')")


    def _on_ready_timer(self) -> None:
        try:
            v, reason = self._calc_traj_exec_ready_reason()
            self._publish_traj_exec_ready(v, reason=reason)

            # Nur reparieren, wenn Controller Problem
            if not v and reason.startswith("controller_"):
                self._mode_mgr.ensure_mode("TRAJ")

        except Exception:
            pass


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
        return {"run": str(k["run"]).strip(), "id": int(k["id"]), "seg": str(k["seg"]).strip(), "op": str(k["op"]).strip()}

    @staticmethod
    def _key_to_json(k: Dict[str, Any]) -> str:
        return json.dumps({"run": k["run"], "id": int(k["id"]), "seg": k["seg"], "op": k["op"]}, separators=(",", ":"))

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

    # ------------------------------------------------------------
    # Trajectory sanitize + cache clear
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
        c = (controller_name or "").strip().strip("/") or DEFAULT_TRAJ_CONTROLLER
        ns = self._ns_prefix()
        out: List[str] = []
        if ns:
            out.append(f"/{ns}/{c}/follow_joint_trajectory")
        out.append(f"/{c}/follow_joint_trajectory")
        uniq: List[str] = []
        seen = set()
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
            self.log.info(f"[robot_model] published robot_description ({len(urdf)} chars) + semantic ({len(srdf)} chars) [latched]")

    # ------------------------------------------------------------
    # Init MoveItPy
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

            # ✅ enforce default TRAJ once ready
            self._mode_mgr.ensure_mode("TRAJ")
            self._publish_traj_exec_ready(self._calc_traj_exec_ready())

        except Exception as e:
            self._moveit_err = repr(e)
            self._moveit_failed.set()
            self.log.error(f"[moveitpy] INIT FAILED: {e!r}")
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
    # planning scene cache service (optional)
    # ------------------------------------------------------------
    def _ensure_get_planning_scene_cache_service(self) -> None:
        if self._gps_srv is not None:
            return
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        qos_scene = QoSProfile(depth=1)
        qos_scene.history = HistoryPolicy.KEEP_LAST
        qos_scene.reliability = ReliabilityPolicy.RELIABLE
        qos_scene.durability = DurabilityPolicy.VOLATILE
        self._gps_sub = self.create_subscription(PlanningScene, "monitored_planning_scene", self._on_monitored_planning_scene, qos_scene)
        self._gps_srv = self.create_service(GetPlanningScene, "get_planning_scene", self._on_get_planning_scene)
        self.log.info("[psm] Cache get_planning_scene Service aktiv: get_planning_scene")

    def _on_monitored_planning_scene(self, msg: PlanningScene) -> None:
        self._last_scene = msg

    def _on_get_planning_scene(self, _req: GetPlanningScene.Request, res: GetPlanningScene.Response) -> GetPlanningScene.Response:
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
    # Planner cfg
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
        try:
            incoming = json.loads(raw)
        except Exception as e:
            self.log.error(f"[config] set_planner_cfg invalid JSON: {e!r}")
            return
        if not isinstance(incoming, dict):
            self.log.error("[config] set_planner_cfg JSON ist kein dict.")
            return
        if "pipeline" in incoming and not str(incoming.get("pipeline") or "").strip():
            incoming.pop("pipeline", None)
        if "planner_id" in incoming and not str(incoming.get("planner_id") or "").strip():
            incoming.pop("planner_id", None)
        self._planner_cfg.update(incoming)
        if not str(self._planner_cfg.get("pipeline") or "").strip():
            self._planner_cfg["pipeline"] = self._default_pipeline_id()
        if self._moveit_ready.is_set():
            try:
                self._apply_planner_cfg_to_params()
            except Exception as e:
                self.log.error(f"[config] apply_planner_cfg failed: {e!r}")

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
    # plan_request router
    # ------------------------------------------------------------
    def _on_plan_request_router(self, msg: MsgString) -> None:
        raw = (msg.data or "").strip()
        if not raw or not raw.startswith("{"):
            self._plan_req.on_plan_request(msg)
            return
        try:
            obj = self._json_load_strict(raw, what="plan_request")
            key = self._require_key_obj(obj)
            op = str(key.get("op") or "").strip()
        except Exception:
            self._plan_req.on_plan_request(msg)
            return
        if op == "execute":
            self._exec_req.on_plan_request(msg)
        else:
            self._plan_req.on_plan_request(msg)

    # ------------------------------------------------------------
    # execute/optimize entry points
    # ------------------------------------------------------------
    def _on_execute_trajectory(self, msg: RobotTrajectoryMsg) -> None:
        self._exec_req.on_execute_trajectory(msg)

    def _on_optimize_trajectory(self, msg: RobotTrajectoryMsg) -> None:
        self._opt_req.on_optimize_trajectory(msg)

    # ------------------------------------------------------------
    # STOP handling
    # ------------------------------------------------------------
    def _on_stop(self, _msg: MsgEmpty) -> None:
        try:
            token = int(self._active_exec_token)
            if token > 0:
                self._canceled_tokens.add(token)

            if self._active_key is not None:
                self._emit_keyed(dict(self._active_key), "STOP:REQ")

            gh = self._external_active_goal
            if gh is not None:
                try:
                    gh.cancel_goal_async()
                except Exception as e:
                    self.log.warning(f"[stop] cancel_goal_async failed: {e!r}")

            # ✅ IMPORTANT: do NOT force mode JOG here (default stays TRAJ)
        except Exception as e:
            self.log.error(f"[stop] exception: {e!r}")

    # ------------------------------------------------------------
    # FollowJT execution (single source of truth; starts recorder here)
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
        if self.is_busy():
            self._emit_keyed(key, "ERROR:BUSY", extra={"src": source})
            return

        # Ensure default TRAJ (best-effort)
        self._mode_mgr.ensure_mode("TRAJ")

        if not self._calc_traj_exec_ready():
            self._emit_keyed(key, "ERROR:EXEC_NOT_READY", extra={"src": source, "controller": self.traj_controller})
            return

        try:
            self._sanitize_traj_msg_inplace(msg, context=f"{source}:planned_in")
            self._tag_traj_msg_with_key(msg, key_json)
        except Exception as e:
            self._emit_keyed(key, "ERROR:TRAJ_INVALID", extra={"err": repr(e), "src": source})
            return

        # micro-wait after controller switch
        t_end = time.time() + 0.50
        while time.time() < t_end:
            if self._calc_traj_exec_ready():
                break
            time.sleep(0.02)

        if not self._calc_traj_exec_ready():
            self._emit_keyed(
                key,
                "ERROR:EXEC_NOT_READY",
                extra={"src": source, "controller": self.traj_controller, "phase": "post_switch_wait"},
            )
            return

        token = self._next_exec_token()
        self._active_exec_token = token
        self.set_busy(True)

        self._active_key = dict(key)
        self._active_key_json = str(key_json or "")

        self._external_active_goal = None
        self._external_active_traj = msg

        # publish planned (latched)
        self.pub_planned.publish(msg)

        # ✅ start executed recorder here (single source)
        try:
            self._exec_rec.start(list(msg.joint_trajectory.joint_names))
        except Exception as e:
            self._exec_rec.cancel()
            self._external_active_goal = None
            self._external_active_traj = None
            self._active_key = None
            self._active_key_json = ""
            self._active_exec_token = 0
            self.set_busy(False)
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
            self.set_busy(False)
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
            self.set_busy(False)
            # ✅ IMPORTANT: do NOT switch to JOG here. Keep TRAJ default.

        def _finish_exec_ok() -> None:
            try:
                executed_msg = self._exec_rec.stop_and_build()
                self._sanitize_traj_msg_inplace(executed_msg, context=f"{source}:executed")
                self._tag_traj_msg_with_key(executed_msg, key_json)
                self.pub_executed.publish(executed_msg)
            except Exception as e:
                self._exec_rec.cancel()
                self._emit_keyed(key, "ERROR:RECORDER_BUILD", extra={"err": repr(e), "src": source})
                return
            self._emit_keyed(key, "EXECUTED:OK", extra={"src": source})

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
                        self._emit_keyed(key, "STOPPED:USER", extra={"src": source, "status": int(status or 0)})
                        return

                    if code == 0:
                        _finish_exec_ok()
                        return

                    self._exec_rec.cancel()
                    self._emit_keyed(
                        key,
                        "ERROR:EXEC_FAILED",
                        extra={"src": source, "error_code": int(code), "status": int(status or 0)},
                    )
                except Exception as e:
                    self._exec_rec.cancel()
                    self._emit_keyed(key, "ERROR:FOLLOWJT_RESULT", extra={"err": repr(e), "src": source})
                finally:
                    _cleanup_after_done()

            res_fut.add_done_callback(_on_result_done)

        send_fut.add_done_callback(_on_goal_sent)


def main(argv: Optional[List[str]] = None) -> None:
    rclpy.init(args=argv)
    node: Optional[MoveItPyNode] = None
    try:
        node = MoveItPyNode()
        rclpy.spin(node)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
