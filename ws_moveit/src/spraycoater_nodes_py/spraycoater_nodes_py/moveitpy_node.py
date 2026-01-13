#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
spraycoater_nodes_py/moveit_py_node.py

MoveItPy wrapper node (MINIMAL TOPICS for GUI Offline-FK)

Publishes (per config_hub topics.yaml):
  - planned_trajectory_rt        (moveit_msgs/msg/RobotTrajectory) [latched]
  - executed_trajectory_rt       (moveit_msgs/msg/RobotTrajectory) [latched]
  - optimized_trajectory_rt      (moveit_msgs/msg/RobotTrajectory) [latched]
  - traj_cache_clear             (std_msgs/msg/Empty)              [default, NOT latched]
  - robot_description            (std_msgs/msg/String)             [latched]
  - robot_description_semantic   (std_msgs/msg/String)             [latched]
  - motion_result                (std_msgs/msg/String)             [default]

Subscribes:
  - plan_pose       (geometry_msgs/msg/PoseStamped)
  - plan_named      (std_msgs/msg/String)
  - execute         (std_msgs/msg/Bool)
  - stop            (std_msgs/msg/Empty)
  - set_speed_mm_s  (std_msgs/msg/Float64)
  - set_planner_cfg (std_msgs/msg/String)

Extended:
  - execute_trajectory  (moveit_msgs/msg/RobotTrajectory)
  - optimize_trajectory (moveit_msgs/msg/RobotTrajectory)
  - set_segment         (std_msgs/msg/String)

Design:
  - publish trajectories BEFORE motion_result (race-free for statemachine)
  - CLEAR caches via traj_cache_clear BEFORE each external "move" call
    (plan_pose, plan_named, execute_trajectory, optimize_trajectory)
  - do NOT clear before "execute" (execute consumes the current planned)
  - publish robot_description(_semantic) ONCE after MoveItPy init (latched)
  - also best-effort re-publish robot_description on first plan request
    (covers UI starting late / transient-local mismatch)
"""

from __future__ import annotations

import json
import math
from threading import Thread, Event, Lock
from typing import Optional, Dict, Any, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from rclpy.action import ActionClient

from std_msgs.msg import String as MsgString, Bool as MsgBool, Empty as MsgEmpty, Float64 as MsgFloat64
from geometry_msgs.msg import PoseStamped

from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg, PlanningScene
from moveit_msgs.srv import GetPlanningScene

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from controller_manager_msgs.srv import SwitchController
from control_msgs.action import FollowJointTrajectory  # type: ignore
from trajectory_msgs.msg import JointTrajectoryPoint

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
    "max_velocity_scaling_factor": 0.10,
    "max_acceleration_scaling_factor": 0.10,
}

DEFAULT_TRAJ_CONTROLLER = "omron_arm_controller"


# ----------------------------
# Controller Mode Manager
# ----------------------------
class _ModeManager:
    """
    Minimal controller mode arbiter (embedded).
    Modes:
      - "JOG":  Trajectory controller OFF
      - "TRAJ": Trajectory controller ON
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
        import time as _time

        mode = (mode or "").strip().upper()
        if mode not in ("JOG", "TRAJ"):
            self.node.get_logger().warning(f"[ModeManager] unknown mode '{mode}'")
            return

        now = _time.time()
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
# MoveItPy Node
# ----------------------------
class MoveItPyNode(Node):
    """
    MoveItPy wrapper node (minimal topics).
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
        self.frame_scene = self.cfg_frames.resolve(self.cfg_frames.get("scene", "scene"))

        # parameters
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

        # IMPORTANT:
        # - If your MoveIt launch injects these params, they will already exist.
        # - We only declare defaults if missing (keeps override intact).
        if not self.has_parameter("robot_description"):
            self.declare_parameter("robot_description", "")
        if not self.has_parameter("robot_description_semantic"):
            self.declare_parameter("robot_description_semantic", "")

        # Optional parameter to override controller name (used for FollowJT + mode switch)
        if not self.has_parameter("traj_controller"):
            self.declare_parameter("traj_controller", DEFAULT_TRAJ_CONTROLLER)
        self.traj_controller = str(self.get_parameter("traj_controller").value or DEFAULT_TRAJ_CONTROLLER).strip()
        if not self.traj_controller:
            self.traj_controller = DEFAULT_TRAJ_CONTROLLER

        backend_lower = str(self.backend).lower()
        self.is_real_backend = ("omron" in backend_lower) and ("sim" not in backend_lower)

        ns = self.get_namespace() or "/"
        self.log.info(
            f"MoveItPyNode starting (backend='{self.backend}', is_real_backend={self.is_real_backend}, ns='{ns}')"
        )

        # controller mode manager
        self._mode_mgr = _ModeManager(self, traj_controller=self.traj_controller, debounce_s=0.25)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # motion config
        self._speed_mm_s: float = 100.0
        self._planner_cfg: Dict[str, Any] = DEFAULT_PLANNER_CFG.copy()
        self._plan_params: Optional[PlanRequestParameters] = None

        # state
        self._planned: Optional[RobotTrajectoryCore] = None
        self._busy: bool = False
        self._cancel: bool = False
        self._last_goal_pose: Optional[PoseStamped] = None

        # segment tag + external trajectory exec state
        self._current_segment: str = ""
        self._external_active_goal = None  # FollowJointTrajectory goal handle
        self._external_active_traj: Optional[RobotTrajectoryMsg] = None

        # Robot model publication guard (thread-safe)
        self._robot_model_lock = Lock()
        self._robot_model_published: bool = False

        # publishers (ONLY the minimal set)
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

        # NEW: cache-clear signal (NOT latched)
        self.pub_traj_cache_clear = self.create_publisher(
            MsgEmpty,
            self.cfg_topics.publish_topic(NODE_KEY, "traj_cache_clear"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "traj_cache_clear"),
        )

        # robot model strings for Offline-FK in GUI (latched via QoS)
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

        # omron cmd publisher (optional)
        self.pub_omron_cmd: Optional[Any] = None
        if self.is_real_backend:
            topic_omron_cmd = self.cfg_topics.publish_topic("omron", "command")
            qos_omron_cmd = self.cfg_topics.qos_by_id("publish", "omron", "command")
            self.pub_omron_cmd = self.create_publisher(MsgString, topic_omron_cmd, qos_omron_cmd)
            self.log.info(f"[{self.backend}] Omron cmd topic enabled: {topic_omron_cmd}")
        else:
            self.log.info(f"[{self.backend}] Omron cmd topic disabled (sim/shadow backend)")

        # FollowJointTrajectory ActionClients (namespaced + root fallback)
        self._followjt_clients: List[Tuple[str, ActionClient]] = []
        for action_name in self._followjt_action_candidates(self.traj_controller):
            self._followjt_clients.append((action_name, ActionClient(self, FollowJointTrajectory, action_name)))
        if self._followjt_clients:
            self.log.info("[followjt] action candidates: " + ", ".join(n for n, _ in self._followjt_clients))

        # subscribers
        self.create_subscription(
            PoseStamped,
            self.cfg_topics.subscribe_topic(NODE_KEY, "plan_pose"),
            self._on_plan_pose,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "plan_pose"),
        )
        self.create_subscription(
            MsgString,
            self.cfg_topics.subscribe_topic(NODE_KEY, "plan_named"),
            self._on_plan_named,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "plan_named"),
        )
        self.create_subscription(
            MsgBool,
            self.cfg_topics.subscribe_topic(NODE_KEY, "execute"),
            self._on_execute,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "execute"),
        )
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

        # execute arbitrary trajectory (from YAML / optimize output)
        try:
            self.create_subscription(
                RobotTrajectoryMsg,
                self.cfg_topics.subscribe_topic(NODE_KEY, "execute_trajectory"),
                self._on_execute_trajectory,
                self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "execute_trajectory"),
            )
            self.log.info("[topics] subscribed: execute_trajectory")
        except Exception as e:
            self.log.warning(f"[topics] execute_trajectory not wired (topics.yaml missing?): {e!r}")

        # optimize trajectory (post-processing / retime) in node
        try:
            self.create_subscription(
                RobotTrajectoryMsg,
                self.cfg_topics.subscribe_topic(NODE_KEY, "optimize_trajectory"),
                self._on_optimize_trajectory,
                self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "optimize_trajectory"),
            )
            self.log.info("[topics] subscribed: optimize_trajectory")
        except Exception as e:
            self.log.warning(f"[topics] optimize_trajectory not wired (topics.yaml missing?): {e!r}")

        # Optional segment tagging
        try:
            self.create_subscription(
                MsgString,
                self.cfg_topics.subscribe_topic(NODE_KEY, "set_segment"),
                self._on_set_segment,
                self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "set_segment"),
            )
            self.log.info("[topics] subscribed: set_segment")
        except Exception:
            pass

        # planning scene cache service (optional but useful)
        self._gps_srv = None
        self._gps_sub = None
        self._last_scene: Optional[PlanningScene] = None
        self._warned_no_scene = False
        self._ensure_get_planning_scene_cache_service()

        # MoveItPy init in background
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

        self.log.info("MoveItPyNode init done (MoveItPy is initializing in background).")

    # ------------------------------------------------------------
    # NEW: CLEAR helpers (explicit clear topic)
    # ------------------------------------------------------------
    def _clear_traj_cache(self, *, reason: str) -> None:
        """
        Emit an explicit cache-clear signal for GUI/Bridge/Statemachine.

        IMPORTANT:
          - This is NOT latched.
          - Do NOT publish empty RobotTrajectory messages to clear latched topics.
            (that breaks pairing logic if subscribers treat empty msgs as received.)
        """
        try:
            self.pub_traj_cache_clear.publish(MsgEmpty())
            self.log.info(f"[traj] cache clear signaled (reason='{reason}')")
        except Exception as e:
            self.log.warning(f"[traj] cache clear publish failed: {e!r}")

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
        out = []
        if ns:
            out.append(f"/{ns}/{c}/follow_joint_trajectory")
        out.append(f"/{c}/follow_joint_trajectory")

        seen = set()
        uniq = []
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
    # Robot model publication (URDF/SRDF)
    # ------------------------------------------------------------
    def _publish_robot_model_strings_once(self) -> None:
        """
        Publish robot_description and robot_description_semantic (latched) exactly once.
        """
        with self._robot_model_lock:
            if self._robot_model_published:
                return

            try:
                urdf = str(self.get_parameter("robot_description").value or "")
            except Exception:
                urdf = ""
            try:
                srdf = str(self.get_parameter("robot_description_semantic").value or "")
            except Exception:
                srdf = ""

            urdf = urdf.strip()
            srdf = srdf.strip()

            if not urdf:
                self.log.error(
                    "[robot_model] robot_description parameter ist leer. "
                    "Ohne URDF kann die GUI kein Offline-FK/TCP berechnen. "
                    "Fix: Stelle sicher, dass der MoveIt launch robot_description an /moveit_py injiziert."
                )
                return

            try:
                self.pub_robot_description.publish(MsgString(data=urdf))
                if srdf:
                    self.pub_robot_description_semantic.publish(MsgString(data=srdf))
                else:
                    self.log.warning("[robot_model] robot_description_semantic ist leer (SRDF fehlt/optional).")

                self._robot_model_published = True
                self.log.info(
                    f"[robot_model] published robot_description ({len(urdf)} chars) "
                    f"+ semantic ({len(srdf)} chars) [latched]"
                )
            except Exception as e:
                self.log.error(f"[robot_model] publish failed: {e!r}")

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

                try:
                    preset = self.plan_request_preset
                    self._plan_params = PlanRequestParameters(self.moveit, preset)
                    self._apply_planner_cfg_to_params()
                    self.log.info(f"[config] PlanRequestParameters loaded preset='{preset}'")
                except Exception as e:
                    self._plan_params = None
                    self.log.warning(
                        f"[config] PlanRequestParameters nicht verfügbar ({e}). Nutze arm.plan() ohne explizite Parameter."
                    )

            # Publish robot model strings AFTER MoveItPy init
            self._publish_robot_model_strings_once()

            self._moveit_ready.set()
            self.log.info(f"MoveItPy ready (group={GROUP_NAME}, ns='{self.get_namespace() or '/'}')")
            self.log.info(f"MoveItPyNode online (backend='{self.backend}').")

        except Exception as e:
            self._moveit_err = repr(e)
            self._moveit_failed.set()
            self.log.error(f"[moveitpy] INIT FAILED: {e!r}")

    def _require_moveit_ready(self) -> bool:
        if self._moveit_ready.is_set():
            return True
        if self._moveit_failed.is_set():
            self._emit(f"ERROR:MOVEIT_INIT_FAILED {self._moveit_err}")
            return False
        self._emit("ERROR:MOVEIT_NOT_READY")
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
    # Utility
    # ------------------------------------------------------------
    def _emit(self, text: str) -> None:
        # motion_result is the state-machine signal; keep it LAST per event.
        self.pub_result.publish(MsgString(data=text))
        self.log.info(text)

    def _emit_with_segment(self, text: str) -> None:
        seg = (self._current_segment or "").strip()
        if seg:
            self._emit(f"{text} seg={seg}")
        else:
            self._emit(text)

    def _apply_planner_cfg_to_params(self) -> None:
        if self._plan_params is None:
            return

        cfg = self._planner_cfg
        p = self._plan_params

        pipeline_id = (str(cfg.get("pipeline") or "").strip()) or "ompl"
        planner_id = (str(cfg.get("planner_id") or "").strip()) or "RRTConnectkConfigDefault"

        try:
            p.planning_pipeline = pipeline_id  # type: ignore[attr-defined]
        except Exception:
            self.log.warning("[config] PlanRequestParameters has no attribute 'planning_pipeline'")

        try:
            p.planner_id = planner_id  # type: ignore[attr-defined]
        except Exception:
            self.log.warning("[config] PlanRequestParameters has no attribute 'planner_id'")

        p.planning_time = float(cfg.get("planning_time", DEFAULT_PLANNER_CFG["planning_time"]))
        p.planning_attempts = int(cfg.get("planning_attempts", DEFAULT_PLANNER_CFG["planning_attempts"]))
        p.max_velocity_scaling_factor = float(
            cfg.get("max_velocity_scaling_factor", DEFAULT_PLANNER_CFG["max_velocity_scaling_factor"])
        )
        p.max_acceleration_scaling_factor = float(
            cfg.get("max_acceleration_scaling_factor", DEFAULT_PLANNER_CFG["max_acceleration_scaling_factor"])
        )

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

        except Exception as e:
            self.log.error(f"[config] set_planner_cfg: invalid JSON '{raw}': {e}")

    # ------------------------------------------------------------
    # segment tag input
    # ------------------------------------------------------------
    def _on_set_segment(self, msg: MsgString) -> None:
        self._current_segment = str(msg.data or "").strip()

    # ------------------------------------------------------------
    # TF helpers for goal input
    # ------------------------------------------------------------
    def _lookup_tf(self, target: str, source: str):
        if not self.tf_buffer.can_transform(target, source, RclpyTime(), Duration(seconds=1.0)):
            raise RuntimeError(f"TF not available {target} <- {source}")
        return self.tf_buffer.lookup_transform(target, source, RclpyTime())

    def _resolve_named_candidates(self, name: str) -> List[str]:
        name = (name or "").strip().strip("/")
        if not name:
            return []
        cand = [name]
        ns = self._ns_prefix()
        if ns:
            cand.append(f"{ns}/{name}")
        return cand

    def _lookup_tf_named(self, name: str):
        last_err: Optional[Exception] = None
        for cand in self._resolve_named_candidates(name):
            try:
                return self._lookup_tf(self.frame_world, cand), cand
            except Exception as e:
                last_err = e
        raise RuntimeError(str(last_err) if last_err else f"TF not available {self.frame_world} <- {name}")

    def _transform_pose_stamped_to_world(self, msg: PoseStamped) -> PoseStamped:
        in_frame = (msg.header.frame_id or "").strip() or self.frame_world

        if in_frame == self.frame_world:
            out = PoseStamped()
            out.header.frame_id = self.frame_world
            out.header.stamp = self.get_clock().now().to_msg()
            out.pose = msg.pose
            return out

        try:
            out = self.tf_buffer.transform(msg, self.frame_world, timeout=Duration(seconds=1.0))
            out.header.frame_id = self.frame_world
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        except Exception as e_primary:
            self.log.warning(f"[tf] buffer.transform failed ({in_frame} -> {self.frame_world}): {e_primary!r}")

        tf = self._lookup_tf(self.frame_world, in_frame)
        p = do_transform_pose(msg.pose, tf)
        out = PoseStamped()
        out.header.frame_id = self.frame_world
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose = p
        return out

    # ------------------------------------------------------------
    # Planning / Execution handlers
    # ------------------------------------------------------------
    def _on_plan_pose(self, msg: PoseStamped) -> None:
        if self._busy:
            self._emit_with_segment("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return

        # NEW: clear caches at boundary of new external move request
        self._clear_traj_cache(reason="plan_pose")

        # Ensure GUI can FK even if it started after node init
        self._publish_robot_model_strings_once()

        self._planned = None
        self._last_goal_pose = None

        in_frame = msg.header.frame_id or self.frame_world
        self.log.info(f"[plan_pose] in_frame={in_frame} → world={self.frame_world}")

        try:
            goal = self._transform_pose_stamped_to_world(msg)

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)

            if self._plan_params is not None:
                self._apply_planner_cfg_to_params()

            result = self.arm.plan(single_plan_parameters=self._plan_params) if self._plan_params else self.arm.plan()
            core = getattr(result, "trajectory", None)
            if core is None:
                self._emit_with_segment("ERROR:NO_TRAJ pose")
                return

            msg_traj = core.get_robot_trajectory_msg()
            self._planned = core
            self._last_goal_pose = goal

            # publish artifact BEFORE result
            self.pub_planned.publish(msg_traj)
            self._emit_with_segment("PLANNED:OK pose")

        except Exception as e:
            self._emit_with_segment(f"ERROR:EX {e}")

    def _on_plan_named(self, msg: MsgString) -> None:
        if self._busy:
            self._emit_with_segment("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return

        # NEW: clear caches at boundary of new external move request
        self._clear_traj_cache(reason="plan_named")

        # Ensure GUI can FK even if it started after node init
        self._publish_robot_model_strings_once()

        name = (msg.data or "").strip()
        if not name:
            self._emit_with_segment("ERROR:EMPTY_NAME")
            return

        self._planned = None
        self._last_goal_pose = None

        try:
            tf, resolved = self._lookup_tf_named(name)
            if resolved != name:
                self.log.info(f"[plan_named] resolved '{name}' -> '{resolved}'")

            goal = PoseStamped()
            goal.header.frame_id = self.frame_world
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = tf.transform.translation.x
            goal.pose.position.y = tf.transform.translation.y
            goal.pose.position.z = tf.transform.translation.z
            goal.pose.orientation = tf.transform.rotation

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)

            if self._plan_params is not None:
                self._apply_planner_cfg_to_params()

            result = self.arm.plan(single_plan_parameters=self._plan_params) if self._plan_params else self.arm.plan()
            core = getattr(result, "trajectory", None)
            if core is None:
                self._emit_with_segment("ERROR:NO_TRAJ named")
                return

            msg_traj = core.get_robot_trajectory_msg()
            self._planned = core
            self._last_goal_pose = goal

            # publish artifact BEFORE result
            self.pub_planned.publish(msg_traj)
            self._emit_with_segment(f"PLANNED:OK named='{name}'")

        except Exception as e:
            self._emit_with_segment(f"ERROR:EX {e}")

    def _on_execute(self, msg: MsgBool) -> None:
        if not msg.data:
            self._on_stop(MsgEmpty())
            return

        # IMPORTANT: do NOT clear here. execute consumes the current planned.
        self._mode_mgr.ensure_mode("TRAJ")

        if self._busy:
            self._emit_with_segment("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return
        if not self._planned:
            self._emit_with_segment("ERROR:NO_PLAN")
            return

        self._busy = True
        self._cancel = False

        try:
            msg_traj = self._planned.get_robot_trajectory_msg()

            if self.is_real_backend:
                ok = self._execute_via_omron(msg_traj)
            else:
                ok = self.moveit.execute(self._planned, controllers=[])  # type: ignore[union-attr]

            if self._cancel:
                self._emit_with_segment("STOPPED:USER")
                return

            if ok:
                # publish executed artifact BEFORE result
                self.pub_executed.publish(msg_traj)
                self._emit_with_segment("EXECUTED:OK")
            else:
                self._emit_with_segment("ERROR:EXEC")

        except Exception as e:
            self._emit_with_segment(f"ERROR:EX {e}")

        finally:
            self._busy = False

    # ------------------------------------------------------------
    # Execute arbitrary RobotTrajectory (from YAML / optimize output)
    # ------------------------------------------------------------
    def _on_execute_trajectory(self, msg: RobotTrajectoryMsg) -> None:
        if self._busy:
            self._emit_with_segment("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return

        # NEW: clear caches at boundary of new external move request
        self._clear_traj_cache(reason="execute_trajectory")

        jt = getattr(msg, "joint_trajectory", None)
        if jt is None or not getattr(jt, "joint_names", None) or not getattr(jt, "points", None):
            self._emit_with_segment("ERROR:EMPTY_TRAJ")
            return

        # ensure controller enabled
        self._mode_mgr.ensure_mode("TRAJ")

        self._busy = True
        self._cancel = False
        self._external_active_goal = None
        self._external_active_traj = msg

        # publish artifact BEFORE result (treat incoming as planned artifact)
        try:
            self.pub_planned.publish(msg)
        except Exception:
            pass

        picked = self._pick_ready_followjt()
        if picked is None:
            for _name, cli in self._followjt_clients:
                try:
                    cli.wait_for_server(timeout_sec=0.0)
                except Exception:
                    pass
            picked = self._pick_ready_followjt()

        if picked is None:
            self._external_active_goal = None
            self._external_active_traj = None
            self._busy = False
            self._emit_with_segment("ERROR:NO_FOLLOWJT_SERVER")
            return

        action_name, client = picked
        self.log.info(f"[followjt] using server: {action_name}")

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = jt

        send_fut = client.send_goal_async(goal)

        def _on_goal_sent(fut):
            try:
                gh = fut.result()
            except Exception as e:
                self._external_active_goal = None
                self._external_active_traj = None
                self._busy = False
                self._emit_with_segment(f"ERROR:FOLLOWJT_SEND {e}")
                return

            if gh is None or not getattr(gh, "accepted", False):
                self._external_active_goal = None
                self._external_active_traj = None
                self._busy = False
                self._emit_with_segment("ERROR:GOAL_REJECTED")
                return

            self._external_active_goal = gh

            if self._cancel:
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

                    if self._cancel:
                        self._emit_with_segment("STOPPED:USER")
                        return

                    if code == 0:
                        try:
                            if self._external_active_traj is not None:
                                self.pub_executed.publish(self._external_active_traj)
                        except Exception:
                            pass
                        self._emit_with_segment("EXECUTED:OK")
                    else:
                        self._emit_with_segment(f"ERROR:EXEC code={code} status={status}")

                except Exception as e:
                    self._emit_with_segment(f"ERROR:FOLLOWJT_RESULT {e}")

                finally:
                    self._external_active_goal = None
                    self._external_active_traj = None
                    self._busy = False

            res_fut.add_done_callback(_on_result_done)

        send_fut.add_done_callback(_on_goal_sent)

    # ------------------------------------------------------------
    # Optimize / retime RobotTrajectory (post-processing in node)
    # ------------------------------------------------------------
    def _on_optimize_trajectory(self, msg: RobotTrajectoryMsg) -> None:
        if self._busy:
            self._emit_with_segment("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return

        # NEW: clear caches at boundary of new external move request
        self._clear_traj_cache(reason="optimize_trajectory")

        jt = getattr(msg, "joint_trajectory", None)
        if jt is None or not getattr(jt, "joint_names", None) or not getattr(jt, "points", None):
            self._emit_with_segment("ERROR:EMPTY_TRAJ")
            return
        if len(jt.points) < 2:
            self._emit_with_segment("ERROR:TRAJ_TOO_SHORT")
            return

        self._busy = True
        self._cancel = False
        try:
            pipeline = str(self._planner_cfg.get("pipeline") or "post").strip() or "post"
            planner_id = str(self._planner_cfg.get("planner_id") or "IterativeParabolicTimeParameterization").strip()

            out_msg = self._optimize_robot_trajectory(msg, pipeline=pipeline, planner_id=planner_id)

            # publish artifact BEFORE result
            try:
                self.pub_optimized.publish(out_msg)
            except Exception:
                pass

            self._emit_with_segment("OPTIMIZED:OK")

        except Exception as e:
            self._emit_with_segment(f"ERROR:OPTIMIZE {e}")
        finally:
            self._busy = False

    def _optimize_robot_trajectory(
        self,
        msg: RobotTrajectoryMsg,
        *,
        pipeline: str,
        planner_id: str,
    ) -> RobotTrajectoryMsg:
        # placeholder until real MoveIt time-parameterization binding is wired in python
        try:
            raise RuntimeError("MoveIt post-processing Python binding not wired")
        except Exception as e:
            self.log.warning(f"[opt] using fallback retime (no post binding): {e!r}")
            return self._optimize_fallback_scale(msg, planner_id=planner_id)

    def _optimize_fallback_scale(self, msg: RobotTrajectoryMsg, *, planner_id: str) -> RobotTrajectoryMsg:
        try:
            vs = float(self._planner_cfg.get("max_velocity_scaling_factor", 1.0))
        except Exception:
            vs = 1.0
        if vs <= 1e-6:
            vs = 1.0

        # Interpret scaling as "allowed speed fraction" => smaller means slower => larger times
        scale = 1.0 / vs

        out = RobotTrajectoryMsg()
        out.joint_trajectory.joint_names = list(msg.joint_trajectory.joint_names)

        last_ns = -1
        for p in msg.joint_trajectory.points:
            sec = int(getattr(p.time_from_start, "sec", 0))
            nsec = int(getattr(p.time_from_start, "nanosec", 0))
            ns = sec * 1_000_000_000 + nsec

            ns2 = int(float(ns) * float(scale))
            if ns2 <= last_ns:
                ns2 = last_ns + 1_000_000  # +1ms
            last_ns = ns2

            q = JointTrajectoryPoint()
            q.positions = list(p.positions)

            try:
                if getattr(p, "velocities", None):
                    q.velocities = list(p.velocities)
            except Exception:
                pass
            try:
                if getattr(p, "accelerations", None):
                    q.accelerations = list(p.accelerations)
            except Exception:
                pass
            try:
                if getattr(p, "effort", None):
                    q.effort = list(p.effort)
            except Exception:
                pass

            q.time_from_start.sec = ns2 // 1_000_000_000
            q.time_from_start.nanosec = ns2 % 1_000_000_000
            out.joint_trajectory.points.append(q)

        return out

    # ------------------------------------------------------------
    # Omron TCP execution (MOVEJ last point)
    # ------------------------------------------------------------
    def _execute_via_omron(self, traj_msg: RobotTrajectoryMsg) -> bool:
        if not self.is_real_backend or self.pub_omron_cmd is None:
            self.log.error("[omron] _execute_via_omron called but omron publisher is disabled (not real backend).")
            return False

        jt = traj_msg.joint_trajectory
        if not jt.points:
            self.log.error("[omron] Trajektorie hat keine Punkte.")
            return False

        last = jt.points[-1]
        positions = list(last.positions)

        if len(positions) < 6:
            self.log.error(f"[omron] Zu wenige Joints in Trajektorie (positions={len(positions)})")
            return False

        j_rad = positions[:6]
        j_deg = [p * 180.0 / math.pi for p in j_rad]

        v_mm_s = float(self._speed_mm_s) if self._speed_mm_s > 0 else 50.0
        max_tcp = float(self.max_tcp_speed_mm_s) if self.max_tcp_speed_mm_s > 0 else 300.0
        speed_pct = 100.0 * v_mm_s / max_tcp
        speed_pct = max(5.0, min(100.0, speed_pct))

        cmd = "MOVEJ " + " ".join(f"{a:.3f}" for a in j_deg) + f" {speed_pct:.1f}"
        self.log.info(f"[omron] MOVEJ via TCP: {cmd}")
        self.pub_omron_cmd.publish(MsgString(data=cmd))
        return True

    # ------------------------------------------------------------
    # Stop / Cancel
    # ------------------------------------------------------------
    def _on_stop(self, _msg: MsgEmpty) -> None:
        self._cancel = True

        gh = self._external_active_goal
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception as e:
                self.log.warning(f"[stop] cancel_goal_async failed: {e!r}")

        self._emit_with_segment("STOP:REQ")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItPyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
