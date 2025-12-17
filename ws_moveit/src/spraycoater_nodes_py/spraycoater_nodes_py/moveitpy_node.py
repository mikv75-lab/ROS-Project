#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import json
import math
from threading import Thread, Event, Lock
from typing import Optional, Dict, Any, List

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String as MsgString, Bool as MsgBool, Empty as MsgEmpty, Float64 as MsgFloat64
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray

from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg, PlanningScene
from moveit_msgs.srv import GetPlanningScene

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from spraycoater_nodes_py.utils.config_hub import topics, frames

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


class MoveItPyNode(Node):
    """
    Wrapper-Node (rclpy), der MoveItPy initialisiert.

    Fixes (nur 2 + 3):
      (2) get_planning_scene Cache-Service SOFORT bereitstellen (RViz-safe)
      (3) MoveItPy init im Background-Thread (damit rclpy sofort spinnen kann)
    """

    def __init__(self) -> None:
        super().__init__(
            "moveit_py",  # ❗ bleibt so (topics hängen an moveit_py)
            automatically_declare_parameters_from_overrides=True,
        )
        self.log = self.get_logger()

        self.cfg_topics = topics()
        self.cfg_frames = frames()

        self.frame_world = self.cfg_frames.resolve(self.cfg_frames.get("world", WORLD_FRAME))
        self.frame_scene = self.cfg_frames.resolve(self.cfg_frames.get("scene", "scene"))

        # -------------------------------------------------
        # App-Params (werden im bringup gesetzt)
        # -------------------------------------------------
        if not self.has_parameter("backend"):
            self.declare_parameter("backend", "default")
        self.backend = self.get_parameter("backend").value or "default"

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)

        if not self.has_parameter("max_tcp_speed_mm_s"):
            self.declare_parameter("max_tcp_speed_mm_s", 300.0)
        self.max_tcp_speed_mm_s = float(self.get_parameter("max_tcp_speed_mm_s").value)

        backend_lower = str(self.backend).lower()
        self.is_real_backend = ("omron" in backend_lower) and ("sim" not in backend_lower)

        ns = self.get_namespace() or "/"
        self.log.info(
            f"MoveItPyMotionNode starting (backend='{self.backend}', is_real_backend={self.is_real_backend}, ns='{ns}')"
        )

        # --- TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # ---------------- intern: Motion-Config (Speed + Planner) ----------------
        self._speed_mm_s: float = 100.0
        self._planner_cfg: Dict[str, Any] = DEFAULT_PLANNER_CFG.copy()
        self._plan_params: Optional[PlanRequestParameters] = None

        # state
        self._planned: Optional[RobotTrajectoryCore] = None
        self._busy = False
        self._cancel = False
        self._last_goal_pose: Optional[PoseStamped] = None

        # ---------------- Publishers (alle Topics/QoS via config_hub) ----------------
        self.pub_target_traj = self.create_publisher(
            JointTrajectory,
            self.cfg_topics.publish_topic(NODE_KEY, "target_trajectory"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "target_trajectory"),
        )

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

        self.pub_preview = self.create_publisher(
            MarkerArray,
            self.cfg_topics.publish_topic(NODE_KEY, "preview_markers"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "preview_markers"),
        )

        self.pub_result = self.create_publisher(
            MsgString,
            self.cfg_topics.publish_topic(NODE_KEY, "motion_result"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "motion_result"),
        )

        # ✅ Option: Omron Topics nur im REAL-Backend erzeugen
        self.pub_omron_cmd: Optional[Any] = None
        self._topic_omron_cmd: Optional[str] = None

        if self.is_real_backend:
            topic_omron_cmd = self.cfg_topics.publish_topic("omron", "command")
            qos_omron_cmd = self.cfg_topics.qos_by_id("publish", "omron", "command")
            self.pub_omron_cmd = self.create_publisher(MsgString, topic_omron_cmd, qos_omron_cmd)
            self._topic_omron_cmd = topic_omron_cmd
            self.log.info(f"[{self.backend}] Omron cmd topic enabled: {topic_omron_cmd}")
        else:
            self.log.info(f"[{self.backend}] Omron cmd topic disabled (sim/shadow backend)")

        # --------------- Subscribers ----------------
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

        # ----------------------------------------------------------
        # (2) get_planning_scene Cache-Service SOFORT bereitstellen
        # ----------------------------------------------------------
        self._gps_srv = None
        self._gps_sub = None
        self._last_scene: Optional[PlanningScene] = None
        self._warned_no_scene = False
        self._ensure_get_planning_scene_cache_service()

        # ----------------------------------------------------------
        # (3) MoveItPy init im Background-Thread
        # ----------------------------------------------------------
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

    # ----------------------------------------------------------
    # MoveItPy background init
    # ----------------------------------------------------------
    def _init_moveitpy_background(self) -> None:
        try:
            with self._moveit_lock:
                self.moveit = MoveItPy(
                    node_name=self.get_name(),   # bleibt "moveit_py" (Parameter kommen so rein wie bisher)
                    name_space=self._name_space,
                    provide_planning_service=True,
                )

                self.arm = self.moveit.get_planning_component(GROUP_NAME)
                self.robot_model = self.moveit.get_robot_model()

                try:
                    self._plan_params = PlanRequestParameters(self.moveit, "plan_request_params")
                    self._apply_planner_cfg_to_params()
                except Exception as e:
                    self._plan_params = None
                    self.log.warning(
                        f"[config] PlanRequestParameters nicht verfügbar ({e}). Nutze arm.plan() ohne explizite Parameter."
                    )

            self._moveit_ready.set()
            self.log.info(f"MoveItPy ready (group={GROUP_NAME}, ns='{self.get_namespace() or '/'}')")
            self.log.info(f"MoveItPyMotionNode online (backend='{self.backend}').")

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

    # ----------------------------------------------------------
    # (2) Cache get_planning_scene Service (RViz-safe)
    # ----------------------------------------------------------
    def _ensure_get_planning_scene_cache_service(self) -> None:
        if self._gps_srv is not None:
            return

        ns = self.get_namespace() or "/"
        srv_name = f"{ns.rstrip('/')}/get_planning_scene" if ns != "/" else "/get_planning_scene"

        qos_scene = QoSProfile(depth=1)
        qos_scene.history = HistoryPolicy.KEEP_LAST
        qos_scene.reliability = ReliabilityPolicy.RELIABLE
        qos_scene.durability = DurabilityPolicy.VOLATILE  # maximal kompatibel

        # relative -> /shadow/monitored_planning_scene
        self._gps_sub = self.create_subscription(
            PlanningScene,
            "monitored_planning_scene",
            self._on_monitored_planning_scene,
            qos_scene,
        )

        # relative -> /shadow/get_planning_scene
        self._gps_srv = self.create_service(
            GetPlanningScene,
            "get_planning_scene",
            self._on_get_planning_scene,
        )

        self.log.info(f"[psm] ✅ Cache get_planning_scene Service aktiv: {srv_name}")

    def _on_monitored_planning_scene(self, msg: PlanningScene) -> None:
        self._last_scene = msg

    def _on_get_planning_scene(
        self,
        _req: GetPlanningScene.Request,
        res: GetPlanningScene.Response,
    ) -> GetPlanningScene.Response:
        # Muss schnell sein (RViz ruft früh auf).
        if self._last_scene is not None:
            res.scene = self._last_scene
            res.scene.is_diff = False
            return res

        if not self._warned_no_scene:
            self._warned_no_scene = True
            self.log.warn("[psm] get_planning_scene requested, aber noch kein monitored_planning_scene empfangen.")
        res.scene = PlanningScene()
        res.scene.is_diff = False
        return res

    # ----------------------------------------------------------
    # TF Helper
    # ----------------------------------------------------------
    def _lookup_tf(self, target: str, source: str):
        if not self.tf_buffer.can_transform(target, source, RclpyTime(), Duration(seconds=1.0)):
            raise RuntimeError(f"TF not available {target} <- {source}")
        return self.tf_buffer.lookup_transform(target, source, RclpyTime())

    def _ns_prefix(self) -> str:
        ns = (self.get_namespace() or "/").strip().strip("/")
        return ns

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

    def _emit(self, text: str) -> None:
        self.pub_result.publish(MsgString(data=text))
        self.log.info(text)

    # ----------------------------------------------------------
    # Planner-Config → PlanRequestParameters
    # ----------------------------------------------------------
    def _apply_planner_cfg_to_params(self) -> None:
        if self._plan_params is None:
            return

        cfg = self._planner_cfg
        p = self._plan_params

        planner_id = (cfg.get("planner_id") or DEFAULT_PLANNER_CFG["planner_id"]).strip()
        try:
            p.planner_id = planner_id  # type: ignore[attr-defined]
        except Exception:
            pass

        p.planning_time = float(cfg.get("planning_time", DEFAULT_PLANNER_CFG["planning_time"]))
        p.planning_attempts = int(cfg.get("planning_attempts", DEFAULT_PLANNER_CFG["planning_attempts"]))
        p.max_velocity_scaling_factor = float(
            cfg.get("max_velocity_scaling_factor", DEFAULT_PLANNER_CFG["max_velocity_scaling_factor"])
        )
        p.max_acceleration_scaling_factor = float(
            cfg.get("max_acceleration_scaling_factor", DEFAULT_PLANNER_CFG["max_acceleration_scaling_factor"])
        )

        self.log.info(
            f"[config] Planner applied: planner_id='{planner_id}', "
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
            cfg = json.loads(raw)
            if not isinstance(cfg, dict):
                raise ValueError("planner_cfg JSON ist kein dict.")
            self._planner_cfg.update(cfg)
            # ggf. MoveIt schon ready -> direkt anwenden
            if self._moveit_ready.is_set():
                self._apply_planner_cfg_to_params()
        except Exception as e:
            self.log.error(f"[config] set_planner_cfg: invalid JSON '{raw}': {e}")

    # ----------------------------------------------------------
    # PLAN POSE
    # ----------------------------------------------------------
    def _on_plan_pose(self, msg: PoseStamped) -> None:
        if self._busy:
            self._emit("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return

        self._planned = None
        self._last_goal_pose = None

        in_frame = msg.header.frame_id or self.frame_world
        self.log.info(f"[plan_pose] in_frame={in_frame} → world={self.frame_world}")

        try:
            if in_frame != self.frame_world:
                tf = self._lookup_tf(self.frame_world, in_frame)
                goal = do_transform_pose(msg, tf)
                goal.header.frame_id = self.frame_world
                goal.header.stamp = self.get_clock().now().to_msg()
            else:
                goal = PoseStamped()
                goal.header.frame_id = self.frame_world
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose = msg.pose

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)

            result = self.arm.plan(single_plan_parameters=self._plan_params) if self._plan_params else self.arm.plan()
            core = getattr(result, "trajectory", None)
            if core is None:
                self._emit("ERROR:NO_TRAJ pose")
                return

            msg_traj = core.get_robot_trajectory_msg()
            self._planned = core
            self._last_goal_pose = goal

            self.pub_planned.publish(msg_traj)
            self._publish_preview(goal)
            self._emit("PLANNED:OK pose")

        except Exception as e:
            self._emit(f"ERROR:EX {e}")

    # ----------------------------------------------------------
    # PLAN NAMED (TF-Frame name)
    # ----------------------------------------------------------
    def _on_plan_named(self, msg: MsgString) -> None:
        if self._busy:
            self._emit("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return

        name = (msg.data or "").strip()
        if not name:
            self._emit("ERROR:EMPTY_NAME")
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

            result = self.arm.plan(single_plan_parameters=self._plan_params) if self._plan_params else self.arm.plan()
            core = getattr(result, "trajectory", None)
            if core is None:
                self._emit("ERROR:NO_TRAJ named")
                return

            msg_traj = core.get_robot_trajectory_msg()
            self._planned = core
            self._last_goal_pose = goal

            self.pub_planned.publish(msg_traj)
            self._publish_preview(goal)
            self._emit(f"PLANNED:OK named='{name}'")

        except Exception as e:
            self._emit(f"ERROR:EX {e}")

    # ----------------------------------------------------------
    # EXECUTE – SIM via MoveIt / REAL via Omron MOVEJ
    # ----------------------------------------------------------
    def _on_execute(self, msg: MsgBool) -> None:
        if not msg.data:
            self._on_stop(MsgEmpty())
            return

        if self._busy:
            self._emit("ERROR:BUSY")
            return
        if not self._require_moveit_ready():
            return
        if not self._planned:
            self._emit("ERROR:NO_PLAN")
            return

        self._busy = True
        self._cancel = False

        try:
            msg_traj = self._planned.get_robot_trajectory_msg()

            if self.is_real_backend:
                ok = self._execute_via_omron(msg_traj)
            else:
                # MoveItPy execute
                ok = self.moveit.execute(self._planned, controllers=[])  # type: ignore[union-attr]

            if self._cancel:
                self._emit("STOPPED:USER")
                return

            if ok:
                self.pub_executed.publish(msg_traj)
                self._emit("EXECUTED:OK")
            else:
                self._emit("ERROR:EXEC")

        except Exception as e:
            self._emit(f"ERROR:EX {e}")

        finally:
            self._busy = False

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

    def _on_stop(self, _msg: MsgEmpty) -> None:
        self._cancel = True
        self._emit("STOP:REQ")

    def _publish_preview(self, goal: PoseStamped) -> None:
        m = Marker()
        m.header.frame_id = goal.header.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = goal.pose

        if (
            m.pose.orientation.x == 0.0 and
            m.pose.orientation.y == 0.0 and
            m.pose.orientation.z == 0.0 and
            m.pose.orientation.w == 0.0
        ):
            m.pose.orientation.w = 1.0

        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 1.0
        self.pub_preview.publish(MarkerArray(markers=[m]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MoveItPyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
