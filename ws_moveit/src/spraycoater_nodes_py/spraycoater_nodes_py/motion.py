#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/motion.py

from __future__ import annotations

import sys
import time
import json
from typing import Optional, Dict, Any
import os
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from std_msgs.msg import String as MsgString, Bool as MsgBool, Empty as MsgEmpty, Float64 as MsgFloat64
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from ament_index_python.packages import get_package_share_directory

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY    = "motion"
GROUP_NAME  = "omron_arm_group"
EE_LINK     = "tcp"
WORLD_FRAME = "world"

# ----------------- Planner Defaults (noch im Code) -----------------

DEFAULT_PLANNER_CFG: Dict[str, Any] = {
    "pipeline": "ompl",
    "planner_id": "RRTConnectkConfigDefault",
    "planning_time": 10.0,
    "planning_attempts": 100,
    "max_velocity_scaling_factor": 0.10,
    "max_acceleration_scaling_factor": 0.10,
}


class Motion(Node):
    def __init__(self) -> None:
        super().__init__("motion")
        self.log = self.get_logger()

        self.cfg_topics = topics()
        self.cfg_frames = frames()

        self.frame_world = self.cfg_frames.resolve(self.cfg_frames.get("world", WORLD_FRAME))
        self.frame_scene = self.cfg_frames.resolve(self.cfg_frames.get("scene", "scene"))

        # Backend-String (z.B. 'default' / 'omron_sim' / 'omron_real')
        self.backend = (
            self.declare_parameter("backend", "default")
            .get_parameter_value()
            .string_value
            or "default"
        )
        backend_lower = self.backend.lower()
        self.is_real_backend = ("omron" in backend_lower) and ("sim" not in backend_lower)

        # maximale TCP-Geschwindigkeit in mm/s, die 100 % entsprechen soll
        self.max_tcp_speed_mm_s = (
            self.declare_parameter("max_tcp_speed_mm_s", 300.0)
            .get_parameter_value()
            .double_value
        )

        self.log.info(
            f"MotionNode starting (backend='{self.backend}', "
            f"is_real_backend={self.is_real_backend}, "
            f"max_tcp_speed_mm_s={self.max_tcp_speed_mm_s})"
        )

        # --- TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # ------------------------------------------------------------------
        # MoveIt-Konfiguration:
        #  - identisch zur robot_sim.launch.py / move_group-Konfiguration
        # ------------------------------------------------------------------
        cfg_pkg = get_package_share_directory("omron_moveit_config")
        launch_dir = os.path.join(cfg_pkg, "launch")
        if launch_dir not in sys.path:
            sys.path.insert(0, launch_dir)

        from moveit_common import create_omron_moveit_config  # type: ignore

        moveit_cfg = create_omron_moveit_config()
        cfg_dict = moveit_cfg.to_dict()

        # ------------------------------------------------------------------
        # ✅ KRITISCHER FIX:
        # MoveItPy API: name_space (nicht namespace / node_namespace)
        # - Node läuft z.B. unter "/shadow"
        # - MoveItPy erwartet typischerweise "" oder "shadow" (ohne führenden '/')
        # ------------------------------------------------------------------
        ns = self.get_namespace() or "/"
        name_space = "" if ns == "/" else ns.strip("/")
        self.log.info(f"[moveitpy] init: node_ns='{ns}', name_space='{name_space}'")

        self.robot = MoveItPy(
            node_name=f"{self.get_name()}_moveit",
            name_space=name_space,           # ✅ korrekt
            config_dict=cfg_dict,
            provide_planning_service=True,   # ✅ stellt /<ns>/get_planning_scene bereit
        )

        time.sleep(0.5)

        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.log.info(f"MoveItPy ready (group={GROUP_NAME}, backend='{self.backend}', ns='{ns}')")

        # ---------------- intern: Motion-Config (Speed + Planner) ----------------
        self._speed_mm_s: float = 100.0
        self._planner_cfg: Dict[str, Any] = DEFAULT_PLANNER_CFG.copy()
        self._allowed_start_tolerance: float = 0.01

        self._plan_params: Optional[PlanRequestParameters] = None
        try:
            self._plan_params = PlanRequestParameters(self.robot, "plan_request_params")
            self._apply_planner_cfg_to_params()
        except Exception as e:
            self._plan_params = None
            self.log.warning(
                f"[config] PlanRequestParameters konnte nicht initialisiert werden: {e}. "
                "Nutze arm.plan() ohne explizite Parameter."
            )

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

        # ✅ Omron-Command-Publisher (REAL) – NICHT hardcoded, aus topics.yaml
        topic_omron_cmd = self.cfg_topics.publish_topic("omron", "command")
        qos_omron_cmd = self.cfg_topics.qos_by_id("publish", "omron", "command")
        self.pub_omron_cmd = self.create_publisher(MsgString, topic_omron_cmd, qos_omron_cmd)
        self.log.info(f"[{self.backend}] Omron cmd topic: {topic_omron_cmd}")

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

        # state
        self._planned: Optional[RobotTrajectoryCore] = None
        self._busy = False
        self._cancel = False
        self._last_goal_pose: Optional[PoseStamped] = None

        self.log.info(f"MotionNode online (backend='{self.backend}').")

    # ----------------------------------------------------------
    # TF Helper
    # ----------------------------------------------------------
    def _lookup_tf(self, target: str, source: str):
        if not self.tf_buffer.can_transform(target, source, RclpyTime(), Duration(seconds=1.0)):
            raise RuntimeError(f"TF not available {target} <- {source}")
        return self.tf_buffer.lookup_transform(target, source, RclpyTime())

    # ----------------------------------------------------------
    # emit helper
    # ----------------------------------------------------------
    def _emit(self, text: str) -> None:
        self.pub_result.publish(MsgString(data=text))
        self.log.info(text)

    # ----------------------------------------------------------
    # CONFIG: Planner-Defaults in PlanRequestParameters schreiben
    # ----------------------------------------------------------
    def _apply_planner_cfg_to_params(self) -> None:
        if self._plan_params is None:
            return

        cfg = self._planner_cfg
        p = self._plan_params

        pipeline = (cfg.get("pipeline") or DEFAULT_PLANNER_CFG["pipeline"]).strip()
        planner_id = (cfg.get("planner_id") or DEFAULT_PLANNER_CFG["planner_id"]).strip()

        if hasattr(self.robot, "set_pipelines"):
            try:
                self.robot.set_pipelines([pipeline])  # type: ignore[attr-defined]
            except Exception as e:
                self.log.warning(f"[config] set_pipelines('{pipeline}') failed: {e}")
        else:
            self.log.debug("[config] MoveItPy hat keine set_pipelines()-Methode – Pipeline kommt aus YAML-Config.")

        try:
            p.planner_id = planner_id  # type: ignore[attr-defined]
        except AttributeError:
            self.log.warning(
                "[config] PlanRequestParameters hat kein Attribut 'planner_id'; "
                "Planner-ID wird nur über die Pipeline-Konfiguration gewählt."
            )

        p.planning_time = float(cfg.get("planning_time", DEFAULT_PLANNER_CFG["planning_time"]))
        p.planning_attempts = int(cfg.get("planning_attempts", DEFAULT_PLANNER_CFG["planning_attempts"]))
        p.max_velocity_scaling_factor = float(
            cfg.get("max_velocity_scaling_factor", DEFAULT_PLANNER_CFG["max_velocity_scaling_factor"])
        )
        p.max_acceleration_scaling_factor = float(
            cfg.get("max_acceleration_scaling_factor", DEFAULT_PLANNER_CFG["max_acceleration_scaling_factor"])
        )

        self.log.info(
            f"[config] Planner applied: pipeline='{pipeline}', "
            f"planner_id='{getattr(p, 'planner_id', planner_id)}', "
            f"time={p.planning_time:.2f}s, "
            f"attempts={p.planning_attempts}, "
            f"vel_scale={p.max_velocity_scaling_factor:.2f}, "
            f"acc_scale={p.max_acceleration_scaling_factor:.2f}"
        )

    # ----------------------------------------------------------
    # CONFIG: Speed + Planner (Topic-Callbacks)
    # ----------------------------------------------------------
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
                raise ValueError("planner_cfg JSON ist kein Objekt (dict).")
            self._planner_cfg.update(cfg)
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

        self._planned = None
        self._last_goal_pose = None

        in_frame = msg.header.frame_id or self.frame_world
        self.log.info(f"[plan_pose] in_frame={in_frame} → world={self.frame_world}")

        try:
            if in_frame != self.frame_world:
                tf = self._lookup_tf(self.frame_world, in_frame)
                goal = do_transform_pose(msg, tf)  # PoseStamped -> PoseStamped
                goal.header.frame_id = self.frame_world
                goal.header.stamp = self.get_clock().now().to_msg()
            else:
                goal = PoseStamped()
                goal.header.frame_id = self.frame_world
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.pose = msg.pose

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)

            if self._plan_params is not None:
                result = self.arm.plan(single_plan_parameters=self._plan_params)
            else:
                result = self.arm.plan()

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
    # PLAN NAMED (home/service via TF-Frame)
    # ----------------------------------------------------------
    def _on_plan_named(self, msg: MsgString) -> None:
        if self._busy:
            self._emit("ERROR:BUSY")
            return

        name = (msg.data or "").strip()
        if not name:
            self._emit("ERROR:EMPTY_NAME")
            return

        self._planned = None
        self._last_goal_pose = None

        try:
            tf = self._lookup_tf(self.frame_world, name)

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
                result = self.arm.plan(single_plan_parameters=self._plan_params)
            else:
                result = self.arm.plan()

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
    # EXECUTE – je nach Backend: MoveIt-Exec (SIM) oder Omron MOVEJ
    # ----------------------------------------------------------
    def _on_execute(self, msg: MsgBool) -> None:
        if not msg.data:
            self._on_stop(MsgEmpty())
            return

        if self._busy:
            self._emit("ERROR:BUSY")
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
                ok = self.robot.execute(self._planned, controllers=[])

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

    # ----------------------------------------------------------
    # REAL-Backend: Trajektorie via Omron MOVEJ
    # ----------------------------------------------------------
    def _execute_via_omron(self, traj_msg: RobotTrajectoryMsg) -> bool:
        jt = traj_msg.joint_trajectory
        if not jt.points:
            self.log.error("[omron] Trajektorie hat keine Punkte.")
            return False

        last = jt.points[-1]
        names = list(jt.joint_names)
        positions = list(last.positions)

        if len(names) < 6 or len(positions) < 6:
            self.log.error(
                f"[omron] Zu wenige Joints in Trajektorie (names={len(names)}, positions={len(positions)})"
            )
            return False

        j_rad = positions[:6]
        j_deg = [p * 180.0 / math.pi for p in j_rad]

        v_mm_s = float(self._speed_mm_s)
        if v_mm_s <= 0.0:
            v_mm_s = 50.0

        max_tcp = float(self.max_tcp_speed_mm_s) if self.max_tcp_speed_mm_s > 0.0 else 300.0
        speed_pct = 100.0 * v_mm_s / max_tcp
        speed_pct = max(5.0, min(100.0, speed_pct))

        cmd = "MOVEJ " + " ".join(f"{a:.3f}" for a in j_deg) + f" {speed_pct:.1f}"
        self.log.info(f"[omron] MOVEJ via TCP: {cmd}")
        self.pub_omron_cmd.publish(MsgString(data=cmd))
        return True

    # ----------------------------------------------------------
    # STOP
    # ----------------------------------------------------------
    def _on_stop(self, _msg: MsgEmpty) -> None:
        self._cancel = True
        self._emit("STOP:REQ")

    # ----------------------------------------------------------
    def _publish_preview(self, goal: PoseStamped) -> None:
        m = Marker()
        m.header.frame_id = goal.header.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "goal"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = goal.pose
        m.scale.x = m.scale.y = m.scale.z = 0.02
        m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 1.0, 0.2, 1.0
        self.pub_preview.publish(MarkerArray(markers=[m]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
