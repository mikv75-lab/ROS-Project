#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/motion.py

from __future__ import annotations

import sys
import time
import json
from typing import Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time as RclpyTime
from rclpy.duration import Duration

from std_msgs.msg import String as MsgString, Bool as MsgBool, Empty as MsgEmpty, Float64 as MsgFloat64
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from moveit.planning import MoveItPy
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY    = "motion"
GROUP_NAME  = "omron_arm_group"
EE_LINK     = "tcp"
WORLD_FRAME = "world"


# ----------------- QoS -----------------

def latched():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )


# ----------------- MotionNode -----------------

class Motion(Node):

    def __init__(self):
        super().__init__("motion")

        self.cfg_topics = topics()
        self.cfg_frames = frames()

        self.frame_world = self.cfg_frames.resolve(self.cfg_frames.get("world", WORLD_FRAME))
        self.frame_scene = self.cfg_frames.resolve(self.cfg_frames.get("scene", "scene"))

        # --- TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # --- MoveIt
        self.log = self.get_logger()
        self.robot = MoveItPy(node_name=self.get_name())
        time.sleep(0.5)
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.log.info(f"MoveItPy ready (group={GROUP_NAME})")

        # ---------------- intern: Motion-Config (Speed + Planner) ----------------
        # Speed in mm/s (z.B. aus UI / Recipe)
        self._speed_mm_s: float = 100.0

        # Planner-Konfiguration (Pipeline/Planner/Params), wie vom UI geschickt
        # Erwartetes Format des JSON-Strings:
        #   {
        #     "pipeline": "ompl",
        #     "planner_id": "RRTConnectkConfigDefault",
        #     "params": { ... }
        #   }
        self._planner_cfg: Dict[str, Any] = {}

        # ---------------- Publishers ----------------

        self.pub_target_traj = self.create_publisher(
            JointTrajectory,
            self.cfg_topics.publish_topic(NODE_KEY, "target_trajectory"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "target_trajectory")
        )

        self.pub_planned = self.create_publisher(
            RobotTrajectoryMsg,
            self.cfg_topics.publish_topic(NODE_KEY, "planned_trajectory_rt"),
            latched()
        )

        self.pub_executed = self.create_publisher(
            RobotTrajectoryMsg,
            self.cfg_topics.publish_topic(NODE_KEY, "executed_trajectory_rt"),
            latched()
        )

        self.pub_preview = self.create_publisher(
            MarkerArray,
            self.cfg_topics.publish_topic(NODE_KEY, "preview_markers"),
            latched()
        )

        self.pub_result = self.create_publisher(
            MsgString,
            self.cfg_topics.publish_topic(NODE_KEY, "motion_result"),
            self.cfg_topics.qos_by_id("publish", NODE_KEY, "motion_result")
        )

        # --------------- Subscribers ----------------

        # 1) freie Pose planen (Posen kommen in m, beliebiger Frame wie 'scene')
        self.create_subscription(
            PoseStamped,
            self.cfg_topics.subscribe_topic(NODE_KEY, "plan_pose"),
            self._on_plan_pose,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "plan_pose")
        )

        # 2) named Pose planen (home/service)
        self.create_subscription(
            MsgString,
            self.cfg_topics.subscribe_topic(NODE_KEY, "plan_named"),
            self._on_plan_named,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "plan_named")
        )

        # 3) geplanten Plan ausführen
        self.create_subscription(
            MsgBool,
            self.cfg_topics.subscribe_topic(NODE_KEY, "execute"),
            self._on_execute,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "execute")
        )

        # 4) Bewegung stoppen
        self.create_subscription(
            MsgEmpty,
            self.cfg_topics.subscribe_topic(NODE_KEY, "stop"),
            self._on_stop,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "stop")
        )

        # 5) Motion-Speed (mm/s) setzen
        self.create_subscription(
            MsgFloat64,
            self.cfg_topics.subscribe_topic(NODE_KEY, "set_speed_mm_s"),
            self._on_set_speed_mm_s,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "set_speed_mm_s")
        )

        # 6) Planner-Konfiguration (Pipeline + Planner-ID + Params) setzen
        self.create_subscription(
            MsgString,
            self.cfg_topics.subscribe_topic(NODE_KEY, "set_planner_cfg"),
            self._on_set_planner_cfg,
            self.cfg_topics.qos_by_id("subscribe", NODE_KEY, "set_planner_cfg")
        )

        # state
        self._planned: Optional[RobotTrajectoryCore] = None
        self._busy = False
        self._cancel = False

        self.log.info("MotionNode online.")

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

    def _emit(self, text: str):
        self.pub_result.publish(MsgString(data=text))
        self.log.info(text)

    # ----------------------------------------------------------
    # CONFIG: Speed + Planner (nur Topics implementiert)
    # ----------------------------------------------------------

    def _on_set_speed_mm_s(self, msg: MsgFloat64):
        v = float(msg.data)
        if v <= 0.0:
            self.log.warn(f"[config] set_speed_mm_s ignoriert (<= 0): {v}")
            return
        self._speed_mm_s = v
        self.log.info(f"[config] Motion speed set to {self._speed_mm_s:.1f} mm/s")

    def _on_set_planner_cfg(self, msg: MsgString):
        raw = msg.data or ""
        try:
            cfg = json.loads(raw)
            if not isinstance(cfg, dict):
                raise ValueError("planner_cfg JSON ist kein Objekt (dict).")

            self._planner_cfg = cfg
            pipeline = cfg.get("pipeline", "<none>")
            planner_id = cfg.get("planner_id", "<none>")
            self.log.info(
                f"[config] Planner cfg updated: pipeline='{pipeline}', planner_id='{planner_id}'"
            )
        except Exception as e:
            self.log.error(f"[config] set_planner_cfg: invalid JSON '{raw}': {e}")

    # ----------------------------------------------------------
    # PLAN POSE (Posen kommen in m, beliebiger Frame wie 'scene')
    # ----------------------------------------------------------

    def _on_plan_pose(self, msg: PoseStamped):
        if self._busy:
            self._emit("ERROR:BUSY")
            return

        self._planned = None

        in_frame = msg.header.frame_id or self.frame_world
        self.log.info(f"[plan_pose] in_frame={in_frame} → world={self.frame_world}")

        # Eingangs-Pose: bereits in m (z.B. aus Setup-StateMachine)
        pose_in = msg.pose
        self.log.info(
            f"[plan_pose] incoming (m) in '{in_frame}': "
            f"({pose_in.position.x:.3f}, {pose_in.position.y:.3f}, {pose_in.position.z:.3f})"
        )

        try:
            # Falls nötig, von in_frame -> world per TF transformieren
            if in_frame != self.frame_world:
                tf = self._lookup_tf(self.frame_world, in_frame)
                # do_transform_pose arbeitet hier mit Pose -> gibt Pose zurück
                pose_world = do_transform_pose(pose_in, tf)
            else:
                pose_world = pose_in

            self.log.info(
                f"[plan_pose] world pose (m): "
                f"({pose_world.position.x:.3f}, {pose_world.position.y:.3f}, {pose_world.position.z:.3f})"
            )

            # Goal: TCP in world
            goal = PoseStamped()
            goal.header.frame_id = self.frame_world
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose = pose_world

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)

            result = self.arm.plan()
            core = getattr(result, "trajectory", None)

            if core is None:
                self._emit("ERROR:NO_TRAJ pose")
                return

            msg_traj = core.get_robot_trajectory_msg()

            self._planned = core
            self.pub_planned.publish(msg_traj)
            self._publish_preview(goal)

            self._emit("PLANNED:OK pose")

        except Exception as e:
            self._emit(f"ERROR:EX {e}")

    # ----------------------------------------------------------
    # PLAN NAMED (home/service) – Frames aus TF, bereits in m
    # ----------------------------------------------------------

    def _on_plan_named(self, msg: MsgString):
        if self._busy:
            self._emit("ERROR:BUSY")
            return

        name = msg.data.strip()
        if not name:
            self._emit("ERROR:EMPTY_NAME")
            return

        self._planned = None

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

            result = self.arm.plan()
            core = getattr(result, "trajectory", None)

            if core is None:
                self._emit("ERROR:NO_TRAJ named")
                return

            msg_traj = core.get_robot_trajectory_msg()

            self._planned = core
            self.pub_planned.publish(msg_traj)
            self._publish_preview(goal)

            self._emit(f"PLANNED:OK named='{name}'")

        except Exception as e:
            self._emit(f"ERROR:EX {e}")

    # ----------------------------------------------------------
    # EXECUTE  (Variante A: MoveIt fährt selbst)
    # ----------------------------------------------------------

    def _on_execute(self, msg: MsgBool):
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
            # geplante Trajektorie aus MoveItPy holen
            msg_traj = self._planned.get_robot_trajectory_msg()

            # WICHTIG:
            #  - KEIN direktes publish auf pub_target_traj mehr!
            #  - MoveIt kümmert sich selbst darum, die Trajektorie
            #    an den FollowJointTrajectory-Controller zu schicken.
            ok = self.robot.execute(self._planned, controllers=[])

            if self._cancel:
                self._emit("STOPPED:USER")
            elif not ok:
                # MoveIt meldet einen echten Ausführungsfehler
                self._emit("ERROR:EXEC")
            else:
                # Ausführung war laut MoveIt erfolgreich:
                # -> Trajektorie als "executed" publishen
                self.pub_executed.publish(msg_traj)
                self._emit("EXECUTED:OK")

        except Exception as e:
            self._emit(f"ERROR:EX {e}")

        finally:
            self._busy = False

    # ----------------------------------------------------------
    # STOP
    # ----------------------------------------------------------

    def _on_stop(self, _msg: MsgEmpty):
        self._cancel = True
        self._emit("STOP:REQ")

    # ----------------------------------------------------------

    def _publish_preview(self, goal: PoseStamped):
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


# ------------------------ main ------------------------

def main(args=None):
    # VSCode/debugpy hängt dir manchmal .yaml args an → rausfiltern
    sys.argv[:] = [a for a in sys.argv if not a.endswith(".yaml")]
    rclpy.init(args=args)
    node = Motion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
