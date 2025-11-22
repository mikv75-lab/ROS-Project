#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/motion.py

from __future__ import annotations

import sys
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String as MsgString, Bool as MsgBool, Empty as MsgEmpty
from geometry_msgs.msg import PoseStamped, PoseArray
from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from moveit.planning import MoveItPy
from moveit.core.robot_trajectory import RobotTrajectory as RobotTrajectoryCore

from tf2_ros import Buffer, TransformListener
from rclpy.time import Time as RclpyTime
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# Achtung: in ROS2 erwartet do_transform_pose(geometry_msgs/Pose, TransformStamped)
from tf2_geometry_msgs import do_transform_pose

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY = "motion"
GROUP_NAME = "omron_arm_group"
EE_LINK = "tcp"
WORLD_FRAME = "world"


# ------------------------ Utilities ------------------------

def _strip_launch_yaml_args():
    """Entfernt ggf. von launch injizierte YAML-Args, um rcl-Parsingfehler zu vermeiden."""
    sys.argv[:] = [
        a for a in sys.argv
        if not (a.endswith(".yaml") or a.startswith("/tmp/launch_params_") or a == "--params-file")
    ]


def _latched_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        depth=1,
    )


def _concat_robot_trajectories(chunks: List[RobotTrajectoryMsg]) -> Optional[RobotTrajectoryMsg]:
    """Hängt mehrere RobotTrajectory-Message-Stücke zeitlich korrekt zusammen."""
    if not chunks:
        return None
    out = RobotTrajectoryMsg()
    out.joint_trajectory.joint_names = chunks[0].joint_trajectory.joint_names[:]
    t_offset = 0.0
    for i, tr in enumerate(chunks):
        jt = tr.joint_trajectory
        if i == 0:
            out.multi_dof_joint_trajectory = tr.multi_dof_joint_trajectory
        if not jt.points:
            continue
        first_t = jt.points[0].time_from_start.sec + jt.points[0].time_from_start.nanosec * 1e-9
        for p in jt.points:
            t = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            t_rel = (t - first_t) + t_offset
            p.time_from_start.sec = int(t_rel)
            p.time_from_start.nanosec = int((t_rel - int(t_rel)) * 1e9)
            out.joint_trajectory.points.append(p)
        t_last = out.joint_trajectory.points[-1].time_from_start
        t_offset = t_last.sec + t_last.nanosec * 1e-9
    return out


# ------------------------ Node ------------------------

class Motion(Node):
    def __init__(self) -> None:
        super().__init__(
            "motion",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # --- Resolver (Topics/QoS/Frames)
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve
        self.frame_world = self._F(self.frames.get("world", WORLD_FRAME))

        # --- TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # --- MoveItPy
        self.log = self.get_logger()
        try:
            self.robot = MoveItPy(node_name=self.get_name())
        except Exception as e:
            self.log.error(f"MoveItPy init failed: {e}")
            raise
        time.sleep(0.5)
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.log.info(f"✅ MoveItPy ready (group={GROUP_NAME}, ee={EE_LINK})")

        # --- Publishers
        topic_traj = self.loader.publish_topic(NODE_KEY, "trajectory")
        qos_traj   = self.loader.qos_by_id("publish", NODE_KEY, "trajectory")

        topic_plan = self.loader.publish_topic(NODE_KEY, "planned_trajectory_rt")
        topic_exec = self.loader.publish_topic(NODE_KEY, "executed_trajectory_rt")
        topic_prev = self.loader.publish_topic(NODE_KEY, "preview_markers")
        topic_res  = self.loader.publish_topic(NODE_KEY, "motion_result")

        self.pub_traj_jt   = self.create_publisher(JointTrajectory,       topic_traj, qos_traj)
        self.pub_traj_plan = self.create_publisher(RobotTrajectoryMsg,    topic_plan, _latched_qos())
        self.pub_traj_exec = self.create_publisher(RobotTrajectoryMsg,    topic_exec, _latched_qos())
        self.pub_preview   = self.create_publisher(MarkerArray,           topic_prev, _latched_qos())
        self.pub_result    = self.create_publisher(
            MsgString,
            topic_res,
            self.loader.qos_by_id("publish", NODE_KEY, "motion_result"),
        )

        # --- Subscribers
        sub_plan_wp    = self.loader.subscribe_topic(NODE_KEY, "plan_waypoints")
        sub_plan_named = self.loader.subscribe_topic(NODE_KEY, "plan_named")
        sub_execute    = self.loader.subscribe_topic(NODE_KEY, "execute")
        sub_stop       = self.loader.subscribe_topic(NODE_KEY, "stop")
        sub_cmd        = self.loader.subscribe_topic(NODE_KEY, "cmd")
        sub_pose       = self.loader.subscribe_topic(NODE_KEY, "target_pose")

        self.create_subscription(
            PoseArray,
            sub_plan_wp,
            self._on_plan_waypoints,
            self.loader.qos_by_id("subscribe", NODE_KEY, "plan_waypoints"),
        )
        self.create_subscription(
            MsgString,
            sub_plan_named,
            self._on_plan_named,
            self.loader.qos_by_id("subscribe", NODE_KEY, "plan_named"),
        )
        self.create_subscription(
            MsgBool,
            sub_execute,
            self._on_execute,
            self.loader.qos_by_id("subscribe", NODE_KEY, "execute"),
        )
        self.create_subscription(
            MsgEmpty,
            sub_stop,
            self._on_stop,
            self.loader.qos_by_id("subscribe", NODE_KEY, "stop"),
        )
        self.create_subscription(
            MsgString,
            sub_cmd,
            self._on_cmd,
            self.loader.qos_by_id("subscribe", NODE_KEY, "cmd"),
        )
        self.create_subscription(
            PoseStamped,
            sub_pose,
            self._on_target_pose,
            self.loader.qos_by_id("subscribe", NODE_KEY, "target_pose"),
        )

        # --- State
        # Wichtig: _planned ist eine *core* RobotTrajectory (nicht Msg!)
        self._planned: Optional[RobotTrajectoryCore] = None
        self._busy: bool = False
        self._cancel: bool = False
        self._vel_scale: float = 0.3
        self._acc_scale: float = 0.3

        self._info("MotionNode online.")

    # ------------------ Trajektorie-Konvertierung ------------------

    def _core_to_msg(self, traj_core: RobotTrajectoryCore) -> RobotTrajectoryMsg:
        """Konvertiert core-RobotTrajectory -> moveit_msgs/RobotTrajectory."""
        return traj_core.get_robot_trajectory_msg()

    def _msg_to_core(self, msg: RobotTrajectoryMsg) -> RobotTrajectoryCore:
        """Konvertiert moveit_msgs/RobotTrajectory -> core-RobotTrajectory."""
        core = RobotTrajectoryCore(self.robot_model, GROUP_NAME)
        core.set_robot_trajectory_msg(self.robot_model, GROUP_NAME, msg)
        return core

    def _extract_traj(self, ctx: str, result) -> Optional[RobotTrajectoryCore]:
        """
        Holt aus dem Plan-Resultat die core-RobotTrajectory.

        Für Logging wird kurz in eine Msg konvertiert.
        """
        if result is None:
            self._emit_result(f"ERROR:NO_RESULT {ctx}")
            return None

        traj_core: RobotTrajectoryCore = getattr(result, "trajectory", None)
        if traj_core is None:
            self._emit_result(f"ERROR:NO_TRAJECTORY {ctx}")
            return None

        try:
            msg = traj_core.get_robot_trajectory_msg()
        except Exception as e:
            self.log.error(f"_extract_traj({ctx}): get_robot_trajectory_msg() failed: {e}")
            self._emit_result(f"ERROR:TRAJ_CONVERT {ctx}")
            return None

        n_pts = len(msg.joint_trajectory.points)
        if n_pts == 0:
            self._emit_result(f"ERROR:PLAN_EMPTY {ctx}")
            return None

        self.log.info(
            f"_extract_traj({ctx}): {n_pts} points, "
            f"joints={msg.joint_trajectory.joint_names}"
        )
        return traj_core

    # ------------------ Topic handlers ------------------

    def _on_plan_waypoints(self, msg: PoseArray) -> None:
        """
        Waypoints können z.B. im Frame 'scene' kommen.
        Hier werden sie (falls nötig) nach world transformiert.
        """
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        if not msg.poses:
            self._emit_result("ERROR:EMPTY_WAYPOINTS")
            return

        # 🔧 alte Planung verwerfen
        self._planned = None

        in_frame = msg.header.frame_id or self.frame_world
        self.log.info(f"[plan_waypoints] in_frame={in_frame}, world={self.frame_world}")

        tf = None
        if in_frame != self.frame_world:
            try:
                tf = self._lookup_tf(self.frame_world, in_frame)
                self.log.info(f"[plan_waypoints] using TF {self.frame_world} <- {in_frame}")
            except Exception as e:
                self._emit_result(f"ERROR:TF_LOOKUP {e}")
                return

        core_chunks: List[RobotTrajectoryCore] = []
        msg_chunks: List[RobotTrajectoryMsg] = []
        markers: List[Marker] = []
        start_state = "current"

        try:
            for idx, p in enumerate(msg.poses):
                if start_state == "current":
                    self.arm.set_start_state_to_current_state()
                else:
                    self.arm.set_start_state(None)

                # p ist geometry_msgs/Pose (kein PoseStamped!)
                if tf is not None:
                    pose_world = do_transform_pose(p, tf)  # Pose(scene) -> Pose(world)
                else:
                    pose_world = p

                goal = PoseStamped()
                goal.header.frame_id = self.frame_world
                goal.pose = pose_world

                self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)
                result = self._plan()
                core_traj = self._extract_traj(f"wp_index={idx}", result)
                if not core_traj:
                    self._planned = None
                    return

                msg_traj = core_traj.get_robot_trajectory_msg()
                core_chunks.append(core_traj)
                msg_chunks.append(msg_traj)
                start_state = "planned"

                m = Marker()
                m.header.frame_id = goal.header.frame_id
                m.ns = "waypoints"
                m.id = idx
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = goal.pose
                m.scale.x = m.scale.y = m.scale.z = 0.01
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 1.0
                markers.append(m)

            merged_msg = _concat_robot_trajectories(msg_chunks)
            if merged_msg is None or not merged_msg.joint_trajectory.points:
                self._emit_result("ERROR:MERGE_FAILED")
                self._planned = None
                return

            merged_core = self._msg_to_core(merged_msg)

            self._planned = merged_core
            self.pub_traj_plan.publish(merged_msg)
            self.pub_preview.publish(MarkerArray(markers=markers))
            self._emit_result(f"PLANNED:OK steps={len(merged_msg.joint_trajectory.points)}")
        except Exception as e:
            self._planned = None
            self._emit_result(f"ERROR:EXCEPTION {e}")

    def _on_plan_named(self, msg: MsgString) -> None:
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        name = (msg.data or "").strip()
        if not name:
            self._emit_result("ERROR:EMPTY_NAME")
            return

        # 🔧 alte Planung verwerfen
        self._planned = None

        try:
            tf = self._lookup_tf(self.frame_world, name)
            self.log.info(
                f"[plan_named] TF {self.frame_world} <- {name}: "
                f"t=({tf.transform.translation.x:.3f}, "
                f"{tf.transform.translation.y:.3f}, "
                f"{tf.transform.translation.z:.3f})"
            )
        except Exception as e:
            self._emit_result(f"ERROR:TF_LOOKUP {e}")
            return

        goal = PoseStamped()
        goal.header.frame_id = self.frame_world
        goal.pose.position.x = tf.transform.translation.x
        goal.pose.position.y = tf.transform.translation.y
        goal.pose.position.z = tf.transform.translation.z
        goal.pose.orientation = tf.transform.rotation

        try:
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)
            result = self._plan()
            core_traj = self._extract_traj(f"named='{name}'", result)
            if not core_traj:
                self._planned = None
                return

            msg_traj = core_traj.get_robot_trajectory_msg()

            self._planned = core_traj
            self.pub_traj_plan.publish(msg_traj)
            self._publish_preview_point(goal)
            self._emit_result(f"PLANNED:OK named='{name}'")
        except Exception as e:
            self._planned = None
            self._emit_result(f"ERROR:EXCEPTION {e}")

    def _on_target_pose(self, msg: PoseStamped) -> None:
        """
        Freie Zielpose:
        - Pose kann z.B. im Frame 'scene' kommen
        - wird (falls nötig) nach world transformiert
        - MoveIt bekommt immer eine Pose im world-Frame
        - TCP (EE_LINK) soll GENAU diese Pose einnehmen.
        """
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return

        # 🔧 alte Planung verwerfen
        self._planned = None

        in_frame = msg.header.frame_id or self.frame_world
        self.log.info(f"[target_pose] in_frame={in_frame}, world={self.frame_world}")

        goal = PoseStamped()
        goal.header.frame_id = self.frame_world

        try:
            if in_frame != self.frame_world:
                self.log.info(f"[target_pose] transform {in_frame} -> {self.frame_world}")
                tf = self._lookup_tf(self.frame_world, in_frame)
                # msg.pose ist geometry_msgs/Pose -> perfekt für do_transform_pose
                pose_world = do_transform_pose(msg.pose, tf)  # Pose(scene) -> Pose(world)
            else:
                pose_world = msg.pose

            self.log.info(
                f"[target_pose] world pose: "
                f"pos=({pose_world.position.x:.3f}, "
                f"{pose_world.position.y:.3f}, "
                f"{pose_world.position.z:.3f}), "
                f"quat=({pose_world.orientation.x:.3f}, "
                f"{pose_world.orientation.y:.3f}, "
                f"{pose_world.orientation.z:.3f}, "
                f"{pose_world.orientation.w:.3f})"
            )

            goal.pose = pose_world

            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)
            result = self._plan()
            core_traj = self._extract_traj("pose", result)
            if not core_traj:
                self._planned = None
                return

            msg_traj = core_traj.get_robot_trajectory_msg()

            self._planned = core_traj
            self.pub_traj_plan.publish(msg_traj)
            self._publish_preview_point(goal)
            self._emit_result("PLANNED:OK pose")
        except Exception as e:
            self._planned = None
            self._emit_result(f"ERROR:EXCEPTION {e}")

    def _on_execute(self, msg: MsgBool) -> None:
        if msg.data is False:
            self._on_stop(MsgEmpty())
            return
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        if not self._planned:
            self._emit_result("ERROR:NO_PLAN")
            return

        self._busy = True
        self._cancel = False
        try:
            msg_traj = self._planned.get_robot_trajectory_msg()

            # JointTrajectory für UI/Logger
            self.pub_traj_jt.publish(msg_traj.joint_trajectory)

            # MoveItPy erwartet core-RobotTrajectory
            status = self.robot.execute(self._planned, controllers=[])

            if self._cancel:
                self._emit_result("STOPPED:USER_REQUEST")
            elif not status:
                self._emit_result("ERROR:EXECUTE_FAILED")
            else:
                self.pub_traj_exec.publish(msg_traj)
                self._emit_result("EXECUTED:OK")
        except Exception as e:
            self._emit_result(f"ERROR:EXCEPTION execute(): {e}")
        finally:
            self._busy = False

    def _on_stop(self, _msg: MsgEmpty) -> None:
        self._cancel = True
        self._emit_result("STOP:REQUESTED")

    def _on_cmd(self, msg: MsgString) -> None:
        cmd = (msg.data or "").strip()
        if not cmd:
            return
        if cmd.startswith("speed:"):
            try:
                val = float(cmd.split(":", 1)[1])
                # Achtung: aktuell wird das als Faktor [0.05..1.0] interpretiert
                self._vel_scale = max(0.05, min(1.0, val))
                self._emit_result(f"OK:SPEED vel={self._vel_scale:.2f}")
            except Exception:
                self._emit_result("ERROR:SPEED_FORMAT")
        elif cmd == "clear_preview":
            self.pub_preview.publish(MarkerArray(markers=[]))
            self._emit_result("OK:CLEAR_PREVIEW")
        elif cmd == "clear_plan":
            self._planned = None
            self._emit_result("OK:CLEAR_PLAN")
        else:
            self._emit_result(f"UNKNOWN_CMD:{cmd}")

    # ------------------ Planning helpers ------------------

    def _plan(self):
        """Einfach: benutze die im MoveIt-Config gesetzte Pipeline (fix)."""
        return self.arm.plan()

    def _lookup_tf(self, target: str, source: str):
        timeout = rclpy.duration.Duration(seconds=1.0)
        if not self.tf_buffer.can_transform(target, source, RclpyTime(), timeout):
            raise RuntimeError(f"TF not available {target}<-{source}")
        try:
            return self.tf_buffer.lookup_transform(target, source, RclpyTime())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            raise RuntimeError(str(e))

    # ------------------ Small helpers ------------------

    def _info(self, text: str) -> None:
        self.log.info(text)

    def _emit_result(self, text: str) -> None:
        self.pub_result.publish(MsgString(data=text))
        self.log.info(text)

    def _publish_preview_point(self, goal: PoseStamped) -> None:
        m = Marker()
        m.header.frame_id = goal.header.frame_id or self.frame_world
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
    _strip_launch_yaml_args()
    rclpy.init(args=args)
    node = Motion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
