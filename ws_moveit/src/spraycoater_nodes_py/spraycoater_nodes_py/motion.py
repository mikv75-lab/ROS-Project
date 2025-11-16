# -*- coding: utf-8 -*-
# spraycoater_nodes_py/motion.py
#!/usr/bin/env python3
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
from moveit_msgs.msg import RobotTrajectory

from moveit.planning import MoveItPy

from tf2_ros import Buffer, TransformListener
from rclpy.time import Time as RclpyTime
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from spraycoater_nodes_py.utils.config_hub import topics, frames

NODE_KEY = "motion"
GROUP_NAME = "omron_arm_group"
EE_LINK = "tool_mount"      # Endeffektor-Link/TCP
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


def _to_robot_traj_msg(core_traj) -> RobotTrajectory:
    """
    Konvertiert eine MoveIt-Core-Trajektorie (moveit.core.robot_trajectory.RobotTrajectory)
    robust in eine ROS-Message moveit_msgs/RobotTrajectory.

    Versucht zuerst binding-spezifische Hilfsfunktionen (to_msg / to_robot_trajectory_msg),
    fällt dann auf ein einfaches Attribute-Kopieren zurück.
    """
    msg = RobotTrajectory()
    if core_traj is None:
        return msg

    # 1) Moderne Python-Bindings: direct to_msg()
    if hasattr(core_traj, "to_msg"):
        return core_traj.to_msg()  # type: ignore[no-any-return]

    # 2) Klassische API: to_robot_trajectory_msg(msg)
    if hasattr(core_traj, "to_robot_trajectory_msg"):
        core_traj.to_robot_trajectory_msg(msg)  # type: ignore[attr-defined]
        return msg

    # 3) Fallback: Versuche bekannte Attribute direkt zu übernehmen
    if hasattr(core_traj, "joint_trajectory"):
        msg.joint_trajectory = core_traj.joint_trajectory  # type: ignore[assignment]
    if hasattr(core_traj, "multi_dof_joint_trajectory"):
        msg.multi_dof_joint_trajectory = core_traj.multi_dof_joint_trajectory  # type: ignore[assignment]

    return msg


def _concat_robot_trajectories(chunks: List[RobotTrajectory]) -> Optional[RobotTrajectory]:
    """Hängt mehrere RobotTrajectory-Stücke zeitlich korrekt zusammen."""
    if not chunks:
        return None
    out = RobotTrajectory()
    out.joint_trajectory.joint_names = chunks[0].joint_trajectory.joint_names[:]
    t_offset = 0.0
    for i, tr in enumerate(chunks):
        jt = tr.joint_trajectory
        if i == 0:
            out.multi_dof_joint_trajectory = tr.multi_dof_joint_trajectory  # falls genutzt
        if not jt.points:
            continue
        # Zeitversatz anwenden
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
            "moveit_py_motion",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # --- Resolver (Topics/QoS/Frames)
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve
        self.frame_world = self._F(self.frames.get("world", WORLD_FRAME))

        # --- TF buffer (für named Ziele)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        # --- MoveItPy (gleicher Node-Name -> erhält dieselben MoveIt-Parameter)
        self.log = self.get_logger()
        try:
            self.robot = MoveItPy(node_name=self.get_name())
        except Exception as e:
            self.log.error(f"MoveItPy init failed: {e}")
            raise
        time.sleep(0.5)  # kurze Pause bis PlanningScene steht
        self.arm = self.robot.get_planning_component(GROUP_NAME)
        self.robot_model = self.robot.get_robot_model()
        self.log.info(f"✅ MoveItPy ready (group={GROUP_NAME}, ee={EE_LINK})")

        # --- Publishers (mit QoS aus config_hub, latched wo sinnvoll)
        topic_traj = self.loader.publish_topic(NODE_KEY, "trajectory")
        qos_traj   = self.loader.qos_by_id("publish", NODE_KEY, "trajectory")

        topic_plan = self.loader.publish_topic(NODE_KEY, "planned_trajectory_rt")
        topic_exec = self.loader.publish_topic(NODE_KEY, "executed_trajectory_rt")
        topic_prev = self.loader.publish_topic(NODE_KEY, "preview_markers")
        topic_res  = self.loader.publish_topic(NODE_KEY, "motion_result")

        self.pub_traj_jt   = self.create_publisher(JointTrajectory, topic_traj, qos_traj)
        self.pub_traj_plan = self.create_publisher(RobotTrajectory, topic_plan, _latched_qos())
        self.pub_traj_exec = self.create_publisher(RobotTrajectory, topic_exec, _latched_qos())
        self.pub_preview   = self.create_publisher(MarkerArray, topic_prev, _latched_qos())
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
        # Wichtig: _planned ist immer eine moveit_msgs/RobotTrajectory-Message (kein Core-Objekt)
        self._planned: Optional[RobotTrajectory] = None
        self._busy: bool = False
        self._cancel: bool = False
        self._vel_scale: float = 0.3
        self._acc_scale: float = 0.3

        self._info("MotionNode online.")

    # ------------------ Topic handlers ------------------

    def _on_plan_waypoints(self, msg: PoseArray) -> None:
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        if not msg.poses:
            self._emit_result("ERROR:EMPTY_WAYPOINTS")
            return
        frame = msg.header.frame_id or self.frame_world

        chunks: List[RobotTrajectory] = []
        markers: List[Marker] = []
        start_state = "current"
        try:
            for idx, p in enumerate(msg.poses):
                if start_state == "current":
                    self.arm.set_start_state_to_current_state()
                else:
                    self.arm.set_start_state(None)

                goal = PoseStamped()
                goal.header.frame_id = frame
                goal.pose = p

                self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)
                result = self._plan()
                if result and getattr(result, "trajectory", None):
                    # Core → Msg konvertieren
                    traj_msg = _to_robot_traj_msg(result.trajectory)
                    if not traj_msg.joint_trajectory.points:
                        self._emit_result(f"ERROR:PLAN_EMPTY index={idx}")
                        return
                    chunks.append(traj_msg)
                    # für nächste Etappe: Start ist Ende der letzten
                    start_state = "planned"
                else:
                    self._emit_result(f"ERROR:PLAN_FAILED index={idx}")
                    return

                # Vorschau-Marker (TCP-Linie)
                m = Marker()
                m.header.frame_id = frame
                m.ns = "waypoints"
                m.id = idx
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose = goal.pose
                m.scale.x = m.scale.y = m.scale.z = 0.01
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 1.0, 1.0
                markers.append(m)

            merged = _concat_robot_trajectories(chunks)
            if merged is None or not merged.joint_trajectory.points:
                self._emit_result("ERROR:MERGE_FAILED")
                return

            self._planned = merged
            self.pub_traj_plan.publish(merged)
            self.pub_preview.publish(MarkerArray(markers=markers))
            self._emit_result(f"PLANNED:OK steps={len(merged.joint_trajectory.points)}")
        except Exception as e:
            self._emit_result(f"ERROR:EXCEPTION {e}")

    def _on_plan_named(self, msg: MsgString) -> None:
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        name = (msg.data or "").strip()
        if not name:
            self._emit_result("ERROR:EMPTY_NAME")
            return

        # Versuche TF (world->name) zu lesen und Pose in 'world' zu planen.
        try:
            tf = self._lookup_tf(self.frame_world, name)
        except Exception as e:
            self._emit_result(f"ERROR:TF_LOOKUP {e}")
            return

        goal = PoseStamped()
        goal.header.frame_id = self.frame_world
        goal.pose.position.x = tf.transform.translation.x
        goal.pose.position.y = tf.transform.translation.y
        goal.pose.position.z = tf.transform.translation.z
        goal.pose.orientation = tf.transform.rotation

        # planen
        try:
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)
            result = self._plan()
            if result and getattr(result, "trajectory", None):
                traj_msg = _to_robot_traj_msg(result.trajectory)
                if not traj_msg.joint_trajectory.points:
                    self._emit_result(f"ERROR:PLAN_EMPTY named='{name}'")
                    return
                self._planned = traj_msg
                self.pub_traj_plan.publish(traj_msg)
                self._publish_preview_point(goal)
                self._emit_result(f"PLANNED:OK named='{name}'")
            else:
                self._emit_result(f"ERROR:PLAN_FAILED named='{name}'")
        except Exception as e:
            self._emit_result(f"ERROR:EXCEPTION {e}")

    def _on_target_pose(self, msg: PoseStamped) -> None:
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        frame = msg.header.frame_id or self.frame_world
        goal = PoseStamped()
        goal.header.frame_id = frame
        goal.pose = msg.pose
        try:
            self.arm.set_start_state_to_current_state()
            self.arm.set_goal_state(pose_stamped_msg=goal, pose_link=EE_LINK)
            result = self._plan()
            if result and getattr(result, "trajectory", None):
                traj_msg = _to_robot_traj_msg(result.trajectory)
                if not traj_msg.joint_trajectory.points:
                    self._emit_result("ERROR:PLAN_EMPTY pose")
                    return
                self._planned = traj_msg
                self.pub_traj_plan.publish(traj_msg)
                self._publish_preview_point(goal)
                self._emit_result("PLANNED:OK pose")
            else:
                self._emit_result("ERROR:PLAN_FAILED pose")
        except Exception as e:
            self._emit_result(f"ERROR:EXCEPTION {e}")

    def _on_execute(self, msg: MsgBool) -> None:
        if msg.data is False:
            self._on_stop(MsgEmpty())
            return
        if self._busy:
            self._emit_result("ERROR:BUSY")
            return
        if not self._planned or not self._planned.joint_trajectory.points:
            self._emit_result("ERROR:NO_PLAN")
            return

        self._busy = True
        self._cancel = False
        try:
            # JointTrajectory sofort für UI/Logger ausgeben
            self.pub_traj_jt.publish(self._planned.joint_trajectory)

            # MoveItPy erwartet eine moveit_msgs/RobotTrajectory-Message
            ok = self.robot.execute(self._planned, controllers=[])

            if self._cancel:
                self._emit_result("STOPPED:USER_REQUEST")
            elif ok is False:
                self._emit_result("ERROR:EXECUTE_FAILED")
            else:
                # am Ende veröffentlichen wir die tatsächlich gefahrene (hier ~= geplante)
                self.pub_traj_exec.publish(self._planned)
                self._emit_result("EXECUTED:OK")
        except Exception as e:
            self._emit_result(f"ERROR:EXCEPTION {e}")
        finally:
            self._busy = False

    def _on_stop(self, _msg: MsgEmpty) -> None:
        # Harte Cancel-API der execute-Action ist in MoveItPy nicht direkt exponiert;
        # wir setzen eine Cancel-Flag, die bei mehrstufiger Planung/Execution beachtet werden kann.
        self._cancel = True
        self._emit_result("STOP:REQUESTED")

    def _on_cmd(self, msg: MsgString) -> None:
        cmd = (msg.data or "").strip()
        if not cmd:
            return
        if cmd.startswith("speed:"):
            try:
                val = float(cmd.split(":", 1)[1])
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
        """Einfache Single-Pipeline-Planung (default_planning_pipeline aus MoveIt-Params)."""
        # Hinweis: Wenn du vel/acc scaling anwenden willst, gib später PlanRequestParameters rein.
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
