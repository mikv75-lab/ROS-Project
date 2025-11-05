#!/usr/bin/env python3
from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory

from spraycoater_nodes_py.utils.frames import load_frames
from spraycoater_nodes_py.utils.topics_loader import TopicsLoader

from moveit.planning import MoveItPy


PARAM_FRAMES_YAML = "frames_yaml"
PARAM_TOPICS_YAML = "topics_yaml"
PARAM_QOS_YAML    = "qos_yaml"


class Motion(Node):
    def __init__(self):
        super().__init__("motion")

        # --- Params (wie in scene.py) ---
        self.declare_parameter(PARAM_FRAMES_YAML, "")
        self.declare_parameter(PARAM_TOPICS_YAML, "")
        self.declare_parameter(PARAM_QOS_YAML, "")

        frames_yaml = self.get_parameter(PARAM_FRAMES_YAML).value
        topics_yaml = self.get_parameter(PARAM_TOPICS_YAML).value
        qos_yaml    = self.get_parameter(PARAM_QOS_YAML).value

        self.frames = load_frames(frames_yaml)
        self._F = self.frames.resolve

        self.loader = TopicsLoader(topics_yaml, qos_yaml)
        node_key = "motion"

        # --- MoveItPy (Gruppe = omron_arm_group) ---
        self._last_rt: RobotTrajectory | None = None
        try:
            self._moveit = MoveItPy(node_name="motion_manager")
            self._arm = self._moveit.get_planning_component("omron_arm_group")
            self._executor = self._moveit.get_trajectory_executor()
            self.get_logger().info("🤖 MoveItPy initialisiert (group=omron_arm_group).")
        except Exception as e:
            self._moveit = None
            self._arm = None
            self._executor = None
            self.get_logger().error(f"MoveItPy init fehlgeschlagen: {e}")

        # --- Subs (aus topics.yaml) ---
        self._subs = []
        self._subs.append(self.create_subscription(
            String,
            self.loader.subscribe_topic(node_key, "plan_named"),
            self._on_plan_named,
            self.loader.qos_by_id("subscribe", node_key, "plan_named"),
        ))
        self._subs.append(self.create_subscription(
            PoseStamped,
            self.loader.subscribe_topic(node_key, "target_pose") if self.loader.has_sub(node_key, "target_pose") else "/unused",
            self._on_target_pose,
            self.loader.qos_by_id("subscribe", node_key, "target_pose") if self.loader.has_sub(node_key, "target_pose") else 10,
        ))
        self._subs.append(self.create_subscription(
            String,
            self.loader.subscribe_topic(node_key, "cmd") if self.loader.has_sub(node_key, "cmd") else "/unused",
            self._on_cmd,
            self.loader.qos_by_id("subscribe", node_key, "cmd") if self.loader.has_sub(node_key, "cmd") else 10,
        ))

        # --- Pubs (aus topics.yaml) ---
        self._pub_result = self.create_publisher(
            String,
            self.loader.publish_topic(node_key, "motion_result"),
            self.loader.qos_by_id("publish", node_key, "motion_result"),
        )
        self._pub_planned = self.create_publisher(
            RobotTrajectory,
            self.loader.publish_topic(node_key, "planned_trajectory_rt") if self.loader.has_pub(node_key, "planned_trajectory_rt") else "/spraycoater/motion/trajectory_planned",
            self.loader.qos_by_id("publish", node_key, "planned_trajectory_rt") if self.loader.has_pub(node_key, "planned_trajectory_rt") else 10,
        )
        self._pub_executed = self.create_publisher(
            RobotTrajectory,
            self.loader.publish_topic(node_key, "executed_trajectory_rt") if self.loader.has_pub(node_key, "executed_trajectory_rt") else "/spraycoater/motion/trajectory_executed",
            self.loader.qos_by_id("publish", node_key, "executed_trajectory_rt") if self.loader.has_pub(node_key, "executed_trajectory_rt") else 10,
        )

        self._pending_pose: PoseStamped | None = None
        self.get_logger().info("✅ MotionManager bereit.")

    # ---------- Helpers ----------
    def _plan_pose(self, pose: PoseStamped, execute: bool) -> bool:
        if not (self._arm and self._executor):
            self.get_logger().warning("MoveItPy nicht verfügbar.")
            return False
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped=pose)
        plan = self._arm.plan()
        if not plan or not plan.trajectory:
            self.get_logger().warning("Planung fehlgeschlagen.")
            return False
        self._last_rt = plan.trajectory
        self._pub_planned.publish(self._last_rt)
        if execute:
            self._executor.execute(self._last_rt)
            self._pub_executed.publish(self._last_rt)
        return True

    def _plan_named(self, name: str, execute: bool) -> bool:
        if not (self._arm and self._executor):
            self.get_logger().warning("MoveItPy nicht verfügbar.")
            return False
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=name)
        plan = self._arm.plan()
        if not plan or not plan.trajectory:
            self.get_logger().warning(f"Planung zu named '{name}' fehlgeschlagen.")
            return False
        self._last_rt = plan.trajectory
        self._pub_planned.publish(self._last_rt)
        if execute:
            self._executor.execute(self._last_rt)
            self._pub_executed.publish(self._last_rt)
        return True

    # ---------- Callbacks ----------
    def _on_plan_named(self, msg: String):
        name = (msg.data or "").strip()
        ok = self._plan_named(name, execute=False)
        self._pub_result.publish(String(data=f"plan_named:{'ok' if ok else 'fail'}"))

    def _on_target_pose(self, msg: PoseStamped):
        self._pending_pose = msg
        self.get_logger().info(f"Pose empfangen (frame={msg.header.frame_id})")

    def _on_cmd(self, msg: String):
        cmd = (msg.data or "").strip()
        ok = False
        try:
            if cmd.startswith("plan_to_named:"):
                ok = self._plan_named(cmd.split(":",1)[1], execute=False)
            elif cmd.startswith("move_to_named:"):
                ok = self._plan_named(cmd.split(":",1)[1], execute=True)
            elif cmd == "plan_to_pose":
                if self._pending_pose:
                    ok = self._plan_pose(self._pending_pose, execute=False)
            elif cmd == "move_to_pose":
                if self._pending_pose:
                    ok = self._plan_pose(self._pending_pose, execute=True)
            elif cmd == "execute_last":
                if self._last_rt and self._executor:
                    self._executor.execute(self._last_rt)
                    self._pub_executed.publish(self._last_rt)
                    ok = True
        except Exception as e:
            self.get_logger().error(f"Fehler bei '{cmd}': {e}")
            ok = False
        self._pub_result.publish(String(data=f"{cmd}:{'ok' if ok else 'fail'}"))


def main(args=None):
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
