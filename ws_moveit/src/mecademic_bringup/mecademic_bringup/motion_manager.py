#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.params import load_params, AppParams

# --- MoveItPy ---
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder


class MotionManager(Node):
    """
    MotionManager
    -------------
    Steuert Bewegungen über Topics:
      - /meca/motion/cmd
      - /meca/motion/target_pose
      - /meca/motion/planned_traj
      - /meca/motion/executed_traj
    """

    def __init__(self):
        super().__init__("motion_manager")
        self.get_logger().info("🚀 MotionManager gestartet")

        # Parameter + Setup
        self.params: AppParams = load_params(self)
        self.frames = FRAMES
        self.topics = Topics()
        self._pending_pose: PoseStamped | None = None
        self._last_traj: JointTrajectory | None = None

        # --- MoveItPy Initialisierung ---
        try:
            self._moveit = MoveItPy()
            self._arm = self._moveit.get_planning_component("meca_arm_group")
            self.get_logger().info("🤖 MoveItPy erfolgreich initialisiert (Parameter aus Launch).")
        except Exception as e:
            self._moveit = None
            self._arm = None
            self.get_logger().warning(f"⚠️ MoveItPy konnte nicht initialisiert werden: {e}")


        # --- Subscriptions ---
        self.create_subscription(String, self.topics.motion_cmd, self._on_cmd, qos_default())
        self.create_subscription(PoseStamped, self.topics.motion_target_pose, self._on_pose, qos_latched())

        # --- Publishers ---
        self._result_pub = self.create_publisher(String, self.topics.motion_result, qos_default())
        self._ready_pub = self.create_publisher(Bool, self.topics.state_ready, qos_latched())
        self._traj_planned_pub = self.create_publisher(JointTrajectory, "/meca/motion/planned_traj", qos_latched())
        self._traj_executed_pub = self.create_publisher(JointTrajectory, "/meca/motion/executed_traj", qos_latched())

        # Ready-Flag
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info("✅ MotionManager bereit und hört auf Topics")

    # ==================================================================
    # --- CALLBACKS -----------------------------------------------------
    # ==================================================================

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"📩 Motion-Command empfangen: {cmd}")
        ok = False

        if not self._arm:
            self.get_logger().warning("⚠️ Kein MoveItPy verfügbar, Befehl ignoriert.")
            return

        try:
            if cmd.startswith("plan_to_named:"):
                name = cmd.split(":", 1)[1]
                ok = self._plan_named(name, execute=False)
            elif cmd.startswith("move_to_named:"):
                name = cmd.split(":", 1)[1]
                ok = self._plan_named(name, execute=True)
            elif cmd.startswith("move_to_pose:"):
                name = cmd.split(":", 1)[1]
                ok = self.move_to_pose_name(name, execute=True)
            elif cmd.startswith("plan_to_pose:"):
                name = cmd.split(":", 1)[1]
                ok = self.move_to_pose_name(name, execute=False)
            elif cmd == "plan_to_pose":
                ok = self._plan_pose(execute=False)
            elif cmd == "move_to_pose":
                ok = self._plan_pose(execute=True)
            elif cmd == "execute_last":
                ok = self._execute_last()
            else:
                self.get_logger().warning(f"❓ Unbekanntes Kommando: {cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei '{cmd}': {e}")
            ok = False

        self._result_pub.publish(String(data=f"{cmd}:{'ok' if ok else 'fail'}"))

    # ==================================================================
    # --- PLANUNG & AUSFÜHRUNG ----------------------------------------
    # ==================================================================

    # ==================================================================
    # --- MOVE TO POSE BY NAME ----------------------------------------
    # ==================================================================
    def move_to_pose_name(self, name: str, execute: bool = True) -> bool:
        """
        Holt Pose von /meca/poses/<name>, plant und (optional) führt aus.
        """
        from geometry_msgs.msg import PoseStamped
        import rclpy

        topic = f"/meca/poses/{name}"
        self.get_logger().info(f"🎯 Warte auf Pose '{name}' ({topic}) ...")

        try:
            pose_msg = rclpy.wait_for_message(PoseStamped, topic, self, timeout=3.0)
        except Exception as e:
            self.get_logger().error(f"❌ Fehler beim Empfangen von Pose '{name}': {e}")
            return False

        if pose_msg is None:
            self.get_logger().error(f"❌ Keine Pose '{name}' empfangen – Timeout.")
            return False

        self.get_logger().info(f"📦 Pose '{name}' empfangen aus Frame '{pose_msg.header.frame_id}'")

        try:
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped=pose_msg)
            plan_result = self._arm.plan()
            if not plan_result or not plan_result.trajectory:
                self.get_logger().warning(f"❌ Planung zu Pose '{name}' fehlgeschlagen.")
                return False

            traj = plan_result.trajectory
            self._traj_planned_pub.publish(traj)
            self._last_traj = traj
            self.get_logger().info(f"🟩 Trajektorie zu '{name}' geplant")

            if execute:
                self._arm.execute()
                self._publish_executed()
                self.get_logger().info(f"✅ Pose '{name}' erfolgreich erreicht")
            return True

        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei move_to_pose_name('{name}'): {e}")
            return False


    def _plan_named(self, name: str, execute: bool) -> bool:
        self.get_logger().info(f"🧭 Plan to named pose: {name} (execute={execute})")
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=name)
        plan_result = self._arm.plan()
        if not plan_result or not plan_result.trajectory:
            self.get_logger().warning("❌ Planung fehlgeschlagen.")
            return False

        traj = plan_result.trajectory
        self._traj_planned_pub.publish(traj)
        self._last_traj = traj
        self.get_logger().info("🟩 Geplante Trajektorie veröffentlicht")

        if execute:
            self._arm.execute()
            self._publish_executed()
        return True

    def _plan_pose(self, execute: bool) -> bool:
        if not self._pending_pose:
            self.get_logger().warning("❌ Keine Zielpose vorhanden.")
            return False

        pose = self._pending_pose
        self.get_logger().info(f"🎯 Plane zu Pose ({pose.header.frame_id}) (execute={execute})")
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped=pose)
        plan_result = self._arm.plan()

        if not plan_result or not plan_result.trajectory:
            self.get_logger().warning("❌ Planung zu Pose fehlgeschlagen.")
            return False

        traj = plan_result.trajectory
        self._traj_planned_pub.publish(traj)
        self._last_traj = traj
        self.get_logger().info("🟩 Geplante Trajektorie veröffentlicht")

        if execute:
            self._arm.execute()
            self._publish_executed()
        return True

    def _execute_last(self) -> bool:
        if not self._last_traj:
            self.get_logger().warning("❌ Keine gespeicherte Trajektorie vorhanden.")
            return False
        self.get_logger().info("▶️ Führe letzte Trajektorie aus …")
        self._arm.execute()
        self._publish_executed()
        return True

    def _publish_executed(self):
        state = self._moveit.get_robot_state()
        jt = JointTrajectory()
        jt.joint_names = self._arm.joint_model_group_variable_names
        pt = JointTrajectoryPoint()
        pt.positions = state.joint_positions
        pt.time_from_start.sec = 0
        jt.points.append(pt)
        self._traj_executed_pub.publish(jt)
        self.get_logger().info("🟦 Gefahrene Trajektorie veröffentlicht")

    # ==================================================================
    # --- POSE CALLBACK ------------------------------------------------
    # ==================================================================

    def _on_pose(self, msg: PoseStamped):
        self._pending_pose = msg
        self.get_logger().info(
            f"✅ Neue Zielpose empfangen im Frame '{msg.header.frame_id or self.frames['world']}'"
        )


# ==================================================================
# --- MAIN ----------------------------------------------------------
# ==================================================================
def main(args=None):
    rclpy.init(args=args)
    node = MotionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
