#!/usr/bin/env python3
from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory  # (bleibt für Typen, falls du’s brauchst)
from moveit_msgs.msg import RobotTrajectory      # <-- neu: passend zu MoveItPy

from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.params import load_params, AppParams

# --- MoveItPy ---
from moveit.planning import MoveItPy


class MotionManager(Node):
    def __init__(self):
        super().__init__("motion_manager")
        self.get_logger().info("🚀 MotionManager gestartet")

        # --- Setup ---
        self.params: AppParams = load_params(self)
        self.frames = FRAMES
        self.topics = Topics()
        self._pending_pose: PoseStamped | None = None

        self._last_rt: RobotTrajectory | None = None   # <-- speichere RobotTrajectory

        # --- MoveItPy Initialisierung ---
        try:
            self._moveit = MoveItPy(node_name="motion_manager")
            self._arm = self._moveit.get_planning_component("meca_arm_group")
            self._executor = self._moveit.get_trajectory_executor()  # <-- wichtig
            self.get_logger().info("🤖 MoveItPy erfolgreich initialisiert.")
        except Exception as e:
            self._moveit = None
            self._arm = None
            self._executor = None
            self.get_logger().warning(f"⚠️ MoveItPy konnte nicht initialisiert werden: {e}")

        # --- Subscriptions (Referenzen halten, sonst GC!) ---
        self._subs = []
        self._subs.append(self.create_subscription(String, self.topics.motion_cmd, self._on_cmd, qos_default()))
        self._subs.append(self.create_subscription(PoseStamped, self.topics.motion_target_pose, self._on_pose, qos_latched()))

        # --- Publishers ---
        self._result_pub         = self.create_publisher(String, self.topics.motion_result, qos_default())
        self._ready_pub          = self.create_publisher(Bool, self.topics.state_ready, qos_latched())
        self._traj_planned_pub   = self.create_publisher(RobotTrajectory, self.topics.motion_last_traj, qos_latched())  # <-- RobotTrajectory
        self._traj_executed_pub  = self.create_publisher(RobotTrajectory, "/meca/motion/executed_traj", qos_latched())

        # --- Ready flag ---
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info("✅ MotionManager bereit und hört auf Topics")

    # ==================================================================
    # --- CALLBACKS -----------------------------------------------------
    # ==================================================================
    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"📩 Motion-Command empfangen: {cmd}")

        if not self._arm or not self._executor:
            self.get_logger().warning("⚠️ Kein MoveItPy verfügbar, Befehl ignoriert.")
            self._result_pub.publish(String(data=f"{cmd}:fail"))
            return

        try:
            if cmd.startswith("plan_to_named:"):
                ok = self._plan_named(cmd.split(":", 1)[1], execute=False)
            elif cmd.startswith("move_to_named:"):
                ok = self._plan_named(cmd.split(":", 1)[1], execute=True)
            elif cmd.startswith("plan_to_pose:"):
                ok = self._move_to_pose_name(cmd.split(":", 1)[1], execute=False)
            elif cmd.startswith("move_to_pose:"):
                ok = self._move_to_pose_name(cmd.split(":", 1)[1], execute=True)
            elif cmd == "plan_to_pose":
                ok = self._plan_pose(execute=False)
            elif cmd == "move_to_pose":
                ok = self._plan_pose(execute=True)
            elif cmd == "execute_last":
                ok = self._execute_last()
            else:
                self.get_logger().warning(f"❓ Unbekanntes Kommando: {cmd}")
                ok = False
        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei '{cmd}': {e}")
            ok = False

        self._result_pub.publish(String(data=f"{cmd}:{'ok' if ok else 'fail'}"))

    # ==================================================================
    # --- PLANUNG & AUSFÜHRUNG ----------------------------------------
    # ==================================================================
    def _plan_named(self, name: str, execute: bool) -> bool:
        self.get_logger().info(f"🧭 Plan to named pose: {name} (execute={execute})")
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(configuration_name=name)
        plan = self._arm.plan()

        if not plan or not plan.trajectory:
            self.get_logger().warning("❌ Planung fehlgeschlagen.")
            return False

        self._last_rt = plan.trajectory  # <-- RobotTrajectory
        self._publish_plan(self._last_rt)
        if execute:
            return self._execute_last()
        return True

    def _plan_pose(self, execute: bool) -> bool:
        if not self._pending_pose:
            self.get_logger().warning("❌ Keine Zielpose vorhanden.")
            return False

        pose = self._pending_pose
        self.get_logger().info(f"🎯 Plane zu Pose ({pose.header.frame_id}) (execute={execute})")
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped=pose)
        plan = self._arm.plan()

        if not plan or not plan.trajectory:
            self.get_logger().warning("❌ Planung zu Pose fehlgeschlagen.")
            return False

        self._last_rt = plan.trajectory
        self._publish_plan(self._last_rt)
        if execute:
            return self._execute_last()
        return True

    def _move_to_pose_name(self, name: str, execute: bool = True) -> bool:
        """Pose von /meca/poses/<name> abfragen, planen & (optional) ausführen."""
        topic = f"/meca/poses/{name}"
        from rclpy.task import Future
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        future: Future = rclpy.task.Future()

        def _cb(msg: PoseStamped):
            if not future.done():
                future.set_result(msg)

        sub = self.create_subscription(PoseStamped, topic, _cb, qos)
        self.get_logger().info(f"🎯 Warte auf Pose '{name}' ({topic}) …")
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        self.destroy_subscription(sub)

        if not future.done():
            self.get_logger().error(f"❌ Keine Pose '{name}' empfangen – Timeout.")
            return False

        pose_msg: PoseStamped = future.result()
        self.get_logger().info(f"📦 Pose '{name}' empfangen aus Frame '{pose_msg.header.frame_id}'")

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped=pose_msg)
        plan = self._arm.plan()

        if not plan or not plan.trajectory:
            self.get_logger().warning(f"❌ Planung zu Pose '{name}' fehlgeschlagen.")
            return False

        self._last_rt = plan.trajectory
        self._publish_plan(self._last_rt)
        if execute:
            return self._execute_last()
        return True

    def _publish_plan(self, rt: RobotTrajectory):
        self._traj_planned_pub.publish(rt)
        self.get_logger().info("🟩 Geplante Trajektorie veröffentlicht")

    def _execute_last(self) -> bool:
        if not self._last_rt:
            self.get_logger().warning("❌ Keine gespeicherte Trajektorie vorhanden.")
            return False
        self._executor.execute(self._last_rt)       # <-- korrekt ausführen
        self._traj_executed_pub.publish(self._last_rt)  # <-- einfache Rückmeldung
        self.get_logger().info("🟦 Trajektorie ausgeführt & veröffentlicht")
        return True

    # ==================================================================
    # --- POSE CALLBACK ------------------------------------------------
    # ==================================================================
    def _on_pose(self, msg: PoseStamped):
        self._pending_pose = msg
        self.get_logger().info(
            f"✅ Neue Zielpose empfangen im Frame '{msg.header.frame_id or self.frames['world']}'"
        )


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
