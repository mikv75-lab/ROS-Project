#!/usr/bin/env python3
from __future__ import annotations
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.params import load_params, AppParams


class MotionManager(Node):
    """
    MotionManager (ROS2)
    ---------------------
    Steuert den Bewegungsablauf des Roboters über Topics.

    Funktionen:
      - Empfängt Kommandos über /meca/motion/cmd
      - Verwaltet aktuelle Zielpose (/meca/motion/target_pose)
      - Publiziert Status (/meca/state/ready, /meca/motion/result)
      - Publiziert geplante Trajektorie (/meca/motion/last_traj)

    Später:
      - Kann MoveItPy oder plan_kinematic_path/execute_kinematic_path Services ansprechen.
      - Kann als zentrales Bindeglied zwischen App, MoveIt und RViz dienen.
    """

    def __init__(self):
        super().__init__("motion_manager")
        self.get_logger().info("🚀 MotionManager gestartet")

        # --- Parameter laden ---
        self.params: AppParams = load_params(self)
        self.frames = FRAMES
        self.topics = Topics()

        # --- State ---
        self._pending_pose: PoseStamped | None = None
        self._last_traj: JointTrajectory | None = None

        # --- Subscriptions ---
        self._cmd_sub = self.create_subscription(
            String, self.topics.base_ns or "/meca/motion/cmd", self._on_cmd, qos_default()
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, self.topics.base_ns or "/meca/motion/target_pose", self._on_pose, qos_latched()
        )

        # --- Publishers ---
        self._result_pub = self.create_publisher(String, "/meca/motion/result", qos_default())
        self._ready_pub = self.create_publisher(Bool, "/meca/state/ready", qos_latched())
        self._traj_pub = self.create_publisher(JointTrajectory, "/meca/motion/last_traj", qos_latched())

        # Markiere bereit
        self._ready_pub.publish(Bool(data=True))
        self.get_logger().info("✅ MotionManager bereit und hört auf Topics")

    # ------------------------------------------------------------------

    def _on_cmd(self, msg: String):
        cmd = msg.data.strip()
        self.get_logger().info(f"📩 Motion-Command empfangen: {cmd}")

        ok = False

        try:
            # --- Planner wechseln ---
            if cmd.startswith("planner:"):
                planner = cmd.split(":", 1)[1]
                self.get_logger().info(f"Planner-Wechsel angefordert: {planner}")
                # (später hier Service/Param call an MoveIt)
                ok = True

            # --- Move/Plan Kommandos (werden an MoveIt delegiert oder nur als Event publiziert) ---
            elif cmd.startswith("move_to_named:"):
                name = cmd.split(":", 1)[1]
                self.get_logger().info(f"Move to named pose: {name}")
                ok = self._publish_motion_event("move_to_named", name)

            elif cmd == "move_to_pose":
                if self._pending_pose:
                    ok = self._publish_motion_event("move_to_pose")
                else:
                    self.get_logger().warn("❌ Kein Ziel-PoseStamped empfangen.")

            elif cmd.startswith("plan_to_named:"):
                name = cmd.split(":", 1)[1]
                ok = self._publish_motion_event("plan_to_named", name)

            elif cmd == "plan_to_pose":
                if self._pending_pose:
                    ok = self._publish_motion_event("plan_to_pose")
                else:
                    self.get_logger().warn("❌ Kein Ziel-PoseStamped empfangen.")

            elif cmd == "execute_last":
                ok = self._publish_motion_event("execute_last")

            else:
                self.get_logger().warn(f"Unbekanntes Kommando: {cmd}")

        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei '{cmd}': {e}")
            ok = False

        # Ergebnis publizieren
        self._result_pub.publish(String(data=f"{cmd}:{'ok' if ok else 'fail'}"))

    # ------------------------------------------------------------------

    def _publish_motion_event(self, action: str, value: str | None = None) -> bool:
        """
        Platzhalter für zukünftige MoveItPy-Integration.
        Aktuell nur Logging + Dummy-Trajektorie für RViz.
        """
        self.get_logger().info(f"🧩 MotionEvent: {action} ({value or '-'})")

        # Beispielhafte Dummy-Trajektorie (nur für Visualisierung)
        traj = JointTrajectory()
        traj.joint_names = [f"meca_axis_{i}_joint" for i in range(1, 7)]
        pt = traj.points.add()
        pt.time_from_start.sec = 2
        self._traj_pub.publish(traj)
        self._last_traj = traj

        return True

    # ------------------------------------------------------------------

    def _on_pose(self, msg: PoseStamped):
        """Speichert die empfangene Zielpose."""
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
