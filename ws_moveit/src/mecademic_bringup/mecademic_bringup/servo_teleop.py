#!/usr/bin/env python3
from __future__ import annotations
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool

from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.qos import qos_sensor_data


class ServoTeleop(Node):
    """
    ServoTeleop
    ------------
    Tastatursteuerung für MoveIt Servo (Rolling API-kompatibel).

    Unterstützt:
      - Kartesische Steuerung (Twist)
      - Gelenkweise Steuerung (JointJog)
      - Umschalten zwischen Modi (m)
      - Stopp (Leertaste)
    """

    def __init__(self):
        super().__init__("servo_teleop")

        # --- Common ---
        self.topics = Topics()
        self.frames = FRAMES

        # --- Publisher ---
        self.pub_twist = self.create_publisher(
            TwistStamped, self.topics.servo_twist, qos_sensor_data()
        )
        self.pub_joint = self.create_publisher(
            JointJog, self.topics.servo_joint, qos_sensor_data()
        )

        # --- Services ---
        self.srv_pause = self.create_client(SetBool, f"{self.topics.servo_ns}/pause_servo")
        self.srv_switch = self.create_client(ServoCommandType, f"{self.topics.servo_ns}/switch_command_type")

        # --- Joint names ---
        self.joint_names = [f"meca_axis_{i}_joint" for i in range(1, 7)]

        # --- Motion parameters ---
        self.step_lin = 0.01
        self.step_rot = 0.05
        self.step_joint = 0.02
        self.mode = "cartesian"  # default

        # --- Init ---
        self.get_logger().info("🚀 Initialisiere ServoTeleop...")
        self._initialize_servo()
        self.print_controls()

    # -------------------------------------------------------
    # Servo initialization
    # -------------------------------------------------------
    def _initialize_servo(self):
        if not self.srv_switch.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("❌ ServoCommandType-Service nicht verfügbar.")
            return

        req = ServoCommandType.Request()
        req.command_type = 1  # TWIST-Modus
        self.get_logger().info("🟢 Setze ServoCommandType = TWIST (1)")
        future = self.srv_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.result():
            self.get_logger().error("❌ Keine Antwort vom ServoCommandType-Service.")
            return

        if getattr(future.result(), "success", False):
            self.get_logger().info("✅ MoveIt Servo akzeptiert Twist-Commands.")
        else:
            self.get_logger().warning("⚠️ ServoCommandType Antwort war negativ.")

    # -------------------------------------------------------
    # Commands
    # -------------------------------------------------------
    def send_cartesian_cmd(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        msg = TwistStamped()
        msg.header.frame_id = self.frames["tcp"]
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vx, vy, vz
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z = wx, wy, wz
        self.pub_twist.publish(msg)
        self.get_logger().info(
            f"[CARTESIAN] vx={vx:+.3f} vy={vy:+.3f} vz={vz:+.3f} wx={wx:+.3f} wy={wy:+.3f} wz={wz:+.3f}"
        )

    def send_joint_cmd(self, joint_index: int, delta: float):
        if 0 <= joint_index < len(self.joint_names):
            msg = JointJog()
            msg.header.frame_id = self.frames["world"]
            msg.joint_names = [self.joint_names[joint_index]]
            msg.velocities = [delta]
            self.pub_joint.publish(msg)
            self.get_logger().info(f"[JOINT] {self.joint_names[joint_index]} += {delta:+.3f} rad")

    # -------------------------------------------------------
    # Mode handling
    # -------------------------------------------------------
    def toggle_mode(self):
        """Zwischen kartesischem und Gelenkmodus umschalten."""
        self.mode = "joint" if self.mode == "cartesian" else "cartesian"
        req = ServoCommandType.Request()
        req.command_type = 0 if self.mode == "joint" else 1
        self.srv_switch.call_async(req)
        self.get_logger().info(f"🔄 Modus gewechselt: {self.mode.upper()}")

    def stop_motion(self):
        """Stoppt aktuelle Bewegung."""
        self.pub_twist.publish(TwistStamped())
        self.get_logger().info("🛑 Bewegung gestoppt")

    # -------------------------------------------------------
    # Keyboard handling
    # -------------------------------------------------------
    def run(self):
        """Startet interaktive Tastatursteuerung."""
        if not sys.stdin.isatty():
            self.get_logger().warning("⚠️ Kein interaktives Terminal – Teleop passiv.")
            rclpy.spin(self)
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while rclpy.ok():
                if sys.stdin in select.select([sys.stdin], [], [], 0.05)[0]:
                    ch = sys.stdin.read(1)
                    if ch == "\x03":  # Ctrl+C
                        break
                    elif ch == "m":
                        self.toggle_mode()
                    elif ch == " ":
                        self.stop_motion()
                    else:
                        self.handle_key(ch)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    # -------------------------------------------------------
    # Key mapping
    # -------------------------------------------------------
    def handle_key(self, ch: str):
        if self.mode == "cartesian":
            if ch == "w": self.send_cartesian_cmd(vz=self.step_lin)
            elif ch == "s": self.send_cartesian_cmd(vz=-self.step_lin)
            elif ch == "a": self.send_cartesian_cmd(vy=self.step_lin)
            elif ch == "d": self.send_cartesian_cmd(vy=-self.step_lin)
            elif ch == "q": self.send_cartesian_cmd(vx=-self.step_lin)
            elif ch == "e": self.send_cartesian_cmd(vx=self.step_lin)
            elif ch == "i": self.send_cartesian_cmd(wx=self.step_rot)
            elif ch == "k": self.send_cartesian_cmd(wx=-self.step_rot)
            elif ch == "j": self.send_cartesian_cmd(wy=self.step_rot)
            elif ch == "l": self.send_cartesian_cmd(wy=-self.step_rot)
            elif ch == "u": self.send_cartesian_cmd(wz=self.step_rot)
            elif ch == "o": self.send_cartesian_cmd(wz=-self.step_rot)
        elif self.mode == "joint" and ch.isdigit():
            idx = int(ch) - 1
            self.send_joint_cmd(idx, self.step_joint)

    # -------------------------------------------------------
    # Help
    # -------------------------------------------------------
    def print_controls(self):
        print("""
=== 🔥 Servo Teleoperation 🔥 ===
CARTESIAN MODE:
  Move XYZ:    w/s  a/d  q/e
  Rotate RPY:  i/k  j/l  u/o
JOINT MODE:
  Single joint jog:  1 2 3 4 5 6

M = Mode switch | SPACE = Stop | CTRL+C = Exit
==============================================
""")


def main(args=None):
    rclpy.init(args=args)
    node = ServoTeleop()
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
