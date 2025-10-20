#!/usr/bin/env python3
import sys
import termios
import tty
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_srvs.srv import Trigger
from moveit_msgs.srv import ServoCommandType

class ServoTeleop(Node):
    def __init__(self):
        super().__init__("servo_teleop")

        self.pub_twist = self.create_publisher(TwistStamped, "/moveit_servo/delta_twist_cmds", 10)
        self.pub_joint = self.create_publisher(JointJog, "/moveit_servo/delta_joint_cmds", 10)

        self.srv_command_type = self.create_client(ServoCommandType, "/moveit_servo/switch_command_type")
        self.srv_start = self.create_client(Trigger, "/moveit_servo/start_servo")
        self.srv_pause = self.create_client(Trigger, "/moveit_servo/pause_servo")

        self.joint_names = [
            "meca_axis_1_joint", "meca_axis_2_joint", "meca_axis_3_joint",
            "meca_axis_4_joint", "meca_axis_5_joint", "meca_axis_6_joint"
        ]

        self.mode = "cartesian"  # "cartesian" or "joint"
        self.step_lin = 0.01
        self.step_rot = 0.05
        self.step_joint = 0.02

        self.get_logger().info("✅ Servo Teleop gestartet (M = Modus wechseln, SPACE = Stop)")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.print_controls()

    # ---------- Logging-fähige Commands ----------
    def send_cartesian_cmd(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        self.get_logger().info(
            f"[TELEOP] Sending CARTESIAN cmd: "
            f"vx={vx:.3f} vy={vy:.3f} vz={vz:.3f} wx={wx:.3f} wy={wy:.3f} wz={wz:.3f}"
        )
        msg = TwistStamped()
        msg.header.frame_id = "meca_tool0"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self.pub_twist.publish(msg)

    def send_joint_cmd(self, joint_index, delta):
        if 0 <= joint_index < len(self.joint_names):
            self.get_logger().info(
                f"[TELEOP] Sending JOINT cmd: {self.joint_names[joint_index]} += {delta:+.3f} rad"
            )
            msg = JointJog()
            msg.joint_names = [self.joint_names[joint_index]]
            msg.velocities = [delta]
            msg.header.frame_id = "world"
            self.pub_joint.publish(msg)

    # ---------- Modes ----------
    def toggle_mode(self):
        self.mode = "joint" if self.mode == "cartesian" else "cartesian"
        cmd_type = 2 if self.mode == "cartesian" else 1
        req = ServoCommandType.Request()
        req.command_type = cmd_type
        if not self.srv_command_type.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service switch_command_type nicht verfügbar")
            return
        self.srv_command_type.call_async(req)
        self.get_logger().info(f"🔧 Modus gewechselt zu: {self.mode.upper()}")

    def stop_motion(self):
        msg = TwistStamped()
        self.pub_twist.publish(msg)
        self.get_logger().info("🛑 Bewegung gestoppt")

    # ---------- Keyboard ----------
    def run(self):
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

    def handle_key(self, ch):
        if self.mode == "cartesian":
            if ch == "w":
                self.send_cartesian_cmd(vz=self.step_lin)
            elif ch == "s":
                self.send_cartesian_cmd(vz=-self.step_lin)
            elif ch == "a":
                self.send_cartesian_cmd(vy=self.step_lin)
            elif ch == "d":
                self.send_cartesian_cmd(vy=-self.step_lin)
            elif ch == "q":
                self.send_cartesian_cmd(vx=-self.step_lin)
            elif ch == "e":
                self.send_cartesian_cmd(vx=self.step_lin)
            elif ch == "i":
                self.send_cartesian_cmd(wx=self.step_rot)
            elif ch == "k":
                self.send_cartesian_cmd(wx=-self.step_rot)
            elif ch == "j":
                self.send_cartesian_cmd(wy=self.step_rot)
            elif ch == "l":
                self.send_cartesian_cmd(wy=-self.step_rot)
            elif ch == "u":
                self.send_cartesian_cmd(wz=self.step_rot)
            elif ch == "o":
                self.send_cartesian_cmd(wz=-self.step_rot)

        elif self.mode == "joint":
            if ch.isdigit():
                idx = int(ch) - 1
                self.send_joint_cmd(idx, self.step_joint)

    def print_controls(self):
        print("""
=== 🔥 Servo Teleoperation 🔥 ===
CARTESIAN MODE:
  Move XYZ:    w/s  a/d  q/e
  Rotate RPY:  i/k  j/l  u/o
JOINT MODE:
  Single joint jog:  1 2 3 4 5 6

M = Mode switch | SPACE = Stop | CTRL+C = Exit
=================================
""")


def main():
    import select
    rclpy.init()
    node = ServoTeleop()
    node.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
