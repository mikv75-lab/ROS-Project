#!/usr/bin/env python3
import os
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PERSIST_FILE = "/root/ws_moveit/src/mecademic_bringup/config/active_tool.txt"
ROS_SETUP = "/opt/ros/rolling/setup.bash"
WS_SETUP = "/root/ws_moveit/install/setup.bash"


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # Aktuelles Tool laden
        self.current_tool = self.load_tool()
        self.process: Optional[subprocess.Popen] = None

        # Start: Roboter direkt hochfahren
        self.start_robot(self.current_tool)

        # Subscriber für Toolwechsel
        self.create_subscription(String, "/meca/tool/set", self.on_tool_change, 10)

        # Topics publizieren (optional)
        self.pub_current = self.create_publisher(String, "/meca/tool/current", 10)
        self.publish_current()

        self.get_logger().info(f"[ToolManager] ✅ gestartet mit Tool: {self.current_tool}")

    def load_tool(self) -> str:
        if os.path.exists(PERSIST_FILE):
            return open(PERSIST_FILE).read().strip() or "none"
        return "none"

    def save_tool(self):
        os.makedirs(os.path.dirname(PERSIST_FILE), exist_ok=True)
        with open(PERSIST_FILE, "w") as f:
            f.write(self.current_tool)

    def publish_current(self):
        self.pub_current.publish(String(data=self.current_tool))

    def stop_robot(self):
        if self.process:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
            except Exception:
                pass
            self.process = None

    def start_robot(self, tool: str):
        # Kill RViz vorher, damit keine doppelte Session stört
        os.system("pkill -9 -f rviz2 || true")

        # Stop alten Roboterprozess
        self.stop_robot()

        # Neues Tool starten
        launch = f'''
            source {ROS_SETUP}
            source {WS_SETUP}
            exec ros2 launch mecademic_bringup robot_with_tool.launch.py tool:={tool} rviz:=true use_fake_hw:=true
        '''
        self.process = subprocess.Popen(["/bin/bash", "-lc", launch], preexec_fn=os.setsid)
        self.get_logger().info(f"[ToolManager] 🚀 Roboter gestartet mit Tool: {tool}")

    def on_tool_change(self, msg: String):
        new_tool = msg.data.strip()
        if new_tool == self.current_tool:
            self.get_logger().info("[ToolManager] Tool ist gleich – kein Neustart.")
            return

        self.get_logger().info(f"[ToolManager] 🔧 Toolwechsel: {self.current_tool} → {new_tool}")
        self.current_tool = new_tool
        self.save_tool()
        self.publish_current()
        self.start_robot(new_tool)


def main():
    rclpy.init()
    node = ToolManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
