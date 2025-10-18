#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from launch_ros.substitutions import FindPackageShare

# zentrale Imports
from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG, PARAM_USE_FAKE_HW


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # Default-Pfad zu tools.yaml
        pkg = FindPackageShare("mecademic_bringup").find("mecademic_bringup")
        default_tools_yaml = os.path.join(pkg, "config", "tools.yaml")

        # Parameter laden
        self.declare_parameter(PARAM_TOOL_CONFIG, default_tools_yaml)
        self.declare_parameter(PARAM_USE_FAKE_HW, True)

        self.tools_yaml_path = self.get_parameter(PARAM_TOOL_CONFIG).value
        self.use_fake_hw = self.get_parameter(PARAM_USE_FAKE_HW).value

        # Tools laden
        self.tools_data = self._load_tools_yaml()
        self.current_tool = self.tools_data.get("active_tool", "none")

        # Publisher & Subscriber
        self.tool_pub = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        # Initial publish
        self.publish_current_tool()
        self.get_logger().info(f"✅ ToolManager gestartet – aktives Tool: '{self.current_tool}'")

    def _load_tools_yaml(self):
        if not os.path.exists(self.tools_yaml_path):
            raise FileNotFoundError(f"❌ tools.yaml nicht gefunden: {self.tools_yaml_path}")
        with open(self.tools_yaml_path, "r") as f:
            return yaml.safe_load(f) or {}

    def _save_active_tool(self):
        self.tools_data["active_tool"] = self.current_tool
        with open(self.tools_yaml_path, "w") as f:
            yaml.dump(self.tools_data, f)
        self.get_logger().info(f"💾 tools.yaml gespeichert – active_tool='{self.current_tool}'")

    def publish_current_tool(self):
        self.tool_pub.publish(String(data=self.current_tool))

    def on_tool_change(self, msg: String):
        new_tool = (msg.data or "").strip()
        tools = self.tools_data.get("tools") or {}
        if new_tool not in tools:
            self.get_logger().error(f"❌ Tool '{new_tool}' nicht definiert in tools.yaml")
            return
        if new_tool == self.current_tool:
            self.get_logger().info(f"ℹ Tool '{new_tool}' ist bereits aktiv – keine Änderung")
            return

        self.current_tool = new_tool
        self._save_active_tool()
        self.publish_current_tool()
        self.get_logger().info(f"🔧 Tool geändert: now active → '{new_tool}' (ohne Neustart)")


def main():
    rclpy.init()
    node = ToolManager()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
