#!/usr/bin/env python3
from __future__ import annotations
import os, time, yaml, importlib
from typing import Optional
from moveit.planning import MoveItPy
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
from mecademic_bringup.utils import rpy_deg_to_quat
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.common.qos import qos_default, qos_latched

SCENE_WAIT_TIMEOUT = 30.0


class ToolManager(Node):
    """
    ToolManager (Single-TCP-Modus)
    ------------------------------
    - Lädt Tooldefinitionen (tools.yaml)
    - Initialisiert 1x TCP-Frame (tool_mount → tcp)
    - Plugins ändern nur TCP-Offset & Mesh
    - Publiziert PlanningScene & Toolstatus
    """

    def __init__(self):
        super().__init__("tool_manager")

        # --- Common ---
        self.frames = FRAMES
        self.topics = Topics()

        # --- Publisher ---
        self.scene_pub = self.create_publisher(PlanningScene, self.topics.apply_planning_scene, qos_default())
        self.pub_current = self.create_publisher(String, self.topics.tool_current, qos_latched())
        self.create_subscription(String, self.topics.tool_set, self.on_tool_change, qos_default())

        # --- Parameter / Tools ---
        default_cfg = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_cfg)
        self.tools_data = self._load_yaml(self.get_parameter(PARAM_TOOL_CONFIG).value)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        # --- MoveIt Scene Client ---
        self.ps_client = self.create_client(GetPlanningScene, self.topics.get_planning_scene)

        # --- Plugin-State ---
        self.current_plugin: Optional[object] = None

        # --- TCP Frame (statisch, nur Offset ändert sich) ---
        self.tf_pub = TransformBroadcaster(self)
        self._tcp_parent = self.frames["tool_mount"]
        self._tcp_xyz = [0.0, 0.0, 0.0]
        self._tcp_rpy = [0.0, 0.0, 0.0]
        self._tcp_timer = self.create_timer(0.05, self._broadcast_tcp)
        self.get_logger().info("✅ TCP-Frame initialisiert @ tool_mount")

        # --- PlanningScene Wait Logic ---
        self._armed_to_publish = False
        self._publish_done = False
        self._scene_wait_start = time.time()
        self._scene_wait_timer = self.create_timer(0.5, self._check_scene_ready)

        self.get_logger().info("⏳ Warte auf MoveIt PlanningScene – ToolAttach folgt danach.")

    # ------------------------------------------------------------------
    # Funktion zum Setzen des TCP als Endeffektor
    # ------------------------------------------------------------------
    def set_tcp_as_ee(self):
        """Setzt das TCP als Endeffektor (EE) im MoveIt-Plan."""
        try:
            # Initialisiere den MoveIt-Client mit der Robotergruppe
            moveit_client = MoveItPy("meca_arm_group")
            moveit_client.wait_for_ready()

            # Setze den TCP als Endeffektor
            tcp_frame = "tool_mount"  # Ersetze mit deinem tatsächlichen TCP-Link
            moveit_client.set_end_effector(tcp_frame)
            self.get_logger().info(f"✅ TCP {tcp_frame} als Endeffektor gesetzt.")
        except Exception as e:
            self.get_logger().error(f"❌ Fehler beim Setzen des TCP als Endeffektor: {e}")

    # ------------------------------------------------------------------
    # TCP Update (Offset-only)
    # ------------------------------------------------------------------
    def update_tcp_offset(self, xyz, rpy_deg):
        """Aktualisiert nur das Offset für den TCP (von Plugins aufgerufen)."""
        self._tcp_xyz = list(map(float, xyz))
        self._tcp_rpy = list(map(float, rpy_deg))
        self.get_logger().info(f"🔹 TCP-Offset aktualisiert: xyz={xyz}, rpy_deg={rpy_deg}")

    def _broadcast_tcp(self):
        """Publiziert dauerhaft den TF tool_mount → tcp."""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._tcp_parent
        tf.child_frame_id = self.frames["tcp"]
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = self._tcp_xyz
        qx, qy, qz, qw = rpy_deg_to_quat(*self._tcp_rpy)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        self.tf_pub.sendTransform(tf)

    # ------------------------------------------------------------------
    # Utils
    # ------------------------------------------------------------------
    def _load_yaml(self, path: str) -> dict:
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"❌ YAML-Laden fehlgeschlagen: {e}")
            return {}

    # ------------------------------------------------------------------
    # Plugin-System
    # ------------------------------------------------------------------
    def _load_plugin_for_tool(self, tool_name: str):
        """Lädt Tool-Plugin oder nutzt Standardbase."""
        from mecademic_bringup.tools.base_tool import ToolPluginBase

        self.current_plugin = None
        tool_cfg = self.tools.get(tool_name, {})
        plugin_name = tool_cfg.get("plugin", "").strip()

        if not plugin_name:
            self.current_plugin = ToolPluginBase()
            self.get_logger().info("ℹ️ Kein Plugin angegeben – ToolPluginBase verwendet.")
            return

        module_path = f"mecademic_bringup.tools.{plugin_name}"
        try:
            mod = importlib.import_module(module_path)
            PluginClass = getattr(mod, "ToolPlugin", ToolPluginBase)
            self.current_plugin = PluginClass()
            self.get_logger().info(f"🔌 Plugin geladen: {module_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Plugin-Laden fehlgeschlagen ({module_path}): {e}")
            self.current_plugin = ToolPluginBase()

    def _plugin_attach(self, tool_name: str) -> bool:
        """Führt on_attach() des Plugins aus."""
        if not self.current_plugin:
            return False
        try:
            return bool(self.current_plugin.on_attach(self, self.tools.get(tool_name, {})))
        except Exception as e:
            self.get_logger().error(f"❌ Plugin on_attach Fehler: {e}")
            return False

    def _plugin_detach(self, tool_name: str) -> bool:
        """Führt on_detach() des Plugins aus."""
        if not self.current_plugin:
            return False
        try:
            return bool(self.current_plugin.on_detach(self, self.tools.get(tool_name, {})))
        except Exception as e:
            self.get_logger().error(f"❌ Plugin on_detach Fehler: {e}")
            return False

    # ------------------------------------------------------------------
    # PlanningScene-Warte-Flow
    # ------------------------------------------------------------------
    def _check_scene_ready(self):
        if (time.time() - self._scene_wait_start) > SCENE_WAIT_TIMEOUT:
            self.get_logger().warning("⚠️ PlanningScene-Timeout – sende trotzdem.")
            return self._arm_and_delay_publish()

        if not self.ps_client.wait_for_service(timeout_sec=0.2):
            return

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.SCENE_SETTINGS
        future = self.ps_client.call_async(req)
        future.add_done_callback(self._on_scene_service_ok)
        self._scene_wait_timer.cancel()

    def _on_scene_service_ok(self, _):
        self.get_logger().info("✅ PlanningScene erreichbar – sende in 3 s.")
        self._arm_and_delay_publish()

    def _arm_and_delay_publish(self):
        if self._armed_to_publish or self._publish_done:
            return
        self._armed_to_publish = True

        def _do_once():
            self._publish_once()
            delay_timer.cancel()

        delay_timer = self.create_timer(3.0, _do_once)

    # ------------------------------------------------------------------
    # Haupt-Publish-Routine
    # ------------------------------------------------------------------
    def _publish_once(self):
        """Wird ausgeführt, sobald MoveIt bereit ist."""
        if self._publish_done:
            return
        self._publish_done = True

        self._load_plugin_for_tool(self.current_tool)
        self._plugin_attach(self.current_tool)

        self.pub_current.publish(String(data=self.current_tool))
        self.get_logger().info(f"✅ ToolManager aktiv: {self.current_tool}")

    # ------------------------------------------------------------------
    # Toolwechsel
    # ------------------------------------------------------------------
    def on_tool_change(self, msg: String):
        """Wird aufgerufen, wenn ein neues Tool angefordert wird."""
        name = msg.data.strip()
        if name not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {name}")
            return

        prev_tool = self.current_tool
        if prev_tool in self.tools:
            self._load_plugin_for_tool(prev_tool)
            self._plugin_detach(prev_tool)

        self.current_tool = name
        self.get_logger().info(f"🔧 Toolwechsel → {name}")

        self._publish_done = False
        self._arm_and_delay_publish()


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = ToolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
