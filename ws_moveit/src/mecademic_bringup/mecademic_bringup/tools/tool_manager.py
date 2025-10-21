#!/usr/bin/env python3
from __future__ import annotations
import os, time, yaml, importlib
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, TransformStamped
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import (
    CollisionObject,
    AttachedCollisionObject,
    PlanningScene,
    PlanningSceneComponents,
    AllowedCollisionMatrix,
    AllowedCollisionEntry,
)
from moveit_msgs.srv import GetPlanningScene
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import trimesh

from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.utils import rpy_deg_to_quat


ATTACHED_OBJ_ID = "active_tool_mesh"
SCENE_WAIT_TIMEOUT = 30.0
DEFAULT_TOOL_ACM_LINKS = ["tool_mount", "meca_axis_6_link", "meca_axis_5_link"]


class ToolManager(Node):
    """
    ToolManager
    ------------
    Lädt Tooldefinitionen (YAML) und verwaltet:
      - Mesh-Attach/Detach an MoveIt
      - TCP-TF (kontinuierlich)
      - Allowed Collision Matrix (ACM)
      - Plugins für Spezialtools
    """

    def __init__(self):
        super().__init__("tool_manager")

        # --- Common ---
        self.frames = FRAMES
        self.topics = Topics()

        # --- Publisher & Subscriber ---
        self.tf_pub = TransformBroadcaster(self)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, self.topics.collision_object, qos_default())
        self.scene_pub = self.create_publisher(PlanningScene, self.topics.apply_planning_scene, qos_default())
        self.pub_current = self.create_publisher(String, self.topics.tool_current, qos_latched())
        self.create_subscription(String, self.topics.tool_set, self.on_tool_change, qos_default())

        # --- Parameter ---
        default_cfg = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_cfg)
        self.tools_data = self._load_yaml(self.get_parameter(PARAM_TOOL_CONFIG).value)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        self.mesh_cache = {}

        # --- MoveIt Scene Client ---
        self.ps_client = self.create_client(GetPlanningScene, self.topics.get_planning_scene)

        # --- State ---
        self._armed_to_publish = False
        self._publish_done = False
        self._scene_wait_start = time.time()
        self._scene_wait_timer = self.create_timer(0.5, self._check_scene_ready)

        # --- TCP Broadcaster ---
        self._tcp_parent = self.frames["tool_mount"]
        self._tcp_xyz = [0.0, 0.0, 0.0]
        self._tcp_rpy = [0.0, 0.0, 0.0]
        self._tcp_timer = self.create_timer(0.05, self._tcp_broadcast)  # ~20 Hz
        self.publish_initial_tcp()

        # --- Plugin-State ---
        self.current_plugin: Optional[object] = None

        self.get_logger().info("⏳ Warte auf MoveIt PlanningScene – nichts wird davor publiziert.")

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

    def resolve_mesh_path(self, uri: str) -> str:
        if uri.startswith("package://"):
            pkg, rel = uri[10:].split("/", 1)
            return os.path.join(get_package_share_directory(pkg), rel)
        return uri

    def load_mesh(self, mesh_path: str) -> Mesh:
        if mesh_path in self.mesh_cache:
            return self.mesh_cache[mesh_path]
        tri = trimesh.load(mesh_path)
        tri.apply_scale(0.001)  # mm → m
        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=list(map(int, f))))
        self.mesh_cache[mesh_path] = mesh
        return mesh

    # ------------------------------------------------------------------
    # TCP (dyn. TF)
    # ------------------------------------------------------------------
    def publish_initial_tcp(self):
        self._tcp_parent = self.frames["tool_mount"]
        self._tcp_xyz = [0.0, 0.0, 0.0]
        self._tcp_rpy = [0.0, 0.0, 0.0]
        self.get_logger().info("✅ Initiales TCPFrame @ tool_mount gesetzt")

    def set_tcp(self, parent: str, xyz, rpy_deg):
        self._tcp_parent = parent
        self._tcp_xyz = list(map(float, xyz))
        self._tcp_rpy = list(map(float, rpy_deg))
        self.get_logger().info(f"🔹 TCP gesetzt @ {parent} xyz={xyz} rpy_deg={rpy_deg}")

    def publish_tcp_once(self, parent, xyz, rpy_deg):
        self.set_tcp(parent, xyz, rpy_deg)

    def _tcp_broadcast(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._tcp_parent
        tf.child_frame_id = self.frames["tcp"]
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = self._tcp_xyz
        qx, qy, qz, qw = rpy_deg_to_quat(*self._tcp_rpy)
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        self.tf_pub.sendTransform(tf)

    # ------------------------------------------------------------------
    # Attach/Detach + ACM
    # ------------------------------------------------------------------
    def make_attach_msg(self, mount_frame, mesh, mesh_offset, mesh_rpy_deg):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = mesh_offset
        qx, qy, qz, qw = rpy_deg_to_quat(*mesh_rpy_deg)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw

        msg = AttachedCollisionObject()
        msg.link_name = mount_frame
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.header.frame_id = mount_frame
        msg.object.meshes = [mesh]
        msg.object.mesh_poses = [pose]
        msg.object.operation = CollisionObject.ADD
        return msg

    def attach_tool(self, mount_frame, mesh, mesh_offset, mesh_rpy_deg):
        self.attached_pub.publish(self.make_attach_msg(mount_frame, mesh, mesh_offset, mesh_rpy_deg))
        self.get_logger().info("🧩 Tool-Mesh attached.")

    def detach_tool_mesh(self):
        aco = AttachedCollisionObject()
        aco.object.id = ATTACHED_OBJ_ID
        aco.object.operation = CollisionObject.REMOVE
        self.attached_pub.publish(aco)
        self.get_logger().info("🧹 Tool-Mesh detached.")

    def set_tool_acm(self, allow_with_links):
        if not allow_with_links:
            return
        acm = AllowedCollisionMatrix()
        acm.entry_names.append(ATTACHED_OBJ_ID)
        acm.entry_names.extend(allow_with_links)

        size = len(acm.entry_names)
        matrix = [[False] * size for _ in range(size)]
        tool_idx = 0
        for i, name in enumerate(acm.entry_names):
            if name in allow_with_links:
                matrix[tool_idx][i] = matrix[i][tool_idx] = True

        for row in matrix:
            ace = AllowedCollisionEntry()
            ace.enabled = row
            acm.entry_values.append(ace)

        ps = PlanningScene(is_diff=True, allowed_collision_matrix=acm)
        self.scene_pub.publish(ps)
        self.get_logger().info(f"🟦 ACM aktualisiert für Tool ↔ {allow_with_links}")

    def reset_tool_acm(self):
        # Platzhalter für künftige Reset-Funktion
        pass

    def set_tool_acm_defaults(self):
        try:
            self.set_tool_acm(DEFAULT_TOOL_ACM_LINKS)
        except Exception as e:
            self.get_logger().warning(f"⚠️ ACM-Default-Update fehlgeschlagen: {e}")

    # ------------------------------------------------------------------
    # Plugin-System
    # ------------------------------------------------------------------
    def _load_plugin_for_tool(self, tool_name: str):
        self.current_plugin = None
        tool_cfg = self.tools.get(tool_name, {})
        plugin_name = tool_cfg.get("plugin", "").strip()
        if not plugin_name:
            self.get_logger().info("ℹ️ Kein Plugin im YAML – Standard-Flow.")
            return
        module_path = f"mecademic_bringup.tools.plugins.{plugin_name}"
        try:
            mod = importlib.import_module(module_path)
            PluginClass = getattr(mod, "ToolPlugin")
            self.current_plugin = PluginClass()
            self.get_logger().info(f"🔌 Plugin geladen: {module_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Plugin-Laden fehlgeschlagen ({module_path}): {e}")

    def _plugin_detach(self, tool_name: str) -> bool:
        if not self.current_plugin:
            return False
        try:
            return bool(self.current_plugin.on_detach(self, self.tools.get(tool_name, {})))
        except Exception as e:
            self.get_logger().error(f"❌ Plugin on_detach Fehler: {e}")
            return False

    def _plugin_attach(self, tool_name: str) -> bool:
        if not self.current_plugin:
            return False
        try:
            return bool(self.current_plugin.on_attach(self, self.tools.get(tool_name, {})))
        except Exception as e:
            self.get_logger().error(f"❌ Plugin on_attach Fehler: {e}")
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
        if self._publish_done:
            return
        self._publish_done = True

        tool = self.tools.get(self.current_tool, {})
        mount_frame = tool.get("mount_frame", self.frames["tool_mount"])
        tcp_xyz = tool.get("tcp_offset", [0, 0, 0])
        tcp_rpy = tool.get("tcp_rpy", [0, 0, 0])
        mesh_uri = tool.get("mesh", "")

        # Plugin laden
        self._load_plugin_for_tool(self.current_tool)

        # 1) Plugin-Hook: attach
        if not self._plugin_attach(self.current_tool):
            # Default-Flow
            self.publish_tcp_once(mount_frame, tcp_xyz, tcp_rpy)

            if mesh_uri:
                mesh = self.load_mesh(self.resolve_mesh_path(mesh_uri))
                self.attached_pub.publish(
                    self.make_attach_msg(
                        mount_frame,
                        mesh,
                        tool.get("mesh_offset", [0, 0, 0]),
                        tool.get("mesh_rpy", [0, 0, 0]),
                    )
                )
                self.set_tool_acm_defaults()

        self.pub_current.publish(String(data=self.current_tool))
        self.get_logger().info(f"✅ ToolManager aktiv: {self.current_tool}")

    # ------------------------------------------------------------------
    # Toolwechsel
    # ------------------------------------------------------------------
    def on_tool_change(self, msg: String):
        name = msg.data.strip()
        if name not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {name}")
            return

        # altes Tool-Plugin detach
        prev_tool = self.current_tool
        if prev_tool in self.tools:
            self._load_plugin_for_tool(prev_tool)
            if not self._plugin_detach(prev_tool):
                self.detach_tool_mesh()
                self.reset_tool_acm()

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
