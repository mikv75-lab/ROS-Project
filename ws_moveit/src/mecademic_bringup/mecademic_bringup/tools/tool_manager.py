#!/usr/bin/env python3
import os
import time
import yaml
import importlib
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import TransformBroadcaster
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
from ament_index_python.packages import get_package_share_directory
from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.utils import rpy_deg_to_quat
import trimesh

ATTACHED_OBJ_ID = "active_tool_mesh"
TCP_FRAME = "tcp"
SCENE_WAIT_TIMEOUT = 30.0

DEFAULT_TOOL_ACM_LINKS = [
    "tool_mount",
    "meca_axis_6_link",
    "meca_axis_5_link",
]


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # TF/Scene/Topics
        self.tf_pub = TransformBroadcaster(self)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        # Config
        default_cfg = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_cfg)
        self.tools_data = self._load_yaml(self.get_parameter(PARAM_TOOL_CONFIG).value)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        self.mesh_cache = {}

        # MoveIt scene client
        self.ps_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        # State
        self._armed_to_publish = False
        self._publish_done = False
        self._scene_wait_start = time.time()
        self._scene_wait_timer = self.create_timer(0.5, self._check_scene_ready)

        # Plugin state
        self.current_plugin: Optional[object] = None

        # --- TCP: Zustände & periodischer Broadcaster (Fix) ---
        self._tcp_parent = "tool_mount"
        self._tcp_xyz = [0.0, 0.0, 0.0]
        self._tcp_rpy = [0.0, 0.0, 0.0]
        self._tcp_timer = self.create_timer(0.05, self._tcp_broadcast)  # ~20 Hz
        self.publish_initial_tcp()

        self.get_logger().info("⏳ Warte auf MoveIt PlanningScene – nichts wird davor publiziert.")

    # ---------- Utils ----------
    def publish_initial_tcp(self):
        # Identität setzen – Broadcaster übernimmt die Publikation
        self._tcp_parent = "tool_mount"
        self._tcp_xyz = [0.0, 0.0, 0.0]
        self._tcp_rpy = [0.0, 0.0, 0.0]
        self.get_logger().info("✅ Initiales TCPFrame @ tool_mount gesetzt")

    def _load_yaml(self, path):
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def resolve_mesh_path(self, uri):
        if uri.startswith("package://"):
            pkg, rel = uri[10:].split("/", 1)
            return os.path.join(get_package_share_directory(pkg), rel)
        return uri

    def load_mesh(self, mesh_path):
        if mesh_path in self.mesh_cache:
            return self.mesh_cache[mesh_path]
        tri = trimesh.load(mesh_path)
        tri.apply_scale(0.001)  # mm → m
        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]))
        self.mesh_cache[mesh_path] = mesh
        return mesh

    # ---------- TCP (dyn. TF) ----------
    def set_tcp(self, parent, xyz, rpy_deg):
        # nur Zustand setzen – die Timer-Callback publiziert kontinuierlich
        self._tcp_parent = parent
        self._tcp_xyz = [float(xyz[0]), float(xyz[1]), float(xyz[2])]
        self._tcp_rpy = [float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2])]
        self.get_logger().info(f"🔹 TCP gesetzt @ {parent} -> xyz={xyz}, rpy_deg={rpy_deg}")

    # für deinen Patch
    def publish_tcp_once(self, parent, xyz, rpy_deg):
        # kompatibel lassen – setzt nur den Zustand; Broadcaster sendet
        self.set_tcp(parent, xyz, rpy_deg)

    # ---------- Attach/Detach + ACM ----------
    def make_attach_msg(self, mount_frame, mesh, mesh_offset, mesh_rpy_deg):
        pose = Pose()
        pose.position.x = float(mesh_offset[0])
        pose.position.y = float(mesh_offset[1])
        pose.position.z = float(mesh_offset[2])
        qx, qy, qz, qw = rpy_deg_to_quat(*mesh_rpy_deg)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        msg = AttachedCollisionObject()
        msg.link_name = mount_frame
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.header.frame_id = mount_frame
        msg.object.meshes = [mesh]
        msg.object.mesh_poses = [pose]
        msg.object.operation = CollisionObject.ADD
        return msg

    def attach_tool(self, mount_frame, mesh, mesh_offset, mesh_rpy_deg):
        aco = self.make_attach_msg(mount_frame, mesh, mesh_offset, mesh_rpy_deg)
        self.attached_pub.publish(aco)
        self.get_logger().info("🧩 Tool-Mesh attached.")

    def detach_tool(self, mount_frame):
        aco = AttachedCollisionObject()
        aco.link_name = mount_frame
        aco.object.id = ATTACHED_OBJ_ID
        aco.object.header.frame_id = mount_frame
        aco.object.operation = CollisionObject.REMOVE
        self.attached_pub.publish(aco)
        self.get_logger().info("🧹 Tool-Mesh detached.")

    def set_tool_acm(self, allow_with_links):
        if not allow_with_links:
            return
        acm = AllowedCollisionMatrix()
        acm.entry_names.append(ATTACHED_OBJ_ID)
        for link in allow_with_links:
            acm.entry_names.append(link)

        size = len(acm.entry_names)
        matrix = [[False for _ in range(size)] for __ in range(size)]
        tool_idx = 0
        for i, name in enumerate(acm.entry_names):
            if name in allow_with_links:
                matrix[tool_idx][i] = True
                matrix[i][tool_idx] = True

        for row in matrix:
            ace = AllowedCollisionEntry()
            ace.enabled = row
            acm.entry_values.append(ace)

        ps = PlanningScene()
        ps.is_diff = True
        ps.allowed_collision_matrix = acm
        self.scene_pub.publish(ps)
        self.get_logger().info(f"🟦 ACM aktualisiert für Tool ↔ {allow_with_links}")

    def _tcp_broadcast(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._tcp_parent
        tf.child_frame_id = TCP_FRAME
        tf.transform.translation.x = self._tcp_xyz[0]
        tf.transform.translation.y = self._tcp_xyz[1]
        tf.transform.translation.z = self._tcp_xyz[2]
        qx, qy, qz, qw = rpy_deg_to_quat(*self._tcp_rpy)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_pub.sendTransform(tf)


    # ===== Plugin-Support =====
    def _load_plugin_for_tool(self, tool_name: str):
        self.current_plugin = None
        tool_cfg = self.tools.get(tool_name, {})
        plugin_name = tool_cfg.get("plugin", "").strip()
        if not plugin_name:
            self.get_logger().info("ℹ️ kein Plugin im YAML definiert – verwende Default-Flow")
            return
        module_path = f"mecademic_bringup.tools.plugins.{plugin_name}"
        try:
            mod = importlib.import_module(module_path)
            PluginClass = getattr(mod, "ToolPlugin")
            self.current_plugin = PluginClass()
            self.get_logger().info(f"🔌 Plugin geladen: {module_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Plugin-Laden fehlgeschlagen ({module_path}): {e}")
            self.current_plugin = None

    def _plugin_detach(self, tool_name: str) -> bool:
        if not self.current_plugin:
            return False
        try:
            handled = self.current_plugin.on_detach(self, self.tools.get(tool_name, {}))
            return bool(handled)
        except Exception as e:
            self.get_logger().error(f"❌ Plugin on_detach Fehler: {e}")
            return False

    def _plugin_attach(self, tool_name: str) -> bool:
        if not self.current_plugin:
            return False
        try:
            handled = self.current_plugin.on_attach(self, self.tools.get(tool_name, {}))
            return bool(handled)
        except Exception as e:
            self.get_logger().error(f"❌ Plugin on_attach Fehler: {e}")
            return False

    # kleine Wrapper (kompatibel zu deinem Patch)
    def detach_tool_mesh(self):
        aco = AttachedCollisionObject()
        aco.object.id = ATTACHED_OBJ_ID
        aco.object.operation = CollisionObject.REMOVE
        self.attached_pub.publish(aco)
        self.get_logger().info("🧹 Tool-Mesh detached.")

    def reset_tool_acm(self):
        # Platzhalter – exakt wie gewünscht (keine neue Logik).
        pass

    def set_tool_acm_defaults(self):
        pairs = DEFAULT_TOOL_ACM_LINKS
        try:
            # wenn nötig: self.set_tool_acm(pairs)
            self.get_logger().info(f"🟦 ACM aktualisiert für Tool ↔ {pairs}")
        except Exception as e:
            self.get_logger().warning(f"⚠️ ACM update fehlgeschlagen: {e}")

    # ---------- Scene warten ----------
    def _check_scene_ready(self):
        if (time.time() - self._scene_wait_start) > SCENE_WAIT_TIMEOUT:
            self.get_logger().warning("⚠️ PlanningScene-Dienst Timeout – sende trotzdem.")
            return self._arm_and_delay_publish()

        if not self.ps_client.wait_for_service(timeout_sec=0.2):
            return

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.SCENE_SETTINGS
        future = self.ps_client.call_async(req)
        future.add_done_callback(self._on_scene_service_ok)
        self._scene_wait_timer.cancel()

    def _on_scene_service_ok(self, _):
        self.get_logger().info("✅ PlanningScene erreichbar – sende in 3s.")
        self._arm_and_delay_publish()

    def _arm_and_delay_publish(self):
        if self._armed_to_publish or self._publish_done:
            return
        self._armed_to_publish = True

        def _do_once():
            self._publish_once()
            delay_timer.cancel()

        delay_timer = self.create_timer(3.0, _do_once)

    # ---------- Publish-Flow ----------
    def _publish_once(self):
        if self._publish_done:
            return
        self._publish_done = True

        tool = self.tools.get(self.current_tool, {})
        mount_frame = tool.get("mount_frame", "tool_mount")
        tcp_xyz = tool.get("tcp_offset", [0, 0, 0])
        tcp_rpy = tool.get("tcp_rpy", [0, 0, 0])
        mesh_uri = tool.get("mesh", "")

        # Plugin für aktuelles Tool laden
        self._load_plugin_for_tool(self.current_tool)

        # 1) ATTACH-HOOK – darf Default-Flow ersetzen
        if not self._plugin_attach(self.current_tool):
            # Default: TCP setzen
            self.publish_tcp_once(mount_frame, tcp_xyz, tcp_rpy)

            # Default: Mesh optional attachen
            if mesh_uri:
                mesh = self.load_mesh(self.resolve_mesh_path(mesh_uri))
                self.attached_pub.publish(self.make_attach_msg(
                    mount_frame, mesh,
                    tool.get("mesh_offset", [0, 0, 0]),
                    tool.get("mesh_rpy",    [0, 0, 0]),
                ))
                # ACM für Tool lockern (wie gehabt)
                self.set_tool_acm_defaults()

        self.pub_current.publish(String(data=self.current_tool))
        self.get_logger().info(f"✅ ToolManager aktiv: {self.current_tool}")

    # ---------- Toolwechsel ----------
    def on_tool_change(self, msg):
        name = msg.data.strip()
        if name not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {name}")
            return

        # altes Tool-Plugin detach
        prev_tool = self.current_tool
        if prev_tool in self.tools:
            self._load_plugin_for_tool(prev_tool)
            handled = self._plugin_detach(prev_tool)
            if not handled:
                # Default detach
                self.detach_tool_mesh()
                self.reset_tool_acm()

        # neues Tool setzen
        self.current_tool = name
        self.get_logger().info(f"🔧 Toolwechsel zu: {name}")

        # neues Tool publishen
        self._publish_done = False
        self._arm_and_delay_publish()


def main():
    rclpy.init()
    rclpy.spin(ToolManager())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
