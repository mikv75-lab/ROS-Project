#!/usr/bin/env python3
import os
import sys
import yaml
import importlib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped
from tf2_ros import StaticTransformBroadcaster
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory

from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.scene.utils import rpy_deg_to_quat


def resolve_package_url(url: str):
    if not url.startswith("package://"):
        return url
    pkg, rel = url[len("package://"):].split("/", 1)
    return os.path.join(get_package_share_directory(pkg), rel)


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        self.static_tf = StaticTransformBroadcaster(self)
        self.ps_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        default_yaml = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_yaml)
        self.tools_yaml_path = self.get_parameter(PARAM_TOOL_CONFIG).value
        self.tools_data = self._load_yaml(self.tools_yaml_path)

        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        self._apply_tool(self.current_tool, init=True)
        self.get_logger().info(f"✅ ToolManager gestartet – aktives Tool: {self.current_tool}")

    # ------------------- YAML Handling -------------------
    def _load_yaml(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"{path} nicht gefunden")
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def _save_current_tool(self):
        self.tools_data["active_tool"] = self.current_tool
        with open(self.tools_yaml_path, "w") as f:
            yaml.safe_dump(self.tools_data, f)

    # ------------------- TF Publish -------------------
    def _publish_tcp(self, mount_frame, xyz, rpy):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy)
        t = TransformStamped()
        t.header.frame_id = mount_frame
        t.child_frame_id = "tcp"
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.static_tf.sendTransform(t)

    # ------------------- Mesh Loader -------------------
    def _load_mesh(self, mesh_path: str) -> Mesh:
        if not os.path.exists(mesh_path):
            raise FileNotFoundError(f"Mesh-Datei nicht gefunden: {mesh_path}")

        mesh = Mesh()
        vertices = []
        triangles = []
        with open(mesh_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if parts and parts[0] == "vertex":
                    vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
                elif parts and parts[0] == "facet":
                    triangles.append([])

                if parts and parts[0] == "endfacet":
                    if len(vertices) >= 3:
                        i = len(vertices)
                        triangles[-1] = [i - 3, i - 2, i - 1]

        mesh.vertices = [self._to_point(v) for v in vertices]
        mesh.triangles = [MeshTriangle(vertex_indices=t) for t in triangles]
        return mesh

    def _to_point(self, v):
        from geometry_msgs.msg import Point
        p = Point()
        p.x, p.y, p.z = v
        return p

    # ------------------- Apply Tool -------------------
    def _apply_tool(self, tool_name, init=False):
        cfg = self.tools[tool_name]
        mount = cfg.get("mount_frame", "tool_mount")
        tcp = cfg.get("tcp_offset", [0, 0, 0])
        rpy = cfg.get("tcp_rpy", [0, 0, 0])
        mesh_uri = cfg.get("mesh", "")

        self._publish_tcp(mount, tcp, rpy)

        if mesh_uri:
            mesh_path = resolve_package_url(mesh_uri)
            mesh = self._load_mesh(mesh_path)
            self._attach_mesh(tool_name, mount, mesh)

        self.pub_current.publish(String(data=tool_name))
        if not init:
            self.current_tool = tool_name
            self._save_current_tool()

    # ------------------- Attach Mesh -------------------
    def _attach_mesh(self, name, frame, mesh):
        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = frame
        obj.meshes = [mesh]
        obj.mesh_poses = [Pose()]
        obj.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = frame
        aco.object = obj

        ps = PlanningScene()
        ps.robot_state.attached_collision_objects = [aco]
        ps.is_diff = True

        self.ps_pub.publish(ps)
        self.get_logger().info(f"🧩 Tool '{name}' Mesh attached")

    # ------------------- Topic Callback -------------------
    def on_tool_change(self, msg):
        new_tool = msg.data.strip()
        if new_tool not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {new_tool}")
            return

        self._apply_tool(new_tool)
        self.get_logger().info(f"🔧 Tool gewechselt: {new_tool}")

def main():
    rclpy.init()
    node = ToolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
