#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from ament_index_python.packages import get_package_share_directory
from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.scene.utils import rpy_deg_to_quat
import trimesh

ATTACHED_OBJ_ID = "active_tool_mesh"
TCP_FRAME = "tcp"

class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # Publishers / Subscriptions
        self.static_tf = StaticTransformBroadcaster(self)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        # Load config
        default_yaml = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_yaml)
        self.tools_yaml_path = self.get_parameter(PARAM_TOOL_CONFIG).value
        self.tools_data = self._load_yaml(self.tools_yaml_path)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        # State
        self.mesh_cache = {}
        self.mount_frame = None
        self.current_has_mesh = False

        # Apply tool and start periodic refresh (1 Hz)
        self._apply_tool(self.current_tool, init=True)
        self.timer = self.create_timer(1.0, self._tick_refresh)

        self.get_logger().info(f"✅ ToolManager aktiv – Tool: {self.current_tool}")

    # ---------- helpers ----------
    def _load_yaml(self, path):
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def _resolve_mesh(self, uri):
        if uri.startswith("package://"):
            pkg, rel = uri[10:].split("/", 1)
            return os.path.join(get_package_share_directory(pkg), rel)
        return uri

    def _publish_tcp(self, parent_frame, xyz, rpy):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy)
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = TCP_FRAME
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        self.static_tf.sendTransform(tf)

    def _load_mesh(self, mesh_path):
        if mesh_path in self.mesh_cache:
            return self.mesh_cache[mesh_path]

        import trimesh
        tri = trimesh.load(mesh_path)

        # ✅ ALLE STL AUTOMATISCH VON MM → M SKALIEREN
        tri.apply_scale(0.001)

        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=list(f)))
        
        self.mesh_cache[mesh_path] = mesh
        return mesh


    # ---------- attached object ops ----------
    def _msg_detach(self):
        msg = AttachedCollisionObject()
        msg.link_name = self.mount_frame
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.header.frame_id = self.mount_frame
        msg.object.operation = CollisionObject.REMOVE
        return msg

    def _msg_attach(self, mesh):
        msg = AttachedCollisionObject()
        msg.link_name = self.mount_frame
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.header.frame_id = self.mount_frame
        msg.object.meshes = [mesh]
        msg.object.mesh_poses = [Pose()]  # identity pose relative to link
        msg.object.operation = CollisionObject.ADD
        return msg

    def _detach_publish(self):
        if not self.mount_frame:
            return
        self.attached_pub.publish(self._msg_detach())

    def _attach_publish(self, mesh):
        self.attached_pub.publish(self._msg_attach(mesh))

    # ---------- periodic refresh ----------
    def _tick_refresh(self):
        if not self.mount_frame:
            return
        # Re-announce the TF (cheap and idempotent)
        tool = self.tools.get(self.current_tool, {})
        self._publish_tcp(self.mount_frame, tool.get("tcp_offset", [0,0,0]), tool.get("tcp_rpy", [0,0,0]))

        # Re-announce attached mesh to keep RViz/MoveIt in sync
        self._detach_publish()
        if self.current_has_mesh:
            mesh_uri = self.tools[self.current_tool]["mesh"]
            mesh_path = self._resolve_mesh(mesh_uri)
            mesh = self._load_mesh(mesh_path)
            self._attach_publish(mesh)

    # ---------- apply tool ----------
    def _apply_tool(self, tool_name, init=False):
        tool = self.tools.get(tool_name, {})
        self.mount_frame = tool.get("mount_frame", "tool_mount")
        tcp = tool.get("tcp_offset", [0, 0, 0])
        rpy = tool.get("tcp_rpy", [0, 0, 0])
        mesh_uri = tool.get("mesh", "")

        # Set TCP TF
        self._publish_tcp(self.mount_frame, tcp, rpy)

        # Remove previous object, then add if mesh exists
        self._detach_publish()
        self.current_has_mesh = bool(mesh_uri)
        if self.current_has_mesh:
            mesh = self._load_mesh(self._resolve_mesh(mesh_uri))
            self._attach_publish(mesh)

        # State topic
        self.pub_current.publish(String(data=tool_name))

        # Persist current tool (optional on init)
        if not init:
            self.tools_data["active_tool"] = tool_name
            with open(self.tools_yaml_path, "w") as f:
                yaml.safe_dump(self.tools_data, f)

    # ---------- callback ----------
    def on_tool_change(self, msg):
        tool = msg.data.strip()
        if tool not in self.tools:
            self.get_logger().error(f"❌ Tool nicht bekannt: {tool}")
            return
        self._apply_tool(tool)


def main():
    rclpy.init()
    rclpy.spin(ToolManager())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
