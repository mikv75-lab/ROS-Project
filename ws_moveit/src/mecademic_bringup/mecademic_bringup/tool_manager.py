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
from mecademic_bringup.utils import rpy_deg_to_quat
import trimesh
from moveit_msgs.srv import GetPlanningScene

ATTACHED_OBJ_ID = "active_tool_mesh"
TCP_FRAME = "tcp"


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # Publizierer
        self.static_tf = StaticTransformBroadcaster(self)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        # Config laden
        config_path = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, config_path)
        self.tools_yaml_path = self.get_parameter(PARAM_TOOL_CONFIG).value
        self.tools_data = self._load_yaml(self.tools_yaml_path)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        # State
        self.mesh_cache = {}
        self.mount_frame = None
        self.current_has_mesh = False
        self.first_publish_done = False

        # ✅ Timer für Publish Retry
        self.timer = self.create_timer(1.0, self._refresh)

        # Service Client für Scene-Check
        self.scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        # Tool initial anwenden
        self._apply_tool(self.current_tool, init=True)

        self.get_logger().info(f"✅ ToolManager aktiv – aktuelles Tool: {self.current_tool}")

    def _load_yaml(self, path):
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def _resolve_mesh(self, uri):
        if uri.startswith("package://"):
            pkg, rel = uri[10:].split("/", 1)
            return os.path.join(get_package_share_directory(pkg), rel)
        return uri

    def _publish_tcp(self, parent, xyz, rpy):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy)
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id = TCP_FRAME
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.static_tf.sendTransform(tf)

    def _load_mesh(self, mesh_path):
        if mesh_path in self.mesh_cache:
            return self.mesh_cache[mesh_path]
        tri = trimesh.load(mesh_path)
        tri.apply_scale(0.001)
        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=list(f)))
        self.mesh_cache[mesh_path] = mesh
        return mesh

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
        msg.object.mesh_poses = [Pose()]
        msg.object.operation = CollisionObject.ADD
        return msg

    def _apply_tool(self, tool_name, init=False):
        tool = self.tools.get(tool_name, {})
        self.mount_frame = tool.get("mount_frame", "tool_mount")
        tcp = tool.get("tcp_offset", [0, 0, 0])
        rpy = tool.get("tcp_rpy", [0, 0, 0])
        mesh_uri = tool.get("mesh", "")

        self._publish_tcp(self.mount_frame, tcp, rpy)
        self.attached_pub.publish(self._msg_detach())
        self.current_has_mesh = bool(mesh_uri)

        if self.current_has_mesh:
            mesh_path = self._resolve_mesh(mesh_uri)
            mesh = self._load_mesh(mesh_path)
            self.attached_pub.publish(self._msg_attach(mesh))

        self.pub_current.publish(String(data=tool_name))
        self.first_publish_done = False
        self.timer.reset()

    def _object_in_scene(self):
        if not self.scene_client.wait_for_service(timeout_sec=0.2):
            return False
        req = GetPlanningScene.Request()
        req.components.components = req.components.WORLD_OBJECT_NAMES
        future = self.scene_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.2)
        if not future.result():
            return False
        return any(o.id == ATTACHED_OBJ_ID for o in future.result().scene.world.collision_objects)

    def _refresh(self):
        if self.first_publish_done:
            return

        if self._object_in_scene():
            self.get_logger().info("✅ Tool ist im PlanningScene – Timer gestoppt.")
            self.first_publish_done = True
            self.timer.cancel()
            return
        else:
            self.get_logger().warning("⏳ Tool noch nicht im PlanningScene – retry...")

    def on_tool_change(self, msg):
        tool = msg.data.strip()
        if tool not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {tool}")
            return
        self._apply_tool(tool)


def main():
    rclpy.init()
    rclpy.spin(ToolManager())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
