#!/usr/bin/env python3
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from ament_index_python.packages import get_package_share_directory
from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.utils import rpy_deg_to_quat
import trimesh

ATTACHED_OBJ_ID = "active_tool_mesh"
TCP_FRAME = "tcp"

RETRY_PERIOD_SEC = 0.5
RETRY_TIMEOUT_SEC = 30.0
SCENE_WAIT_TIMEOUT = 30.0  # ✅ NEU: Wartezeit bevor Tool gestartet wird


class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        self.static_tf = StaticTransformBroadcaster(self)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        self.ps_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        self._retry_timer = None
        self._retry_deadline = 0.0
        self._last_mount_frame = "tool_mount"

        config_path = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, config_path)
        self.tools_data = self._load_yaml(self.get_parameter(PARAM_TOOL_CONFIG).value)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")
        self.mesh_cache = {}

        # ✅ NEU: Warten bis PlanningScene bereit
        self.scene_wait_start = time.time()
        self.scene_wait_timer = self.create_timer(1.0, self._wait_for_scene_ready)

        self.get_logger().info("⏳ Warte auf MoveIt PlanningScene bevor Tool geladen wird...")

    # ✅ NEU: warte bis MoveIt bereit ist
    def _wait_for_scene_ready(self):
        if self.ps_client.wait_for_service(timeout_sec=1.0):
            self.scene_wait_timer.cancel()
            self.get_logger().info("✅ MoveIt PlanningScene bereit – Tool wird geladen!")
            self.apply_tool(self.current_tool)
        elif time.time() - self.scene_wait_start > SCENE_WAIT_TIMEOUT:
            self.scene_wait_timer.cancel()
            self.get_logger().warning("⚠️ MoveIt nicht verfügbar – starte ToolManager trotzdem.")
            self.apply_tool(self.current_tool)

    def _load_yaml(self, path):
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def resolve_mesh_path(self, uri):
        if uri.startswith("package://"):
            pkg, rel = uri[10:].split("/", 1)
            return os.path.join(get_package_share_directory(pkg), rel)
        return uri

    def publish_tcp(self, parent, xyz, rpy):
        q = rpy_deg_to_quat(*rpy)
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id = TCP_FRAME
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = q
        self.static_tf.sendTransform(tf)

    def load_mesh(self, mesh_path):
        if mesh_path in self.mesh_cache:
            return self.mesh_cache[mesh_path]
        tri = trimesh.load(mesh_path)
        tri.apply_scale(0.001)
        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=v[0], y=v[1], z=v[2]))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=list(f)))
        self.mesh_cache[mesh_path] = mesh
        return mesh

    def msg_attach(self, mount_frame, mesh):
        msg = AttachedCollisionObject()
        msg.link_name = mount_frame
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.header.frame_id = mount_frame
        msg.object.meshes = [mesh]
        msg.object.mesh_poses = [Pose()]
        msg.object.operation = CollisionObject.ADD
        return msg

    def msg_detach(self, mount_frame):
        msg = AttachedCollisionObject()
        msg.link_name = mount_frame
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.operation = CollisionObject.REMOVE
        return msg

    def _start_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None
        self._retry_deadline = time.time() + RETRY_TIMEOUT_SEC
        self._retry_timer = self.create_timer(RETRY_PERIOD_SEC, self._check_tool_attached)

    def _stop_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

    def _check_tool_attached(self):
        if time.time() > self._retry_deadline:
            self._stop_retry_timer()
            self.get_logger().error("❌ Timeout – Tool wurde nicht in MoveIt erkannt.")
            return

        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS |
            PlanningSceneComponents.WORLD_OBJECT_NAMES
        )
        future = self.ps_client.call_async(req)

        def _done(_):
            try:
                res = future.result()
            except Exception as e:
                self.get_logger().error(f"PlanningScene Fehler: {e}")
                return

            attached_ids = [a.object.id for a in res.scene.robot_state.attached_collision_objects]
            if ATTACHED_OBJ_ID in attached_ids:
                self.get_logger().info("✅ Tool erfolgreich in MoveIt attached!")
                self._stop_retry_timer()
            else:
                self.get_logger().warning("⏳ Tool noch nicht übernommen – retry...")

        future.add_done_callback(_done)

    def apply_tool(self, name):
        tool = self.tools.get(name, {})
        mount_frame = tool.get("mount_frame", "tool_mount")
        self.attached_pub.publish(self.msg_detach(mount_frame))

        self.publish_tcp(mount_frame, tool.get("tcp_offset", [0, 0, 0]), tool.get("tcp_rpy", [0, 0, 0]))

        mesh_uri = tool.get("mesh", "")
        if mesh_uri:
            mesh_path = self.resolve_mesh_path(mesh_uri)
            mesh = self.load_mesh(mesh_path)
            self.attached_pub.publish(self.msg_attach(mount_frame, mesh))

        self.pub_current.publish(String(data=name))
        self._start_retry_timer()

    def on_tool_change(self, msg):
        tool = msg.data.strip()
        if tool in self.tools:
            self.apply_tool(tool)

def main():
    rclpy.init()
    rclpy.spin(ToolManager())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
