#!/usr/bin/env python3
import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from ament_index_python.packages import get_package_share_directory

from mecademic_bringup.common.topics import TOPIC_TOOL_SET, TOPIC_TOOL_CURRENT
from mecademic_bringup.common.params import PARAM_TOOL_CONFIG
from mecademic_bringup.scene.utils import rpy_deg_to_quat

TCP_FRAME = "tcp"
MOUNT_FRAME = "tool_mount"
ATTACHED_OBJ_ID = "active_tool_mesh"

# ✅ QoS Profil für PlanningScene – wichtig für MoveIt!
PS_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

def resolve_package_url(url: str):
    if not url or not url.startswith("package://"):
        return url
    pkg, rel = url[len("package://"):].split("/", 1)
    return os.path.join(get_package_share_directory(pkg), rel)

class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        self.static_tf = StaticTransformBroadcaster(self)
        self.ps_pub = self.create_publisher(PlanningScene, "/monitored_planning_scene", qos_profile=PS_QOS)
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        default_yaml = os.path.join(
            get_package_share_directory("mecademic_bringup"),
            "config", "tools.yaml"
        )
        self.declare_parameter(PARAM_TOOL_CONFIG, default_yaml)
        self.tools_yaml_path = self.get_parameter(PARAM_TOOL_CONFIG).value
        self.tools_data = self._load_yaml(self.tools_yaml_path)

        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        # ✅ Immer TCP-TF setzen
        self._publish_tcp_offset([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        # ✅ Tool direkt anwenden beim Start
        self._apply_tool(self.current_tool, init=True)
        self.get_logger().info(f"✅ ToolManager gestartet – aktives Tool: {self.current_tool}")

    # -------- YAML --------
    def _load_yaml(self, path):
        if not os.path.exists(path):
            raise FileNotFoundError(f"{path} nicht gefunden")
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def _save_current_tool(self):
        self.tools_data["active_tool"] = self.current_tool
        with open(self.tools_yaml_path, "w") as f:
            yaml.safe_dump(self.tools_data, f)

    # -------- TCP → TF --------
    def _publish_tcp_offset(self, xyz, rpy_deg):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        t = TransformStamped()
        t.header.frame_id = MOUNT_FRAME
        t.child_frame_id = TCP_FRAME
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = xyz
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.static_tf.sendTransform(t)

    # -------- STL laden --------
    def _load_mesh(self, mesh_path: str) -> Mesh:
        if not mesh_path or not os.path.exists(mesh_path):
            raise FileNotFoundError(f"Mesh-Datei nicht gefunden: {mesh_path}")
        import trimesh
        tri = trimesh.load(mesh_path, force="mesh")
        if tri.is_empty:
            raise RuntimeError(f"STL ist leer: {mesh_path}")
        msg = Mesh()
        for v in tri.vertices:
            p = Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))
            msg.vertices.append(p)
        for f in tri.faces:
            tri_msg = MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])])
            msg.triangles.append(tri_msg)
        return msg

    # -------- Attach --------
    def _attach_mesh(self, mesh_msg: Mesh):
        obj = CollisionObject()
        obj.id = ATTACHED_OBJ_ID
        obj.header.frame_id = MOUNT_FRAME
        obj.meshes = [mesh_msg]
        obj.mesh_poses = [Pose()]
        obj.operation = CollisionObject.ADD

        aco = AttachedCollisionObject()
        aco.link_name = MOUNT_FRAME
        aco.object = obj

        ps = PlanningScene()
        ps.robot_model_name = "meca_500_r3"
        ps.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        self.ps_pub.publish(ps)
        self.get_logger().info("🧩 Tool-Mesh attached")

    # -------- Detach --------
    def _detach_mesh(self):
        co = CollisionObject()
        co.id = ATTACHED_OBJ_ID
        co.operation = CollisionObject.REMOVE

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects.append(co)
        self.ps_pub.publish(ps)
        self.get_logger().info("🧹 Tool-Mesh detached")

    # -------- Apply Tool --------
    def _apply_tool(self, tool_name: str, init=False):
        cfg = self.tools.get(tool_name, {})
        tcp_xyz = cfg.get("tcp_offset", [0.0, 0.0, 0.0])
        tcp_rpy = cfg.get("tcp_rpy", [0.0, 0.0, 0.0])
        mesh_uri = cfg.get("mesh", "")

        self._publish_tcp_offset(tcp_xyz, tcp_rpy)
        self._detach_mesh()

        if mesh_uri and mesh_uri != "":
            try:
                mesh_path = resolve_package_url(mesh_uri)
                mesh_msg = self._load_mesh(mesh_path)
                self._attach_mesh(mesh_msg)
            except Exception as e:
                self.get_logger().warn(f"⚠️ Mesh konnte nicht geladen werden: {e}")

        self.pub_current.publish(String(data=tool_name))
        if not init:
            self.current_tool = tool_name
            self._save_current_tool()

        self.get_logger().info(f"🔧 Tool aktiv: {tool_name}")

    def on_tool_change(self, msg: String):
        tool = msg.data.strip()
        if tool not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {tool}")
            return
        self._apply_tool(tool)

def main():
    rclpy.init()
    node = ToolManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
