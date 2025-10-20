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
SCENE_WAIT_TIMEOUT = 30.0  # max. warten bis PlanningScene-Dienst da ist

class ToolManager(Node):
    def __init__(self):
        super().__init__("tool_manager")

        # Publisher / Subscriptions
        self.static_tf = StaticTransformBroadcaster(self)
        self.attached_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self.pub_current = self.create_publisher(String, TOPIC_TOOL_CURRENT, 10)
        self.create_subscription(String, TOPIC_TOOL_SET, self.on_tool_change, 10)

        # Config laden
        default_cfg = os.path.join(get_package_share_directory("mecademic_bringup"), "config", "tools.yaml")
        self.declare_parameter(PARAM_TOOL_CONFIG, default_cfg)
        self.tools_data = self._load_yaml(self.get_parameter(PARAM_TOOL_CONFIG).value)
        self.tools = self.tools_data.get("tools", {})
        self.current_tool = self.tools_data.get("active_tool", "none")

        self.mesh_cache = {}

        # MoveIt Scene Service
        self.ps_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        # State
        self._scene_ready = False
        self._armed_to_publish = False  # wird auf True gesetzt, sobald Scene ready ist (nach 3s Pause)
        self._publish_done = False      # einmalig senden, danach nie wieder automatisch

        # Timer: auf Scene warten
        self._scene_wait_start = time.time()
        self._scene_wait_timer = self.create_timer(0.5, self._check_scene_ready)

        self.get_logger().info("⏳ Warte auf MoveIt PlanningScene – nichts wird davor publiziert.")

    # ---------- Hilfsfunktionen ----------
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
        tri.apply_scale(0.001)
        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]))
        self.mesh_cache[mesh_path] = mesh
        return mesh

    def publish_tcp_once(self, parent, xyz, rpy_deg):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id = TCP_FRAME
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.static_tf.sendTransform([tf])
        self.get_logger().info(f"🔹 TCP gesetzt @ {parent} -> xyz={xyz}, rpy_deg={rpy_deg}")

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

    # ---------- Scene-Wait-Logik ----------
    def _check_scene_ready(self):
        # Timeout?
        if (time.time() - self._scene_wait_start) > SCENE_WAIT_TIMEOUT:
            self.get_logger().warning("⚠️ PlanningScene-Dienst nicht rechtzeitig da – sende trotzdem in 3s einmalig.")
            self._arm_and_delay_publish()
            return

        if not self.ps_client.wait_for_service(timeout_sec=0.2):
            return  # weiter warten, NICHTS publizieren

        # Einmaliger Service-Call (nur zur Anwesenheitsprüfung)
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.SCENE_SETTINGS
        future = self.ps_client.call_async(req)
        future.add_done_callback(self._on_scene_service_ok)

        # Warten stoppen, bis Callback entscheidet
        self._scene_wait_timer.cancel()

    def _on_scene_service_ok(self, future):
        try:
            _ = future.result()
            self.get_logger().info("✅ PlanningScene erreichbar – sende in 3s einmalig TCP + Tool.")
        except Exception as e:
            self.get_logger().warning(f"⚠️ PlanningScene-Check fehlgeschlagen ({e}) – sende trotzdem in 3s einmalig.")
        finally:
            self._arm_and_delay_publish()

    def _arm_and_delay_publish(self):
        # 3s Verzögerung, dann EINMALIG publish
        if self._armed_to_publish or self._publish_done:
            return
        self._armed_to_publish = True
        def _do_once():
            try:
                self._publish_once()
            finally:
                delay_timer.cancel()
        delay_timer = self.create_timer(3.0, _do_once)

    # ---------- Einmaliges Publizieren ----------
    def _publish_once(self):
        if self._publish_done:
            return
        self._publish_done = True

        name = self.current_tool
        tool = self.tools.get(name, {})
        mount_frame = tool.get("mount_frame", "tool_mount")

        # TCP
        tcp_xyz = tool.get("tcp_offset", [0.0, 0.0, 0.0])
        tcp_rpy = tool.get("tcp_rpy",   [0.0, 0.0, 0.0])
        self.publish_tcp_once(mount_frame, tcp_xyz, tcp_rpy)

        # Mesh (optional)
        mesh_uri = tool.get("mesh", "")
        if mesh_uri:
            mesh_path = self.resolve_mesh_path(mesh_uri)
            try:
                mesh = self.load_mesh(mesh_path)
                attach_msg = self.make_attach_msg(
                    mount_frame,
                    mesh,
                    tool.get("mesh_offset", [0.0, 0.0, 0.0]),
                    tool.get("mesh_rpy",    [0.0, 0.0, 0.0]),
                )
                self.attached_pub.publish(attach_msg)
                self.get_logger().info(f"📦 Tool-Mesh attached (mesh='{mesh_path}')")
            except Exception as e:
                self.get_logger().error(f"❌ Mesh laden/attachen fehlgeschlagen: {e}")
        else:
            self.get_logger().info("ℹ️ Kein Mesh konfiguriert – nur TCP veröffentlicht.")

        # Aktives Tool bekanntgeben (informativ)
        self.pub_current.publish(String(data=name))
        self.get_logger().info("✅ ToolManager: einmalige Publikation abgeschlossen.")

    # ---------- Toolwechsel (manuell) ----------
    def on_tool_change(self, msg: String):
        # Toolwechsel sind erlaubt – aber nur EINMALIGE Publikation erneut durchführen
        name = msg.data.strip()
        if name not in self.tools:
            self.get_logger().error(f"❌ Unbekanntes Tool: {name}")
            return
        self.current_tool = name
        self.get_logger().info(f"🔧 Toolwechsel zu: {name}")

        # Nur wenn die erste Publikation bereits erfolgt ist, sofort einmalig wiederholen;
        # wenn noch nicht publiziert wurde (Scene noch nicht ready), bleiben wir im Wartezustand.
        if self._publish_done:
            # "Einmalig" für neues Tool erneut
            self._publish_done = False
            self._armed_to_publish = True
            # kleine Verzögerung (0.5s) damit RViz/MoveIt sauber updaten kann
            def _repub():
                try:
                    self._publish_once()
                finally:
                    timer.cancel()
            timer = self.create_timer(0.5, _repub)

def main():
    rclpy.init()
    rclpy.spin(ToolManager())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
