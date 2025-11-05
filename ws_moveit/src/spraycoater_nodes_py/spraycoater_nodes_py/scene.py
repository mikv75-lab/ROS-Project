#!/usr/bin/env python3
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_msgs.msg import Header
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG
from mecademic_bringup.utils import rpy_deg_to_quat
import trimesh

SCENE_TOPIC = "/collision_object"
MAX_SCENE_WAIT = 30.0  # Warten auf MoveIt

class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")

        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        yaml_path = self.get_parameter(PARAM_SCENE_CONFIG).value
        if not yaml_path or not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Scene YAML fehlt: {yaml_path}")

        with open(yaml_path, "r") as f:
            self.scene_data = yaml.safe_load(f) or {}

        self.scene_objects = self.scene_data.get("scene_objects", [])
        self.base_dir = os.path.dirname(os.path.abspath(yaml_path))

        self.static_tf = StaticTransformBroadcaster(self)
        self.scene_pub = self.create_publisher(CollisionObject, SCENE_TOPIC, 10)
        self.mesh_cache = {}

        self.final_scene_sent = False

        self.get_logger().info("⏳ Warte auf MoveIt – Szene später senden...")
        self._start_scene_wait()

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

    def _pose_from_offset(self, xyz, rpy_deg):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        return pose

    def _make_tf(self, parent, child, xyz, rpy_deg):
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        tf = TransformStamped()
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        return tf

    def _publish_scene(self):
        if self.final_scene_sent:
            return
        self.final_scene_sent = True

        tfs = []
        for obj in self.scene_objects:
            tfs.append(self._make_tf(obj.get("frame", "world"), obj["id"], obj["position"], obj["rpy_deg"]))
        self.static_tf.sendTransform(tfs)
        self.get_logger().info(f"✅ Static TFs veröffentlicht ({len(tfs)})")

        for obj in self.scene_objects:
            if not obj.get("mesh"):
                continue
            mesh_path = os.path.join(self.base_dir, obj["mesh"])
            if not os.path.exists(mesh_path):
                self.get_logger().error(f"❌ Mesh fehlt: {mesh_path}")
                continue

            mesh = self._load_mesh(mesh_path)
            co = CollisionObject()
            co.id = obj["id"]
            co.header.frame_id = obj["id"]
            co.meshes = [mesh]
            co.mesh_poses = [self._pose_from_offset(obj["mesh_offset"], obj["mesh_rpy"])]
            co.operation = CollisionObject.ADD
            self.scene_pub.publish(co)

        self.get_logger().info("🎯 Szene FINAL & EINMALIG an MoveIt gesendet.")

    def _start_scene_wait(self):
        self.deadline = time.time() + MAX_SCENE_WAIT
        self.wait_timer = self.create_timer(1.0, self._check_moveit_ready)

    def _check_moveit_ready(self):
        if self.final_scene_sent:
            self.wait_timer.cancel()
            return

        if time.time() > self.deadline:
            self.get_logger().warning("⚠️ Timeout – sende Szene trotzdem.")
            self._publish_scene()
            self.wait_timer.cancel()
            return

        if not hasattr(self, "ps_client"):
            self.ps_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        if not self.ps_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("⏳ Warte auf MoveIt PlanningScene Service...")
            return

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.SCENE_SETTINGS
        future = self.ps_client.call_async(req)
        future.add_done_callback(self._moveit_ready_callback)

    def _moveit_ready_callback(self, _):
        self.get_logger().info("✅ MoveIt bereit – sende Szene in 3 Sekunden...")
        self.wait_timer.cancel()
        self.create_timer(3.0, self._publish_scene)


def main():
        rclpy.init()
        rclpy.spin(SceneManager())
        rclpy.shutdown()


if __name__ == "__main__":
    main()
