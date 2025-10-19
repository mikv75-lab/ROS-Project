#!/usr/bin/env python3
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, Point
from shape_msgs.msg import Mesh as RosMesh, MeshTriangle
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from mecademic_bringup.common.frames import FRAME_WORLD
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG
from mecademic_bringup.utils import rpy_deg_to_quat

def _deg_to_rad_list(rpy_deg):
    return [math.radians(rpy_deg[0]), math.radians(rpy_deg[1]), math.radians(rpy_deg[2])]

def _make_static_tf(parent, child, xyz, rpy_deg):
    rx, ry, rz = _deg_to_rad_list(rpy_deg)
    qx, qy, qz, qw = rpy_deg_to_quat(rx, ry, rz)
    tf = TransformStamped()
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return tf

def _load_mesh(path, logger):
    if not os.path.exists(path):
        logger.error(f"❌ Mesh nicht gefunden: {path}")
        return None
    try:
        import trimesh
        raw = trimesh.load(path, force="mesh")
        raw.apply_scale(0.001)
        mesh = RosMesh()
        for v in raw.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in raw.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=list(f)))
        return mesh
    except Exception as e:
        logger.error(f"❌ Mesh Fehler in {path}: {e}")
        return None

class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        yaml_path = self.get_parameter(PARAM_SCENE_CONFIG).value

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Scene YAML fehlt: {yaml_path}")

        with open(yaml_path) as f:
            self.scene_objects = yaml.safe_load(f).get("scene_objects", [])

        self.base_dir = os.path.dirname(os.path.abspath(yaml_path))
        self.static_tf = StaticTransformBroadcaster(self)
        self.co_pub = self.create_publisher(CollisionObject, "collision_object", 10)

        self.step = 0
        self.timer = self.create_timer(1.0, self._step_publish)
        self.get_logger().info("⏳ SceneManager wartet bis MoveIt bereit ist...")

    def _step_publish(self):
        if self.step == 0:
            self._publish_tfs()
            self.step += 1
        elif self.step == 1:
            self._publish_meshes()
            self.timer.cancel()
            self.get_logger().info("✅ Szene vollständig geladen.")

    def _publish_tfs(self):
        tfs = []
        for obj in self.scene_objects:
            tfs.append(_make_static_tf(
                obj.get("frame", FRAME_WORLD),
                obj["id"],
                obj.get("position", [0,0,0]),
                obj.get("rpy_deg", [0,0,0])
            ))
        self.static_tf.sendTransform(tfs)
        self.get_logger().info(f"✅ TF Frames veröffentlicht ({len(tfs)})")

    def _publish_meshes(self):
        count = 0
        for obj in self.scene_objects:
            mesh_rel = obj.get("mesh", "")
            if not mesh_rel:
                continue
            mesh_path = os.path.join(self.base_dir, mesh_rel)
            mesh = _load_mesh(mesh_path, self.get_logger())
            if mesh:
                co = CollisionObject()
                co.id = obj["id"]
                co.header = Header(frame_id=obj["id"])
                co.meshes = [mesh]
                co.mesh_poses = [Pose()]
                co.operation = CollisionObject.ADD
                self.co_pub.publish(co)
                count += 1
        self.get_logger().info(f"✅ Meshes veröffentlicht ({count})")

def main():
    rclpy.init()
    rclpy.spin(SceneManager())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
