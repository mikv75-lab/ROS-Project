#!/usr/bin/env python3
# mecademic_bringup/scene/scene_manager_node.py

from __future__ import annotations
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Point
from shape_msgs.msg import Mesh as RosMesh, MeshTriangle
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from mecademic_bringup.common.frames import FRAME_WORLD
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG
from mecademic_bringup.utils import rpy_deg_to_quat

def _deg_to_rad_list(rpy_deg):
    r, p, y = rpy_deg
    return [math.radians(r), math.radians(p), math.radians(y)]

def _make_static_tf(parent: str, child: str, xyz, rpy_deg) -> TransformStamped:
    rx, ry, rz = _deg_to_rad_list(rpy_deg)
    qx, qy, qz, qw = rpy_deg_to_quat(rx, ry, rz)
    tf = TransformStamped()
    tf.header.frame_id = parent
    tf.child_frame_id = child
    tf.transform.translation.x = float(xyz[0])
    tf.transform.translation.y = float(xyz[1])
    tf.transform.translation.z = float(xyz[2])
    tf.transform.rotation.x = qx
    tf.transform.rotation.y = qy
    tf.transform.rotation.z = qz
    tf.transform.rotation.w = qw
    return tf

def _load_stl_as_shape_msgs_mesh(abs_path: str, logger) -> RosMesh | None:
    if not abs_path or not os.path.exists(abs_path):
        logger.warning(f"Mesh nicht gefunden: {abs_path}")
        return None
    try:
        import trimesh
        mesh_raw = trimesh.load(abs_path, force='mesh')
        mesh_raw.apply_scale(0.001)  # ✅ mm → m
        ros_mesh = RosMesh()
        for v in mesh_raw.vertices:
            ros_mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in mesh_raw.faces:
            ros_mesh.triangles.append(MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]))
        return ros_mesh
    except Exception as e:
        logger.warning(f"Mesh Fehler {abs_path}: {e}")
        return None

class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        scene_yaml = self.get_parameter(PARAM_SCENE_CONFIG).value
        if not scene_yaml or not os.path.exists(scene_yaml):
            raise FileNotFoundError(f"Scene YAML fehlt: {scene_yaml}")

        with open(scene_yaml, "r") as f:
            data = yaml.safe_load(f) or {}
            self.scene_objects = data.get("scene_objects", [])

        self.static_tf = StaticTransformBroadcaster(self)
        self.co_pub = self.create_publisher(CollisionObject, "collision_object", 10)
        self.base_dir = os.path.dirname(os.path.abspath(scene_yaml))

        # **Einmal initial senden**
        self._publish_scene()

        # ✅ Timer: republish alle 1 Sekunde
        self.timer = self.create_timer(1.0, self._publish_scene)

    def _publish_scene(self):
        tfs = []
        for entry in self.scene_objects:
            obj_id = entry.get("id")
            parent = entry.get("frame", FRAME_WORLD)
            xyz = entry.get("position", [0, 0, 0])
            rpy_deg = entry.get("rpy_deg", [0, 0, 0])
            mesh_rel = entry.get("mesh", "")

            tfs.append(_make_static_tf(parent, obj_id, xyz, rpy_deg))

            if mesh_rel:
                abs_mesh = mesh_rel if os.path.isabs(mesh_rel) else os.path.join(self.base_dir, mesh_rel)
                mesh = _load_stl_as_shape_msgs_mesh(abs_mesh, self.get_logger())
                if mesh:
                    co = CollisionObject()
                    co.id = obj_id
                    co.header = Header(frame_id=obj_id)
                    co.meshes = [mesh]
                    co.mesh_poses.append(PoseStamped().pose)
                    co.operation = CollisionObject.ADD
                    self.co_pub.publish(co)

        self.static_tf.sendTransform(tfs)

def main():
    rclpy.init()
    rclpy.spin(SceneManager())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
