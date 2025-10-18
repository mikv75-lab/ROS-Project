#!/usr/bin/env python3
from __future__ import annotations
import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import StaticTransformBroadcaster

from mecademic_bringup.utils import rpy_deg_to_quat, resolve_mesh_path
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG

VALID_FRAMES = {"world", "scene", "meca_mount"}


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")

        # ---- Parameter: scene_yaml ----
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        yaml_file = self.get_parameter(PARAM_SCENE_CONFIG).get_parameter_value().string_value

        if not yaml_file or not os.path.exists(yaml_file):
            self.get_logger().error(f"❌ Szene YAML nicht gefunden: {yaml_file}")
            sys.exit(1)

        self.get_logger().info(f"📄 Szene YAML geladen: {yaml_file}")

        # ---- Publisher ----
        self.pub_markers = self.create_publisher(MarkerArray, "/scene/visual", 10)
        self.pub_collision = self.create_publisher(CollisionObject, "/collision_object", 10)

        self.scene_objects = self._load_scene_yaml(yaml_file)

        # ---- Publish Scene ----
        self.publish_scene_full()

    # ============================================================
    #    YAML LOAD
    # ============================================================
    def _load_scene_yaml(self, path):
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if "scene_objects" not in data or not isinstance(data["scene_objects"], list):
            raise RuntimeError("scene_yaml muss eine Liste 'scene_objects' enthalten")

        objects = []
        for o in data["scene_objects"]:
            for key in ["id", "mesh", "frame", "position", "rpy_deg"]:
                if key not in o:
                    raise RuntimeError(f"{key} fehlt in Szeneobjekt: {o}")

            if o["frame"] not in VALID_FRAMES:
                raise RuntimeError(f"Ungültiger Frame '{o['frame']}', erlaubt: {VALID_FRAMES}")

            mesh_file = resolve_mesh_path(o["mesh"])
            if not os.path.exists(mesh_file):
                raise RuntimeError(f"Mesh existiert nicht: {mesh_file}")

            objects.append({
                "id": o["id"],
                "mesh": mesh_file,
                "frame": o["frame"],
                "position": list(map(float, o["position"])),
                "rpy_deg": list(map(float, o["rpy_deg"])),
                "optional": o.get("optional", False)
            })
        return objects

    # ============================================================
    #    SCENE PUBLISH
    # ============================================================
    def publish_scene_full(self):
        marker_array = MarkerArray()

        for o in self.scene_objects:
            m = Marker()
            m.header.frame_id = o["frame"]
            m.ns = "scene"
            m.id = hash(o["id"]) % 10000
            m.type = Marker.MESH_RESOURCE
            m.mesh_resource = "file://" + o["mesh"]
            m.pose.position.x, m.pose.position.y, m.pose.position.z = o["position"]
            q = rpy_deg_to_quat(*o["rpy_deg"])
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            m.scale.x = m.scale.y = m.scale.z = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = (0.7, 0.7, 0.7, 1.0)
            marker_array.markers.append(m)

            collision = CollisionObject()
            collision.id = o["id"]
            collision.header.frame_id = o["frame"]
            collision.meshes = [self._load_mesh(o["mesh"])]
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = o["position"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            collision.mesh_poses.append(pose)
            collision.operation = CollisionObject.ADD

            self.pub_collision.publish(collision)
            self.get_logger().info(f"🧱 Collision hinzugefügt: {o['id']}")

        self.pub_markers.publish(marker_array)
        self.get_logger().info("✅ Szene geladen + publiziert")

    # ============================================================
    #    MESH LOAD
    # ============================================================
    def _load_mesh(self, mesh_path):
        from trimesh.exchange.stl import load_stl
        mesh_data = load_stl(mesh_path, solid=True)

        mesh = Mesh()
        for triangle in mesh_data.faces:
            tri = MeshTriangle()
            tri.vertex_indices = triangle
            mesh.triangles.append(tri)
        for vertex in mesh_data.vertices:
            pt = type(mesh.vertices[0])() if mesh.vertices else type(mesh.vertices.append)  # Safe type
            mesh.vertices.append(vertex)
        return mesh


def main():
    rclpy.init()
    node = SceneManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
