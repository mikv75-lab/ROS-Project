#!/usr/bin/env python3
from __future__ import annotations
import os
import sys
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle
from visualization_msgs.msg import MarkerArray, Marker

from mecademic_bringup.utils import rpy_deg_to_quat, resolve_mesh_path
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG
import trimesh

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

        # ---- Publisher (nur Visualisierung) ----
        self.pub_markers = self.create_publisher(MarkerArray, "/scene/visual", 10)

        # ---- ApplyPlanningScene-Service-Client ----
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        if not self.apply_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("⏳ /apply_planning_scene noch nicht verfügbar – warte weiter...")
            # nochmal länger warten, weil move_group oft später hochkommt
            if not self.apply_client.wait_for_service(timeout_sec=20.0):
                self.get_logger().error("❌ Service /apply_planning_scene nicht verfügbar. Beende.")
                sys.exit(1)

        # ---- Szene aus YAML laden ----
        self.scene_objects = self._load_scene_yaml(yaml_file)

        # ---- Szene publizieren ----
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
                "optional": o.get("optional", False),
            })
        return objects

    # ============================================================
    #    SCENE PUBLISH
    # ============================================================
    def publish_scene_full(self):
        """Baut Marker (RViz) + CollisionObjects und sendet sie per ApplyPlanningScene (Diff) an MoveIt."""
        marker_array = MarkerArray()
        collision_objects = []

        for o in self.scene_objects:
            # --- RViz Marker ---
            m = Marker()
            m.header.frame_id = o["frame"]
            m.header.stamp = self.get_clock().now().to_msg()
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

            # --- MoveIt CollisionObject ---
            co = CollisionObject()
            co.id = o["id"]
            co.header.frame_id = o["frame"]
            co.header.stamp = self.get_clock().now().to_msg()

            mesh_msg = self._load_mesh(o["mesh"])
            co.meshes = [mesh_msg]

            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = o["position"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
            co.mesh_poses.append(pose)

            co.operation = CollisionObject.ADD
            collision_objects.append(co)

            self.get_logger().info(f"🧱 Collision vorbereitet: {o['id']}")

        # --- Visualisierung raus ---
        self.pub_markers.publish(marker_array)

        # --- ApplyPlanningScene (Diff) senden ---
        ps = PlanningScene()
        ps.is_diff = True  # wichtig: nur Änderungen anwenden
        ps.world.collision_objects = collision_objects

        req = ApplyPlanningScene.Request()
        req.scene = ps

        future = self.apply_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if not future.done() or future.result() is None or not future.result().success:
            self.get_logger().error("❌ ApplyPlanningScene fehlgeschlagen – Szene wurde nicht übernommen.")
        else:
            self.get_logger().info("✅ Szene an MoveIt übergeben (ApplyPlanningScene).")

    # ============================================================
    #    MESH LOAD
    # ============================================================
    def _load_mesh(self, mesh_path: str) -> Mesh:
        """Load STL mesh and convert to shape_msgs/Mesh"""
        try:
            mesh = trimesh.load(mesh_path, force='mesh')
        except Exception as e:
            self.get_logger().error(f"❌ Could not load mesh: {mesh_path} ({e})")
            raise

        if mesh.is_empty:
            raise RuntimeError(f"Mesh is empty: {mesh_path}")

        mesh_msg = Mesh()

        # Convert vertices
        for vertex in mesh.vertices:
            p = Point()
            p.x, p.y, p.z = float(vertex[0]), float(vertex[1]), float(vertex[2])
            mesh_msg.vertices.append(p)

        # Convert faces
        for face in mesh.faces:
            tri = MeshTriangle()
            tri.vertex_indices[0] = int(face[0])
            tri.vertex_indices[1] = int(face[1])
            tri.vertex_indices[2] = int(face[2])
            mesh_msg.triangles.append(tri)

        return mesh_msg


def main():
    rclpy.init()
    node = SceneManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
