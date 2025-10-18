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

SUBSTRATE_MOUNT_CONFIG = "config/substrate_mounts.yaml"  # <-- neue Mount YAML


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")

        # ---- Szene YAML laden ----
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        scene_yaml = self.get_parameter(PARAM_SCENE_CONFIG).get_parameter_value().string_value

        if not scene_yaml or not os.path.exists(scene_yaml):
            self.get_logger().error(f"❌ Szene YAML nicht gefunden: {scene_yaml}")
            sys.exit(1)

        # ---- Substrate Mount YAML laden ----
        mount_yaml = resolve_mesh_path(SUBSTRATE_MOUNT_CONFIG)
        if not os.path.exists(mount_yaml):
            self.get_logger().error(f"❌ substrate_mounts.yaml nicht gefunden: {mount_yaml}")
            sys.exit(1)

        self.mount_data = self._load_mount_yaml(mount_yaml)
        self.scene_objects = self._load_scene_yaml(scene_yaml)

        # ---- Publisher ----
        self.pub_markers = self.create_publisher(MarkerArray, "/scene/visual", 10)
        self.apply_client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.apply_client.wait_for_service()

        # Starte Szene
        self.publish_scene_full()

    def _load_mount_yaml(self, path):
        with open(path, "r") as f:
            mount_config = yaml.safe_load(f)
        active = mount_config.get("active_mount")
        mounts = mount_config.get("mounts", {})
        if active not in mounts:
            raise RuntimeError(f"Active mount '{active}' fehlt in substrate_mounts.yaml!")
        return mounts[active]

    def _load_scene_yaml(self, path):
        with open(path, "r") as f:
            return yaml.safe_load(f).get("scene_objects", [])

    def publish_scene_full(self):
        marker_array = MarkerArray()
        collision_objects = []

        # --- Substrate Mount zuerst ---
        mount_mesh = resolve_mesh_path(self.mount_data["mesh"])
        scene_offset = self.mount_data["scene_offset"]["xyz"]
        scene_rpy = self.mount_data["scene_offset"]["rpy_deg"]

        mount_co = self._make_collision_object(
            "substrate_mount",
            mount_mesh,
            "world",
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )
        collision_objects.append(mount_co)

        # --- Dynamischen frame "scene" simulieren ---
        # Alle scene-Objekte bekommen automatisch Offset aus Mount YAML
        for obj in self.scene_objects:
            mesh_file = resolve_mesh_path(obj["mesh"])
            pos = obj["position"]
            rpy = obj["rpy_deg"]

            if obj["frame"] == "scene":
                obj["frame"] = "world"
                pos = [
                    pos[0] + scene_offset[0],
                    pos[1] + scene_offset[1],
                    pos[2] + scene_offset[2],
                ]

            co = self._make_collision_object(obj["id"], mesh_file, obj["frame"], pos, rpy)
            collision_objects.append(co)

        ps = PlanningScene()
        ps.is_diff = True
        ps.world.collision_objects = collision_objects

        req = ApplyPlanningScene.Request()
        req.scene = ps
        self.apply_client.call_async(req)

        self.get_logger().info("✅ Szene aufgebaut mit substrate_mount + scene-Offset.")

    def _make_collision_object(self, obj_id, mesh_path, frame, pos, rpy):
        co = CollisionObject()
        co.id = obj_id
        co.header.frame_id = frame

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        q = rpy_deg_to_quat(*rpy)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

        co.meshes = [self._load_mesh(mesh_path)]
        co.mesh_poses = [pose]
        co.operation = CollisionObject.ADD
        return co

    def _load_mesh(self, mesh_path):
        mesh = trimesh.load(mesh_path)
        msg = Mesh()
        for v in mesh.vertices:
            p = Point(x=v[0], y=v[1], z=v[2])
            msg.vertices.append(p)
        for f in mesh.faces:
            tri = MeshTriangle(vertex_indices=f)
            msg.triangles.append(tri)
        return msg


def main():
        rclpy.init()
        node = SceneManager()
        rclpy.spin(node)
        rclpy.shutdown()


if __name__ == "__main__":
    main()
