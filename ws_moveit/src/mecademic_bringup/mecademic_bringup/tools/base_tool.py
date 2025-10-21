#!/usr/bin/env python3
from __future__ import annotations
import os, trimesh
from moveit_msgs.msg import PlanningScene, AttachedCollisionObject, CollisionObject
from geometry_msgs.msg import Pose, Point
from shape_msgs.msg import Mesh, MeshTriangle
from ament_index_python.packages import get_package_share_directory
from mecademic_bringup.utils import rpy_deg_to_quat

ATTACHED_OBJ_ID = "active_tool_mesh"


class ToolPluginBase:
    """Basisverhalten für alle Tools: TCP-Offset setzen + Mesh anhängen/entfernen."""

    def _resolve_mesh_path(self, uri: str) -> str:
        if uri.startswith("package://"):
            pkg, rel = uri[10:].split("/", 1)
            return os.path.join(get_package_share_directory(pkg), rel)
        return uri

    def _load_mesh(self, mesh_path: str) -> Mesh:
        tri = trimesh.load(mesh_path)
        tri.apply_scale(0.001)
        mesh = Mesh()
        for v in tri.vertices:
            mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        for f in tri.faces:
            mesh.triangles.append(MeshTriangle(vertex_indices=list(map(int, f))))
        return mesh

    # ------------------------------------------------------------------
    # Attach
    # ------------------------------------------------------------------
    def on_attach(self, node, cfg) -> bool:
        """Standardattach: TCP verschieben + Mesh anhängen."""
        tcp_xyz = cfg.get("tcp_offset", [0, 0, 0])
        tcp_rpy = cfg.get("tcp_rpy", [0, 0, 0])
        mesh_uri = cfg.get("mesh", "")
        mesh_off = cfg.get("mesh_offset", [0, 0, 0])
        mesh_rpy = cfg.get("mesh_rpy", [0, 0, 0])
        mount = cfg.get("mount_frame", "tool_mount")

        # TCP Offset setzen
        node.update_tcp_offset(tcp_xyz, tcp_rpy)

        if not mesh_uri:
            node.get_logger().info("⚙️ Kein Mesh definiert – nur TCP gesetzt.")
            return True

        # Mesh vorbereiten
        mesh = self._load_mesh(self._resolve_mesh_path(mesh_uri))
        aco = self._make_attach_msg(mount, mesh, mesh_off, mesh_rpy)

        # Scene-Diff erzeugen
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]

        node.scene_pub.publish(scene)
        node.get_logger().info("📤 Scene-Diff (Attached Object) publiziert")
        return True

    # ------------------------------------------------------------------
    # Detach
    # ------------------------------------------------------------------
    def on_detach(self, node, cfg) -> bool:
        """Standarddetach: Tool-Mesh entfernen."""
        aco = AttachedCollisionObject()
        aco.object.id = ATTACHED_OBJ_ID
        aco.link_name = node.frames["tool_mount"]
        aco.object.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects = [aco]

        node.scene_pub.publish(scene)
        node.get_logger().info("🧹 Tool entfernt + Scene aktualisiert")
        return True

    # ------------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------------
    def _make_attach_msg(self, mount, mesh, offset, rpy):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = offset
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw

        msg = AttachedCollisionObject()
        msg.object.header.frame_id = mount        # ✅ nur dieses Feld existiert
        msg.link_name = mount
        msg.object.id = ATTACHED_OBJ_ID
        msg.object.meshes = [mesh]
        msg.object.mesh_poses = [pose]
        msg.object.operation = CollisionObject.ADD
        return msg
