#!/usr/bin/env python3
# mecademic_bringup/scene/scene_manager_node.py

from __future__ import annotations

import os
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import Mesh as RosMesh
from shape_msgs.msg import MeshTriangle
from geometry_msgs.msg import Point
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

# Paket-Utils
from mecademic_bringup.common.frames import FRAME_WORLD
from mecademic_bringup.common.params import PARAM_SCENE_CONFIG
from mecademic_bringup.utils import rpy_deg_to_quat

# ---- Helfer -----------------------------------------------------------------

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
    """
    Sehr robuste Mesh-Ladung:
    - Prüft Existenz
    - Keine Exceptions nach oben (nur Warnungen), damit der Node niemals crasht.
    - Nutzt eine minimalistische STL-Lesung ohne externe Abhängigkeiten,
      wenn trimesh nicht verfügbar oder fehlschlägt.
    """
    if not abs_path:
        return None
    if not os.path.isabs(abs_path):
        logger.warning(f"Mesh-Pfad ist nicht absolut: '{abs_path}'")
    if not os.path.exists(abs_path):
        logger.warning(f"Mesh-Datei nicht gefunden: '{abs_path}'")
        return None

    # Versuch 1: trimesh (falls installiert)
    try:
        import trimesh  # type: ignore
        mesh_raw = trimesh.load(abs_path, force='mesh')
        if mesh_raw is None or mesh_raw.is_empty:
            logger.warning(f"Mesh leer oder unlesbar: '{abs_path}'")
            return None

        ros_mesh = RosMesh()
        # Vertices
        for v in mesh_raw.vertices:
            ros_mesh.vertices.append(Point(x=float(v[0]), y=float(v[1]), z=float(v[2])))
        # Faces
        for f in mesh_raw.faces:
            tri = MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])])
            ros_mesh.triangles.append(tri)
        return ros_mesh

    except Exception as e:
        logger.warning(f"trimesh Fehler für '{abs_path}': {e}. Versuche Fallback.")

# ---- Node -------------------------------------------------------------------

class SceneManager(Node):
    """
    Liest scene.yaml und:
      1) Sendet IMMER die statischen TFs (auch ohne Mesh).
      2) Lädt und publiziert CollisionObjects NUR, wenn ein gültiges Mesh existiert.
    """

    def __init__(self):
        super().__init__("scene_manager")

        # Parameter: Pfad zur scene.yaml
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        scene_yaml = self.get_parameter(PARAM_SCENE_CONFIG).value

        if not scene_yaml or not os.path.exists(scene_yaml):
            self.get_logger().error(f"❌ Szene YAML nicht gefunden: {scene_yaml}")
            raise FileNotFoundError(f"Scene YAML fehlt: {scene_yaml}")

        with open(scene_yaml, "r") as f:
            doc = yaml.safe_load(f) or {}

        self.scene_objects = doc.get("scene_objects", [])
        if not isinstance(self.scene_objects, list):
            self.get_logger().error("❌ 'scene_objects' muss eine Liste sein.")
            self.scene_objects = []

        # TF Broadcaster
        self.static_tf = StaticTransformBroadcaster(self)

        # Publisher für CollisionObjects
        self.co_pub = self.create_publisher(CollisionObject, "collision_object", 10)

        # Basis-Verzeichnis für relative Mesh-Pfade (= Ordner der YAML-Datei)
        self.base_dir = os.path.dirname(os.path.abspath(scene_yaml))

        # Ausführen
        self._publish_static_tfs_and_meshes()

    # -- Hauptlogik -----------------------------------------------------------

    def _publish_static_tfs_and_meshes(self):
        tfs: list[TransformStamped] = []

        for entry in self.scene_objects:
            try:
                obj_id: str = entry["id"]
                parent: str = entry.get("frame", FRAME_WORLD)

                # Position/Orientierung (defaults)
                xyz = entry.get("position", [0.0, 0.0, 0.0])
                rpy_deg = entry.get("rpy_deg", [0.0, 0.0, 0.0])

                # 1) Immer TF senden
                tf = _make_static_tf(parent, obj_id, xyz, rpy_deg)
                tfs.append(tf)

                # 2) Optional: Mesh laden & CollisionObject publizieren
                mesh_rel = entry.get("mesh", "")
                optional = bool(entry.get("optional", False))

                if mesh_rel:
                    abs_path = mesh_rel
                    if not os.path.isabs(abs_path):
                        abs_path = os.path.join(self.base_dir, mesh_rel)

                    ros_mesh = _load_stl_as_shape_msgs_mesh(abs_path, self.get_logger())
                    if ros_mesh is None:
                        msg = f"⚠️ Mesh für '{obj_id}' nicht geladen → {mesh_rel}"
                        if optional:
                            self.get_logger().warning(msg + " (optional, übersprungen)")
                            continue
                        else:
                            self.get_logger().warning(msg + " (nicht optional, übersprungen)")
                            continue

                    # CollisionObject zusammenbauen
                    co = CollisionObject()
                    co.header = Header(frame_id=obj_id)  # Mesh im lokalen Frame des Objektes
                    co.id = obj_id
                    co.meshes = [ros_mesh]

                    # Pose des Mesh relativ zu seinem eigenen Frame → Identität
                    mesh_pose = PoseStamped()
                    mesh_pose.header.frame_id = obj_id
                    mesh_pose.pose.orientation.w = 1.0
                    co.mesh_poses.append(mesh_pose.pose)

                    co.operation = CollisionObject.ADD
                    self.co_pub.publish(co)
                    self.get_logger().info(f"✅ Mesh geladen & publiziert: {obj_id} ← {mesh_rel}")
                else:
                    # Kein Mesh → nur TF (genau so gewollt)
                    self.get_logger().info(f"ℹ️ Nur TF (kein Mesh): {obj_id}")

            except Exception as e:
                self.get_logger().warning(f"⚠️ Fehler in Szene-Eintrag {entry}: {e}")

        # Alle TFs auf einmal senden
        if tfs:
            self.static_tf.sendTransform(tfs)
            self.get_logger().info("✅ Static TFs gesendet (aus scene.yaml)")

# ---- main -------------------------------------------------------------------

def main():
    rclpy.init()
    node = SceneManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
