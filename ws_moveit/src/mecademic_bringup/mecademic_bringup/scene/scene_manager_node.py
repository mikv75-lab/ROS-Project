#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseArray
from ament_index_python.packages import get_package_share_directory

from .utils import resolve_mesh_path
from .object_registry import ObjectRegistry, SceneObjectState
import yaml
from .mesh_manager import MeshManager
from .spray_path_manager import SprayPathManager

PKG = 'mecademic_bringup'

# Topics (final)
TOPICS = {
    "mount_set":        "/scene/mount/set",
    "mount_clear":      "/scene/mount/clear",
    "mount_info":       "/scene/mount/info",
    "substrate_set":    "/scene/substrate/set",
    "substrate_clear":  "/scene/substrate/clear",
    "substrate_info":   "/scene/substrate/info",
    "env_set":          "/scene/environment/set",
    "env_clear":        "/scene/environment/clear",
    "env_info":         "/scene/environment/info",
    "spray_set":        "/scene/spraypath/set",
    "spray_clear":      "/scene/spraypath/clear",
}


def load_scene_yaml(path: str):
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data.get("scene_objects", [])

class SceneManagerNode(Node):
    def __init__(self):
        super().__init__("scene_manager")

        # Params
        pkg_share = get_package_share_directory(PKG)
        default_scene = os.path.join(pkg_share, "config", "scene.yaml")
        self.declare_parameter("scene_config", default_scene)

        # State & helpers
        self.registry = ObjectRegistry()
        self.mesh = MeshManager(self)
        self.spray = SprayPathManager(self, frame="workspace_center")

        # Register known objects (ids + frames). Mesh wird dynamisch gesetzt.
        self.registry.set(SceneObjectState(id="environment",     frame="world"))
        self.registry.set(SceneObjectState(id="substrate_mount", frame="world"))
        self.registry.set(SceneObjectState(id="substrate",       frame="workspace_center"))

        # Publishers (info)
        self.pub_mount_info     = self.create_publisher(String, TOPICS["mount_info"], 10)
        self.pub_substrate_info = self.create_publisher(String, TOPICS["substrate_info"], 10)
        self.pub_env_info       = self.create_publisher(String, TOPICS["env_info"], 10)

        # Subscriptions
        self.create_subscription(String, TOPICS["mount_set"],       self._on_mount_set, 10)
        self.create_subscription(Empty,  TOPICS["mount_clear"],     self._on_mount_clear, 10)

        self.create_subscription(String, TOPICS["substrate_set"],   self._on_substrate_set, 10)
        self.create_subscription(Empty,  TOPICS["substrate_clear"], self._on_substrate_clear, 10)

        self.create_subscription(String, TOPICS["env_set"],         self._on_env_set, 10)
        self.create_subscription(Empty,  TOPICS["env_clear"],       self._on_env_clear, 10)

        self.create_subscription(PoseArray, TOPICS["spray_set"],    self._on_spray_set, 10)
        self.create_subscription(Empty,     TOPICS["spray_clear"],  self._on_spray_clear, 10)

        # Boot: scene.yaml laden und vorhandene Meshes anzeigen
        scene_file = self.get_parameter("scene_config").get_parameter_value().string_value
        self._load_initial_scene(scene_file)

    
    # ---- init scene ----
    def _load_initial_scene(self, scene_file: str):
        self.get_logger().info(f"📦 Scene laden: {scene_file}")

        try:
            cfg = load_scene_yaml(scene_file)
        except Exception as e:
            self.get_logger().error(f"❌ Scene konnte nicht geladen werden: {scene_file}: {e}")
            return

        if not cfg:
            self.get_logger().warn("⚠️ Scene YAML enthält keine 'scene_objects'.")
            return

        for o in cfg:
            obj_id = o.get("id")
            if obj_id not in ("environment", "substrate_mount", "substrate"):
                continue  # andere Objekte aktuell ignorieren

            mesh_rel = o.get("mesh", "")
            mesh_abs = resolve_mesh_path(mesh_rel) if mesh_rel else ""

            # Optional-Flag berücksichtigen
            optional = o.get("optional", False)
            if not mesh_abs:
                if optional:
                    self.get_logger().info(f"ℹ️ Optionales Objekt '{obj_id}' ohne Mesh – übersprungen.")
                    continue
                else:
                    self.get_logger().warn(f"⚠️ Objekt '{obj_id}' hat kein Mesh gesetzt!")
                    continue

            # Zustandsdaten im Registry pflegen
            st = self.registry.get(obj_id)
            if not st:
                self.get_logger().warn(f"⚠️ Unbekanntes Scene-Objekt in YAML: '{obj_id}'")
                continue

            # Mesh laden
            pos = o.get("position", [0,0,0])
            rpy = o.get("rpy_deg", [0,0,0])
            self.registry.update_mesh(obj_id, mesh_abs)
            self.mesh.show_or_update(
                obj_id=obj_id,
                parent=st.frame,
                mesh_abs=mesh_abs,
                xyz=pos,
                rpy_deg=rpy
            )

            self.get_logger().info(f"✅ {obj_id} geladen: {mesh_abs} @ {st.frame}")

        # Status publizieren
        self._publish_infos()




    # ---- infos ----
    def _publish_infos(self):
        s = String()
        s.data = self.registry.mesh("substrate_mount")
        self.pub_mount_info.publish(s)
        s = String()
        s.data = self.registry.mesh("substrate")
        self.pub_substrate_info.publish(s)
        s = String()
        s.data = self.registry.mesh("environment")
        self.pub_env_info.publish(s)

    # ---- handlers: mount ----
    def _on_mount_set(self, msg: String):
        p = resolve_mesh_path(msg.data.strip())
        if not os.path.isfile(p):
            self.get_logger().error(f"Mount: Datei nicht gefunden: {p}")
            return
        self.registry.update_mesh("substrate_mount", p)
        st = self.registry.get("substrate_mount")
        self.mesh.show_or_update(obj_id="substrate_mount", parent=st.frame, mesh_abs=p)
        self._publish_infos()

    def _on_mount_clear(self, _msg: Empty):
        st = self.registry.get("substrate_mount")
        self.mesh.clear(obj_id="substrate_mount", parent=st.frame)
        self.registry.update_mesh("substrate_mount", "")
        self._publish_infos()

    # ---- handlers: substrate ----
    def _on_substrate_set(self, msg: String):
        p = resolve_mesh_path(msg.data.strip())
        if not os.path.isfile(p):
            self.get_logger().error(f"Substrate: Datei nicht gefunden: {p}")
            return
        self.registry.update_mesh("substrate", p)
        st = self.registry.get("substrate")
        self.mesh.show_or_update(obj_id="substrate", parent=st.frame, mesh_abs=p)
        self._publish_infos()

    def _on_substrate_clear(self, _msg: Empty):
        st = self.registry.get("substrate")
        self.mesh.clear(obj_id="substrate", parent=st.frame)
        self.registry.update_mesh("substrate", "")
        self._publish_infos()

    # ---- handlers: environment ----
    def _on_env_set(self, msg: String):
        p = resolve_mesh_path(msg.data.strip())
        if not os.path.isfile(p):
            self.get_logger().error(f"Environment: Datei nicht gefunden: {p}")
            return
        self.registry.update_mesh("environment", p)
        st = self.registry.get("environment")
        self.mesh.show_or_update(obj_id="environment", parent=st.frame, mesh_abs=p, rgba=(0.5,0.7,0.9,0.35))
        self._publish_infos()

    def _on_env_clear(self, _msg: Empty):
        st = self.registry.get("environment")
        self.mesh.clear(obj_id="environment", parent=st.frame)
        self.registry.update_mesh("environment", "")
        self._publish_infos()

    # ---- handlers: spray ----
    def _on_spray_set(self, msg: PoseArray):
        # ignoriert frame_id absichtlich – wir zeichnen immer in 'workspace_center'
        self.spray.draw(list(msg.poses))

    def _on_spray_clear(self, _msg: Empty):
        self.spray.clear()

def main():
    rclpy.init()
    rclpy.spin(SceneManagerNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
