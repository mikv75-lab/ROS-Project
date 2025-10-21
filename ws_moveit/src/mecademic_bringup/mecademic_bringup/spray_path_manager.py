#!/usr/bin/env python3
from __future__ import annotations
import os, yaml, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Empty, String
from mecademic_bringup.common.params import PARAM_SPRAY_PATH_CONFIG
from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES


class SprayPathManager(Node):
    """
    SprayPathManager
    ----------------
    Lädt, publiziert und verwaltet Spray-Pfade.
    Unterstützt:
      • load/reload (/meca/spray_path/set)
      • clear       (/meca/spray_path/clear)
      • poses       (/meca/spray_path/poses)
      • current     (/meca/spray_path/current)
    """

    def __init__(self):
        super().__init__("spray_path_manager")

        # --- Common ---
        self.topics = Topics()
        self.frames = FRAMES
        self._frame = self.frames["workspace_center"]

        # --- Parameter ---
        self.declare_parameter(PARAM_SPRAY_PATH_CONFIG, "")
        self.spray_paths_yaml = self.get_parameter(PARAM_SPRAY_PATH_CONFIG).get_parameter_value().string_value
        if not self.spray_paths_yaml or not os.path.exists(self.spray_paths_yaml):
            raise FileNotFoundError(f"❌ {PARAM_SPRAY_PATH_CONFIG} Datei fehlt: {self.spray_paths_yaml}")

        # --- Publisher ---
        self._pub_poses = self.create_publisher(PoseArray, self.topics.spray_path_poses, qos_default())
        self._pub_current = self.create_publisher(String, self.topics.spray_path_current, qos_latched())

        # --- Subscriber ---
        self._sub_reload = self.create_subscription(Empty, self.topics.spray_path_set, self._on_reload, qos_default())
        self._sub_clear = self.create_subscription(Empty, self.topics.spray_path_clear, self._on_clear, qos_default())

        # --- Internals ---
        self._spray_paths_dir = ""
        self._spray_path_file = ""
        self._last_data = None

        # --- Initial Load ---
        self.load_config()

        # Publish verzögert, um QoS-Sync zu garantieren
        self._init_timer = self.create_timer(1.0, self._initial_publish_once)

        self.get_logger().info(f"✅ SprayPathManager gestartet (Config: {self.spray_paths_yaml})")

    # ---------------------------------------------------------------
    def _initial_publish_once(self):
        """Publiziert initialen Path nach Startup."""
        self.publish_current_path()
        self._init_timer.cancel()

    # ---------------------------------------------------------------
    def load_config(self):
        """Lädt aktuelle Konfiguration (spray_paths.yaml)."""
        with open(self.spray_paths_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        spray_cfg = data.get("spray_paths", {})
        base_dir = os.path.dirname(self.spray_paths_yaml)
        self._spray_paths_dir = os.path.join(base_dir, spray_cfg.get("dir", "spray_paths"))
        self._spray_path_file = spray_cfg.get("current", "")

        self.get_logger().info(f"📂 SprayPath-Ordner: {self._spray_paths_dir}")
        self.get_logger().info(f"🎯 Aktiver Path: {self._spray_path_file}")
        self.publish_current_name()

    # ---------------------------------------------------------------
    def _load_spray_data(self, path_file: str) -> dict:
        """Robustes Laden des SprayPath-YAMLs."""
        try:
            with open(path_file, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self.get_logger().error(f"❌ Fehler beim Lesen von {path_file}: {e}")
            return {}
        for key in ("spray_path", "path", "data"):
            if key in data and isinstance(data[key], dict):
                return data[key]
        return data if isinstance(data, dict) else {}

    # ---------------------------------------------------------------
    def publish_current_path(self):
        """Publiziert aktuellen SprayPath als PoseArray."""
        if not self._spray_path_file:
            self.get_logger().warning("⚠️ Kein aktiver Spray-Path definiert")
            self.publish_current_name()
            return

        path_file = os.path.join(self._spray_paths_dir, self._spray_path_file)
        if not os.path.exists(path_file):
            self.get_logger().error(f"❌ Datei fehlt: {path_file}")
            self.publish_current_name()
            return

        spray_data = self._load_spray_data(path_file)
        if not spray_data:
            self.get_logger().error(f"❌ Ungültiges Format: {path_file}")
            return

        points = spray_data.get("points", [])
        if not points:
            self.get_logger().warning(f"⚠️ Keine Punkte in {self._spray_path_file}")
            return

        frame = spray_data.get("frame", self._frame)
        self._frame = frame

        poses = []
        for p in points:
            pose = Pose()
            pose.position.x = float(p.get("x", 0.0))
            pose.position.y = float(p.get("y", 0.0))
            pose.position.z = float(p.get("z", 0.0))
            pose.orientation.x = float(p.get("qx", 0.0))
            pose.orientation.y = float(p.get("qy", 0.0))
            pose.orientation.z = float(p.get("qz", 0.0))
            pose.orientation.w = float(p.get("qw", 1.0))
            poses.append(pose)

        pa = PoseArray()
        pa.header.frame_id = frame
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = poses
        self._pub_poses.publish(pa)

        self.publish_current_name()
        self._last_data = spray_data

        self.get_logger().info(f"✅ Publiziert '{self._spray_path_file}' mit {len(poses)} Punkten (Frame: {frame})")

    # ---------------------------------------------------------------
    def publish_current_name(self):
        """Publiziert den aktuellen SprayPath-Dateinamen."""
        msg = String()
        msg.data = self._spray_path_file or ""
        self._pub_current.publish(msg)
        self.get_logger().debug(f"📰 Aktueller SprayPath: {msg.data}")

    # ---------------------------------------------------------------
    def clear(self, _msg: Empty = None):
        """Leert aktuellen Path."""
        pa = PoseArray()
        pa.header.frame_id = self._frame
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = []
        self._pub_poses.publish(pa)
        self._spray_path_file = ""
        self.publish_current_name()
        self.get_logger().info("🧹 SprayPath gelöscht")

    # ---------------------------------------------------------------
    def _on_reload(self, _msg: Empty):
        self.get_logger().info("🔄 Reload angefordert")
        self.load_config()
        self.publish_current_path()

    def _on_clear(self, _msg: Empty):
        self.clear()


# ------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SprayPathManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
