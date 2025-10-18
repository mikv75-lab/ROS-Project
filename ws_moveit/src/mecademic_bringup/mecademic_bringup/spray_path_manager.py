#!/usr/bin/env python3
from __future__ import annotations
import os
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

from mecademic_bringup.common.topics import (
    TOPIC_SCENE_SPRAY_SET,
    TOPIC_SCENE_SPRAY_CLEAR,
    TOPIC_SCENE_SPRAY_INFO,
    TOPIC_SPRAY_PATH_MARKERS,
)


class SprayPathManager(Node):
    def __init__(self):
        super().__init__("spray_path_manager")

        # Pfade
        pkg_share = os.path.join(
            os.path.dirname(__file__), "..", ".."
        )
        self.config_yaml = os.path.abspath(os.path.join(pkg_share, "config", "spray_paths.yaml"))

        # Directory + Current Rezept
        cfg = self._load_yaml_file(self.config_yaml)
        self.paths_dir = cfg.get("spray_paths", {}).get("dir", "spray_paths")
        self.current_recipe = cfg.get("spray_paths", {}).get("current", None)
        self.paths_dir = os.path.join(pkg_share, self.paths_dir)

        # Publisher
        self.marker_pub = self.create_publisher(Marker, TOPIC_SPRAY_PATH_MARKERS, 10)
        self.info_pub = self.create_publisher(String, TOPIC_SCENE_SPRAY_INFO, 10)

        # Subscriber
        self.create_subscription(String, TOPIC_SCENE_SPRAY_SET, self.on_set, 10)
        self.create_subscription(String, TOPIC_SCENE_SPRAY_CLEAR, self.on_clear, 10)

        # Automatischer Start
        if self.current_recipe:
            full_path = os.path.join(self.paths_dir, self.current_recipe)
            if os.path.exists(full_path):
                self._load_and_publish(full_path)
            else:
                self._publish_info(f"⚠️ Aktives Rezept nicht gefunden: {full_path}")
        else:
            self._publish_info("ℹ️ Kein aktives SprayPath – starte leer")

        self.get_logger().info("✅ SprayPathManager aktiv")

    # -------------------------- EVENTS --------------------------
    def on_set(self, msg: String):
        recipe_file = msg.data.strip()
        full_path = os.path.join(self.paths_dir, recipe_file)
        self._load_and_publish(full_path)
        self._save_current_recipe(recipe_file)

    def on_clear(self, _msg: String):
        m = Marker()
        m.action = Marker.DELETE
        self.marker_pub.publish(m)
        self._publish_info("🗑️ SprayPath cleared")

    # -------------------------- LOGIK --------------------------
    def _load_and_publish(self, file_path: str):
        spray = self._load_recipe(file_path)
        marker = self._make_marker(spray)
        self.marker_pub.publish(marker)
        self._publish_info(f"✅ SprayPath geladen: {spray['name']} ({len(spray['points'])} Punkte)")

    def _save_current_recipe(self, name: str):
        cfg = self._load_yaml_file(self.config_yaml)
        cfg.setdefault("spray_paths", {})["current"] = name
        with open(self.config_yaml, "w") as f:
            yaml.dump(cfg, f)
        self._publish_info(f"💾 Aktives Rezept gespeichert: {name}")

    # -------------------------- HILFSFUNKTIONEN --------------------------
    def _load_recipe(self, file_path):
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"SprayPath YAML nicht gefunden: {file_path}")
        data = self._load_yaml_file(file_path)
        if "spray_path" not in data:
            raise ValueError("YAML muss 'spray_path:' enthalten")
        sp = data["spray_path"]
        return sp

    @staticmethod
    def _load_yaml_file(path: str):
        if not os.path.exists(path):
            return {}
        with open(path, "r") as f:
            return yaml.safe_load(f) or {}

    def _make_marker(self, spray: dict) -> Marker:
        m = Marker()
        m.header.frame_id = spray.get("frame", "world")
        m.ns = "spray_path"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        # Lila Linie
        m.scale.x = 0.004
        m.color.r = 0.6
        m.color.g = 0.2
        m.color.b = 0.8
        m.color.a = 1.0

        for p in spray["points"]:
            pt = Point()
            pt.x, pt.y, pt.z = p["x"], p["y"], p["z"]
            m.points.append(pt)

        m.lifetime = Duration(sec=0, nanosec=0)
        return m

    def _publish_info(self, text: str):
        self.info_pub.publish(String(data=text))
        self.get_logger().info(text)


def main(args=None):
        rclpy.init(args=args)
        node = SprayPathManager()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
