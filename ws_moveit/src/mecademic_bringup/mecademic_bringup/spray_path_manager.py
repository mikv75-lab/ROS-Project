#!/usr/bin/env python3
from __future__ import annotations
import os, yaml
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import Empty
from geometry_msgs.msg import Pose, Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException

from mecademic_bringup.common.qos import qos_latched, qos_default
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.params import PARAM_SPRAY_PATH_CONFIG


class SprayPathManager(Node):
    """
    SprayPathManager
    ----------------
    Lädt Spray-Path-YAML-Dateien, überwacht Änderungen,
    und publiziert PoseArray + RViz-Marker für Visualisierung.
    """

    def __init__(self):
        super().__init__("spray_path_manager")

        # --- Common ---
        self.frames = FRAMES
        self.topics = Topics()

        # --- Default Frame ---
        self._frame = self.frames["workspace_center"]

        # --- QoS ---
        qos_latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # --- Publisher & Subscriber ---
        self._pub_markers = self.create_publisher(MarkerArray, self.topics.recipe_markers, qos_latched)
        self._pub_poses = self.create_publisher(PoseArray, self.topics.spray_path_current, qos_latched)
        self._sub_reload = self.create_subscription(Empty, self.topics.spray_path_set, self._on_reload, qos_default())
        self._sub_clear = self.create_subscription(Empty, self.topics.spray_path_clear, self._on_clear, qos_default())

        # --- Parameter ---
        self.declare_parameter(PARAM_SPRAY_PATH_CONFIG, "")
        self.spray_paths_yaml = self.get_parameter(PARAM_SPRAY_PATH_CONFIG).get_parameter_value().string_value
        if not self.spray_paths_yaml:
            self.get_logger().error(f"❌ {PARAM_SPRAY_PATH_CONFIG} Parameter fehlt!")
            raise FileNotFoundError("Kein Spray-Path-YAML angegeben")

        self._spray_paths_dir = None
        self._spray_path_file = None
        self._last_mtime = 0.0
        self._axes_last = 0

        # --- TF Buffer ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Timer ---
        self._timer_reload = self.create_timer(2.0, self._check_yaml_update)
        self._timer_delayed = self.create_timer(2.0, self._delayed_publish_once)

        # --- Initial Load ---
        self.load_spray_path_config()

        self.get_logger().info(f"✅ SprayPathManager gestartet (Config: {self.spray_paths_yaml})")

    # ------------------------------------------------------------------
    # Initialer Versuch – Frame prüfen, dann publish
    # ------------------------------------------------------------------
    def _delayed_publish_once(self):
        try:
            self.tf_buffer.lookup_transform(self.frames["world"], self._frame, rclpy.time.Time())
            self.publish_current_path()
            self.get_logger().info(f"✅ Frame '{self._frame}' verfügbar – Spray-Path publiziert")
            self._timer_delayed.cancel()
        except LookupException:
            self.get_logger().debug(f"⏳ Frame '{self._frame}' noch nicht verfügbar...")

    # ------------------------------------------------------------------
    # YAML laden
    # ------------------------------------------------------------------
    def load_spray_path_config(self):
        if not os.path.exists(self.spray_paths_yaml):
            self.get_logger().error(f"❌ spray_paths.yaml nicht gefunden: {self.spray_paths_yaml}")
            return

        with open(self.spray_paths_yaml, "r") as f:
            data = yaml.safe_load(f) or {}

        spray_cfg = data.get("spray_paths", {})
        base_dir = os.path.dirname(self.spray_paths_yaml)
        self._spray_paths_dir = os.path.join(base_dir, spray_cfg.get("dir", "spray_paths"))
        self._spray_path_file = spray_cfg.get("current")
        self._last_mtime = os.path.getmtime(self.spray_paths_yaml)

        self.get_logger().info(f"📂 Spray-Path-Ordner: {self._spray_paths_dir}")
        self.get_logger().info(f"🎯 Aktiver Spray-Path: {self._spray_path_file}")

    # ------------------------------------------------------------------
    # Auto-Reload bei Änderungen
    # ------------------------------------------------------------------
    def _check_yaml_update(self):
        if not os.path.exists(self.spray_paths_yaml):
            return
        mtime = os.path.getmtime(self.spray_paths_yaml)
        if mtime > self._last_mtime:
            self._last_mtime = mtime
            self.get_logger().info("🌀 spray_paths.yaml geändert → neu laden...")
            self.load_spray_path_config()
            self.publish_current_path()

    # ------------------------------------------------------------------
    # Topic-Callbacks
    # ------------------------------------------------------------------
    def _on_reload(self, _msg: Empty):
        self.get_logger().info("🔄 Reload angefordert")
        self.load_spray_path_config()
        self.publish_current_path()

    def _on_clear(self, _msg: Empty):
        self.get_logger().info("🧹 Clear angefordert")
        self.clear()

    # ------------------------------------------------------------------
    # Pfad publishen
    # ------------------------------------------------------------------
    def publish_current_path(self):
        if not self._spray_path_file:
            self.get_logger().warning("⚠️ Kein aktueller Spray-Path definiert")
            self.clear()
            return

        path_file = os.path.join(self._spray_paths_dir, self._spray_path_file)
        if not os.path.exists(path_file):
            self.get_logger().error(f"❌ Spray-Path-Datei nicht gefunden: {path_file}")
            self.clear()
            return

        with open(path_file, "r") as f:
            data = yaml.safe_load(f)

        poses = []
        for p in data.get("spray_path", {}).get("points", []):
            pose = Pose()
            pose.position.x = p["x"]
            pose.position.y = p["y"]
            pose.position.z = p["z"]
            pose.orientation.x = p["qx"]
            pose.orientation.y = p["qy"]
            pose.orientation.z = p["qz"]
            pose.orientation.w = p["qw"]
            poses.append(pose)

        if not poses:
            self.get_logger().warning("⚠️ Keine Punkte im Spray-Path gefunden")
            self.clear()
            return

        self.get_logger().info(f"🎯 Publiziere Spray-Path '{self._spray_path_file}' mit {len(poses)} Punkten")
        self.draw(poses)

        pose_array = PoseArray()
        pose_array.header.frame_id = self._frame
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = poses
        self._pub_poses.publish(pose_array)
        self.get_logger().info("✅ PoseArray publiziert")

    # ------------------------------------------------------------------
    # Marker
    # ------------------------------------------------------------------
    def clear(self):
        ma = MarkerArray()
        for mid in (3001, 3002):
            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "spray_path"
            m.id = mid
            m.action = Marker.DELETE
            ma.markers.append(m)

        for i in range(self._axes_last):
            arr = Marker()
            arr.header.frame_id = self._frame
            arr.header.stamp = self.get_clock().now().to_msg()
            arr.ns = "spray_axes"
            arr.id = 400000 + i
            arr.action = Marker.DELETE
            ma.markers.append(arr)

        self._pub_markers.publish(ma)
        self._axes_last = 0
        self.get_logger().debug(f"🧽 Marker unter Frame '{self._frame}' gelöscht")

    def draw(self, poses: List[Pose], *, line_w=0.003):
        """Zeichnet den Spray-Path als grüne Linie."""
        self.clear()
        if not poses:
            return

        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        line = Marker()
        line.header.frame_id = self._frame
        line.header.stamp = now
        line.ns = "spray_path"
        line.id = 3001
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = line_w
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 1.0, 0.0, 1.0
        line.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in poses]

        ma.markers.append(line)
        self._pub_markers.publish(ma)
        self.get_logger().info("✅ MarkerArray (Linie) publiziert")


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
