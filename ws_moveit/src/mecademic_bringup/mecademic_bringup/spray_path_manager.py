#!/usr/bin/env python3
from typing import List
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Point, PoseArray
from std_msgs.msg import Empty
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
import rclpy
import yaml
import os
from tf2_ros import Buffer, TransformListener, LookupException

from mecademic_bringup.common.topics import TOPIC_SPRAY_PATH_MARKERS
from mecademic_bringup.common.frames import FRAME_SCENE
from mecademic_bringup.common.params import PARAM_SPRAY_PATH_CONFIG


class SprayPathManager(Node):
    def __init__(self):
        super().__init__("spray_path_manager")

        # --- Frame ---
        self._frame = FRAME_SCENE

        # --- QoS (transient_local = latched) ---
        qos_latched = QoSProfile(depth=1)
        qos_latched.history = QoSHistoryPolicy.KEEP_LAST
        qos_latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # --- Publisher & Subscriber ---
        self._pub_markers = self.create_publisher(MarkerArray, TOPIC_SPRAY_PATH_MARKERS, qos_latched)
        self._pub_poses = self.create_publisher(PoseArray, "/spray_path/poses", qos_latched)
        self._sub_reload = self.create_subscription(Empty, "/spray_path/reload", self._on_reload, 10)
        self._sub_clear = self.create_subscription(Empty, "/spray_path/clear", self._on_clear, 10)

        # --- State ---
        self._axes_last = 0
        self.spray_paths_yaml = self.declare_parameter(
            PARAM_SPRAY_PATH_CONFIG, PARAM_SPRAY_PATH_CONFIG
        ).get_parameter_value().string_value
        self._spray_paths_dir = None
        self._spray_path_file = None
        self._last_mtime = 0.0

        # --- TF vor Timern initialisieren ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Timer für Auto-Reload ---
        self._timer = self.create_timer(2.0, self._check_yaml_update)

        # --- Initialer Ladevorgang ---
        self.load_spray_path_config()

        # 👉 Verzögerter Publish nach TF-Verfügbarkeit
        self._delayed_timer = self.create_timer(2.0, self._delayed_publish_once)

        self.get_logger().info("✅ SprayPathManager gestartet (Topics: /spray_path/reload, /spray_path/clear)")

    # -------------------------------------------------
    # Verzögerter erster Publish (wartet auf Frame)
    # -------------------------------------------------
    def _delayed_publish_once(self):
        try:
            self.tf_buffer.lookup_transform("world", self._frame, rclpy.time.Time())
            self.publish_current_path()
            self.get_logger().info(f"✅ Frame '{self._frame}' gefunden – Spray-Path publiziert")
            self._delayed_timer.cancel()  # nur einmal ausführen
        except LookupException:
            self.get_logger().warning(f"⏳ Frame '{self._frame}' noch nicht da, warte weiter...")

    # -------------------------------------------------
    # spray_paths.yaml lesen
    # -------------------------------------------------
    def load_spray_path_config(self):
        """Liest spray_paths.yaml und bestimmt aktuelles Rezept."""
        if not os.path.exists(self.spray_paths_yaml):
            self.get_logger().error(f"spray_paths.yaml nicht gefunden: {self.spray_paths_yaml}")
            return

        with open(self.spray_paths_yaml, "r") as f:
            data = yaml.safe_load(f)

        spray_cfg = data.get("spray_paths", {})
        base_dir = os.path.dirname(self.spray_paths_yaml)
        self._spray_paths_dir = os.path.join(base_dir, spray_cfg.get("dir", "spray_paths"))
        self._spray_path_file = spray_cfg.get("current", None)
        self._last_mtime = os.path.getmtime(self.spray_paths_yaml)

        self.get_logger().info(f"Spray-Path-Ordner: {self._spray_paths_dir}")
        self.get_logger().info(f"Aktiver Spray-Path: {self._spray_path_file}")

    # -------------------------------------------------
    # Auto-Reload bei Dateiänderung
    # -------------------------------------------------
    def _check_yaml_update(self):
        if not os.path.exists(self.spray_paths_yaml):
            return
        mtime = os.path.getmtime(self.spray_paths_yaml)
        if mtime > self._last_mtime:
            self._last_mtime = mtime
            self.get_logger().info("🌀 spray_paths.yaml geändert → neu laden...")
            self.load_spray_path_config()
            self.publish_current_path()

    # -------------------------------------------------
    # Topic-Callbacks
    # -------------------------------------------------
    def _on_reload(self, msg: Empty):
        self.get_logger().info("🔄 Reload per Topic angefordert → neu laden...")
        self.load_spray_path_config()
        self.publish_current_path()

    def _on_clear(self, msg: Empty):
        self.get_logger().info("🧹 Clear-Command empfangen → Marker löschen...")
        self.clear()

    # -------------------------------------------------
    # Spray-Path laden und publizieren
    # -------------------------------------------------
    def publish_current_path(self):
        if not self._spray_path_file:
            self.get_logger().warning("Kein aktueller Spray-Path definiert")
            self.clear()
            return

        path_file = os.path.join(self._spray_paths_dir, self._spray_path_file)
        if not os.path.exists(path_file):
            self.get_logger().error(f"Spray-Path-Datei nicht gefunden: {path_file}")
            self.clear()
            return

        with open(path_file, "r") as f:
            data = yaml.safe_load(f)

        poses = []
        for p in data["spray_path"]["points"]:
            pose = Pose()
            pose.position.x = p["x"]
            pose.position.y = p["y"]
            pose.position.z = p["z"]
            pose.orientation.x = p["qx"]
            pose.orientation.y = p["qy"]
            pose.orientation.z = p["qz"]
            pose.orientation.w = p["qw"]
            poses.append(pose)

        # --- Marker + PoseArray publizieren ---
        self.get_logger().info(f"Publiziere Spray-Path '{self._spray_path_file}' mit {len(poses)} Punkten")
        self.draw(poses)

        pose_array = PoseArray()
        pose_array.header.frame_id = self._frame
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = poses
        self._pub_poses.publish(pose_array)
        self.get_logger().info("PoseArray publiziert ✅")

    # -------------------------------------------------
    # Marker zeichnen / löschen
    # -------------------------------------------------
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
        self.get_logger().info(f"🧽 Marker unter Frame '{self._frame}' gelöscht")

    def draw(self, poses: List[Pose], *, line_w=0.003):
        """Zeichnet nur die Linie des Spray-Paths."""
        self.clear()
        if not poses:
            return

        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # Linie
        line = Marker()
        line.header.frame_id = self._frame
        line.header.stamp = now
        line.ns = "spray_path"
        line.id = 3001
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = line_w
        line.color.r = 0.0
        line.color.g = 1.0
        line.color.b = 0.0
        line.color.a = 1.0
        line.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in poses]
        ma.markers.append(line)

        self._pub_markers.publish(ma)
        self.get_logger().info("MarkerArray (Linie) publiziert ✅")


def quat_to_dir(qx, qy, qz, qw):
    """Wandelt Quaternion in Vorwärtsrichtung (lokale Z-Achse) um."""
    # Quaternion -> Rotationsmatrix, nur Z-Achsenrichtung
    # Formel aus Wikipedia / ROS tf2
    x = 2 * (qx * qz + qw * qy)
    y = 2 * (qy * qz - qw * qx)
    z = 1 - 2 * (qx * qx + qy * qy)
    return (x, y, z)

def main(args=None):
    rclpy.init(args=args)
    node = SprayPathManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
