#!/usr/bin/env python3
from __future__ import annotations
import os, yaml, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker
from std_msgs.msg import Empty, String
from mecademic_bringup.common.params import PARAM_SPRAY_PATH_CONFIG
from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.frames import FRAMES


class SprayPathManager(Node):
    """
    SprayPathManager
    ----------------
    Lädt, publiziert und verwaltet Spray-Pfade (PoseArray + Marker).
    Unterstützt:
      • load/reload (/meca/spray_path/set)
      • clear       (/meca/spray_path/clear)
      • poses       (/meca/spray_path/poses)
      • current_name (String, latched)
      • markers     (/meca/spray_path/markers)
    """

    def __init__(self):
        super().__init__("spray_path_manager")

        # --- Common ---
        self.topics = Topics()
        self.frames = FRAMES
        self._frame = self.frames["scene"]

        # --- Parameter ---
        self.declare_parameter(PARAM_SPRAY_PATH_CONFIG, "")
        self.spray_paths_yaml = self.get_parameter(PARAM_SPRAY_PATH_CONFIG).get_parameter_value().string_value
        if not self.spray_paths_yaml or not os.path.exists(self.spray_paths_yaml):
            raise FileNotFoundError(f"❌ {PARAM_SPRAY_PATH_CONFIG} Datei fehlt: {self.spray_paths_yaml}")

        # --- Publisher ---
        self._pub_poses = self.create_publisher(PoseArray, self.topics.spray_path_poses, qos_default())
        self._pub_name = self.create_publisher(String, f"{self.topics.spray_path_current}_name", qos_latched())
        self._pub_marker = self.create_publisher(Marker, self.topics.spray_path_markers, qos_default())

        # --- Subscriber ---
        self._sub_reload = self.create_subscription(Empty, self.topics.spray_path_set, self._on_reload, qos_default())
        self._sub_clear = self.create_subscription(Empty, self.topics.spray_path_clear, self._on_clear, qos_default())

        # --- Internals ---
        self._spray_paths_dir = ""
        self._spray_path_file = ""
        self._last_data = None
        self._posearray_msg = None
        self._marker_msg = None

        # --- Initial Load ---
        self.load_config()

        # Verzögerter Start (damit TFs und RViz bereit sind)
        self.create_timer(3.0, self._initial_publish_once)

        # Re-Publisher alle 1 s, damit Marker stabil bleiben
        self._timer_repub = self.create_timer(1.0, self._republish_marker)

        self.get_logger().info(f"✅ SprayPathManager gestartet (Config: {self.spray_paths_yaml})")

    # ---------------------------------------------------------------
    def _initial_publish_once(self):
        """Erst-Publish verzögert starten, um TF/RViz zu warten."""
        if not self._posearray_msg and not self._marker_msg:
            self.publish_current_path()

    # ---------------------------------------------------------------
    def load_config(self):
        """Lädt aktuelle Konfiguration (spray_paths.yaml)."""
        with open(self.spray_paths_yaml, "r") as f:
            data = yaml.safe_load(f) or {}

        spray_cfg = data.get("spray_paths", {})
        base_dir = os.path.dirname(self.spray_paths_yaml)
        self._spray_paths_dir = os.path.join(base_dir, spray_cfg.get("dir", "spray_paths"))
        self._spray_path_file = spray_cfg.get("current", "")

        self.get_logger().info(f"📂 SprayPath-Ordner: {self._spray_paths_dir}")
        self.get_logger().info(f"🎯 Aktiver Path: {self._spray_path_file}")
        self.publish_current_name()

    # ---------------------------------------------------------------
    def publish_current_path(self):
        """Publiziert den aktuellen SprayPath als PoseArray + Marker."""
        if not self._spray_path_file:
            self.get_logger().warning("⚠️ Kein aktiver Spray-Path definiert")
            self.publish_current_name()
            return

        path_file = os.path.join(self._spray_paths_dir, self._spray_path_file)
        if not os.path.exists(path_file):
            self.get_logger().error(f"❌ Spray-Path-Datei fehlt: {path_file}")
            self.publish_current_name()
            return

        with open(path_file, "r") as f:
            data = yaml.safe_load(f) or {}

        spray_data = data.get("spray_path", {})
        frame = spray_data.get("frame", self._frame)
        self._frame = frame

        poses = []
        for p in spray_data.get("points", []):
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = p["x"], p["y"], p["z"]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
                p["qx"], p["qy"], p["qz"], p["qw"]
            )
            poses.append(pose)

        if not poses:
            self.get_logger().warning("⚠️ Keine Punkte im SprayPath gefunden")
            self.publish_current_name()
            return

        # PoseArray
        pa = PoseArray()
        pa.header.frame_id = frame
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = poses
        self._pub_poses.publish(pa)
        self._posearray_msg = pa

        # Marker (grüne Linie)
        marker = Marker()
        marker.header.frame_id = frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "spray_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.002
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
        marker.pose.orientation.w = 1.0
        marker.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in poses]
        if len(marker.points) > 2:
            marker.points.append(marker.points[0])
        self._pub_marker.publish(marker)
        self._marker_msg = marker

        self.publish_current_name()
        self._last_data = data
        self.get_logger().info(f"✅ Publiziert SprayPath '{self._spray_path_file}' mit {len(poses)} Punkten (Frame: {frame})")

    # ---------------------------------------------------------------
    def _republish_marker(self):
        """Erneutes Publizieren, damit RViz Marker dauerhaft sieht."""
        if self._marker_msg:
            self._pub_marker.publish(self._marker_msg)
        if self._posearray_msg:
            self._pub_poses.publish(self._posearray_msg)

    # ---------------------------------------------------------------
    def publish_current_name(self):
        msg = String()
        msg.data = self._spray_path_file or ""
        self._pub_name.publish(msg)

    # ---------------------------------------------------------------
    def clear(self, _msg: Empty = None):
        pa = PoseArray()
        pa.header.frame_id = self._frame
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = []
        self._pub_poses.publish(pa)

        marker = Marker()
        marker.header.frame_id = self._frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        self._pub_marker.publish(marker)

        self._spray_path_file = ""
        self._posearray_msg = None
        self._marker_msg = None
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
