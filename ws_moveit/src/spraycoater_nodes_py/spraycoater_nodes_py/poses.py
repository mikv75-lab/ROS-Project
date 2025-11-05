#!/usr/bin/env python3
from __future__ import annotations
import os, yaml, math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener

from mecademic_bringup.utils import rpy_deg_to_quat
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.qos import qos_latched, qos_default
from mecademic_bringup.common.topics import Topics
from mecademic_bringup.common.params import PARAM_POSES_CONFIG
from tf_transformations import euler_from_quaternion

class PosesManager(Node):
    """
    PosesManager
    ------------
    Lädt und verwaltet definierte Posen (z. B. home, service, workspace_center)
    aus einer YAML-Datei und publiziert sie als:
      - PoseStamped-Topics (/meca/poses/<name>)
      - statische TF-Frames (world → <pose>)
    """

    def __init__(self):
        super().__init__("poses_manager")

        # --- Parameter ---
        self.declare_parameter(PARAM_POSES_CONFIG, "")
        self.yaml_path = self.get_parameter(PARAM_POSES_CONFIG).value

        if not self.yaml_path or not os.path.exists(self.yaml_path):
            self.get_logger().error(f"❌ Poses YAML nicht gefunden: {self.yaml_path}")
            raise FileNotFoundError(f"Pose YAML fehlt: {self.yaml_path}")

        self.poses = self._load_yaml()
        self.frames = FRAMES
        self.topics = Topics()

        # --- TF + Publisher Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)

        # Posen einzeln als latched Topics
        self.pose_publishers = {
            name: self.create_publisher(PoseStamped, self.topics.pose(name), qos_latched())
            for name in self.poses
        }

        # --- Subscriber für „Pose aus TCP übernehmen“ ---
        self.create_subscription(
            String,
            self.topics.poses_set_from_tcp,
            self.set_from_tcp,
            qos_default()
        )

        self.publish_all()
        self.get_logger().info(f"✅ PosesManager aktiv – {len(self.poses)} Posen geladen aus {self.yaml_path}")

    # ------------------------------------------------------------------

    def _load_yaml(self) -> dict:
        with open(self.yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}
        return data

    # ------------------------------------------------------------------

    def publish_all(self):
        """Publiziert alle geladenen Posen als PoseStamped + TF."""
        tfs = []
        for name, pose in self.poses.items():
            msg, tf = self._make_pose(name, pose)
            self.pose_publishers[name].publish(msg)
            tfs.append(tf)
        if tfs:
            self.static_tf.sendTransform(tfs)
        self.get_logger().info(f"📡 {len(tfs)} Posen veröffentlicht")

    # ------------------------------------------------------------------

    def _make_pose(self, name: str, pose: dict):
        parent_frame = pose.get("frame", self.frames["tool_mount"])  # Standard: tool_mount

        x, y, z = pose.get("xyz", [0, 0, 0])
        r_deg, p_deg, y_deg = pose.get("rpy_deg", [0, 0, 0])
        r, p, y_ = map(math.radians, (r_deg, p_deg, y_deg))
        qx, qy, qz, qw = rpy_deg_to_quat(r, p, y_)

        # PoseStamped
        msg = PoseStamped()
        msg.header.frame_id = parent_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # Static TF
        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id = name
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        return msg, tf

    # ------------------------------------------------------------------

    def set_from_tcp(self, msg: String):
        """Aktualisiert eine Pose basierend auf einem TCP-Frame."""
        try:
            # Eingabe flexibel parsen (z. B. "service", "service scene", "service:scene")
            parts = msg.data.replace(":", " ").split()
            pose_name = parts[0]
            frame = parts[1] if len(parts) > 1 else self.frames.get("scene", "scene")

            if pose_name not in self.poses:
                self.get_logger().warning(f"⚠️ Unbekannte Pose '{pose_name}' — wird neu angelegt.")
                self.poses[pose_name] = {}

            # Transform vom Frame zur Base
            tf = self.tf_buffer.lookup_transform(
                self.frames["meca_mount"],
                frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )

            # Quaternion → Euler (in Grad)
            q = tf.transform.rotation
            roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            rpy_deg = [round(math.degrees(a), 3) for a in (roll, pitch, yaw)]

            # Pose speichern (Euler statt Quaternion)
            self.poses[pose_name] = {
                "frame": frame,
                "xyz": [
                    round(tf.transform.translation.x, 4),
                    round(tf.transform.translation.y, 4),
                    round(tf.transform.translation.z, 4),
                ],
                "rpy_deg": rpy_deg,
            }

            # Speichern + publizieren
            with open(self.yaml_path, "w") as f:
                yaml.dump(self.poses, f, sort_keys=False)

            self.publish_all()
            self.get_logger().info(
                f"💾 Pose '{pose_name}' aus Frame '{frame}' gespeichert "
                f"→ XYZ={self.poses[pose_name]['xyz']} RPY={rpy_deg}"
            )

        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei set_from_tcp: {e}")
# ------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PosesManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
