#!/usr/bin/env python3
# mecademic_bringup/poses_manager.py
from __future__ import annotations

import os
import yaml
import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener

# ✅ Korrekt als Paket importieren
from .utils import rpy_deg_to_quat

# ✅ Zentrale Imports
from mecademic_bringup.common.frames import (
    FRAME_MECA_MOUNT,
    FRAME_POSE_HOME,
    FRAME_POSE_PREDISPENSE,
    FRAME_POSE_SERVICE,
)
from mecademic_bringup.common.topics import (
    TOPIC_POSES_PREFIX,
    TOPIC_POSE_SET_FROM_TCP,
)
from mecademic_bringup.common.params import PARAM_POSES_CONFIG


# Pose-Namen → Frame Mapping
POSE_FRAME_MAP = {
    "home": FRAME_POSE_HOME,
    "predispense": FRAME_POSE_PREDISPENSE,
    "service": FRAME_POSE_SERVICE,
}

ALLOWED_POSES = list(POSE_FRAME_MAP.keys())


class PosesManager(Node):
    def __init__(self):
        super().__init__("poses_manager")

        # ✅ YAML laden
        self.declare_parameter(PARAM_POSES_CONFIG, "")
        self.yaml_path = self.get_parameter(PARAM_POSES_CONFIG).value

        if not self.yaml_path or not os.path.exists(self.yaml_path):
            self.get_logger().error(f"❌ Pose YAML nicht gefunden: {self.yaml_path}")
            raise FileNotFoundError(f"Pose YAML fehlt: {self.yaml_path}")

        self.poses = self._load_yaml()

        # ✅ TF Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)

        # ✅ Publisher für jede Pose
        self.pose_publishers = {
            name: self.create_publisher(PoseStamped, f"{TOPIC_POSES_PREFIX}/{name}", 10)
            for name in self.poses
        }

        # ✅ Nur noch Pose aus TCP speichern
        self.create_subscription(String, TOPIC_POSE_SET_FROM_TCP, self.set_from_tcp, 10)

        # ✅ Pose initial publizieren
        self.publish_all()
        self.get_logger().info(f"✅ PosesManager aktiv – geladen aus {self.yaml_path}")


    def _load_yaml(self):
        with open(self.yaml_path, "r") as f:
            raw = yaml.safe_load(f) or {}
        return {k: v for k, v in raw.items() if k in ALLOWED_POSES}


    def _save_yaml(self):
        """💾 Speichert aktuelle Posen persistent zurück in YAML."""
        with open(self.yaml_path, "w") as f:
            yaml.dump(self.poses, f)
        self.get_logger().info(f"💾 Posen gespeichert → {self.yaml_path}")


    def publish_all(self):
        transforms = []
        for name, pose in self.poses.items():
            msg, tf = self.make_pose_messages(name, pose)
            self.pose_publishers[name].publish(msg)
            transforms.append(tf)
        self.static_tf.sendTransform(transforms)


    def make_pose_messages(self, name, pose):
        assert name in ALLOWED_POSES, f"❌ Ungültige Pose '{name}'"
        x, y, z = pose["xyz"]
        rr, pp, yy = map(math.radians, pose["rpy_deg"])
        qx, qy, qz, qw = rpy_deg_to_quat(rr, pp, yy)

        msg = PoseStamped()
        msg.header.frame_id = FRAME_MECA_MOUNT
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qx, qy, qz, qw

        tf = TransformStamped()
        tf.header.frame_id = FRAME_MECA_MOUNT
        tf.child_frame_id = POSE_FRAME_MAP[name]
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        return msg, tf


    def set_from_tcp(self, msg):
        try:
            pose_name, frame = msg.data.split()
            if pose_name not in self.poses:
                self.get_logger().warning(f"⚠️ Unbekannte Pose '{pose_name}'")
                return

            tf = self.tf_buffer.lookup_transform(
                FRAME_MECA_MOUNT, frame, rclpy.time.Time(), timeout=Duration(seconds=1)
            )

            self.poses[pose_name] = {
                "xyz": [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z,
                ],
                "rpy_deg": [0.0, 0.0, 0.0],
            }

            self.publish_all()
            self._save_yaml()
            self.get_logger().info(f"✅ Pose '{pose_name}' aus TCP gespeichert (Frame '{frame}')")
        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei set_from_tcp: {e}")


def main():
    rclpy.init()
    node = PosesManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
