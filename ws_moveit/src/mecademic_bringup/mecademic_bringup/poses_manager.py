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
from mecademic_bringup.common.frames import FRAME_MECA_MOUNT
from mecademic_bringup.common.params import PARAM_POSES_CONFIG
from mecademic_bringup.common.topics import TOPIC_POSES_PREFIX, TOPIC_POSE_SET_FROM_TCP

class PosesManager(Node):
    def __init__(self):
        super().__init__("poses_manager")

        self.declare_parameter(PARAM_POSES_CONFIG, "")
        self.yaml_path = self.get_parameter(PARAM_POSES_CONFIG).value

        if not self.yaml_path or not os.path.exists(self.yaml_path):
            self.get_logger().error(f"❌ Poses YAML nicht gefunden: {self.yaml_path}")
            raise FileNotFoundError(f"Pose YAML fehlt: {self.yaml_path}")

        self.poses = self._load_yaml()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)

        # Publiziere Posen sauber unter /poses/<name>
        self.pose_publishers = {
            name: self.create_publisher(PoseStamped, f"{TOPIC_POSES_PREFIX}/{name}", 10)
            for name in self.poses
        }

        self.create_subscription(String, TOPIC_POSE_SET_FROM_TCP, self.set_from_tcp, 10)

        self.publish_all()
        self.get_logger().info(f"✅ PosesManager aktiv – Frames geladen aus {self.yaml_path}")

    def _load_yaml(self):
        with open(self.yaml_path, "r") as f:
            raw = yaml.safe_load(f) or {}
        return raw  # ALLES aus YAML übernehmen, keine Prefixe, kein Filter

    def publish_all(self):
        tfs = []
        for name, pose in self.poses.items():
            msg, tf = self.make_pose(name, pose)
            self.pose_publishers[name].publish(msg)
            tfs.append(tf)
        self.static_tf.sendTransform(tfs)

    def make_pose(self, name, pose):
        parent_frame = pose.get("frame", FRAME_MECA_MOUNT)  # 👈 Frame flexibel

        x, y, z = pose["xyz"]
        r, p, y_deg = map(math.radians, pose["rpy_deg"])
        qx, qy, qz, qw = rpy_deg_to_quat(r, p, y_deg)

        # Pose Publisher (nur Info)
        msg = PoseStamped()
        msg.header.frame_id = parent_frame
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = x, y, z
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = qx, qy, qz, qw

        # Static TF senden
        tf = TransformStamped()
        tf.header.frame_id = parent_frame           # ✅ Frame dynamisch
        tf.child_frame_id = name                    # ✅ Keine Prefixe mehr!
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
                "xyz": [tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z],
                "rpy_deg": [0.0, 0.0, 0.0],  # noch kein RPY solve
            }

            self.publish_all()
            with open(self.yaml_path, "w") as f:
                yaml.dump(self.poses, f)

            self.get_logger().info(f"✅ Pose '{pose_name}' aktualisiert aus Frame '{frame}'")
        except Exception as e:
            self.get_logger().error(f"❌ Fehler bei set_from_tcp: {e}")

def main():
    rclpy.init()
    node = PosesManager()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
