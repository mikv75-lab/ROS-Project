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
from std_msgs.msg import String, Empty
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener


# --- Konfiguration ---
ALLOWED_POSES = ["home", "predispense", "service"]  # workspace_center hier NICHT erlaubt
PARENT_FRAME = "meca_mount"


# --- Mathe Helfer ---
def rpy_to_quat(r, p, y):
    cr, sr = math.cos(r/2), math.sin(r/2)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    return (
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    )


# --- Node ---
class PosesManager(Node):
    def __init__(self):
        super().__init__("poses_manager")

        # Lade YAML
        self.declare_parameter("positions_yaml", "")
        self.yaml_path = self.get_parameter("positions_yaml").get_parameter_value().string_value
        self.poses = self._load_yaml(self.yaml_path)

        # Infrastruktur
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)

        # Publisher
        self.publishers = {}
        for name in self.poses:
            topic = f"/meca/poses/{name}"
            self.publishers[name] = self.create_publisher(PoseStamped, topic, 1)

        # Subscriber
        for name in self.poses:
            self.create_subscription(PoseStamped, f"/meca/poses/set/{name}", self.make_set_cb(name), 10)
        self.create_subscription(String, "/meca/poses/set_from_tcp", self.set_from_tcp, 10)

        # Start publish
        self.publish_all()
        self.get_logger().info(f"[poses_manager] ✅ Ready. Posen: {list(self.poses.keys())}")

    # YAML lesen
    def _load_yaml(self, path):
        if not os.path.isfile(path):
            self.get_logger().warning(f"[poses_manager] ⚠️ Datei fehlt: {path}")
            return {}
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}

        poses = {k: v for k, v in data.items() if k in ALLOWED_POSES}
        self.get_logger().info(f"[poses_manager] ✅ Posen geladen: {list(poses.keys())}")
        return poses

    # Alle senden
    def publish_all(self):
        tfs = []
        for name, pose in self.poses.items():
            msg, tf = self.make_pose_messages(name, pose)
            self.publishers[name].publish(msg)
            tfs.append(tf)
        self.static_tf.sendTransform(tfs)

    # Pose Messages bauen
    def make_pose_messages(self, name, pose):
        x,y,z = pose["xyz"]
        rr,pp,yy = [math.radians(v) for v in pose["rpy_deg"]]
        qx,qy,qz,qw = rpy_to_quat(rr,pp,yy)

        msg = PoseStamped()
        msg.header.frame_id = PARENT_FRAME
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        tf = TransformStamped()
        tf.header.frame_id = PARENT_FRAME
        tf.child_frame_id = f"meca_pose_{name}"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.translation.z = z
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        return msg, tf

    # Set callbacks
    def make_set_cb(self, name):
        def cb(msg):
            self.poses[name] = {
                "xyz": [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                "rpy_deg": [0,0,0],  # roll,pitch,yaw wird ignoriert
            }
            self.publish_all()
        return cb

    # Set via TCP
    def set_from_tcp(self, msg):
        parts = msg.data.split()
        if len(parts) != 2:
            self.get_logger().warn(f"[poses_manager] Falsches Format: {msg.data}")
            return
        name, frame = parts
        if name not in self.poses:
            self.get_logger().warn(f"[poses_manager] Unbekannte Pose: {name}")
            return

        try:
            tf = self.tf_buffer.lookup_transform(PARENT_FRAME, frame, rclpy.time.Time(), Duration(seconds=1))
            self.poses[name] = {
                "xyz": [
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z
                ],
                "rpy_deg": [0,0,0],
            }
            self.publish_all()
            self.get_logger().info(f"[poses_manager] ✅ Pose '{name}' aus TCP gesetzt.")
        except:
            self.get_logger().error(f"[poses_manager] ❌ kein Transform von {PARENT_FRAME} <- {frame}")



def main():
    rclpy.init()
    node = PosesManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
