#!/usr/bin/env python3
from __future__ import annotations
import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String, Empty
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener, TransformException

MOUNT_PARENT = "world"            # parent für TF-Lookup
MOUNT_FRAME  = "substrate_mount"  # Mount-Frame (existiert als TF in der Szene)

class MountManager(Node):
    """
    Verwalten des Substratemount-Meshs (RViz Marker) relativ zu 'substrate_mount'.
    Erwartet, dass TF world->substrate_mount existiert (Pose extern gesetzt).
    """
    def __init__(self):
        super().__init__("mount_manager")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)

        self.create_subscription(String, "meca/mount/load", self.on_load, 10)
        self.create_subscription(Empty,  "meca/mount/remove", self.on_remove, 10)

        self._current_mount = ""

    def on_load(self, msg: String):
        path = msg.data.strip()
        if not os.path.isfile(path):
            self.get_logger().error(f"❌ Mount STL existiert nicht: {path}")
            return
        pose = self._lookup_mount_pose()
        if pose is None:
            self.get_logger().warning(f"⚠️ Kein TF {MOUNT_PARENT}->{MOUNT_FRAME} gefunden. Abbruch.")
            return

        self._current_mount = path
        self._publish_marker(path, pose)
        self.get_logger().info(f"✅ Mount geladen: {path}")

    def on_remove(self, _):
        self._delete_marker()
        self._current_mount = ""
        self.get_logger().info("🗑 Mount entfernt.")

    # ---------- helpers ----------

    def _lookup_mount_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                MOUNT_PARENT, MOUNT_FRAME, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().debug(f"TF-Lookup fehlgeschlagen: {e}")
            return None

        pos = (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z)
        quat = (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w)
        return pos, quat

    def _publish_marker(self, stl_path: str, pose):
        pos, quat = pose
        m = Marker()
        m.header.frame_id = MOUNT_PARENT
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "mount"
        m.id = 1
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = f"file://{stl_path}"
        m.pose = Pose()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = m.color.g = m.color.b = 0.7
        m.color.a = 1.0
        self.marker_pub.publish(m)

    def _delete_marker(self):
        m = Marker()
        m.header.frame_id = MOUNT_PARENT
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "mount"
        m.id = 1
        m.action = Marker.DELETE
        self.marker_pub.publish(m)

def main():
    rclpy.init()
    rclpy.spin(MountManager())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
