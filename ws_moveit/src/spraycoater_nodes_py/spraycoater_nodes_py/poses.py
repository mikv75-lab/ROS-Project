# -*- coding: utf-8 -*-
# spraycoater_nodes_py/poses.py
#!/usr/bin/env python3
from __future__ import annotations
import os
import yaml
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Empty
from tf2_ros import (
    StaticTransformBroadcaster,
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)

from spraycoater_nodes_py.utils.config_hub import topics, frames, config_path

NODE_KEY = "poses"


# ---------- Math utils (Legacy RPY nur als Fallback) ----------

def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    r = math.radians(float(roll_deg))
    p = math.radians(float(pitch_deg))
    y = math.radians(float(yaw_deg))
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    qw = cz * cy * cx + sz * sy * sx
    qx = cx * cy * sz - cz * sy * sx
    qy = cz * sx * cy + sz * cx * sy
    qz = cz * cx * sy - sz * cy * sx
    # normieren
    n = math.sqrt(qw * qw + qx * qx + qy * qy + qz * qz) or 1.0
    return (qx / n, qy / n, qz / n, qw / n)


def quat_to_rpy_deg(qx: float, qy: float, qz: float, qw: float):
    # Roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # Pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # Yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _normalize_quat(qx: float, qy: float, qz: float, qw: float):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    return (qx / n, qy / n, qz / n, qw / n)


class Poses(Node):
    """
    Nur statische TFs.

    Topics:
      subscribe:
        - poses.set_home     (std_msgs/Empty)
        - poses.set_service  (std_msgs/Empty)
      publish:
        - poses.home_pose    (geometry_msgs/PoseStamped, latched)
        - poses.service_pose (geometry_msgs/PoseStamped, latched)

    Verhalten:
      - set_home/set_service:
          * world->tcp per TF lesen
          * Pose (im world-Frame) mit EXAKTEM Quaternion (quat_xyzw) in YAML schreiben
          * ALLE Posen als /tf_static neu aussenden (inkl. home/service)
          * passende Pose gelatched publizieren
      - Startup:
          * ALLE Posen als /tf_static aussenden
          * vorhandene home/service zusätzlich als Pose publishen
    """

    NAMED = ("home", "service")

    def __init__(self):
        super().__init__("poses")

        # Resolver / Frames
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve
        self.frame_world = self._F(self.frames.get("world", "world"))
        # Home/Service beziehen sich jetzt auf das TCP-Frame
        self.frame_tcp = self._F(self.frames.get("tcp", "tcp"))

        # YAML laden
        poses_yaml = config_path("poses.yaml")
        if not poses_yaml or not os.path.exists(poses_yaml):
            raise FileNotFoundError(f"poses.yaml nicht gefunden: {poses_yaml}")
        with open(poses_yaml, "r", encoding="utf-8") as f:
            self.poses = yaml.safe_load(f) or {}
        if not isinstance(self.poses, dict):
            raise ValueError(
                "poses.yaml muss ein Mapping {name: {frame, xyz, quat_xyzw?(x,y,z,w) | rpy_deg?}} enthalten."
            )
        self.yaml_path = poses_yaml

        # Topics / QoS
        topic_set_home     = self.loader.subscribe_topic(NODE_KEY, "set_home")
        qos_set_home       = self.loader.qos_by_id("subscribe", NODE_KEY, "set_home")
        topic_set_service  = self.loader.subscribe_topic(NODE_KEY, "set_service")
        qos_set_service    = self.loader.qos_by_id("subscribe", NODE_KEY, "set_service")

        topic_home_pose    = self.loader.publish_topic(NODE_KEY, "home_pose")
        qos_home_pose      = self.loader.qos_by_id("publish", NODE_KEY, "home_pose")
        topic_service_pose = self.loader.publish_topic(NODE_KEY, "service_pose")
        qos_service_pose   = self.loader.qos_by_id("publish", NODE_KEY, "service_pose")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.tf_static  = StaticTransformBroadcaster(self)  # nur statisch

        # Publisher
        self.pub_home_pose    = self.create_publisher(PoseStamped, topic_home_pose, qos_home_pose)
        self.pub_service_pose = self.create_publisher(PoseStamped, topic_service_pose, qos_service_pose)

        # Subscriber
        self.create_subscription(Empty, topic_set_home, self._on_set_home, qos_set_home)
        self.create_subscription(Empty, topic_set_service, self._on_set_service, qos_set_service)

        # Initial: alle statischen TFs senden + evtl. Pose publishen
        self._send_all_static_tfs()
        self._publish_named_pose_if_exists("home")
        self._publish_named_pose_if_exists("service")

        self.get_logger().info(
            f"✅ PosesManager aktiv – {len(self.poses)} Posen (world='{self.frame_world}', tcp='{self.frame_tcp}') – nur /tf_static"
        )

    # ---------- TF/Publish Utilities ----------

    def _send_all_static_tfs(self) -> None:
        tfs = []
        now = self.get_clock().now().to_msg()
        for name, pose in self.poses.items():
            msg, tf = self._compose_pose_and_tf(name, pose, force_parent_world=True)
            tf.header.stamp = now
            tfs.append(tf)
        if tfs:
            self.tf_static.sendTransform(tfs)
        self.get_logger().info(f"📡 /tf_static gesendet: {len(tfs)} Posen")

    def _publish_named_pose_if_exists(self, name: str) -> None:
        pose = self.poses.get(name)
        if not isinstance(pose, dict):
            return
        self._publish_named_pose(name, pose)

    def _publish_named_pose(self, name: str, pose_dict: dict) -> None:
        # Pose gelatched publishen (Frame world)
        msg, tf = self._compose_pose_and_tf(name, pose_dict, force_parent_world=True)
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        tf.header.stamp = now

        if name == "home":
            self.pub_home_pose.publish(msg)
        elif name == "service":
            self.pub_service_pose.publish(msg)

        # Nur statisch senden (einzeln zusätzlich ok)
        self.tf_static.sendTransform([tf])
        self.get_logger().info(
            f"🧭 /tf_static aktualisiert für '{name}' (parent={tf.header.frame_id}) + Pose gepublished"
        )

    def _compose_pose_and_tf(
        self, name: str, pose: dict, *, force_parent_world: bool = True
    ) -> tuple[PoseStamped, TransformStamped]:
        parent_frame = self.frame_world if force_parent_world else self._F(
            pose.get("frame", self.frame_world)
        )
        x, y, z = pose.get("xyz", [0.0, 0.0, 0.0])

        if "quat_xyzw" in pose:
            qx, qy, qz, qw = pose["quat_xyzw"]
        else:
            # Legacy-Fallback (rpy_deg)
            r_deg, p_deg, y_deg = pose.get("rpy_deg", [0.0, 0.0, 0.0])
            qx, qy, qz, qw = rpy_deg_to_quat(r_deg, p_deg, y_deg)

        qx, qy, qz, qw = _normalize_quat(float(qx), float(qy), float(qz), float(qw))

        msg = PoseStamped()
        msg.header.frame_id = parent_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        tf = TransformStamped()
        tf.header.frame_id = parent_frame
        tf.child_frame_id  = name
        tf.transform.translation.x = float(x)
        tf.transform.translation.y = float(y)
        tf.transform.translation.z = float(z)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        return msg, tf

    # ---------- Core: tcp → world speichern (EXAKT) ----------

    def _lookup_tcp_in_world(self) -> dict | None:
        target = self.frame_world
        source = self.frame_tcp
        timeout = Duration(seconds=1.0)

        if not self.tf_buffer.can_transform(target, source, Time(), timeout):
            self.get_logger().warning(
                f"TF fehlt: '{target}' ↔ '{source}' nicht verbunden – set_* abgebrochen."
            )
            return None

        try:
            tf = self.tf_buffer.lookup_transform(target, source, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(f"TF lookup fehlgeschlagen (world→tcp): {e}")
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        qx, qy, qz, qw = _normalize_quat(q.x, q.y, q.z, q.w)

        return {
            "frame": target,  # explizit 'world'
            "xyz": [float(t.x), float(t.y), float(t.z)],
            "quat_xyzw": [float(qx), float(qy), float(qz), float(qw)],
            # optional: nur zur menschlichen Lesbarkeit
            # "rpy_deg": list(quat_to_rpy_deg(qx, qy, qz, qw)),
        }

    def _save_pose_and_publish(self, name: str) -> None:
        pose_dict = self._lookup_tcp_in_world()
        if pose_dict is None:
            self.get_logger().warning(
                f"{name}: Abbruch – kein world→tcp TF vorhanden."
            )
            return

        # YAML persistieren (immer frame='world', mit exaktem quat_xyzw)
        self.poses[name] = pose_dict
        try:
            with open(self.yaml_path, "w", encoding="utf-8") as f:
                yaml.dump(self.poses, f, sort_keys=False)
            self.get_logger().info(
                f"💾 {name}: YAML aktualisiert (frame=world, quat_xyzw) aus TF world→tcp"
            )
        except Exception as e:
            self.get_logger().error(f"{name}: YAML schreiben fehlgeschlagen: {e}")
            return

        # ALLE statisch neu senden (robust) + Pose publishen
        self._send_all_static_tfs()
        self._publish_named_pose(name, pose_dict)

    # ---------- Callbacks ----------

    def _on_set_home(self, _: Empty) -> None:
        self._save_pose_and_publish("home")

    def _on_set_service(self, _: Empty) -> None:
        self._save_pose_and_publish("service")


def main(args=None):
    rclpy.init(args=args)
    node = Poses()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
