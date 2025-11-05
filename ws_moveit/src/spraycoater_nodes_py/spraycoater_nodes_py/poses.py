# -*- coding: utf-8 -*-
# spraycoater_nodes_py/poses.py
#!/usr/bin/env python3
from __future__ import annotations
import os, yaml, math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, TransformStamped, PoseArray, Pose
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener
from rclpy.qos import QoSProfile
from tf_transformations import euler_from_quaternion

PARAM_POSES_CONFIG = "poses_config"
PARAM_TOPICS_YAML  = "topics_yaml"
PARAM_QOS_YAML     = "qos_yaml"
PARAM_FRAMES_YAML  = "frames_yaml"
NODE_KEY           = "poses"  # <- Blockname in deinen Topics

def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    r = math.radians(float(roll_deg))
    p = math.radians(float(pitch_deg))
    y = math.radians(float(yaw_deg))
    cx, sx = math.cos(r), math.sin(r)
    cy, sy = math.cos(p), math.sin(p)
    cz, sz = math.cos(y), math.sin(y)
    qw = cz*cy*cx + sz*sy*sx
    qx = cx*cy*sz - cz*sy*sx
    qy = cz*sx*cy + sz*cx*sy
    qz = cz*cx*sy - sz*cy*sx
    return (qx, qy, qz, qw)

# Pflicht: TopicsLoader vorhanden
from .utils.topics_loader import TopicsLoader
from spraycoater_nodes_py.utils.frames import load_frames


class Poses(Node):
    """
    - Lädt Posen aus YAML
    - veröffentlicht statische TFs für alle Posen
    - veröffentlicht PoseArray (latched) + String "pose_changed"
    - akzeptiert PoseSet via PoseStamped auf Topic aus TopicsLoader
      Schema: msg.header.frame_id = "<parent>#<name>"
    """
    def __init__(self):
        super().__init__("poses_manager")

        # --- Parameter strikt prüfen ---
        self.declare_parameter(PARAM_POSES_CONFIG, "")
        self.declare_parameter(PARAM_TOPICS_YAML, "")
        self.declare_parameter(PARAM_QOS_YAML, "")
        self.declare_parameter(PARAM_FRAMES_YAML, "")

        poses_yaml  = self.get_parameter(PARAM_POSES_CONFIG).value
        topics_yaml = self.get_parameter(PARAM_TOPICS_YAML).value
        qos_yaml    = self.get_parameter(PARAM_QOS_YAML).value
        frames_yaml = self.get_parameter(PARAM_FRAMES_YAML).value

        if not poses_yaml or not os.path.exists(poses_yaml):
            raise FileNotFoundError(f"{PARAM_POSES_CONFIG} fehlt/ungültig: {poses_yaml}")
        if not topics_yaml or not os.path.exists(topics_yaml):
            raise FileNotFoundError(f"{PARAM_TOPICS_YAML} fehlt/ungültig: {topics_yaml}")
        if not qos_yaml or not os.path.exists(qos_yaml):
            raise FileNotFoundError(f"{PARAM_QOS_YAML} fehlt/ungültig: {qos_yaml}")
        if not frames_yaml or not os.path.exists(frames_yaml):
            raise FileNotFoundError(f"{PARAM_FRAMES_YAML} fehlt/ungültig: {frames_yaml}")

        # Posen laden (Mapping {name: {frame, xyz, rpy_deg}})
        with open(poses_yaml, "r", encoding="utf-8") as f:
            self.poses = yaml.safe_load(f) or {}
        if not isinstance(self.poses, dict):
            raise ValueError("Poses YAML muss ein Mapping {name: spec} enthalten.")
        self.yaml_path = poses_yaml

        # Topics/QoS aus Loader (strict)
        tl = TopicsLoader(topics_yaml, qos_yaml)
        self.topic_pose_set      = tl.subscribe_topic(NODE_KEY, "pose_set")
        self.qos_pose_set: QoSProfile = tl.qos_by_id("subscribe", NODE_KEY, "pose_set")

        self.topic_poses_list    = tl.publish_topic (NODE_KEY, "poses_list")
        self.qos_poses_list: QoSProfile = tl.qos_by_id("publish",  NODE_KEY, "poses_list")

        self.topic_pose_changed  = tl.publish_topic (NODE_KEY, "pose_changed")
        self.qos_pose_changed: QoSProfile = tl.qos_by_id("publish",  NODE_KEY, "pose_changed")

        # Frames laden
        self.frames  = load_frames(frames_yaml)
        self._F      = self.frames.resolve  # Kurzform

        # --- TF & Publisher/Subscriber Setup ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf = StaticTransformBroadcaster(self)

        self.pub_poses_list   = self.create_publisher(PoseArray, self.topic_poses_list, self.qos_poses_list)
        self.pub_pose_changed = self.create_publisher(String,     self.topic_pose_changed, self.qos_poses_changed_or_default())

        # Subscriber für Pose-Set (PoseStamped)
        self.create_subscription(PoseStamped, self.topic_pose_set, self._on_pose_set, self.qos_pose_set)

        # initial veröffentlichen
        self.publish_all()
        self.get_logger().info(f"✅ PosesManager aktiv (node_key='{NODE_KEY}') – {len(self.poses)} Posen")

    def qos_poses_changed_or_default(self) -> QoSProfile:
        # Falls die QoS-ID fehlt, fallback auf self.qos_poses_list (robust)
        try:
            # In TopicsLoader muss eine id "pose_changed" existieren, wir versuchen sie sauber zu laden:
            return self.qos_poses_changed  # type: ignore[attr-defined]
        except Exception:
            return self.qos_poses_list

    # -------------------- Publishing --------------------
    def publish_all(self):
        """Statische TFs + PoseArray publizieren."""
        tfs = []
        for name, pose in self.poses.items():
            msg, tf = self._make_pose(name, pose)
            tfs.append(tf)
        if tfs:
            self.static_tf.sendTransform(tfs)
        self._publish_pose_array()
        self._emit_changed("published")
        self.get_logger().info(f"📡 {len(tfs)} Posen als TF + PoseArray veröffentlicht")

    def _publish_pose_array(self):
        pa = PoseArray()
        pa.header.frame_id = self.frames.get("world", "world")  # einheitlicher Frame für Liste
        for name, pose in self.poses.items():
            x, y, z = pose.get("xyz", [0,0,0])
            r, p, y_ = pose.get("rpy_deg", [0,0,0])
            qx, qy, qz, qw = rpy_deg_to_quat(r, p, y_)
            ps = Pose()
            ps.position.x, ps.position.y, ps.position.z = float(x), float(y), float(z)
            ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = qx, qy, qz, qw
            pa.poses.append(ps)
        self.pub_poses_list.publish(pa)

    def _emit_changed(self, name: str):
        s = String()
        s.data = name
        self.pub_pose_changed.publish(s)

    # -------------------- Helpers --------------------
    def _make_pose(self, name: str, pose: dict):
        parent_frame = self._F(pose.get("frame", self.frames.get("tool_mount", "tool_mount")))
        x, y, z = pose.get("xyz", [0, 0, 0])
        r_deg, p_deg, y_deg = pose.get("rpy_deg", [0, 0, 0])
        qx, qy, qz, qw = rpy_deg_to_quat(r_deg, p_deg, y_deg)

        # PoseStamped (nur intern)
        msg = PoseStamped()
        msg.header.frame_id = parent_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        # Static TF
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

    # -------------------- Callback --------------------
    def _on_pose_set(self, msg: PoseStamped):
        """
        Setzt/überschreibt eine Pose.
        Schema (strikt): msg.header.frame_id = "<parent>#<name>"
        Beispiel: "world#home"
        """
        frame_id = (msg.header.frame_id or "").strip()
        if "#" not in frame_id:
            raise ValueError("pose_set: frame_id muss Schema '<parent>#<name>' haben, z.B. 'world#home'")
        parent, name = frame_id.split("#", 1)
        parent = self._F(parent.strip())
        name = name.strip()
        if not parent or not name:
            raise ValueError("pose_set: ungültiges frame_id Schema '<parent>#<name>'")

        # Pose übernehmen
        q = msg.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.poses[name] = {
            "frame": parent,
            "xyz": [
                round(msg.pose.position.x, 4),
                round(msg.pose.position.y, 4),
                round(msg.pose.position.z, 4),
            ],
            "rpy_deg": [
                round(math.degrees(roll), 3),
                round(math.degrees(pitch), 3),
                round(math.degrees(yaw), 3),
            ],
        }

        # speichern
        with open(self.yaml_path, "w", encoding="utf-8") as f:
            yaml.dump(self.poses, f, sort_keys=False)

        # republish TFs + Liste + changed(name)
        self.publish_all()
        self._emit_changed(name)
        self.get_logger().info(f"💾 pose_set: '{name}' in Parent '{parent}' gespeichert.")


# ------------------------------------------------------------
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
