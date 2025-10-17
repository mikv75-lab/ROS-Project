#!/usr/bin/env python3
# mecademic_bringup/poses_manager.py

from __future__ import annotations

import math
import os
import re
from typing import Dict, Tuple, Any

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped, TransformStamped
from builtin_interfaces.msg import Time as TimeMsg

from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster


# -------- QoS (latched) --------
LATCHED = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

# bekannte/übliche Pose-Namen – nur als Orientierung
KNOWN_POSE_NAMES = {"home", "workspace_center", "predispense", "service"}


# -------- Mathe-Helfer --------
def rpy_deg_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Tuple[float, float, float, float]:
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r * 0.5), math.sin(r * 0.5)
    cp, sp = math.cos(p * 0.5), math.sin(p * 0.5)
    cy, sy = math.cos(y * 0.5), math.sin(y * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    # normieren
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw) or 1.0
    return (qx/n, qy/n, qz/n, qw/n)


def quat_to_rpy_deg(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    # roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)
    # yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# -------- YAML Laden (robust) --------
def _as_vec3(v: Any, default=(0.0, 0.0, 0.0)) -> Tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        return tuple(default)
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:
        return tuple(default)


def _extract_pose_dict(candidate: Any) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    """
    Erwartet dict wie:
      { name: {xyz: [..], rpy_deg: [..]}, ... }
    liefert {name: {"xyz": (..), "rpy_deg": (..)}, ...} oder {}.
    """
    out: Dict[str, Dict[str, Tuple[float, float, float]]] = {}
    if not isinstance(candidate, dict):
        return out
    for k, v in candidate.items():
        if not isinstance(v, dict):
            continue
        xyz = _as_vec3(v.get("xyz"))
        rpy = _as_vec3(v.get("rpy_deg"))
        out[str(k)] = {"xyz": xyz, "rpy_deg": rpy}
    # filtern: nur Einträge behalten, die sinnvolle 3er-Vektoren haben
    out = {k: vv for k, vv in out.items()
           if isinstance(vv.get("xyz"), tuple) and isinstance(vv.get("rpy_deg"), tuple)}
    return out


def load_poses_from_yaml(path: str) -> Dict[str, Dict[str, Tuple[float, float, float]]]:
    """
    Unterstützte Layouts:

    1) poses: { home: {xyz:[], rpy_deg:[]}, ... }
    2) fixed_positions: { home: {...}, ... }
    3) { home: {...}, workspace_center: {...}, ... }   # direkt am Top-Level
    4) Falls top-level nur EINEN Schlüssel hat (z.B. "fixed_positions"): verwende dessen Wert.

    Fallback: leeres Dict → keine Posen, Node läuft trotzdem.
    """
    try:
        if not path or not os.path.isfile(path):
            return {}
        with open(path, "r") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}

    # 1) 'poses'
    if "poses" in data and isinstance(data["poses"], dict):
        poses = _extract_pose_dict(data["poses"])
        if poses:
            return poses

    # 2) 'fixed_positions'
    if "fixed_positions" in data and isinstance(data["fixed_positions"], dict):
        poses = _extract_pose_dict(data["fixed_positions"])
        if poses:
            return poses

    # 3) direkt am Top-Level
    poses = _extract_pose_dict(data)
    if poses and (KNOWN_POSE_NAMES.intersection(poses.keys()) or len(poses) >= 1):
        return poses

    # 4) einziger Top-Level-Key, der wieder ein Dict mit Posen enthält
    if isinstance(data, dict) and len(data) == 1:
        only_val = next(iter(data.values()))
        poses = _extract_pose_dict(only_val)
        if poses:
            return poses

    return {}


# -------- Node --------
class PosesManager(Node):
    def __init__(self) -> None:
        super().__init__("poses_manager")

        self.declare_parameter("positions_yaml", "")
        self.declare_parameter("parent_frame", "meca_mount")

        self.yaml_path: str = self.get_parameter("positions_yaml").get_parameter_value().string_value
        self.parent_frame: str = self.get_parameter("parent_frame").get_parameter_value().string_value

        # TF infra
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Posen laden
        self.poses: Dict[str, Dict[str, Tuple[float, float, float]]] = load_poses_from_yaml(self.yaml_path)
        if not self.poses:
            self.get_logger().warning(f"[poses_manager] ⚠️ keine Posen aus YAML geladen: {self.yaml_path}")

        # Topic Publisher pro Pose (latched)
        self.pose_publishers: Dict[str, rclpy.publisher.Publisher] = {}
        for name in self.poses.keys():
            self.pose_publishers[name] = self.create_publisher(PoseStamped, f"/meca/poses/{name}", LATCHED)

        # Subscriptions: Posen direkt setzen
        for name in list(self.poses.keys()):
            self.create_subscription(PoseStamped, f"/meca/poses/set/{name}", self._make_set_cb(name), 10)

        # Aus TCP übernehmen (String, z. B. "home tcp_frame" oder "name=home frame=tcp_frame")
        self.create_subscription(String, "/meca/poses/set_from_tcp", self._on_set_from_tcp, 10)

        # Re-publish aller Posen
        self.create_subscription(Empty, "/meca/poses/republish", self._on_republish, 10)

        # Einmal initial publizieren
        self._publish_all()

        self.get_logger().info(
            f"[poses_manager] ✅ Ready, parent_frame='{self.parent_frame}', poses={list(self.poses.keys())}"
        )

    # ---------- Publish Helpers ----------
    def _pose_to_tf(self, name: str, pose: Dict[str, Tuple[float, float, float]]) -> TransformStamped:
        xyz = pose["xyz"]
        rpy = pose["rpy_deg"]
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy)

        t = TransformStamped()
        # **statisch**: Zeitstempel 0 → latched auf /tf_static
        t.header.stamp = TimeMsg(sec=0, nanosec=0)
        t.header.frame_id = self.parent_frame
        t.child_frame_id = f"meca_pose_{name}"
        t.transform.translation.x = float(xyz[0])
        t.transform.translation.y = float(xyz[1])
        t.transform.translation.z = float(xyz[2])
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)
        return t

    def _publish_pose_topic(self, name: str, pose: Dict[str, Tuple[float, float, float]]) -> None:
        xyz = pose["xyz"]
        rpy = pose["rpy_deg"]
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy)

        msg = PoseStamped()
        msg.header.frame_id = self.parent_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.position.z = float(xyz[2])
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)

        pub = self.pose_publishers.get(name)
        if pub is None:
            pub = self.create_publisher(PoseStamped, f"/meca/poses/{name}", LATCHED)
            self.pose_publishers[name] = pub
        pub.publish(msg)

    def _publish_all(self) -> None:
        # Topics
        for name, p in self.poses.items():
            self._publish_pose_topic(name, p)
        # TFs statisch (einmalig)
        tfs = [self._pose_to_tf(name, p) for name, p in self.poses.items()]
        if tfs:
            self.static_broadcaster.sendTransform(tfs)

    def _apply_and_republish(self, name: str, new_pose: Dict[str, Tuple[float, float, float]]) -> None:
        self.poses[name] = new_pose
        self._publish_pose_topic(name, new_pose)
        self.static_broadcaster.sendTransform([self._pose_to_tf(name, new_pose)])
        self.get_logger().info(f"[poses_manager] pose '{name}' updated → republished (topic + /tf_static)")

    # ---------- Callbacks ----------
    def _make_set_cb(self, name: str):
        def _cb(msg: PoseStamped) -> None:
            xyz = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            q = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            rpy_deg = quat_to_rpy_deg(*q)
            self._apply_and_republish(name, {"xyz": xyz, "rpy_deg": rpy_deg})
        return _cb

    def _on_set_from_tcp(self, msg: String) -> None:
        """
        Akzeptiert:
          "home tcp_frame"
          "name=home frame=tcp_frame"
        Holt Transform parent_frame <- tcp_frame (aktuellste) und übernimmt xyz + rpy_deg.
        """
        text = msg.data.strip()
        if not text:
            self.get_logger().warning("[poses_manager] set_from_tcp: leerer String")
            return

        # Parsen
        name = None
        frame = None
        m_name = re.search(r"(?:^|\s)name\s*=\s*([^\s]+)", text)
        m_frame = re.search(r"(?:^|\s)frame\s*=\s*([^\s]+)", text)
        if m_name:
            name = m_name.group(1)
        if m_frame:
            frame = m_frame.group(1)
        if name is None or frame is None:
            parts = text.split()
            if len(parts) == 2:
                name = name or parts[0]
                frame = frame or parts[1]

        if not name or not frame:
            self.get_logger().warning(f"[poses_manager] set_from_tcp: unverständlich: {text!r}")
            return

        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame=self.parent_frame,
                source_frame=frame,
                time=Time(),  # "latest"
                timeout=Duration(seconds=1.0),
            )
        except Exception as e:
            self.get_logger().warning(
                f"[poses_manager] set_from_tcp: TF lookup {self.parent_frame} <- {frame} fehlgeschlagen: {e}"
            )
            return

        xyz = (
            float(tf.transform.translation.x),
            float(tf.transform.translation.y),
            float(tf.transform.translation.z),
        )
        q = (
            float(tf.transform.rotation.x),
            float(tf.transform.rotation.y),
            float(tf.transform.rotation.z),
            float(tf.transform.rotation.w),
        )
        rpy_deg = quat_to_rpy_deg(*q)

        # Publisher/Subscription für neue Posen on-the-fly anlegen
        if name not in self.pose_publishers:
            self.pose_publishers[name] = self.create_publisher(PoseStamped, f"/meca/poses/{name}", LATCHED)
            self.create_subscription(PoseStamped, f"/meca/poses/set/{name}", self._make_set_cb(name), 10)

        self._apply_and_republish(name, {"xyz": xyz, "rpy_deg": rpy_deg})

    def _on_republish(self, _: Empty) -> None:
        self.get_logger().info("[poses_manager] republish all requested")
        self._publish_all()


# -------- main --------
def main() -> None:
    rclpy.init()
    node = PosesManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()