# spraycoater_nodes_py/spray_path.py
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics, frames


class SprayPath(Node):
    """
    SprayPath-Manager.

    SUB:
      - spraycoater/spray_path/set               (MarkerArray)
      - spraycoater/spray_path/executed_poses_in (PoseArray)

    PUB (QoS via config_hub, i.d.R. latched/TRANSIENT_LOCAL + 1 Hz republish):
      - spraycoater/spray_path/poses
      - spraycoater/spray_path/markers
      - spraycoater/spray_path/current
      - spraycoater/spray_path/executed_poses
      - spraycoater/spray_path/executed_markers

    Namespace kommt ausschließlich über den Node (Launchfile).
    """

    GROUP = "spray_path"

    def __init__(self) -> None:
        super().__init__("spraypath_manager")

        # ---------------- Backend-Parameter ----------------
        self.declare_parameter("backend", "default")
        self.backend: str = (
            self.get_parameter("backend").get_parameter_value().string_value or "default"
        )

        # Config-Hub
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve

        # ---------------- Topics & QoS via config_hub ----------------
        # SUB
        topic_set = self.loader.subscribe_topic(self.GROUP, "set")
        qos_set = self.loader.qos_by_id("subscribe", self.GROUP, "set")

        topic_exec_in = self.loader.subscribe_topic(self.GROUP, "executed_poses_in")
        qos_exec_in = self.loader.qos_by_id("subscribe", self.GROUP, "executed_poses_in")

        # PUB (QoS kommt aus qos.yaml über die IDs)
        topic_current = self.loader.publish_topic(self.GROUP, "current")
        qos_current = self.loader.qos_by_id("publish", self.GROUP, "current")

        topic_poses = self.loader.publish_topic(self.GROUP, "poses")
        qos_poses = self.loader.qos_by_id("publish", self.GROUP, "poses")

        topic_markers = self.loader.publish_topic(self.GROUP, "markers")
        qos_markers = self.loader.qos_by_id("publish", self.GROUP, "markers")

        topic_exec_poses = self.loader.publish_topic(self.GROUP, "executed_poses")
        qos_exec_poses = self.loader.qos_by_id("publish", self.GROUP, "executed_poses")

        topic_exec_markers = self.loader.publish_topic(self.GROUP, "executed_markers")
        qos_exec_markers = self.loader.qos_by_id("publish", self.GROUP, "executed_markers")

        # ---------------- Publisher / Subscriber ----------------
        self.pub_current = self.create_publisher(String, topic_current, qos_current)
        self.pub_poses = self.create_publisher(PoseArray, topic_poses, qos_poses)
        self.pub_markers = self.create_publisher(MarkerArray, topic_markers, qos_markers)
        self.pub_exec_poses = self.create_publisher(PoseArray, topic_exec_poses, qos_exec_poses)
        self.pub_exec_markers = self.create_publisher(MarkerArray, topic_exec_markers, qos_exec_markers)

        self.sub_set = self.create_subscription(
            MarkerArray,
            topic_set,
            self._on_set_spraypath,
            qos_set,
        )

        self.sub_exec = self.create_subscription(
            PoseArray,
            topic_exec_in,
            self._on_executed_poses,
            qos_exec_in,
        )

        # ---------------- Interner Zustand ----------------
        self._last_frame = self.frames.get("scene", "scene")
        self._last_name: str = ""
        self._last_pa: PoseArray | None = None
        self._last_markers: MarkerArray | None = None

        self._last_exec_frame = self._last_frame
        self._last_exec_pa: PoseArray | None = None
        self._last_exec_markers: MarkerArray | None = None

        # 1 Hz Republish Timer
        self._timer = self.create_timer(1.0, self._republish_all)

        self.get_logger().info(
            f"✅ SprayPathManager bereit (backend='{self.backend}'): "
            f"set→poses/markers, exec_in→executed_*, QoS via config_hub + 1Hz Republish."
        )

    # ----------------------------------------------------------------------
    # Helper
    # ----------------------------------------------------------------------

    @staticmethod
    def _extract_name(ma: MarkerArray) -> str:
        for m in ma.markers:
            if m.type == Marker.TEXT_VIEW_FACING:
                t = (m.text or "").strip()
                if t:
                    return t
        for m in ma.markers:
            ns = (m.ns or "").strip()
            if ns:
                return ns
        return ""

    @staticmethod
    def _prefer_linestrip(ma: MarkerArray) -> Marker | None:
        best = None
        for m in ma.markers:
            if len(m.points) >= 2:
                if m.type == Marker.LINE_STRIP:
                    return m
                if best is None:
                    best = m
        return best

    def _publish_current_name(self, name: str) -> None:
        self.pub_current.publish(String(data=name))

    # ----------------------------------------------------------------------
    # Handler: Sollpfad
    # ----------------------------------------------------------------------

    def _on_set_spraypath(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self.get_logger().warning(
                f"[{self.backend}] ⚠️ spray_path.set: leeres MarkerArray – ignoriere"
            )
            return

        frame = self._last_frame
        for m in msg.markers:
            fid = (m.header.frame_id or "").strip()
            if fid:
                frame = fid
                break
        self._last_frame = frame

        name = self._extract_name(msg)
        m_path = self._prefer_linestrip(msg)
        if m_path is None:
            self.get_logger().warning(
                f"[{self.backend}] ⚠️ spray_path.set: kein gültiger Pfad – ignoriere"
            )
            return

        now = self.get_clock().now().to_msg()

        # PoseArray (Sollpfad)
        pa = PoseArray()
        pa.header.frame_id = frame
        pa.header.stamp = now

        for pt in m_path.points:
            p = Pose()
            p.position.x = pt.x
            p.position.y = pt.y
            p.position.z = pt.z
            p.orientation.w = 1.0
            pa.poses.append(p)

        # MarkerArray (Sollpfad)
        out_ma = MarkerArray()
        for src in msg.markers:
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = src.ns
            m.id = src.id
            m.type = src.type
            m.action = src.action
            m.pose = src.pose
            m.scale = src.scale
            m.color = src.color
            m.lifetime = src.lifetime
            m.frame_locked = src.frame_locked
            m.points = list(src.points)
            m.colors = list(src.colors)
            m.text = src.text
            m.mesh_resource = src.mesh_resource
            m.mesh_use_embedded_materials = src.mesh_use_embedded_materials
            out_ma.markers.append(m)

        # Zustand merken
        self._last_pa = pa
        self._last_markers = out_ma
        self._last_name = name

        # One-Shot + latched (über QoS aus config_hub)
        self.pub_poses.publish(pa)
        self.pub_markers.publish(out_ma)
        self._publish_current_name(name)

        self.get_logger().info(
            f"[{self.backend}] 🎯 Sollpfad gesetzt: name='{name}', "
            f"frame='{frame}', points={len(pa.poses)}"
        )

    # ----------------------------------------------------------------------
    # Handler: Istpfad
    # ----------------------------------------------------------------------

    def _on_executed_poses(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            self.get_logger().warning(
                f"[{self.backend}] ⚠️ executed_poses_in: leer – ignoriere"
            )
            return

        frame = (msg.header.frame_id or "").strip() or self._last_exec_frame
        self._last_exec_frame = frame

        now = self.get_clock().now().to_msg()

        # PoseArray (Istpfad)
        pa = PoseArray()
        pa.header.frame_id = frame
        pa.header.stamp = now
        pa.poses = list(msg.poses)

        if len(pa.poses) < 2:
            self.get_logger().warning(
                f"[{self.backend}] ⚠️ executed_poses_in: <2 Posen – ignoriere"
            )
            return

        # MarkerArray (Istpfad) – Marker Properties sind ok, QoS kommt vom Publisher
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = now
        m.ns = "executed_path"
        m.id = 1000
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.002
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 1.0

        for p in pa.poses:
            pt = Point()
            pt.x = p.position.x
            pt.y = p.position.y
            pt.z = p.position.z
            m.points.append(pt)

        out_ma = MarkerArray()
        out_ma.markers.append(m)

        # Zustand merken
        self._last_exec_pa = pa
        self._last_exec_markers = out_ma

        # One-Shot + latched
        self.pub_exec_poses.publish(pa)
        self.pub_exec_markers.publish(out_ma)

        self.get_logger().info(
            f"[{self.backend}] ✅ Istpfad gesetzt: points={len(pa.poses)}, frame='{frame}'"
        )

    # ----------------------------------------------------------------------
    # 1 Hz REPUBLISH
    # ----------------------------------------------------------------------

    def _republish_all(self) -> None:
        if self._last_pa is not None:
            self.pub_poses.publish(self._last_pa)
        if self._last_markers is not None:
            self.pub_markers.publish(self._last_markers)

        if self._last_name:
            self._publish_current_name(self._last_name)

        if self._last_exec_pa is not None:
            self.pub_exec_poses.publish(self._last_exec_pa)
        if self._last_exec_markers is not None:
            self.pub_exec_markers.publish(self._last_exec_markers)


def main(args=None):
    rclpy.init(args=args)
    node = SprayPath()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
