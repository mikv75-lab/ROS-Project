# spraycoater_nodes_py/spray_path.py
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics, frames


class SprayPath(Node):
    """
    SprayPath-Manager.

    SUB:
      - spray_path.set (MarkerArray)
        → Kommando vom PyQt-Editor (fertiger Pfad als MarkerArray)

    PUB:
      - spray_path.poses   (PoseArray): Punkte des Pfads als neutrale Posen
      - spray_path.markers (MarkerArray, latched): aktuelles MarkerArray für RViz
      - spray_path.current (String, latched): aktueller Pfad-Name (optional für UI)

    Semantik:
      - JEDER set-Aufruf ersetzt den aktuellen Pfad vollständig.
      - Es wird NICHT auf /spraycoater/spray_path/set zurück publiziert.
    """

    GROUP = "spray_path"

    def __init__(self):
        super().__init__("spraypath_manager")

        # Config-Hub
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve

        # Topics & QoS
        topic_set     = self.loader.subscribe_topic(self.GROUP, "set")
        qos_set       = self.loader.qos_by_id("subscribe", self.GROUP, "set")

        topic_current = self.loader.publish_topic(self.GROUP, "current")
        qos_current   = self.loader.qos_by_id("publish",   self.GROUP, "current")

        topic_poses   = self.loader.publish_topic(self.GROUP, "poses")
        qos_poses     = self.loader.qos_by_id("publish",   self.GROUP, "poses")

        topic_markers = self.loader.publish_topic(self.GROUP, "markers")
        qos_markers   = self.loader.qos_by_id("publish",   self.GROUP, "markers")

        # Publisher / Subscriber
        self.pub_current = self.create_publisher(String,      topic_current, qos_current)
        self.pub_poses   = self.create_publisher(PoseArray,   topic_poses,   qos_poses)
        self.pub_markers = self.create_publisher(MarkerArray, topic_markers, qos_markers)

        self.sub_set = self.create_subscription(
            MarkerArray,
            topic_set,
            self._on_set_spraypath,
            qos_set,
        )

        # interner Zustand
        self._last_frame = self.frames.get("scene", "scene")
        self._last_name: str = ""
        self._last_pa: PoseArray | None = None
        self._last_markers: MarkerArray | None = None

        self.get_logger().info(
            "✅ SprayPathManager bereit: /set → /poses + /markers (+ /current), kein Publish auf /set"
        )

    # ----------------------------- helpers -----------------------------

    @staticmethod
    def _extract_name(ma: MarkerArray) -> str:
        # 1) TEXT_VIEW_FACING.text
        for m in ma.markers:
            if m.type == Marker.TEXT_VIEW_FACING:
                t = (m.text or "").strip()
                if t:
                    return t

        # 2) erstes non-empty ns
        for m in ma.markers:
            ns = (m.ns or "").strip()
            if ns:
                return ns

        # kein Fallback-Text – dann bleibt Name ""
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

    # ----------------------------- handler -----------------------------

    def _on_set_spraypath(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self.get_logger().warning("⚠️ spray_path.set: leeres MarkerArray – ignoriere")
            return

        # Frame bestimmen: erster Marker mit non-empty frame_id
        frame = self._last_frame
        for m in msg.markers:
            fid = (m.header.frame_id or "").strip()
            if fid:
                frame = fid
                break
        self._last_frame = frame

        # Name aus Text/Namespace holen (kann leer sein)
        name = self._extract_name(msg)

        # Pfad-Marker wählen (bevorzugt LINE_STRIP)
        m_path = self._prefer_linestrip(msg)
        if m_path is None:
            self.get_logger().warning("⚠️ spray_path.set: kein Marker mit ≥2 Punkten – ignoriere")
            return

        now = self.get_clock().now().to_msg()

        # PoseArray bauen
        pa = PoseArray()
        pa.header.frame_id = frame
        pa.header.stamp = now

        for pt in m_path.points:
            p = Pose()
            p.position.x = pt.x
            p.position.y = pt.y
            p.position.z = pt.z
            p.orientation.w = 1.0  # neutral
            pa.poses.append(p)

        # MarkerArray für RViz: wir übernehmen msg unverändert,
        # setzen aber einheitlich Frame + Timestamp
        out_ma = MarkerArray()
        for src in msg.markers:
            m = Marker()
            # Felder manuell kopieren (kein Fallback, keine Magie)
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

        # Zustand REPLAZEN
        self._last_pa = pa
        self._last_markers = out_ma
        self._last_name = name

        # ONE SHOT Publish
        self.pub_poses.publish(pa)
        self.pub_markers.publish(out_ma)
        self._publish_current_name(name)

        self.get_logger().info(
            f"🎯 SprayPath gesetzt: name='{name}', frame='{frame}', "
            f"points={len(pa.poses)}, markers={len(out_ma.markers)}"
        )


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
