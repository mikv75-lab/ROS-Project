# spraycoater_nodes_py/spray_path.py
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray
from visualization_msgs.msg import MarkerArray  # nur für SUB (set)

# ZENTRALER HUB: Topics/QoS/Frames aus bringup/config
from spraycoater_nodes_py.utils.config_hub import topics, frames


class SprayPath(Node):
    """
    Minimaler SprayPath-Manager (config_hub-only).

    SUB:
      - spray_path.set (MarkerArray):
          erwartet mind. einen Marker mit >=2 Punkten,
          bevorzugt LINE_STRIP.

    PUB:
      - spray_path.current (std_msgs/String, latched): *genauer* Name (kein Fallback-Text)
      - spray_path.poses   (geometry_msgs/PoseArray): neutral orientierte Posen der Pfadpunkte

    Verhalten:
      - Frame = header.frame_id des ersten Markers; wenn leer, bleibt letzter Wert bestehen
      - Re-Publish PoseArray per Timer, damit RViz die Posen stabil hält
    """

    GROUP = "spray_path"

    def __init__(self):
        super().__init__("spraypath_manager")

        # Loader / Frames
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve  # aktuell nicht genutzt, aber bereit

        # Topic-Namen & QoS aus topics.yaml / qos.yaml
        topic_set     = self.loader.subscribe_topic(self.GROUP, "set")
        qos_set       = self.loader.qos_by_id("subscribe", self.GROUP, "set")

        topic_current = self.loader.publish_topic(self.GROUP, "current")
        qos_current   = self.loader.qos_by_id("publish",   self.GROUP, "current")

        topic_poses   = self.loader.publish_topic(self.GROUP, "poses")
        qos_poses     = self.loader.qos_by_id("publish",   self.GROUP, "poses")

        # Publisher / Subscriber
        self.pub_current = self.create_publisher(String,    topic_current, qos_current)
        self.pub_poses   = self.create_publisher(PoseArray, topic_poses,   qos_poses)

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

        # Re-Publisher (nur PoseArray, damit RViz die Posen behält)
        self.create_timer(1.0, self._republish_if_any)

        self.get_logger().info("✅ SprayPathManager bereit (Topics: set/current/poses)")

    # ----------------------------- helpers -----------------------------
    @staticmethod
    def _extract_name(ma: MarkerArray) -> str:
        """
        Name aus MarkerArray extrahieren:
          1) TEXT_VIEW_FACING.text (getrimmt)
          2) erstes non-empty ns
          3) sonst exakt leer
        """
        from visualization_msgs.msg import Marker  # local import, um Kreisabhängigkeit zu vermeiden

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

        # Kein Fallback-Text – exakt leer, wenn nichts geliefert wurde
        return ""

    @staticmethod
    def _prefer_linestrip(ma: MarkerArray):
        """
        Wähle den besten Pfad-Marker:
          - bevorzugt einen LINE_STRIP mit >= 2 Punkten
          - sonst den ersten Marker mit >= 2 Punkten
        """
        from visualization_msgs.msg import Marker  # local import

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

    def _republish_if_any(self) -> None:
        """
        Hält das letzte PoseArray “lebendig”, damit RViz bei Re-Subscribe
        sofort wieder etwas sieht (kein echtes latched, aber nah dran).
        """
        if self._last_pa is not None:
            self.pub_poses.publish(self._last_pa)

    # ----------------------------- handler -----------------------------
    def _on_set_spraypath(self, msg: MarkerArray) -> None:
        if msg is None or len(msg.markers) == 0:
            self.get_logger().warning("⚠️ set_spraypath: leeres MarkerArray – ignoriere")
            return

        # Frame bestimmen (nur wenn geliefert)
        first_hdr = msg.markers[0].header
        if first_hdr and (first_hdr.frame_id or "").strip():
            self._last_frame = first_hdr.frame_id.strip()

        # Name exakt übernehmen (ggf. leer)
        name = self._extract_name(msg)

        # Pfad-Marker auswählen
        m = self._prefer_linestrip(msg)
        if m is None:
            self.get_logger().warning("⚠️ set_spraypath: kein Marker mit ≥2 Punkten – ignoriere")
            return

        # PoseArray erzeugen (nur Positionen, Orientierung neutral)
        pa = PoseArray()
        pa.header.frame_id = self._last_frame
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.poses = []

        for pt in m.points:
            p = Pose()
            p.position.x = pt.x
            p.position.y = pt.y
            p.position.z = pt.z
            p.orientation.w = 1.0  # neutral (kein Drehen)
            pa.poses.append(p)

        # Publish + Merken (für Re-Publish)
        self.pub_poses.publish(pa)
        self._last_pa = pa

        # Current-Name (latched) – exakt was reinkommt (auch "" möglich)
        self._last_name = name
        self._publish_current_name(name)

        self.get_logger().info(
            f"🎯 SprayPath gesetzt: name='{name}', frame='{self._last_frame}', points={len(pa.poses)}"
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
