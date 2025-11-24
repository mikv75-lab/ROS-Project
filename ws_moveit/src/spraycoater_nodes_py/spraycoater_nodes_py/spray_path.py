# spraycoater_nodes_py/spray_path.py
# -*- coding: utf-8 -*-
#!/usr/bin/env python3
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import MarkerArray, Marker

from spraycoater_nodes_py.utils.config_hub import topics, frames


class SprayPath(Node):
    """
    SprayPath-Manager.

    SUB:
      - spray_path.set              (MarkerArray)
        → Kommando vom PyQt-Editor (fertiger Pfad als MarkerArray, Sollpfad)
      - spray_path.executed_poses_in (PoseArray)
        → gefahrene Posen vom ProcessTab / Recorder (Istpfad)

    PUB (alle TRANSIENT_LOCAL / latched):
      - spray_path.poses            (PoseArray): Punkte des Sollpfads als neutrale Posen
      - spray_path.markers          (MarkerArray): aktuelles MarkerArray für RViz (Sollpfad)
      - spray_path.current          (String): aktueller Pfad-Name (optional für UI)
      - spray_path.executed_poses   (PoseArray): gefahrene Posen (durchgereicht)
      - spray_path.executed_markers (MarkerArray): Marker für den Istpfad (z.B. andere Farbe)

    Semantik:
      - JEDER set-Aufruf ersetzt den aktuellen Sollpfad vollständig.
      - JEDER executed_poses_in-Aufruf ersetzt den aktuellen Istpfad vollständig.
      - Kein Timer-Republish; Late Joiner bekommen letzten Stand über QoS.
    """

    GROUP = "spray_path"

    def __init__(self):
        super().__init__("spraypath_manager")

        # Config-Hub
        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve

        # Topics & QoS aus config_hub für SUB
        topic_set = self.loader.subscribe_topic(self.GROUP, "set")
        qos_set = self.loader.qos_by_id("subscribe", self.GROUP, "set")

        # ⚠️ ID muss zu topics.yaml passen:
        #   subscribe:
        #     - id: executed_poses_in
        #       name: /spraycoater/spray_path/executed_poses_in
        #       type: geometry_msgs/msg/PoseArray
        topic_exec_in = self.loader.subscribe_topic(self.GROUP, "executed_poses_in")
        qos_exec_in = self.loader.qos_by_id("subscribe", self.GROUP, "executed_poses_in")

        # Latched QoS für unsere Publisher (wie bei Scene)
        latched_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        topic_current = self.loader.publish_topic(self.GROUP, "current")
        topic_poses = self.loader.publish_topic(self.GROUP, "poses")
        topic_markers = self.loader.publish_topic(self.GROUP, "markers")
        # Node → UI/RViz: gefahrene Posen (Istpfad)
        topic_exec_poses = self.loader.publish_topic(self.GROUP, "executed_poses")
        topic_exec_markers = self.loader.publish_topic(self.GROUP, "executed_markers")

        # Publisher / Subscriber
        self.pub_current = self.create_publisher(String, topic_current, latched_qos)
        self.pub_poses = self.create_publisher(PoseArray, topic_poses, latched_qos)
        self.pub_markers = self.create_publisher(MarkerArray, topic_markers, latched_qos)

        self.pub_exec_poses = self.create_publisher(PoseArray, topic_exec_poses, latched_qos)
        self.pub_exec_markers = self.create_publisher(MarkerArray, topic_exec_markers, latched_qos)

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

        # interner Zustand (für evtl. Debug/inspektieren)
        self._last_frame = self.frames.get("scene", "scene")
        self._last_name: str = ""
        self._last_pa: PoseArray | None = None
        self._last_markers: MarkerArray | None = None

        self._last_exec_frame = self._last_frame
        self._last_exec_pa: PoseArray | None = None
        self._last_exec_markers: MarkerArray | None = None

        self.get_logger().info(
            "✅ SprayPathManager bereit: "
            "/set → /poses + /markers (+ /current), "
            "executed_poses_in → executed_poses + executed_markers, "
            "TRANSIENT_LOCAL (latched), kein 1 Hz-Republish."
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

    # ----------------------------- handler: Sollpfad -----------------------------

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

        # Zustand merken
        self._last_pa = pa
        self._last_markers = out_ma
        self._last_name = name

        # ONE SHOT Publish (latched)
        self.pub_poses.publish(pa)
        self.pub_markers.publish(out_ma)
        self._publish_current_name(name)

        self.get_logger().info(
            f"🎯 SprayPath gesetzt (Soll): name='{name}', frame='{frame}', "
            f"points={len(pa.poses)}, markers={len(out_ma.markers)}"
        )

    # ----------------------------- handler: Istpfad -----------------------------

    def _on_executed_poses(self, msg: PoseArray) -> None:
        if msg is None or len(msg.poses) == 0:
            self.get_logger().warning("⚠️ spray_path.executed_poses_in: leeres PoseArray – ignoriere")
            return

        frame = (msg.header.frame_id or "").strip() or self._last_exec_frame
        self._last_exec_frame = frame

        now = self.get_clock().now().to_msg()

        # PoseArray ggf. mit aktualisiertem Header weiterreichen
        pa = PoseArray()
        pa.header.frame_id = frame
        pa.header.stamp = now
        pa.poses = list(msg.poses)

        if len(pa.poses) < 2:
            self.get_logger().warning(
                "⚠️ spray_path.executed_poses_in: <2 Posen, Marker-LINE_STRIP wäre degeneriert – ignoriere"
            )
            return

        # Marker als LINE_STRIP aus den Posen bauen
        m = Marker()
        m.header.frame_id = frame
        m.header.stamp = now
        m.ns = "executed_path"
        m.id = 1000
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD

        # einfache Skalierung, damit Linie sichtbar ist
        m.scale.x = 0.002  # Liniendicke
        m.scale.y = 0.0
        m.scale.z = 0.0

        # Farbe (z.B. abgesetzt vom Rezeptpfad – hier grün)
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

        # ONE SHOT Publish (latched)
        self.pub_exec_poses.publish(pa)
        self.pub_exec_markers.publish(out_ma)

        self.get_logger().info(
            f"✅ Executed SprayPath gesetzt (Ist): frame='{frame}', "
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
