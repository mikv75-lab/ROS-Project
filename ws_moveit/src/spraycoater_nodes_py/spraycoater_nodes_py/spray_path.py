#!/usr/bin/env python3
# spraycoater_nodes_py/spraypath.py
from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray


def qos_default() -> QoSProfile:
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.VOLATILE
    return q


def qos_latched() -> QoSProfile:
    q = QoSProfile(depth=1)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.history = HistoryPolicy.KEEP_LAST
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL  # latched
    return q


class SprayPath(Node):
    """
    Minimaler SprayPath-Manager.

    SUB:
      /spraycoater/spraypath/set (visualization_msgs/MarkerArray)
        Erwartet mind. einen Marker mit Punkten (bevorzugt LINE_STRIP).
        Name: 1) TEXT_VIEW_FACING.text, 2) ns, 3) 'unnamed'
        Frame: header.frame_id des ersten Markers (fallback letzter bekannter Frame)

    PUB:
      /spraycoater/spraypath/current  (std_msgs/String, latched)
      /spraycoater/spraypath/poses    (geometry_msgs/PoseArray)
      /spraycoater/spraypath/markers  (visualization_msgs/Marker, LINE_STRIP)
    """

    def __init__(self):
        super().__init__("spraypath_manager")

        # Publisher
        self.pub_current = self.create_publisher(String, "/spraycoater/spraypath/current", qos_latched())
        self.pub_poses   = self.create_publisher(PoseArray, "/spraycoater/spraypath/poses", qos_default())
        self.pub_marker  = self.create_publisher(Marker, "/spraycoater/spraypath/markers", qos_default())

        # Subscriber
        self.sub_set = self.create_subscription(
            MarkerArray,
            "/spraycoater/spraypath/set",
            self._on_set_spraypath,
            qos_default(),
        )

        # interner Zustand
        self._last_frame = "scene"
        self._last_name  = ""
        self._last_pa: PoseArray | None = None
        self._last_mk: Marker | None = None

        # Republisher, damit Marker stabil bleiben (RViz verliert volatile Marker sonst gerne)
        self.create_timer(1.0, self._republish_if_any)

        self.get_logger().info("✅ SprayPathManager bereit (Topics: set/current/poses/markers)")

    # ----------------------------- helpers -----------------------------
    def _extract_name(self, ma: MarkerArray) -> str:
        # 1) TEXT_VIEW_FACING mit Text
        for m in ma.markers:
            if m.type == Marker.TEXT_VIEW_FACING and (m.text or "").strip():
                return m.text.strip()
        # 2) erstes non-empty ns
        for m in ma.markers:
            if (m.ns or "").strip():
                return m.ns.strip()
        return "unnamed"

    def _prefer_linestrip(self, ma: MarkerArray) -> Marker | None:
        # Bevorzugt LINE_STRIP; sonst erstes Element mit Punkten
        best = None
        for m in ma.markers:
            if len(m.points) >= 2:
                if m.type == Marker.LINE_STRIP:
                    return m
                if best is None:
                    best = m
        return best

    def _publish_current_name(self, name: str):
        msg = String()
        msg.data = name
        self.pub_current.publish(msg)

    def _republish_if_any(self):
        if self._last_mk is not None:
            self.pub_marker.publish(self._last_mk)
        if self._last_pa is not None:
            self.pub_poses.publish(self._last_pa)

    # ----------------------------- handler -----------------------------
    def _on_set_spraypath(self, msg: MarkerArray):
        if msg is None or len(msg.markers) == 0:
            self.get_logger().warning("⚠️ set_spraypath: leeres MarkerArray – ignoriere")
            return

        # Frame bestimmen
        frame = msg.markers[0].header.frame_id.strip() if msg.markers[0].header.frame_id else ""
        if frame:
            self._last_frame = frame

        # Name bestimmen
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
            p.position.x, p.position.y, p.position.z = pt.x, pt.y, pt.z
            p.orientation.w = 1.0
            pa.poses.append(p)

        # Marker als LINE_STRIP (für RViz)
        mk = Marker()
        mk.header.frame_id = self._last_frame
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = "spray_path"
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale.x = 0.002
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 0.0, 1.0, 0.0, 1.0
        mk.pose.orientation.w = 1.0
        mk.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in pa.poses]
        if len(mk.points) > 2:
            mk.points.append(mk.points[0])  # optional schließen

        # Publish + Merken (für Re-Publish)
        self.pub_poses.publish(pa)
        self.pub_marker.publish(mk)
        self._last_pa = pa
        self._last_mk = mk

        # Current-Name (latched)
        self._last_name = name
        self._publish_current_name(name)

        self.get_logger().info(
            f"🎯 SprayPath gesetzt: name='{name}', frame='{self._last_frame}', points={len(pa.poses)}"
        )


# ----------------------------------------------------------------------
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
