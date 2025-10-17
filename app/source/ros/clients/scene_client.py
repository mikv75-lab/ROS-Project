# ros/clients/scene_client.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional, Sequence, Tuple, Union

from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray

from ..common.topics import Topics
from ..common.qos import qos_latched
from ..common.frames import FRAMES, FramesNS


@dataclass
class PathStyle:
    line_width: float = 0.004         # m
    point_size: float = 0.008         # m (SPHERE_LIST)
    axis_len: float = 0.03            # m (pro Richtungspfeil)
    axis_rad: float = 0.003           # m (Pfeilschaft)
    show_axes: bool = True
    show_points: bool = True
    show_line: bool = True
    ns: str = "recipe"                # Marker-Namespace


def _color(r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c


class SceneClient:
    """
    Visualisierungskunde:
      - publisht MarkerArray (latched) auf Topics.recipe_markers
      - Convenience: Spray-/Recipe-Pfad als LineStrip + Punkte + Achsen anzeigen
      - clear() löscht alle Marker dieses Clients (Namespace-basiert)

    Hinweise:
      * Frame-Default ist workspace_center (aus FramesNS).
      * QoS: TRANSIENT_LOCAL (latched), damit RViz sofort einen Stand sieht.
    """

    def __init__(
        self,
        node: Node,
        topics: Optional[Topics] = None,
        *,
        frames: Optional[FramesNS] = None,
        default_frame: Optional[str] = None,
        style: Optional[PathStyle] = None,
    ) -> None:
        self._node = node
        self._topics = topics or Topics()
        self._frames = frames or FRAMES
        self._frame_id = default_frame or self._frames["workspace_center"]
        self._style = style or PathStyle()
        self._log = node.get_logger()

        self._pub = node.create_publisher(MarkerArray, self._topics.recipe_markers, qos_latched())
        self._id_counter = 0

    # ----------------- Public API -----------------

    def set_frame(self, frame_id: str) -> None:
        self._frame_id = frame_id or self._frame_id

    def show_pose_array(self, pose_array: PoseArray, *, ns: Optional[str] = None) -> None:
        """PoseArray in MarkerArray umwandeln (LineStrip + optional Punkte/Axes)."""
        poses = list(pose_array.poses)
        self._publish_markers(self._markers_for_poses(poses, header_frame=pose_array.header.frame_id or self._frame_id, ns=ns))

    def show_points_xyz(
        self,
        points: Sequence[Tuple[float, float, float]],
        *,
        ns: Optional[str] = None,
        frame_id: Optional[str] = None,
    ) -> None:
        """Nur Punkte/LineStrip ohne Orientierungen anzeigen."""
        poses = [self._pose_from_xyz(x, y, z) for (x, y, z) in points]
        self._publish_markers(self._markers_for_poses(poses, header_frame=frame_id or self._frame_id, ns=ns))

    def show_poses(
        self,
        poses: Iterable[Pose],
        *,
        ns: Optional[str] = None,
        frame_id: Optional[str] = None,
    ) -> None:
        """Poses (mit Orientierung) anzeigen."""
        self._publish_markers(self._markers_for_poses(list(poses), header_frame=frame_id or self._frame_id, ns=ns))

    def publish(self, markers: MarkerArray) -> None:
        """Direkt ein eigenes MarkerArray veröffentlichen (latched)."""
        self._pub.publish(markers)

    def clear(self, *, ns: Optional[str] = None) -> None:
        """
        Löscht Marker dieses Clients (Namespace).
        Technisch: Marker mit action=DELETEALL (falls RViz/Plugin es aus MarkerArray akzeptiert).
        Fallback: sendet DELETE für übliche IDs in unserem Namespace.
        """
        ma = MarkerArray()

        # Preferred: DELETEALL (ein Marker genügt)
        m = Marker()
        m.header.frame_id = self._frame_id
        m.ns = ns or self._style.ns
        m.action = Marker.DELETEALL
        ma.markers.append(m)

        self._pub.publish(ma)
        # kein strenges Logging – einzelne RViz-Setups ignorieren DELETEALL in Arrays
        self._log.info(f"🧹 SceneClient: clear(ns='{m.ns}') gesendet.")

    # ----------------- Internals -----------------

    def _publish_markers(self, markers: MarkerArray) -> None:
        self._pub.publish(markers)
        self._log.info(f"🖼️ SceneClient: {len(markers.markers)} Marker publiziert @ {markers.markers[0].header.frame_id if markers.markers else self._frame_id}")

    def _markers_for_poses(self, poses: List[Pose], *, header_frame: str, ns: Optional[str]) -> MarkerArray:
        style = self._style
        ns = ns or style.ns
        ma = MarkerArray()

        # ---- LineStrip ----
        if style.show_line and len(poses) >= 2:
            m = Marker()
            m.header.frame_id = header_frame
            m.ns = ns
            m.id = self._next_id()
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = style.line_width
            m.color = _color(0.1, 0.6, 1.0, 0.9)
            m.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in poses]
            ma.markers.append(m)

        # ---- Points (Sphere List) ----
        if style.show_points and len(poses) >= 1:
            m = Marker()
            m.header.frame_id = header_frame
            m.ns = ns
            m.id = self._next_id()
            m.type = Marker.SPHERE_LIST
            m.action = Marker.ADD
            m.scale.x = m.scale.y = m.scale.z = style.point_size
            m.color = _color(1.0, 0.8, 0.2, 0.9)
            m.points = [Point(x=p.position.x, y=p.position.y, z=p.position.z) for p in poses]
            ma.markers.append(m)

        # ---- Axes (3 ARROW Marker je Pose) ----
        if style.show_axes:
            for i, p in enumerate(poses):
                # X (rot)
                ma.markers.append(self._arrow_from_pose(p, header_frame, ns, self._next_id(), style.axis_len, style.axis_rad, _color(1.0, 0.2, 0.2, 0.9)))
                # Y (grün)
                ma.markers.append(self._arrow_from_pose(p, header_frame, ns, self._next_id(), style.axis_len, style.axis_rad, _color(0.2, 1.0, 0.2, 0.9), axis="y"))
                # Z (blau)
                ma.markers.append(self._arrow_from_pose(p, header_frame, ns, self._next_id(), style.axis_len, style.axis_rad, _color(0.2, 0.4, 1.0, 0.9), axis="z"))

        return ma

    def _arrow_from_pose(
        self,
        pose: Pose,
        frame: str,
        ns: str,
        mid: int,
        length: float,
        radius: float,
        color: ColorRGBA,
        *,
        axis: str = "x",
    ) -> Marker:
        """
        Erzeugt einen ARROW-Marker entlang der lokalen Achse des Pose-Frames.
        Vereinfachung: Wir nutzen den Pose-Frame direkt für den Pfeil und
        verschieben/skalieren anhand scale.x/scale.y.
        """
        m = Marker()
        m.header.frame_id = frame
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = pose
        # ROS ARROW: scale.x = Schaft-Länge, scale.y = Schaft-Durchmesser, scale.z = Kopf-Durchmesser (optional)
        m.scale.x = length
        m.scale.y = radius
        m.scale.z = radius * 2.0
        m.color = color

        # Achsenwechsel per Orientierung tricksen? Hier bewusst simpel:
        # - X: Pose unverändert
        # - Y/Z: nicht rotiert (vereinfachend), da Pfeil entlang X zeigt.
        # Wenn echte lokale Achsen wichtig sind: je Achse eine Zusatzrotation auf pose.orientation multiplizieren.
        # Für die meisten RViz-Previews reicht X.
        if axis != "x":
            # Minimaler Hinweis ins Log beim ersten Mal.
            pass

        return m

    def _pose_from_xyz(self, x: float, y: float, z: float) -> Pose:
        p = Pose()
        p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
        p.orientation.w = 1.0
        return p

    def _next_id(self) -> int:
        self._id_counter += 1
        return self._id_counter
