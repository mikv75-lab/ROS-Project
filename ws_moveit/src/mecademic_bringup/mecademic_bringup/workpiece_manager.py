#!/usr/bin/env python3
import os
import re
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String, Empty
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Point

from tf2_ros import Buffer, TransformListener, TransformException, TransformBroadcaster

# Für Richtungspfeile (aus Orientierung + Länge einen Pfeil zeichnen)
from tf_transformations import quaternion_matrix

CURRENT_WORKPIECE_FILE = "/root/ws_moveit/src/mecademic_bringup/config/current_workpiece.txt"

# Frames
PARENT_FRAME = "meca_mount"                   # Planungsszene / Welt für CollisionObject-Header
WS_FRAME     = "meca_pose_workspace_center"   # Hier wird Workpiece & Pfad dargestellt


class WorkpieceManager(Node):
    def __init__(self):
        super().__init__("workpiece_manager")

        # ---------- Parameter ----------
        # Optional: kleine TF-Frames je Waypoint senden
        self.publish_tf_per_waypoint = self.declare_parameter("publish_tf_per_waypoint", False).value
        self.tf_rate_hz              = float(self.declare_parameter("tf_rate_hz", 5.0).value)
        self.max_tf_waypoints        = int(self.declare_parameter("max_tf_waypoints", 100).value)

        # Marker-Optik
        self.path_line_width = float(self.declare_parameter("path_line_width", 0.003).value)
        self.point_diameter  = float(self.declare_parameter("point_diameter", 0.006).value)
        # Orientierungspfeile (als echte Pfeile mit Start/Endpunkt)
        self.arrow_len       = float(self.declare_parameter("arrow_len", 0.03).value)
        self.arrow_shaft_d   = float(self.declare_parameter("arrow_shaft_d", 0.004).value)
        self.arrow_head_d    = float(self.declare_parameter("arrow_head_d", 0.008).value)
        self.arrow_head_len  = float(self.declare_parameter("arrow_head_len", 0.012).value)
        # Achse, entlang der der Pfeil ausgerichtet wird ('x'|'y'|'z')
        self.arrow_axis      = str(self.declare_parameter("arrow_axis", "x").value).lower()

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # ---------- Publisher ----------
        self.marker_pub  = self.create_publisher(Marker, "/visualization_marker", 10)   # Workpiece-Mesh
        self.co_pub      = self.create_publisher(CollisionObject, "/collision_object", 10)
        self.path_pub    = self.create_publisher(MarkerArray, "/spray_path/markers", 10)

        # ---------- Subscriber ----------
        self.create_subscription(String, "/meca/workspace/load",   self.on_load_cmd, 10)
        self.create_subscription(Empty,  "/meca/workspace/remove", self.on_remove_cmd, 10)
        self.create_subscription(PoseArray, "/meca/spray_path/set",   self.on_path_set, 10)
        self.create_subscription(Empty,      "/meca/spray_path/clear", self.on_path_clear, 10)

        # ---------- State ----------
        self.current_workpiece = ""
        self.path_poses: list[Pose] = []
        self._prev_arrow_count = 0

        # Optionaler Timer für TF-Broadcasts
        self._tf_timer = None
        if self.publish_tf_per_waypoint:
            period = 1.0 / max(0.1, self.tf_rate_hz)
            self._tf_timer = self.create_timer(period, self._broadcast_wp_tfs_timer)

        # Persistentes Workpiece (falls vorhanden) laden
        self.load_from_file()

    # ========== Commands: Workpiece ==========
    def on_load_cmd(self, msg: String):
        path = msg.data.strip()
        self.load_workpiece(path)

    def on_remove_cmd(self, _):
        self.delete_workpiece()

    # ========== Commands: Spray Path ==========
    def on_path_set(self, msg: PoseArray):
        # Erwartet Pfad in WS_FRAME
        if msg.header.frame_id and msg.header.frame_id != WS_FRAME:
            self.get_logger().warn(
                f"⚠️ PoseArray header.frame_id='{msg.header.frame_id}', erwartet '{WS_FRAME}'. "
                f"Marker werden in '{WS_FRAME}' gezeichnet."
            )
        self.path_poses = list(msg.poses)
        self._publish_path_markers()
        self.get_logger().info(f"🧭 Spray-Pfad gesetzt/aktualisiert: {len(self.path_poses)} Wegpunkte")

    def on_path_clear(self, _):
        self._clear_path_markers()
        self.path_poses = []
        self.get_logger().warning("🧹 Spray-Pfad gelöscht")

    # ========== API: Workpiece ==========
    def load_workpiece(self, stl_path: str) -> bool:
        if not os.path.exists(stl_path):
            self.get_logger().error(f"❌ STL existiert nicht: {stl_path}")
            return False

        self.current_workpiece = stl_path
        self.save_to_file()

        pose = self._lookup_ws_pose()
        if pose is None:
            self.get_logger().warning(
                f"⚠️ Kein TF {PARENT_FRAME} -> {WS_FRAME} gefunden. "
                f"Spawnen wird übersprungen – bitte TF prüfen und erneut laden."
            )
            return False

        pos, quat = pose  # ((x,y,z),(qx,qy,qz,qw))
        self.publish_marker(pos, quat)

        h_m, r_m = self._dims_from_filename_or_default(stl_path)
        self.publish_cylinder_collision(height=h_m, radius=r_m, pos=pos, quat=quat)

        self.get_logger().info(f"✅ Workpiece geladen: {stl_path}")
        return True

    def delete_workpiece(self):
        if not self.current_workpiece:
            self.get_logger().warning("⚠ Kein Workpiece gesetzt.")
        self.current_workpiece = ""
        self.save_to_file()
        self.delete_marker()
        self.clear_collision_object()
        self.get_logger().warning("🗑 Workpiece entfernt!")

    # ========== Persistenz ==========
    def save_to_file(self):
        os.makedirs(os.path.dirname(CURRENT_WORKPIECE_FILE), exist_ok=True)
        with open(CURRENT_WORKPIECE_FILE, "w") as f:
            f.write(self.current_workpiece)

    def load_from_file(self):
        if os.path.exists(CURRENT_WORKPIECE_FILE):
            with open(CURRENT_WORKPIECE_FILE, "r") as f:
                path = f.read().strip()
            if path:
                self.current_workpiece = path
                self.get_logger().info(f"🔁 Lade Workpiece beim Start: {path}")
                pose = self._lookup_ws_pose()
                if pose is None:
                    self.get_logger().warning(
                        f"⚠️ Kein TF {PARENT_FRAME} -> {WS_FRAME} gefunden – "
                        f"Marker/CollisionObject werden noch nicht publiziert."
                    )
                    return
                pos, quat = pose
                self.publish_marker(pos, quat)
                h_m, r_m = self._dims_from_filename_or_default(path)
                self.publish_cylinder_collision(height=h_m, radius=r_m, pos=pos, quat=quat)

    # ========== TF Lookup ==========
    def _lookup_ws_pose(self):
        """Liest den Transform PARENT_FRAME -> WS_FRAME und gibt ((x,y,z),(qx,qy,qz,qw)) zurück."""
        try:
            tf = self.tf_buffer.lookup_transform(
                PARENT_FRAME, WS_FRAME, Time(), timeout=Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().debug(f"TF-Lookup fehlgeschlagen: {e}")
            return None

        pos = (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        )
        quat = (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        )
        return pos, quat

    # ========== RViz Marker: Workpiece ==========
    def publish_marker(self, pos, quat):
        """Mesh-Marker im Zentrum von WS_FRAME (Transform bereits in pos/quat enthalten)."""
        if not self.current_workpiece:
            return
        m = Marker()
        m.header.frame_id = PARENT_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workpiece"
        m.id = 1
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = f"file://{self.current_workpiece}"
        m.pose.position.x, m.pose.position.y, m.pose.position.z = pos
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r = m.color.g = m.color.b = 0.8
        m.color.a = 1.0
        self.marker_pub.publish(m)
        self.get_logger().info("📦 RViz Marker publiziert (MESH @ workspace_center)")

    def delete_marker(self):
        m = Marker()
        m.header.frame_id = PARENT_FRAME
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "workpiece"
        m.id = 1
        m.action = Marker.DELETE
        self.marker_pub.publish(m)

    # ========== CollisionObject (Zylinder) ==========
    def publish_cylinder_collision(self, height: float, radius: float, pos, quat):
        """
        Publiziert einen Zylinder, dessen Mittelpunkt GENAU im workspace_center liegt.
        (SolidPrimitive.CYLINDER ist definitionsgemäß am Mittelpunkt orientiert.)
        """
        # gleiche ID wiederverwenden -> ggf. vorheriges Objekt überschreiben
        # sicherheitshalber kurz Remove vor Add schicken
        self.clear_collision_object()

        co = CollisionObject()
        co.header.frame_id = PARENT_FRAME
        co.id = "workpiece_cylinder"

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        prim.dimensions = [height, radius]  # [height, radius]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pos
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        self.co_pub.publish(co)
        self.get_logger().info(
            f"🧱 CollisionObject @ workspace_center (CYL h={height:.4f} m, r={radius:.4f} m)"
        )

    def clear_collision_object(self):
        co = CollisionObject()
        co.header.frame_id = PARENT_FRAME
        co.id = "workpiece_cylinder"
        co.operation = CollisionObject.REMOVE
        self.co_pub.publish(co)

    # ========== Spray-Path: Marker/TF ==========
    def _publish_path_markers(self):
        """Zeichnet LINE_STRIP, SPHERE_LIST und orientierte ARROWs je Waypoint."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # 1) LINE_STRIP (Pfadlinie)
        line = Marker()
        line.header.frame_id = WS_FRAME
        line.header.stamp = now
        line.ns = "spray_path"
        line.id = 1
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.path_line_width
        line.color.r, line.color.g, line.color.b, line.color.a = (0.1, 0.6, 1.0, 1.0)
        line.points = []
        for p in self.path_poses:
            q = Point(x=p.position.x, y=p.position.y, z=p.position.z)
            line.points.append(q)
        ma.markers.append(line)

        # 2) SPHERE_LIST (Wegpunkte)
        pts = Marker()
        pts.header.frame_id = WS_FRAME
        pts.header.stamp = now
        pts.ns = "spray_path"
        pts.id = 2
        pts.type = Marker.SPHERE_LIST
        pts.action = Marker.ADD
        pts.scale.x = pts.scale.y = pts.scale.z = self.point_diameter
        pts.color.r, pts.col
