#!/usr/bin/env python3
# mecademic_bringup/mecademic_bringup/scene_manager.py
from __future__ import annotations

import os
import math
import re
from typing import Optional, Tuple, List

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from std_msgs.msg import String, Empty
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener, TransformException, StaticTransformBroadcaster


# ---------------- Defaults / Frames / Topics ----------------
DEFAULT_PARENT_FRAME    = "meca_mount"
DEFAULT_MOUNT_FRAME     = "workspace_mount"
DEFAULT_WORKSPACE_FRAME = "meca_pose_workspace_center"

TOPIC_MOUNT_LOAD        = "meca/mount/load"          # String(path.stl)
TOPIC_WORKPIECE_LOAD    = "meca/workspace/load"      # String(path.stl)
TOPIC_WORKPIECE_CLEAR   = "meca/workspace/remove"    # Empty()
TOPIC_SPRAY_SET         = "meca/spray_path/set"      # PoseArray (frame_id optional; wir zeichnen in workspace_frame)
TOPIC_SPRAY_CLEAR       = "meca/spray_path/clear"    # Empty()

# Inferenz Substrat-Zylinder
DEFAULT_WAFER_THICKNESS_M = 0.0007
DEFAULT_FALLBACK_R_M      = 0.05
DEFAULT_FALLBACK_H_M      = 0.005


def rpy_deg_to_quat(r: float, p: float, y: float) -> Tuple[float, float, float, float]:
    rx, ry, rz = map(math.radians, (r, p, y))
    cr, sr = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cp, sp = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cy, sy = math.cos(rz * 0.5), math.sin(rz * 0.5)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    return (qx/n, qy/n, qz/n, qw/n)


class SceneManager(Node):
    """
    Ein Node für die komplette Workspace-Szene:
      - Mount laden/ersetzen (TF parent->mount kann extern kommen; Entfernen nicht vorgesehen)
      - Static TF mount->workspace_center (z-Offset)
      - Substrat laden/entfernen (Mesh + Kollision als Zylinder)
      - Environment/Cage im Init einmalig laden (Anzeige + optional Collision-Fallback)
      - Spray Path visualisieren (LINE_STRIP + SPHERE_LIST + TF-Achsen an jedem Punkt, im workspace_frame)
    """

    def __init__(self):
        super().__init__("scene_manager")

        # ---------- Parameters ----------
        # Frames
        self.declare_parameter("parent_frame", DEFAULT_PARENT_FRAME)
        self.declare_parameter("mount_frame", DEFAULT_MOUNT_FRAME)
        self.declare_parameter("workspace_frame", DEFAULT_WORKSPACE_FRAME)

        # workspace_center Offset relativ zum Mount (mm)
        self.declare_parameter("workspace_z_offset_mm", 50.0)

        # Mount-Handling
        self.declare_parameter("mount_use_tf", True)       # TRUE: TF parent->mount wird gelesen
        self.declare_parameter("mount_xyz", [0.2, 0.0, 0.0])
        self.declare_parameter("mount_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("mount_initial_mesh", "")   # optional: beim Start direkt laden

        # Environment / Cage (Init)
        self.declare_parameter("environment_enable", True)
        self.declare_parameter("environment_mesh", "")
        self.declare_parameter("environment_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("environment_rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("environment_collision_box_fallback", [1.2, 1.2, 1.2])  # 0/0/0 => aus

        # Farben & Größen
        self.declare_parameter("color_mount_rgba", [0.7, 0.7, 0.7, 1.0])
        self.declare_parameter("color_workpiece_rgba", [0.8, 0.8, 0.8, 1.0])
        self.declare_parameter("spray_line_color_rgba", [0.0, 0.3, 1.0, 1.0])  # fest: blau
        self.declare_parameter("spray_point_diameter", 0.006)  # m
        self.declare_parameter("spray_line_width", 0.003)      # m

        # Spray-Achsen (TF-Style)
        self.declare_parameter("spray_show_axes", True)
        self.declare_parameter("spray_axis_length", 0.03)      # m (30 mm)
        self.declare_parameter("spray_axis_thickness", 0.003)  # m
        self.declare_parameter("spray_tcp_axis", "z")          # "x" oder "z" (nur Info; Achsen werden immer alle drei gezeichnet)

        # ---------- Resolve ----------
        self.parent_frame    = self.get_parameter("parent_frame").get_parameter_value().string_value
        self.mount_frame     = self.get_parameter("mount_frame").get_parameter_value().string_value
        self.workspace_frame = self.get_parameter("workspace_frame").get_parameter_value().string_value
        self.ws_z_off_m      = float(self.get_parameter("workspace_z_offset_mm").value) / 1000.0

        self.mount_use_tf    = bool(self.get_parameter("mount_use_tf").value)
        self.mount_xyz       = [float(x) for x in self.get_parameter("mount_xyz").value]
        self.mount_rpy_deg   = [float(x) for x in self.get_parameter("mount_rpy_deg").value]
        self.mount_initial   = self.get_parameter("mount_initial_mesh").get_parameter_value().string_value

        self.env_enable      = bool(self.get_parameter("environment_enable").value)
        self.env_mesh        = self.get_parameter("environment_mesh").get_parameter_value().string_value
        self.env_xyz         = [float(x) for x in self.get_parameter("environment_xyz").value]
        self.env_rpy_deg     = [float(x) for x in self.get_parameter("environment_rpy_deg").value]
        self.env_box_fb      = [float(x) for x in self.get_parameter("environment_collision_box_fallback").value]

        self.color_mount     = [float(x) for x in self.get_parameter("color_mount_rgba").value]
        self.color_workpiece = [float(x) for x in self.get_parameter("color_workpiece_rgba").value]

        self.spray_line_rgba = [float(x) for x in self.get_parameter("spray_line_color_rgba").value]
        self.spray_point_d   = float(self.get_parameter("spray_point_diameter").value)
        self.spray_line_w    = float(self.get_parameter("spray_line_width").value)

        self.spray_show_axes = bool(self.get_parameter("spray_show_axes").value)
        self.spray_axis_len  = float(self.get_parameter("spray_axis_length").value)
        self.spray_axis_th   = float(self.get_parameter("spray_axis_thickness").value)
        self.spray_tcp_axis  = str(self.get_parameter("spray_tcp_axis").value).lower()

        # ---------- Infra ----------
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.stf         = StaticTransformBroadcaster(self)

        self.marker_pub  = self.create_publisher(Marker, "visualization_marker", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "spray_path/markers", 10)
        self.co_pub      = self.create_publisher(CollisionObject, "collision_object", 10)

        self.ps_client   = self.create_client(ApplyPlanningScene, "apply_planning_scene")

        # ---------- Subscriptions ----------
        self.create_subscription(String, TOPIC_MOUNT_LOAD,        self._on_mount_load, 10)
        self.create_subscription(String, TOPIC_WORKPIECE_LOAD,    self._on_workpiece_load, 10)
        self.create_subscription(Empty,  TOPIC_WORKPIECE_CLEAR,   self._on_workpiece_remove, 10)
        self.create_subscription(PoseArray, TOPIC_SPRAY_SET,      self._on_spray_set, 10)
        self.create_subscription(Empty,     TOPIC_SPRAY_CLEAR,    self._on_spray_clear, 10)

        # ---------- State ----------
        self._current_mount_mesh: str = ""
        self._current_workpiece_mesh: str = ""
        self._spray_count_last_axes: int = 0

        # ---------- Boot ----------
        self._boot()

    # ---------------- Boot sequence ----------------
    def _boot(self):
    self.get_logger().info("SceneManager init …")

    # (1) Mount-TF
    if not self.mount_use_tf:
        self._publish_static_tf(
            parent=self.parent_frame, child=self.mount_frame,
            xyz=self.mount_xyz, rpy_deg=self.mount_rpy_deg
        )
        self.get_logger().info(f"Static TF gesetzt: {self.parent_frame} -> {self.mount_frame}")

    # (2) workspace_center relativ zum Mount
    self._publish_static_tf(
        parent=self.mount_frame, child=self.workspace_frame,
        xyz=[0.0, 0.0, self.ws_z_off_m], rpy_deg=[0.0, 0.0, 0.0]
    )
    self.get_logger().info(
        f"Static TF gesetzt: {self.mount_frame} -> {self.workspace_frame} (z={self.ws_z_off_m:.3f} m)"
    )

    # (3) Environment (Cage) – Mesh + Collision (kein Fallback)
    if self.env_enable and self.env_mesh:
        self._spawn_mesh_marker(
            ns="environment", mesh_path=self.env_mesh, parent=self.parent_frame,
            xyz=self.env_xyz, rpy_deg=self.env_rpy_deg, rgba=[0.5, 0.7, 0.9, 0.35], mid=1001
        )
        self._publish_mesh_collision_from_stl(
            object_id="environment_cage",
            parent=self.parent_frame,
            mesh_path=self.env_mesh,
            xyz=self.env_xyz,
            rpy_deg=self.env_rpy_deg
        )
        self.get_logger().info(f"Environment geladen (Mesh+Collision): {self.env_mesh}")
    else:
        self.get_logger().info("Environment: disabled oder kein Mesh angegeben.")

    # (4) Optional initialer Mount
    if self.mount_initial:
        self._mount_set(self.mount_initial)
        
    # ---------------- Subscriptions ----------------
    def _on_mount_load(self, msg: String):
        path = msg.data.strip()
        if not path:
            self.get_logger().error("Mount-Load: leerer Pfad.")
            return
        if not os.path.isfile(path):
            self.get_logger().error(f"Mount-Load: Datei existiert nicht: {path}")
            return
        self._mount_set(path)

    def _on_workpiece_load(self, msg: String):
    path = msg.data.strip()
    if not path:
        self.get_logger().error("Workpiece-Load: leerer Pfad.")
        return
    if not os.path.isfile(path):
        self.get_logger().error(f"Workpiece-Load: Datei existiert nicht: {path}")
        return

    self._current_workpiece_mesh = path

    pose = self._lookup_pose(self.parent_frame, self.workspace_frame)
    if pose is None:
        self.get_logger().warning(
            f"Kein TF {self.parent_frame} -> {self.workspace_frame} gefunden. Abbruch."
        )
        return
    pos, quat = pose

    # RViz Mesh
    self._spawn_mesh_marker(
        ns="workpiece", mesh_path=path, parent=self.parent_frame,
        xyz=[pos[0], pos[1], pos[2]], quat=quat,
        rgba=self.color_workpiece, mid=2001
    )

    # Kollision: echtes Mesh (kein Zylinder mehr)
    self._publish_mesh_collision_from_stl(
        object_id="workpiece_mesh",
        parent=self.parent_frame,
        mesh_path=path,
        xyz=[pos[0], pos[1], pos[2]],
        quat=quat
    )

    self.get_logger().info(f"Workpiece geladen (Mesh+Collision): {path}")


    def _on_workpiece_remove(self, _msg: Empty):
        # RViz-Mesh löschen
        self._delete_marker(ns="workpiece", mid=2001, parent=self.parent_frame)
        # Collision-Mesh löschen (alte Zylinder-ID zur Sicherheit mit entfernen)
        self._remove_collision_object("workpiece_mesh", self.parent_frame)
        self._remove_collision_object("workpiece_cylinder", self.parent_frame)
        self._current_workpiece_mesh = ""
        self.get_logger().info("Workpiece entfernt (Mesh & Collision).")


    def _on_spray_set(self, msg: PoseArray):
        # Wir visualisieren immer in workspace_frame (Pfadkoordinaten sind relativ dazu gedacht).
        # Header.frame_id wird nur geprüft und geloggt.
        if msg.header.frame_id and msg.header.frame_id != self.workspace_frame:
            self.get_logger().warn(
                f"PoseArray frame_id='{msg.header.frame_id}', erwartet '{self.workspace_frame}'. Zeichne trotzdem in '{self.workspace_frame}'."
            )
        poses: List[Pose] = list(msg.poses)
        self._publish_spray_markers(poses)

    def _on_spray_clear(self, _msg: Empty):
        self._clear_spray_markers()

    # ---------------- Mount handling ----------------
    def _mount_set(self, mesh_path: str):
    self._current_mount_mesh = mesh_path

    # Pose des Mounts (aus TF oder Param)
    if self.mount_use_tf:
        pose = self._lookup_pose(self.parent_frame, self.mount_frame)
        if pose is None:
            self.get_logger().warning(
                f"Kein TF {self.parent_frame} -> {self.mount_frame} gefunden. "
                f"Setze ersatzweise Static TF aus Parametern."
            )
            self._publish_static_tf(self.parent_frame, self.mount_frame, self.mount_xyz, self.mount_rpy_deg)
            pose = (tuple(self.mount_xyz), rpy_deg_to_quat(*self.mount_rpy_deg))
    else:
        pose = (tuple(self.mount_xyz), rpy_deg_to_quat(*self.mount_rpy_deg))

    pos, quat = pose

    # Mount-Mesh (RViz)
    self._spawn_mesh_marker(
        ns="mount", mesh_path=mesh_path, parent=self.parent_frame,
        xyz=[pos[0], pos[1], pos[2]], quat=quat, rgba=self.color_mount, mid=1000
    )

    # Mount-Collision (echtes Mesh)
    self._publish_mesh_collision_from_stl(
        object_id="mount_mesh",
        parent=self.parent_frame,
        mesh_path=mesh_path,
        xyz=[pos[0], pos[1], pos[2]],
        quat=quat
    )

    # workspace_center (Offset bleibt gleich)
    self._publish_static_tf(
        parent=self.mount_frame, child=self.workspace_frame,
        xyz=[0.0, 0.0, self.ws_z_off_m], rpy_deg=[0.0, 0.0, 0.0]
    )

    self.get_logger().info(f"Mount gesetzt (Mesh+Collision): {mesh_path}")
    # Spray-Pfad NICHT löschen (Anforderung).

    # ---------------- Spray markers ----------------
    def _clear_spray_markers(self):
        ma = MarkerArray()

        # Hauptmarker (Linie, Punkte)
        for mid in (3001, 3002):
            m = Marker()
            m.header.frame_id = self.workspace_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "spray_path"
            m.id = mid
            m.action = Marker.DELETE
            ma.markers.append(m)

        # Bisher gezeichnete Achsen löschen
        for i in range(self._spray_count_last_axes):
            for axis in range(3):  # X,Y,Z
                m = Marker()
                m.header.frame_id = self.workspace_frame
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = "spray_axes"
                m.id = 400000 + i*3 + axis
                m.action = Marker.DELETE
                ma.markers.append(m)

        self.markers_pub.publish(ma)
        self._spray_count_last_axes = 0
        self.get_logger().info("Spray-Pfad Visualisierung gelöscht.")


    def _publish_spray_markers(self, poses: List[Pose]):
        # Bestehende Spray-Marker ersetzen (sauber löschen)
        self._clear_spray_markers()

        if not poses:
            return

        now = self.get_clock().now().to_msg()
        ma = MarkerArray()

        # 1) LINE_STRIP (blau)
        line = Marker()
        line.header.frame_id = self.workspace_frame
        line.header.stamp = now
        line.ns = "spray_path"
        line.id = 3001
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = self.spray_line_w
        line.color.r, line.color.g, line.color.b, line.color.a = self.spray_line_rgba
        line.points = [geometry_msgs__Point(p.position.x, p.position.y, p.position.z) for p in poses]
        ma.markers.append(line)

        # 2) SPHERE_LIST (gleichfarbig)
        pts = Marker()
        pts.header.frame_id = self.workspace_frame
        pts.header.stamp = now
        pts.ns = "spray_path"
        pts.id = 3002
        pts.type = Marker.SPHERE_LIST
        pts.action = Marker.ADD
        pts.scale.x = pts.scale.y = pts.scale.z = self.spray_point_d
        pts.color.r, pts.color.g, pts.color.b, pts.color.a = self.spray_line_rgba
        pts.points = [geometry_msgs__Point(p.position.x, p.position.y, p.position.z) for p in poses]
        ma.markers.append(pts)

        # 3) TF-Style Achsen (X rot, Y grün, Z blau)
        if self.spray_show_axes and self.spray_axis_len > 0.0 and self.spray_axis_th > 0.0:
            for i, p in enumerate(poses):
                q = (p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
                pos = (p.position.x, p.position.y, p.position.z)
                ma.markers.append(self._axis_arrow_marker(
                    ns="spray_axes", mid=400000 + i*3 + 0,
                    base_frame=self.workspace_frame, stamp=now,
                    pos=pos, quat=q, axis='x', length=self.spray_axis_len, thickness=self.spray_axis_th,
                    color=(1.0, 0.0, 0.0, 1.0)
                ))
                ma.markers.append(self._axis_arrow_marker(
                    ns="spray_axes", mid=400000 + i*3 + 1,
                    base_frame=self.workspace_frame, stamp=now,
                    pos=pos, quat=q, axis='y', length=self.spray_axis_len, thickness=self.spray_axis_th,
                    color=(0.0, 1.0, 0.0, 1.0)
                ))
                ma.markers.append(self._axis_arrow_marker(
                    ns="spray_axes", mid=400000 + i*3 + 2,
                    base_frame=self.workspace_frame, stamp=now,
                    pos=pos, quat=q, axis='z', length=self.spray_axis_len, thickness=self.spray_axis_th,
                    color=(0.0, 0.3, 1.0, 1.0)
                ))
            self._spray_count_last_axes = len(poses)

        self.markers_pub.publish(ma)
        self.get_logger().info(f"Spray-Pfad gezeichnet: {len(poses)} Wegpunkte (mit Achsen={self.spray_show_axes}).")


    # Helper für Achsen (Marker.ARROW)
    def _axis_arrow_marker(
        self, *, ns: str, mid: int, base_frame: str, stamp,
        pos: Tuple[float,float,float], quat: Tuple[float,float,float,float],
        axis: str, length: float, thickness: float, color: Tuple[float,float,float,float]
    ) -> Marker:
        # Lokalen Achsenvektor rotieren
        ax = axis.lower()
        vx, vy, vz = {'x': (1,0,0), 'y': (0,1,0), 'z': (0,0,1)}[ax]
        rx, ry, rz = self._rotate_vec_by_quat((vx, vy, vz), quat)
        ex = pos[0] + rx * length
        ey = pos[1] + ry * length
        ez = pos[2] + rz * length

        m = Marker()
        m.header.frame_id = base_frame
        m.header.stamp = stamp
        m.ns = ns
        m.id = mid
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.points = [geometry_msgs__Point(*pos), geometry_msgs__Point(ex, ey, ez)]
        m.scale.x = thickness         # Schaft-Durchmesser
        m.scale.y = thickness * 2.0   # Kopf-Durchmesser
        m.scale.z = thickness * 2.5   # Kopf-Länge
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m


    @staticmethod
    def _rotate_vec_by_quat(v: Tuple[float,float,float], q: Tuple[float,float,float,float]) -> Tuple[float,float,float]:
        # v' = q * (v,0) * conj(q)
        x, y, z = v
        qx, qy, qz, qw = q
        # quat * vec
        ix =  qw * x + qy * z - qz * y
        iy =  qw * y + qz * x - qx * z
        iz =  qw * z + qx * y - qy * x
        iw = -qx * x - qy * y - qz * z
        # * conj(quat)
        rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
        ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
        rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
        return (rx, ry, rz)

    # ---------------- Marker & Collision Helpers ----------------
    def _lookup_pose(self, parent: str, child: str) -> Optional[Tuple[Tuple[float,float,float], Tuple[float,float,float,float]]]:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time(), timeout=Duration(seconds=0.5))
        except TransformException as e:
            self.get_logger().debug(f"TF lookup failed {parent}->{child}: {e}")
            return None
        pos = (tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z)
        quat = (tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w)
        return pos, quat

    def _publish_static_tf(self, parent: str, child: str, xyz: List[float], rpy_deg: List[float], quat=None):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = parent
        tf.child_frame_id  = child
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        if quat is None:
            qx, qy, qz, qw = rpy_deg_to_quat(float(rpy_deg[0]), float(rpy_deg[1]), float(rpy_deg[2]))
        else:
            qx, qy, qz, qw = quat
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.stf.sendTransform(tf)

    def _spawn_mesh_marker(
        self, *, ns: str, mesh_path: str, parent: str,
        xyz: List[float], rpy_deg: Optional[List[float]] = None, quat: Optional[Tuple[float,float,float,float]] = None,
        rgba: List[float], mid: int
    ):
        m = Marker()
        m.header.frame_id = parent
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = f"file://{mesh_path}"
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = rgba

        m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
        if quat is None and rpy_deg is not None:
            qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)
        elif quat is not None:
            qx, qy, qz, qw = quat
        else:
            qx, qy, qz, qw = (0.0, 0.0, 0.0, 1.0)
        m.pose.orientation.x = qx
        m.pose.orientation.y = qy
        m.pose.orientation.z = qz
        m.pose.orientation.w = qw

        self.marker_pub.publish(m)

    def _delete_marker(self, *, ns: str, mid: int, parent: str):
        m = Marker()
        m.header.frame_id = parent
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.action = Marker.DELETE
        self.marker_pub.publish(m)

    def _publish_mesh_collision_from_stl(
        self, *, object_id: str, parent: str, mesh_path: str,
        xyz: List[float], rpy_deg: Optional[List[float]] = None,
        quat: Optional[Tuple[float,float,float,float]] = None
    ):
        """
        Lädt ein STL als echtes Collision-Mesh in MoveIt (CollisionObject.meshes + mesh_poses).
        Benötigt 'trimesh' (pip install trimesh).
        """
        try:
            import numpy as np
            import trimesh
            from shape_msgs.msg import Mesh, MeshTriangle
            from geometry_msgs.msg import Pose as _Pose
            from geometry_msgs.msg import Point as _Point
        except Exception as e:
            self.get_logger().error(f"Mesh-Collision benötigt 'trimesh': {e}")
            return

        # STL laden
        try:
            tm = trimesh.load(mesh_path, force='mesh')
            if tm.is_empty:
                self.get_logger().error(f"Mesh leer: {mesh_path}")
                return
            tm = tm.as_triangles()
        except Exception as e:
            self.get_logger().error(f"STL laden fehlgeschlagen: {mesh_path}: {e}")
            return

        # shape_msgs/Mesh füllen
        mesh_msg = Mesh()
        # Vertices
        verts = np.asarray(tm.vertices, dtype=float)
        mesh_msg.vertices = [_Point(float(v[0]), float(v[1]), float(v[2])) for v in verts]
        # Triangles (indexiert)
        faces = np.asarray(tm.faces, dtype=np.int32)
        mesh_msg.triangles = [MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]) for f in faces]

        # Pose
        pose = _Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        if quat is None:
            qx, qy, qz, qw = rpy_deg_to_quat(*(rpy_deg or [0,0,0]))
        else:
            qx, qy, qz, qw = quat
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        # CollisionObject
        co = CollisionObject()
        co.header.frame_id = parent
        co.id = object_id
        co.meshes.append(mesh_msg)
        co.mesh_poses.append(pose)
        co.operation = CollisionObject.ADD

        # Senden (direkt über collision_object Topic)
        self.co_pub.publish(co)
        self.get_logger().info(f"Collision-Mesh publiziert: id='{object_id}' faces={len(faces)} verts={len(verts)}")


    def _publish_box_collision(self, *, object_id: str, parent: str, size: List[float], xyz: List[float], rpy_deg: List[float]):
        if not self.ps_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("ApplyPlanningScene nicht verfügbar – skip Collision-Box.")
            return

        L, W, H = size
        cx, cy, cz = xyz[0], xyz[1], xyz[2] + H*0.5
        q = rpy_deg_to_quat(*rpy_deg)

        co = CollisionObject()
        co.header.frame_id = parent
        co.id = object_id

        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [L, W, H]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (cx, cy, cz)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q

        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        req = ApplyPlanningScene.Request()
        req.scene = scene

        fut = self.ps_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.result() is None:
            self.get_logger().warn(f"ApplyPlanningScene fehlgeschlagen: {fut.exception()}")

    def _remove_collision_object(self, object_id: str, parent: str):
        co = CollisionObject()
        co.header.frame_id = parent
        co.id = object_id
        co.operation = CollisionObject.REMOVE
        self.co_pub.publish(co)

    # ---------------- Inferenz Dimensions ----------------
    def _infer_cylinder_dims_from_filename(self, path: str) -> Tuple[float, float]:
        name = os.path.basename(path).lower()

        # wafer_200mm.stl  => r=0.1, h=0.0007
        m = re.search(r"wafer[^0-9]*([0-9]+)\s*mm", name)
        if m:
            d_mm = float(m.group(1))
            r = (d_mm / 2.0) / 1000.0
            h = DEFAULT_WAFER_THICKNESS_M
            return (h, r)

        # disk_200x3.stl => d=200mm, h=3mm
        m2 = re.search(r"disk[^0-9]*([0-9]+)[xX]([0-9]+)", name)
        if m2:
            d_mm = float(m2.group(1))
            h_mm = float(m2.group(2))
            return (h_mm/1000.0, (d_mm/2.0)/1000.0)

        return (DEFAULT_FALLBACK_H_M, DEFAULT_FALLBACK_R_M)


# --- small helpers for geometry_msgs/Point construction (perf) ---
def geometry_msgs__Point(x: float, y: float, z: float):
    pt = type("P", (), {})()
    pt.x, pt.y, pt.z = x, y, z
    return pt


def main():
    rclpy.init()
    rclpy.spin(SceneManager())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
