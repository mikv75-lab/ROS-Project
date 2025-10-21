#!/usr/bin/env python3
from __future__ import annotations
import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

from moveit.planning import MoveItPy

from mecademic_bringup.common.qos import qos_default, qos_latched
from mecademic_bringup.common.frames import FRAMES
from mecademic_bringup.common.topics import Topics


class TrajectoryMarkerNode(Node):
    """
    Visualisiert den geplanten Pfad aus /meca/motion/last_traj in RViz.
    """

    def __init__(self):
        super().__init__("trajectory_marker_node")

        self.frames = FRAMES
        self.topics = Topics()

        self.get_logger().info("🎨 TrajectoryMarkerNode gestartet")

        # --- Subscriber ---
        self._traj_sub = self.create_subscription(
            JointTrajectory,
            self.topics.joint_trajectory(),  # nutzt Controller-Topic
            self._on_traj,
            qos_default(),
        )

        # --- Publisher ---
        self._marker_pub = self.create_publisher(
            MarkerArray,
            self.topics.recipe_markers,  # latched Marker-Topic
            qos_latched(),
        )

        # --- MoveItPy für Forward Kinematics ---
        self._moveit = MoveItPy(node_name="traj_marker_fk")
        self._robot = self._moveit.get_robot_model()
        self._pc = self._moveit.get_planning_component("meca_arm_group")
        self._scene = self._moveit.get_planning_scene_monitor()

        # --- Endeffektor-Link ---
        self._ee_link = self.frames["tool_mount"]

    # ------------------------------------------------------------------

    def _on_traj(self, msg: JointTrajectory):
        n_pts = len(msg.points)
        if n_pts == 0:
            self.get_logger().warn("⚠️ Trajektorie leer – nichts zu visualisieren.")
            return

        self.get_logger().info(f"🎯 Trajektorie empfangen ({n_pts} Punkte)")

        fk_points: list[Point] = []
        rs = self._moveit.get_current_state()

        # --- Forward Kinematics über MoveIt ---
        for pt in msg.points:
            joint_positions = dict(zip(msg.joint_names, pt.positions))
            for jname, pos in joint_positions.items():
                rs.set_joint_positions(jname, pos)
            pose = rs.get_pose(self._ee_link)
            if pose:
                fk_points.append(Point(
                    x=pose.position.x,
                    y=pose.position.y,
                    z=pose.position.z
                ))

        if not fk_points:
            self.get_logger().error("❌ Keine FK-Punkte berechnet.")
            return

        # --- MarkerArray erzeugen ---
        markers = MarkerArray()
        frame = self.frames["world"]

        # Linie
        line_marker = Marker()
        line_marker.header.frame_id = frame
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.ns = "trajectory_line"
        line_marker.id = 0
        line_marker.scale.x = 0.002  # Linienbreite (2 mm)
        line_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.7)
        line_marker.points = fk_points
        markers.markers.append(line_marker)

        # Punkte + Labels
        for i, p in enumerate(fk_points):
            # Kugel
            m = Marker()
            m.header.frame_id = frame
            m.ns = "trajectory_points"
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = p
            m.scale.x = m.scale.y = m.scale.z = 0.006  # 6 mm
            m.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.8)
            markers.markers.append(m)

            # Textlabel
            t = Marker()
            t.header.frame_id = frame
            t.ns = "trajectory_labels"
            t.id = 1000 + i
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position = Point(x=p.x, y=p.y, z=p.z + 0.01)
            t.scale.z = 0.012
            t.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.8)
            t.text = str(i)
            markers.markers.append(t)

        self._marker_pub.publish(markers)
        self.get_logger().info(f"✅ {len(fk_points)} Pfadpunkte visualisiert (Frame={frame}).")
