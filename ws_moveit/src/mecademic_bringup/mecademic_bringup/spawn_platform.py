#!/usr/bin/env python3
import math
import re
from typing import Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from tf2_ros import StaticTransformBroadcaster


def yaw_quat(yaw_rad: float) -> Tuple[float, float, float, float]:
    s = math.sin(yaw_rad * 0.5)
    c = math.cos(yaw_rad * 0.5)
    return (0.0, 0.0, s, c)  # x,y,z,w


class PlatformSpawner(Node):
    """
    Spawnt eine Box an vorgegebener Position (x,y,yaw) im parent_frame so,
    dass sie mit ihrer Unterseite auf z=0 "steht".
    Zusätzlich werden zwei statische TFs publiziert:
      - <child_frame_base> @ (x, y, 0)      – Mitte der Unterseite
      - <child_frame_top>  @ (x, y, height) – Mitte der Oberseite
    """

    def __init__(self):
        super().__init__('platform_spawner')

        # ---------------- Params ----------------
        self.declare_parameter('parent_frame', 'meca_mount')
        self.declare_parameter('x', 0.20)
        self.declare_parameter('y', 0.00)
        self.declare_parameter('yaw_deg', 0.0)

        # Entweder explizit L/W/H in Metern …
        self.declare_parameter('length', 0.0)   # m (0 = auto)
        self.declare_parameter('width', 0.0)    # m
        self.declare_parameter('height', 0.0)   # m
        # … oder aus Dateinamen-Muster, z.B. "base_platform_200x200x50.stl"
        self.declare_parameter('size_hint', 'base_platform_200x200x50.stl')

        self.declare_parameter('object_id', 'base_platform')
        self.declare_parameter('publish_static_tf', True)
        self.declare_parameter('child_frame_base', 'platform_base')
        self.declare_parameter('child_frame_top', 'platform_top')

        parent_frame   = self.get_parameter('parent_frame').get_parameter_value().string_value
        x              = float(self.get_parameter('x').value)
        y              = float(self.get_parameter('y').value)
        yaw_deg        = float(self.get_parameter('yaw_deg').value)
        L_param        = float(self.get_parameter('length').value)
        W_param        = float(self.get_parameter('width').value)
        H_param        = float(self.get_parameter('height').value)
        size_hint      = self.get_parameter('size_hint').get_parameter_value().string_value
        object_id      = self.get_parameter('object_id').get_parameter_value().string_value
        do_stat_tf     = bool(self.get_parameter('publish_static_tf').value)
        child_base     = self.get_parameter('child_frame_base').get_parameter_value().string_value
        child_top      = self.get_parameter('child_frame_top').get_parameter_value().string_value

        # ---------------- Abmessungen ----------------
        if L_param > 0.0 and W_param > 0.0 and H_param > 0.0:
            L, W, H = L_param, W_param, H_param
        else:
            # aus "<...>_LxWxH.stl" (mm) parsen; Fallback 0.20/0.20/0.05 m
            m = re.search(r'(\d+)[xX](\d+)[xX](\d+)', size_hint)
            if m:
                L, W, H = (float(m.group(1))/1000.0,
                           float(m.group(2))/1000.0,
                           float(m.group(3))/1000.0)
            else:
                L, W, H = 0.20, 0.20, 0.05

        yaw = math.radians(yaw_deg)
        qx, qy, qz, qw = yaw_quat(yaw)

        # ---------------- Static TF (optional) ----------------
        if do_stat_tf:
            stf = StaticTransformBroadcaster(self)

            # Base: Mittelpunkt der Unterseite
            tf_base = TransformStamped()
            tf_base.header.stamp = self.get_clock().now().to_msg()
            tf_base.header.frame_id = parent_frame
            tf_base.child_frame_id = child_base
            tf_base.transform.translation.x = x
            tf_base.transform.translation.y = y
            tf_base.transform.translation.z = 0.0
            tf_base.transform.rotation.x = qx
            tf_base.transform.rotation.y = qy
            tf_base.transform.rotation.z = qz
            tf_base.transform.rotation.w = qw

            # Top: Mittelpunkt der Oberseite (praktisch für workspace_center)
            tf_top = TransformStamped()
            tf_top.header.stamp = tf_base.header.stamp
            tf_top.header.frame_id = parent_frame
            tf_top.child_frame_id = child_top
            tf_top.transform.translation.x = x
            tf_top.transform.translation.y = y
            tf_top.transform.translation.z = H
            tf_top.transform.rotation.x = qx
            tf_top.transform.rotation.y = qy
            tf_top.transform.rotation.z = qz
            tf_top.transform.rotation.w = qw

            stf.sendTransform([tf_base, tf_top])
            self.get_logger().info(
                f"🔩 static TFs: '{child_base}' @ z=0.000,  '{child_top}' @ z={H:.3f} im '{parent_frame}'"
            )

        # ---------------- CollisionObject (steht auf Boden) ----------------
        # Pose: Box-Mittelpunkt liegt bei z = H/2, damit Unterseite bei z=0 ist.
        center_z = H * 0.5

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = center_z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [L, W, H]  # X, Y, Z

        co = CollisionObject()
        co.header.frame_id = parent_frame
        co.id = object_id
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        self.client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Service 'apply_planning_scene' nicht verfügbar (läuft move_group?).")
            rclpy.shutdown()
            return

        req = ApplyPlanningScene.Request()
        req.scene = scene

        self.get_logger().info(
            f"🧱 Adding box '{object_id}' {L:.3f}×{W:.3f}×{H:.3f} m in '{parent_frame}' at "
            f"(x={x:.3f}, y={y:.3f}, z={center_z:.3f}), yaw={yaw_deg:.1f}°"
        )

        fut = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if fut.result() is None:
            self.get_logger().error(f"❌ ApplyPlanningScene fehlgeschlagen: {fut.exception()}")
        else:
            self.get_logger().info("✅ Platform zur PlanningScene hinzugefügt.")
        rclpy.shutdown()


def main():
    rclpy.init()
    PlatformSpawner()


if __name__ == '__main__':
    main()
