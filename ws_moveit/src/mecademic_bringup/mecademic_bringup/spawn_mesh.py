#!/usr/bin/env python3
import math
from typing import Sequence

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from tf2_ros import StaticTransformBroadcaster


def rpy_deg_to_quat(rpy_deg: Sequence[float]):
    rx, ry, rz = [math.radians(x) for x in rpy_deg]
    cx, sx = math.cos(rx * 0.5), math.sin(rx * 0.5)
    cy, sy = math.cos(ry * 0.5), math.sin(ry * 0.5)
    cz, sz = math.cos(rz * 0.5), math.sin(rz * 0.5)
    qw = cx * cy * cz + sx * sy * sz
    qx = sx * cy * cz - cx * sy * sz
    qy = cx * sy * cz + sx * cy * sz
    qz = cx * cy * sz - sx * sy * cz
    return (qx, qy, qz, qw)


class MeshSpawner(Node):
    """
    Spawnt ein Objekt als CollisionObject:
      - ohne trimesh (es wird eine Fallback-Box verwendet)
      - wartet optional unbegrenzt auf den Service 'apply_planning_scene'
    """
    def __init__(self):
        super().__init__("mesh_spawner")

        # --- Parameter ---
        self.declare_parameter("parent_frame", "meca_mount")
        self.declare_parameter("object_id", "workspace_mount")
        self.declare_parameter("mesh_path", "")               # ignoriert (kein trimesh)
        self.declare_parameter("xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("rpy_deg", [0.0, 0.0, 0.0])
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("publish_static_tf", True)
        self.declare_parameter("child_frame", "mesh_center")
        self.declare_parameter("fallback_box", [0.0, 0.0, 0.0])   # L,W,H (m)
        self.declare_parameter("wait_for_service", True)          # Warten bis move_group da ist
        self.declare_parameter("wait_log_period", 5.0)            # s, Log-Intervall beim Warten

        parent_frame = self.get_parameter("parent_frame").get_parameter_value().string_value
        object_id = self.get_parameter("object_id").get_parameter_value().string_value
        # mesh_path = self.get_parameter("mesh_path").get_parameter_value().string_value  # bewusst ungenutzt
        xyz = list(self.get_parameter("xyz").get_parameter_value().double_array_value)
        rpy_deg = list(self.get_parameter("rpy_deg").get_parameter_value().double_array_value)
        scale = float(self.get_parameter("scale").value)
        publish_tf = bool(self.get_parameter("publish_static_tf").value)
        child_frame = self.get_parameter("child_frame").get_parameter_value().string_value
        fb_box = list(self.get_parameter("fallback_box").get_parameter_value().double_array_value)
        wait_for_service = bool(self.get_parameter("wait_for_service").value)
        wait_log_period = float(self.get_parameter("wait_log_period").value)

        # --- Pose berechnen ---
        qx, qy, qz, qw = rpy_deg_to_quat(rpy_deg)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw

        # --- Optional: statischer TF ---
        if publish_tf:
            stf = StaticTransformBroadcaster(self)
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = parent_frame
            tf.child_frame_id = child_frame
            tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
            tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
            stf.sendTransform([tf])
            self.get_logger().info(f"🔩 static TF '{child_frame}' in '{parent_frame}' gesetzt.")

        # --- CollisionObject erzeugen (nur Box-Fallback) ---
        co = CollisionObject()
        co.header.frame_id = parent_frame
        co.id = object_id

        if not all(d > 0.0 for d in fb_box):
            self.get_logger().error(
                f"Kein Mesh-Import aktiv und Fallback-Box ungültig (fallback_box={fb_box}) -> breche ab."
            )
            rclpy.shutdown()
            return

        L, W, H = fb_box
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [L * scale, W * scale, H * scale]
        co.primitives.append(box)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        self.get_logger().warning(f"⚠️ Verwende Fallback-Box {L}×{W}×{H} m (scale={scale}) für '{object_id}'.")

        # --- Scene anwenden (mit robustem Warten auf move_group) ---
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)

        client = self.create_client(ApplyPlanningScene, "apply_planning_scene")

        if wait_for_service:
            self._wait_for_apply_planning_scene(client, wait_log_period)
        else:
            if not client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error("❌ Service 'apply_planning_scene' nicht verfügbar und wait_for_service=false.")
                rclpy.shutdown()
                return

        req = ApplyPlanningScene.Request()
        req.scene = scene

        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() and fut.result().success:
            self.get_logger().info(f"✅ '{object_id}' zur PlanningScene hinzugefügt.")
        else:
            self.get_logger().error(f"❌ ApplyPlanningScene fehlgeschlagen: {fut.exception()}")
        rclpy.shutdown()

    def _wait_for_apply_planning_scene(self, client, log_period: float):
        """
        Wartet unbegrenzt (mit periodischem Log) bis der Service verfügbar ist.
        """
        self.get_logger().info("⏳ warte auf 'apply_planning_scene' (move_group)…")
        last_log = 0.0
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - last_log >= max(1.0, log_period):
                self.get_logger().info("… immer noch am Warten auf move_group.")
                last_log = now


def main():
    rclpy.init()
    MeshSpawner()


if __name__ == "__main__":
    main()
