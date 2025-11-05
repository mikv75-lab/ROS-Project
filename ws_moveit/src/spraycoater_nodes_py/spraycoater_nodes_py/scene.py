# -*- coding: utf-8 -*-
# spraycoater_nodes_py/scene.py
#!/usr/bin/env python3
import os
import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from spraycoater_nodes_py.utils.utils import rpy_deg_to_quat
from spraycoater_nodes_py.utils.frames import load_frames
import trimesh

SCENE_TOPIC = "/collision_object"
MAX_SCENE_WAIT = 30.0
PARAM_SCENE_CONFIG = "scene_config"
PARAM_FRAMES_YAML  = "frames_yaml"
PARAM_FRAMES_GROUP = "frames_group"


def _require_vec3(node: dict, key: str):
    if key not in node:
        raise KeyError(f"YAML: fehlendes Feld '{key}'")
    val = node[key]
    if (not isinstance(val, (list, tuple))) or len(val) != 3:
        raise ValueError(f"YAML: '{key}' muss eine Liste mit 3 Zahlen sein, bekommen: {val!r}")
    return [float(val[0]), float(val[1]), float(val[2])]


def _require_str(node: dict, key: str):
    if key not in node:
        raise KeyError(f"YAML: fehlendes Feld '{key}'")
    val = node[key]
    if not isinstance(val, str):
        raise ValueError(f"YAML: '{key}' muss String sein, bekommen: {type(val).__name__}")
    return val


def _quat_mul(a, b):
    """Hamilton-Produkt q = a ⊗ b (x,y,z,w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw*bx + ax*bw + ay*bz - az*by,
        aw*by - ax*bz + ay*bw + az*bx,
        aw*bz + ax*by - ay*bx + az*bw,
        aw*bw - ax*bx - ay*by - az*bz,
    )


def _quat_from_rpy_deg(r, p, y):
    return rpy_deg_to_quat(float(r), float(p), float(y))


def _rotmat_from_quat(q):
    x, y, z, w = q
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return [
        [1 - 2*(yy+zz),     2*(xy - wz),     2*(xz + wy)],
        [    2*(xy + wz), 1 - 2*(xx+zz),     2*(yz - wx)],
        [    2*(xz - wy),     2*(yz + wx), 1 - 2*(xx+yy)],
    ]


def _rot_apply(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


class Scene(Node):
    def __init__(self):
        super().__init__("scene")

        # Parameter
        self.declare_parameter(PARAM_SCENE_CONFIG, "")
        self.declare_parameter(PARAM_FRAMES_YAML, "")
        self.declare_parameter(PARAM_FRAMES_GROUP, "meca")

        yaml_path = self.get_parameter(PARAM_SCENE_CONFIG).value
        if not yaml_path or not os.path.exists(yaml_path):
            raise FileNotFoundError(f"Scene YAML fehlt: {yaml_path}")

        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        if "scene_objects" not in data or not isinstance(data["scene_objects"], list):
            raise ValueError("YAML: 'scene_objects' fehlt oder ist nicht Liste")
        self.scene_objects = data["scene_objects"]
        self.base_dir = os.path.dirname(os.path.abspath(yaml_path))

        # Frames laden
        frames_yaml  = self.get_parameter(PARAM_FRAMES_YAML).value
        frames_group = self.get_parameter(PARAM_FRAMES_GROUP).value
        self.frames  = load_frames(frames_yaml, frames_group)
        self._F      = self.frames.resolve  # Kurzform

        # ROS I/O
        self.static_tf = StaticTransformBroadcaster(self)
        self.scene_pub = self.create_publisher(CollisionObject, SCENE_TOPIC, 10)
        self.mesh_cache = {}
        self.final_scene_sent = False

        self.get_logger().info("⏳ Warte auf MoveIt – Szene später senden...")
        self._start_scene_wait()

    # ---------- Helpers ----------
    def _resolve_mesh_path(self, mesh_rel: str) -> str:
        p = (mesh_rel or "").strip()
        if not p:
            return ""  # leer → kein Mesh
        cand = os.path.join(self.base_dir, p)
        if os.path.exists(cand):
            return cand
        if os.path.isabs(p) and os.path.exists(p):
            return p
        if os.path.exists(p):
            return os.path.abspath(p)
        raise FileNotFoundError(f"Mesh nicht gefunden: {p} (base={self.base_dir})")

    def _load_mesh_mm(self, mesh_path: str) -> Mesh:
        cache_key = f"{mesh_path}::mm"
        if cache_key in self.mesh_cache:
            return self.mesh_cache[cache_key]

        tri = trimesh.load(mesh_path, force="mesh")
        tri.apply_scale(0.001)  # mm → m

        msg = Mesh()
        msg.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in tri.vertices]
        msg.triangles = [MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]) for f in tri.faces]

        self.mesh_cache[cache_key] = msg
        return msg

    @staticmethod
    def _pose_from_xyz_quat(xyz, quat) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        qx, qy, qz, qw = quat
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        return pose

    def _make_tf(self, parent: str, child: str, xyz, rpy_deg) -> TransformStamped:
        qx, qy, qz, qw = _quat_from_rpy_deg(*rpy_deg)
        tf = TransformStamped()
        tf.header.frame_id = self._F(parent)
        tf.child_frame_id  = self._F(child)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        return tf

    # ---------- Scene publication ----------
    def _publish_scene(self):
        if self.final_scene_sent:
            return
        self.final_scene_sent = True
        F = self._F

        # 1) Static TFs
        tfs = []
        for obj in self.scene_objects:
            if "id" not in obj:
                raise KeyError("YAML: jedes Objekt braucht ein 'id'")
            oid = str(obj["id"])
            frame = F(obj.get("frame", "world"))
            xyz = _require_vec3(obj, "position")
            rpy = _require_vec3(obj, "rpy_deg")
            tfs.append(self._make_tf(frame, oid, xyz, rpy))
        if tfs:
            self.static_tf.sendTransform(tfs)
        self.get_logger().info(f"✅ Static TFs veröffentlicht ({len(tfs)})")

        # 2) CollisionObjects (Pose = (position,rpy_deg) ⊕ (mesh_offset,mesh_rpy) im Parent-Frame)
        for obj in self.scene_objects:
            oid = str(obj["id"])
            frame = F(obj.get("frame", "world"))
            pos = _require_vec3(obj, "position")
            rpy = _require_vec3(obj, "rpy_deg")

            mesh_rel = _require_str(obj, "mesh")  # leer → kein Mesh
            if mesh_rel.strip() == "":
                continue

            mesh_path = self._resolve_mesh_path(mesh_rel)
            mesh_msg = self._load_mesh_mm(mesh_path)

            mpos = _require_vec3(obj, "mesh_offset")
            mrpy = _require_vec3(obj, "mesh_rpy")

            # Rotation/Translation zusammensetzen:
            q_frame = _quat_from_rpy_deg(*rpy)       # Rotation des Objekts (frame→id)
            R_frame = _rotmat_from_quat(q_frame)
            mpos_in_parent = _rot_apply(R_frame, mpos)
            p_total = [pos[0] + mpos_in_parent[0],
                       pos[1] + mpos_in_parent[1],
                       pos[2] + mpos_in_parent[2]]

            q_mesh = _quat_from_rpy_deg(*mrpy)       # lokale Mesh-Rotation
            q_total = _quat_mul(q_frame, q_mesh)     # erst Frame, dann Mesh

            co = CollisionObject()
            co.id = oid
            co.header.frame_id = frame               # Parent-Frame (bereits aufgelöst)
            co.meshes = [mesh_msg]
            co.mesh_poses = [self._pose_from_xyz_quat(p_total, q_total)]
            co.operation = CollisionObject.ADD
            self.scene_pub.publish(co)

        self.get_logger().info("🎯 Szene FINAL & EINMALIG an MoveIt gesendet.")

    # ---------- MoveIt readiness wait ----------
    def _start_scene_wait(self):
        self.deadline = time.time() + MAX_SCENE_WAIT
        self.wait_timer = self.create_timer(1.0, self._check_moveit_ready)

    def _check_moveit_ready(self):
        if self.final_scene_sent:
            self.wait_timer.cancel()
            return

        if time.time() > self.deadline:
            self.get_logger().warning("⚠️ Timeout – sende Szene trotzdem.")
            self._publish_scene()
            self.wait_timer.cancel()
            return

        if not hasattr(self, "ps_client"):
            self.ps_client = self.create_client(GetPlanningScene, "/get_planning_scene")

        if not self.ps_client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("⏳ Warte auf MoveIt PlanningScene Service...")
            return

        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.SCENE_SETTINGS
        future = self.ps_client.call_async(req)
        future.add_done_callback(self._moveit_ready_callback)

    def _moveit_ready_callback(self, _):
        self.get_logger().info("✅ MoveIt bereit – sende Szene in 3 Sekunden...")
        self.wait_timer.cancel
        self.create_timer(3.0, self._publish_scene)


def main():
    rclpy.init()
    rclpy.spin(Scene())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
