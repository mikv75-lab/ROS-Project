#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/scene.py
from __future__ import annotations

import os
import time
import yaml

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, TransformStamped, Point
from tf2_ros import StaticTransformBroadcaster
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import CollisionObject

from spraycoater_nodes_py.utils.utils import rpy_deg_to_quat
from spraycoater_nodes_py.utils.config_hub import (
    topics, frames, config_path, load_yaml, resolve_resource_dirs
)

import trimesh

MAX_SCENE_WAIT = 30.0

# ✅ Republish deaktiviert - verhindert "Moved backwards in time" Warning
# Wenn du Republish brauchst, setze z.B. auf 300.0 (5 Minuten)
REPUBLISH_PERIOD_S = 0.0

OID_ALIAS = {
    "cage": "cage",
    "mount": "substrate_mount",
    "substrate": "substrate",
}


def _logical_to_oid(logical: str) -> str:
    return OID_ALIAS.get(logical, logical)


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
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_from_rpy_deg(r, p, y):
    return rpy_deg_to_quat(float(r), float(p), float(y))


def _rotmat_from_quat(q):
    x, y, z, w = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
        [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
    ]


def _rot_apply(R, v):
    return [
        R[0][0] * v[0] + R[0][1] * v[1] + R[0][2] * v[2],
        R[1][0] * v[0] + R[1][1] * v[1] + R[1][2] * v[2],
        R[2][0] * v[0] + R[2][1] * v[1] + R[2][2] * v[2],
    ]


class Scene(Node):
    """
    Scene-Node (MoveItPy-only):

      - liest scene.yaml / robot.yaml aus config_hub
      - publisht Cage/Mount/Substrate Listen+Current via topics.yaml
      - publisht CollisionObjects via topics.yaml (collision_object)
      - wartet NICHT auf move_group/get_planning_scene
      - wartet stattdessen auf Subscriber (MoveItPy) auf collision_object und sendet dann die Szene
      - republished CollisionObjects periodisch (für Late-Joiner), wenn REPUBLISH_PERIOD_S > 0
    """

    GROUP = "scene"

    def __init__(self):
        super().__init__("scene")

        self.declare_parameter("backend", "default")
        self.backend: str = (
            self.get_parameter("backend").get_parameter_value().string_value or "default"
        )

        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve

        scene_yaml = config_path("scene.yaml")
        robot_cfg = load_yaml("robot.yaml")

        with open(scene_yaml, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if "scene_objects" not in data or not isinstance(data["scene_objects"], list):
            raise ValueError("YAML: 'scene_objects' fehlt oder ist nicht Liste")

        self.scene_objects = data["scene_objects"]
        self.base_dir = os.path.dirname(os.path.abspath(scene_yaml))

        self._scene_yaml_path = scene_yaml
        self._scene_yaml_data = data

        sel = (robot_cfg.get("selected") or "").strip()
        robots = robot_cfg.get("robots") or {}
        r = robots.get(sel, {}) if isinstance(robots.get(sel, {}), dict) else {}

        self._roots_cage = resolve_resource_dirs(r.get("cage_dirs", []))
        self._roots_mount = resolve_resource_dirs(r.get("mount_dirs", []))
        self._roots_substrate = resolve_resource_dirs(r.get("substrate_dirs", []))

        # UI Topics
        self.pub_cage_current = self.create_publisher(
            String,
            self.loader.publish_topic(self.GROUP, "cage_current"),
            self.loader.qos_by_id("publish", self.GROUP, "cage_current"),
        )
        self.pub_mount_current = self.create_publisher(
            String,
            self.loader.publish_topic(self.GROUP, "mount_current"),
            self.loader.qos_by_id("publish", self.GROUP, "mount_current"),
        )
        self.pub_substrate_current = self.create_publisher(
            String,
            self.loader.publish_topic(self.GROUP, "substrate_current"),
            self.loader.qos_by_id("publish", self.GROUP, "substrate_current"),
        )

        self.pub_cage_list = self.create_publisher(
            String,
            self.loader.publish_topic(self.GROUP, "cage_list"),
            self.loader.qos_by_id("publish", self.GROUP, "cage_list"),
        )
        self.pub_mount_list = self.create_publisher(
            String,
            self.loader.publish_topic(self.GROUP, "mount_list"),
            self.loader.qos_by_id("publish", self.GROUP, "mount_list"),
        )
        self.pub_substrate_list = self.create_publisher(
            String,
            self.loader.publish_topic(self.GROUP, "substrate_list"),
            self.loader.qos_by_id("publish", self.GROUP, "substrate_list"),
        )

        self.create_subscription(
            String,
            self.loader.subscribe_topic(self.GROUP, "cage_set"),
            self._on_set_cage,
            self.loader.qos_by_id("subscribe", self.GROUP, "cage_set"),
        )
        self.create_subscription(
            String,
            self.loader.subscribe_topic(self.GROUP, "mount_set"),
            self._on_set_mount,
            self.loader.qos_by_id("subscribe", self.GROUP, "mount_set"),
        )
        self.create_subscription(
            String,
            self.loader.subscribe_topic(self.GROUP, "substrate_set"),
            self._on_set_substrate,
            self.loader.qos_by_id("subscribe", self.GROUP, "substrate_set"),
        )

        # TF
        self.static_tf = StaticTransformBroadcaster(self)

        # CollisionObject Topic + QoS aus topics.yaml/qos.yaml
        topic_co = self.loader.publish_topic(self.GROUP, "collision_object")
        qos_co = self.loader.qos_by_id("publish", self.GROUP, "collision_object")
        self.scene_pub = self.create_publisher(CollisionObject, topic_co, qos_co)

        self.mesh_cache = {}
        self.final_scene_sent = False
        self.initial_state_published = False

        # ✅ WICHTIG: logical keys verwenden (cage/mount/substrate)
        self.current_cage: str = self._initial_current_from_yaml("cage")
        self.current_mount: str = self._initial_current_from_yaml("mount")
        self.current_substrate: str = self._initial_current_from_yaml("substrate")

        # Cache für Republish (wird nach _publish_scene gefüllt)
        self._cached_cos: dict[str, CollisionObject] = {}
        self._republish_timer = None
        self._last_republish_log = 0.0

        self._publish_lists_once()
        self._publish_current_once()

        ns = self.get_namespace() or "/"
        self.get_logger().info(
            f"⏳ SceneNode aktiv (backend='{self.backend}', ns='{ns}') – "
            f"publisht CollisionObjects auf '{topic_co}' (QoS via config_hub), "
            "warte auf Subscriber (MoveItPy) und sende dann die Szene einmalig."
        )
        self._start_scene_wait()

    # ------------------------
    # Asset-Scan
    # ------------------------
    def _scan_assets(self, roots):
        out = []
        for root in roots:
            root = os.path.expanduser(root)
            if not os.path.isdir(root):
                continue
            for _, _, files in os.walk(root):
                for f in files:
                    if f.lower().endswith(".stl"):
                        out.append(f)
        return sorted(set(out))

    @staticmethod
    def _list_to_string(names):
        return ",".join(names)

    def _publish_lists_once(self):
        if self.initial_state_published:
            return
        cages = self._scan_assets(self._roots_cage)
        mounts = self._scan_assets(self._roots_mount)
        subs = self._scan_assets(self._roots_substrate)

        self.pub_cage_list.publish(String(data=self._list_to_string(cages)))
        self.pub_mount_list.publish(String(data=self._list_to_string(mounts)))
        self.pub_substrate_list.publish(String(data=self._list_to_string(subs)))

    def _publish_current_once(self):
        if self.initial_state_published:
            return
        self.pub_cage_current.publish(String(data=self.current_cage))
        self.pub_mount_current.publish(String(data=self.current_mount))
        self.pub_substrate_current.publish(String(data=self.current_substrate))
        self.initial_state_published = True

    def _save_scene_yaml(self):
        try:
            with open(self._scene_yaml_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(self._scene_yaml_data, f, sort_keys=False, allow_unicode=True)
            self.get_logger().info(f"💾 scene.yaml aktualisiert: {self._scene_yaml_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Fehler beim Schreiben von scene.yaml: {e!r}")

    # ------------------------
    # Initial "current" aus YAML bestimmen
    # ------------------------
    def _find_scene_object(self, oid: str) -> dict | None:
        for obj in self.scene_objects:
            if str(obj.get("id", "")) == oid:
                return obj
        return None

    def _initial_current_from_yaml(self, oid_logical: str) -> str:
        oid = _logical_to_oid(oid_logical)
        obj = self._find_scene_object(oid)
        if obj is None:
            return ""
        try:
            mesh_rel = _require_str(obj, "mesh")
        except Exception:
            return ""
        mesh_rel = (mesh_rel or "").strip()
        return os.path.basename(mesh_rel) if mesh_rel else ""

    # ------------------------
    # Dynamic mesh replacement
    # ------------------------
    def _resolve_mesh_path(self, mesh_rel: str) -> str:
        p = (mesh_rel or "").strip()
        if not p:
            return ""
        cand = os.path.join(self.base_dir, p)
        if os.path.exists(cand):
            return cand
        if os.path.isabs(p) and os.path.exists(p):
            return p
        if os.path.exists(p):
            return os.path.abspath(p)
        raise FileNotFoundError(f"Mesh nicht gefunden: {p} (base={self.base_dir})")

    def _resolve_asset_candidate(self, name: str, roots: list[str]) -> str | None:
        if not name:
            return None
        name = name.strip()
        if not name:
            return None
        if name.startswith("package://") or os.path.isabs(name):
            return name
        for root in roots:
            cand = os.path.abspath(os.path.join(os.path.expanduser(root), name))
            if os.path.exists(cand):
                return cand
        return None

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
        tf.child_frame_id = self._F(child)
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = xyz
        tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = qx, qy, qz, qw
        return tf

    def _make_co_from_obj_and_mesh(self, obj: dict, mesh_path: str) -> CollisionObject:
        oid = str(obj["id"])
        frame = self._F(obj.get("frame", "world"))
        pos = _require_vec3(obj, "position")
        rpy = _require_vec3(obj, "rpy_deg")
        mpos = _require_vec3(obj, "mesh_offset")
        mrpy = _require_vec3(obj, "mesh_rpy")

        mesh_msg = self._load_mesh_mm(mesh_path)

        q_frame = _quat_from_rpy_deg(*rpy)
        R_frame = _rotmat_from_quat(q_frame)
        mpos_in_parent = _rot_apply(R_frame, mpos)
        p_total = [pos[0] + mpos_in_parent[0],
                   pos[1] + mpos_in_parent[1],
                   pos[2] + mpos_in_parent[2]]
        q_mesh = _quat_from_rpy_deg(*mrpy)
        q_total = _quat_mul(q_frame, q_mesh)

        co = CollisionObject()
        co.id = oid
        co.header.frame_id = frame
        co.meshes = [mesh_msg]
        co.mesh_poses = [self._pose_from_xyz_quat(p_total, q_total)]
        co.operation = CollisionObject.ADD
        return co

    def _remove_object(self, oid: str):
        co = CollisionObject()
        co.id = oid
        co.operation = CollisionObject.REMOVE
        self.scene_pub.publish(co)

        # Cache sauber halten (für Republish)
        self._cached_cos.pop(oid, None)

        self.get_logger().info(f"🧹 REMOVE CollisionObject id={oid}")

    def _exists_in(self, name: str, roots):
        if not name:
            return False
        if name.startswith("package://") or os.path.isabs(name):
            return True
        avail = set(self._scan_assets(roots))
        return (name in avail)

    def _handle_set(self, *, kind: str, name: str, roots: list[str], pub, state_attr: str):
        oid = _logical_to_oid(kind)
        obj = self._find_scene_object(oid)
        if obj is None:
            self.get_logger().error(f"YAML-Objekt mit id='{oid}' nicht gefunden – Abbruch.")
            return

        if name == "":
            self._remove_object(oid)
            setattr(self, state_attr, "")
            pub.publish(String(data=""))
            obj["mesh"] = ""
            self._save_scene_yaml()
            self.get_logger().info(f"✅ {kind}: mesh entfernt (current='').")
            return

        if not self._exists_in(name, roots):
            self.get_logger().error(f"{kind}: Mesh '{name}' nicht gefunden in {roots}.")
            return

        mesh_path = self._resolve_asset_candidate(name, roots)
        if mesh_path is None:
            self.get_logger().error(f"{kind}: Mesh '{name}' konnte nicht aufgelöst werden.")
            return

        self._remove_object(oid)
        co = self._make_co_from_obj_and_mesh(obj, mesh_path)
        self.scene_pub.publish(co)

        # Cache aktualisieren (Republish soll genau das wieder senden)
        self._cached_cos[oid] = co

        setattr(self, state_attr, name)
        pub.publish(String(data=name))

        try:
            new_rel = os.path.relpath(mesh_path, self.base_dir)
        except Exception:
            new_rel = mesh_path

        obj["mesh"] = new_rel
        self._save_scene_yaml()
        self.get_logger().info(f"🎯 {kind}: mesh gesetzt -> {name} (yaml='{new_rel}')")

    def _on_set_cage(self, msg: String):
        self._handle_set(kind="cage", name=(msg.data or "").strip(),
                         roots=self._roots_cage, pub=self.pub_cage_current, state_attr="current_cage")

    def _on_set_mount(self, msg: String):
        self._handle_set(kind="mount", name=(msg.data or "").strip(),
                         roots=self._roots_mount, pub=self.pub_mount_current, state_attr="current_mount")

    def _on_set_substrate(self, msg: String):
        self._handle_set(kind="substrate", name=(msg.data or "").strip(),
                         roots=self._roots_substrate, pub=self.pub_substrate_current, state_attr="current_substrate")

    # ------------------------
    # Scene publication (Initial aus YAML)
    # ------------------------
    def _publish_scene(self):
        if self.final_scene_sent:
            return
        self.final_scene_sent = True
        F = self._F

        # TFs nur einmal senden (StaticTransformBroadcaster ist latched)
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

        # CollisionObjects senden + Cache füllen
        self._cached_cos = {}
        for obj in self.scene_objects:
            mesh_rel = _require_str(obj, "mesh")
            if mesh_rel.strip() == "":
                continue
            mesh_path = self._resolve_mesh_path(mesh_rel)
            co = self._make_co_from_obj_and_mesh(obj, mesh_path)
            self.scene_pub.publish(co)
            self._cached_cos[str(obj["id"])] = co

        ns = self.get_namespace() or "/"
        self.get_logger().info(f"🎯 Szene FINAL gesendet (backend='{self.backend}', ns='{ns}').")

        self._publish_lists_once()
        self._publish_current_once()

        # ✅ Republish-Timer starten (nur CO, nicht TF) - nur wenn REPUBLISH_PERIOD_S > 0
        if self._republish_timer is None and REPUBLISH_PERIOD_S > 0.0:
            self._republish_timer = self.create_timer(REPUBLISH_PERIOD_S, self._republish_collision_objects)
            self.get_logger().info(f"🔁 Republish aktiv: CollisionObjects alle {REPUBLISH_PERIOD_S:.1f}s")
        elif REPUBLISH_PERIOD_S == 0.0:
            self.get_logger().info("ℹ️  Republish deaktiviert (REPUBLISH_PERIOD_S = 0.0)")

    def _republish_collision_objects(self):
        """Republished CollisionObjects für Late-Joiner, ohne Log-Spam."""
        if not self.final_scene_sent:
            return
        if not self._cached_cos:
            return

        for co in self._cached_cos.values():
            self.scene_pub.publish(co)

        # optional: sehr selten loggen (z.B. alle 60s)
        # now = time.time()
        # if now - self._last_republish_log > 60.0:
        #     self._last_republish_log = now
        #     self.get_logger().debug(f"🔁 Republished {len(self._cached_cos)} CollisionObjects")

    # ------------------------
    # Readiness wait (MoveItPy-only)
    # ------------------------
    def _start_scene_wait(self):
        self.deadline = time.time() + MAX_SCENE_WAIT
        self.wait_timer = self.create_timer(0.5, self._check_ready_by_subscribers)

    def _check_ready_by_subscribers(self):
        if self.final_scene_sent:
            self.wait_timer.cancel()
            return

        sub_count = self.scene_pub.get_subscription_count()
        if sub_count > 0:
            self.get_logger().info(f"✅ Subscriber auf collision_object erkannt (count={sub_count}) – sende Szene.")
            self.wait_timer.cancel()
            self._publish_scene()
            return

        if time.time() > self.deadline:
            self.get_logger().warning(
                "⚠️ Timeout – kein Subscriber erkannt, sende Szene trotzdem (Republish fängt Late-Joiner)."
            )
            self.wait_timer.cancel()
            self._publish_scene()
            return

        # Debug/Info (nicht zu spammy: 0.5s ist ok, aber du kannst hier auch drosseln)
        ns = self.get_namespace() or "/"
        topic_co = self.loader.publish_topic(self.GROUP, "collision_object")
        self.get_logger().info(f"⏳ Warte auf Subscriber (MoveItPy) auf '{ns}/{topic_co}'... (subs={sub_count})")


def main():
    rclpy.init()
    rclpy.spin(Scene())
    rclpy.shutdown()


if __name__ == "__main__":
    main()