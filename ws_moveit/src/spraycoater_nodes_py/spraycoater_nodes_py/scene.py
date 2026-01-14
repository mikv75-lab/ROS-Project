#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# spraycoater_nodes_py/scene.py
from __future__ import annotations

import os
import time
import yaml
from typing import Any, Dict, List, Optional, Tuple

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

# ament index for package:// resolution (needed for mount meshes)
from ament_index_python.packages import get_package_share_directory  # type: ignore


MAX_SCENE_WAIT = 30.0

# Republish (nur CollisionObjects). 0.0 = aus
REPUBLISH_PERIOD_S = 10.0

# Delay zwischen TFs und CollisionObjects (Race-Condition vermeiden)
CO_PUBLISH_DELAY_S = 0.2

# Robust UI Republish, falls QoS/Start-Reihenfolge nicht "latched" ist
UI_REPUBLISH_PERIOD_S = 1.0
UI_REPUBLISH_DURATION_S = 15.0

OID_ALIAS = {
    "cage": "cage",
    "mount": "substrate_mount",
    "substrate": "substrate",
}


def _logical_to_oid(logical: str) -> str:
    return OID_ALIAS.get(logical, logical)


def _require_vec3(node: dict, key: str) -> List[float]:
    if key not in node:
        raise KeyError(f"YAML: fehlendes Feld '{key}'")
    val = node[key]
    if (not isinstance(val, (list, tuple))) or len(val) != 3:
        raise ValueError(f"YAML: '{key}' muss eine Liste mit 3 Zahlen sein, bekommen: {val!r}")
    return [float(val[0]), float(val[1]), float(val[2])]


def _quat_from_rpy_deg(r: float, p: float, y: float) -> Tuple[float, float, float, float]:
    return rpy_deg_to_quat(float(r), float(p), float(y))


def _quat_mul(
    a: Tuple[float, float, float, float],
    b: Tuple[float, float, float, float],
) -> Tuple[float, float, float, float]:
    """
    Hamilton product: a ⊗ b
    Applies rotation b after a in the common ROS convention when composing transforms:
      q_total = q_parent ⊗ q_child
    """
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    x = aw * bx + ax * bw + ay * bz - az * by
    y = aw * by - ax * bz + ay * bw + az * bx
    z = aw * bz + ax * by - ay * bx + az * bw
    w = aw * bw - ax * bx - ay * by - az * bz
    return (float(x), float(y), float(z), float(w))


def _mm_to_m_xyz(xyz_mm: List[float]) -> List[float]:
    return [float(xyz_mm[0]) / 1000.0, float(xyz_mm[1]) / 1000.0, float(xyz_mm[2]) / 1000.0]


def _mesh_basename(mesh_uri_or_path: str) -> str:
    s = (mesh_uri_or_path or "").strip()
    if not s:
        return ""
    return os.path.basename(s)


def _dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _resolve_package_uri_to_file(uri: str) -> str:
    """
    Resolve package://<pkg>/<relpath> to an absolute file path inside the package share dir.
    """
    s = (uri or "").strip()
    if not s.startswith("package://"):
        raise ValueError(f"not a package uri: {uri!r}")

    rest = s[len("package://") :]
    # split "pkg/path/inside"
    parts = rest.split("/", 1)
    if not parts or not parts[0].strip():
        raise ValueError(f"invalid package uri: {uri!r}")
    pkg = parts[0].strip()
    rel = parts[1] if len(parts) > 1 else ""
    share = get_package_share_directory(pkg)
    path = os.path.join(share, rel)
    if not os.path.exists(path):
        raise FileNotFoundError(f"package uri not found: {uri} -> {path}")
    return os.path.abspath(path)


class Scene(Node):
    """
    Scene Node Contract (updated):

    - cage:     mesh selectable via scene.yaml + cage_set (file path)
    - substrate mesh selectable via scene.yaml + substrate_set (file path)
    - mount:    mesh selectable via substrate_mounts.yaml active_mount (package://)

    TF:
      - static TFs from scene.yaml for all objects EXCEPT substrate:
          parent=obj.frame -> child=obj.id  (position/rpy_deg in meters/degrees)
      - substrate TF is SPECIAL:
          parent is ALWAYS 'substrate_mount' (regardless of scene.yaml)
          transform = mount.scene_offset (mm, deg) + substrate.position/rpy_deg (m, deg)
          published as STATIC (latched) so MoveIt never misses it.

    CollisionObjects:
      - header.frame_id == object id (oid), mesh_pose only uses mesh_offset/mesh_rpy (if present)
      - mount CO uses the active mount mesh (package://), pose identity (or scene.yaml mesh_offset/mesh_rpy if given)
    """

    GROUP = "scene"

    def __init__(self) -> None:
        super().__init__("scene")

        self.declare_parameter("backend", "default")
        self.backend: str = (
            self.get_parameter("backend").get_parameter_value().string_value or "default"
        )

        self.loader = topics()
        self.frames = frames()
        self._F = self.frames.resolve

        # Paths
        self._scene_yaml_path = config_path("scene.yaml")
        self._mounts_yaml_path = config_path("substrate_mounts.yaml")

        # robot.yaml (asset scan roots)
        robot_cfg = load_yaml("robot.yaml")
        sel = (robot_cfg.get("selected") or "").strip()
        robots = robot_cfg.get("robots") or {}
        r = robots.get(sel, {}) if isinstance(robots.get(sel, {}), dict) else {}
        self._roots_cage = resolve_resource_dirs(r.get("cage_dirs", []))
        self._roots_substrate = resolve_resource_dirs(r.get("substrate_dirs", []))

        # Load scene.yaml (STRICT-ish)
        with open(self._scene_yaml_path, "r", encoding="utf-8") as f:
            scene_data = yaml.safe_load(f) or {}
        if "scene_objects" not in scene_data or not isinstance(scene_data["scene_objects"], list):
            raise ValueError("scene.yaml: 'scene_objects' fehlt oder ist nicht Liste")
        self._scene_yaml_data = scene_data
        self.scene_objects: List[dict] = scene_data["scene_objects"]
        self.base_dir = os.path.dirname(os.path.abspath(self._scene_yaml_path))

        # Ensure required objects exist
        if self._find_scene_object(_logical_to_oid("mount")) is None:
            raise ValueError("scene.yaml: required object id='substrate_mount' fehlt (STRICT).")
        if self._find_scene_object(_logical_to_oid("substrate")) is None:
            raise ValueError("scene.yaml: required object id='substrate' fehlt (STRICT).")

        # Load mounts.yaml (STRICT)
        self._mounts_yaml_data = self._load_mounts_yaml_strict()
        self._mount_defs: Dict[str, Any] = dict(self._mounts_yaml_data.get("mounts") or {})
        self._active_mount_key: str = str(self._mounts_yaml_data.get("active_mount") or "").strip()
        if self._active_mount_key:
            self._require_mount_def(self._active_mount_key)

        # UI pubs
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

        # UI subs
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

        # TF (static, latched)
        self.static_tf = StaticTransformBroadcaster(self)

        # CollisionObject pub
        topic_co = self.loader.publish_topic(self.GROUP, "collision_object")
        qos_co = self.loader.qos_by_id("publish", self.GROUP, "collision_object")
        self.scene_pub = self.create_publisher(CollisionObject, topic_co, qos_co)

        # state/cache
        self.mesh_cache: Dict[str, Mesh] = {}
        self.final_scene_sent = False
        self.initial_state_published = False
        self._cos_sent = False
        self._co_timer = None
        self._cached_cos: Dict[str, CollisionObject] = {}
        self._republish_timer = None

        # currents
        self.current_cage = self._initial_current_from_scene_yaml_mesh_basename("cage")
        self.current_substrate = self._initial_current_from_scene_yaml_mesh_basename("substrate")
        self.current_mount = self._active_mount_key  # key, NICHT basename

        # publish initial UI state
        self._publish_lists_once()
        self._publish_current_once()

        # robust UI republish
        self._ui_repub_deadline = time.time() + float(UI_REPUBLISH_DURATION_S)
        self._ui_repub_timer = self.create_timer(float(UI_REPUBLISH_PERIOD_S), self._republish_ui_state_until_seen)

        ns = self.get_namespace() or "/"
        self.get_logger().info(
            f"SceneNode aktiv (backend='{self.backend}', ns='{ns}') – "
            f"CollisionObjects auf '{topic_co}', warte auf Subscriber und sende Szene einmalig."
        )

        # publish TFs first (latched), then wait/publish COs
        self._publish_all_static_tfs()                 # includes world->cage, world->substrate_mount, etc.
        self._publish_substrate_tf_from_active_mount(reason="startup")  # substrate_mount->substrate (with offset)
        self._start_scene_wait()

    # ------------------------
    # Mount YAML (STRICT)
    # ------------------------
    def _load_mounts_yaml_strict(self) -> dict:
        with open(self._mounts_yaml_path, "r", encoding="utf-8") as f:
            mounts_data = yaml.safe_load(f) or {}
        if not isinstance(mounts_data, dict):
            raise ValueError("substrate_mounts.yaml: muss dict sein.")
        if int(mounts_data.get("version", 0) or 0) != 1:
            raise ValueError("substrate_mounts.yaml: version muss 1 sein.")
        if "mounts" not in mounts_data or not isinstance(mounts_data["mounts"], dict):
            raise ValueError("substrate_mounts.yaml: 'mounts' fehlt oder ist nicht dict.")
        return mounts_data

    def _require_mount_def(self, key: str) -> dict:
        if not key:
            raise ValueError("substrate_mounts.yaml: mount key ist leer (STRICT).")
        md = self._mount_defs.get(key)
        if not isinstance(md, dict):
            raise KeyError(f"substrate_mounts.yaml: mount '{key}' existiert nicht oder ist kein dict.")
        if "mesh" not in md:
            raise KeyError(f"substrate_mounts.yaml: mounts['{key}'].mesh fehlt.")
        if "scene_offset" not in md or not isinstance(md["scene_offset"], dict):
            raise KeyError(f"substrate_mounts.yaml: mounts['{key}'].scene_offset fehlt/ist nicht dict.")
        so = md["scene_offset"]
        _ = _require_vec3(so, "xyz")      # mm
        _ = _require_vec3(so, "rpy_deg")  # deg
        return md

    def _save_mounts_yaml(self) -> None:
        with open(self._mounts_yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self._mounts_yaml_data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"substrate_mounts.yaml aktualisiert: {self._mounts_yaml_path}")

    # ------------------------
    # Lookup objects
    # ------------------------
    def _find_scene_object(self, oid: str) -> Optional[dict]:
        for obj in self.scene_objects:
            if str(obj.get("id", "")) == oid:
                return obj
        return None

    def _initial_current_from_scene_yaml_mesh_basename(self, oid_logical: str) -> str:
        oid = _logical_to_oid(oid_logical)
        obj = self._find_scene_object(oid)
        if obj is None:
            return ""
        mesh_rel = str(obj.get("mesh", "") or "").strip()
        return os.path.basename(mesh_rel) if mesh_rel else ""

    # ------------------------
    # Asset scanning
    # ------------------------
    def _scan_assets(self, roots: List[str]) -> List[str]:
        out: List[str] = []
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
    def _list_to_string(names: List[str]) -> str:
        return ",".join(names)

    # ------------------------
    # UI publish
    # ------------------------
    def _publish_lists_once(self) -> None:
        if self.initial_state_published:
            return

        cages = self._scan_assets(self._roots_cage)
        subs = self._scan_assets(self._roots_substrate)
        mount_keys = sorted([k for k in self._mount_defs.keys() if isinstance(k, str)])

        self.pub_cage_list.publish(String(data=self._list_to_string(cages)))
        self.pub_mount_list.publish(String(data=self._list_to_string(mount_keys)))
        self.pub_substrate_list.publish(String(data=self._list_to_string(subs)))

    def _publish_current_once(self) -> None:
        if self.initial_state_published:
            return
        self.pub_cage_current.publish(String(data=self.current_cage))
        self.pub_mount_current.publish(String(data=self.current_mount))  # mount key
        self.pub_substrate_current.publish(String(data=self.current_substrate))
        self.initial_state_published = True

    def _republish_ui_state_until_seen(self) -> None:
        if time.time() > self._ui_repub_deadline:
            try:
                self._ui_repub_timer.cancel()
            except Exception:
                pass
            return

        cages = self._scan_assets(self._roots_cage)
        subs = self._scan_assets(self._roots_substrate)
        mount_keys = sorted([k for k in self._mount_defs.keys() if isinstance(k, str)])

        self.pub_cage_list.publish(String(data=self._list_to_string(cages)))
        self.pub_mount_list.publish(String(data=self._list_to_string(mount_keys)))
        self.pub_substrate_list.publish(String(data=self._list_to_string(subs)))

        self.pub_cage_current.publish(String(data=self.current_cage))
        self.pub_mount_current.publish(String(data=self.current_mount))
        self.pub_substrate_current.publish(String(data=self.current_substrate))

        subs_count = (
            self.pub_cage_list.get_subscription_count()
            + self.pub_mount_list.get_subscription_count()
            + self.pub_substrate_list.get_subscription_count()
            + self.pub_cage_current.get_subscription_count()
            + self.pub_mount_current.get_subscription_count()
            + self.pub_substrate_current.get_subscription_count()
        )
        if subs_count > 0:
            try:
                self._ui_repub_timer.cancel()
            except Exception:
                pass

    # ------------------------
    # YAML save
    # ------------------------
    def _save_scene_yaml(self) -> None:
        with open(self._scene_yaml_path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self._scene_yaml_data, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f"scene.yaml aktualisiert: {self._scene_yaml_path}")

    # ------------------------
    # Mesh resolve/load
    # ------------------------
    def _resolve_mesh_path(self, mesh_rel: str) -> str:
        p = (mesh_rel or "").strip()
        if not p:
            return ""
        if p.startswith("package://"):
            return _resolve_package_uri_to_file(p)

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
        tri.apply_scale(0.001)  # STL mm -> m

        msg = Mesh()
        msg.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in tri.vertices]
        msg.triangles = [MeshTriangle(vertex_indices=[int(f[0]), int(f[1]), int(f[2])]) for f in tri.faces]

        self.mesh_cache[cache_key] = msg
        return msg

    @staticmethod
    def _pose_from_xyz_quat(xyz_m: List[float], quat: Tuple[float, float, float, float]) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = float(xyz_m[0]), float(xyz_m[1]), float(xyz_m[2])
        qx, qy, qz, qw = quat
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        return pose

    def _make_tf(self, parent: str, child: str, xyz_m: List[float], rpy_deg: List[float]) -> TransformStamped:
        qx, qy, qz, qw = _quat_from_rpy_deg(*rpy_deg)

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._F(parent)
        tf.child_frame_id = self._F(child)
        tf.transform.translation.x = float(xyz_m[0])
        tf.transform.translation.y = float(xyz_m[1])
        tf.transform.translation.z = float(xyz_m[2])
        tf.transform.rotation.x = float(qx)
        tf.transform.rotation.y = float(qy)
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        return tf

    def _make_co_from_obj_and_mesh_path(self, obj: dict, mesh_path: str) -> CollisionObject:
        """
        CollisionObject IM Objekt-Frame (oid).
        Offsets/Rotationen wirken via TFs:
          parent->oid (position/rpy_deg) + mesh_pose (mesh_offset/mesh_rpy)
        """
        oid = str(obj["id"])

        # mesh_offset + mesh_rpy: if missing, default to identity (keeps mount object optional fields)
        try:
            mpos_m = _require_vec3(obj, "mesh_offset")
        except Exception:
            mpos_m = [0.0, 0.0, 0.0]
        try:
            mrpy_deg = _require_vec3(obj, "mesh_rpy")
        except Exception:
            mrpy_deg = [0.0, 0.0, 0.0]

        mesh_msg = self._load_mesh_mm(mesh_path)
        q_mesh = _quat_from_rpy_deg(*mrpy_deg)

        co = CollisionObject()
        co.id = oid
        co.header.stamp = self.get_clock().now().to_msg()
        co.header.frame_id = self._F(oid)  # IM Objekt-Frame!
        co.meshes = [mesh_msg]
        co.mesh_poses = [self._pose_from_xyz_quat(mpos_m, q_mesh)]
        co.operation = CollisionObject.ADD
        return co

    def _remove_object(self, oid: str) -> None:
        co = CollisionObject()
        co.id = oid
        co.operation = CollisionObject.REMOVE
        self.scene_pub.publish(co)
        self._cached_cos.pop(oid, None)
        self.get_logger().info(f"REMOVE CollisionObject id={oid}")

    # ------------------------
    # TF: substrate_mount -> substrate includes active mount offset
    # ------------------------
    def _publish_substrate_tf_from_active_mount(self, *, reason: str) -> None:
        """
        Build substrate_mount -> substrate from:
          - active_mount.scene_offset (mm, deg)
          - substrate.position/rpy_deg (m, deg) from scene.yaml

        Publish as STATIC (latched).
        """
        # reload mounts.yaml
        self._mounts_yaml_data = self._load_mounts_yaml_strict()
        self._mount_defs = dict(self._mounts_yaml_data.get("mounts") or {})
        key = str(self._mounts_yaml_data.get("active_mount") or "").strip()

        sub_oid = _logical_to_oid("substrate")
        sub_obj = self._find_scene_object(sub_oid)
        if sub_obj is None:
            raise ValueError("scene.yaml: required object id='substrate' fehlt (STRICT).")

        # substrate local (relative to mount)
        s_xyz_m = _require_vec3(sub_obj, "position")
        s_rpy_deg = _require_vec3(sub_obj, "rpy_deg")
        q_sub = _quat_from_rpy_deg(*s_rpy_deg)

        if not key:
            # No active mount => publish substrate TF purely from substrate local (still parent=substrate_mount)
            xyz = [float(s_xyz_m[0]), float(s_xyz_m[1]), float(s_xyz_m[2])]
            q = q_sub
            self.get_logger().warning(
                f"active_mount leer ({reason}) – substrate TF nutzt nur substrate.position/rpy_deg (kein mount offset)."
            )
        else:
            md = self._require_mount_def(key)
            so = md["scene_offset"]
            m_xyz_mm = _require_vec3(so, "xyz")
            m_rpy_deg = _require_vec3(so, "rpy_deg")
            m_xyz_m = _mm_to_m_xyz(m_xyz_mm)
            q_mount = _quat_from_rpy_deg(*m_rpy_deg)

            xyz = [m_xyz_m[0] + s_xyz_m[0], m_xyz_m[1] + s_xyz_m[1], m_xyz_m[2] + s_xyz_m[2]]
            q = _quat_mul(q_mount, q_sub)

        mount_oid = _logical_to_oid("mount")

        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self._F(mount_oid)  # parent: substrate_mount
        tf.child_frame_id = self._F(sub_oid)     # child: substrate
        tf.transform.translation.x = float(xyz[0])
        tf.transform.translation.y = float(xyz[1])
        tf.transform.translation.z = float(xyz[2])
        tf.transform.rotation.x = float(q[0])
        tf.transform.rotation.y = float(q[1])
        tf.transform.rotation.z = float(q[2])
        tf.transform.rotation.w = float(q[3])

        # static (latched): MoveIt cannot miss it
        self.static_tf.sendTransform(tf)

        # update current mount UI (key)
        self._active_mount_key = key
        self.current_mount = key
        self.pub_mount_current.publish(String(data=key))

        self.get_logger().info(
            f"substrate TF updated ({reason}): parent=substrate_mount child=substrate "
            f"active_mount='{key}' xyz_m={xyz}"
        )

    # ------------------------
    # Mount collision object (mesh from substrate_mounts.yaml)
    # ------------------------
    def _publish_mount_collision_object(self, *, reason: str) -> None:
        """
        Publish/update CollisionObject for substrate_mount using active_mount.mesh (package://).
        """
        mount_oid = _logical_to_oid("mount")
        mount_obj = self._find_scene_object(mount_oid)
        if mount_obj is None:
            raise ValueError("scene.yaml: required object id='substrate_mount' fehlt (STRICT).")

        # reload mounts.yaml
        self._mounts_yaml_data = self._load_mounts_yaml_strict()
        self._mount_defs = dict(self._mounts_yaml_data.get("mounts") or {})
        key = str(self._mounts_yaml_data.get("active_mount") or "").strip()

        if not key:
            # If no active mount => remove mount mesh CO
            self._remove_object(mount_oid)
            self.get_logger().warning(f"mount CO removed ({reason}): active_mount ist leer.")
            return

        md = self._require_mount_def(key)
        mesh_uri = str(md.get("mesh") or "").strip()
        if not mesh_uri:
            raise ValueError(f"substrate_mounts.yaml: mounts['{key}'].mesh ist leer (STRICT).")

        mesh_path = self._resolve_mesh_path(mesh_uri)
        co = self._make_co_from_obj_and_mesh_path(mount_obj, mesh_path)
        self.scene_pub.publish(co)
        self._cached_cos[mount_oid] = co

        self.get_logger().info(f"mount CO published ({reason}): active_mount='{key}' mesh='{mesh_uri}'")

    # ------------------------
    # Publish TFs + Scene
    # ------------------------
    def _publish_all_static_tfs(self) -> None:
        """
        Publisht static TFs aus scene.yaml für alle Objekte EXCEPT substrate:
          parent=obj.frame -> child=obj.id  (position/rpy_deg)
        substrate TF wird separat via _publish_substrate_tf_from_active_mount() erzeugt.
        """
        tfs: List[TransformStamped] = []
        substrate_oid = _logical_to_oid("substrate")

        for obj in self.scene_objects:
            if "id" not in obj:
                raise KeyError("scene.yaml: jedes Objekt braucht ein 'id'")
            oid = str(obj["id"])

            if oid == substrate_oid:
                continue  # handled separately (mount offset + substrate local)

            parent = str(obj.get("frame", "") or "").strip()
            if not parent:
                raise ValueError(f"scene.yaml: Objekt '{oid}' hat kein 'frame' (STRICT).")

            xyz_m = _require_vec3(obj, "position")   # meters
            rpy_deg = _require_vec3(obj, "rpy_deg")  # degrees
            tfs.append(self._make_tf(parent, oid, xyz_m, rpy_deg))

        if tfs:
            self.static_tf.sendTransform(tfs)
            self.get_logger().info(f"Static TFs gesendet/aktualisiert ({len(tfs)})")

    def _publish_scene(self) -> None:
        if self.final_scene_sent:
            return
        self.final_scene_sent = True

        # 1) TFs first (latched)
        self._publish_all_static_tfs()
        self._publish_substrate_tf_from_active_mount(reason="publish_scene")

        # 2) delay then collision objects
        if self._co_timer is None:
            self._co_timer = self.create_timer(CO_PUBLISH_DELAY_S, self._publish_collision_objects_once)
            self.get_logger().info(f"CollisionObjects folgen nach {CO_PUBLISH_DELAY_S:.2f}s (TF-Buffer warmup)")

    def _publish_collision_objects_once(self) -> None:
        if self._cos_sent:
            return
        self._cos_sent = True

        try:
            if self._co_timer is not None:
                self._co_timer.cancel()
        except Exception:
            pass

        self._cached_cos = {}
        sent = 0

        # 1) mount mesh (from mounts.yaml)
        try:
            self._publish_mount_collision_object(reason="collision_objects_once")
            # counts as 1 if active_mount set
            if self._active_mount_key:
                sent += 1
        except Exception as e:
            self.get_logger().error(f"Mount CO publish failed: {e!r}")

        # 2) cage + substrate from scene.yaml
        for obj in self.scene_objects:
            oid = str(obj.get("id", ""))

            # mount handled above
            if oid == _logical_to_oid("mount"):
                continue

            mesh_rel = str(obj.get("mesh", "") or "").strip()
            if mesh_rel == "":
                continue

            mesh_path = self._resolve_mesh_path(mesh_rel)
            co = self._make_co_from_obj_and_mesh_path(obj, mesh_path)
            self.scene_pub.publish(co)
            self._cached_cos[oid] = co
            sent += 1

        ns = self.get_namespace() or "/"
        self.get_logger().info(
            f"Szene FINAL gesendet (backend='{self.backend}', ns='{ns}'). CollisionObjects={sent}"
        )

        self._publish_lists_once()
        self._publish_current_once()

        if self._republish_timer is None and REPUBLISH_PERIOD_S > 0.0:
            self._republish_timer = self.create_timer(REPUBLISH_PERIOD_S, self._republish_collision_objects)
            self.get_logger().info(f"Republish aktiv: CollisionObjects alle {REPUBLISH_PERIOD_S:.1f}s")

    def _republish_collision_objects(self) -> None:
        if not self.final_scene_sent:
            return
        if not self._cached_cos:
            return
        for co in self._cached_cos.values():
            self.scene_pub.publish(co)

    # ------------------------
    # Topic handlers
    # ------------------------
    def _on_set_cage(self, msg: String) -> None:
        name = (msg.data or "").strip()
        oid = _logical_to_oid("cage")
        obj = self._find_scene_object(oid)
        if obj is None:
            raise ValueError("scene.yaml: required object id='cage' fehlt (STRICT).")

        if name == "":
            self._remove_object(oid)
            self.current_cage = ""
            self.pub_cage_current.publish(String(data=""))
            obj["mesh"] = ""
            self._save_scene_yaml()
            self._publish_all_static_tfs()
            return

        avail = set(self._scan_assets(self._roots_cage))
        if name not in avail:
            raise ValueError(f"cage_set: Mesh '{name}' nicht gefunden in cage_dirs (STRICT).")

        mesh_path = None
        for root in self._roots_cage:
            cand = os.path.abspath(os.path.join(os.path.expanduser(root), name))
            if os.path.exists(cand):
                mesh_path = cand
                break
        if mesh_path is None:
            raise ValueError(f"cage_set: Mesh '{name}' konnte nicht aufgelöst werden (STRICT).")

        self._remove_object(oid)
        co = self._make_co_from_obj_and_mesh_path(obj, mesh_path)
        self.scene_pub.publish(co)
        self._cached_cos[oid] = co

        self.current_cage = name
        self.pub_cage_current.publish(String(data=name))

        try:
            obj["mesh"] = os.path.relpath(mesh_path, self.base_dir)
        except Exception:
            obj["mesh"] = mesh_path
        self._save_scene_yaml()
        self._publish_all_static_tfs()

    def _on_set_substrate(self, msg: String) -> None:
        name = (msg.data or "").strip()
        oid = _logical_to_oid("substrate")
        obj = self._find_scene_object(oid)
        if obj is None:
            raise ValueError("scene.yaml: required object id='substrate' fehlt (STRICT).")

        if name == "":
            self._remove_object(oid)
            self.current_substrate = ""
            self.pub_substrate_current.publish(String(data=""))
            obj["mesh"] = ""
            self._save_scene_yaml()
            # TFs: substrate TF still exists; keep it (no mesh is fine)
            self._publish_all_static_tfs()
            self._publish_substrate_tf_from_active_mount(reason="substrate_set(clear)")
            return

        avail = set(self._scan_assets(self._roots_substrate))
        if name not in avail:
            raise ValueError(f"substrate_set: Mesh '{name}' nicht gefunden in substrate_dirs (STRICT).")

        mesh_path = None
        for root in self._roots_substrate:
            cand = os.path.abspath(os.path.join(os.path.expanduser(root), name))
            if os.path.exists(cand):
                mesh_path = cand
                break
        if mesh_path is None:
            raise ValueError(f"substrate_set: Mesh '{name}' konnte nicht aufgelöst werden (STRICT).")

        self._remove_object(oid)
        co = self._make_co_from_obj_and_mesh_path(obj, mesh_path)
        self.scene_pub.publish(co)
        self._cached_cos[oid] = co

        self.current_substrate = name
        self.pub_substrate_current.publish(String(data=name))

        try:
            obj["mesh"] = os.path.relpath(mesh_path, self.base_dir)
        except Exception:
            obj["mesh"] = mesh_path
        self._save_scene_yaml()

        # Re-publish TFs (latched) to ensure chain is present
        self._publish_all_static_tfs()
        self._publish_substrate_tf_from_active_mount(reason="substrate_set")

    def _on_set_mount(self, msg: String) -> None:
        """
        mount_set bekommt den MOUNT-KEY (z.B. "post_d30mm_h100mm").
        Es wird:
          - substrate_mounts.yaml active_mount gesetzt
          - TF substrate_mount->substrate neu publiziert (inkl. mount offset)
          - mount CollisionObject neu publiziert (mesh aus mounts.yaml)
        """
        key = (msg.data or "").strip()

        # Reload mounts.yaml strictly
        self._mounts_yaml_data = self._load_mounts_yaml_strict()
        self._mount_defs = dict(self._mounts_yaml_data.get("mounts") or {})

        if key == "":
            self._mounts_yaml_data["active_mount"] = ""
            self._active_mount_key = ""
            self._save_mounts_yaml()

            self.current_mount = ""
            self.pub_mount_current.publish(String(data=""))

            # remove mount CO, update substrate TF (without mount offset)
            self._publish_mount_collision_object(reason="mount_set(clear)")
            self._publish_substrate_tf_from_active_mount(reason="mount_set(clear)")

            self.get_logger().info("mount_set: active_mount gelöscht.")
            return

        _ = self._require_mount_def(key)

        self._mounts_yaml_data["active_mount"] = key
        self._active_mount_key = key
        self._save_mounts_yaml()

        self.current_mount = key
        self.pub_mount_current.publish(String(data=key))

        # ensure base TFs present
        self._publish_all_static_tfs()

        # publish substrate TF from mount offset + substrate local
        self._publish_substrate_tf_from_active_mount(reason="mount_set")

        # publish mount mesh CO
        self._publish_mount_collision_object(reason="mount_set")

    # ------------------------
    # Readiness wait
    # ------------------------
    def _start_scene_wait(self) -> None:
        self.deadline = time.time() + MAX_SCENE_WAIT
        self.wait_timer = self.create_timer(0.5, self._check_ready_by_subscribers)

    def _check_ready_by_subscribers(self) -> None:
        if self.final_scene_sent and self._cos_sent:
            try:
                self.wait_timer.cancel()
            except Exception:
                pass
            return

        sub_count = self.scene_pub.get_subscription_count()
        if sub_count > 0:
            self.get_logger().info(f"Subscriber auf collision_object erkannt (count={sub_count}) – sende Szene.")
            try:
                self.wait_timer.cancel()
            except Exception:
                pass
            self._publish_scene()
            return

        if time.time() > self.deadline:
            self.get_logger().warning(
                "Timeout – kein Subscriber erkannt, sende Szene trotzdem (Republish fängt Late-Joiner)."
            )
            try:
                self.wait_timer.cancel()
            except Exception:
                pass
            self._publish_scene()
            return

        ns = self.get_namespace() or "/"
        topic_co = self.loader.publish_topic(self.GROUP, "collision_object")
        self.get_logger().info(f"Warte auf Subscriber (MoveItPy) auf '{ns}/{topic_co}'... (subs={sub_count})")


def main() -> None:
    rclpy.init()
    rclpy.spin(Scene())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
