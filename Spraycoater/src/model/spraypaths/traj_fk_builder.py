# -*- coding: utf-8 -*-
# File: src/model/recipe/traj_fk_builder.py
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple, Union

import yaml

from model.spraypaths.draft import Draft
from Spraycoater.src.model.spraypaths.trajectory import JTBySegment


# ============================================================
# Helpers (STRICT)
# ============================================================

def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def _dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _require_dict(d: Any, name: str) -> Dict[str, Any]:
    if not isinstance(d, dict):
        raise ValueError(f"{name} must be dict, got {type(d).__name__}")
    return d


def _require_list3(v: Any, name: str) -> Tuple[float, float, float]:
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(f"{name} must be list[3], got {v!r}")
    return (float(v[0]), float(v[1]), float(v[2]))


def _quat_dot(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def _quat_norm(q: Tuple[float, float, float, float]) -> float:
    return math.sqrt(_quat_dot(q, q))


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    n = _quat_norm(q)
    if n <= 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _rpy_deg_to_quat_xyzw(r: float, p: float, y: float) -> Tuple[float, float, float, float]:
    rr = math.radians(float(r))
    pp = math.radians(float(p))
    yy = math.radians(float(y))

    cr = math.cos(rr * 0.5)
    sr = math.sin(rr * 0.5)
    cp = math.cos(pp * 0.5)
    sp = math.sin(pp * 0.5)
    cy = math.cos(yy * 0.5)
    sy = math.sin(yy * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return _quat_normalize((qx, qy, qz, qw))


def _quat_to_rot3(q: Tuple[float, float, float, float]) -> List[List[float]]:
    qx, qy, qz, qw = _quat_normalize(q)

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def _rot3_to_quat(R: List[List[float]]) -> Tuple[float, float, float, float]:
    # Convert rotation matrix to quaternion (x,y,z,w), normalized
    tr = R[0][0] + R[1][1] + R[2][2]
    if tr > 0.0:
        S = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * S
        qx = (R[2][1] - R[1][2]) / S
        qy = (R[0][2] - R[2][0]) / S
        qz = (R[1][0] - R[0][1]) / S
    elif (R[0][0] > R[1][1]) and (R[0][0] > R[2][2]):
        S = math.sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]) * 2.0
        qw = (R[2][1] - R[1][2]) / S
        qx = 0.25 * S
        qy = (R[0][1] + R[1][0]) / S
        qz = (R[0][2] + R[2][0]) / S
    elif R[1][1] > R[2][2]:
        S = math.sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]) * 2.0
        qw = (R[0][2] - R[2][0]) / S
        qx = (R[0][1] + R[1][0]) / S
        qy = 0.25 * S
        qz = (R[1][2] + R[2][1]) / S
    else:
        S = math.sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]) * 2.0
        qw = (R[1][0] - R[0][1]) / S
        qx = (R[0][2] + R[2][0]) / S
        qy = (R[1][2] + R[2][1]) / S
        qz = 0.25 * S

    return _quat_normalize((qx, qy, qz, qw))


def _mat4_identity() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_from_xyz_quat_mm(
    xyz_mm: Tuple[float, float, float],
    q: Tuple[float, float, float, float],
) -> List[List[float]]:
    R = _quat_to_rot3(q)
    tx, ty, tz = xyz_mm
    return [
        [R[0][0], R[0][1], R[0][2], float(tx)],
        [R[1][0], R[1][1], R[1][2], float(ty)],
        [R[2][0], R[2][1], R[2][2], float(tz)],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_mul(A: List[List[float]], B: List[List[float]]) -> List[List[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = (
                A[i][0] * B[0][j]
                + A[i][1] * B[1][j]
                + A[i][2] * B[2][j]
                + A[i][3] * B[3][j]
            )
    return out


def _mat4_inv_rigid(T: List[List[float]]) -> List[List[float]]:
    # inverse for rigid transform (R,t)
    R = [[T[r][c] for c in range(3)] for r in range(3)]
    t = [T[r][3] for r in range(3)]

    Rt = [
        [R[0][0], R[1][0], R[2][0]],
        [R[0][1], R[1][1], R[2][1]],
        [R[0][2], R[1][2], R[2][2]],
    ]
    tinv = [
        -(Rt[0][0] * t[0] + Rt[0][1] * t[1] + Rt[0][2] * t[2]),
        -(Rt[1][0] * t[0] + Rt[1][1] * t[1] + Rt[1][2] * t[2]),
        -(Rt[2][0] * t[0] + Rt[2][1] * t[1] + Rt[2][2] * t[2]),
    ]
    return [
        [Rt[0][0], Rt[0][1], Rt[0][2], tinv[0]],
        [Rt[1][0], Rt[1][1], Rt[1][2], tinv[1]],
        [Rt[2][0], Rt[2][1], Rt[2][2], tinv[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _apply_T_to_point_mm(T: List[List[float]], p: Tuple[float, float, float]) -> Tuple[float, float, float]:
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    xo = T[0][0] * x + T[0][1] * y + T[0][2] * z + T[0][3]
    yo = T[1][0] * x + T[1][1] * y + T[1][2] * z + T[1][3]
    zo = T[2][0] * x + T[2][1] * y + T[2][2] * z + T[2][3]
    return (xo, yo, zo)


def _apply_T_to_quat(T: List[List[float]], q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    # q_to = R_to_from * q_from
    R_T = [[T[r][c] for c in range(3)] for r in range(3)]
    R_q = _quat_to_rot3(q)
    R = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            R[i][j] = R_T[i][0] * R_q[0][j] + R_T[i][1] * R_q[1][j] + R_T[i][2] * R_q[2][j]
    return _rot3_to_quat(R)


def _load_yaml_file(path: str) -> Dict[str, Any]:
    if not path:
        raise ValueError("YAML path is empty")
    p = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(p):
        raise FileNotFoundError(p)
    with open(p, "r", encoding="utf-8") as f:
        return _dict(yaml.safe_load(f) or {})


# ============================================================
# Offline TF: scene.yaml chain + robot.yaml (+ substrate_mounts.yaml)
# ============================================================

def _scene_object_by_id(scene_doc: Mapping[str, Any], oid: str) -> Dict[str, Any]:
    objs = scene_doc.get("scene_objects")
    if not isinstance(objs, list):
        raise ValueError("scene.yaml: 'scene_objects' missing or not list")
    for o in objs:
        if isinstance(o, dict) and str(o.get("id", "")) == oid:
            return o
    raise KeyError(f"scene.yaml: object id={oid!r} not found")


def _tf_from_scene_obj_meters(scene_obj: Mapping[str, Any]) -> List[List[float]]:
    pos_m = _require_list3(scene_obj.get("position"), "scene_obj.position(m)")
    rpy = _require_list3(scene_obj.get("rpy_deg"), "scene_obj.rpy_deg(deg)")
    pos_mm = (pos_m[0] * 1000.0, pos_m[1] * 1000.0, pos_m[2] * 1000.0)
    q = _rpy_deg_to_quat_xyzw(rpy[0], rpy[1], rpy[2])
    return _mat4_from_xyz_quat_mm(pos_mm, q)


def _tf_world_robot_mount_from_robot_yaml(robot_doc: Mapping[str, Any]) -> List[List[float]]:
    blk = _require_dict(robot_doc.get("world_to_robot_mount"), "robot.yaml.world_to_robot_mount")
    xyz_m = _require_list3(blk.get("xyz"), "robot.yaml.world_to_robot_mount.xyz(m)")
    rpy = _require_list3(blk.get("rpy_deg"), "robot.yaml.world_to_robot_mount.rpy_deg(deg)")
    xyz_mm = (xyz_m[0] * 1000.0, xyz_m[1] * 1000.0, xyz_m[2] * 1000.0)
    q = _rpy_deg_to_quat_xyzw(rpy[0], rpy[1], rpy[2])
    return _mat4_from_xyz_quat_mm(xyz_mm, q)


def _tf_sub_mount_offset_from_mounts_yaml_mm(mounts_doc: Mapping[str, Any]) -> List[List[float]]:
    """
    Returns the active mount "scene_offset" as a rigid transform (mm).

    substrate_mounts.yaml:
      active_mount: <key>
      mounts:
        <key>:
          scene_offset:
            xyz: [..,..,..]   # mm
            rpy_deg: [..,..,..]
    """
    if not isinstance(mounts_doc, Mapping):
        raise ValueError("substrate_mounts.yaml: mounts_doc is not a mapping")

    active = mounts_doc.get("active_mount")
    if not isinstance(active, str) or not active.strip():
        raise ValueError("substrate_mounts.yaml: 'active_mount' missing/empty")
    active = active.strip()

    mounts = mounts_doc.get("mounts")
    if not isinstance(mounts, Mapping) or not mounts:
        raise ValueError("substrate_mounts.yaml: 'mounts' missing/empty")

    m = mounts.get(active)
    if not isinstance(m, Mapping):
        raise KeyError(f"substrate_mounts.yaml: active_mount {active!r} not found in mounts")

    so = m.get("scene_offset")
    if not isinstance(so, Mapping):
        raise ValueError(f"substrate_mounts.yaml: mounts[{active}].scene_offset missing/invalid")

    xyz_mm = _require_list3(so.get("xyz"), f"substrate_mounts.yaml: mounts[{active}].scene_offset.xyz(mm)")
    rpy = _require_list3(so.get("rpy_deg"), f"substrate_mounts.yaml: mounts[{active}].scene_offset.rpy_deg(deg)")
    q = _rpy_deg_to_quat_xyzw(rpy[0], rpy[1], rpy[2])
    return _mat4_from_xyz_quat_mm((xyz_mm[0], xyz_mm[1], xyz_mm[2]), q)


def compute_T_substrate_robot_mount_mm_from_docs(
    *,
    scene_doc: Mapping[str, Any],
    robot_doc: Mapping[str, Any],
    mounts_doc: Optional[Mapping[str, Any]] = None,
    substrate_mount_id: str = "substrate_mount",
    substrate_id: str = "substrate",
) -> List[List[float]]:
    """
    p_substrate = T_substrate_robot_mount * p_robot_mount

    Uses:
      scene.yaml:
        world -> substrate_mount
        substrate_mount -> substrate   (usually identity)
      robot.yaml:
        world -> robot_mount
      substrate_mounts.yaml (OPTIONAL):
        substrate_mount -> active mount scene_offset (mm)

    STRICT:
      - No "fallback resolution" of mounts here. You must pass mounts_doc if you want offsets.
    """
    o_mount = _scene_object_by_id(scene_doc, substrate_mount_id)
    o_substrate = _scene_object_by_id(scene_doc, substrate_id)

    # world -> substrate_mount
    if str(o_mount.get("frame", "world")) != "world":
        raise ValueError("scene.yaml: substrate_mount.frame must be 'world'")
    T_world_sub_mount = _tf_from_scene_obj_meters(o_mount)

    # substrate_mount -> substrate
    if str(o_substrate.get("frame", "")) != substrate_mount_id:
        raise ValueError(f"scene.yaml: substrate.frame must be {substrate_mount_id!r}")
    T_sub_mount_substrate = _tf_from_scene_obj_meters(o_substrate)

    # substrate_mount -> mount_offset (mm, from substrate_mounts.yaml)
    T_sub_mount_mount_offset = _mat4_identity()
    if mounts_doc is not None:
        T_sub_mount_mount_offset = _tf_sub_mount_offset_from_mounts_yaml_mm(mounts_doc)

    # world -> substrate (effective)
    T_world_substrate = _mat4_mul(_mat4_mul(T_world_sub_mount, T_sub_mount_mount_offset), T_sub_mount_substrate)

    # world -> robot_mount
    T_world_robot_mount = _tf_world_robot_mount_from_robot_yaml(robot_doc)

    # substrate <- robot_mount : inv(world->substrate) * (world->robot_mount)
    T_substrate_world = _mat4_inv_rigid(T_world_substrate)
    return _mat4_mul(T_substrate_world, T_world_robot_mount)


def compute_T_substrate_robot_mount_mm_from_paths(
    *,
    scene_yaml_path: str,
    robot_yaml_path: str,
    mounts_yaml_path: str = "",
    strict_mounts: bool = False,
) -> List[List[float]]:
    """
    STRICT mounts resolution:
      - If mounts_yaml_path is provided: load it.
      - Else: mounts_doc=None (no mount offset applied).
      - If strict_mounts=True and mounts_yaml_path is empty: hard error.
    """
    scene_doc = _load_yaml_file(scene_yaml_path)
    robot_doc = _load_yaml_file(robot_yaml_path)

    mpath = str(mounts_yaml_path or "").strip()
    if not mpath:
        if strict_mounts:
            raise ValueError("strict_mounts=True aber mounts_yaml_path ist leer.")
        mounts_doc = None
    else:
        mounts_doc = _load_yaml_file(mpath)

    return compute_T_substrate_robot_mount_mm_from_docs(scene_doc=scene_doc, robot_doc=robot_doc, mounts_doc=mounts_doc)


# ============================================================
# Minimal pose containers used by Draft YAML output
# ============================================================

@dataclass(frozen=True)
class PoseQuat:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class PathSide:
    poses_quat: List[PoseQuat]


@dataclass(frozen=True)
class TrajFkConfig:
    """
    STRICT FK config (KDL).

    - FK is computed in base_link frame (default 'robot_mount').
    - If T_out_from_base_mm is set, FK output is mapped into out_frame:
        p_out = T_out_from_base_mm * p_base
        q_out = R_out_from_base * q_base
    """
    group_name: str = "omron_arm_group"
    base_link: str = "robot_mount"
    ee_link: str = "tcp"

    step_mm: float = 5.0
    max_points: int = 0
    meters_to_mm: float = 1000.0

    # Optional output transform (mm)
    T_out_from_base_mm: Optional[List[List[float]]] = None
    out_frame: Optional[str] = None


class TrajFkBuilder:
    """
    STRICT Traj->FK builder (OFFLINE, no ROS nodes).

    STRICT:
      - No boundary de-duplication anywhere. No synthetic edits of the signal.
      - No legacy TCP draft builder kept here. Use build_tcp_draft_yaml() only.

    Output (TCP YAML with per-segment isolation):
      {
        "version": 1,
        "frame": "...",
        "sides": {side: {"poses_quat":[...]}},
        "segments": {seg: {"sides": {side: {"poses_quat":[...]}}}},
        "meta": {"segment_slices": {seg: {side: [a,b]}}},
        "segments_included": [...]
      }
    """

    DEFAULT_TCP_SEGMENTS: Tuple[str, str] = ("MOVE_RECIPE", "MOVE_RETREAT")

    # ------------------------------------------------------------
    # Robot model (KDL)
    # ------------------------------------------------------------

    @staticmethod
    def build_robot_model_from_urdf_srdf(
        *,
        urdf_path: str = "",
        srdf_path: str = "",
        urdf_xml: str = "",
        srdf_xml: str = "",
    ) -> Dict[str, Any]:
        if urdf_xml and srdf_xml:
            return TrajFkBuilder.build_robot_model_from_urdf_srdf_xml(urdf_xml=urdf_xml, srdf_xml=srdf_xml)
        if not urdf_path or not srdf_path:
            raise ValueError("build_robot_model_from_urdf_srdf: urdf/srdf fehlen (path oder xml).")
        with open(os.path.abspath(os.path.expanduser(urdf_path)), "r", encoding="utf-8") as f:
            uxml = f.read()
        with open(os.path.abspath(os.path.expanduser(srdf_path)), "r", encoding="utf-8") as f:
            sxml = f.read()
        return TrajFkBuilder.build_robot_model_from_urdf_srdf_xml(urdf_xml=uxml, srdf_xml=sxml)

    @staticmethod
    def build_robot_model_from_urdf_srdf_xml(*, urdf_xml: str, srdf_xml: str) -> Dict[str, Any]:
        if not isinstance(urdf_xml, str) or not urdf_xml.strip():
            raise ValueError("TrajFkBuilder(KDL): urdf_xml ist leer")
        if not isinstance(srdf_xml, str) or not srdf_xml.strip():
            raise ValueError("TrajFkBuilder(KDL): srdf_xml ist leer")

        try:
            from urdf_parser_py.urdf import URDF  # type: ignore
            from kdl_parser_py.urdf import treeFromUrdfModel  # type: ignore
        except Exception as e:
            raise RuntimeError(
                "TrajFkBuilder(KDL): Import fehlgeschlagen. Benötigt: urdf_parser_py, kdl_parser_py. "
                f"Details: {e!r}"
            )

        try:
            urdf_model = URDF.from_xml_string(urdf_xml)
        except Exception as e:
            raise RuntimeError(f"TrajFkBuilder(KDL): URDF parsing fehlgeschlagen: {e!r}") from e

        ok, tree = treeFromUrdfModel(urdf_model)
        if not ok or tree is None:
            raise RuntimeError("TrajFkBuilder(KDL): kdl_parser_py treeFromUrdfModel() lieferte kein Tree.")

        return {"_kdl_tree": tree, "_urdf": urdf_model}

    # ------------------------------------------------------------
    # Draft YAML transform (mm, rigid)
    # ------------------------------------------------------------

    @staticmethod
    def transform_draft_yaml(
        draft: Dict[str, Any],
        *,
        T_to_from_mm: List[List[float]],
        out_frame: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Transforms Draft-like TCP YAML.

        STRICT:
          - transforms top-level sides
          - transforms nested segments[seg].sides too (if present)
          - keeps meta.segment_slices unchanged (indices remain valid)
        """
        if not isinstance(draft, dict):
            raise TypeError("transform_draft_yaml: draft must be dict")

        out = dict(draft)

        def _xform_pose_list(poses: Any) -> List[Dict[str, Any]]:
            if not isinstance(poses, list):
                return []
            new_poses: List[Dict[str, Any]] = []
            for p in poses:
                if not isinstance(p, dict):
                    continue
                try:
                    x = float(p.get("x", 0.0))
                    y = float(p.get("y", 0.0))
                    z = float(p.get("z", 0.0))
                    qx = float(p.get("qx", 0.0))
                    qy = float(p.get("qy", 0.0))
                    qz = float(p.get("qz", 0.0))
                    qw = float(p.get("qw", 1.0))
                except Exception:
                    continue

                x2, y2, z2 = _apply_T_to_point_mm(T_to_from_mm, (x, y, z))
                q2 = _apply_T_to_quat(T_to_from_mm, (qx, qy, qz, qw))

                new_poses.append(
                    {
                        "x": x2,
                        "y": y2,
                        "z": z2,
                        "qx": q2[0],
                        "qy": q2[1],
                        "qz": q2[2],
                        "qw": q2[3],
                    }
                )
            return new_poses

        # top-level sides
        sides = out.get("sides")
        if isinstance(sides, dict):
            out_sides: Dict[str, Any] = {}
            for side, sobj in sides.items():
                if not isinstance(sobj, dict):
                    continue
                out_sides[str(side)] = {"poses_quat": _xform_pose_list(sobj.get("poses_quat"))}
            out["sides"] = out_sides

        # nested segments
        segs = out.get("segments")
        if isinstance(segs, dict):
            out_segs: Dict[str, Any] = {}
            for seg_id, seg_obj in segs.items():
                if not isinstance(seg_obj, dict):
                    continue
                ss = seg_obj.get("sides")
                if not isinstance(ss, dict):
                    out_segs[str(seg_id)] = dict(seg_obj)
                    continue
                out_ss: Dict[str, Any] = {}
                for side, sobj in ss.items():
                    if not isinstance(sobj, dict):
                        continue
                    out_ss[str(side)] = {"poses_quat": _xform_pose_list(sobj.get("poses_quat"))}
                out_segs[str(seg_id)] = {"sides": out_ss}
            out["segments"] = out_segs

        if out_frame is not None:
            out["frame"] = str(out_frame)
        return out

    # ------------------------------------------------------------
    # FK -> TCP YAML (STRICT, per-segment isolation)
    # ------------------------------------------------------------

    @staticmethod
    def build_tcp_draft_yaml(
        traj: Union[JTBySegment, Mapping[str, Any]],
        *,
        robot_model: Any,
        cfg: TrajFkConfig,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        frame_id: Optional[str] = None,
        include_segments: Optional[Sequence[str]] = None,
        require_all_segments: bool = True,
    ) -> Dict[str, Any]:
        jt = TrajFkBuilder._coerce_jt_by_segment(traj)

        if robot_model is None:
            raise ValueError("TrajFkBuilder(KDL): robot_model ist None.")
        if not isinstance(cfg.base_link, str) or not cfg.base_link.strip():
            raise ValueError("TrajFkBuilder(KDL): cfg.base_link ist leer.")
        if not isinstance(cfg.ee_link, str) or not cfg.ee_link.strip():
            raise ValueError("TrajFkBuilder(KDL): cfg.ee_link ist leer.")
        if not (isinstance(cfg.step_mm, (int, float)) and float(cfg.step_mm) > 0.0):
            raise ValueError("TrajFkBuilder(KDL): cfg.step_mm muss > 0 sein.")
        if cfg.max_points and int(cfg.max_points) < 2:
            raise ValueError("TrajFkBuilder(KDL): cfg.max_points muss 0 oder >= 2 sein.")

        km = TrajFkBuilder._require_kdl_robot_model(
            robot_model,
            base_link=str(cfg.base_link),
            ee_link=str(cfg.ee_link),
        )

        seg_ids = (
            list(TrajFkBuilder.DEFAULT_TCP_SEGMENTS)
            if include_segments is None
            else [str(s).strip() for s in include_segments if str(s).strip()]
        )
        if not seg_ids:
            raise ValueError("TrajFkBuilder.build_tcp_draft_yaml: include_segments ist leer.")

        missing = [sid for sid in seg_ids if sid not in jt.segments]
        if missing and require_all_segments:
            raise KeyError(f"TrajFkBuilder(KDL): JTBySegment fehlt Segmente: {missing!r}")

        global_sides: Dict[str, Dict[str, Any]] = {}
        out_segments: Dict[str, Dict[str, Any]] = {}
        seg_slices: Dict[str, Dict[str, List[int]]] = {}

        def _ensure_global_side(side: str) -> List[Dict[str, Any]]:
            side = str(side or default_side).strip() or default_side
            if side not in global_sides:
                global_sides[side] = {"poses_quat": []}
            pq = global_sides[side].get("poses_quat")
            if not isinstance(pq, list):
                pq = []
                global_sides[side]["poses_quat"] = pq
            return pq  # type: ignore[return-value]

        for seg_id in seg_ids:
            if seg_id not in jt.segments:
                continue

            seg = jt.segments[seg_id]

            side = default_side
            if segment_to_side is not None:
                if seg_id not in segment_to_side:
                    raise KeyError(f"TrajFkBuilder(KDL): segment_to_side hat keinen Eintrag für seg_id={seg_id!r}")
                side = str(segment_to_side[seg_id] or default_side).strip() or default_side

            raw_poses = TrajFkBuilder._fk_segment_raw_kdl(seg_id=seg_id, seg=seg, km=km, cfg=cfg)
            if len(raw_poses) < 2:
                raise RuntimeError(f"TrajFkBuilder(KDL): FK ergab <2 Posen für Segment {seg_id!r}.")

            sampled = TrajFkBuilder._resample_posequats_by_step_mm(
                raw_poses,
                step_mm=float(cfg.step_mm),
                max_points=int(cfg.max_points or 0),
            )
            if len(sampled) < 2:
                raise RuntimeError(f"TrajFkBuilder(KDL): Resampling ergab <2 Posen für Segment {seg_id!r}.")

            if cfg.T_out_from_base_mm is not None:
                T = cfg.T_out_from_base_mm
                mapped: List[PoseQuat] = []
                for p in sampled:
                    x2, y2, z2 = _apply_T_to_point_mm(T, (p.x, p.y, p.z))
                    q2 = _apply_T_to_quat(T, (p.qx, p.qy, p.qz, p.qw))
                    mapped.append(PoseQuat(x=x2, y=y2, z=z2, qx=q2[0], qy=q2[1], qz=q2[2], qw=q2[3]))
                sampled = mapped

            seg_local: List[Dict[str, Any]] = [
                {
                    "x": float(p.x),
                    "y": float(p.y),
                    "z": float(p.z),
                    "qx": float(p.qx),
                    "qy": float(p.qy),
                    "qz": float(p.qz),
                    "qw": float(p.qw),
                }
                for p in sampled
            ]

            # STRICT: no boundary edits, ever
            gl = _ensure_global_side(side)
            a = len(gl)
            gl.extend(seg_local)
            b = len(gl)

            out_segments.setdefault(str(seg_id), {"sides": {}})
            out_segments[str(seg_id)]["sides"][str(side)] = {"poses_quat": list(seg_local)}

            seg_slices.setdefault(str(seg_id), {})
            seg_slices[str(seg_id)][str(side)] = [int(a), int(b)]

        if not global_sides:
            raise RuntimeError("TrajFkBuilder(KDL): keine TCP Posen erzeugt (global_sides leer).")

        fr = frame_id if frame_id is not None else (cfg.out_frame if cfg.out_frame else cfg.base_link)

        return {
            "version": 1,
            "frame": str(fr),
            "sides": global_sides,
            "segments": out_segments,
            "meta": {"segment_slices": seg_slices},
            "segments_included": list(seg_ids),
        }

    # ------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------

    @staticmethod
    def _coerce_jt_by_segment(traj: Union[JTBySegment, Mapping[str, Any]]) -> JTBySegment:
        if isinstance(traj, JTBySegment):
            return traj
        dd = _dict(dict(traj))
        fn = getattr(JTBySegment, "from_yaml_dict", None)
        if not callable(fn):
            raise RuntimeError("JTBySegment.from_yaml_dict() fehlt (Projekt-API passt nicht).")
        return fn(dd)

    @staticmethod
    def _require_kdl_robot_model(robot_model: Any, *, base_link: str, ee_link: str) -> Dict[str, Any]:
        tree = None
        if isinstance(robot_model, dict):
            tree = robot_model.get("_kdl_tree")
        else:
            tree = getattr(robot_model, "_kdl_tree", None)

        if tree is None:
            raise ValueError("TrajFkBuilder(KDL): robot_model enthält keinen _kdl_tree")

        try:
            import PyKDL  # type: ignore
        except Exception as e:
            raise RuntimeError(f"TrajFkBuilder(KDL): Import PyKDL fehlgeschlagen: {e!r}") from e

        chain = tree.getChain(str(base_link), str(ee_link))  # type: ignore[attr-defined]
        if chain is None:
            raise RuntimeError(
                f"TrajFkBuilder(KDL): getChain({base_link!r}->{ee_link!r}) lieferte None "
                "(Frames/Joints nicht im selben Baum?)"
            )

        try:
            nj = int(chain.getNrOfJoints())
        except Exception:
            nj = -1
        if nj <= 0:
            raise RuntimeError(
                f"TrajFkBuilder(KDL): KDL chain getChain({base_link!r}->{ee_link!r}) hat keine Joints (nj={nj}). "
                "Base/EE Link stimmen vermutlich nicht."
            )

        fk = PyKDL.ChainFkSolverPos_recursive(chain)
        return {"tree": tree, "chain": chain, "fk": fk}

    @staticmethod
    def _fk_segment_raw_kdl(*, seg_id: str, seg: Any, km: Dict[str, Any], cfg: TrajFkConfig) -> List[PoseQuat]:
        joint_names = getattr(seg, "joint_names", None)
        if not isinstance(joint_names, list) or not joint_names:
            raise ValueError(f"JT segment {seg_id!r}: joint_names fehlt/leer")

        points = getattr(seg, "points", None)
        if not isinstance(points, list) or not points:
            raise ValueError(f"JT segment {seg_id!r}: points fehlt/leer")

        try:
            import PyKDL  # type: ignore
        except Exception as e:
            raise RuntimeError(f"TrajFkBuilder(KDL): Import PyKDL fehlgeschlagen: {e!r}") from e

        fk = km["fk"]
        chain = km["chain"]

        out: List[PoseQuat] = []
        for pt in points:
            positions = getattr(pt, "positions", None)
            if not isinstance(positions, list) or len(positions) != len(joint_names):
                raise ValueError(f"JT segment {seg_id!r}: point.positions hat falsche Länge")

            qj = PyKDL.JntArray(chain.getNrOfJoints())
            for i, val in enumerate(positions):
                qj[i] = float(val)

            frame = PyKDL.Frame()
            fk.JntToCart(qj, frame)

            x_mm = float(frame.p[0]) * float(cfg.meters_to_mm)
            y_mm = float(frame.p[1]) * float(cfg.meters_to_mm)
            z_mm = float(frame.p[2]) * float(cfg.meters_to_mm)

            rot = frame.M
            R = [
                [float(rot[0, 0]), float(rot[0, 1]), float(rot[0, 2])],
                [float(rot[1, 0]), float(rot[1, 1]), float(rot[1, 2])],
                [float(rot[2, 0]), float(rot[2, 1]), float(rot[2, 2])],
            ]
            qx, qy, qz, qw = _rot3_to_quat(R)

            out.append(PoseQuat(x=x_mm, y=y_mm, z=z_mm, qx=qx, qy=qy, qz=qz, qw=qw))

        return out

    @staticmethod
    def _resample_posequats_by_step_mm(poses: List[PoseQuat], *, step_mm: float, max_points: int = 0) -> List[PoseQuat]:
        if not poses or len(poses) < 2:
            return list(poses)

        out: List[PoseQuat] = [poses[0]]

        def dist(a: PoseQuat, b: PoseQuat) -> float:
            dx = b.x - a.x
            dy = b.y - a.y
            dz = b.z - a.z
            return math.sqrt(dx * dx + dy * dy + dz * dz)

        def slerp(
            qa: Tuple[float, float, float, float],
            qb: Tuple[float, float, float, float],
            t: float,
        ) -> Tuple[float, float, float, float]:
            qa = _quat_normalize(qa)
            qb = _quat_normalize(qb)
            dot = _quat_dot(qa, qb)
            if dot < 0.0:
                qb = (-qb[0], -qb[1], -qb[2], -qb[3])
                dot = -dot
            dot = _clamp(dot, -1.0, 1.0)

            if dot > 0.9995:
                q = (
                    qa[0] + (qb[0] - qa[0]) * t,
                    qa[1] + (qb[1] - qa[1]) * t,
                    qa[2] + (qb[2] - qa[2]) * t,
                    qa[3] + (qb[3] - qa[3]) * t,
                )
                return _quat_normalize(q)

            theta0 = math.acos(dot)
            sin0 = math.sin(theta0)
            theta = theta0 * t
            sin_t = math.sin(theta)
            s0 = math.cos(theta) - dot * sin_t / sin0
            s1 = sin_t / sin0
            return (
                qa[0] * s0 + qb[0] * s1,
                qa[1] * s0 + qb[1] * s1,
                qa[2] * s0 + qb[2] * s1,
                qa[3] * s0 + qb[3] * s1,
            )

        accum = 0.0
        next_d = float(step_mm)

        for i in range(len(poses) - 1):
            a = poses[i]
            b = poses[i + 1]
            seg_len = dist(a, b)
            if seg_len <= 1e-12:
                continue

            while accum + seg_len >= next_d:
                alpha = (next_d - accum) / seg_len

                x = a.x + (b.x - a.x) * alpha
                y = a.y + (b.y - a.y) * alpha
                z = a.z + (b.z - a.z) * alpha

                q = slerp((a.qx, a.qy, a.qz, a.qw), (b.qx, b.qy, b.qz, b.qw), alpha)
                out.append(PoseQuat(x=x, y=y, z=z, qx=q[0], qy=q[1], qz=q[2], qw=q[3]))

                if max_points and max_points > 0 and len(out) >= int(max_points):
                    return out

                next_d += float(step_mm)

            accum += seg_len

        if out[-1] != poses[-1]:
            out.append(poses[-1])
        return out

    @staticmethod
    def _draft_to_yaml_dict(d: Draft) -> Dict[str, Any]:
        sides_obj = getattr(d, "sides", None)
        if not isinstance(sides_obj, dict):
            raise TypeError("Draft.sides ist nicht dict (unerwartet).")

        sides_out: Dict[str, Any] = {}
        for side, side_obj in sides_obj.items():
            poses_obj = getattr(side_obj, "poses_quat", None)
            if not isinstance(poses_obj, list):
                raise TypeError(f"Draft.sides[{side!r}].poses_quat ist nicht list.")
            poses: List[Dict[str, Any]] = []
            for p in poses_obj:
                poses.append(
                    {
                        "x": float(getattr(p, "x")),
                        "y": float(getattr(p, "y")),
                        "z": float(getattr(p, "z")),
                        "qx": float(getattr(p, "qx")),
                        "qy": float(getattr(p, "qy")),
                        "qz": float(getattr(p, "qz")),
                        "qw": float(getattr(p, "qw")),
                    }
                )
            sides_out[str(side)] = {"poses_quat": poses}

        ver = int(getattr(d, "version", 1) or 1)
        return {"version": ver, "sides": sides_out}
