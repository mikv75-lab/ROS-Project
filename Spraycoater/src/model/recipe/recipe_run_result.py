# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple, List

import os
import yaml
import functools


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


# ============================================================
# Minimal rigid transform helpers (mm)
# ============================================================


def _rpy_deg_to_quat_xyzw(r: float, p: float, y: float) -> Tuple[float, float, float, float]:
    import math

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

    n = (qx * qx + qy * qy + qz * qz + qw * qw) ** 0.5
    if n <= 0.0:
        raise ValueError("quat norm is zero")
    return (qx / n, qy / n, qz / n, qw / n)


def _quat_to_rot3(q: Tuple[float, float, float, float]) -> list[list[float]]:
    import math

    qx, qy, qz, qw = q
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n <= 0.0:
        raise ValueError("quat norm is zero")
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

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


def _mat4_from_xyz_quat_mm(
    xyz_mm: Tuple[float, float, float],
    q: Tuple[float, float, float, float],
) -> list[list[float]]:
    R = _quat_to_rot3(q)
    tx, ty, tz = xyz_mm
    return [
        [R[0][0], R[0][1], R[0][2], float(tx)],
        [R[1][0], R[1][1], R[1][2], float(ty)],
        [R[2][0], R[2][1], R[2][2], float(tz)],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _mat4_mul(A: list[list[float]], B: list[list[float]]) -> list[list[float]]:
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


def _mat4_inv_rigid(T: list[list[float]]) -> list[list[float]]:
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


# ============================================================
# YAML loading + scene/mount helpers
# ============================================================


def _load_yaml_file(path: str) -> Dict[str, Any]:
    if not path:
        raise ValueError("YAML path is empty")
    p = os.path.abspath(os.path.expanduser(path))
    if not os.path.exists(p):
        raise FileNotFoundError(p)
    try:
        mtime = float(os.path.getmtime(p))
    except Exception:
        mtime = 0.0
    return _load_yaml_file_cached(p, mtime)


@functools.lru_cache(maxsize=64)
def _load_yaml_file_cached(abs_path: str, mtime: float) -> Dict[str, Any]:
    _ = float(mtime)  # part of cache key; invalidates when file changes
    with open(abs_path, "r", encoding="utf-8") as f:
        v = yaml.safe_load(f)
        return v if isinstance(v, dict) else {}


def _scene_object_by_id(scene_doc: Dict[str, Any], oid: str) -> Dict[str, Any]:
    objs = scene_doc.get("scene_objects")
    if not isinstance(objs, list):
        raise ValueError("scene.yaml: 'scene_objects' missing or not list")
    for o in objs:
        if isinstance(o, dict) and str(o.get("id", "")) == oid:
            return o
    raise KeyError(f"scene.yaml: object id={oid!r} not found")


def _tf_from_scene_obj_meters(scene_obj: Dict[str, Any]) -> list[list[float]]:
    pos_m = _require_list3(scene_obj.get("position"), "scene_obj.position(m)")
    rpy = _require_list3(scene_obj.get("rpy_deg"), "scene_obj.rpy_deg(deg)")
    pos_mm = (pos_m[0] * 1000.0, pos_m[1] * 1000.0, pos_m[2] * 1000.0)
    q = _rpy_deg_to_quat_xyzw(rpy[0], rpy[1], rpy[2])
    return _mat4_from_xyz_quat_mm(pos_mm, q)


def _tf_world_robot_mount_from_robot_yaml(robot_doc: Dict[str, Any]) -> list[list[float]]:
    blk = _require_dict(robot_doc.get("world_to_robot_mount"), "robot.yaml.world_to_robot_mount")
    xyz_m = _require_list3(blk.get("xyz"), "robot.yaml.world_to_robot_mount.xyz(m)")
    rpy = _require_list3(blk.get("rpy_deg"), "robot.yaml.world_to_robot_mount.rpy_deg(deg)")
    xyz_mm = (xyz_m[0] * 1000.0, xyz_m[1] * 1000.0, xyz_m[2] * 1000.0)
    q = _rpy_deg_to_quat_xyzw(rpy[0], rpy[1], rpy[2])
    return _mat4_from_xyz_quat_mm(xyz_mm, q)


def _tf_substrate_mount_to_substrate_from_mounts_yaml(*, mounts_doc: Dict[str, Any]) -> list[list[float]]:
    active = str(mounts_doc.get("active_mount") or "").strip()
    if not active:
        raise ValueError("substrate_mounts.yaml: active_mount missing/empty")
    mounts = _require_dict(mounts_doc.get("mounts"), "substrate_mounts.yaml.mounts")
    m = _require_dict(mounts.get(active), f"substrate_mounts.yaml.mounts[{active!r}]")
    so = _require_dict(m.get("scene_offset"), f"substrate_mounts.yaml.mounts[{active!r}].scene_offset")
    xyz = _require_list3(so.get("xyz"), f"scene_offset.xyz(mm) for mount {active!r}")
    rpy = _require_list3(so.get("rpy_deg"), f"scene_offset.rpy_deg(deg) for mount {active!r}")
    q = _rpy_deg_to_quat_xyzw(rpy[0], rpy[1], rpy[2])
    return _mat4_from_xyz_quat_mm((xyz[0], xyz[1], xyz[2]), q)


def _compute_T_substrate_robot_mount_mm(
    *,
    scene_yaml_path: str,
    robot_yaml_path: str,
    mounts_yaml_path: str,
    substrate_mount_id: str = "substrate_mount",
) -> list[list[float]]:
    scene_doc = _load_yaml_file(scene_yaml_path)
    robot_doc = _load_yaml_file(robot_yaml_path)
    mounts_doc = _load_yaml_file(mounts_yaml_path)

    o_mount = _scene_object_by_id(scene_doc, substrate_mount_id)
    if str(o_mount.get("frame", "world")) != "world":
        raise ValueError("scene.yaml: substrate_mount.frame must be 'world' for this helper")
    T_world_sub_mount = _tf_from_scene_obj_meters(o_mount)

    T_sub_mount_substrate = _tf_substrate_mount_to_substrate_from_mounts_yaml(mounts_doc=mounts_doc)
    T_world_substrate = _mat4_mul(T_world_sub_mount, T_sub_mount_substrate)

    T_world_robot_mount = _tf_world_robot_mount_from_robot_yaml(robot_doc)

    T_substrate_world = _mat4_inv_rigid(T_world_substrate)
    return _mat4_mul(T_substrate_world, T_world_robot_mount)


# ============================================================
# RunResult (STRICT, no legacy/fallback)
# ============================================================


@dataclass
class RunResult:
    """
    STRICT run result container.
    """

    urdf_xml: str = ""
    srdf_xml: str = ""

    planned_run: Dict[str, Any] = field(default_factory=dict)
    executed_run: Dict[str, Any] = field(default_factory=dict)

    fk_meta: Dict[str, Any] = field(default_factory=dict)
    eval: Dict[str, Any] = field(default_factory=dict)

    valid: bool = True
    invalid_reason: str = ""

    TCP_TARGET_FRAME: str = "substrate"
    DEFAULT_RECIPE_SEGMENT: str = "MOVE_RECIPE"

    _attached_recipe: Any = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.urdf_xml = str(self.urdf_xml or "")
        self.srdf_xml = str(self.srdf_xml or "")

        self.planned_run = _dict(self.planned_run)
        self.executed_run = _dict(self.executed_run)

        self.planned_run.setdefault("traj", {})
        self.planned_run.setdefault("tcp", {})
        self.executed_run.setdefault("traj", {})
        self.executed_run.setdefault("tcp", {})

        self.fk_meta = _dict(self.fk_meta)
        self.eval = _dict(self.eval)

        self.valid = bool(self.valid)
        self.invalid_reason = str(self.invalid_reason or "")

        if self.TCP_TARGET_FRAME != "substrate":
            raise ValueError("RunResult: TCP_TARGET_FRAME must be 'substrate' (strict invariant).")

    # ------------------------------------------------------------
    # recipe attachment (STRICT)
    # ------------------------------------------------------------

    def attach_recipe(self, recipe: Any) -> None:
        if recipe is None:
            raise ValueError("RunResult.attach_recipe: recipe is None (strict)")
        self._attached_recipe = recipe

    def attach_recipe_context(self, recipe: Any) -> None:
        self.attach_recipe(recipe)

    def _require_attached_recipe(self) -> Any:
        r = self._attached_recipe
        if r is None:
            raise RuntimeError("RunResult: recipe not attached. Call rr.attach_recipe(recipe) first.")
        return r

    # ------------------------------------------------------------
    # basic setters
    # ------------------------------------------------------------

    def set_robot_description(self, *, urdf_xml: str, srdf_xml: str) -> None:
        self.urdf_xml = str(urdf_xml or "")
        self.srdf_xml = str(srdf_xml or "")

    def set_planned(self, *, traj: Dict[str, Any] | None = None, tcp: Dict[str, Any] | None = None) -> None:
        if traj is not None:
            self.planned_run["traj"] = _dict(traj)
        if tcp is not None:
            self.planned_run["tcp"] = _dict(tcp)

    def set_executed(self, *, traj: Dict[str, Any] | None = None, tcp: Dict[str, Any] | None = None) -> None:
        if traj is not None:
            self.executed_run["traj"] = _dict(traj)
        if tcp is not None:
            self.executed_run["tcp"] = _dict(tcp)

    def set_fk_meta(self, meta: Dict[str, Any] | None) -> None:
        self.fk_meta = _dict(meta)

    def invalidate(self, reason: str) -> None:
        self.valid = False
        self.invalid_reason = str(reason or "invalid")

    # ------------------------------------------------------------
    # counters
    # ------------------------------------------------------------

    @staticmethod
    def _count_tcp_poses(doc: Any) -> int:
        if not isinstance(doc, dict):
            return 0
        sides = doc.get("sides")
        if not isinstance(sides, dict):
            return 0
        n = 0
        for _, s in sides.items():
            if isinstance(s, dict):
                pq = s.get("poses_quat")
                if isinstance(pq, list):
                    n += len(pq)
        return int(n)

    @staticmethod
    def _count_traj_points(doc: Any) -> int:
        if not isinstance(doc, dict):
            return 0
        segs = doc.get("segments")
        if not isinstance(segs, dict):
            return 0
        total = 0
        for _, s in segs.items():
            if not isinstance(s, dict):
                continue
            pts = s.get("points")
            if isinstance(pts, list):
                total += len(pts)
                continue
            jt = s.get("joint_trajectory")
            if isinstance(jt, dict):
                pts2 = jt.get("points")
                if isinstance(pts2, list):
                    total += len(pts2)
        return int(total)

    # ------------------------------------------------------------
    # Speed metric helpers (display-only; no scoring)
    # ------------------------------------------------------------

    @staticmethod
    def _duration_s_from_tfs(tfs: Any) -> float:
        if isinstance(tfs, dict):
            sec = int(tfs.get("sec", 0))
            nsec = int(tfs.get("nanosec", 0))
            return float(sec) + float(nsec) * 1e-9
        if isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
            sec = int(tfs[0])
            nsec = int(tfs[1])
            return float(sec) + float(nsec) * 1e-9
        return 0.0

    @classmethod
    def _segment_duration_s_from_traj(cls, traj_doc: Dict[str, Any], seg_name: str) -> Optional[float]:
        segs = traj_doc.get("segments")
        if not isinstance(segs, dict):
            return None
        seg = segs.get(seg_name)
        if not isinstance(seg, dict):
            return None
        pts = seg.get("points")
        if not isinstance(pts, list) or len(pts) < 2:
            jt = seg.get("joint_trajectory")
            if isinstance(jt, dict):
                pts = jt.get("points")
        if not isinstance(pts, list) or len(pts) < 2:
            return None

        t0 = cls._duration_s_from_tfs(_dict(pts[0]).get("time_from_start"))
        t1 = cls._duration_s_from_tfs(_dict(pts[-1]).get("time_from_start"))
        dt = float(t1 - t0)
        if dt <= 0.0:
            return None
        return dt

    @staticmethod
    def _extract_xyz_mm_from_pose(p: Any) -> Optional[Tuple[float, float, float]]:
        """
        STRICT support for your PoseQuat YAML entries:

          {x: mm, y: mm, z: mm, qx:..., qy:..., qz:..., qw:...}

        plus simple list-like [x,y,z].
        """
        if isinstance(p, (list, tuple)) and len(p) >= 3:
            try:
                return (float(p[0]), float(p[1]), float(p[2]))
            except Exception:
                return None

        if isinstance(p, dict):
            # --- exact PoseQuat dict used in planned_tcp/executed_tcp
            if ("x" in p) and ("y" in p) and ("z" in p):
                try:
                    return (float(p["x"]), float(p["y"]), float(p["z"]))
                except Exception:
                    return None

            # (keep minimal extra key for internal cases)
            for k in ("xyz_mm", "xyz", "position", "p"):
                v = p.get(k)
                if isinstance(v, (list, tuple)) and len(v) == 3:
                    try:
                        return (float(v[0]), float(v[1]), float(v[2]))
                    except Exception:
                        return None

        return None

    @classmethod
    def _tcp_segment_poses_quat(cls, tcp_doc: Dict[str, Any], seg_name: str) -> Optional[List[Any]]:
        """
        Your current TCP YAML (planned_tcp / executed_tcp) is:

          version: 1
          sides:
            top:
              poses_quat: [ {x,y,z,qx,qy,qz,qw}, ... ]

        This helper returns the first side with >=2 poses_quat.
        (Segments are not expected in the TCP YAML.)
        """
        sides = tcp_doc.get("sides")
        if isinstance(sides, dict):
            for _, s in sides.items():
                if not isinstance(s, dict):
                    continue
                pq = s.get("poses_quat")
                if isinstance(pq, list) and len(pq) >= 2:
                    return pq
        return None

    @classmethod
    def _tcp_length_mm_from_poses(cls, poses_quat: List[Any]) -> Optional[float]:
        import math

        prev = None
        L = 0.0
        n_valid = 0

        for p in poses_quat:
            xyz = cls._extract_xyz_mm_from_pose(p)
            if xyz is None:
                continue
            n_valid += 1
            if prev is not None:
                dx = xyz[0] - prev[0]
                dy = xyz[1] - prev[1]
                dz = xyz[2] - prev[2]
                L += math.sqrt(dx * dx + dy * dy + dz * dz)
            prev = xyz

        if n_valid < 2:
            return None
        if L <= 0.0:
            return None
        return float(L)



    def _compute_avg_speed_recipe_segment(
        self,
        *,
        recipe: Any,
        which: str,
        seg_name: str,
    ) -> Dict[str, Any]:
        which = str(which or "").strip().lower()
        if which not in ("planned", "executed"):
            return {"ok": False, "reason": f"invalid_which:{which!r}"}

        speed_set = recipe.parameters['speed_mm_s']
        if speed_set is None:
            return {"ok": False, "reason": "missing_speed_setpoint(speed_mm_s)"}

        run_doc = _dict(self.planned_run) if which == "planned" else _dict(self.executed_run)
        traj_doc = _dict(run_doc.get("traj"))
        tcp_doc = _dict(run_doc.get("tcp"))

        dt = self._segment_duration_s_from_traj(traj_doc, seg_name)
        if dt is None:
            return {"ok": False, "reason": f"missing_segment_duration(traj:{seg_name})", "speed_set_mm_s": float(speed_set)}

        pq = self._tcp_segment_poses_quat(tcp_doc, seg_name)
        if pq is None:
            return {"ok": False, "reason": f"missing_tcp_poses_quat(tcp:{seg_name})", "speed_set_mm_s": float(speed_set)}

        L = self._tcp_length_mm_from_poses(pq)
        if L is None:
            return {"ok": False, "reason": f"tcp_length_invalid_or_zero(tcp:{seg_name})", "speed_set_mm_s": float(speed_set)}

        v = float(L / dt) if dt > 0.0 else 0.0
        pct = float(100.0 * (v / speed_set)) if speed_set > 0.0 else 0.0

        return {
            "ok": True,
            "speed_set_mm_s": float(speed_set),
            "avg_speed_mm_s": float(v),
            "speed_percent": float(pct),
            "seg_name": str(seg_name),
            "which": str(which),
            "duration_s": float(dt),
            "path_len_mm": float(L),
        }

    # ------------------------------------------------------------
    # report formatting
    # ------------------------------------------------------------

    @staticmethod
    def _fmt_float(v: Any, nd: int) -> str:
        try:
            return f"{float(v):.{nd}f}"
        except Exception:
            return "0.000" if nd >= 3 else "0.0"

    @staticmethod
    def _fmt_bool_yesno(v: Any) -> str:
        return "yes" if bool(v) else "no"

    @staticmethod
    def _created_local_str() -> str:
        dt = __import__("datetime").datetime.now().astimezone()
        return dt.strftime("%Y-%m-%d %H:%M:%S.") + f"{int(dt.microsecond / 1000):03d}"

    def report_text(self, *, title: str = "=== CURRENT RUN (Live) ===") -> str:
        ev = _dict(self.eval)

        sel = _dict(ev.get("selection"))
        recipe_id = sel.get("recipe", None)
        tool = sel.get("tool", None)
        substrate = sel.get("substrate", None)
        mount = sel.get("mount", None)

        created = self._created_local_str()

        pl_traj = _dict(_dict(self.planned_run).get("traj"))
        ex_traj = _dict(_dict(self.executed_run).get("traj"))
        pl_tcp = _dict(_dict(self.planned_run).get("tcp"))
        ex_tcp = _dict(_dict(self.executed_run).get("tcp"))

        pl_traj_points = self._count_traj_points(pl_traj)
        ex_traj_points = self._count_traj_points(ex_traj)
        pl_tcp_samples = self._count_tcp_poses(pl_tcp)
        ex_tcp_samples = self._count_tcp_poses(ex_tcp)

        overall_valid = bool(ev.get("valid", False))
        reason = str(ev.get("invalid_reason") or "").strip()
        reason_txt = "no" if overall_valid else (reason if reason else "yes")

        thr = float(ev.get("threshold", 90.0))
        overall_score = float(ev.get("score", 0.0))

        pl_ok = bool(ev.get("planned_valid", False))
        ex_ok = bool(ev.get("executed_valid", False))
        pl_score = float(ev.get("planned_score", 0.0))
        ex_score = float(ev.get("executed_score", 0.0))

        pl_sum = _dict(_dict(ev.get("planned")).get("summary"))
        ex_sum = _dict(_dict(ev.get("executed")).get("summary"))

        def gsum(d: Dict[str, Any], key: str, default: Optional[float] = None) -> Optional[float]:
            if not isinstance(d, dict) or key not in d:
                return default
            try:
                v = float(d.get(key))
            except Exception:
                return default
            return v

        pl_pos_mean = gsum(pl_sum, "mean_l2_mm", 0.0) or 0.0
        pl_pos_p95 = gsum(pl_sum, "p95_l2_mm", 0.0) or 0.0
        pl_pos_worst = gsum(pl_sum, "max_l2_mm", 0.0) or 0.0

        ex_pos_mean = gsum(ex_sum, "mean_l2_mm", 0.0) or 0.0
        ex_pos_p95 = gsum(ex_sum, "p95_l2_mm", 0.0) or 0.0
        ex_pos_worst = gsum(ex_sum, "max_l2_mm", 0.0) or 0.0

        pl_ori_mean = gsum(pl_sum, "mean_angle_deg", 0.0) or 0.0
        pl_ori_p95 = gsum(pl_sum, "p95_angle_deg", 0.0) or 0.0
        pl_ori_worst = gsum(pl_sum, "max_angle_deg", 0.0) or 0.0

        ex_ori_mean = gsum(ex_sum, "mean_angle_deg", 0.0) or 0.0
        ex_ori_p95 = gsum(ex_sum, "p95_angle_deg", 0.0) or 0.0
        ex_ori_worst = gsum(ex_sum, "max_angle_deg", 0.0) or 0.0

        pl_z_mean = gsum(pl_sum, "mean_abs_dz_mm", 0.0) or 0.0
        pl_z_p95 = gsum(pl_sum, "p95_abs_dz_mm", 0.0) or 0.0
        pl_z_worst = gsum(pl_sum, "max_abs_dz_mm", 0.0) or 0.0

        ex_z_mean = gsum(ex_sum, "mean_abs_dz_mm", 0.0) or 0.0
        ex_z_p95 = gsum(ex_sum, "p95_abs_dz_mm", 0.0) or 0.0
        ex_z_worst = gsum(ex_sum, "max_abs_dz_mm", 0.0) or 0.0

        pl_delta_L = gsum(pl_sum, "delta_L_percent", 0.0) or 0.0
        ex_delta_L = gsum(ex_sum, "delta_L_percent", 0.0) or 0.0

        sp = gsum(ex_sum, "speed_set_mm_s", None)
        if sp is None:
            sp = gsum(pl_sum, "speed_set_mm_s", None)

        pl_v = gsum(pl_sum, "avg_speed_mm_s", None)
        ex_v = gsum(ex_sum, "avg_speed_mm_s", None)
        pl_vpct = gsum(pl_sum, "speed_percent", None)
        ex_vpct = gsum(ex_sum, "speed_percent", None)

        pl_reason = str(pl_sum.get("speed_reason") or "").strip()
        ex_reason = str(ex_sum.get("speed_reason") or "").strip()

        show_speed_block = (
            (sp is not None)
            or bool(pl_reason)
            or bool(ex_reason)
            or (pl_v is not None)
            or (ex_v is not None)
        )

        lines: list[str] = []
        lines.append(str(title or "").strip())
        lines.append(f"Recipe:    {recipe_id if recipe_id is not None else 'None'}")
        lines.append(f"created:   {created}")
        lines.append(f"Valid:             {self._fmt_bool_yesno(overall_valid)}")
        lines.append(f"Reason:            {reason_txt}")
        lines.append("")
        lines.append(f"Tool:      {tool if tool is not None else '-'}")
        lines.append(f"Substrate: {substrate if substrate is not None else '-'}")
        lines.append(f"Mount:     {mount if mount is not None else '-'}")
        lines.append("")

        lines.append("Counts (planned | executed):")
        lines.append(f"  Trajectory points: {pl_traj_points} | {ex_traj_points}")
        lines.append(f"  TCP samples:       {pl_tcp_samples} | {ex_tcp_samples}")
        lines.append("")

        lines.append("Deviations vs. draft (planned | executed):")
        lines.append(
            f"  Position (XYZ) mean:    {self._fmt_float(pl_pos_mean, 3)} mm | {self._fmt_float(ex_pos_mean, 3)} mm"
        )
        lines.append(
            f"  Position (XYZ) p95:     {self._fmt_float(pl_pos_p95, 3)} mm | {self._fmt_float(ex_pos_p95, 3)} mm"
        )
        lines.append(
            f"  Position (XYZ) worst:   {self._fmt_float(pl_pos_worst, 3)} mm | {self._fmt_float(ex_pos_worst, 3)} mm"
        )
        lines.append("")
        lines.append(
            f"  Orientation (RPY) mean: {self._fmt_float(pl_ori_mean, 3)} deg | {self._fmt_float(ex_ori_mean, 3)} deg"
        )
        lines.append(
            f"  Orientation (RPY) p95:  {self._fmt_float(pl_ori_p95, 3)} deg | {self._fmt_float(ex_ori_p95, 3)} deg"
        )
        lines.append(
            f"  Orientation (RPY) worst:{self._fmt_float(pl_ori_worst, 3)} deg | {self._fmt_float(ex_ori_worst, 3)} deg"
        )
        lines.append("")
        lines.append(
            f"  Stand-off (Z) mean:     {self._fmt_float(pl_z_mean, 3)} mm | {self._fmt_float(ex_z_mean, 3)} mm"
        )
        lines.append(
            f"  Stand-off (Z) p95:      {self._fmt_float(pl_z_p95, 3)} mm | {self._fmt_float(ex_z_p95, 3)} mm"
        )
        lines.append(
            f"  Stand-off (Z) worst:    {self._fmt_float(pl_z_worst, 3)} mm | {self._fmt_float(ex_z_worst, 3)} mm"
        )
        lines.append("")
        lines.append(f"Path length delta:  {self._fmt_float(pl_delta_L, 3)} % | {self._fmt_float(ex_delta_L, 3)} %")

        if show_speed_block:
            lines.append("")
            lines.append("Avg speed (recipe segment) (planned | executed):")
            if sp is not None:
                lines.append(f"  Target:   {self._fmt_float(sp, 1)} mm/s")
            else:
                lines.append("  Target:   -")

            if (pl_v is not None) and (pl_vpct is not None):
                pl_txt = f"{self._fmt_float(pl_v, 1)} mm/s ({self._fmt_float(pl_vpct, 1)} %)"
            else:
                pl_txt = f"- (reason: {pl_reason})" if pl_reason else "-"

            if (ex_v is not None) and (ex_vpct is not None):
                ex_txt = f"{self._fmt_float(ex_v, 1)} mm/s ({self._fmt_float(ex_vpct, 1)} %)"
            else:
                ex_txt = f"- (reason: {ex_reason})" if ex_reason else "-"

            lines.append(f"  Achieved: {pl_txt} | {ex_txt}")

        lines.append("")
        lines.append(f"Eval threshold:    {self._fmt_float(thr, 3)}")
        lines.append(f"Mode valid:        {self._fmt_bool_yesno(pl_ok)} | {self._fmt_bool_yesno(ex_ok)}")
        lines.append(f"Mode score:        {self._fmt_float(pl_score, 3)} / 100 | {self._fmt_float(ex_score, 3)} / 100")
        lines.append(f"Overall score:     {self._fmt_float(overall_score, 3)} / 100")

        return "\n".join(lines).rstrip()

    # ------------------------------------------------------------
    # eval persistence (STRICT)
    # ------------------------------------------------------------

    def set_eval(self, eval_dict: Dict[str, Any] | None) -> None:
        self.eval = _dict(eval_dict)

        pl_tcp_doc = _dict(self.planned_run.get("tcp"))
        ex_tcp_doc = _dict(self.executed_run.get("tcp"))

        if self.fk_meta:
            pl_tcp_doc["fk_meta"] = dict(self.fk_meta)
            ex_tcp_doc["fk_meta"] = dict(self.fk_meta)

        if self.eval:
            self.eval["report_text_live"] = self.report_text(title="=== CURRENT RUN (Live) ===")
            self.eval["report_text_disk"] = self.report_text(title="=== STORED EVAL (Disk) ===")

        pl_tcp_doc["eval"] = dict(self.eval)
        ex_tcp_doc["eval"] = dict(self.eval)

        self.planned_run["tcp"] = pl_tcp_doc
        self.executed_run["tcp"] = ex_tcp_doc

    # ------------------------------------------------------------
    # payloads (ProcessThread -> UI)
    # ------------------------------------------------------------

    def to_process_payload(self) -> Dict[str, Any]:
        return {
            "urdf_xml": self.urdf_xml,
            "srdf_xml": self.srdf_xml,
            "robot_description": {"urdf_xml": self.urdf_xml, "srdf_xml": self.srdf_xml},
            "planned_run": self.planned_run,
            "executed_run": self.executed_run,
            "fk_meta": self.fk_meta,
            "eval": self.eval,
            "valid": self.valid,
            "invalid_reason": self.invalid_reason,
        }

    @classmethod
    def from_process_payload(cls, payload: Dict[str, Any]) -> "RunResult":
        if not isinstance(payload, dict):
            raise ValueError("RunResult.from_process_payload: payload muss dict sein.")

        urdf_xml = str(payload.get("urdf_xml") or "")
        srdf_xml = str(payload.get("srdf_xml") or "")

        if not urdf_xml.strip() or not srdf_xml.strip():
            rd = payload.get("robot_description")
            rd = rd if isinstance(rd, dict) else {}
            urdf_xml = str(rd.get("urdf_xml") or urdf_xml or "")
            srdf_xml = str(rd.get("srdf_xml") or srdf_xml or "")

        return cls(
            urdf_xml=urdf_xml,
            srdf_xml=srdf_xml,
            planned_run=payload.get("planned_run"),
            executed_run=payload.get("executed_run"),
            fk_meta=payload.get("fk_meta"),
            eval=payload.get("eval"),
            valid=payload.get("valid", True),
            invalid_reason=payload.get("invalid_reason", ""),
        )

    # ------------------------------------------------------------
    # FK/TCP building (NO eval by default)
    # ------------------------------------------------------------

    def postprocess(
        self,
        *,
        recipe: Any,
        segment_order: Tuple[str, ...] | Sequence[str],
        ee_link: str = "tcp",
        step_mm: float = 1.0,
        max_points: int = 0,
        require_tcp: bool = True,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        scene_yaml_path: str,
        robot_yaml_path: str,
        mounts_yaml_path: str,
        evaluate: bool = False,
        gate_valid_on_eval: bool = False,
    ) -> None:
        if not self.urdf_xml.strip() or not self.srdf_xml.strip():
            raise ValueError("RunResult.postprocess: urdf_xml/srdf_xml fehlt (leer).")

        self.postprocess_from_urdf_srdf(
            urdf_xml=self.urdf_xml,
            srdf_xml=self.srdf_xml,
            recipe=recipe,
            segment_order=segment_order,
            ee_link=ee_link,
            step_mm=step_mm,
            max_points=max_points,
            require_tcp=require_tcp,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            scene_yaml_path=str(scene_yaml_path),
            robot_yaml_path=str(robot_yaml_path),
            mounts_yaml_path=str(mounts_yaml_path),
            evaluate=evaluate,
            gate_valid_on_eval=gate_valid_on_eval,
        )

    def postprocess_from_urdf_srdf(
        self,
        *,
        urdf_xml: str,
        srdf_xml: str,
        recipe: Any,
        segment_order: Tuple[str, ...] | Sequence[str],
        ee_link: str = "tcp",
        step_mm: float = 1.0,
        max_points: int = 0,
        require_tcp: bool = True,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        scene_yaml_path: str,
        robot_yaml_path: str,
        mounts_yaml_path: str,
        evaluate: bool = False,
        gate_valid_on_eval: bool = False,
    ) -> None:
        from model.spray_paths.traj_fk_builder import TrajFkBuilder, TrajFkConfig

        urdf_xml = str(urdf_xml or "")
        srdf_xml = str(srdf_xml or "")
        if not urdf_xml.strip() or not srdf_xml.strip():
            raise ValueError("RunResult.postprocess_from_urdf_srdf: urdf_xml/srdf_xml ist leer.")

        self.urdf_xml = urdf_xml
        self.srdf_xml = srdf_xml

        robot_model = TrajFkBuilder.build_robot_model_from_urdf_srdf(urdf_xml=urdf_xml, srdf_xml=srdf_xml)

        planned_traj = _dict(self.planned_run.get("traj"))
        executed_traj = _dict(self.executed_run.get("traj"))
        if not planned_traj:
            raise ValueError("RunResult.postprocess_from_urdf_srdf: planned_run.traj ist leer.")
        if not executed_traj:
            raise ValueError("RunResult.postprocess_from_urdf_srdf: executed_run.traj ist leer.")

        cfg = TrajFkConfig(ee_link=str(ee_link or "tcp"), step_mm=float(step_mm), max_points=int(max_points or 0))
        base_link = str(cfg.base_link)

        seg_ids = [str(s).strip() for s in list(segment_order) if str(s).strip()]
        if not seg_ids:
            raise ValueError("RunResult.postprocess_from_urdf_srdf: segment_order ist leer (seg_ids).")

        planned_tcp = TrajFkBuilder.build_tcp_draft_yaml(
            planned_traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            frame_id=base_link,
            include_segments=seg_ids,
            require_all_segments=True,
        )
        executed_tcp = TrajFkBuilder.build_tcp_draft_yaml(
            executed_traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            frame_id=base_link,
            include_segments=seg_ids,
            require_all_segments=True,
        )

        if base_link != "robot_mount":
            raise ValueError(
                f"postprocess_from_urdf_srdf: expected cfg.base_link='robot_mount' for offline substrate transform, got {base_link!r}"
            )
        if not scene_yaml_path or not robot_yaml_path or not mounts_yaml_path:
            raise ValueError("postprocess_from_urdf_srdf: substrate transform requires scene/robot/mounts yaml paths.")

        T_substrate_robot_mount = _compute_T_substrate_robot_mount_mm(
            scene_yaml_path=str(scene_yaml_path),
            robot_yaml_path=str(robot_yaml_path),
            mounts_yaml_path=str(mounts_yaml_path),
        )
        planned_tcp = TrajFkBuilder.transform_draft_yaml(planned_tcp, T_to_from_mm=T_substrate_robot_mount, out_frame="substrate")
        executed_tcp = TrajFkBuilder.transform_draft_yaml(executed_tcp, T_to_from_mm=T_substrate_robot_mount, out_frame="substrate")

        self.planned_run["tcp"] = _dict(planned_tcp)
        self.executed_run["tcp"] = _dict(executed_tcp)

        self.fk_meta = {
            "ee_link": str(cfg.ee_link),
            "base_link": str(cfg.base_link),
            "step_mm": float(cfg.step_mm),
            "max_points": int(cfg.max_points),
            "segment_order": list(seg_ids),
            "backend": "kdl",
            "tcp_frame": "substrate",
            "segments_included": list(seg_ids),
        }

        if require_tcp:
            if not _dict(self.planned_run.get("tcp")) or not _dict(self.executed_run.get("tcp")):
                raise ValueError("RunResult.postprocess_from_urdf_srdf: TCP Draft leer (require_tcp=true).")

        if evaluate:
            self.evaluate_tcp_against_draft(
                recipe=recipe,
                segment_order=list(seg_ids),
                domain="tcp",
                gate_valid_on_eval=gate_valid_on_eval,
            )

    # ------------------------------------------------------------
    # Explicit evaluation (call from RecipePanel)
    # ------------------------------------------------------------

    def evaluate_tcp_against_draft(
        self,
        *,
        recipe: Any | None = None,
        segment_order: Optional[Sequence[str]] = None,
        domain: str = "tcp",
        gate_valid_on_eval: bool = False,
    ) -> Dict[str, Any]:
        if recipe is None:
            recipe = self._require_attached_recipe()

        planned_tcp = _dict(self.planned_run.get("tcp"))
        executed_tcp = _dict(self.executed_run.get("tcp"))

        from .recipe_eval import RecipeEvaluator  # strict (no fallback module)

        evaluator = RecipeEvaluator()
        eval_dict = evaluator.evaluate_runs(
            recipe=recipe,
            planned_tcp=planned_tcp,
            executed_tcp=executed_tcp,
            segment_order=list(segment_order) if segment_order else None,
            domain=str(domain or "tcp"),
            traj_points_planned=self._count_traj_points(_dict(self.planned_run.get("traj"))),
            traj_points_executed=self._count_traj_points(_dict(self.executed_run.get("traj"))),
            tcp_poses_planned=self._count_tcp_poses(planned_tcp),
            tcp_poses_executed=self._count_tcp_poses(executed_tcp),
        )

        # SPEED (display-only; include reason if not computable)
        try:
            seg_recipe = self.DEFAULT_RECIPE_SEGMENT
            if segment_order and self.DEFAULT_RECIPE_SEGMENT in [str(s).strip() for s in segment_order]:
                seg_recipe = self.DEFAULT_RECIPE_SEGMENT

            pl_speed = self._compute_avg_speed_recipe_segment(recipe=recipe, which="planned", seg_name=seg_recipe)
            ex_speed = self._compute_avg_speed_recipe_segment(recipe=recipe, which="executed", seg_name=seg_recipe)

            eval_dict = _dict(eval_dict)
            eval_dict.setdefault("planned", {})
            eval_dict.setdefault("executed", {})
            eval_dict["planned"] = _dict(eval_dict.get("planned"))
            eval_dict["executed"] = _dict(eval_dict.get("executed"))
            eval_dict["planned"].setdefault("summary", {})
            eval_dict["executed"].setdefault("summary", {})
            ps = _dict(eval_dict["planned"].get("summary"))
            es = _dict(eval_dict["executed"].get("summary"))

            if pl_speed.get("speed_set_mm_s") is not None:
                ps["speed_set_mm_s"] = float(pl_speed["speed_set_mm_s"])
            if ex_speed.get("speed_set_mm_s") is not None:
                es["speed_set_mm_s"] = float(ex_speed["speed_set_mm_s"])

            if bool(pl_speed.get("ok")) and bool(ex_speed.get("ok")):
                for k in ("avg_speed_mm_s", "speed_percent"):
                    ps[k] = float(pl_speed[k])
                    es[k] = float(ex_speed[k])
                ps["speed_dbg"] = {"seg": pl_speed.get("seg_name"), "dt_s": pl_speed.get("duration_s"), "L_mm": pl_speed.get("path_len_mm")}
                es["speed_dbg"] = {"seg": ex_speed.get("seg_name"), "dt_s": ex_speed.get("duration_s"), "L_mm": ex_speed.get("path_len_mm")}
            else:
                ps["speed_reason"] = str(pl_speed.get("reason") or "unknown")
                es["speed_reason"] = str(ex_speed.get("reason") or "unknown")

            eval_dict["planned"]["summary"] = ps
            eval_dict["executed"]["summary"] = es
        except Exception as e:
            eval_dict = _dict(eval_dict)
            eval_dict.setdefault("planned", {})
            eval_dict.setdefault("executed", {})
            eval_dict["planned"] = _dict(eval_dict.get("planned"))
            eval_dict["executed"] = _dict(eval_dict.get("executed"))
            eval_dict["planned"].setdefault("summary", {})
            eval_dict["executed"].setdefault("summary", {})
            ps = _dict(eval_dict["planned"].get("summary"))
            es = _dict(eval_dict["executed"].get("summary"))
            msg = f"speed_exception:{type(e).__name__}:{e}"
            ps["speed_reason"] = msg
            es["speed_reason"] = msg
            eval_dict["planned"]["summary"] = ps
            eval_dict["executed"]["summary"] = es

        self.set_eval(_dict(eval_dict))

        if gate_valid_on_eval and self.eval and self.eval.get("valid") is False:
            self.invalidate(f"eval_invalid: {self.eval.get('invalid_reason') or 'invalid'}")

        return _dict(eval_dict)
