# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import os
import yaml
import functools


def _dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _get_nested(d: Dict[str, Any], path: Sequence[str], default: Any = None) -> Any:
    cur: Any = d
    for k in path:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
    return cur if cur is not None else default


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
    """
    Load YAML with an mtime-aware LRU cache.
    This avoids re-reading SSoT YAMLs on every preview/eval cycle.
    """
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
        return _dict(yaml.safe_load(f) or {})


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
    """
    Compute substrate_mount -> substrate from substrate_mounts.yaml SSoT.

    Expected schema:
      version: 1
      active_mount: <name>
      mounts:
        <name>:
          scene_offset:
            xyz: [x,y,z]   # mm
            rpy_deg: [r,p,y]
    """
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
    """
    Compute rigid transform: p_substrate = T_substrate_robot_mount * p_robot_mount

    Uses:
      scene.yaml:
        world -> substrate_mount (id='substrate_mount', frame='world')
      substrate_mounts.yaml:
        substrate_mount -> substrate (active_mount.scene_offset)  [SSoT]
      robot.yaml:
        world -> robot_mount
    """
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
# RunResult
# ============================================================


@dataclass
class RunResult:
    """
    STRICT run result container (SSoT).

    IMPORTANT:
      - Process collects ONLY raw artifacts (traj + robot_description).
      - FK/TCP building is a deterministic post-step.
      - Evaluation is explicit (call evaluate_tcp_against_draft()).

    planned_run  = {"traj": <JTBySegment YAML dict>, "tcp": <TCP YAML dict>}
    executed_run = {"traj": <JTBySegment YAML dict>, "tcp": <TCP YAML dict>}
    """

    urdf_xml: str = ""
    srdf_xml: str = ""

    planned_run: Dict[str, Any] = field(default_factory=dict)
    executed_run: Dict[str, Any] = field(default_factory=dict)

    fk_meta: Dict[str, Any] = field(default_factory=dict)
    eval: Dict[str, Any] = field(default_factory=dict)

    valid: bool = True
    invalid_reason: str = ""

    # STRICT invariant: TCP target frame is ALWAYS substrate.
    TCP_TARGET_FRAME: str = "substrate"

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
    # formatting helpers (STRICT: do not collapse False/0 to empty)
    # ------------------------------------------------------------

    @staticmethod
    def _fmt_bool(v: Any) -> str:
        if v is True:
            return "true"
        if v is False:
            return "false"
        return "–"

    @staticmethod
    def _fmt_num3(v: Any) -> str:
        if v is None:
            return "–"
        if isinstance(v, bool):
            return "true" if v else "false"
        if isinstance(v, (int, float)):
            return f"{float(v):.3f}"
        return str(v)

    @staticmethod
    def _fmt_text(v: Any) -> str:
        if v is None:
            return "–"
        if isinstance(v, bool):
            return "true" if v else "false"
        s = str(v).strip()
        return s if s else "–"

    @staticmethod
    def _pad_table(rows: list[tuple[str, str, str]]) -> str:
        if not rows:
            return ""
        c0 = max(len(r[0]) for r in rows)
        c1 = max(len(r[1]) for r in rows)
        c2 = max(len(r[2]) for r in rows)

        def line(a: str, b: str, c: str) -> str:
            return f"{a.ljust(c0)} | {b.ljust(c1)} | {c.ljust(c2)}"

        out: list[str] = []
        out.append(line("topic", "planned", "executed"))
        out.append(line("-" * 5, "-" * 6, "-" * 8))
        for a, b, c in rows:
            out.append(line(a, b, c))
        return "\n".join(out)

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

    def report_text(
        self,
        *,
        title: str = "=== CURRENT RUN (Live) ===",
        include_traj: bool = True,
        include_tcp: bool = True,
        include_reason: bool = True,
    ) -> str:
        """
        Builds the exact table string for the UI. Panel must only display this string.

        IMPORTANT:
          - Uses unified top-level eval fields if present (planned_score/executed_score, planned_valid/executed_valid).
        """
        pl_tcp = self.planned_run.get("tcp") if isinstance(self.planned_run, dict) else {}
        ex_tcp = self.executed_run.get("tcp") if isinstance(self.executed_run, dict) else {}
        pl_traj = self.planned_run.get("traj") if isinstance(self.planned_run, dict) else {}
        ex_traj = self.executed_run.get("traj") if isinstance(self.executed_run, dict) else {}

        ev = self.eval if isinstance(self.eval, dict) else {}

        thr = ev.get("threshold", None)
        overall_valid = ev.get("valid", None)

        total_score = ev.get("score")
        if total_score is None and isinstance(ev.get("total"), dict):
            total_score = ev["total"].get("score")

        # prefer explicit top-level per-mode fields
        p_score = ev.get("planned_score", None)
        e_score = ev.get("executed_score", None)
        p_thr_ok = ev.get("planned_valid", None)
        e_thr_ok = ev.get("executed_valid", None)

        # fallback to legacy nested modes
        p_mode = ev.get("planned") if isinstance(ev.get("planned"), dict) else {}
        e_mode = ev.get("executed") if isinstance(ev.get("executed"), dict) else {}
        if p_score is None:
            p_score = p_mode.get("score")
        if e_score is None:
            e_score = e_mode.get("score")

        rows: list[tuple[str, str, str]] = []

        if include_traj:
            rows.append(("traj_points", str(self._count_traj_points(pl_traj)), str(self._count_traj_points(ex_traj))))

        if include_tcp:
            rows.append(("tcp_poses", str(self._count_tcp_poses(pl_tcp)), str(self._count_tcp_poses(ex_tcp))))
            rows.append(("tcp_frame", self._fmt_text(pl_tcp.get("frame")), self._fmt_text(ex_tcp.get("frame"))))

        rows.append(("eval_valid", self._fmt_bool(overall_valid), self._fmt_bool(overall_valid)))
        rows.append(("eval_thr", self._fmt_num3(thr), self._fmt_num3(thr)))
        rows.append(("eval_score", self._fmt_num3(total_score), self._fmt_num3(total_score)))

        # threshold-aware per-mode flags (if present)
        if p_thr_ok is not None or e_thr_ok is not None:
            rows.append(("mode_ok", self._fmt_bool(p_thr_ok), self._fmt_bool(e_thr_ok)))

        # per-mode scores
        if p_score is not None or e_score is not None:
            rows.append(("mode_score", self._fmt_num3(p_score), self._fmt_num3(e_score)))

        comp = ev.get("comparison")
        if isinstance(comp, list):
            for r in comp:
                if not isinstance(r, dict):
                    continue
                metric = self._fmt_text(r.get("metric"))
                rows.append((f"metric:{metric}", self._fmt_text(r.get("planned")), self._fmt_text(r.get("executed"))))

        out: list[str] = []
        out.append(str(title or "").strip())
        out.append(self._pad_table(rows))

        if include_reason:
            inv = self._fmt_text(ev.get("invalid_reason"))
            if inv != "–":
                out.append("")
                out.append(f"reason: {inv}")

        return "\n".join(out).strip()

    # ------------------------------------------------------------
    # eval persistence helpers (split planned/executed)
    # ------------------------------------------------------------

    @staticmethod
    def _split_eval_for_tcp_docs(full_eval: Dict[str, Any]) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        Convert a unified evaluator output into two per-TCP eval dicts:

          - planned_tcp.eval   -> planned score/valid/invalid_reason
          - executed_tcp.eval  -> executed score/valid/invalid_reason

        STRICT:
          - Prefer evaluator-provided fields:
              planned_valid / executed_valid  (threshold-aware)
              planned_score / executed_score
          - Fallback to legacy nested dicts if those are absent.
        """
        ev = _dict(full_eval)

        # threshold (fallback only)
        thr = ev.get("threshold", None)
        try:
            thr_f = float(thr) if thr is not None else 90.0
        except Exception:
            thr_f = 90.0

        p_mode = _dict(ev.get("planned"))
        e_mode = _dict(ev.get("executed"))

        def _float_or0(v: Any) -> float:
            try:
                return float(v) if v is not None else 0.0
            except Exception:
                return 0.0

        # scores: prefer top-level
        p_score = ev.get("planned_score", None)
        e_score = ev.get("executed_score", None)
        if p_score is None:
            p_score = p_mode.get("score")
            if p_score is None and isinstance(p_mode.get("total"), dict):
                p_score = p_mode["total"].get("score")
        if e_score is None:
            e_score = e_mode.get("score")
            if e_score is None and isinstance(e_mode.get("total"), dict):
                e_score = e_mode["total"].get("score")

        p_score_f = _float_or0(p_score)
        e_score_f = _float_or0(e_score)

        # validity: prefer threshold-aware booleans if evaluator provides them
        if "planned_valid" in ev:
            p_valid = bool(ev.get("planned_valid"))
        else:
            # legacy: mode.valid meant computed_ok; apply threshold here as fallback
            p_valid = bool(p_mode.get("valid", False)) and (p_score_f >= thr_f)

        if "executed_valid" in ev:
            e_valid = bool(ev.get("executed_valid"))
        else:
            e_valid = bool(e_mode.get("valid", False)) and (e_score_f >= thr_f)

        base: Dict[str, Any] = dict(ev)
        base.pop("fk_meta", None)

        # mode-specific invalid reasons:
        # - if computed failed: keep mode invalid_reason
        # - else if threshold failed: score_below_threshold
        def _mode_reason(mode: Dict[str, Any], ok: bool, *, score: float) -> str:
            if ok:
                return ""
            mr = mode.get("invalid_reason")
            if isinstance(mr, str) and mr.strip():
                return mr.strip()
            if score < thr_f:
                return "score_below_threshold"
            # last resort: propagate global invalid_reason
            ir = ev.get("invalid_reason")
            return str(ir or "invalid")

        planned_ev = dict(base)
        planned_ev["mode"] = "planned"
        planned_ev["valid"] = bool(p_valid)
        planned_ev["score"] = float(p_score_f)
        planned_ev["total"] = {"score": float(p_score_f)}
        planned_ev["invalid_reason"] = _mode_reason(p_mode, p_valid, score=p_score_f)

        executed_ev = dict(base)
        executed_ev["mode"] = "executed"
        executed_ev["valid"] = bool(e_valid)
        executed_ev["score"] = float(e_score_f)
        executed_ev["total"] = {"score": float(e_score_f)}
        executed_ev["invalid_reason"] = _mode_reason(e_mode, e_valid, score=e_score_f)

        return planned_ev, executed_ev

    def set_eval(self, eval_dict: Dict[str, Any] | None) -> None:
        """
        Stores eval dict on RunResult AND embeds per-mode eval into planned/executed TCP YAML docs.

        Additionally:
          - Writes prebuilt report strings so UI panels only display them.
            * self.eval["report_text_live"]
            * self.eval["report_text_disk"]
            * planned_tcp.eval["report_text"] / executed_tcp.eval["report_text"]  (disk view)
        """
        self.eval = _dict(eval_dict)

        pl_tcp_doc = _dict(self.planned_run.get("tcp"))
        ex_tcp_doc = _dict(self.executed_run.get("tcp"))

        if self.fk_meta:
            pl_tcp_doc["fk_meta"] = dict(self.fk_meta)
            ex_tcp_doc["fk_meta"] = dict(self.fk_meta)

        # Precompute report strings (do this BEFORE splitting, uses self.eval)
        if self.eval:
            try:
                self.eval["report_text_live"] = self.report_text(
                    title="=== CURRENT RUN (Live) ===", include_traj=True, include_tcp=True, include_reason=True
                )
            except Exception:
                pass
            try:
                self.eval["report_text_disk"] = self.report_text(
                    title="=== STORED EVAL (Disk) ===", include_traj=False, include_tcp=True, include_reason=True
                )
            except Exception:
                pass

        if self.eval:
            pl_eval, ex_eval = self._split_eval_for_tcp_docs(self.eval)

            rep_disk = self.eval.get("report_text_disk")
            if isinstance(rep_disk, str) and rep_disk.strip():
                pl_eval["report_text"] = rep_disk
                ex_eval["report_text"] = rep_disk

            pl_tcp_doc["eval"] = pl_eval
            ex_tcp_doc["eval"] = ex_eval

        self.planned_run["tcp"] = pl_tcp_doc
        self.executed_run["tcp"] = ex_tcp_doc

    # ------------------------------------------------------------
    # derived helpers (UI / persist)
    # ------------------------------------------------------------

    def eval_invalid_reason(self) -> str:
        if not self.eval:
            return "unevaluated"
        v = self.eval.get("valid")
        if v is True:
            return ""
        return str(self.eval.get("invalid_reason") or "eval_invalid")

    def eval_summary_line(self) -> str:
        if not self.eval:
            return "unevaluated"

        valid = self.eval.get("valid")
        dom = self.eval.get("domain") or ""
        total = _get_nested(self.eval, ["total", "score"], None)
        thr = self.eval.get("threshold", None)

        parts = []
        if dom:
            parts.append(str(dom))
        if total is not None:
            try:
                parts.append(f"score={float(total):.3f}")
            except Exception:
                parts.append(f"score={total}")
        if thr is not None:
            try:
                parts.append(f"thr={float(thr):.3f}")
            except Exception:
                parts.append(f"thr={thr}")
        if valid is True:
            parts.append("VALID")
        elif valid is False:
            parts.append("INVALID")
        else:
            parts.append("unknown")

        return " | ".join(parts) if parts else "–"

    def format_eval_text(self, *, include_tcp: bool = True) -> str:
        """
        UI text:
          - overall eval line
          - per-segment summary if present
          - tcp pose counts (global sides)
          - fk_meta essentials
        """
        lines: list[str] = []
        lines.append(self.eval_summary_line())

        if self.eval and isinstance(self.eval, dict):
            inv = self.eval.get("invalid_reason") or ""
            if inv:
                lines.append(f"reason: {inv}")

            segs = self.eval.get("segments") or self.eval.get("by_segment")
            if isinstance(segs, dict) and segs:
                lines.append("")
                lines.append("segments:")
                for seg_id, seg_v in segs.items():
                    if not isinstance(seg_v, dict):
                        continue
                    score = _get_nested(seg_v, ["score"], None)
                    if score is None:
                        score = _get_nested(seg_v, ["total", "score"], None)
                    v = seg_v.get("valid", None)
                    ok = "OK" if v is True else ("FAIL" if v is False else "UNK")
                    if score is not None:
                        try:
                            s_txt = f"{float(score):.3f}"
                        except Exception:
                            s_txt = str(score)
                        lines.append(f"  - {seg_id}: {ok} | score={s_txt}")
                    else:
                        lines.append(f"  - {seg_id}: {ok}")

        if include_tcp:
            pl_tcp = self.planned_run.get("tcp") if isinstance(self.planned_run, dict) else {}
            ex_tcp = self.executed_run.get("tcp") if isinstance(self.executed_run, dict) else {}

            f_pl = (pl_tcp.get("frame") if isinstance(pl_tcp, dict) else "") or ""
            f_ex = (ex_tcp.get("frame") if isinstance(ex_tcp, dict) else "") or ""
            frame_txt = f" frame(planned={f_pl or 'n/a'}, executed={f_ex or 'n/a'})"
            lines.append("")
            lines.append(
                f"tcp: planned={self._count_tcp_poses(pl_tcp)} poses | executed={self._count_tcp_poses(ex_tcp)} poses |{frame_txt}"
            )

        if self.fk_meta:
            ee = self.fk_meta.get("ee_link") or ""
            step = self.fk_meta.get("step_mm")
            base = self.fk_meta.get("base_link") or ""
            segs = self.fk_meta.get("segments_included") or self.fk_meta.get("segment_order") or []
            lines.append("")
            if base:
                lines.append(f"fk: base_link={base}")
            if ee:
                lines.append(f"fk: ee_link={ee}")
            if step is not None:
                try:
                    lines.append(f"fk: step_mm={float(step):.3f}")
                except Exception:
                    lines.append(f"fk: step_mm={step}")
            if isinstance(segs, list) and segs:
                lines.append(f"fk: segments={len(segs)}")

        return "\n".join(lines).strip() if lines else "–"

    # ------------------------------------------------------------
    # payloads (ProcessThread -> UI)
    # ------------------------------------------------------------

    def to_process_payload(self) -> Dict[str, Any]:
        return {
            "robot_description": {
                "urdf_xml": self.urdf_xml,
                "srdf_xml": self.srdf_xml,
            },
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

        rd = payload.get("robot_description")
        rd = rd if isinstance(rd, dict) else {}

        return cls(
            urdf_xml=str(rd.get("urdf_xml") or ""),
            srdf_xml=str(rd.get("srdf_xml") or ""),
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

        if self.TCP_TARGET_FRAME != "substrate":
            raise ValueError(f"RunResult TCP_TARGET_FRAME must be 'substrate', got {self.TCP_TARGET_FRAME!r}")

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

        # STRICT: always transform into substrate.
        if base_link != "robot_mount":
            raise ValueError(
                f"postprocess_from_urdf_srdf: expected cfg.base_link='robot_mount' for offline substrate transform, got {base_link!r}"
            )

        if not scene_yaml_path or not robot_yaml_path or not mounts_yaml_path:
            raise ValueError(
                "postprocess_from_urdf_srdf: substrate transform requires scene_yaml_path, robot_yaml_path, mounts_yaml_path."
            )

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
            "tcp_frame": str((_dict(self.planned_run.get("tcp"))).get("frame") or base_link),
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
        recipe: Any,
        segment_order: Optional[Sequence[str]] = None,
        domain: str = "tcp",
        gate_valid_on_eval: bool = False,
    ) -> Dict[str, Any]:
        """
        Explicit evaluation step. Intended to be called in RecipePanel (NOT in process thread).
        Stores result via set_eval() and embeds per-mode eval into TCP docs.
        """
        planned_tcp = _dict(self.planned_run.get("tcp"))
        executed_tcp = _dict(self.executed_run.get("tcp"))

        try:
            from .recipe_eval import RecipeEvaluator  # local import to avoid circulars

            evaluator = RecipeEvaluator()
            eval_dict = evaluator.evaluate_runs(
                recipe=recipe,
                planned_tcp=planned_tcp,
                executed_tcp=executed_tcp,
                segment_order=list(segment_order) if segment_order else None,
                domain=str(domain or "tcp"),
            )
        except Exception as e:
            eval_dict = {"valid": False, "invalid_reason": f"eval_failed: {e}", "domain": str(domain or "tcp")}

        self.set_eval(_dict(eval_dict))

        if gate_valid_on_eval and self.eval and self.eval.get("valid") is False:
            self.invalidate(f"eval_invalid: {self.eval.get('invalid_reason') or 'invalid'}")

        return _dict(eval_dict)
