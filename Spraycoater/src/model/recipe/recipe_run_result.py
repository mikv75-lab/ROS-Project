# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple

import os
import yaml


def _dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _as_bool(v: Any, default: bool = False) -> bool:
    if v is None:
        return default
    return bool(v)


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


def _mat4_from_xyz_quat_mm(xyz_mm: Tuple[float, float, float], q: Tuple[float, float, float, float]) -> list[list[float]]:
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
    with open(p, "r", encoding="utf-8") as f:
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

    # world -> substrate_mount (from scene objects)
    o_mount = _scene_object_by_id(scene_doc, substrate_mount_id)
    if str(o_mount.get("frame", "world")) != "world":
        raise ValueError("scene.yaml: substrate_mount.frame must be 'world' for this helper")
    T_world_sub_mount = _tf_from_scene_obj_meters(o_mount)

    # substrate_mount -> substrate (SSoT: mounts yaml)
    T_sub_mount_substrate = _tf_substrate_mount_to_substrate_from_mounts_yaml(mounts_doc=mounts_doc)

    # world -> substrate
    T_world_substrate = _mat4_mul(T_world_sub_mount, T_sub_mount_substrate)

    # world -> robot_mount
    T_world_robot_mount = _tf_world_robot_mount_from_robot_yaml(robot_doc)

    # substrate <- robot_mount : inv(world->substrate) * (world->robot_mount)
    T_substrate_world = _mat4_inv_rigid(T_world_substrate)
    return _mat4_mul(T_substrate_world, T_world_robot_mount)


# ============================================================
# RunResult
# ============================================================

@dataclass
class RunResult:
    """
    STRICT run result container (SSoT).

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

    def set_eval(self, eval_dict: Dict[str, Any] | None) -> None:
        """
        Stores eval dict AND embeds it into planned/executed TCP YAML docs.
        IMPORTANT: This function must not recurse.
        """
        self.eval = _dict(eval_dict)

        # Persist eval into the TCP YAML docs (NOT into traj YAML).
        try:
            pl_tcp_doc = _dict(self.planned_run.get("tcp"))
            ex_tcp_doc = _dict(self.executed_run.get("tcp"))

            if self.fk_meta:
                pl_tcp_doc["fk_meta"] = dict(self.fk_meta)
                ex_tcp_doc["fk_meta"] = dict(self.fk_meta)

            if self.eval:
                # If caller provides v2 split, honor it; else store same dict
                pl_eval = _dict(self.eval.get("planned"))
                ex_eval = _dict(self.eval.get("executed"))

                if not pl_eval and self.eval.get("version") != 2:
                    pl_eval = dict(self.eval)
                if not ex_eval and self.eval.get("version") != 2:
                    ex_eval = dict(self.eval)

                pl_tcp_doc["eval"] = pl_eval
                ex_tcp_doc["eval"] = ex_eval

            self.planned_run["tcp"] = pl_tcp_doc
            self.executed_run["tcp"] = ex_tcp_doc
        except Exception:
            pass

    def invalidate(self, reason: str) -> None:
        self.valid = False
        self.invalid_reason = str(reason or "invalid")

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

            def _count_tcp_global(doc: Any) -> int:
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
                return n

            f_pl = (pl_tcp.get("frame") if isinstance(pl_tcp, dict) else "") or ""
            f_ex = (ex_tcp.get("frame") if isinstance(ex_tcp, dict) else "") or ""
            frame_txt = f" frame(planned={f_pl or 'n/a'}, executed={f_ex or 'n/a'})"
            lines.append("")
            lines.append(
                f"tcp: planned={_count_tcp_global(pl_tcp)} poses | executed={_count_tcp_global(ex_tcp)} poses |{frame_txt}"
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
    # FK/TCP + Eval
    # ------------------------------------------------------------

    def postprocess(
        self,
        *,
        recipe: Any,
        segment_order: Tuple[str, ...] | Sequence[str],
        ee_link: str = "tcp",
        step_mm: float = 1.0,
        max_points: int = 0,
        gate_valid_on_eval: bool = False,
        require_tcp: bool = True,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        tcp_target_frame: str = "substrate",
        scene_yaml_path: Optional[str] = None,
        robot_yaml_path: Optional[str] = None,
        mounts_yaml_path: Optional[str] = None,
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
            gate_valid_on_eval=gate_valid_on_eval,
            require_tcp=require_tcp,
            segment_to_side=segment_to_side,
            default_side=default_side,
            tcp_target_frame=tcp_target_frame,
            scene_yaml_path=scene_yaml_path,
            robot_yaml_path=robot_yaml_path,
            mounts_yaml_path=mounts_yaml_path,
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
        gate_valid_on_eval: bool = False,
        require_tcp: bool = True,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        tcp_target_frame: str = "substrate",
        scene_yaml_path: Optional[str] = None,
        robot_yaml_path: Optional[str] = None,
        mounts_yaml_path: Optional[str] = None,
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
        base_frame = str(cfg.base_link)

        seg_ids = [str(s).strip() for s in list(segment_order) if str(s).strip()]
        if not seg_ids:
            raise ValueError("RunResult.postprocess_from_urdf_srdf: segment_order ist leer (seg_ids).")

        # Build TCP YAML with per-segment isolation for ALL segments in segment_order.
        # NOTE: TrajFkBuilder is STRICT and does NOT support boundary de-duplication.
        planned_tcp = TrajFkBuilder.build_tcp_draft_yaml(
            planned_traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            frame_id=base_frame,
            include_segments=seg_ids,
            require_all_segments=True,
        )

        executed_tcp = TrajFkBuilder.build_tcp_draft_yaml(
            executed_traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            frame_id=base_frame,
            include_segments=seg_ids,
            require_all_segments=True,
        )

        # Optional: transform TCP into substrate (offline; includes mount scene_offset).
        target = str(tcp_target_frame or "").strip()
        if target and target != base_frame:
            if target != "substrate":
                raise ValueError(
                    f"postprocess_from_urdf_srdf: unsupported tcp_target_frame={target!r} "
                    f"(supported: {base_frame!r} or 'substrate')"
                )
            if base_frame != "robot_mount":
                raise ValueError(
                    f"postprocess_from_urdf_srdf: expected cfg.base_link='robot_mount' for offline transform, got {base_frame!r}"
                )
            if not scene_yaml_path or not robot_yaml_path:
                raise ValueError(
                    "postprocess_from_urdf_srdf: tcp_target_frame='substrate', "
                    "aber scene_yaml_path/robot_yaml_path fehlen."
                )
            if not mounts_yaml_path:
                raise ValueError(
                    "postprocess_from_urdf_srdf: tcp_target_frame='substrate', "
                    "aber mounts_yaml_path fehlt (substrate_mounts.yaml ist SSoT für mount scene_offset)."
                )

            T_substrate_robot_mount = _compute_T_substrate_robot_mount_mm(
                scene_yaml_path=str(scene_yaml_path),
                robot_yaml_path=str(robot_yaml_path),
                mounts_yaml_path=str(mounts_yaml_path),
            )
            planned_tcp = TrajFkBuilder.transform_draft_yaml(
                planned_tcp,
                T_to_from_mm=T_substrate_robot_mount,
                out_frame="substrate",
            )
            executed_tcp = TrajFkBuilder.transform_draft_yaml(
                executed_tcp,
                T_to_from_mm=T_substrate_robot_mount,
                out_frame="substrate",
            )

        self.planned_run["tcp"] = _dict(planned_tcp)
        self.executed_run["tcp"] = _dict(executed_tcp)

        self.fk_meta = {
            "ee_link": str(cfg.ee_link),
            "base_link": str(cfg.base_link),
            "step_mm": float(cfg.step_mm),
            "max_points": int(cfg.max_points),
            "segment_order": list(seg_ids),
            "backend": "kdl",
            "tcp_frame": str((_dict(self.planned_run.get("tcp"))).get("frame") or base_frame),
            "segments_included": list(seg_ids),
        }

        # Eval (strict): evaluator isolates MOVE_RECIPE from tcp.yaml.segments or meta.segment_slices.
        eval_dict: Dict[str, Any] = {}
        try:
            from .recipe_eval import RecipeEvaluator  # type: ignore

            evaluator = RecipeEvaluator()
            fn = getattr(evaluator, "evaluate_runs", None)
            if callable(fn):
                eval_dict = fn(
                    recipe=recipe,
                    planned_tcp=planned_tcp,
                    executed_tcp=executed_tcp,
                    segment_order=list(seg_ids),
                    domain="tcp",
                )
            else:
                fn2 = getattr(evaluator, "evaluate", None)
                if callable(fn2):
                    eval_dict = fn2(recipe=recipe, planned=planned_tcp, executed=executed_tcp)
                else:
                    raise RuntimeError("RecipeEvaluator: keine evaluate_runs()/evaluate() API gefunden.")
        except Exception as e:
            eval_dict = {"valid": False, "invalid_reason": f"eval_failed: {e}"}

        self.set_eval(_dict(eval_dict))

        if require_tcp:
            if not _dict(self.planned_run.get("tcp")) or not _dict(self.executed_run.get("tcp")):
                raise ValueError("RunResult.postprocess_from_urdf_srdf: TCP Draft leer (require_tcp=true).")

        if gate_valid_on_eval:
            if self.eval and self.eval.get("valid") is False:
                self.invalidate(f"eval_invalid: {self.eval.get('invalid_reason') or 'invalid'}")
