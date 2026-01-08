# -*- coding: utf-8 -*-
# File: src/model/recipe/traj_fk_builder.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple

import math

from .recipe import Draft, PoseQuat, PathSide, JTBySegment, JTSegment


def _as_dict(x: Any, *, name: str) -> Dict[str, Any]:
    if not isinstance(x, dict):
        raise ValueError(f"{name} muss dict sein, ist {type(x).__name__}")
    return x


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _vec_norm3(x: float, y: float, z: float) -> float:
    return math.sqrt(x * x + y * y + z * z)


def _quat_norm(q: Tuple[float, float, float, float]) -> float:
    return math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])


def _quat_normalize(q: Tuple[float, float, float, float]) -> Tuple[float, float, float, float]:
    n = _quat_norm(q)
    if n <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (q[0] / n, q[1] / n, q[2] / n, q[3] / n)


def _quat_dot(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]


def _quat_slerp(qa: Tuple[float, float, float, float], qb: Tuple[float, float, float, float], t: float) -> Tuple[float, float, float, float]:
    t = _clamp(float(t), 0.0, 1.0)
    qa = _quat_normalize(qa)
    qb = _quat_normalize(qb)

    dot = _quat_dot(qa, qb)

    if dot < 0.0:
        qb = (-qb[0], -qb[1], -qb[2], -qb[3])
        dot = -dot

    if dot > 0.9995:
        out = (
            qa[0] + t * (qb[0] - qa[0]),
            qa[1] + t * (qb[1] - qa[1]),
            qa[2] + t * (qb[2] - qa[2]),
            qa[3] + t * (qb[3] - qa[3]),
        )
        return _quat_normalize(out)

    theta_0 = math.acos(_clamp(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    if sin_theta_0 <= 1e-12:
        return qa

    theta = theta_0 * t
    sin_theta = math.sin(theta)

    s0 = math.cos(theta) - dot * (sin_theta / sin_theta_0)
    s1 = sin_theta / sin_theta_0

    out = (
        s0 * qa[0] + s1 * qb[0],
        s0 * qa[1] + s1 * qb[1],
        s0 * qa[2] + s1 * qb[2],
        s0 * qa[3] + s1 * qb[3],
    )
    return _quat_normalize(out)


def _safe_vec3(v: Any) -> Optional[Tuple[float, float, float]]:
    if v is None:
        return None
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:
        pass
    try:
        return (float(v.x), float(v.y), float(v.z))
    except Exception:
        pass
    try:
        return (float(v.x()), float(v.y()), float(v.z()))
    except Exception:
        return None


def _safe_quat_xyzw(q: Any) -> Optional[Tuple[float, float, float, float]]:
    if q is None:
        return None
    try:
        return (float(q.x), float(q.y), float(q.z), float(q.w))
    except Exception:
        pass
    try:
        return (float(q.x()), float(q.y()), float(q.z()), float(q.w()))
    except Exception:
        return None


def _tf_to_posequat_mm(tf: Any, *, meters_to_mm: float = 1000.0) -> Optional[PoseQuat]:
    if tf is None:
        return None

    t = None
    q = None

    for getter in ("translation", "get_translation"):
        try:
            t = getattr(tf, getter)
            break
        except Exception:
            continue
    if callable(t):
        try:
            t = t()
        except Exception:
            t = None

    for getter in ("rotation", "get_rotation"):
        try:
            q = getattr(tf, getter)
            break
        except Exception:
            continue
    if callable(q):
        try:
            q = q()
        except Exception:
            q = None

    xyz = _safe_vec3(t)
    xyzw = _safe_quat_xyzw(q)

    if xyz is None:
        try:
            m = tf.matrix()  # type: ignore[attr-defined]
            xyz = (float(m[0][3]), float(m[1][3]), float(m[2][3]))
        except Exception:
            return None

    if xyzw is None:
        xyzw = (0.0, 0.0, 0.0, 1.0)

    x_m, y_m, z_m = xyz
    qx, qy, qz, qw = _quat_normalize(xyzw)

    return PoseQuat(
        x=float(x_m) * float(meters_to_mm),
        y=float(y_m) * float(meters_to_mm),
        z=float(z_m) * float(meters_to_mm),
        qx=float(qx),
        qy=float(qy),
        qz=float(qz),
        qw=float(qw),
    )


@dataclass(frozen=True)
class TrajFkConfig:
    """
    Draft-identisches Sampling (mm, NICHT time):

    - step_mm: resample in TCP-space by arclength (mm)
    - max_points: 0 = unlimited, sonst cap
    """
    ee_link: str = "tcp"
    meters_to_mm: float = 1000.0
    step_mm: float = 1.0
    max_points: int = 0  # 0 = unlimited


class TrajFkBuilder:
    """
    JTBySegment -> TCP-Pfad (Draft-Schema) via FK.

    Output:
      - Draft(version=1, sides={...poses_quat...})
      - oder als YAML dict via build_tcp_draft_yaml()
    """

    @staticmethod
    def build_tcp_draft(
        traj: JTBySegment | Mapping[str, Any],
        *,
        robot_model: Any,
        cfg: TrajFkConfig,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        drop_duplicate_boundary: bool = True,
    ) -> Draft:
        jt = TrajFkBuilder._coerce_jt_by_segment(traj)

        if not (isinstance(cfg.step_mm, (int, float)) and float(cfg.step_mm) > 0.0):
            raise ValueError("TrajFkBuilder: cfg.step_mm muss > 0 sein.")
        if cfg.max_points and int(cfg.max_points) < 2:
            raise ValueError("TrajFkBuilder: cfg.max_points muss 0 oder >= 2 sein.")

        by_side: Dict[str, List[PoseQuat]] = {}

        for seg_id, seg in jt.segments.items():
            side = default_side
            if segment_to_side and seg_id in segment_to_side:
                side = str(segment_to_side[seg_id] or default_side)
            side = side.strip() or default_side

            raw_poses = TrajFkBuilder._fk_segment_raw(seg, robot_model=robot_model, cfg=cfg)
            if len(raw_poses) < 2:
                continue

            sampled = TrajFkBuilder._resample_posequats_by_step_mm(
                raw_poses,
                step_mm=float(cfg.step_mm),
                max_points=int(cfg.max_points or 0),
            )
            if not sampled:
                continue

            if drop_duplicate_boundary and side in by_side and by_side[side]:
                a = by_side[side][-1]
                b = sampled[0]
                if _vec_norm3(b.x - a.x, b.y - a.y, b.z - a.z) <= 1e-9:
                    sampled = sampled[1:]
                    if not sampled:
                        continue

            by_side.setdefault(side, []).extend(sampled)

        if not by_side:
            raise ValueError("TrajFkBuilder: keine TCP Posen erzeugt (leere Traj oder FK fehlgeschlagen).")

        sides: Dict[str, PathSide] = {s: PathSide(poses_quat=list(poses)) for s, poses in by_side.items()}
        return Draft(version=1, sides=sides)

    @staticmethod
    def build_tcp_draft_yaml(
        traj: JTBySegment | Mapping[str, Any],
        *,
        robot_model: Any,
        cfg: TrajFkConfig,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        drop_duplicate_boundary: bool = True,
    ) -> Dict[str, Any]:
        """
        Convenience: gibt Draft als YAML-dict zurück (für Persistenz/Qt payload).
        """
        d = TrajFkBuilder.build_tcp_draft(
            traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=default_side,
            drop_duplicate_boundary=drop_duplicate_boundary,
        )
        return TrajFkBuilder._draft_to_yaml_dict(d)

    # ------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------

    @staticmethod
    def _draft_to_yaml_dict(d: Draft) -> Dict[str, Any]:
        # bevorzugt: echtes API, falls vorhanden
        for fn_name in ("to_yaml_dict", "to_dict", "as_dict"):
            fn = getattr(d, fn_name, None)
            if callable(fn):
                out = fn()
                return out if isinstance(out, dict) else {}
        # fallback: best-effort
        sides_out: Dict[str, Any] = {}
        try:
            for side, side_obj in (getattr(d, "sides", {}) or {}).items():
                poses = []
                for p in (getattr(side_obj, "poses_quat", []) or []):
                    poses.append(
                        {
                            "x": float(getattr(p, "x", 0.0)),
                            "y": float(getattr(p, "y", 0.0)),
                            "z": float(getattr(p, "z", 0.0)),
                            "qx": float(getattr(p, "qx", 0.0)),
                            "qy": float(getattr(p, "qy", 0.0)),
                            "qz": float(getattr(p, "qz", 0.0)),
                            "qw": float(getattr(p, "qw", 1.0)),
                        }
                    )
                sides_out[str(side)] = {"poses_quat": poses}
        except Exception:
            sides_out = {}
        return {"version": int(getattr(d, "version", 1) or 1), "sides": sides_out}

    @staticmethod
    def _coerce_jt_by_segment(traj: JTBySegment | Mapping[str, Any]) -> JTBySegment:
        if isinstance(traj, JTBySegment):
            return traj
        dd = _as_dict(dict(traj), name="traj")
        return JTBySegment.from_yaml_dict(dd)

    @staticmethod
    def _fk_segment_raw(seg: JTSegment, *, robot_model: Any, cfg: TrajFkConfig) -> List[PoseQuat]:
        if robot_model is None:
            raise ValueError("TrajFkBuilder: robot_model ist None (MoveIt RobotModel benötigt).")

        try:
            from moveit.core.robot_state import RobotState  # type: ignore
        except Exception as e:
            raise RuntimeError(f"TrajFkBuilder: moveit RobotState import fehlgeschlagen: {e!r}")

        pts = list(seg.points or [])
        if not pts:
            return []

        state = RobotState(robot_model)
        state.set_to_default_values()

        out: List[PoseQuat] = []

        for p in pts:
            q = list(p.positions or [])
            if not q:
                continue
            if len(q) != len(seg.joint_names):
                continue

            try:
                state.set_variable_positions(seg.joint_names, q)
                state.update()
                tf = state.get_global_link_transform(cfg.ee_link)
                pq = _tf_to_posequat_mm(tf, meters_to_mm=float(cfg.meters_to_mm))
                if pq is not None:
                    out.append(pq)
            except Exception:
                continue

        if len(out) >= 2:
            filtered = [out[0]]
            for i in range(1, len(out)):
                a = filtered[-1]
                b = out[i]
                d = _vec_norm3(b.x - a.x, b.y - a.y, b.z - a.z)
                if d > 1e-9:
                    filtered.append(b)
            out = filtered

        return out

    @staticmethod
    def _resample_posequats_by_step_mm(poses: List[PoseQuat], *, step_mm: float, max_points: int) -> List[PoseQuat]:
        if len(poses) < 2:
            return list(poses)

        step_mm = float(step_mm)
        if step_mm <= 0.0:
            return list(poses)

        s: List[float] = [0.0]
        for i in range(1, len(poses)):
            a = poses[i - 1]
            b = poses[i]
            ds = _vec_norm3(b.x - a.x, b.y - a.y, b.z - a.z)
            s.append(s[-1] + ds)

        total = s[-1]
        if total <= 1e-9:
            return [poses[0]]

        n = int(math.floor(total / step_mm)) + 1
        targets = [i * step_mm for i in range(n)]
        if targets[-1] < total:
            targets.append(total)

        out: List[PoseQuat] = []
        j = 0

        for st in targets:
            while j < len(s) - 2 and s[j + 1] < st:
                j += 1

            s0 = s[j]
            s1 = s[j + 1]
            a = poses[j]
            b = poses[j + 1]

            alpha = 0.0 if s1 <= s0 else (st - s0) / (s1 - s0)
            alpha = _clamp(alpha, 0.0, 1.0)

            x = a.x + (b.x - a.x) * alpha
            y = a.y + (b.y - a.y) * alpha
            z = a.z + (b.z - a.z) * alpha

            qa = (a.qx, a.qy, a.qz, a.qw)
            qb = (b.qx, b.qy, b.qz, b.qw)
            qx, qy, qz, qw = _quat_slerp(qa, qb, alpha)

            out.append(PoseQuat(x=x, y=y, z=z, qx=qx, qy=qy, qz=qz, qw=qw))

            if max_points and max_points > 0 and len(out) >= max_points:
                break

        return out
