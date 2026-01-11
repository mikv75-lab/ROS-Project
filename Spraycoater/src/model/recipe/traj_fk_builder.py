# -*- coding: utf-8 -*-
# File: src/model/recipe/traj_fk_builder.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Mapping, Optional, Tuple

import math
import os
import tempfile

from .recipe import Draft, PoseQuat, PathSide, JTBySegment, JTSegment


# ------------------------------------------------------------------
# Tempfile retention:
# Some MoveIt loaders parse lazily, so deleting temp files immediately
# can cause "XML_ERROR_EMPTY_DOCUMENT" / "Could not open file [<?xml ...>]"
# ------------------------------------------------------------------
_RETAINED_MODEL_FILES: List[str] = []


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


def _quat_slerp(
    qa: Tuple[float, float, float, float],
    qb: Tuple[float, float, float, float],
    t: float,
) -> Tuple[float, float, float, float]:
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
    ee_link: str = "tcp"
    meters_to_mm: float = 1000.0
    step_mm: float = 1.0
    max_points: int = 0  # 0 = unlimited


class TrajFkBuilder:
    # ============================================================
    # XML/file-path robust loader
    # ============================================================

    @staticmethod
    def _looks_like_xml(s: str) -> bool:
        ss = (s or "").lstrip()
        return bool(ss) and ss[0] == "<"

    @staticmethod
    def _write_temp_file(text: str, *, suffix: str) -> str:
        fd, path = tempfile.mkstemp(prefix="spraycoater_model_", suffix=suffix)
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text or "")
            # Make sure content is fully written to disk (helps with timing-sensitive loaders)
            f.flush()
            try:
                os.fsync(f.fileno())
            except Exception:
                pass
        # RETAIN (do not delete immediately; some loaders parse lazily)
        _RETAINED_MODEL_FILES.append(path)
        return path

    @staticmethod
    def build_robot_model_from_urdf_srdf(*, urdf_xml: str, srdf_xml: str) -> Any:
        """
        Accepts URDF/SRDF as XML strings OR as file paths.

        IMPORTANT (Rolling pitfall):
          Some MoveIt/urdfdom bindings interpret passed strings as filenames.
          If you pass XML directly, you can get noisy logs like:
            "Could not open file [<?xml ...]" / "XML_ERROR_EMPTY_DOCUMENT"

        Strategy:
          - If input looks like XML => write to tempfiles FIRST and only then call bindings.
            (prevents the noisy "open file [<?xml ...]" direct-attempt spam)
          - Else (non-XML) => treat as path/param and try directly.
        """
        urdf_in = str(urdf_xml or "")
        srdf_in = str(srdf_xml or "")

        if not urdf_in.strip():
            raise ValueError("TrajFkBuilder: urdf ist leer.")
        if not srdf_in.strip():
            raise ValueError("TrajFkBuilder: srdf ist leer.")

        def _try_build(urdf_arg: str, srdf_arg: str) -> Any:
            last_exc: Optional[Exception] = None

            try:
                from moveit.core.robot_model import RobotModel  # type: ignore
                return RobotModel(urdf_arg, srdf_arg)  # type: ignore[call-arg]
            except Exception as e:
                last_exc = e

            try:
                from moveit.core._moveit_core import RobotModel  # type: ignore
                return RobotModel(urdf_arg, srdf_arg)  # type: ignore[call-arg]
            except Exception as e:
                last_exc = e

            try:
                from moveit.core.robot_model_loader import RobotModelLoader  # type: ignore
                loader = RobotModelLoader(urdf_arg, srdf_arg)  # type: ignore[call-arg]
                fn = getattr(loader, "get_model", None)
                if callable(fn):
                    return fn()
                m = getattr(loader, "model", None)
                if m is not None:
                    return m
            except Exception as e:
                last_exc = e

            if last_exc is not None:
                raise last_exc
            raise RuntimeError("TrajFkBuilder: robot_model build failed (no binding path).")

        urdf_is_xml = TrajFkBuilder._looks_like_xml(urdf_in)
        srdf_is_xml = TrajFkBuilder._looks_like_xml(srdf_in)

        # If it looks like XML, DO NOT attempt a "direct" call first.
        # Rolling frequently interprets strings as file paths and will spam urdfdom errors.
        if urdf_is_xml or srdf_is_xml:
            urdf_arg = TrajFkBuilder._write_temp_file(urdf_in, suffix=".urdf") if urdf_is_xml else urdf_in
            srdf_arg = TrajFkBuilder._write_temp_file(srdf_in, suffix=".srdf") if srdf_is_xml else srdf_in
            try:
                return _try_build(urdf_arg, srdf_arg)
            except Exception as e_file:
                raise RuntimeError(
                    "TrajFkBuilder: konnte robot_model nicht aus URDF/SRDF bauen (XML->Tempfile). "
                    f"Letzter Fehler: {e_file}"
                ) from e_file

        # Non-XML inputs: treat as paths and try directly
        try:
            return _try_build(urdf_in, srdf_in)
        except Exception as e_direct:
            raise RuntimeError(f"TrajFkBuilder: robot_model build failed from paths: {e_direct}") from e_direct

    # ============================================================
    # Existing API (unverändert)
    # ============================================================

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
        d = TrajFkBuilder.build_tcp_draft(
            traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=default_side,
            drop_duplicate_boundary=drop_duplicate_boundary,
        )
        return TrajFkBuilder._draft_to_yaml_dict(d)

    @staticmethod
    def _draft_to_yaml_dict(d: Draft) -> Dict[str, Any]:
        for fn_name in ("to_yaml_dict", "to_dict", "as_dict"):
            fn = getattr(d, fn_name, None)
            if callable(fn):
                out = fn()
                return out if isinstance(out, dict) else {}
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
