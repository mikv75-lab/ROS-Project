# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_markers.py
from __future__ import annotations

from typing import Optional, Any, Dict

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion  # type: ignore
from visualization_msgs.msg import Marker, MarkerArray  # type: ignore

from .recipe import Recipe


def _make_point(x_mm: float, y_mm: float, z_mm: float) -> Point:
    # RViz expects meters
    p = Point()
    p.x = float(x_mm) / 1000.0
    p.y = float(y_mm) / 1000.0
    p.z = float(z_mm) / 1000.0
    return p


def _make_pose_quat_mm(x_mm: float, y_mm: float, z_mm: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
    # RViz expects meters
    p = Pose()
    p.position.x = float(x_mm) / 1000.0
    p.position.y = float(y_mm) / 1000.0
    p.position.z = float(z_mm) / 1000.0
    p.orientation = Quaternion(x=float(qx), y=float(qy), z=float(qz), w=float(qw))
    return p


def _line_strip_marker(*, frame_id: str, ns: str, mid: int) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = int(mid)
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.0015  # line width in meters
    m.color.a = 1.0
    m.color.r = 0.2
    m.color.g = 0.9
    m.color.b = 0.2
    return m


def _text_marker(
    *,
    frame_id: str,
    ns: str,
    mid: int,
    text: str,
    x_m: float = 0.0,
    y_m: float = 0.0,
    z_m: float = 0.0,
    size_m: float = 0.03,
) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.ns = ns
    m.id = int(mid)
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = float(x_m)
    m.pose.position.y = float(y_m)
    m.pose.position.z = float(z_m)
    m.pose.orientation.w = 1.0
    m.scale.z = float(size_m)
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.text = str(text or "")
    return m


def _score_from_eval(ev: Dict[str, Any]) -> Optional[float]:
    if not isinstance(ev, dict):
        return None
    total = ev.get("total")
    if isinstance(total, dict):
        s = total.get("score")
        if isinstance(s, (int, float)):
            return float(s)
    s = ev.get("score")
    if isinstance(s, (int, float)):
        return float(s)
    return None


def _threshold_from_eval(ev: Dict[str, Any]) -> Optional[float]:
    if not isinstance(ev, dict):
        return None
    t = ev.get("threshold")
    if isinstance(t, (int, float)):
        return float(t)
    return None


def _valid_from_eval(ev: Dict[str, Any]) -> Optional[bool]:
    if not isinstance(ev, dict):
        return None
    v = ev.get("valid")
    if isinstance(v, bool):
        return bool(v)
    return None


def _fmt_eval_line(prefix: str, ev: Dict[str, Any]) -> str:
    s = _score_from_eval(ev)
    t = _threshold_from_eval(ev)
    v = _valid_from_eval(ev)
    parts = [prefix]
    if s is not None:
        parts.append(f"score={s:.3f}")
    if t is not None:
        parts.append(f"thr={t:.3f}")
    if v is not None:
        parts.append(f"valid={v}")
    return " | ".join(parts)


# ============================================================
# NEW: TCP YAML -> PoseArray + MarkerArray
# ============================================================

def _as_float(v: Any) -> Optional[float]:
    try:
        if v is None:
            return None
        return float(v)
    except Exception:
        return None


def build_tcp_pose_array_from_tcp_yaml(
    tcp_doc: Dict[str, Any],
    *,
    default_frame: str = "scene",
) -> PoseArray:
    """
    Convert persisted tcp yaml (planned_tcp.yaml / executed_tcp.yaml) into PoseArray.

    Expected (your TrajFkBuilder Draft schema):
      {
        "frame": "scene",
        "sides": {
          "<side>": {
            "poses_quat": [
              {"x":mm,"y":mm,"z":mm,"qx":...,"qy":...,"qz":...,"qw":...},
              ...
            ]
          }
        }
      }

    Units: positions are mm -> meters.
    """
    pa = PoseArray()
    if not isinstance(tcp_doc, dict):
        pa.header.frame_id = default_frame
        return pa

    frame_id = str(tcp_doc.get("frame") or default_frame)
    pa.header.frame_id = frame_id

    sides = tcp_doc.get("sides")
    if not isinstance(sides, dict):
        return pa

    for _, side_doc in sides.items():
        if not isinstance(side_doc, dict):
            continue
        pq = side_doc.get("poses_quat")
        if not isinstance(pq, list):
            continue
        for d in pq:
            if not isinstance(d, dict):
                continue
            x = _as_float(d.get("x"))
            y = _as_float(d.get("y"))
            z = _as_float(d.get("z"))
            qx = _as_float(d.get("qx"))
            qy = _as_float(d.get("qy"))
            qz = _as_float(d.get("qz"))
            qw = _as_float(d.get("qw"))
            if None in (x, y, z, qx, qy, qz, qw):
                continue
            pa.poses.append(_make_pose_quat_mm(x, y, z, qx, qy, qz, qw))

    return pa


def build_tcp_marker_array_from_tcp_yaml(
    tcp_doc: Dict[str, Any],
    *,
    ns_prefix: str,
    mid_start: int = 20000,
    default_frame: str = "scene",
    include_text: bool = True,
) -> MarkerArray:
    """
    Build LINE_STRIP markers for TCP (per side), so RViz shows the TCP path.

    Namespaces:
      - f"{ns_prefix}/<side>"
      - optional text marker at first point: ns_prefix
    """
    arr = MarkerArray()
    if not isinstance(tcp_doc, dict):
        return arr

    frame_id = str(tcp_doc.get("frame") or default_frame)
    sides = tcp_doc.get("sides")
    if not isinstance(sides, dict) or not sides:
        return arr

    mid = int(mid_start)

    # anchor for optional text marker
    anchor_x = anchor_y = anchor_z = None

    total_pts = 0
    for side, side_doc in sides.items():
        if not isinstance(side_doc, dict):
            continue
        pq = side_doc.get("poses_quat")
        if not isinstance(pq, list) or not pq:
            continue

        m = _line_strip_marker(frame_id=frame_id, ns=f"{ns_prefix}/{side}", mid=mid)
        mid += 1

        for d in pq:
            if not isinstance(d, dict):
                continue
            x = _as_float(d.get("x"))
            y = _as_float(d.get("y"))
            z = _as_float(d.get("z"))
            if None in (x, y, z):
                continue
            m.points.append(_make_point(x, y, z))
            total_pts += 1

        if m.points:
            if anchor_x is None:
                p0 = m.points[0]
                anchor_x, anchor_y, anchor_z = float(p0.x), float(p0.y), float(p0.z)
            arr.markers.append(m)

    if include_text and anchor_x is not None:
        arr.markers.append(
            _text_marker(
                frame_id=frame_id,
                ns=ns_prefix,
                mid=mid,
                text=f"{ns_prefix}: points={total_pts}",
                x_m=float(anchor_x),
                y_m=float(anchor_y),
                z_m=float(anchor_z) + 0.04,
                size_m=0.022,
            )
        )

    return arr


# ============================================================
# Existing: Recipe -> MarkerArray (Draft + Traj summary/eval)
# ============================================================

def build_marker_array_from_recipe(
    recipe: Recipe,
    *,
    frame_id: str = "substrate",
    show_draft: bool = True,
    show_planned: bool = True,
    show_executed: bool = True,
) -> MarkerArray:
    """
    UI visualization:
      - draft.yaml -> LINE_STRIP per side (workspace TCP poses in mm -> meters)
      - planned_traj / executed_traj:
          * summary TEXT marker (segments/points)
          * optional eval TEXT line (from recipe.meta["planned_eval"]/["executed_eval"])
    Marker namespaces used for downstream splitting (IMPORTANT):
      - "draft/<side>"
      - "planned_traj"
      - "executed_traj"
    """
    arr = MarkerArray()
    mid = 0

    # --- Draft (workspace path) ---
    if show_draft and getattr(recipe, "draft", None) is not None:
        try:
            for side, s in recipe.draft.sides.items():  # type: ignore[union-attr]
                m = _line_strip_marker(frame_id=frame_id, ns=f"draft/{side}", mid=mid)
                mid += 1
                for pose in s.poses_quat:
                    m.points.append(_make_point(pose.x, pose.y, pose.z))
                arr.markers.append(m)
        except Exception:
            pass

    def _traj_summary(kind: str) -> Optional[str]:
        traj = recipe.planned_traj if kind == "planned" else recipe.executed_traj
        if traj is None:
            return None
        segs = traj.segments
        total_pts = sum(len(s.points) for s in segs.values())
        seg_names = ", ".join(segs.keys())
        if len(seg_names) > 80:
            seg_names = seg_names[:77] + "..."
        return f"{kind}_traj: segments={len(segs)}, points={total_pts}\n{seg_names}"

    # anchor text near first draft point (if available)
    anchor_x = anchor_y = anchor_z = 0.0
    try:
        if getattr(recipe, "draft", None) is not None:
            for s in recipe.draft.sides.values():  # type: ignore[union-attr]
                if s.poses_quat:
                    p0 = s.poses_quat[0]
                    anchor_x = float(p0.x) / 1000.0
                    anchor_y = float(p0.y) / 1000.0
                    anchor_z = float(p0.z) / 1000.0 + 0.05
                    break
    except Exception:
        pass

    # eval dicts (mirrored by ProcessTab)
    meta = getattr(recipe, "meta", {}) or {}
    planned_eval = meta.get("planned_eval") if isinstance(meta, dict) else None
    executed_eval = meta.get("executed_eval") if isinstance(meta, dict) else None
    planned_eval = planned_eval if isinstance(planned_eval, dict) else {}
    executed_eval = executed_eval if isinstance(executed_eval, dict) else {}

    # --- Planned / Executed (text markers) ---
    if show_planned:
        txt = _traj_summary("planned")
        if txt:
            arr.markers.append(
                _text_marker(
                    frame_id=frame_id,
                    ns="planned_traj",
                    mid=mid,
                    text=txt,
                    x_m=anchor_x,
                    y_m=anchor_y,
                    z_m=anchor_z,
                    size_m=0.03,
                )
            )
            mid += 1

        if planned_eval:
            arr.markers.append(
                _text_marker(
                    frame_id=frame_id,
                    ns="planned_traj",  # IMPORTANT: keep same ns so your publisher includes it
                    mid=mid,
                    text=_fmt_eval_line("EVAL PLANNED", planned_eval),
                    x_m=anchor_x,
                    y_m=anchor_y,
                    z_m=anchor_z - 0.03,
                    size_m=0.022,
                )
            )
            mid += 1

    if show_executed:
        txt = _traj_summary("executed")
        if txt:
            arr.markers.append(
                _text_marker(
                    frame_id=frame_id,
                    ns="executed_traj",
                    mid=mid,
                    text=txt,
                    x_m=anchor_x,
                    y_m=anchor_y,
                    z_m=anchor_z + 0.07,
                    size_m=0.03,
                )
            )
            mid += 1

        if executed_eval:
            arr.markers.append(
                _text_marker(
                    frame_id=frame_id,
                    ns="executed_traj",  # IMPORTANT: keep same ns so your publisher includes it
                    mid=mid,
                    text=_fmt_eval_line("EVAL EXECUTED", executed_eval),
                    x_m=anchor_x,
                    y_m=anchor_y,
                    z_m=anchor_z + 0.04,
                    size_m=0.022,
                )
            )
            mid += 1

    return arr
