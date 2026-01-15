# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_markers.py
from __future__ import annotations

from typing import Optional, Any, Dict, Tuple, List, Iterable

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion  # type: ignore
from visualization_msgs.msg import Marker, MarkerArray  # type: ignore

from .recipe import Recipe


# ============================================================
# Low-level helpers
# ============================================================

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


def _apply_rgba(m: Marker, rgba: Tuple[float, float, float, float]) -> None:
    m.color.r = float(rgba[0])
    m.color.g = float(rgba[1])
    m.color.b = float(rgba[2])
    m.color.a = float(rgba[3])


def _line_strip_marker(
    *,
    frame_id: str,
    ns: str,
    mid: int,
    width_m: float = 0.0010,
    rgba: Tuple[float, float, float, float] = (0.2, 0.9, 0.2, 1.0),
) -> Marker:
    """
    LINE_STRIP is cheap and good for path overview.
    'width_m' controls thickness (meters).
    Note: RViz line strips are not truly round; they are billboarded quads.
    """
    m = Marker()
    m.header.frame_id = str(frame_id)
    m.ns = str(ns)
    m.id = int(mid)
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(width_m)  # line width in meters
    _apply_rgba(m, rgba)
    return m


def _points_marker(
    *,
    frame_id: str,
    ns: str,
    mid: int,
    diameter_m: float = 0.0015,
    rgba: Tuple[float, float, float, float] = (0.2, 0.9, 0.2, 1.0),
) -> Marker:
    """
    POINTS marker draws camera-facing squares; with a small size it looks like a dotted 'round' line.
    If you want actual spheres, use SPHERE_LIST (heavier).
    """
    m = Marker()
    m.header.frame_id = str(frame_id)
    m.ns = str(ns)
    m.id = int(mid)
    m.type = Marker.POINTS
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(diameter_m)
    m.scale.y = float(diameter_m)
    _apply_rgba(m, rgba)
    return m


def _sphere_list_marker(
    *,
    frame_id: str,
    ns: str,
    mid: int,
    diameter_m: float = 0.0018,
    rgba: Tuple[float, float, float, float] = (0.2, 0.9, 0.2, 1.0),
) -> Marker:
    """
    SPHERE_LIST gives truly round beads, best visibility of curvature,
    but heavier than LINE_STRIP/POINTS.
    """
    m = Marker()
    m.header.frame_id = str(frame_id)
    m.ns = str(ns)
    m.id = int(mid)
    m.type = Marker.SPHERE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(diameter_m)
    m.scale.y = float(diameter_m)
    m.scale.z = float(diameter_m)
    _apply_rgba(m, rgba)
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
    m.header.frame_id = str(frame_id)
    m.ns = str(ns)
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


def _deleteall_marker(frame_id: str) -> Marker:
    """
    RViz marker topic hard-clear. This removes ALL markers currently shown on this topic.
    """
    m = Marker()
    m.header.frame_id = str(frame_id)
    m.action = Marker.DELETEALL
    return m


# ============================================================
# Schema parsing helpers
# ============================================================

def _as_float(v: Any) -> Optional[float]:
    try:
        if v is None:
            return None
        return float(v)
    except Exception:
        return None


def _to_tcp_doc(obj: Any) -> Dict[str, Any]:
    """
    Accept:
      - dict
      - Draft-like object with .to_yaml_dict() / .to_dict() / .as_dict()
      - else: {}
    """
    if obj is None:
        return {}
    if isinstance(obj, dict):
        return obj
    for meth in ("to_yaml_dict", "to_dict", "as_dict"):
        fn = getattr(obj, meth, None)
        if callable(fn):
            try:
                d = fn()
                return d if isinstance(d, dict) else {}
            except Exception:
                return {}
    return {}


# ============================================================
# RViz outputs (SSoT): 3 layers, each PoseArray + MarkerArray
#   - draft (compiled)
#   - planned_tcp
#   - executed_tcp
# ============================================================

def build_draft_pose_and_markers(
    recipe: Recipe,
    *,
    frame_id: str = "substrate",
    ns_prefix: str = "draft",
    clear_legacy: bool = True,
    line_width_m: float = 0.0008,
    rgba_line: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0),  # BLUE
    round_style: str = "none",  # "none" | "points" | "spheres"
    round_diameter_m: float = 0.0012,
    rgba_round: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[PoseArray, MarkerArray]:
    """
    Draft (workspace path) -> PoseArray + MarkerArray (LINE_STRIP per side).
    Units: mm -> meters.

    round_style:
      - "none": only LINE_STRIP
      - "points": add POINTS marker along the same points (cheap)
      - "spheres": add SPHERE_LIST (round, heavier)
    """
    fid = str(frame_id)

    pa = PoseArray()
    pa.header.frame_id = fid

    ma = MarkerArray()
    if clear_legacy:
        ma.markers.append(_deleteall_marker(fid))

    d = getattr(recipe, "draft", None)
    if d is None:
        return pa, ma

    sides = getattr(d, "sides", None)
    if not isinstance(sides, dict) or not sides:
        return pa, ma

    if rgba_round is None:
        rgba_round = rgba_line

    mid = 0
    for side_name, side in sides.items():
        poses = getattr(side, "poses_quat", None)
        if not isinstance(poses, list) or not poses:
            continue

        # LINE
        m_line = _line_strip_marker(
            frame_id=fid,
            ns=f"{ns_prefix}/{side_name}",
            mid=mid,
            width_m=line_width_m,
            rgba=rgba_line,
        )
        mid += 1

        # optional round overlay
        m_round: Optional[Marker] = None
        if round_style == "points":
            m_round = _points_marker(
                frame_id=fid,
                ns=f"{ns_prefix}/{side_name}/round",
                mid=mid,
                diameter_m=round_diameter_m,
                rgba=rgba_round,
            )
            mid += 1
        elif round_style == "spheres":
            m_round = _sphere_list_marker(
                frame_id=fid,
                ns=f"{ns_prefix}/{side_name}/round",
                mid=mid,
                diameter_m=round_diameter_m,
                rgba=rgba_round,
            )
            mid += 1

        for pq in poses:
            try:
                pa.poses.append(_make_pose_quat_mm(pq.x, pq.y, pq.z, pq.qx, pq.qy, pq.qz, pq.qw))
                pt = _make_point(pq.x, pq.y, pq.z)
                m_line.points.append(pt)
                if m_round is not None:
                    m_round.points.append(pt)
            except Exception:
                continue

        if m_line.points:
            ma.markers.append(m_line)
            if m_round is not None and m_round.points:
                ma.markers.append(m_round)

    return pa, ma


def build_tcp_pose_and_markers_from_tcp_yaml(
    tcp_doc: Dict[str, Any],
    *,
    ns_prefix: str,
    mid_start: int,
    default_frame: str = "substrate",
    force_frame: Optional[str] = None,
    clear_legacy: bool = True,
    include_text: bool = False,
    line_width_m: float = 0.0008,
    rgba_line: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0),  # GREEN
    round_style: str = "none",  # "none" | "points" | "spheres"
    round_diameter_m: float = 0.0012,
    rgba_round: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[PoseArray, MarkerArray]:
    """
    TCP yaml (planned_tcp.yaml / executed_tcp.yaml) -> PoseArray + MarkerArray.

    Expected:
      {
        "frame": "...",
        "sides": {
          "<side>": {"poses_quat": [{"x":mm,"y":mm,"z":mm,"qx":...,"qy":...,"qz":...,"qw":...}, ...]}
        }
      }
    """
    fid = str(force_frame or (tcp_doc.get("frame") if isinstance(tcp_doc, dict) else None) or default_frame)

    pa = PoseArray()
    pa.header.frame_id = fid

    ma = MarkerArray()
    if clear_legacy:
        ma.markers.append(_deleteall_marker(fid))

    if not isinstance(tcp_doc, dict):
        return pa, ma

    sides = tcp_doc.get("sides")
    if not isinstance(sides, dict) or not sides:
        return pa, ma

    if rgba_round is None:
        rgba_round = rgba_line

    mid = int(mid_start)

    anchor_x = anchor_y = anchor_z = None
    total_pts = 0

    for side, side_doc in sides.items():
        if not isinstance(side_doc, dict):
            continue
        pq = side_doc.get("poses_quat")
        if not isinstance(pq, list) or not pq:
            continue

        m_line = _line_strip_marker(
            frame_id=fid,
            ns=f"{ns_prefix}/{side}",
            mid=mid,
            width_m=line_width_m,
            rgba=rgba_line,
        )
        mid += 1

        m_round: Optional[Marker] = None
        if round_style == "points":
            m_round = _points_marker(
                frame_id=fid,
                ns=f"{ns_prefix}/{side}/round",
                mid=mid,
                diameter_m=round_diameter_m,
                rgba=rgba_round,
            )
            mid += 1
        elif round_style == "spheres":
            m_round = _sphere_list_marker(
                frame_id=fid,
                ns=f"{ns_prefix}/{side}/round",
                mid=mid,
                diameter_m=round_diameter_m,
                rgba=rgba_round,
            )
            mid += 1

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
            pt = _make_point(x, y, z)
            m_line.points.append(pt)
            if m_round is not None:
                m_round.points.append(pt)
            total_pts += 1

        if m_line.points:
            if anchor_x is None:
                p0 = m_line.points[0]
                anchor_x, anchor_y, anchor_z = float(p0.x), float(p0.y), float(p0.z)
            ma.markers.append(m_line)
            if m_round is not None and m_round.points:
                ma.markers.append(m_round)

    if include_text and anchor_x is not None:
        ma.markers.append(
            _text_marker(
                frame_id=fid,
                ns=ns_prefix,
                mid=mid,
                text=f"{ns_prefix}: points={total_pts}",
                x_m=float(anchor_x),
                y_m=float(anchor_y),
                z_m=float(anchor_z) + 0.04,
                size_m=0.022,
            )
        )

    return pa, ma


def build_tcp_pose_and_markers(
    tcp_obj: Any,
    *,
    ns_prefix: str,
    mid_start: int,
    default_frame: str = "substrate",
    force_frame: Optional[str] = None,
    clear_legacy: bool = True,
    include_text: bool = False,
    line_width_m: float = 0.0008,
    rgba_line: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0),
    round_style: str = "none",
    round_diameter_m: float = 0.0012,
    rgba_round: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[PoseArray, MarkerArray]:
    tcp_doc = _to_tcp_doc(tcp_obj)
    return build_tcp_pose_and_markers_from_tcp_yaml(
        tcp_doc,
        ns_prefix=ns_prefix,
        mid_start=mid_start,
        default_frame=default_frame,
        force_frame=force_frame,
        clear_legacy=clear_legacy,
        include_text=include_text,
        line_width_m=line_width_m,
        rgba_line=rgba_line,
        round_style=round_style,
        round_diameter_m=round_diameter_m,
        rgba_round=rgba_round,
    )


def build_rviz_layers(
    recipe: Recipe,
    *,
    frame_id: str = "substrate",
    tcp_default_frame: str = "substrate",
    tcp_force_frame: Optional[str] = None,
    clear_legacy: bool = True,
    # visual styling
    draft_width_m: float = 0.0007,
    tcp_width_m: float = 0.0007,
    draft_rgba: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0),  # blue
    tcp_rgba: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0),    # green
    round_style: str = "none",  # "none" | "points" | "spheres"
    round_diameter_m: float = 0.0012,
) -> Dict[str, Dict[str, Any]]:
    """
    SSoT for RViz publication: returns exactly 3 layers.

    Returns:
      {
        "draft":   {"poses": PoseArray, "markers": MarkerArray},
        "planned": {"poses": PoseArray, "markers": MarkerArray},
        "executed":{"poses": PoseArray, "markers": MarkerArray},
      }
    """
    draft_pa, draft_ma = build_draft_pose_and_markers(
        recipe,
        frame_id=frame_id,
        ns_prefix="draft",
        clear_legacy=clear_legacy,
        line_width_m=draft_width_m,
        rgba_line=draft_rgba,
        round_style=round_style,
        round_diameter_m=round_diameter_m,
        rgba_round=draft_rgba,
    )

    planned_pa, planned_ma = build_tcp_pose_and_markers(
        getattr(recipe, "planned_tcp", None),
        ns_prefix="planned_tcp",
        mid_start=30000,
        default_frame=tcp_default_frame,
        force_frame=tcp_force_frame,
        clear_legacy=clear_legacy,
        include_text=False,
        line_width_m=tcp_width_m,
        rgba_line=tcp_rgba,
        round_style=round_style,
        round_diameter_m=round_diameter_m,
        rgba_round=tcp_rgba,
    )

    executed_pa, executed_ma = build_tcp_pose_and_markers(
        getattr(recipe, "executed_tcp", None),
        ns_prefix="executed_tcp",
        mid_start=40000,
        default_frame=tcp_default_frame,
        force_frame=tcp_force_frame,
        clear_legacy=clear_legacy,
        include_text=False,
        line_width_m=tcp_width_m,
        rgba_line=tcp_rgba,
        round_style=round_style,
        round_diameter_m=round_diameter_m,
        rgba_round=tcp_rgba,
    )

    return {
        "draft": {"poses": draft_pa, "markers": draft_ma},
        "planned": {"poses": planned_pa, "markers": planned_ma},
        "executed": {"poses": executed_pa, "markers": executed_ma},
    }


# ============================================================
# Legacy API compatibility (so your existing imports keep working)
# ============================================================

def build_tcp_pose_array_from_tcp_yaml(
    tcp_doc: Dict[str, Any],
    *,
    default_frame: str = "substrate",
) -> PoseArray:
    pa, _ = build_tcp_pose_and_markers_from_tcp_yaml(
        tcp_doc,
        ns_prefix="tcp",
        mid_start=20000,
        default_frame=default_frame,
        force_frame=None,
        clear_legacy=False,
        include_text=False,
    )
    return pa


def build_tcp_marker_array_from_tcp_yaml(
    tcp_doc: Dict[str, Any],
    *,
    ns_prefix: str,
    mid_start: int = 20000,
    default_frame: str = "substrate",
    include_text: bool = True,
) -> MarkerArray:
    _, ma = build_tcp_pose_and_markers_from_tcp_yaml(
        tcp_doc,
        ns_prefix=ns_prefix,
        mid_start=mid_start,
        default_frame=default_frame,
        force_frame=None,
        clear_legacy=False,
        include_text=include_text,
    )
    return ma


# ============================================================
# Legacy/UI-only: Recipe -> MarkerArray (Draft + Traj summary/eval)
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
    Legacy/UI visualization (NOT RViz path topics):
      - draft.yaml -> LINE_STRIP per side
      - planned_traj / executed_traj -> TEXT markers (summary + eval)
    """
    arr = MarkerArray()
    mid = 0

    # --- Draft (workspace path) ---
    if show_draft and getattr(recipe, "draft", None) is not None:
        try:
            for side, s in recipe.draft.sides.items():  # type: ignore[union-attr]
                m = _line_strip_marker(
                    frame_id=frame_id,
                    ns=f"draft/{side}",
                    mid=mid,
                    width_m=0.0010,
                    rgba=(0.0, 0.0, 1.0, 1.0),  # blue for draft here too
                )
                mid += 1
                for pose in s.poses_quat:
                    m.points.append(_make_point(pose.x, pose.y, pose.z))
                arr.markers.append(m)
        except Exception:
            pass

    # ---- eval helpers (kept local so file stays self-contained) ----
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
                    ns="planned_traj",
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
                    ns="executed_traj",
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
