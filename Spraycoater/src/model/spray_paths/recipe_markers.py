# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_markers.py
from __future__ import annotations

from typing import Optional, Any, Dict, Tuple, List

from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

from model.recipe.recipe import Recipe

# ============================================================
# Low-level helpers
# ============================================================

def _make_point(x_mm: float, y_mm: float, z_mm: float) -> Point:
    p = Point()
    p.x = float(x_mm) / 1000.0
    p.y = float(y_mm) / 1000.0
    p.z = float(z_mm) / 1000.0
    return p


def _make_pose_quat_mm(x_mm: float, y_mm: float, z_mm: float, qx: float, qy: float, qz: float, qw: float) -> Pose:
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
    m = Marker()
    m.header.frame_id = str(frame_id)
    m.ns = str(ns)
    m.id = int(mid)
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(width_m)
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


def _deleteall_marker(frame_id: str) -> Marker:
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
# NEW: Comparison Logic (Ghosting)
# ============================================================

def build_comparison_markers(
    stored_doc: Dict[str, Any],
    new_doc: Dict[str, Any],
    *,
    ns_prefix: str,
    default_frame: str = "substrate",
    mid_start: int = 0
) -> MarkerArray:
    """
    Creates a combined MarkerArray showing both STORED (ghost) and NEW (highlight) paths.
    Used for comparison mode without needing extra ROS topics.
    """
    ma = MarkerArray()
    
    # Clean slate
    frame_id = str(stored_doc.get("frame") or new_doc.get("frame") or default_frame)
    ma.markers.append(_deleteall_marker(frame_id))

    # 1. STORED (Ghost) - Gray, thin, transparent
    if stored_doc:
        _, ma_stored = build_tcp_pose_and_markers_from_tcp_yaml(
            stored_doc,
            ns_prefix=f"{ns_prefix}/stored",
            mid_start=mid_start,
            default_frame=frame_id,
            clear_legacy=False, # Don't clear, we append
            line_width_m=0.0005,
            rgba_line=(0.7, 0.7, 0.7, 0.4), # Ghost gray
            round_style="none"
        )
        ma.markers.extend(ma_stored.markers)
        mid_start += 1000

    # 2. NEW (Active) - Standard color (green/red depending on context), thicker
    if new_doc:
        # Determine color based on type (planned=green, executed=red usually)
        is_exec = "executed" in ns_prefix
        color = (1.0, 0.0, 0.0, 1.0) if is_exec else (0.0, 1.0, 0.0, 1.0)
        
        _, ma_new = build_tcp_pose_and_markers_from_tcp_yaml(
            new_doc,
            ns_prefix=f"{ns_prefix}/new",
            mid_start=mid_start,
            default_frame=frame_id,
            clear_legacy=False,
            line_width_m=0.0012, # Thicker to stand out
            rgba_line=color,
            round_style="points",
            rgba_round=color
        )
        ma.markers.extend(ma_new.markers)

    return ma


# ============================================================
# Core Builders
# ============================================================

def build_draft_pose_and_markers(
    recipe: Recipe,
    *,
    frame_id: str = "substrate",
    ns_prefix: str = "draft",
    clear_legacy: bool = True,
    line_width_m: float = 0.0008,
    rgba_line: Tuple[float, float, float, float] = (0.0, 0.0, 1.0, 1.0),
    round_style: str = "none",
    round_diameter_m: float = 0.0012,
    rgba_round: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[PoseArray, MarkerArray]:
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

        m_line = _line_strip_marker(
            frame_id=fid,
            ns=f"{ns_prefix}/{side_name}",
            mid=mid,
            width_m=line_width_m,
            rgba=rgba_line,
        )
        mid += 1

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
    rgba_line: Tuple[float, float, float, float] = (0.0, 1.0, 0.0, 1.0),
    round_style: str = "none",
    round_diameter_m: float = 0.0012,
    rgba_round: Optional[Tuple[float, float, float, float]] = None,
) -> Tuple[PoseArray, MarkerArray]:
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
            if not isinstance(d, dict): continue
            x, y, z = _as_float(d.get("x")), _as_float(d.get("y")), _as_float(d.get("z"))
            qx, qy, qz, qw = _as_float(d.get("qx")), _as_float(d.get("qy")), _as_float(d.get("qz")), _as_float(d.get("qw"))
            
            if None in (x, y, z, qx, qy, qz, qw): continue

            pa.poses.append(_make_pose_quat_mm(x, y, z, qx, qy, qz, qw)) # type: ignore
            pt = _make_point(x, y, z) # type: ignore
            m_line.points.append(pt)
            if m_round is not None:
                m_round.points.append(pt)

        if m_line.points:
            ma.markers.append(m_line)
            if m_round is not None and m_round.points:
                ma.markers.append(m_round)

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


# ============================================================
# Main SSoT builder for RViz layers
# ============================================================

def build_rviz_layers(
    recipe: Recipe,
    *,
    frame_id: str = "substrate",
    tcp_default_frame: str = "substrate",
    clear_legacy: bool = True,
    # Comparison optional args
    planned_tcp_stored: Optional[Dict] = None,
    executed_tcp_stored: Optional[Dict] = None
) -> Dict[str, Dict[str, Any]]:
    """
    Generates all RViz layers (Compiled, Planned, Executed).
    Handles comparison logic (Stored vs New) internally if provided.
    """
    
    # 1. Draft (Compiled)
    draft_pa, draft_ma = build_draft_pose_and_markers(
        recipe,
        frame_id=frame_id,
        ns_prefix="draft",
        clear_legacy=clear_legacy,
        rgba_line=(0.0, 0.0, 1.0, 1.0), # Blue
    )

    # 2. Planned (Comparision or Single)
    planned_doc = _to_tcp_doc(getattr(recipe, "planned_tcp", None))
    
    if planned_tcp_stored and planned_doc:
        # Comparison Mode
        planned_ma = build_comparison_markers(
            planned_tcp_stored, planned_doc,
            ns_prefix="planned_tcp",
            default_frame=tcp_default_frame,
            mid_start=30000
        )
        planned_pa = PoseArray() # Poses not really useful in comparison mode
    else:
        # Standard Mode
        planned_pa, planned_ma = build_tcp_pose_and_markers(
            planned_doc,
            ns_prefix="planned_tcp",
            mid_start=30000,
            default_frame=tcp_default_frame,
            clear_legacy=clear_legacy,
            rgba_line=(0.0, 1.0, 0.0, 1.0), # Green
        )

    # 3. Executed (Comparision or Single)
    executed_doc = _to_tcp_doc(getattr(recipe, "executed_tcp", None))
    
    if executed_tcp_stored and executed_doc:
        executed_ma = build_comparison_markers(
            executed_tcp_stored, executed_doc,
            ns_prefix="executed_tcp",
            default_frame=tcp_default_frame,
            mid_start=40000
        )
        executed_pa = PoseArray()
    else:
        executed_pa, executed_ma = build_tcp_pose_and_markers(
            executed_doc,
            ns_prefix="executed_tcp",
            mid_start=40000,
            default_frame=tcp_default_frame,
            clear_legacy=clear_legacy,
            rgba_line=(1.0, 0.0, 0.0, 1.0), # Red
        )

    return {
        "draft": {"poses": draft_pa, "markers": draft_ma},
        "planned": {"poses": planned_pa, "markers": planned_ma},
        "executed": {"poses": executed_pa, "markers": executed_ma},
    }


# ============================================================
# Legacy API wrappers (for compatibility)
# ============================================================

def build_tcp_pose_array_from_tcp_yaml(tcp_doc: Dict[str, Any], *, default_frame: str = "substrate") -> PoseArray:
    pa, _ = build_tcp_pose_and_markers_from_tcp_yaml(tcp_doc, ns_prefix="tcp", mid_start=20000, default_frame=default_frame, clear_legacy=False)
    return pa

def build_tcp_marker_array_from_tcp_yaml(tcp_doc: Dict[str, Any], *, ns_prefix: str, mid_start: int = 20000, default_frame: str = "substrate", include_text: bool = True) -> MarkerArray:
    _, ma = build_tcp_pose_and_markers_from_tcp_yaml(tcp_doc, ns_prefix=ns_prefix, mid_start=mid_start, default_frame=default_frame, clear_legacy=False, include_text=include_text)
    return ma

def build_marker_array_from_recipe(recipe: Recipe, *, frame_id: str = "substrate") -> MarkerArray:
    # Minimal stub for old calls, mostly covered by build_rviz_layers now
    _, ma = build_draft_pose_and_markers(recipe, frame_id=frame_id, clear_legacy=False)
    return ma