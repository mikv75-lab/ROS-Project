# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_markers.py
from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Point  # type: ignore
from visualization_msgs.msg import Marker, MarkerArray  # type: ignore

from .recipe import Recipe


def _make_point(x_mm: float, y_mm: float, z_mm: float) -> Point:
    # RViz expects meters
    p = Point()
    p.x = float(x_mm) / 1000.0
    p.y = float(y_mm) / 1000.0
    p.z = float(z_mm) / 1000.0
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
    m.scale.z = 0.03
    m.color.a = 1.0
    m.color.r = 1.0
    m.color.g = 1.0
    m.color.b = 1.0
    m.text = text
    return m


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
      - planned_traj / executed_traj -> summary TEXT marker (segments/points)

    Marker namespaces used for downstream splitting:
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
            # draft schema mismatch -> just skip draft markers
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
                    z_m=anchor_z + 0.05,
                )
            )
            mid += 1

    return arr
