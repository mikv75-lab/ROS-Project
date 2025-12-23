# app/model/recipe/recipe_markers.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Iterable, List, Optional, Literal

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .recipe import Recipe


# ----------------------------
# Types
# ----------------------------

SourceKey = Literal["compiled_path", "traj", "executed_traj"]


# ----------------------------
# Marker helpers
# ----------------------------

def _make_point(x: float, y: float, z: float) -> Point:
    p = Point()
    p.x = float(x)
    p.y = float(y)
    p.z = float(z)
    return p


def _points_mm_to_markerarray(
    points_mm: np.ndarray,
    *,
    frame_id: str,
    name: str,
    ns: Optional[str] = None,
    line_id: int = 0,
    dots_id: int = 1,
    text_id: int = 2,
) -> MarkerArray:
    """
    Baut aus mm-Punkten ein MarkerArray:
      - LINE_STRIP für den Verlauf
      - SPHERE_LIST für einzelne Punkte
      - optional TEXT_VIEW_FACING am Startpunkt (name)

    IDs sind parametrisierbar, damit mehrere Quellen in einem gemeinsamen Array
    nicht kollidieren.
    """
    ma = MarkerArray()
    if points_mm is None or np.size(points_mm) == 0:
        return ma

    P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
    if P.shape[0] == 0:
        return ma

    # mm -> m (ROS/RViz)
    Pm = P * 1e-3

    ns = ns or (name or "recipe_path")

    # ---------------- LINE_STRIP ----------------
    line = Marker()
    line.header.frame_id = frame_id
    line.ns = ns
    line.id = int(line_id)
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.scale.x = 0.002  # 2 mm
    line.color.r = 0.0
    line.color.g = 1.0
    line.color.b = 0.0
    line.color.a = 1.0
    line.pose.orientation.w = 1.0
    for x, y, z in Pm:
        line.points.append(_make_point(x, y, z))
    ma.markers.append(line)

    # ---------------- SPHERE_LIST ----------------
    dots = Marker()
    dots.header.frame_id = frame_id
    dots.ns = ns
    dots.id = int(dots_id)
    dots.type = Marker.SPHERE_LIST
    dots.action = Marker.ADD
    dots.scale.x = 0.004  # 4 mm
    dots.scale.y = 0.004
    dots.scale.z = 0.004
    dots.color.r = 1.0
    dots.color.g = 0.0
    dots.color.b = 0.0
    dots.color.a = 1.0
    dots.pose.orientation.w = 1.0
    for x, y, z in Pm:
        dots.points.append(_make_point(x, y, z))
    ma.markers.append(dots)

    # ---------------- Text am Startpunkt ----------------
    if name:
        txt = Marker()
        txt.header.frame_id = frame_id
        txt.ns = ns
        txt.id = int(text_id)
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.text = name
        txt.scale.z = 0.02  # 20 mm
        txt.color.r = 1.0
        txt.color.g = 1.0
        txt.color.b = 1.0
        txt.color.a = 1.0
        txt.pose.orientation.w = 1.0

        x0, y0, z0 = Pm[0]
        txt.pose.position.x = float(x0)
        txt.pose.position.y = float(y0)
        txt.pose.position.z = float(z0) + 0.01  # 10 mm darüber
        ma.markers.append(txt)

    return ma


# ----------------------------
# Strict extraction
# ----------------------------

def _poses_quat_to_points_mm(poses: list) -> np.ndarray:
    """
    Erwartet: Liste von Dicts mit x/y/z (mm).
    """
    if not poses:
        return np.zeros((0, 3), dtype=float)
    return np.array(
        [[float(p["x"]), float(p["y"]), float(p["z"])] for p in poses],
        dtype=float,
    ).reshape(-1, 3)


def _require_dict(obj, path: str) -> dict:
    if not isinstance(obj, dict):
        raise TypeError(f"Expected dict at {path}, got {type(obj).__name__}")
    return obj


def _points_mm_for_source_side(recipe: Recipe, source: SourceKey, side: str) -> np.ndarray:
    """
    Strict reader für Punkte (mm).

    source:
      - "compiled_path" -> recipe.paths_compiled["sides"][side]["poses_quat"]
      - "traj"          -> recipe.trajectories["traj"]["sides"][side]["poses_quat"]
      - "executed_traj" -> recipe.trajectories["executed_traj"]["sides"][side]["poses_quat"]
    """
    side = str(side)

    if source == "compiled_path":
        pc = _require_dict(recipe.paths_compiled, "recipe.paths_compiled")
        sides = _require_dict(pc.get("sides"), "recipe.paths_compiled['sides']")
        sdata = _require_dict(sides.get(side), f"recipe.paths_compiled['sides']['{side}']")
        poses = sdata.get("poses_quat")
        if poses is None:
            raise KeyError(f"Missing poses_quat at recipe.paths_compiled['sides']['{side}']['poses_quat']")
        return _poses_quat_to_points_mm(poses)

    traj_root = _require_dict(recipe.trajectories, "recipe.trajectories")
    tdata = _require_dict(traj_root.get(source), f"recipe.trajectories['{source}']")
    sides = _require_dict(tdata.get("sides"), f"recipe.trajectories['{source}']['sides']")
    sdata = _require_dict(sides.get(side), f"recipe.trajectories['{source}']['sides']['{side}']")
    poses = sdata.get("poses_quat")
    if poses is None:
        raise KeyError(f"Missing poses_quat at recipe.trajectories['{source}']['sides']['{side}']['poses_quat']")
    return _poses_quat_to_points_mm(poses)


def _frame_for_source(recipe: Recipe, source: SourceKey, default: str) -> str:
    """
    Strict frame reader:
      - compiled_path -> recipe.paths_compiled["frame"]
      - traj/executed -> recipe.trajectories[source]["frame"]
    """
    if source == "compiled_path":
        pc = _require_dict(recipe.paths_compiled, "recipe.paths_compiled")
        return str(pc.get("frame") or default)

    traj_root = _require_dict(recipe.trajectories, "recipe.trajectories")
    tdata = _require_dict(traj_root.get(source), f"recipe.trajectories['{source}']")
    return str(tdata.get("frame") or default)


# ----------------------------
# Public API
# ----------------------------

def build_marker_array_for_recipe_side(
    recipe: Recipe,
    *,
    side: str,
    frame_id: str,
    source: SourceKey = "compiled_path",
    name: Optional[str] = None,
    ns: Optional[str] = None,
) -> MarkerArray:
    """
    Baut Marker für *eine Side* aus einer Quelle.

    source (strict):
      - "compiled_path" | "traj" | "executed_traj"
    """
    pts_mm = _points_mm_for_source_side(recipe, source, side)
    if pts_mm.size == 0:
        return MarkerArray()

    base_name = (name if name is not None else (recipe.id or "")).strip()
    label = base_name if base_name else f"{source}:{side}"
    ns_out = (ns or base_name or f"{source}_{side}").strip()

    frame = _frame_for_source(recipe, source, frame_id)

    return _points_mm_to_markerarray(
        pts_mm,
        frame_id=frame,
        name=label,
        ns=ns_out,
    )


def build_marker_array_from_recipe(
    recipe: Recipe,
    *,
    sides: Optional[Iterable[str]] = None,
    frame_id: str = "scene",
    source: SourceKey = "compiled_path",
    name: Optional[str] = None,
    ns_prefix: Optional[str] = None,
) -> MarkerArray:
    """
    Baut ein gemeinsames MarkerArray für mehrere Seiten aus einer Quelle.

    source (strict):
      - "compiled_path" | "traj" | "executed_traj"
    """
    ma = MarkerArray()

    if sides is not None:
        side_list: List[str] = [str(s) for s in sides]
    else:
        if source == "compiled_path":
            pc = _require_dict(recipe.paths_compiled, "recipe.paths_compiled")
            sdata = _require_dict(pc.get("sides"), "recipe.paths_compiled['sides']")
        else:
            traj_root = _require_dict(recipe.trajectories, "recipe.trajectories")
            tdata = _require_dict(traj_root.get(source), f"recipe.trajectories['{source}']")
            sdata = _require_dict(tdata.get("sides"), f"recipe.trajectories['{source}']['sides']")

        side_list = list(sdata.keys())

    if not side_list:
        return ma

    frame = _frame_for_source(recipe, source, frame_id)

    global_id = 0
    for side in side_list:
        base_name = (name if name is not None else (recipe.id or "")).strip()
        label = base_name if base_name else f"{source}:{side}"
        ns = (ns_prefix or base_name or source).strip()

        pts_mm = _points_mm_for_source_side(recipe, source, side)
        if pts_mm.size == 0:
            continue

        side_ma = _points_mm_to_markerarray(
            pts_mm,
            frame_id=frame,
            name=label,
            ns=f"{ns}/{side}",
            line_id=global_id + 0,
            dots_id=global_id + 1,
            text_id=global_id + 2,
        )
        ma.markers.extend(side_ma.markers)
        global_id += 3

    return ma
