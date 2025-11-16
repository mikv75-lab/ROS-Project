# app/model/recipe/recipe_markers.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Iterable, List

import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from .recipe import Recipe


def _points_mm_to_markerarray(
    points_mm: np.ndarray,
    *,
    frame_id: str,
    name: str,
    ns: Optional[str] = None,
) -> MarkerArray:
    """
    Baut aus mm-Punkten ein MarkerArray:
      - LINE_STRIP (grün)
      - Punkte (rot)
      - Text (Rezeptname) am Startpunkt
    """
    ma = MarkerArray()
    if points_mm is None or points_mm.size == 0:
        return ma

    P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
    # mm -> m für ROS/RViz
    Pm = P * 1e-3

    ns = ns or (name or "recipe_path")

    # LINE_STRIP
    line = Marker()
    line.header.frame_id = frame_id
    line.ns = ns
    line.id = 0
    line.type = Marker.LINE_STRIP
    line.action = Marker.ADD
    line.scale.x = 0.002  # 2 mm
    line.color.r = 0.0
    line.color.g = 1.0
    line.color.b = 0.0
    line.color.a = 1.0
    line.pose.orientation.w = 1.0
    for x, y, z in Pm:
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        line.points.append(p)
    ma.markers.append(line)

    # Punkte (SPHERE_LIST)
    dots = Marker()
    dots.header.frame_id = frame_id
    dots.ns = ns
    dots.id = 1
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
        p = Point()
        p.x, p.y, p.z = float(x), float(y), float(z)
        dots.points.append(p)
    ma.markers.append(dots)

    # Text (Rezeptname) am Startpunkt
    if name:
        txt = Marker()
        txt.header.frame_id = frame_id
        txt.ns = ns
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.text = name
        txt.scale.z = 0.02  # 20 mm
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.pose.orientation.w = 1.0
        x0, y0, z0 = Pm[0]
        txt.pose.position.x = float(x0)
        txt.pose.position.y = float(y0)
        txt.pose.position.z = float(z0) + 0.01  # 10 mm drüber
        ma.markers.append(txt)

    return ma


def _compiled_points_mm_for_side(recipe: Recipe, side: str) -> np.ndarray:
    """
    Liest die kompilierten Weltpunkte (mm) für eine Side aus recipe.paths_compiled.

    Kein Fallback:
      - wenn paths_compiled oder die Side fehlen -> (0,3)-Array
      - wenn keine poses_quat da sind -> (0,3)-Array
    """
    pc = recipe.paths_compiled or {}
    sides = pc.get("sides") or {}
    sdata = sides.get(str(side))
    if not isinstance(sdata, dict):
        return np.zeros((0, 3), dtype=float)

    poses = sdata.get("poses_quat") or []
    if not poses:
        return np.zeros((0, 3), dtype=float)

    P = np.array(
        [[float(p.get("x", 0.0)),
          float(p.get("y", 0.0)),
          float(p.get("z", 0.0))]
         for p in poses],
        dtype=float,
    ).reshape(-1, 3)
    return P


def build_marker_array_for_recipe_side(
    recipe: Recipe,
    *,
    side: str,
    frame_id: str,
) -> MarkerArray:
    """
    Baut Marker für GENAU EINE Side.

    STRICT:
      - verwendet ausschließlich recipe.paths_compiled["sides"][side]["poses_quat"]
      - keine rebuild_paths, keine Verwendung von recipe.paths, keine Defaults
      - wenn nichts kompiliert ist -> leeres MarkerArray
    """
    pts_mm = _compiled_points_mm_for_side(recipe, side)
    if pts_mm is None or pts_mm.size == 0:
        return MarkerArray()

    name = recipe.id or ""
    return _points_mm_to_markerarray(pts_mm, frame_id=frame_id, name=name, ns=name or side)


# ================== mehrere Seiten in einem MarkerArray ==================

def build_marker_array_from_recipe(
    recipe: Recipe,
    *,
    sides: Optional[Iterable[str]] = None,
    frame_id: str = "scene",   # Standard-Frame ist 'scene'
) -> MarkerArray:
    """
    Baut ein gemeinsames MarkerArray für alle angegebenen Seiten.

    STRICT:
      - wenn `sides` None: verwendet ausschließlich Keys aus recipe.paths_compiled["sides"]
      - keine Fallbacks auf recipe.paths oder recipe.paths_by_side
      - wenn nichts kompiliert ist -> leeres MarkerArray
    """
    ma = MarkerArray()

    # Seitenauswahl STRICT aus paths_compiled
    if sides is not None:
        side_list: List[str] = [str(s) for s in sides]
    else:
        pc = recipe.paths_compiled or {}
        sdata = pc.get("sides") or {}
        if not isinstance(sdata, dict) or not sdata:
            return ma
        side_list = list(sdata.keys())

    if not side_list:
        return ma

    global_id = 0
    for side in side_list:
        side_ma = build_marker_array_for_recipe_side(
            recipe,
            side=side,
            frame_id=frame_id,
        )
        if not side_ma.markers:
            continue

        # IDs neu vergeben, um Kollisionen zu vermeiden
        for m in side_ma.markers:
            m.id = global_id
            global_id += 1
            # Frame-Sicherheit
            if not m.header.frame_id:
                m.header.frame_id = frame_id
            ma.markers.append(m)

    return ma
