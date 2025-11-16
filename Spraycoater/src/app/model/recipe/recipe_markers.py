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
    line.scale.x = 0.002
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

    # Punkte
    dots = Marker()
    dots.header.frame_id = frame_id
    dots.ns = ns
    dots.id = 1
    dots.type = Marker.SPHERE_LIST
    dots.action = Marker.ADD
    dots.scale.x = 0.004
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

    # Text (Rezeptname)
    if name:
        txt = Marker()
        txt.header.frame_id = frame_id
        txt.ns = ns
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.text = name
        txt.scale.z = 0.02
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.pose.orientation.w = 1.0
        x0, y0, z0 = Pm[0]
        txt.pose.position.x = float(x0)
        txt.pose.position.y = float(y0)
        txt.pose.position.z = float(z0) + 0.01
        ma.markers.append(txt)

    return ma


def build_marker_array_for_recipe_side(
    recipe: Recipe,
    *,
    side: str,
    frame_id: str,
) -> MarkerArray:
    # Geometrie sicherstellen
    if side not in (recipe.paths or {}):
        recipe.rebuild_paths(sides=[side])

    paths = recipe.paths or {}
    pts_mm = paths.get(side)
    if pts_mm is None or pts_mm.size == 0:
        return MarkerArray()

    name = recipe.id or ""
    return _points_mm_to_markerarray(pts_mm, frame_id=frame_id, name=name, ns=name or side)


# ================== mehrere Seiten in einem MarkerArray ==================

def build_marker_array_from_recipe(
    recipe: Recipe,
    *,
    sides: Optional[Iterable[str]] = None,
    frame_id: str = "scene",   # 👈 Standard-Frame ist jetzt 'scene'
) -> MarkerArray:
    """
    Baut ein gemeinsames MarkerArray für alle angegebenen Seiten.

    - nutzt build_marker_array_for_recipe_side(...) je Seite
    - sorgt für eindeutige Marker-IDs innerhalb des Arrays
    - Frame wird über frame_id vorgegeben (Default: "scene")
    """
    ma = MarkerArray()

    # Wenn keine Seiten angegeben: alles nehmen, was da ist
    side_list: List[str] = []
    if sides is not None:
        side_list = [str(s) for s in sides]
    else:
        # Versuche zuerst recipe.paths (numerische Punkte)
        if isinstance(recipe.paths, dict) and recipe.paths:
            side_list = list(recipe.paths.keys())
        # Fallback: Seiten-Keys aus paths_by_side
        elif isinstance(getattr(recipe, "paths_by_side", None), dict) and recipe.paths_by_side:
            side_list = list(recipe.paths_by_side.keys())

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
