#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
cagebuilder.py — rechteckiger Käfig aus Rahmenstangen + optionalen geschlossenen Panels (keine Gitter).
- Maße in mm, Innenraum hohl.
- 12 Rahmenkanten immer vorhanden.
- Pro Seite (front, back, left, right, top, bottom) optional eine dünne Platte.
- Ursprung (0,0,0) unten mittig, Z nach oben.

Beispiel:
  python3 cagebuilder.py --width 500 --depth 500 --height 500 --bar 10 \
      --panel-thickness 3 --closed-sides front,right,left,back,top --out cage_with_panels.stl
"""

import argparse
import os
import numpy as np
import trimesh


def _beam(length: float, bar: float, axis: str, center):
    """Quaderförmige Stange entlang axis ∈ {'x','y','z'}."""
    if axis == 'x':
        ext = (length, bar,   bar)
    elif axis == 'y':
        ext = (bar,   length, bar)
    elif axis == 'z':
        ext = (bar,   bar,    length)
    else:
        raise ValueError("axis must be 'x','y' or 'z'")
    m = trimesh.creation.box(extents=ext)
    m.apply_translation(np.asarray(center, dtype=float))
    return m


def _panel(size_a: float, size_b: float, thick: float, center):
    """
    Dünne Platte als Box (Größe size_a × size_b × thick), um Kollisionen mit Rahmen zu vermeiden
    etwas kleiner als Innenlichte gesetzt.
    """
    m = trimesh.creation.box(extents=(size_a, size_b, thick))
    m.apply_translation(np.asarray(center, dtype=float))
    return m


def build_cage_with_panels(
    width: float,
    depth: float,
    height: float,
    bar: float = 10.0,
    panel_thickness: float = 3.0,
    closed_sides=None,
    panel_inset: float = 1.0,
):
    """
    Erzeugt Käfig mit Rahmen + optionalen geschlossenen Panels.
    - closed_sides: Iterable[str] der Seiten, die eine Platte erhalten.
      Gültig: {"front","back","left","right","top","bottom"}
    - panel_inset: Abstand der Panelkanten vom Innenrahmen (mm), um Überschneidung zu vermeiden.
    """
    closed_sides = set(closed_sides or [])
    valid = {"front", "back", "left", "right", "top", "bottom"}
    if not closed_sides.issubset(valid):
        raise ValueError(f"Ungültige Seiten: {closed_sides - valid}")

    W, D, H, T = map(float, (width, depth, height, bar))
    t = float(panel_thickness)
    inset = float(panel_inset)

    # Außenkanten
    x_min, x_max = -W / 2.0, W / 2.0
    y_min, y_max = -D / 2.0, D / 2.0
    z_min, z_max = 0.0, H

    parts = []

    # ---------------------------
    # 1) Rahmenkanten (12 Stäbe)
    # ---------------------------

    # Vertikale Ecken (4×)
    for sx in (x_min + T / 2.0, x_max - T / 2.0):
        for sy in (y_min + T / 2.0, y_max - T / 2.0):
            parts.append(_beam(H - T, T, 'z', (sx, sy, z_min + (H - T) / 2.0)))

    # Obere Rechteckkante (X-Richtung, 2×)
    for sy in (y_min + T / 2.0, y_max - T / 2.0):
        parts.append(_beam(W - T, T, 'x', (0.0, sy, z_max - T / 2.0)))
    # Obere Rechteckkante (Y-Richtung, 2×)
    for sx in (x_min + T / 2.0, x_max - T / 2.0):
        parts.append(_beam(D - T, T, 'y', (sx, 0.0, z_max - T / 2.0)))

    # Untere Rechteckkante (X-Richtung, 2×)
    for sy in (y_min + T / 2.0, y_max - T / 2.0):
        parts.append(_beam(W - T, T, 'x', (0.0, sy, z_min + T / 2.0)))
    # Untere Rechteckkante (Y-Richtung, 2×)
    for sx in (x_min + T / 2.0, x_max - T / 2.0):
        parts.append(_beam(D - T, T, 'y', (sx, 0.0, z_min + T / 2.0)))

    # -----------------------------------------
    # 2) Geschlossene Panels (wenn angefordert)
    # -----------------------------------------

    # Innenlichte
    inner_W = W - 2 * T - 2 * inset
    inner_D = D - 2 * T - 2 * inset
    inner_H = H - 2 * T - 2 * inset
    if inner_W <= 0 or inner_D <= 0 or inner_H <= 0:
        raise ValueError("Innenlichte <= 0 — vergrößere Abmessungen oder verringere bar/inset")

    z_center = z_min + T + inset + inner_H / 2.0

    # Front (Y+): Dicke entlang Y
    if "front" in closed_sides:
        y_plane = y_max - T - inset - t / 2.0
        parts.append(trimesh.creation.box(
            extents=(inner_W, t, inner_H)
        ).apply_translation((0.0, y_plane, z_center)))

    # Back (Y-): Dicke entlang Y
    if "back" in closed_sides:
        y_plane = y_min + T + inset + t / 2.0
        parts.append(trimesh.creation.box(
            extents=(inner_W, t, inner_H)
        ).apply_translation((0.0, y_plane, z_center)))

    # Left (X-): Dicke entlang X
    if "left" in closed_sides:
        x_plane = x_min + T + inset + t / 2.0
        parts.append(trimesh.creation.box(
            extents=(t, inner_D, inner_H)
        ).apply_translation((x_plane, 0.0, z_center)))

    # Right (X+): Dicke entlang X
    if "right" in closed_sides:
        x_plane = x_max - T - inset - t / 2.0
        parts.append(trimesh.creation.box(
            extents=(t, inner_D, inner_H)
        ).apply_translation((x_plane, 0.0, z_center)))

    # Top (Z+): Dicke entlang Z
    if "top" in closed_sides:
        z_plane = z_max - T - inset - t / 2.0
        parts.append(trimesh.creation.box(
            extents=(inner_W, inner_D, t)
        ).apply_translation((0.0, 0.0, z_plane)))

    # Bottom (Z-): Dicke entlang Z
    if "bottom" in closed_sides:
        z_plane = z_min + T + inset + t / 2.0
        parts.append(trimesh.creation.box(
            extents=(inner_W, inner_D, t)
        ).apply_translation((0.0, 0.0, z_plane)))


    cage = trimesh.util.concatenate(parts)
    cage.remove_unreferenced_vertices()
    return cage


def main():
    ap = argparse.ArgumentParser(description="Erzeuge Käfig (Rahmen + geschlossene Panels) als STL.")
    ap.add_argument("--width", type=float, default=500.0, help="Breite W (mm)")
    ap.add_argument("--depth", type=float, default=500.0, help="Tiefe D (mm)")
    ap.add_argument("--height", type=float, default=500.0, help="Höhe H (mm)")
    ap.add_argument("--bar", type=float, default=10.0, help="Stabdicke Rahmen (mm)")
    ap.add_argument("--panel-thickness", type=float, default=3.0, help="Plattendicke (mm)")
    ap.add_argument("--panel-inset", type=float, default=1.0, help="Innenversatz Panel zu Rahmen (mm)")
    ap.add_argument("--closed-sides", default="", help="Kommagetrennt: front,back,left,right,top,bottom")
    ap.add_argument("--out", default="cage_with_panels.stl", help="Zieldatei")
    args = ap.parse_args()

    closed_sides = set([s.strip() for s in args.closed_sides.split(",") if s.strip()])

    mesh = build_cage_with_panels(
        width=args.width,
        depth=args.depth,
        height=args.height,
        bar=args.bar,
        panel_thickness=args.panel_thickness,
        closed_sides=closed_sides,
        panel_inset=args.panel_inset,
    )
    os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
    mesh.export(args.out)
    print(f"✅ STL: {os.path.abspath(args.out)}")
    print(f"   Geschlossene Seiten: {', '.join(sorted(closed_sides)) or 'keine'}")


if __name__ == "__main__":
    main()
