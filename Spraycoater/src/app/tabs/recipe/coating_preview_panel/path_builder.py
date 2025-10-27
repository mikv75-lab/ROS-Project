# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Tuple
import math
import numpy as np


@dataclass
class PathData:
    """Repräsentiert einen rohen (ungeoffseteten) Pfad in mm."""
    points_mm: np.ndarray          # (N, 3)
    meta: Dict[str, Any]

    def arclength_mm(self) -> np.ndarray:
        P = self.points_mm
        if len(P) < 2:
            return np.zeros((len(P),), dtype=float)
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])


class PathBuilder:
    """
    Bauen von **rohen Pfaden** (mm) aus einzelnen Path-Definitionen.
    Unterstützte path.type:
      - meander_plane
      - spiral_plane
      - spiral_cylinder (Helix)
      - direkt: points_mm / polyline_mm
    """

    # ---------------- Public API ----------------
    @staticmethod
    def from_side(
        recipe: Any,
        *,
        side: str,
        sample_step_mm: float = 1.0,
        max_points: int = 200_000,
    ) -> PathData:
        """
        Erzeugt PathData für GENAU EINE Side aus recipe.paths_by_side[side].
        """
        p = PathBuilder._extract_path_for_side(recipe, side)
        return PathBuilder._from_path_dict(
            p, sample_step_mm=sample_step_mm, max_points=max_points
        )

    @staticmethod
    def from_recipe_paths(
        recipe: Any,
        *,
        sides: Iterable[str],
        sample_step_mm: float = 1.0,
        max_points: int = 200_000,
    ) -> List[Tuple[str, PathData]]:
        """
        Baut mehrere Sides und gibt Liste [(side, PathData), ...] zurück.
        """
        out: List[Tuple[str, PathData]] = []
        for s in sides:
            pd = PathBuilder.from_side(
                recipe, side=s, sample_step_mm=sample_step_mm, max_points=max_points
            )
            out.append((s, pd))
        return out

    # ---------------- Internals ----------------
    @staticmethod
    def _extract_path_for_side(recipe: Any, side: str) -> Dict[str, Any]:
        if recipe is None:
            raise TypeError("PathBuilder: recipe ist None.")
        # Dict
        if isinstance(recipe, dict):
            pbs = dict(recipe.get("paths_by_side") or {})
        else:
            # Objekt mit Attribut
            pbs = dict(getattr(recipe, "paths_by_side", {}) or {})
        if not pbs:
            raise KeyError("PathBuilder: recipe.paths_by_side ist leer.")
        if side not in pbs:
            raise KeyError(f"PathBuilder: side '{side}' nicht in paths_by_side.")
        p = dict(pbs[side] or {})
        if not p:
            raise ValueError(f"PathBuilder: leere Path-Definition für side '{side}'.")
        return p

    @staticmethod
    def _from_path_dict(
        p: Dict[str, Any],
        *,
        sample_step_mm: float,
        max_points: int,
    ) -> PathData:
        # direkte Punkte?
        pts = p.get("points_mm") or p.get("polyline_mm")
        if pts is not None:
            P = np.asarray(pts, dtype=float).reshape(-1, 3)
            if len(P) > max_points:
                stride = max(1, len(P) // max_points)
                P = P[::stride]
            return PathData(points_mm=P, meta={"source": "points"})

        # generativ
        ptype = str(p.get("type", "")).strip().lower()
        if ptype in ("meander", "meander_plane"):
            P = PathBuilder._meander_plane(p, sample_step_mm, max_points)
            meta = {"source": "meander_plane"}
        elif ptype in ("spiral", "spiral_plane"):
            P = PathBuilder._spiral_plane(p, sample_step_mm, max_points)
            meta = {"source": "spiral_plane"}
        elif ptype in ("spiral_cylinder", "helix", "spiral_cyl"):
            P = PathBuilder._spiral_cylinder_centerline(p, sample_step_mm, max_points)
            meta = {"source": "spiral_cylinder"}
        else:
            raise ValueError(f"Unsupported path.type: {ptype!r}")
        return PathData(points_mm=P, meta=meta)

    @staticmethod
    def _meander_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        area = p.get("area", {}) or {}
        shape = str(area.get("shape", "rect")).lower()
        cx, cy = (area.get("center_xy_mm") or [0.0, 0.0])
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        angle = float(p.get("angle_deg", 0.0))
        boust = bool(p.get("boustrophedon", True))
        edge_extend = float(p.get("edge_extend_mm", 0.0))

        if shape in ("circle", "disk"):
            R = float(area.get("radius_mm", 50.0))
            ys = np.arange(-R, R + 1e-9, pitch)
            rows = []
            for i, y in enumerate(ys):
                span = math.sqrt(max(R * R - y * y, 0.0))
                x0, x1 = -span - edge_extend, span + edge_extend
                nseg = max(2, int((x1 - x0) / max(step, 1e-6)) + 1)
                xs = np.linspace(x0, x1, nseg)
                if boust and (i % 2 == 1):
                    xs = xs[::-1]
                rows.append(np.c_[xs, np.full_like(xs, y)])
            poly2d = np.vstack(rows) if rows else np.zeros((0, 2))
        else:
            sx, sy = area.get("size_mm", [100.0, 100.0])
            hx, hy = 0.5 * float(sx), 0.5 * float(sy)
            ys = np.arange(-hy, hy + 1e-9, pitch)
            rows = []
            for i, y in enumerate(ys):
                x0, x1 = -hx - edge_extend, hx + edge_extend
                nseg = max(2, int((x1 - x0) / max(step, 1e-6)) + 1)
                xs = np.linspace(x0, x1, nseg)
                if boust and (i % 2 == 1):
                    xs = xs[::-1]
                rows.append(np.c_[xs, np.full_like(xs, y)])
            poly2d = np.vstack(rows) if rows else np.zeros((0, 2))

        # Rotation
        if abs(angle) > 1e-9:
            th = math.radians(angle)
            R2 = np.array([[math.cos(th), -math.sin(th)],
                           [math.sin(th),  math.cos(th)]])
            poly2d = poly2d @ R2.T

        # Offset ins Zentrum
        poly2d += np.array([cx, cy], dtype=float)

        P = np.c_[poly2d, np.zeros((len(poly2d),), dtype=float)]
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P

    @staticmethod
    def _spiral_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        cx, cy = (p.get("center_xy_mm") or [0.0, 0.0])
        r_outer = float(p.get("r_outer_mm", p.get("r_end_mm", 70.0)))
        r_inner = float(p.get("r_inner_mm", p.get("r_start_mm", 10.0)))
        pitch   = max(1e-6, float(p.get("pitch_mm", 5.0)))

        turns = max(int((r_outer - r_inner) / pitch), 1)
        theta_max = 2.0 * math.pi * turns

        dtheta = step / max(r_outer, 1e-6)
        N = int(theta_max / max(dtheta, 1e-6)) + 2
        theta = np.linspace(0.0, theta_max, N)

        r = r_outer - (r_outer - r_inner) * (theta / theta_max)
        x = cx + r * np.cos(theta)
        y = cy + r * np.sin(theta)

        P = np.c_[x, y, np.zeros_like(x)]
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P

    @staticmethod
    def _spiral_cylinder_centerline(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        pitch   = max(1e-6, float(p.get("pitch_mm", 10.0)))
        m_top   = float(p.get("margin_top_mm", 0.0))
        m_bot   = float(p.get("margin_bottom_mm", 0.0))
        start_from = str(p.get("start_from", "top")).lower()
        direction  = str(p.get("direction",  "ccw")).lower()
        radius = float(p.get("radius_mm", 10.0)) + float(p.get("outside_mm", 1.0))
        height = float(p.get("height_mm", 100.0))

        usable_h = max(height - m_top - m_bot, 0.0)
        turns = max(int(usable_h / pitch), 1)
        theta_max = 2.0 * math.pi * turns
        sgn = -1.0 if direction == "cw" else 1.0

        denom = math.sqrt(radius * radius + (pitch / (2.0 * math.pi)) ** 2)
        dtheta = max(1e-4, step / max(denom, 1e-9))
        N = int(theta_max / dtheta) + 2
        theta = np.linspace(0.0, sgn * theta_max, N)

        if start_from == "top":
            z0 = height / 2.0 - m_top
            z = z0 - (usable_h * (np.abs(theta) / theta_max))
        else:
            z0 = -height / 2.0 + m_bot
            z = z0 + (usable_h * (np.abs(theta) / theta_max))

        x = radius * np.cos(theta)
        y = radius * np.sin(theta)

        P = np.c_[x, y, z]
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P
