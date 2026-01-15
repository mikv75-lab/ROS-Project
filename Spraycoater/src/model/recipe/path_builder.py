# -*- coding: utf-8 -*-
# File: src/model/recipe/path_builder.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Tuple

import math
import numpy as np


@dataclass
class PathData:
    """
    Pfad-Daten in Millimeter.

    - points_mm: Nx3 Punkte (x,y,z) in mm
    - meta: freie Metadaten (Quelle, Parameter, etc.) -> JSON-safe halten
    """
    points_mm: np.ndarray  # shape: (N, 3)
    meta: Dict[str, Any]

    def arclength_mm(self) -> np.ndarray:
        """Kumulierte Bogenlänge (mm) entlang points_mm."""
        P = np.asarray(self.points_mm, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2:
            return np.zeros((P.shape[0],), dtype=float)
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

    def as_list_mm(self) -> List[List[float]]:
        """Für Recipe/JSON: [[x,y,z], ...]"""
        P = np.asarray(self.points_mm, dtype=float).reshape(-1, 3)
        return P.tolist()


class PathBuilder:
    """
    Erzeugt Pfade (mm) aus einer Path-Definition je Side.

    Unterstützte `type`-Werte:
      - path.meander.plane (Z=0)
      - path.spiral.plane (Z=0)
      - path.spiral.cylinder (3D)
      - path.perimeter_follow.plane (Z=0)
      - path.polyhelix.pyramid (3D)
      - path.polyhelix.cube (3D)
    """

    def __new__(cls, *args: Any, **kwargs: Any):
        return super().__new__(cls)

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        _ = args, kwargs  # no-op

    # ---------------------------
    # Public API
    # ---------------------------

    @staticmethod
    def from_side(
        recipe: Any,
        *,
        side: str,
        globals_params: Dict[str, Any],
        sample_step_mm: float,
        max_points: int,
    ) -> PathData:
        # 1. Parameter aus dem Rezept (parameters:) priorisieren
        globals_params, sample_step_mm, max_points = PathBuilder._resolve_params(
            recipe, globals_params, sample_step_mm, max_points
        )

        PathBuilder._validate_inputs(globals_params, sample_step_mm, max_points)
        p = PathBuilder._extract_path_for_side(recipe, side)

        pd = PathBuilder._from_path_dict(
            p,
            sample_step_mm=float(sample_step_mm),
            max_points=int(max_points),
        )

        pd = PathBuilder._postprocess(pd)
        pd = PathBuilder._with_globals_meta(pd, globals_params)

        return pd

    @staticmethod
    def from_recipe_paths(
        recipe: Any,
        *,
        sides: Iterable[str],
        globals_params: Dict[str, Any],
        sample_step_mm: float,
        max_points: int,
    ) -> List[Tuple[str, PathData]]:
        globals_params, sample_step_mm, max_points = PathBuilder._resolve_params(
            recipe, globals_params, sample_step_mm, max_points
        )

        PathBuilder._validate_inputs(globals_params, sample_step_mm, max_points)
        
        out: List[Tuple[str, PathData]] = []
        for s in sides:
            try:
                pd = PathBuilder.from_side(
                    recipe,
                    side=s,
                    globals_params=globals_params,
                    sample_step_mm=sample_step_mm,
                    max_points=max_points,
                )
                out.append((s, pd))
            except (KeyError, ValueError):
                continue
        return out

    # ---------------------------
    # Validation & helpers
    # ---------------------------

    @staticmethod
    def _resolve_params(
        recipe: Any,
        g_params: Dict[str, Any],
        step: float,
        max_pts: int
    ) -> Tuple[Dict[str, Any], float, int]:
        eff_globals = dict(g_params)
        
        rec_params = {}
        if isinstance(recipe, dict):
            rec_params = recipe.get("parameters", {}) or {}
        else:
            rec_params = getattr(recipe, "parameters", {}) or {}
            
        if not isinstance(rec_params, dict):
            rec_params = {}

        if "sample_step_mm" in rec_params:
            try:
                val = float(rec_params["sample_step_mm"])
                if val > 0:
                    step = val
            except Exception: pass

        if "max_points" in rec_params:
            try:
                val = int(rec_params["max_points"])
                if val >= 2:
                    max_pts = val
            except Exception: pass

        for k in ("stand_off_mm", "max_angle_deg"):
            if k in rec_params:
                eff_globals[k] = rec_params[k]

        return eff_globals, step, max_pts

    @staticmethod
    def _validate_inputs(g: Dict[str, Any], sample_step_mm: float, max_points: int) -> None:
        if not isinstance(g, dict):
            raise ValueError("PathBuilder: globals_params muss ein Dict sein.")
        
        if not (isinstance(sample_step_mm, (int, float)) and float(sample_step_mm) > 0.0):
            raise ValueError("PathBuilder: sample_step_mm muss > 0 sein.")
        if not (isinstance(max_points, int) and max_points >= 2):
            raise ValueError("PathBuilder: max_points muss int >= 2 sein.")

    @staticmethod
    def _decimate(P: np.ndarray, max_points: int) -> np.ndarray:
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        n = P.shape[0]
        if n <= max_points:
            return P
        idx = np.linspace(0, n - 1, num=max_points, dtype=int)
        return P[idx]

    @staticmethod
    def _as_points_mm(pts: Any, max_points: int) -> np.ndarray:
        P = np.asarray(pts, dtype=float).reshape(-1, 3)
        if P.size == 0:
            return np.zeros((0, 3), dtype=float)
        mask = np.isfinite(P).all(axis=1)
        P = P[mask]
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _with_globals_meta(pd: PathData, g: Dict[str, Any]) -> PathData:
        meta = dict(pd.meta or {})
        if "stand_off_mm" in g:
            meta["stand_off_mm"] = float(g["stand_off_mm"])
        if "max_angle_deg" in g:
            meta["max_angle_deg"] = float(g["max_angle_deg"])
        meta = PathBuilder._json_safe_meta(meta)
        return PathData(points_mm=np.asarray(pd.points_mm, dtype=float).reshape(-1, 3), meta=meta)

    @staticmethod
    def _json_safe_meta(meta: Dict[str, Any]) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for k, v in (meta or {}).items():
            if isinstance(v, (np.floating, np.integer)):
                out[k] = v.item()
            elif isinstance(v, np.ndarray):
                out[k] = v.tolist()
            else:
                out[k] = v
        return out

    @staticmethod
    def _postprocess(pd: PathData) -> PathData:
        P = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        if P.size == 0:
            return PathData(points_mm=P, meta=dict(pd.meta or {}))
        mask = np.isfinite(P).all(axis=1)
        P = P[mask]
        return PathData(points_mm=P, meta=dict(pd.meta or {}))

    @staticmethod
    def _extract_path_for_side(recipe: Any, side: str) -> Dict[str, Any]:
        if recipe is None:
            raise TypeError("PathBuilder: recipe ist None.")
        if not isinstance(side, str) or not side.strip():
            raise ValueError("PathBuilder: side muss ein nicht-leerer String sein.")

        if isinstance(recipe, dict):
            pbs = dict(recipe.get("paths_by_side") or {})
        else:
            pbs = dict(getattr(recipe, "paths_by_side", {}) or {})

        if side not in pbs:
            raise KeyError(f"PathBuilder: side '{side}' nicht in paths_by_side.")

        p = dict(pbs[side] or {})
        if not p:
            raise ValueError(f"PathBuilder: leere Path-Definition für side '{side}'.")
        return p

    # ---------------------------
    # Path dispatch
    # ---------------------------

    @staticmethod
    def _from_path_dict(
        p: Dict[str, Any],
        *,
        sample_step_mm: float,
        max_points: int,
    ) -> PathData:
        pts = p.get("points_mm") or p.get("polyline_mm")
        if pts is not None:
            P = PathBuilder._as_points_mm(pts, max_points)
            return PathData(points_mm=P, meta={"source": "points"})

        raw_type = p.get("type")
        if raw_type is None or str(raw_type).strip() == "":
            raise ValueError("PathBuilder: path.type fehlt oder ist leer.")

        ptype = str(raw_type).strip().lower()

        if ptype == "path.meander.plane":
            P = PathBuilder._meander_plane(p, sample_step_mm, max_points)
        elif ptype == "path.spiral.plane":
            P = PathBuilder._spiral_plane(p, sample_step_mm, max_points)
        elif ptype == "path.spiral.cylinder":
            P = PathBuilder._spiral_cylinder_centerline(p, sample_step_mm, max_points)
        elif ptype == "path.perimeter_follow.plane":
            P = PathBuilder._perimeter_follow_plane(p, sample_step_mm, max_points)
        elif ptype == "path.polyhelix.pyramid":
            P = PathBuilder._polyhelix_pyramid(p, sample_step_mm, max_points)
        elif ptype == "path.polyhelix.cube":
            P = PathBuilder._polyhelix_cube(p, sample_step_mm, max_points)
        else:
            raise ValueError(f"PathBuilder: unbekannter path.type: {raw_type!r}")

        return PathData(points_mm=P, meta={"source": ptype, "params": p})

    # ============================================================
    # Generators
    # ============================================================

    @staticmethod
    def _meander_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        """Ebenen-Mäander (Z=0)."""
        area = p.get("area", {}) or {}
        shape = str(area.get("shape", "rect")).lower()
        cx, cy = (area.get("center_xy_mm") or [0.0, 0.0])
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        angle = float(p.get("angle_deg", 0.0))
        edge_extend = float(p.get("edge_extend_mm", 0.0))
        margin = max(0.0, float(p.get("margin_mm", 0.0)))
        start_corner = str(p.get("start", "auto")).lower()
        
        reverse_y = start_corner in ("minx_maxy", "maxx_maxy")
        reverse_x_base = start_corner in ("maxx_miny", "maxx_maxy")

        line_segments = []
        if shape in ("circle", "disk"):
            R = max(0.0, float(area.get("radius_mm", 50.0)) - margin)
            if R <= 0: return np.zeros((0,3), float)
            n_lines = max(1, int(2.0 * R / pitch))
            ys = np.linspace(-(n_lines-1)*pitch/2.0, (n_lines-1)*pitch/2.0, n_lines)
            if reverse_y: ys = ys[::-1]
            for y in ys:
                if abs(y) >= R: continue
                span = math.sqrt(max(R * R - y * y, 0.0))
                line_segments.append(((-span - edge_extend, y), (span + edge_extend, y)))
        else:
            sx, sy = area.get("size_mm", [100.0, 100.0])
            hx = max(0.0, 0.5 * float(sx) - margin)
            hy = max(0.0, 0.5 * float(sy) - margin)
            if hx <= 0 or hy <= 0: return np.zeros((0,3), float)
            n_lines = int(2.0 * hy / pitch) + 1
            ys = np.array([-((n_lines - 1) * pitch) / 2.0 + i*pitch for i in range(n_lines)])
            if reverse_y: ys = ys[::-1]
            for y in ys:
                line_segments.append(((-hx - edge_extend, y), (hx + edge_extend, y)))

        full_pts = []
        current_rev = reverse_x_base 
        for i, (p0, p1) in enumerate(line_segments):
            start, end = (p1, p0) if current_rev else (p0, p1)
            dist = math.hypot(end[0]-start[0], end[1]-start[1])
            n = max(2, int(dist / max(step, 1e-6)) + 1)
            full_pts.extend([(start[0]*(1-t) + end[0]*t, start[1]*(1-t) + end[1]*t) for t in np.linspace(0, 1, n)][int(i>0):])
            current_rev = not current_rev

        poly2d = np.array(full_pts, dtype=float)
        if abs(angle) > 1e-9 and poly2d.size > 0:
            th = math.radians(angle)
            R2 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
            poly2d = poly2d @ R2.T
        poly2d += np.array([cx, cy])
        
        # Z-Koordinate STRENG auf 0 setzen
        return PathBuilder._decimate(np.c_[poly2d, np.zeros(poly2d.shape[0])], max_points)

    @staticmethod
    def _spiral_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        """Ebene Spirale (Z=0)."""
        cx, cy = (p.get("center_xy_mm") or [0.0, 0.0])
        r_outer = float(p.get("r_outer_mm", p.get("r_end_mm", 70.0)))
        r_inner = float(p.get("r_inner_mm", p.get("r_start_mm", 10.0)))
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        direction = str(p.get("direction", "ccw")).lower()

        turns = max(int(abs(r_outer - r_inner) / pitch), 1)
        theta_max = 2.0 * math.pi * turns
        N = int(theta_max / max(step / max(r_outer, r_inner, 1e-6), 1e-6)) + 2
        theta = np.linspace(0.0, theta_max, N)
        r = r_outer - (r_outer - r_inner) * (theta / theta_max)
        if direction == "cw": theta = -theta

        x, y = cx + r * np.cos(theta), cy + r * np.sin(theta)
        
        # Z-Koordinate STRENG auf 0 setzen (ignoriert z_mm für die Generierung)
        return PathBuilder._decimate(np.c_[x, y, np.zeros_like(x)], max_points)

    @staticmethod
    def _spiral_cylinder_centerline(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        """Zylindrische Helix (3D bleibt erhalten)."""
        pitch, height = max(1e-6, float(p.get("pitch_mm", 10.0))), float(p.get("height_mm", 100.0))
        radius = float(p.get("radius_mm", 20.0)) + float(p.get("outside_mm", 1.0))
        usable_h = max(height - float(p.get("margin_top_mm", 0.0)) - float(p.get("margin_bottom_mm", 0.0)), 0.0)
        if usable_h <= 1e-9: return np.zeros((0, 3))

        turns = usable_h / pitch
        num_points = max(2, int((turns * math.sqrt((2*math.pi*radius)**2 + pitch**2)) / max(step, 1e-6)) + 1)
        t = np.linspace(0.0, 1.0, num_points)
        z = (height/2.0 - float(p.get("margin_top_mm", 0.0))) - t * usable_h if str(p.get("start_from", "top")).lower() == "top" else (-height/2.0 + float(p.get("margin_bottom_mm", 0.0))) + t * usable_h
        angles = (-1 if str(p.get("direction", "ccw")).lower() == "cw" else 1) * t * turns * 2.0 * math.pi
        return PathBuilder._decimate(np.c_[radius * np.cos(angles), radius * np.sin(angles), z], max_points)

    @staticmethod
    def _perimeter_follow_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        """Perimeter-Bahn (Z=0)."""
        area, loops = dict(p.get("area") or {}), max(int(p.get("loops", 1)), 1)
        shape, cx, cy = str(area.get("shape", "circle")).lower(), *(area.get("center_xy_mm") or [0.0, 0.0])
        off0, offstep, blend = float(p.get("offset_start_mm", 1.0)), float(p.get("offset_step_mm", 1.0)), max(0.0, float(p.get("corner_blend_mm", 0.0)))
        
        polylines = []
        for k in range(loops):
            off = off0 + k * offstep
            if shape in ("circle", "disk"):
                r = float(area.get("radius_mm", 50.0)) - off
                if r > 0: polylines.append(PathBuilder._polyline_circle(cx, cy, r, step))
            else:
                sx, sy = area.get("size_mm", [100.0, 100.0])
                hx, hy = 0.5 * float(sx) - off, 0.5 * float(sy) - off
                if hx > 0 and hy > 0: polylines.append(PathBuilder._polyline_rounded_rect(cx, cy, hx, hy, min(blend, hx, hy), step))
            if polylines and (k % 2) == 1: polylines[-1] = polylines[-1][::-1].copy()

        return PathBuilder._decimate(np.vstack(polylines) if polylines else np.zeros((0,3)), max_points)

    @staticmethod
    def _polyline_circle(cx: float, cy: float, r: float, step: float) -> np.ndarray:
        n = max(12, int((2.0 * math.pi * r) / max(step, 1e-6)) + 1)
        theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        return np.c_[cx + r * np.cos(theta), cy + r * np.sin(theta), np.zeros(n)]

    @staticmethod
    def _polyline_rounded_rect(cx: float, cy: float, hx: float, hy: float, r: float, step: float) -> np.ndarray:
        pts, r = [], max(0.0, min(r, hx, hy))
        def add_l(p0, p1): pts.append((1.0 - np.linspace(0,1,max(2,int(np.linalg.norm(p1-p0)/max(step,1e-6))+1),False))[:,None]*p0 + np.linspace(0,1,max(2,int(np.linalg.norm(p1-p0)/max(step,1e-6))+1),False)[:,None]*p1)
        def add_a(c, rr, a0, a1): 
            ang = np.linspace(a0, a1, max(2, int(0.5*math.pi*rr/max(step,1e-6))+1), False)
            pts.append(np.c_[c[0]+rr*np.cos(ang), c[1]+rr*np.sin(ang), np.zeros_like(ang)])
        
        c = [np.array([cx+hx-r, cy+hy-r]), np.array([cx-hx+r, cy+hy-r]), np.array([cx-hx+r, cy-hy+r]), np.array([cx+hx-r, cy-hy+r])]
        add_l(np.array([cx-hx+r, cy-hy]), np.array([cx+hx-r, cy-hy])); add_a(c[3], r, -0.5*math.pi, 0)
        add_l(np.array([cx+hx, cy-hy+r]), np.array([cx+hx, cy+hy-r])); add_a(c[0], r, 0, 0.5*math.pi)
        add_l(np.array([cx+hx-r, cy+hy]), np.array([cx-hx+r, cy+hy])); add_a(c[1], r, 0.5*math.pi, math.pi)
        add_l(np.array([cx-hx, cy+hy-r]), np.array([cx-hx, cy-hy+r])); add_a(c[2], r, math.pi, 1.5*math.pi)
        return np.vstack(pts)

    @staticmethod
    def _polyhelix_pyramid(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        return PathBuilder._polyhelix_generic(p, step, max_points, is_cube=False)

    @staticmethod
    def _polyhelix_cube(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        return PathBuilder._polyhelix_generic(p, step, max_points, is_cube=True)

    @staticmethod
    def _polyhelix_generic(p: Dict[str, Any], step: float, max_points: int, is_cube: bool) -> np.ndarray:
        """Helix-Strukturen (3D bleibt erhalten)."""
        edge = float(p.get("edge_len_mm" if is_cube else "base_edge_len_mm", 100.0))
        n_sides = 4 if is_cube else int(p.get("base_polygon_sides", 4))
        R_actual = (0 if is_cube else edge / (2.0 * math.sin(math.pi / n_sides))) + float(p.get("stand_off_mm", 25.0))
        
        if is_cube:
            base2d = PathBuilder._polyline_rounded_rect(0, 0, 0.5*edge+float(p.get("stand_off_mm",25)), 0.5*edge+float(p.get("stand_off_mm",25)), float(p.get("corner_roll_radius_mm", 8.0)), max(1.0, step))[:,:2]
        else:
            verts = [[R_actual * math.cos(math.radians(float(p.get("start_phase_deg", 0.0))) + 2*math.pi*i/n_sides), R_actual * math.sin(math.radians(float(p.get("start_phase_deg", 0.0))) + 2*math.pi*i/n_sides)] for i in range(n_sides)]
            base2d = np.vstack([ (1-t)[:,None]*np.array(verts[i]) + t[:,None]*np.array(verts[(i+1)%n_sides]) for i in range(n_sides) for t in [np.linspace(0,1,10,False)] ])

        L_base = np.linalg.norm(base2d[1:] - base2d[:-1], axis=1).sum()
        if L_base <= 1e-9: return np.zeros((0, 3))

        height, pitch, dz = float(p.get("height_mm", 100.0)), max(1e-6, float(p.get("pitch_mm", 6.0))), max(1e-6, float(p.get("dz_mm", 1.0)))
        pts, s_acc, z = [], 0.0, -0.5 * height
        while z <= (0.5 * height - (max(0.0, float(p.get("cap_top_mm", 8.0))) if not is_cube else 0.0)) + 1e-9:
            scale = max(0.0, (0.5*height - z) / height) if not is_cube else 1.0
            pt_xy = PathBuilder._point_on_polyline_by_arclength(np.c_[base2d, np.zeros(base2d.shape[0])], s_acc) * scale
            pts.append([pt_xy[0], pt_xy[1], z])
            s_acc += (L_base / pitch) * dz; z += dz
        return PathBuilder._decimate(np.array(pts), max_points)

    @staticmethod
    def _point_on_polyline_by_arclength(poly: np.ndarray, s: float) -> np.ndarray:
        P = poly[:, :2]; seglen = np.linalg.norm(P[1:] - P[:-1], axis=1); L = seglen.sum()
        if L <= 1e-12: return P[0].copy()
        s %= L; cum = np.concatenate([[0.0], np.cumsum(seglen)])
        i = max(0, min(np.searchsorted(cum, s, side="right") - 1, len(seglen) - 1))
        return P[i] if seglen[i] <= 1e-12 else (1.0 - (s-cum[i])/seglen[i]) * P[i] + ((s-cum[i])/seglen[i]) * P[i+1]