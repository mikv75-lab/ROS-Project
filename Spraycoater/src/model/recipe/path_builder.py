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
    """
    points_mm: np.ndarray  # shape: (N, 3)
    meta: Dict[str, Any]

    def arclength_mm(self) -> np.ndarray:
        P = np.asarray(self.points_mm, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2:
            return np.zeros((P.shape[0],), dtype=float)
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

    def as_list_mm(self) -> List[List[float]]:
        P = np.asarray(self.points_mm, dtype=float).reshape(-1, 3)
        return P.tolist()


class PathBuilder:
    """
    Erzeugt Pfade (mm) aus einer Path-Definition je Side.
    Die Punkte werden im lokalen Koordinatensystem (meist Z=0) erzeugt.
    Die Platzierung (z.B. über dem Substrat) erfolgt im PreviewPanel.
    """

    def __new__(cls, *args: Any, **kwargs: Any):
        return super().__new__(cls)

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        _ = args, kwargs

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
        # Resolve globals
        globals_params, sample_step_mm, max_points = PathBuilder._resolve_params(
            recipe, globals_params, sample_step_mm, max_points
        )

        PathBuilder._validate_inputs(globals_params, sample_step_mm, max_points)
        p = PathBuilder._extract_path_for_side(recipe, side)

        # 1. Generate Base Path (Z=0 for planar paths)
        pd = PathBuilder._from_path_dict(
            p,
            sample_step_mm=float(sample_step_mm),
            max_points=int(max_points),
        )

        # 2. Air-move Extension
        pd = PathBuilder._add_airmove_endpoints(pd, p)

        # Meta (globals)
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
            except (KeyError, ValueError, TypeError):
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

        if not isinstance(rec_params, dict): rec_params = {}

        if "sample_step_mm" in rec_params:
            try:
                val = float(rec_params["sample_step_mm"])
                if val > 0: step = val
            except Exception: pass

        if "max_points" in rec_params:
            try:
                val = int(rec_params["max_points"])
                if val >= 2: max_pts = val
            except Exception: pass

        for k in ("stand_off_mm", "max_angle_deg"):
            if k in rec_params:
                eff_globals[k] = rec_params[k]

        return eff_globals, step, max_pts

    @staticmethod
    def _validate_inputs(g: Dict[str, Any], sample_step_mm: float, max_points: int) -> None:
        if not isinstance(g, dict):
            raise ValueError("PathBuilder: globals_params must be dict.")
        if not (isinstance(sample_step_mm, (int, float)) and float(sample_step_mm) > 0.0):
            raise ValueError("PathBuilder: sample_step_mm must be > 0.")
        if not (isinstance(max_points, int) and max_points >= 2):
            raise ValueError("PathBuilder: max_points must be int >= 2.")

    @staticmethod
    def _decimate(P: np.ndarray, max_points: int) -> np.ndarray:
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        n = P.shape[0]
        if n <= max_points: return P
        idx = np.linspace(0, n - 1, num=max_points, dtype=int)
        return P[idx]

    @staticmethod
    def _as_points_mm(pts: Any, max_points: int) -> np.ndarray:
        P = np.asarray(pts, dtype=float).reshape(-1, 3)
        if P.size == 0: return np.zeros((0, 3), dtype=float)
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
    def _extract_path_for_side(recipe: Any, side: str) -> Dict[str, Any]:
        if recipe is None: raise TypeError("PathBuilder: recipe is None.")
        if not isinstance(side, str) or not side.strip(): raise ValueError("PathBuilder: side invalid.")
        
        if isinstance(recipe, dict):
            pbs = dict(recipe.get("paths_by_side") or {})
        else:
            pbs = dict(getattr(recipe, "paths_by_side", {}) or {})
        
        if side not in pbs: raise KeyError(f"PathBuilder: side '{side}' not found.")
        p = dict(pbs[side] or {})
        if not p: raise ValueError(f"PathBuilder: path definition empty for '{side}'.")
        return p

    @staticmethod
    def _add_airmove_endpoints(pd: PathData, p: Dict[str, Any]) -> PathData:
        P = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2: return pd

        def _clamp_nonneg(x: Any) -> float:
            try: return max(0.0, float(x))
            except Exception: return 0.0

        pre_mm = _clamp_nonneg(p.get("predispense_offset_mm", 0.0))
        post_mm = _clamp_nonneg(p.get("retreat_offset_mm", 0.0))
        if pre_mm <= 0.0 and post_mm <= 0.0: return pd

        out = P
        if pre_mm > 0.0:
            v0 = out[0] - out[1]
            n0 = float(np.linalg.norm(v0))
            if n0 > 1e-9:
                out = np.vstack([(out[0] + (v0/n0)*pre_mm)[None, :], out])
        if post_mm > 0.0:
            v1 = out[-1] - out[-2]
            n1 = float(np.linalg.norm(v1))
            if n1 > 1e-9:
                out = np.vstack([out, (out[-1] + (v1/n1)*post_mm)[None, :]])
        
        meta = dict(pd.meta or {})
        if pre_mm > 0.0: meta["predispense_offset_mm"] = float(pre_mm)
        if post_mm > 0.0: meta["retreat_offset_mm"] = float(post_mm)
        return PathData(points_mm=out, meta=meta)

    @staticmethod
    def _from_path_dict(p: Dict[str, Any], *, sample_step_mm: float, max_points: int) -> PathData:
        pts = p.get("points_mm") or p.get("polyline_mm")
        if pts is not None:
            P = PathBuilder._as_points_mm(pts, max_points)
            return PathData(points_mm=P, meta={"source": "points", "params": dict(p or {})})

        raw_type = str(p.get("type") or "").strip().lower()
        if not raw_type: raise ValueError("PathBuilder: path.type missing.")

        if raw_type == "path.meander.plane":
            P = PathBuilder._meander_plane(p, sample_step_mm, max_points)
        elif raw_type == "path.spiral.plane":
            P = PathBuilder._spiral_plane(p, sample_step_mm, max_points)
        elif raw_type == "path.spiral.cylinder":
            P = PathBuilder._spiral_cylinder_centerline(p, sample_step_mm, max_points)
        elif raw_type == "path.perimeter_follow.plane":
            P = PathBuilder._perimeter_follow_plane(p, sample_step_mm, max_points)
        elif raw_type == "path.polyhelix.pyramid":
            P = PathBuilder._polyhelix_pyramid(p, sample_step_mm, max_points)
        elif raw_type == "path.polyhelix.cube":
            P = PathBuilder._polyhelix_cube(p, sample_step_mm, max_points)
        else:
            raise ValueError(f"PathBuilder: unknown type {raw_type!r}")

        P = np.asarray(P, dtype=float).reshape(-1, 3)
        if P.size:
            P = P[np.isfinite(P).all(axis=1)]
        return PathData(points_mm=PathBuilder._decimate(P, max_points), meta={"source": raw_type, "params": dict(p)})

    # --- Generators (Shortened for brevity, logic unchanged from previous working versions) ---
    # These generate geometries at Z=0 (for plane) or around Z=0 (for 3D)
    
    @staticmethod
    def _meander_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
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
            if R <= 0: return np.zeros((0, 3), float)
            n_lines = max(1, int(2.0 * R / pitch))
            ys = np.linspace(-(n_lines - 1) * pitch / 2.0, (n_lines - 1) * pitch / 2.0, n_lines)
            if reverse_y: ys = ys[::-1]
            for y in ys:
                if abs(y) >= R: continue
                span = math.sqrt(max(R * R - y * y, 0.0))
                line_segments.append(((-span - edge_extend, y), (span + edge_extend, y)))
        else:
            sx, sy = area.get("size_mm", [100.0, 100.0])
            hx = max(0.0, 0.5 * float(sx) - margin)
            hy = max(0.0, 0.5 * float(sy) - margin)
            if hx <= 0 or hy <= 0: return np.zeros((0, 3), float)
            n_lines = int(2.0 * hy / pitch) + 1
            ys = np.array([-(n_lines - 1) * pitch / 2.0 + i * pitch for i in range(n_lines)], dtype=float)
            if reverse_y: ys = ys[::-1]
            for y in ys:
                line_segments.append(((-hx - edge_extend, y), (hx + edge_extend, y)))

        full_pts = []
        current_rev = reverse_x_base
        for i, (p0, p1) in enumerate(line_segments):
            start, end = (p1, p0) if current_rev else (p0, p1)
            dist = math.hypot(end[0] - start[0], end[1] - start[1])
            n = max(2, int(dist / max(step, 1e-6)) + 1)
            pts_line = [(start[0] * (1 - t) + end[0] * t, start[1] * (1 - t) + end[1] * t) for t in np.linspace(0, 1, n)]
            if i > 0 and pts_line: pts_line = pts_line[1:]
            full_pts.extend(pts_line)
            current_rev = not current_rev

        poly2d = np.array(full_pts, dtype=float)
        if poly2d.size == 0: return np.zeros((0, 3), float)
        if abs(angle) > 1e-9:
            th = math.radians(angle)
            R2 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]], dtype=float)
            poly2d = poly2d @ R2.T
        poly2d += np.array([cx, cy], dtype=float)
        return PathBuilder._decimate(np.c_[poly2d, np.zeros((poly2d.shape[0],), dtype=float)], max_points)

    @staticmethod
    def _spiral_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        cx, cy = (p.get("center_xy_mm") or [0.0, 0.0])
        r_outer = float(p.get("r_outer_mm", p.get("r_end_mm", 70.0)))
        r_inner = float(p.get("r_inner_mm", p.get("r_start_mm", 10.0)))
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        direction = str(p.get("direction", "ccw")).lower()
        radial_offset = float(p.get("radial_offset_mm", 0.0))
        r_outer = max(0.0, r_outer + radial_offset)
        r_inner = max(0.0, r_inner + radial_offset)
        turns = max(int(abs(r_outer - r_inner) / pitch), 1)
        theta_max = 2.0 * math.pi * turns
        r_ref = max(r_outer, r_inner, 1e-6)
        dtheta = max(step / r_ref, 1e-6)
        N = int(theta_max / dtheta) + 2
        theta = np.linspace(0.0, theta_max, N)
        r = r_outer - (r_outer - r_inner) * (theta / theta_max if theta_max > 1e-12 else 0.0)
        if direction == "cw": theta = -theta
        x = float(cx) + r * np.cos(theta)
        y = float(cy) + r * np.sin(theta)
        return PathBuilder._decimate(np.c_[x, y, np.zeros_like(x)], max_points)

    @staticmethod
    def _spiral_cylinder_centerline(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        pitch = max(1e-6, float(p.get("pitch_mm", 10.0)))
        height = float(p.get("height_mm", 100.0))
        radius = float(p.get("radius_mm", 20.0)) + float(p.get("outside_mm", 0.0))
        top_extend = max(0.0, float(p.get("top_extend_mm", 0.0)))
        bottom_extend = max(0.0, float(p.get("bottom_extend_mm", 0.0)))
        z_top = 0.5 * height + top_extend
        z_bot = -0.5 * height - bottom_extend
        usable_h = max(z_top - z_bot, 0.0)
        if usable_h <= 1e-9: return np.zeros((0, 3), float)
        turns = usable_h / pitch
        L_turn = math.sqrt((2.0 * math.pi * radius) ** 2 + pitch ** 2)
        total_L = turns * L_turn
        num_points = max(2, int(total_L / max(step, 1e-6)) + 1)
        t = np.linspace(0.0, 1.0, num_points)
        start_from = str(p.get("start_from", "top")).lower()
        z = (z_top - t * usable_h) if start_from == "top" else (z_bot + t * usable_h)
        direction = str(p.get("direction", "ccw")).lower()
        sign = -1.0 if direction == "cw" else 1.0
        angles = sign * t * turns * 2.0 * math.pi
        P = np.c_[radius * np.cos(angles), radius * np.sin(angles), z]
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _perimeter_follow_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        area = dict(p.get("area") or {})
        loops = max(int(p.get("loops", 1)), 1)
        shape = str(area.get("shape", "circle")).lower()
        cx, cy = (area.get("center_xy_mm") or [0.0, 0.0])
        off0 = float(p.get("offset_start_mm", 1.0))
        offstep = float(p.get("offset_step_mm", 1.0))
        blend = max(0.0, float(p.get("corner_blend_mm", 0.0)))
        polylines = []
        for k in range(loops):
            off = off0 + k * offstep
            if shape in ("circle", "disk"):
                r = float(area.get("radius_mm", 50.0)) - off
                if r > 0: polylines.append(PathBuilder._polyline_circle(float(cx), float(cy), r, step))
            else:
                sx, sy = area.get("size_mm", [100.0, 100.0])
                hx = 0.5 * float(sx) - off
                hy = 0.5 * float(sy) - off
                if hx > 0 and hy > 0:
                    polylines.append(PathBuilder._polyline_rounded_rect(float(cx), float(cy), hx, hy, min(blend, hx, hy), step))
            if polylines and (k % 2) == 1: polylines[-1] = polylines[-1][::-1].copy()
        P = np.vstack(polylines) if polylines else np.zeros((0, 3), float)
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _polyline_circle(cx: float, cy: float, r: float, step: float) -> np.ndarray:
        n = max(12, int((2.0 * math.pi * r) / max(step, 1e-6)) + 1)
        theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        return np.c_[cx + r * np.cos(theta), cy + r * np.sin(theta), np.zeros((n,), dtype=float)]

    @staticmethod
    def _polyline_rounded_rect(cx: float, cy: float, hx: float, hy: float, r: float, step: float) -> np.ndarray:
        # (same as before implementation)
        pts = []
        r = max(0.0, min(float(r), float(hx), float(hy)))
        def add_l(p0, p1):
            dist = float(np.linalg.norm(p1 - p0))
            n = max(2, int(dist / max(step, 1e-6)) + 1)
            t = np.linspace(0, 1, n, endpoint=False)
            seg = (1.0 - t)[:, None] * p0[None, :] + t[:, None] * p1[None, :]
            pts.append(np.c_[seg, np.zeros((seg.shape[0],), dtype=float)])
        def add_a(c, rr, a0, a1):
            arc_len = abs(a1 - a0) * rr
            n = max(2, int(arc_len / max(step, 1e-6)) + 1)
            ang = np.linspace(a0, a1, n, endpoint=False)
            seg = np.c_[c[0] + rr * np.cos(ang), c[1] + rr * np.sin(ang), np.zeros_like(ang)]
            pts.append(seg)
        # corners
        c0 = np.array([cx+hx-r, cy+hy-r], float); c1 = np.array([cx-hx+r, cy+hy-r], float)
        c2 = np.array([cx-hx+r, cy-hy+r], float); c3 = np.array([cx+hx-r, cy-hy+r], float)
        add_l(np.array([cx-hx+r, cy-hy]), np.array([cx+hx-r, cy-hy]))
        add_a(c3, r, -0.5*math.pi, 0.0)
        add_l(np.array([cx+hx, cy-hy+r]), np.array([cx+hx, cy+hy-r]))
        add_a(c0, r, 0.0, 0.5*math.pi)
        add_l(np.array([cx+hx-r, cy+hy]), np.array([cx-hx+r, cy+hy]))
        add_a(c1, r, 0.5*math.pi, math.pi)
        add_l(np.array([cx-hx, cy+hy-r]), np.array([cx-hx, cy-hy+r]))
        add_a(c2, r, math.pi, 1.5*math.pi)
        return np.vstack(pts) if pts else np.zeros((0,3), float)

    @staticmethod
    def _polyhelix_pyramid(p, step, max_points): return PathBuilder._polyhelix_generic(p, step, max_points, False)
    @staticmethod
    def _polyhelix_cube(p, step, max_points): return PathBuilder._polyhelix_generic(p, step, max_points, True)
    
    @staticmethod
    def _polyhelix_generic(p: Dict[str, Any], step: float, max_points: int, is_cube: bool) -> np.ndarray:
        edge = float(p.get("edge_len_mm" if is_cube else "base_edge_len_mm", 100.0))
        n_sides = 4 if is_cube else int(p.get("base_polygon_sides", 4))
        stand_off = float(p.get("stand_off_mm", 25.0))

        if is_cube:
            base2d = PathBuilder._polyline_rounded_rect(0.0, 0.0, 0.5*edge+stand_off, 0.5*edge+stand_off, float(p.get("corner_roll_radius_mm", 8.0)), max(1.0, step))[:, :2]
        else:
            phase = math.radians(float(p.get("start_phase_deg", 0.0)))
            R_poly = edge / (2.0 * math.sin(math.pi / max(n_sides, 3)))
            R_actual = R_poly + stand_off
            verts = np.array([[R_actual*math.cos(phase + 2*math.pi*i/n_sides), R_actual*math.sin(phase + 2*math.pi*i/n_sides)] for i in range(n_sides)], float)
            chunks = []
            for i in range(n_sides):
                a, b = verts[i], verts[(i+1)%n_sides]
                dist = float(np.linalg.norm(b-a))
                n = max(2, int(dist/max(step, 1e-6)) + 1)
                t = np.linspace(0, 1, n, endpoint=False)
                chunks.append((1.0-t)[:,None]*a[None,:] + t[:,None]*b[None,:])
            base2d = np.vstack(chunks) if chunks else np.zeros((0,2), float)

        if base2d.shape[0] < 2: return np.zeros((0,3), float)
        seg = base2d[1:] - base2d[:-1]
        L_base = float(np.linalg.norm(seg, axis=1).sum())
        if L_base <= 1e-9: return np.zeros((0,3), float)

        height = float(p.get("height_mm", 100.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz = max(1e-6, float(p.get("dz_mm", 1.0)))
        cap_top = max(0.0, float(p.get("cap_top_mm", 0.0))) if not is_cube else 0.0
        z_min, z_max = -0.5*height, 0.5*height - cap_top

        pts = []
        s_acc = 0.0
        z = z_min
        while z <= z_max + 1e-9:
            scale = 1.0
            if not is_cube: scale = max(0.0, (0.5*height - z)/max(height, 1e-9))
            pt_xy = PathBuilder._point_on_polyline_by_arclength(base2d, s_acc) * scale
            pts.append([float(pt_xy[0]), float(pt_xy[1]), float(z)])
            s_acc += (L_base/pitch) * dz
            z += dz

        P = np.asarray(pts, dtype=float).reshape(-1, 3)
        start_off = float(p.get("start_offset_mm", 0.0))
        end_off = float(p.get("end_offset_mm", 0.0))
        if abs(start_off) > 1e-9 or abs(end_off) > 1e-9:
            P = PathBuilder._trim_by_arclength(P, start_off, end_off)
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _point_on_polyline_by_arclength(poly2d: np.ndarray, s: float) -> np.ndarray:
        P = np.asarray(poly2d, dtype=float).reshape(-1, 2)
        seg = P[1:] - P[:-1]
        seglen = np.linalg.norm(seg, axis=1)
        L = float(seglen.sum())
        if L <= 1e-12: return P[0].copy()
        s = float(s) % L
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        i = int(np.searchsorted(cum, s, side="right") - 1)
        i = max(0, min(i, len(seglen) - 1))
        t = (s - float(cum[i])) / float(seglen[i]) if seglen[i] > 1e-12 else 0.0
        return (1.0 - t) * P[i] + t * P[i + 1]

    @staticmethod
    def _trim_by_arclength(P: np.ndarray, start_off_mm: float, end_off_mm: float) -> np.ndarray:
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2: return P
        start = max(0.0, float(start_off_mm))
        end = max(0.0, float(end_off_mm))
        if start <= 0.0 and end <= 0.0: return P
        seg = P[1:] - P[:-1]
        seglen = np.linalg.norm(seg, axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        L = float(cum[-1])
        if L <= 1e-9: return P
        s0 = min(start, L)
        s1 = max(0.0, L - end)
        if s1 <= s0 + 1e-9: return np.zeros((0, 3), float)

        def interp_at(s):
            i = int(np.searchsorted(cum, s, side="right") - 1)
            i = max(0, min(i, len(seglen) - 1))
            t = (s - float(cum[i])) / float(seglen[i]) if seglen[i] > 1e-12 else 0.0
            return (1.0 - t) * P[i] + t * P[i + 1]

        pts = [interp_at(s0)]
        inside = np.where((cum[1:-1] > s0) & (cum[1:-1] < s1))[0] + 1
        for idx in inside.tolist(): pts.append(P[int(idx)].copy())
        pts.append(interp_at(s1))
        return np.vstack(pts)