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
      - perimeter_follow_plane
      - polyhelix_pyramid
      - polyhelix_cube
      - direkt: points_mm / polyline_mm

    WICHTIG:
    - KEINE Fallbacks auf Rezept-Defaults/Schemas/etc.
    - `globals_params` MUSS die folgenden Keys enthalten:
        stand_off_mm
        max_angle_deg
    """

    # --------- Public API (ohne Fallbacks) ---------
    @staticmethod
    def from_side(
        recipe: Any,
        *,
        side: str,
        globals_params: Dict[str, Any],
        sample_step_mm: float,
        max_points: int,
    ) -> PathData:
        """
        Erzeugt PathData für GENAU EINE Side aus recipe.paths_by_side[side].
        `globals_params` ist Pflicht (siehe Klassendocstring).
        """
        PathBuilder._validate_globals(globals_params)
        p = PathBuilder._extract_path_for_side(recipe, side)
        pd = PathBuilder._from_path_dict(
            p, sample_step_mm=sample_step_mm, max_points=max_points
        )
        # Pre/Retreat nicht mehr geometrisch anhängen (keine Distanzen in Globals).
        pd = PathBuilder._apply_predispense_retreat(pd, globals_params)
        PathBuilder._inject_globals_meta(pd, globals_params)
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
        """Baut mehrere Sides und gibt Liste [(side, PathData), ...] zurück."""
        PathBuilder._validate_globals(globals_params)
        out: List[Tuple[str, PathData]] = []
        for s in sides:
            pd = PathBuilder.from_side(
                recipe,
                side=s,
                globals_params=globals_params,
                sample_step_mm=sample_step_mm,
                max_points=max_points,
            )
            out.append((s, pd))
        return out

    # ---------------- Internals ----------------
    @staticmethod
    def _validate_globals(g: Dict[str, Any]) -> None:
        required = [
            "stand_off_mm",
            "max_angle_deg",
        ]
        if not isinstance(g, dict):
            raise ValueError("PathBuilder: globals_params muss ein Dict sein (kein Fallback).")
        missing = [k for k in required if k not in g]
        if missing:
            raise ValueError(f"PathBuilder: globals_params fehlen Keys: {missing} (kein Fallback).")

    @staticmethod
    def _extract_path_for_side(recipe: Any, side: str) -> Dict[str, Any]:
        if recipe is None:
            raise TypeError("PathBuilder: recipe ist None.")
        if isinstance(recipe, dict):
            pbs = dict(recipe.get("paths_by_side") or {})
        else:
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
        pts = p.get("points_mm") or p.get("polyline_mm")
        if pts is not None:
            P = np.asarray(pts, dtype=float).reshape(-1, 3)
            if len(P) > max_points:
                stride = max(1, len(P) // max_points)
                P = P[::stride]
            return PathData(points_mm=P, meta={"source": "points"})

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

        elif ptype in ("perimeter_follow_plane", "perimeter_follow", "perimeter_meander", "perimeter"):
            P = PathBuilder._perimeter_follow_plane(p, sample_step_mm, max_points)
            meta = {"source": "perimeter_follow_plane"}

        elif ptype in ("polyhelix_pyramid",):
            P = PathBuilder._polyhelix_pyramid(p, sample_step_mm, max_points)
            meta = {"source": "polyhelix_pyramid"}

        elif ptype in ("polyhelix_cube",):
            P = PathBuilder._polyhelix_cube(p, sample_step_mm, max_points)
            meta = {"source": "polyhelix_cube"}

        else:
            raise ValueError(f"Unsupported path.type: {ptype!r}")

        return PathData(points_mm=P, meta=meta)

    # --------------- Post-Prozess: (kein) Approach/Retreat ---------------
    @staticmethod
    def _apply_predispense_retreat(pd: PathData, g: Dict[str, Any]) -> PathData:
        """
        Kein Pre/Retreat-Anhängsel mehr – die UI-Hints (angle_mode/angle_deg)
        verbleiben in den Path-Parametern und werden nicht in die Geometrie
        übernommen.
        """
        P = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        meta = dict(pd.meta)
        return PathData(points_mm=P, meta=meta)

    @staticmethod
    def _inject_globals_meta(pd: PathData, g: Dict[str, Any]) -> None:
        pd.meta["stand_off_mm"] = float(g["stand_off_mm"])
        pd.meta["max_angle_deg"] = float(g["max_angle_deg"])

    # ------------------- Plane: Meander -------------------
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

        if abs(angle) > 1e-9:
            th = math.radians(angle)
            R2 = np.array([[math.cos(th), -math.sin(th)],
                           [math.sin(th),  math.cos(th)]])
            poly2d = poly2d @ R2.T

        poly2d += np.array([cx, cy], dtype=float)

        P = np.c_[poly2d, np.zeros((len(poly2d),), dtype=float)]
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P

    # ------------------- Plane: Spiral -------------------
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

    # ------------------- Cylinder: Helix -------------------
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

    # ------------------- Plane: Perimeter Follow -------------------
    @staticmethod
    def _perimeter_follow_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        area     = dict(p.get("area") or {})
        shape    = str(area.get("shape", "circle")).lower()
        cx, cy   = (area.get("center_xy_mm") or [0.0, 0.0])
        loops    = max(int(p.get("loops", 1)), 1)
        off0     = float(p.get("offset_start_mm", 1.0))
        offstep  = float(p.get("offset_step_mm", 1.0))
        blend    = max(0.0, float(p.get("corner_blend_mm", 0.0)))
        lead_in  = float(p.get("lead_in_mm", 0.0))

        polylines: list[np.ndarray] = []

        for k in range(loops):
            off = off0 + k * offstep
            if shape in ("circle", "disk"):
                R = float(area.get("radius_mm", 50.0))
                r = R - off
                if r <= 0.0:
                    continue
                poly = PathBuilder._polyline_circle(cx, cy, r, step)
            else:
                sx, sy = area.get("size_mm", [100.0, 100.0])
                hx = 0.5 * float(sx) - off
                hy = 0.5 * float(sy) - off
                if hx <= 0.0 or hy <= 0.0:
                    continue
                poly = PathBuilder._polyline_rounded_rect(
                    cx=cx, cy=cy, hx=hx, hy=hy, r=max(0.0, min(blend, hx, hy)), step=step
                )

            if poly.shape[0] == 0:
                continue

            if (k % 2) == 1:
                poly = poly[::-1].copy()

            if lead_in > 1e-9 and poly.shape[0] >= 2:
                dir_vec = (poly[1] - poly[0])
                n = np.linalg.norm(dir_vec)
                if n > 1e-9:
                    d = (dir_vec / n) * (-lead_in)
                    lead_pt = poly[0] + d
                    poly = np.vstack([lead_pt, poly])

            polylines.append(poly)

        if not polylines:
            return np.zeros((0, 3), dtype=float)

        P = np.vstack(polylines)
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P

    @staticmethod
    def _polyline_circle(cx: float, cy: float, r: float, step: float) -> np.ndarray:
        r = float(r)
        if r <= 0.0:
            return np.zeros((0, 3), dtype=float)
        n = max(12, int((2.0 * math.pi * r) / max(step, 1e-6)) + 1)
        theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        x = cx + r * np.cos(theta)
        y = cy + r * np.sin(theta)
        return np.c_[x, y, np.zeros_like(x)]

    @staticmethod
    def _polyline_rounded_rect(cx: float, cy: float, hx: float, hy: float, r: float, step: float) -> np.ndarray:
        hx = float(hx); hy = float(hy)
        r = max(0.0, min(float(r), hx, hy))
        straight_x = max(0.0, 2.0 * (hx - r))
        straight_y = max(0.0, 2.0 * (hy - r))
        perimeter = 2.0 * (straight_x + straight_y) + 2.0 * math.pi * r

        n_total = max(8, int(perimeter / max(step, 1e-6)) + 1)

        pts = []

        def append_line(p0, p1, nseg):
            if nseg <= 1:
                pts.append(p0)
                return
            t = np.linspace(0.0, 1.0, nseg, endpoint=False)
            seg = (1.0 - t)[:, None] * p0 + t[:, None] * p1
            pts.append(seg)

        def append_arc(center, r, a0, a1, nseg):
            if r <= 0.0 or nseg <= 1:
                return
            ang = np.linspace(a0, a1, nseg, endpoint=False)
            x = center[0] + r * np.cos(ang)
            y = center[1] + r * np.sin(ang)
            pts.append(np.c_[x, y, np.zeros_like(x)])

        def n_for_len(L):
            return max(2, int(L / max(step, 1e-6)) + 1)

        z0 = 0.0
        c_tr = np.array([cx + hx - r, cy + hy - r, z0])  # top-right
        c_tl = np.array([cx - hx + r, cy + hy - r, z0])  # top-left
        c_bl = np.array([cx - hx + r, cy - hy + r, z0])  # bottom-left
        c_br = np.array([cx + hx - r, cy - hy + r, z0])  # bottom-right

        p_bl = np.array([cx - hx + r, cy - hy, z0])
        p_br = np.array([cx + hx - r, cy - hy, z0])
        append_line(p_bl, p_br, n_for_len(2.0 * (hx - r)))
        append_arc(c_br, r, -0.5 * math.pi, 0.0, n_for_len(0.5 * math.pi * r))

        p_r_bot = np.array([cx + hx, cy - hy + r, z0])
        p_r_top = np.array([cx + hx, cy + hy - r, z0])
        append_line(p_r_bot, p_r_top, n_for_len(2.0 * (hy - r)))
        append_arc(c_tr, r, 0.0, 0.5 * math.pi, n_for_len(0.5 * math.pi * r))

        p_tr = np.array([cx + hx - r, cy + hy, z0])
        p_tl = np.array([cx - hx + r, cy + hy, z0])
        append_line(p_tr, p_tl, n_for_len(2.0 * (hx - r)))
        append_arc(c_tl, r, 0.5 * math.pi, math.pi, n_for_len(0.5 * math.pi * r))

        p_l_top = np.array([cx - hx, cy + hy - r, z0])
        p_l_bot = np.array([cx - hx, cy - hy + r, z0])
        append_line(p_l_top, p_l_bot, n_for_len(2.0 * (hy - r)))
        append_arc(c_bl, r, math.pi, 1.5 * math.pi, n_for_len(0.5 * math.pi * r))

        arrs = []
        for a in pts:
            if isinstance(a, np.ndarray):
                arrs.append(a)
            else:
                arrs.append(np.asarray(a, dtype=float).reshape(-1, 3))
        if not arrs:
            return np.zeros((0, 3), dtype=float)
        poly = np.vstack(arrs)

        if poly.shape[0] >= 2:
            mask = np.ones((poly.shape[0],), dtype=bool)
            mask[1:] = np.linalg.norm(poly[1:] - poly[:-1], axis=1) > 1e-9
            poly = poly[mask]
        return poly

    # ------------------- Polyhelix Helper -------------------
    @staticmethod
    def _point_on_polyline_by_arclength(poly2d: np.ndarray, s: float) -> np.ndarray:
        if poly2d.shape[0] < 2:
            return np.array([0.0, 0.0])
        P = poly2d[:, :2]
        seg = P[1:] - P[:-1]
        seglen = np.linalg.norm(seg, axis=1)
        L = float(seglen.sum())
        if L <= 1e-12:
            return P[0]
        s = float(s) % L
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        i = np.searchsorted(cum, s, side="right") - 1
        i = max(0, min(i, len(seglen) - 1))
        ds = s - cum[i]
        if seglen[i] <= 1e-12:
            return P[i+1]
        t = ds / seglen[i]
        pt = (1.0 - t) * P[i] + t * P[i+1]
        return pt

    # ------------------- Polyhelix: Pyramid -------------------
    @staticmethod
    def _polyhelix_pyramid(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        n = int(p.get("base_polygon_sides", 4))
        n = min(128, max(3, n))
        edge_len = float(p.get("base_edge_len_mm", 100.0))
        height   = float(p.get("height_mm", 60.0))
        pitch    = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz       = max(1e-6, float(p.get("dz_mm", 1.0)))
        phase    = math.radians(float(p.get("start_phase_deg", 0.0)))
        stand_off= float(p.get("stand_off_mm", 25.0))
        cap_top  = max(0.0, float(p.get("cap_top_mm", 8.0)))

        R_base = edge_len / (2.0 * math.sin(math.pi / n)) + stand_off

        z_min = -0.5 * height
        z_max =  0.5 * height - cap_top
        if z_max <= z_min:
            return np.zeros((0, 3), dtype=float)

        def ngon_polyline(radius: float, samples_per_edge: int = 8) -> np.ndarray:
            verts = []
            for i in range(n):
                a = phase + 2.0 * math.pi * (i / n)
                verts.append([radius * math.cos(a), radius * math.sin(a)])
            verts = np.asarray(verts, float)
            segs = []
            for i in range(n):
                p0 = verts[i]
                p1 = verts[(i + 1) % n]
                m = max(2, samples_per_edge)
                t = np.linspace(0.0, 1.0, m, endpoint=False)
                segs.append((1.0 - t)[:, None] * p0 + t[:, None] * p1)
            poly = np.vstack(segs)
            return np.c_[poly, np.zeros((poly.shape[0],))]

        base_poly = ngon_polyline(R_base, samples_per_edge=max(3, int(max(4.0, step) // 1.0)))
        base2d = base_poly[:, :2]
        seg = base2d[1:] - base2d[:-1]
        L_base = float(np.linalg.norm(seg, axis=1).sum())
        if L_base <= 1e-9:
            return np.zeros((0, 3), dtype=float)

        ds_per_dz = L_base / pitch
        pts = []
        s_acc = 0.0
        z = z_min
        while z <= z_max + 1e-9:
            alpha = (z - z_min) / max((z_max - z_min), 1e-9)
            scale = max(1e-3, 1.0 - alpha)
            Rz = R_base * scale
            poly_z = base2d * (Rz / max(R_base, 1e-9))
            pt_xy = PathBuilder._point_on_polyline_by_arclength(np.c_[poly_z, np.zeros((poly_z.shape[0],))], s_acc)
            pts.append([pt_xy[0], pt_xy[1], z])
            s_acc += ds_per_dz * dz
            z += dz

        P = np.asarray(pts, dtype=float)
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P

    # ------------------- Polyhelix: Cube -------------------
    @staticmethod
    def _polyhelix_cube(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        edge = float(p.get("edge_len_mm", 100.0))
        height = float(p.get("height_mm", 100.0))
        pitch  = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz     = max(1e-6, float(p.get("dz_mm", 1.0)))
        phase  = math.radians(float(p.get("start_phase_deg", 0.0)))
        stand_off = float(p.get("stand_off_mm", 25.0))
        corner_r  = float(p.get("corner_roll_radius_mm", 8.0))

        hx = 0.5 * edge + stand_off
        hy = 0.5 * edge + stand_off

        base_poly = PathBuilder._polyline_rounded_rect(0.0, 0.0, hx, hy, corner_r, step=max(1.0, step))
        base_xy = base_poly[:, :2]

        if abs(phase) > 1e-12:
            R2 = np.array([[math.cos(phase), -math.sin(phase)],
                           [math.sin(phase),  math.cos(phase)]], dtype=float)
            base_xy = base_xy @ R2.T

        seg = base_xy[1:] - base_xy[:-1]
        L = float(np.linalg.norm(seg, axis=1).sum())
        if L <= 1e-9:
            return np.zeros((0, 3), dtype=float)

        z_min = -0.5 * height
        z_max =  0.5 * height
        ds_per_dz = L / pitch

        pts = []
        s_acc = 0.0
        z = z_min
        while z <= z_max + 1e-9:
            pt_xy = PathBuilder._point_on_polyline_by_arclength(np.c_[base_xy, np.zeros((base_xy.shape[0],))], s_acc)
            pts.append([pt_xy[0], pt_xy[1], z])
            s_acc += ds_per_dz * dz
            z += dz

        P = np.asarray(pts, dtype=float)
        if len(P) > max_points:
            stride = max(1, len(P) // max_points)
            P = P[::stride]
        return P
