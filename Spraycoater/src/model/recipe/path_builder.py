# app/model/recipe/path_builder.py
# -*- coding: utf-8 -*-
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
      - path.meander.plane
      - path.spiral.plane
      - path.spiral.cylinder
      - path.perimeter_follow.plane
      - path.polyhelix.pyramid
      - path.polyhelix.cube

    Alternativ:
      - points_mm oder polyline_mm im dict -> direkt übernommen

    Hinweis:
    - bewusst strikt: fehlende Pflichtparameter / unknown types => Exception
    """

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        """
        Kompatibilitäts-Init (no-op).

        In älterem Code wurde PathBuilder teils instanziiert (z.B. PathBuilder(...)).
        Diese Klasse ist aber als statische Utility gedacht. Damit Save/Compile nicht
        crasht, akzeptieren wir args/kwargs stillschweigend.
        """
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
        """Erzeugt PathData für eine Side aus recipe.paths_by_side[side]."""
        PathBuilder._validate_inputs(globals_params, sample_step_mm, max_points)
        p = PathBuilder._extract_path_for_side(recipe, side)

        pd = PathBuilder._from_path_dict(
            p,
            sample_step_mm=float(sample_step_mm),
            max_points=int(max_points),
        )

        # Postprocessing Hook (aktuell: no-op + cleanup)
        pd = PathBuilder._postprocess(pd)

        # Globals in meta übernehmen (Eval/Debug)
        pd = PathBuilder._with_globals_meta(pd, globals_params)

        # Safety: mind. 2 Punkte
        P = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2:
            raise ValueError(f"PathBuilder: side '{side}' hat zu wenige Punkte ({P.shape[0]}).")

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
        PathBuilder._validate_inputs(globals_params, sample_step_mm, max_points)
        out: List[Tuple[str, PathData]] = []
        for s in sides:
            out.append(
                (s, PathBuilder.from_side(
                    recipe,
                    side=s,
                    globals_params=globals_params,
                    sample_step_mm=sample_step_mm,
                    max_points=max_points,
                ))
            )
        return out

    # ---------------------------
    # Validation & helpers
    # ---------------------------

    @staticmethod
    def _validate_inputs(g: Dict[str, Any], sample_step_mm: float, max_points: int) -> None:
        if not isinstance(g, dict):
            raise ValueError("PathBuilder: globals_params muss ein Dict sein.")
        missing = [k for k in ("stand_off_mm", "max_angle_deg") if k not in g]
        if missing:
            raise ValueError(f"PathBuilder: globals_params fehlen Keys: {missing}")
        if not (isinstance(sample_step_mm, (int, float)) and float(sample_step_mm) > 0.0):
            raise ValueError("PathBuilder: sample_step_mm muss > 0 sein.")
        if not (isinstance(max_points, int) and max_points >= 2):
            raise ValueError("PathBuilder: max_points muss int >= 2 sein.")

    @staticmethod
    def _decimate(P: np.ndarray, max_points: int) -> np.ndarray:
        """
        Downsampling, deterministisch.

        Vorteil ggü. Stride:
        - garantiert erstes UND letztes Sample enthalten
        - exakt <= max_points
        """
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        n = P.shape[0]
        if n <= max_points:
            return P
        idx = np.linspace(0, n - 1, num=max_points, dtype=int)
        return P[idx]

    @staticmethod
    def _as_points_mm(pts: Any, max_points: int) -> np.ndarray:
        """Konvertiert Punkte nach (N,3) float, filtert NaNs, begrenzt Punktzahl."""
        P = np.asarray(pts, dtype=float).reshape(-1, 3)
        if P.size == 0:
            return np.zeros((0, 3), dtype=float)

        mask = np.isfinite(P).all(axis=1)
        P = P[mask]
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _with_globals_meta(pd: PathData, g: Dict[str, Any]) -> PathData:
        meta = dict(pd.meta or {})
        meta["stand_off_mm"] = float(g["stand_off_mm"])
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

        if not pbs:
            raise KeyError("PathBuilder: recipe.paths_by_side ist leer.")
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

        return PathData(points_mm=P, meta={"source": ptype})

    # ============================================================
    # Generators
    # ============================================================

    @staticmethod
    def _meander_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        area = p.get("area", {}) or {}
        shape = str(area.get("shape", "rect")).lower()

        cx, cy = (area.get("center_xy_mm") or [0.0, 0.0])
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        angle = float(p.get("angle_deg", 0.0))
        boust = bool(p.get("boustrophedon", True))
        edge_extend = float(p.get("edge_extend_mm", 0.0))

        rows = []
        if shape in ("circle", "disk"):
            R = float(area.get("radius_mm", 50.0))
            ys = np.arange(-R, R + 1e-9, pitch)
            for i, y in enumerate(ys):
                span = math.sqrt(max(R * R - y * y, 0.0))
                x0, x1 = -span - edge_extend, span + edge_extend
                nseg = max(2, int((x1 - x0) / max(step, 1e-6)) + 1)
                xs = np.linspace(x0, x1, nseg)
                if boust and (i % 2 == 1):
                    xs = xs[::-1]
                rows.append(np.c_[xs, np.full_like(xs, y)])
        else:
            sx, sy = area.get("size_mm", [100.0, 100.0])
            hx, hy = 0.5 * float(sx), 0.5 * float(sy)
            ys = np.arange(-hy, hy + 1e-9, pitch)
            for i, y in enumerate(ys):
                x0, x1 = -hx - edge_extend, hx + edge_extend
                nseg = max(2, int((x1 - x0) / max(step, 1e-6)) + 1)
                xs = np.linspace(x0, x1, nseg)
                if boust and (i % 2 == 1):
                    xs = xs[::-1]
                rows.append(np.c_[xs, np.full_like(xs, y)])

        poly2d = np.vstack(rows) if rows else np.zeros((0, 2), dtype=float)

        if abs(angle) > 1e-9 and poly2d.shape[0] > 0:
            th = math.radians(angle)
            R2 = np.array([[math.cos(th), -math.sin(th)],
                           [math.sin(th),  math.cos(th)]], dtype=float)
            poly2d = poly2d @ R2.T

        poly2d += np.array([cx, cy], dtype=float)
        P = np.c_[poly2d, np.zeros((poly2d.shape[0],), dtype=float)]
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _spiral_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        cx, cy = (p.get("center_xy_mm") or [0.0, 0.0])
        r_outer = float(p.get("r_outer_mm", p.get("r_end_mm", 70.0)))
        r_inner = float(p.get("r_inner_mm", p.get("r_start_mm", 10.0)))
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))

        turns = max(int((r_outer - r_inner) / pitch), 1)
        theta_max = 2.0 * math.pi * turns

        dtheta = step / max(r_outer, 1e-6)
        N = int(theta_max / max(dtheta, 1e-6)) + 2
        theta = np.linspace(0.0, theta_max, N)

        r = r_outer - (r_outer - r_inner) * (theta / theta_max)
        x = cx + r * np.cos(theta)
        y = cy + r * np.sin(theta)

        P = np.c_[x, y, np.zeros_like(x)]
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _spiral_cylinder_centerline(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        pitch = max(1e-6, float(p.get("pitch_mm", 10.0)))
        m_top = float(p.get("margin_top_mm", 0.0))
        m_bot = float(p.get("margin_bottom_mm", 0.0))
        start_from = str(p.get("start_from", "top")).lower()
        direction = str(p.get("direction", "ccw")).lower()

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
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _perimeter_follow_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        area = dict(p.get("area") or {})
        shape = str(area.get("shape", "circle")).lower()
        cx, cy = (area.get("center_xy_mm") or [0.0, 0.0])

        loops = max(int(p.get("loops", 1)), 1)
        off0 = float(p.get("offset_start_mm", 1.0))
        offstep = float(p.get("offset_step_mm", 1.0))
        blend = max(0.0, float(p.get("corner_blend_mm", 0.0)))
        lead_in = float(p.get("lead_in_mm", 0.0))

        polylines: List[np.ndarray] = []

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
                    cx=cx, cy=cy,
                    hx=hx, hy=hy,
                    r=max(0.0, min(blend, hx, hy)),
                    step=step,
                )

            if poly.shape[0] == 0:
                continue

            if (k % 2) == 1:
                poly = poly[::-1].copy()

            if lead_in > 1e-9 and poly.shape[0] >= 2:
                dir_vec = (poly[1] - poly[0])
                n = np.linalg.norm(dir_vec)
                if n > 1e-9:
                    lead_pt = poly[0] - (dir_vec / n) * lead_in
                    poly = np.vstack([lead_pt, poly])

            polylines.append(poly)

        if not polylines:
            return np.zeros((0, 3), dtype=float)

        P = np.vstack(polylines)
        return PathBuilder._decimate(P, max_points)

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
        hx = float(hx)
        hy = float(hy)
        r = max(0.0, min(float(r), hx, hy))

        def n_for_len(L: float) -> int:
            return max(2, int(L / max(step, 1e-6)) + 1)

        pts: List[np.ndarray] = []

        def append_line(p0: np.ndarray, p1: np.ndarray, nseg: int) -> None:
            t = np.linspace(0.0, 1.0, nseg, endpoint=False)
            pts.append((1.0 - t)[:, None] * p0 + t[:, None] * p1)

        def append_arc(center: np.ndarray, rr: float, a0: float, a1: float, nseg: int) -> None:
            if rr <= 0.0:
                return
            ang = np.linspace(a0, a1, nseg, endpoint=False)
            x = center[0] + rr * np.cos(ang)
            y = center[1] + rr * np.sin(ang)
            pts.append(np.c_[x, y, np.zeros_like(x)])

        z0 = 0.0
        c_tr = np.array([cx + hx - r, cy + hy - r, z0])
        c_tl = np.array([cx - hx + r, cy + hy - r, z0])
        c_bl = np.array([cx - hx + r, cy - hy + r, z0])
        c_br = np.array([cx + hx - r, cy - hy + r, z0])

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

        if not pts:
            return np.zeros((0, 3), dtype=float)

        poly = np.vstack(pts)

        if poly.shape[0] >= 2:
            mask = np.ones((poly.shape[0],), dtype=bool)
            mask[1:] = np.linalg.norm(poly[1:] - poly[:-1], axis=1) > 1e-9
            poly = poly[mask]

        return poly

    @staticmethod
    def _point_on_polyline_by_arclength(poly2d: np.ndarray, s: float) -> np.ndarray:
        """Punkt auf Polyline per Bogenlänge (wrap modulo Gesamtlänge)."""
        poly2d = np.asarray(poly2d, dtype=float)
        if poly2d.shape[0] < 2:
            return np.array([0.0, 0.0], dtype=float)

        P = poly2d[:, :2]
        seg = P[1:] - P[:-1]
        seglen = np.linalg.norm(seg, axis=1)
        L = float(seglen.sum())
        if L <= 1e-12:
            return P[0].copy()

        s = float(s) % L
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        i = np.searchsorted(cum, s, side="right") - 1
        i = max(0, min(i, len(seglen) - 1))

        ds = s - cum[i]
        if seglen[i] <= 1e-12:
            return P[i + 1].copy()

        t = ds / seglen[i]
        return (1.0 - t) * P[i] + t * P[i + 1]

    @staticmethod
    def _polyhelix_pyramid(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        n = int(p.get("base_polygon_sides", 4))
        n = min(128, max(3, n))

        edge_len = float(p.get("base_edge_len_mm", 100.0))
        height = float(p.get("height_mm", 60.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz = max(1e-6, float(p.get("dz_mm", 1.0)))

        phase = math.radians(float(p.get("start_phase_deg", 0.0)))
        stand_off = float(p.get("stand_off_mm", 25.0))
        cap_top = max(0.0, float(p.get("cap_top_mm", 8.0)))

        R_base = edge_len / (2.0 * math.sin(math.pi / n)) + stand_off

        z_min = -0.5 * height
        z_max = 0.5 * height - cap_top
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
            return np.c_[poly, np.zeros((poly.shape[0],), dtype=float)]

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

            poly_z = base2d * scale
            pt_xy = PathBuilder._point_on_polyline_by_arclength(
                np.c_[poly_z, np.zeros((poly_z.shape[0],), dtype=float)],
                s_acc,
            )
            pts.append([float(pt_xy[0]), float(pt_xy[1]), float(z)])

            s_acc += ds_per_dz * dz
            z += dz

        return PathBuilder._decimate(np.asarray(pts, dtype=float), max_points)

    @staticmethod
    def _polyhelix_cube(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        """
        Construct a polyhelix cube path.

        Parameters:
            p (Dict[str, Any]): The path definition dictionary with keys like
                "edge_len_mm", "height_mm", "pitch_mm", "dz_mm", "start_phase_deg",
                "stand_off_mm", and "corner_roll_radius_mm".
            step (float): The sampling step size in millimetres.
            max_points (int): The maximum number of points in the generated path.

        Returns:
            np.ndarray: Array of shape (N, 3) containing the 3D path points.
        """
        edge = float(p.get("edge_len_mm", 100.0))
        height = float(p.get("height_mm", 100.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz = max(1e-6, float(p.get("dz_mm", 1.0)))

        phase = math.radians(float(p.get("start_phase_deg", 0.0)))
        stand_off = float(p.get("stand_off_mm", 25.0))
        corner_r = float(p.get("corner_roll_radius_mm", 8.0))

        hx = 0.5 * edge + stand_off
        hy = 0.5 * edge + stand_off

        base_poly = PathBuilder._polyline_rounded_rect(
            0.0, 0.0, hx, hy, corner_r, step=max(1.0, step)
        )
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
        z_max = 0.5 * height
        ds_per_dz = L / pitch

        pts: List[List[float]] = []
        s_acc = 0.0
        z = z_min
        while z <= z_max + 1e-9:
            pt_xy = PathBuilder._point_on_polyline_by_arclength(
                np.c_[base_xy, np.zeros((base_xy.shape[0],), dtype=float)],
                s_acc,
            )
            pts.append([float(pt_xy[0]), float(pt_xy[1]), float(z)])

            s_acc += ds_per_dz * dz
            z += dz

        return PathBuilder._decimate(np.asarray(pts, dtype=float), max_points)

    @staticmethod
    def _polyhelix_cube(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        edge = float(p.get("edge_len_mm", 100.0))
        height = float(p.get("height_mm", 100.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz = max(1e-6, float(p.get("dz_mm", 1.0)))

        phase = math.radians(float(p.get("start_phase_deg", 0.0)))
        stand_off = float(p.get("stand_off_mm", 25.0))
        corner_r = float(p.get("corner_roll_radius_mm", 8.0))

        hx = 0.5 * edge + stand_off
        hy = 0.5 * edge + stand_off

        base_poly = PathBuilder._polyline_rounded_rect(
            0.0, 0.0, hx, hy, corner_r, step=max(1.0, step)
        )
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
        z_max = 0.5 * height
        ds_per_dz = L / pitch

        pts = []
        s_acc = 0.0
        z = z_min
        while z <= z_max + 1e-9:
            pt_xy = PathBuilder._point_on_polyline_by_arclength(
                np.c_[base_xy, np.zeros((base_xy.shape[0],), dtype=float)],
                s_acc,
            )
            pts.append([float(pt_xy[0]), float(pt_xy[1]), float(z)])

            s_acc += ds_per_dz * dz
            z += dz

        return PathBuilder._decimate(np.asarray(pts, dtype=float), max_points)
