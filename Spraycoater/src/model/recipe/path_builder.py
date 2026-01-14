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
      - path.meander.plane
      - path.spiral.plane
      - path.spiral.cylinder
      - path.perimeter_follow.plane
      - path.polyhelix.pyramid
      - path.polyhelix.cube

    FIX: Berücksichtigt 'recipe.parameters' (z.B. max_points, sample_step_mm) vorrangig.
    FIX: Mäander erzeugt nun immer eine durchgängige, rund verbundene Bahn.
    FIX: Helix (Cylinder) Berechnung robust (float turns) und zentriert.
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

        P = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2:
            pass # Leere Pfade erlaubt

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
        # Auch hier vorab auflösen, damit Validierung korrekt ist
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

        if not pbs:
            pass 
            
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
        """
        Erzeugt einen durchgängigen Mäander-Pfad (Schlangenlinie).
        Verbindet die Zeilen automatisch mit einem 180° Bogen (rund).
        """
        area = p.get("area", {}) or {}
        shape = str(area.get("shape", "rect")).lower()

        cx, cy = (area.get("center_xy_mm") or [0.0, 0.0])
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        angle = float(p.get("angle_deg", 0.0))
        edge_extend = float(p.get("edge_extend_mm", 0.0))
        
        margin = max(0.0, float(p.get("margin_mm", 0.0)))
        start_corner = str(p.get("start", "auto")).lower()
        
        # Logik-Flags für Startpunkt
        reverse_y = False
        reverse_x_base = False
        
        if start_corner == "minx_maxy": # Top-Left
            reverse_y = True
        elif start_corner == "maxx_miny": # Bottom-Right
            reverse_x_base = True
        elif start_corner == "maxx_maxy": # Top-Right
            reverse_y = True
            reverse_x_base = True

        # 1. Zeilen-Segmente berechnen
        line_segments = [] # [(x0, y), (x1, y)]
        
        if shape in ("circle", "disk"):
            R = max(0.0, float(area.get("radius_mm", 50.0)) - margin)
            if R <= 0: return np.zeros((0,3), float)

            # Anzahl Linien, zentriert um 0
            n_lines = int(2.0 * R / pitch)
            if n_lines < 1: n_lines = 1
            ys = np.linspace(-(n_lines-1)*pitch/2.0, (n_lines-1)*pitch/2.0, n_lines)

            if reverse_y: ys = ys[::-1]

            for y in ys:
                # Sehnenlänge im Kreis bei y
                if abs(y) >= R: continue
                span = math.sqrt(max(R * R - y * y, 0.0))
                x0, x1 = -span - edge_extend, span + edge_extend
                line_segments.append(((x0, y), (x1, y)))
        else:
            sx, sy = area.get("size_mm", [100.0, 100.0])
            hx = max(0.0, 0.5 * float(sx) - margin)
            hy = max(0.0, 0.5 * float(sy) - margin)
            if hx <= 0 or hy <= 0: return np.zeros((0,3), float)

            # Bereich ist 2*hy. Wir wollen Linien im Abstand pitch.
            n_lines = int(2.0 * hy / pitch) + 1
            # Startpunkt so wählen, dass es zentriert ist
            # y_span = (n_lines-1)*pitch
            # start = -y_span/2
            start_y = -((n_lines - 1) * pitch) / 2.0
            ys = np.array([start_y + i*pitch for i in range(n_lines)])
            
            if reverse_y: ys = ys[::-1]

            for y in ys:
                x0, x1 = -hx - edge_extend, hx + edge_extend
                line_segments.append(((x0, y), (x1, y)))

        if not line_segments:
            return np.zeros((0, 3), dtype=float)

        # 2. Verbinden der Segmente zu einer durchgängigen Polyline
        full_pts = []
        
        def discretize_line(p_start, p_end, step_size):
            dist = math.hypot(p_end[0]-p_start[0], p_end[1]-p_start[1])
            n = max(2, int(dist / max(step_size, 1e-6)) + 1)
            ts = np.linspace(0, 1, n)
            return [(p_start[0]*(1-t) + p_end[0]*t, p_start[1]*(1-t) + p_end[1]*t) for t in ts]

        def discretize_arc_connect(p1, p2, step_size):
            """Erzeugt einen Halbkreis von p1 nach p2."""
            # Mittelpunkt
            mx, my = (p1[0] + p2[0]) / 2.0, (p1[1] + p2[1]) / 2.0
            # Vektor P1->P2 (Basis des Halbkreises)
            dx, dy = p2[0] - p1[0], p2[1] - p1[1]
            dist = math.hypot(dx, dy)
            if dist < 1e-6: return [p1]
            
            radius = dist / 2.0
            # arc length approx pi*r
            n = max(4, int((math.pi * radius) / max(step_size, 1e-6)) + 1)
            
            # Basis-Vektoren für Halbkreis
            u_vec = np.array([dx, dy]) / dist
            v_vec = np.array([u_vec[1], -u_vec[0]]) # Rotate 90 deg
            
            # Bulge Richtung bestimmen (immer nach Außen relativ zur Mitte)
            # mx > 0 -> rechts -> bulge nach rechts (+X)
            # mx < 0 -> links -> bulge nach links (-X)
            bulge_dir = 1.0 if mx > 0 else -1.0
            
            # Wenn v_vec in die falsche Richtung zeigt, umdrehen
            if (v_vec[0] * bulge_dir) < 0:
                v_vec = -v_vec
            
            pts = []
            for i in range(1, n): # Start/Ende weglassen (werden von Linien gedeckt)
                t = i / float(n)
                lin_pt = np.array(p1) * (1-t) + np.array(p2) * t
                offset = v_vec * radius * math.sin(t * math.pi) # Sine bump
                pt = lin_pt + offset
                pts.append((pt[0], pt[1]))
                
            return pts

        current_rev = reverse_x_base 
        
        for i, (p0, p1) in enumerate(line_segments):
            # Start/Ende der aktuellen Zeile festlegen
            start = p1 if current_rev else p0
            end   = p0 if current_rev else p1
            
            # 1. Verbindung vom vorherigen Zeilenende zum aktuellen Start
            if i > 0:
                prev_end = full_pts[-1]
                # Runde Verbindung einfügen
                connector = discretize_arc_connect(prev_end, start, step)
                full_pts.extend(connector)
            else:
                full_pts.append(start)
            
            # 2. Die Zeile selbst
            line_pts = discretize_line(start, end, step)
            full_pts.extend(line_pts[1:]) # Startpunkt ist schon da
            
            # Richtung für nächste Zeile umkehren
            current_rev = not current_rev

        poly2d = np.array(full_pts, dtype=float)

        # Rotation
        if abs(angle) > 1e-9 and poly2d.shape[0] > 0:
            th = math.radians(angle)
            R2 = np.array([[math.cos(th), -math.sin(th)],
                           [math.sin(th),  math.cos(th)]], dtype=float)
            poly2d = poly2d @ R2.T

        # Offset Center
        poly2d += np.array([cx, cy], dtype=float)
        
        # Z-Koordinate hinzufügen
        P = np.c_[poly2d, np.zeros((poly2d.shape[0],), dtype=float)]
        
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _spiral_plane(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        cx, cy = (p.get("center_xy_mm") or [0.0, 0.0])
        r_outer = float(p.get("r_outer_mm", p.get("r_end_mm", 70.0)))
        r_inner = float(p.get("r_inner_mm", p.get("r_start_mm", 10.0)))
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        
        z_level = float(p.get("z_mm", 10.0))
        direction = str(p.get("direction", "ccw")).lower()

        turns = max(int(abs(r_outer - r_inner) / pitch), 1)
        theta_max = 2.0 * math.pi * turns

        dtheta = step / max(max(r_outer, r_inner), 1e-6)
        N = int(theta_max / max(dtheta, 1e-6)) + 2
        theta = np.linspace(0.0, theta_max, N)

        # Interpolation r_outer -> r_inner
        r = r_outer - (r_outer - r_inner) * (theta / theta_max)
        
        if direction == "cw":
            theta = -theta

        x = cx + r * np.cos(theta)
        y = cy + r * np.sin(theta)
        z = np.full_like(x, z_level)

        P = np.c_[x, y, z]
        return PathBuilder._decimate(P, max_points)

    @staticmethod
    def _spiral_cylinder_centerline(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        """
        Erzeugt Helix-Punkte zentriert um (0,0,0) in XY, Z verläuft vertikal.
        Verwendet explizite 'radius_mm' und 'height_mm' Parameter, wenn vorhanden.
        """
        pitch = max(1e-6, float(p.get("pitch_mm", 10.0)))
        m_top = float(p.get("margin_top_mm", 0.0))
        m_bot = float(p.get("margin_bottom_mm", 0.0))
        start_from = str(p.get("start_from", "top")).lower()
        direction = str(p.get("direction", "ccw")).lower()

        # Radius und Höhe robust lesen (Fallback auf 100/20 falls nicht da)
        radius = float(p.get("radius_mm", 20.0)) + float(p.get("outside_mm", 1.0))
        height = float(p.get("height_mm", 100.0))

        # Nutzbare Höhe berechnen
        usable_h = max(height - m_top - m_bot, 0.0)
        
        # Falls keine Höhe da ist, return leeren Pfad
        if usable_h <= 1e-9:
            return np.zeros((0, 3), dtype=float)

        # Turns als Float berechnen, um "Abschneiden" bei int() zu verhindern
        turns = usable_h / pitch
        
        # Bogenlänge pro Windung: sqrt((2*pi*r)^2 + pitch^2)
        len_per_turn = math.sqrt((2 * math.pi * radius)**2 + pitch**2)
        total_len = turns * len_per_turn
        
        # Anzahl Punkte bestimmen
        num_points = max(2, int(total_len / max(step, 1e-6)) + 1)
        
        # Parameter t von 0 bis 1
        t = np.linspace(0.0, 1.0, num_points)
        
        # Z-Koordinate (Zentriert um 0: von +H/2 bis -H/2)
        z_top_limit = height / 2.0 - m_top
        z_bot_limit = -height / 2.0 + m_bot
        
        if start_from == "top":
            z = z_top_limit - t * usable_h
        else:
            z = z_bot_limit + t * usable_h
            
        # Winkel (Theta)
        total_angle = turns * 2.0 * math.pi
        if direction == "cw":
            angles = -t * total_angle
        else:
            angles = t * total_angle
            
        x = radius * np.cos(angles)
        y = radius * np.sin(angles)

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
        return PathBuilder._polyhelix_generic(p, step, max_points, is_cube=False)

    @staticmethod
    def _polyhelix_cube(p: Dict[str, Any], step: float, max_points: int) -> np.ndarray:
        return PathBuilder._polyhelix_generic(p, step, max_points, is_cube=True)

    @staticmethod
    def _polyhelix_generic(p: Dict[str, Any], step: float, max_points: int, is_cube: bool) -> np.ndarray:
        if is_cube:
            edge = float(p.get("edge_len_mm", 100.0))
            n_sides = 4
            R_base = 0 
        else:
            n_sides = int(p.get("base_polygon_sides", 4))
            edge = float(p.get("base_edge_len_mm", 100.0))
            R_base = edge / (2.0 * math.sin(math.pi / n_sides))

        height = float(p.get("height_mm", 100.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 6.0)))
        dz = max(1e-6, float(p.get("dz_mm", 1.0)))
        stand_off = float(p.get("stand_off_mm", 25.0))
        
        if is_cube:
            corner_r = float(p.get("corner_roll_radius_mm", 8.0))
            hx = 0.5 * edge + stand_off
            hy = 0.5 * edge + stand_off
            base_poly = PathBuilder._polyline_rounded_rect(
                0.0, 0.0, hx, hy, corner_r, step=max(1.0, step)
            )
            base2d = base_poly[:, :2]
        else:
            phase = math.radians(float(p.get("start_phase_deg", 0.0)))
            R_actual = R_base + stand_off
            verts = []
            for i in range(n_sides):
                a = phase + 2.0 * math.pi * (i / n_sides)
                verts.append([R_actual * math.cos(a), R_actual * math.sin(a)])
            verts = np.asarray(verts, float)
            segs = []
            for i in range(n_sides):
                p0 = verts[i]
                p1 = verts[(i + 1) % n_sides]
                m = max(2, int(10)) 
                t = np.linspace(0.0, 1.0, m, endpoint=False)
                segs.append((1.0 - t)[:, None] * p0 + t[:, None] * p1)
            poly = np.vstack(segs)
            base2d = poly

        seg = base2d[1:] - base2d[:-1]
        L_base = float(np.linalg.norm(seg, axis=1).sum())
        if L_base <= 1e-9:
            return np.zeros((0, 3), dtype=float)

        ds_per_dz = L_base / pitch
        cap_top = max(0.0, float(p.get("cap_top_mm", 8.0))) if not is_cube else 0.0
        z_min = -0.5 * height
        z_max = 0.5 * height - cap_top

        pts = []
        s_acc = 0.0
        z = z_min
        
        while z <= z_max + 1e-9:
            scale = 1.0
            if not is_cube:
                dist_from_top = (0.5*height) - z
                scale = max(0.0, dist_from_top / height)

            poly_z = base2d * scale
            pt_xy_base = PathBuilder._point_on_polyline_by_arclength(
                np.c_[base2d, np.zeros((base2d.shape[0],), dtype=float)],
                s_acc,
            )
            pt_xy = pt_xy_base * scale
            pts.append([float(pt_xy[0]), float(pt_xy[1]), float(z)])

            s_acc += ds_per_dz * dz
            z += dz

        return PathBuilder._decimate(np.asarray(pts, dtype=float), max_points)