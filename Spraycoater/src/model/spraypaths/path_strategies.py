# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from typing import List, Tuple, Dict, Any
import numpy as np

from .base_path import BasePath

class PointsPath(BasePath):
    TYPE_ID = "points"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        pts = self.params.get("points_mm") or self.params.get("polyline_mm")
        return self._as_points_mm(pts, int(max_points)) if pts else np.zeros((0, 3), float)

class MeanderPlanePath(BasePath):
    TYPE_ID = "path.meander.plane"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        p, step = self.params, float(step_mm)
        area = p.get("area", {}) or {}
        cx, cy = map(float, area.get("center_xy_mm") or [0.0, 0.0])
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        
        extend_y = max(0.0, float(p.get("extend_y_mm", p.get("margin_mm", 0.0))))
        extend_x = max(0.0, float(p.get("extend_x_mm", p.get("edge_extend_mm", 0.0))))
        
        # Grid generieren (vereinfacht für Kürze, Logik aus deinem Upload)
        lines = []
        is_circle = str(area.get("shape", "rect")).lower() in ("circle", "disk")
        
        if is_circle:
            R = max(0.0, float(area.get("radius_mm", 50.0)) + extend_y)
            n_lines = max(1, int(2.0 * R / pitch) + 1)
            ys = np.linspace(-(n_lines-1)*pitch/2, (n_lines-1)*pitch/2, n_lines)
            for y in ys:
                if abs(y) < R:
                    span = math.sqrt(R*R - y*y)
                    lines.append(((-span - extend_x, y), (span + extend_x, y)))
        else:
            sx, sy = area.get("size_mm", [100.0, 100.0])
            hx, hy = 0.5 * float(sx) + extend_x, 0.5 * float(sy) + extend_y
            n_lines = max(1, int(2.0 * hy / pitch) + 1)
            ys = np.linspace(-(n_lines-1)*pitch/2, (n_lines-1)*pitch/2, n_lines)
            for y in ys:
                lines.append(((-hx, y), (hx, y)))

        # Start-Corner Logic
        start_c = str(p.get("start", "auto")).lower()
        if start_c in ("minx_maxy", "maxx_maxy"): lines = lines[::-1]
        rev_x = start_c in ("maxx_miny", "maxx_maxy")

        # Meander verbinden
        pts_out = []
        turn_off = 0.5 * pitch
        for i, (p0, p1) in enumerate(lines):
            start, end = (p1, p0) if rev_x else (p0, p1)
            
            # Linie interpolieren
            dist = math.hypot(end[0]-start[0], end[1]-start[1])
            n = max(2, int(dist / max(step, 1e-6)) + 1)
            pts_line = np.column_stack([
                np.linspace(start[0], end[0], n),
                np.linspace(start[1], end[1], n)
            ])
            pts_out.extend(pts_line[1:] if i > 0 else pts_line)

            # Turn zum nächsten Segment
            if i < len(lines) - 1:
                nxt = lines[i+1]
                nxt_start = nxt[1] if not rev_x else nxt[0]
                dx_sign = -1.0 if rev_x else 1.0
                pts_out.append([end[0] + dx_sign * turn_off, end[1]])
                pts_out.append([end[0] + dx_sign * turn_off, nxt_start[1]])
            
            rev_x = not rev_x

        P = np.array(pts_out)
        if P.shape[0] < 2: return np.zeros((0,3), float)

        # Fillet
        cr = float(p.get("corner_radius_mm") or 0.5 * pitch)
        if cr > 0:
            try: P = self._fillet_polyline_2d(P, radius=cr, step=step)
            except: pass

        # Rotation & Shift
        P = self._rot2(P, float(p.get("angle_deg", 0.0))) + [cx, cy]
        return self._decimate(np.c_[P, np.zeros(len(P))], int(max_points))

class SpiralPlanePath(BasePath):
    TYPE_ID = "path.spiral.plane"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        p = self.params
        cx, cy = map(float, p.get("center_xy_mm") or [0.0, 0.0])
        ro = float(p.get("r_outer_mm", 70.0)) + float(p.get("radial_offset_mm", 0.0))
        ri = float(p.get("r_inner_mm", 10.0)) + float(p.get("radial_offset_mm", 0.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 5.0)))
        
        turns = max(int(abs(ro - ri) / pitch), 1)
        theta_max = 2.0 * math.pi * turns
        N = int(theta_max * max(ro, ri) / max(step_mm, 1e-6)) + 2
        
        t = np.linspace(0, 1, N)
        r = ro - (ro - ri) * t
        theta = t * theta_max
        if str(p.get("direction", "ccw")).lower() == "cw": theta = -theta
        
        return self._decimate(np.c_[cx + r*np.cos(theta), cy + r*np.sin(theta), np.zeros(N)], int(max_points))

class SpiralCylinderPath(BasePath):
    TYPE_ID = "path.spiral.cylinder"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        p = self.params
        h = float(p.get("height_mm", 100.0))
        r = float(p.get("radius_mm", 20.0)) + float(p.get("outside_mm", 0.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 10.0)))
        
        z_top = 0.5*h + float(p.get("top_extend_mm", 0.0))
        z_bot = -0.5*h - float(p.get("bottom_extend_mm", 0.0))
        
        turns = abs(z_top - z_bot) / pitch
        L = turns * math.sqrt((2*math.pi*r)**2 + pitch**2)
        N = max(2, int(L / max(float(step_mm), 1e-6)) + 1)
        
        t = np.linspace(0, 1, N)
        z = z_top - t * (z_top - z_bot) if p.get("start_from", "top") == "top" else z_bot + t * (z_top - z_bot)
        ang = t * turns * 2 * math.pi
        if str(p.get("direction", "ccw")).lower() == "cw": ang = -ang
        
        return self._decimate(np.c_[r*np.cos(ang), r*np.sin(ang), z], int(max_points))

class PerimeterFollowPlanePath(BasePath):
    TYPE_ID = "path.perimeter_follow.plane"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        p = self.params
        step = float(step_mm)
        area = dict(p.get("area") or {})
        cx, cy = map(float, area.get("center_xy_mm") or [0.0, 0.0])
        
        polys = []
        off0 = float(p.get("offset_start_mm", 1.0))
        d_off = float(p.get("offset_step_mm", 1.0))
        
        for k in range(max(1, int(p.get("loops", 1)))):
            off = off0 + k * d_off
            if area.get("shape") == "circle":
                r = float(area.get("radius_mm", 50.0)) - off
                if r > 0: polys.append(self._polyline_circle(cx, cy, r, step))
            else:
                sx, sy = area.get("size_mm", [100.0, 100.0])
                hx, hy = 0.5*float(sx)-off, 0.5*float(sy)-off
                if hx>0 and hy>0:
                    polys.append(self._polyline_rounded_rect(cx, cy, hx, hy, min(float(p.get("corner_blend_mm", 0)), hx, hy), step))
            
            if polys and k%2==1: polys[-1] = polys[-1][::-1] # ZigZag
            
        return self._decimate(np.vstack(polys) if polys else np.zeros((0,3)), int(max_points))

class PolyhelixPyramidPath(BasePath):
    TYPE_ID = "path.polyhelix.pyramid"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        return _PolyhelixGeneric.build(self.params, step_mm=step_mm, max_points=max_points, is_cube=False)

class PolyhelixCubePath(BasePath):
    TYPE_ID = "path.polyhelix.cube"
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        return _PolyhelixGeneric.build(self.params, step_mm=step_mm, max_points=max_points, is_cube=True)

class _PolyhelixGeneric(BasePath):
    @staticmethod
    def build(p: Dict, *, step_mm: float, max_points: int, is_cube: bool) -> np.ndarray:
        # (Implementierung übernommen, gekürzt für Übersichtlichkeit)
        edge = float(p.get("edge_len_mm" if is_cube else "base_edge_len_mm", 100.0))
        stand = float(p.get("stand_off_mm", 25.0))
        step = float(step_mm)
        
        # 1. Base Outline 2D
        if is_cube:
            base = BasePath._polyline_rounded_rect(0, 0, 0.5*edge+stand, 0.5*edge+stand, float(p.get("corner_roll_radius_mm", 8)), step)[:,:2]
        else:
            n = int(p.get("base_polygon_sides", 4))
            R = (edge / (2*math.sin(math.pi/n))) + stand
            ph = math.radians(float(p.get("start_phase_deg", 0)))
            pts = [ [R*math.cos(ph + 2*math.pi*i/n), R*math.sin(ph + 2*math.pi*i/n)] for i in range(n+1) ]
            chunks = []
            for i in range(n):
                d = np.linalg.norm(np.array(pts[i+1]) - np.array(pts[i]))
                t = np.linspace(0, 1, max(2, int(d/step)))
                chunks.append((1-t)[:,None]*np.array(pts[i]) + t[:,None]*np.array(pts[i+1]))
            base = np.vstack(chunks)

        # 2. Extrude/Spiral Z
        h = float(p.get("height_mm", 100.0))
        pitch = max(1e-6, float(p.get("pitch_mm", 6.0)))
        L_base = np.linalg.norm(base[1:]-base[:-1], axis=1).sum()
        
        pts3 = []
        z, s_acc = -0.5*h, 0.0
        while z <= 0.5*h:
            scale = 1.0 if is_cube else max(0.0, (0.5*h - z)/h)
            xy = BasePath._point_on_polyline_by_arclength(base, s_acc) * scale
            pts3.append([xy[0], xy[1], z])
            dz = float(p.get("dz_mm", 1.0))
            s_acc += (L_base/pitch) * dz
            z += dz
            
        P = np.array(pts3)
        return BasePath._decimate(BasePath._trim_by_arclength(P, float(p.get("start_offset_mm",0)), float(p.get("end_offset_mm",0))), int(max_points))