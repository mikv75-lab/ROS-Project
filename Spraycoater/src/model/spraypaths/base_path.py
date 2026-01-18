# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, List, Tuple, Optional
import math
import numpy as np

from .path_data import PathData

class BasePath:
    """
    Basisklasse für alle Pfad-Generatoren.
    Stellt Geometrie-Helfer bereit (Fillet, Trim, Rotation).
    """

    TYPE_ID: str = ""

    def __init__(self, params: Dict[str, Any]) -> None:
        self.params: Dict[str, Any] = dict(params or {})

    # ---------- Public Contract ----------
    def build_points(self, *, step_mm: float, max_points: int) -> np.ndarray:
        """Muss von Unterklassen implementiert werden. Liefert Nx3 Array."""
        raise NotImplementedError

    def to_pathdata(self, *, step_mm: float, max_points: int) -> PathData:
        P = self.build_points(step_mm=step_mm, max_points=max_points)
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        if P.size:
            # Filter NaNs/Infs
            P = P[np.isfinite(P).all(axis=1)]
        
        meta = {
            "source": (self.TYPE_ID or "unknown"), 
            "params": self._json_safe_meta(dict(self.params or {}))
        }
        return PathData(points_mm=self._decimate(P, int(max_points)), meta=meta)

    # ---------- Shared Helpers ----------
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
        return BasePath._decimate(P, int(max_points))

    @staticmethod
    def _json_safe_meta(meta: Dict[str, Any]) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for k, v in (meta or {}).items():
            if isinstance(v, (np.floating, np.integer)):
                out[k] = v.item()
            elif isinstance(v, np.ndarray):
                out[k] = v.tolist()
            elif isinstance(v, dict):
                out[k] = BasePath._json_safe_meta(v)
            elif isinstance(v, (list, tuple)):
                tmp = []
                for x in v:
                    if isinstance(x, (np.floating, np.integer)):
                        tmp.append(x.item())
                    elif isinstance(x, np.ndarray):
                        tmp.append(x.tolist())
                    else:
                        tmp.append(x)
                out[k] = tmp
            else:
                out[k] = v
        return out

    # ---------- Geometry Helpers ----------
    @staticmethod
    def _rot2(points_xy: np.ndarray, angle_deg: float) -> np.ndarray:
        P = np.asarray(points_xy, dtype=float).reshape(-1, 2)
        a = float(angle_deg)
        if abs(a) <= 1e-12:
            return P
        th = math.radians(a)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]], dtype=float)
        return P @ R.T

    @staticmethod
    def _fillet_polyline_2d(poly: np.ndarray, *, radius: float, step: float) -> np.ndarray:
        # (Vollständige Implementierung aus deinem Upload übernommen)
        P = np.asarray(poly, dtype=float).reshape(-1, 2)
        if P.shape[0] < 3:
            return P
        r = max(0.0, float(radius))
        if r <= 1e-9:
            return P

        def unit(v: np.ndarray) -> np.ndarray:
            n = float(np.linalg.norm(v))
            if n <= 1e-12 or not np.isfinite(n):
                return np.array([1.0, 0.0], dtype=float)
            return v / n

        out: List[np.ndarray] = [P[0].copy()]

        for i in range(1, P.shape[0] - 1):
            p0, p1, p2 = P[i - 1], P[i], P[i + 1]
            v_in, v_out = p0 - p1, p2 - p1
            Lin, Lout = float(np.linalg.norm(v_in)), float(np.linalg.norm(v_out))
            if Lin <= 1e-9 or Lout <= 1e-9:
                out.append(p1.copy())
                continue

            u_in, u_out = unit(v_in), unit(v_out)
            a, b = -u_in, u_out
            dot = float(np.clip(np.dot(a, b), -1.0, 1.0))
            theta = math.acos(dot)
            
            if theta <= 1e-6 or abs(math.pi - theta) <= 1e-6:
                out.append(p1.copy())
                continue

            t = r / math.tan(theta / 2.0)
            t_max = 0.45 * min(Lin, Lout)
            t = min(t, t_max) if (np.isfinite(t) and t > 1e-9) else t_max
            
            if t <= 1e-9:
                out.append(p1.copy())
                continue

            p_tan_in = p1 + (-a) * t
            p_tan_out = p1 + b * t

            cross = float(a[0] * b[1] - a[1] * b[0])
            n_a = np.array([-a[1], a[0]], dtype=float)
            n_b = np.array([-b[1], b[0]], dtype=float)

            if cross > 0.0:
                oa, ob = p_tan_in + n_a * r, p_tan_out + n_b * r
            else:
                oa, ob = p_tan_in - n_a * r, p_tan_out - n_b * r

            # Kreisbogen berechnen
            A = np.stack([a, -b], axis=1)
            try:
                st = np.linalg.solve(A, (ob - oa))
                c = oa + a * float(st[0])
                ang0 = math.atan2(p_tan_in[1] - c[1], p_tan_in[0] - c[0])
                ang1 = math.atan2(p_tan_out[1] - c[1], p_tan_out[0] - c[0])
                
                if cross > 0.0:
                    if ang1 < ang0: ang1 += 2.0 * math.pi
                    sweep = ang1 - ang0
                else:
                    if ang1 > ang0: ang1 -= 2.0 * math.pi
                    sweep = ang0 - ang1
                
                nseg = max(2, int(abs(sweep) * r / max(step, 1e-6)) + 1)
                if np.linalg.norm(out[-1] - p_tan_in) > 1e-9:
                    out.append(p_tan_in.copy())

                for k in range(1, nseg):
                    u = k / float(nseg)
                    ang = ang0 + u * sweep
                    out.append(c + r * np.array([math.cos(ang), math.sin(ang)], dtype=float))
                out.append(p_tan_out.copy())
            except Exception:
                out.append(p1.copy())

        out.append(P[-1].copy())
        return np.asarray(out, dtype=float).reshape(-1, 2)

    @staticmethod
    def _polyline_circle(cx: float, cy: float, r: float, step: float) -> np.ndarray:
        n = max(12, int((2.0 * math.pi * r) / max(step, 1e-6)) + 1)
        theta = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
        return np.c_[cx + r * np.cos(theta), cy + r * np.sin(theta), np.zeros((n,), dtype=float)]

    @staticmethod
    def _polyline_rounded_rect(cx: float, cy: float, hx: float, hy: float, r: float, step: float) -> np.ndarray:
        pts: List[np.ndarray] = []
        r = max(0.0, min(float(r), float(hx), float(hy)))

        def add_l(p0: np.ndarray, p1: np.ndarray) -> None:
            dist = float(np.linalg.norm(p1 - p0))
            n = max(2, int(dist / max(step, 1e-6)) + 1)
            t = np.linspace(0, 1, n, endpoint=False)
            seg = (1.0 - t)[:, None] * p0[None, :] + t[:, None] * p1[None, :]
            pts.append(np.c_[seg, np.zeros((seg.shape[0],), dtype=float)])

        def add_a(c: np.ndarray, rr: float, a0: float, a1: float) -> None:
            arc_len = abs(a1 - a0) * rr
            n = max(2, int(arc_len / max(step, 1e-6)) + 1)
            ang = np.linspace(a0, a1, n, endpoint=False)
            seg = np.c_[c[0] + rr * np.cos(ang), c[1] + rr * np.sin(ang), np.zeros_like(ang)]
            pts.append(seg)

        # 4 Ecken + 4 Seiten
        c0, c1 = np.array([cx + hx - r, cy + hy - r]), np.array([cx - hx + r, cy + hy - r])
        c2, c3 = np.array([cx - hx + r, cy - hy + r]), np.array([cx + hx - r, cy - hy + r])
        
        add_l(np.array([cx - hx + r, cy - hy]), np.array([cx + hx - r, cy - hy])) # Bottom
        add_a(c3, r, -0.5 * math.pi, 0.0) # BR
        add_l(np.array([cx + hx, cy - hy + r]), np.array([cx + hx, cy + hy - r])) # Right
        add_a(c0, r, 0.0, 0.5 * math.pi) # TR
        add_l(np.array([cx + hx - r, cy + hy]), np.array([cx - hx + r, cy + hy])) # Top
        add_a(c1, r, 0.5 * math.pi, math.pi) # TL
        add_l(np.array([cx - hx, cy + hy - r]), np.array([cx - hx, cy - hy + r])) # Left
        add_a(c2, r, math.pi, 1.5 * math.pi) # BL
        
        return np.vstack(pts) if pts else np.zeros((0, 3), float)

    @staticmethod
    def _trim_by_arclength(P: np.ndarray, start_off_mm: float, end_off_mm: float) -> np.ndarray:
        P = np.asarray(P, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2: return P
        start, end = max(0.0, float(start_off_mm)), max(0.0, float(end_off_mm))
        if start <= 0.0 and end <= 0.0: return P
        
        seglen = np.linalg.norm(P[1:] - P[:-1], axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        L = float(cum[-1])
        if L <= 1e-9: return P
        
        s0, s1 = min(start, L), max(0.0, L - end)
        if s1 <= s0 + 1e-9: return np.zeros((0, 3), float)

        def interp(s: float) -> np.ndarray:
            i = np.searchsorted(cum, s, side="right") - 1
            i = max(0, min(i, len(seglen) - 1))
            t = (s - cum[i]) / seglen[i] if seglen[i] > 1e-12 else 0.0
            return (1.0 - t) * P[i] + t * P[i + 1]

        pts = [interp(s0)]
        pts.extend(P[np.where((cum > s0) & (cum < s1))])
        pts.append(interp(s1))
        return np.vstack(pts)

    @staticmethod
    def _point_on_polyline_by_arclength(poly2d: np.ndarray, s: float) -> np.ndarray:
        P = np.asarray(poly2d, dtype=float).reshape(-1, 2)
        seglen = np.linalg.norm(P[1:] - P[:-1], axis=1)
        L = float(seglen.sum())
        if L <= 1e-12: return P[0].copy()
        s = float(s) % L
        cum = np.concatenate([[0.0], np.cumsum(seglen)])
        i = np.searchsorted(cum, s, side="right") - 1
        i = max(0, min(i, len(seglen) - 1))
        t = (s - cum[i]) / seglen[i] if seglen[i] > 1e-12 else 0.0
        return (1.0 - t) * P[i] + t * P[i + 1]