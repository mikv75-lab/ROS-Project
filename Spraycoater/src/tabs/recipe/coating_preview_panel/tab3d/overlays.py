# -*- coding: utf-8 -*-
# app/tabs/recipe/coating_preview_panel/views_3d/overlays.py

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, List

import logging
import numpy as np
import pyvista as pv

_LOG = logging.getLogger("tabs.recipe.preview.overlays")


# ============================================================
# Types
# ============================================================

Bounds = Tuple[float, float, float, float, float, float]


@dataclass
class OverlayOut:
    """
    Output bundle for a single side that Views3DBox can consume.

    All fields are optional; consumers should null-check.
    """
    side: str

    # base path (local mm, world already in raycast_result if needed)
    path_poly: Optional[pv.PolyData] = None
    path_markers: Optional[pv.PolyData] = None

    # raycast visuals (already prepared by raycast_projector wrapper)
    rays_hit_poly: Optional[pv.PolyData] = None
    rays_miss_poly: Optional[pv.PolyData] = None
    tcp_poly: Optional[pv.PolyData] = None

    # normals/frames derived from raycast result (or provided)
    normals_poly: Optional[pv.PolyData] = None
    frames: Optional[Dict[str, Any]] = None  # {origins,x_dirs,z_dirs,scale_mm,step}

    # visibility resolved here (views can still override)
    visibility: Dict[str, bool] = None  # type: ignore[assignment]


# ============================================================
# Helpers (STRICT, UI-neutral)
# ============================================================

def _safe_norm(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(-1)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < eps:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


def _sanitize_points(P: Any) -> np.ndarray:
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    if P.size == 0:
        return np.zeros((0, 3), dtype=float)
    mask = np.isfinite(P).all(axis=1)
    return P[mask]


def _decimate_indices(n: int, max_keep: int) -> np.ndarray:
    if n <= 0:
        return np.zeros((0,), dtype=int)
    if max_keep <= 0 or n <= max_keep:
        return np.arange(n, dtype=int)
    return np.linspace(0, n - 1, num=max_keep, dtype=int)


def _polyline_from_points(P: np.ndarray) -> Optional[pv.PolyData]:
    P = _sanitize_points(P)
    if P.shape[0] < 2:
        return None
    try:
        return pv.lines_from_points(P, close=False)
    except Exception:
        _LOG.exception("lines_from_points failed")
        return None


def _point_markers(P: np.ndarray, *, max_points: int) -> Optional[pv.PolyData]:
    P = _sanitize_points(P)
    if P.shape[0] == 0:
        return None
    idx = _decimate_indices(P.shape[0], max_points)
    if idx.size == 0:
        return None
    try:
        return pv.PolyData(P[idx])
    except Exception:
        _LOG.exception("marker PolyData failed")
        return None


def _poly_segments(starts: np.ndarray, dirs: np.ndarray, length: float) -> Optional[pv.PolyData]:
    """
    Create line segments from start points and direction vectors.
    """
    try:
        S = _sanitize_points(starts)
        D = np.asarray(dirs, dtype=float).reshape(-1, 3)
        n = min(S.shape[0], D.shape[0])
        if n <= 0:
            return None
        S = S[:n]
        D = D[:n]

        # normalize dirs
        Dn = np.empty_like(D)
        for i in range(n):
            Dn[i] = _safe_norm(D[i])

        E = S + Dn * float(length)

        pts = np.vstack([S, E])
        lines = np.empty((n, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(0, n, dtype=np.int64)
        lines[:, 2] = np.arange(n, 2 * n, dtype=np.int64)

        poly = pv.PolyData(pts)
        poly.lines = lines.reshape(-1)
        return poly
    except Exception:
        _LOG.exception("poly_segments failed")
        return None


# ============================================================
# Overlay config normalization
# ============================================================

def _get_bool(cfg: Dict[str, Any], keys: Tuple[str, ...], default: bool) -> bool:
    for k in keys:
        if k in cfg:
            return bool(cfg.get(k))
    return bool(default)


def _get_int(cfg: Dict[str, Any], keys: Tuple[str, ...], default: int) -> int:
    for k in keys:
        if k in cfg:
            try:
                return int(cfg.get(k))
            except Exception:
                pass
    return int(default)


def _get_float(cfg: Dict[str, Any], keys: Tuple[str, ...], default: float) -> float:
    for k in keys:
        if k in cfg:
            try:
                return float(cfg.get(k))
            except Exception:
                pass
    return float(default)


def _normalize_overlay_cfg(cfg: Any) -> Dict[str, Any]:
    """
    Accepts arbitrary groupbox cfg formats; returns a normalized dict.
    """
    if not isinstance(cfg, dict):
        cfg = {}

    # common keys from your existing panels: mask/path/hits/misses/normals/frames
    vis = {
        "path": _get_bool(cfg, ("path", "show_path"), True),
        "hits": _get_bool(cfg, ("hits", "show_hits"), False),
        "misses": _get_bool(cfg, ("misses", "show_misses"), False),
        "tcp": _get_bool(cfg, ("tcp", "show_tcp"), True),
        "normals": _get_bool(cfg, ("normals", "show_normals"), False),
        "frames": _get_bool(cfg, ("frames", "show_frames"), False),
    }

    # sampling/scale knobs (safe defaults)
    out = {
        "visibility": vis,
        "path_marker_max": _get_int(cfg, ("path_marker_max", "path_markers", "path_marker_points"), 200),
        "frame_step_max": _get_int(cfg, ("frame_step_max", "frames_max", "frame_max"), 50),
        "frame_scale_mm": _get_float(cfg, ("frame_scale_mm", "frame_scale"), 10.0),
        "normal_len_mm": _get_float(cfg, ("normal_len_mm", "normal_length"), 10.0),
        "normal_step_max": _get_int(cfg, ("normal_step_max", "normals_max", "normal_max"), 200),
    }
    return out


# ============================================================
# OverlayRenderer (SSoT, UI-neutral)
# ============================================================

class OverlayRenderer:
    """
    Produces overlay geometries for a given side, WITHOUT rendering.

    The 3D view layer (Views3DBox) decides how to display (colors/layers).
    """

    def __init__(self) -> None:
        # reserved for future caching if needed
        self._cache: Dict[str, Any] = {}

    def render_for_side(
        self,
        *,
        side: str,
        scene: Any,
        points_local_mm: np.ndarray,
        raycast_result: Any,
        hit_poly: Optional[pv.PolyData],
        miss_poly: Optional[pv.PolyData],
        tcp_poly: Optional[pv.PolyData],
        overlay_cfg: Any,
    ) -> OverlayOut:
        cfg = _normalize_overlay_cfg(overlay_cfg)
        vis = dict(cfg["visibility"])

        P_local = _sanitize_points(points_local_mm)

        out = OverlayOut(
            side=str(side),
            visibility=vis,
        )

        # --- path poly + markers (local; world embedding is done elsewhere if needed) ---
        if vis.get("path", True):
            out.path_poly = _polyline_from_points(P_local)
            out.path_markers = _point_markers(P_local, max_points=int(cfg["path_marker_max"]))

        # --- raycast polydatas (already computed in projector wrapper) ---
        if vis.get("hits", False):
            out.rays_hit_poly = hit_poly
        if vis.get("misses", False):
            out.rays_miss_poly = miss_poly
        if vis.get("tcp", True):
            out.tcp_poly = tcp_poly

        # --- normals / frames ---
        # We derive normals from raycast_result if possible.
        # Expected from raycast_projector: rc.valid, rc.hit_mm, rc.normal, rc.tcp_mm (or similar).
        try:
            rc = raycast_result
            valid = np.asarray(getattr(rc, "valid", None))
            normals = np.asarray(getattr(rc, "normal", None))
            hits = np.asarray(getattr(rc, "hit_mm", None))
            tcps = np.asarray(getattr(rc, "tcp_mm", None))

            # sanitize
            if valid is None or normals is None:
                valid = np.zeros((0,), dtype=bool)
            else:
                valid = np.asarray(valid, dtype=bool).reshape(-1)

            if normals is None:
                normals = np.zeros((0, 3), dtype=float)
            else:
                normals = np.asarray(normals, dtype=float).reshape(-1, 3)

            if hits is None:
                hits = np.zeros((0, 3), dtype=float)
            else:
                hits = np.asarray(hits, dtype=float).reshape(-1, 3)

            if tcps is None:
                tcps = np.zeros((0, 3), dtype=float)
            else:
                tcps = np.asarray(tcps, dtype=float).reshape(-1, 3)

            if vis.get("normals", False) and hits.shape[0] and normals.shape[0]:
                # only valid indices
                vidx = np.where(valid)[0] if valid.size else np.arange(min(hits.shape[0], normals.shape[0]))
                if vidx.size:
                    # decimate to avoid huge polydata
                    keep = _decimate_indices(vidx.size, int(cfg["normal_step_max"]))
                    vidx = vidx[keep]
                    seg = _poly_segments(hits[vidx], normals[vidx], float(cfg["normal_len_mm"]))
                    out.normals_poly = seg

            if vis.get("frames", False) and tcps.shape[0] and normals.shape[0]:
                # frames at TCP points; z_dir = normal; x_dir unknown here (views may compute)
                vidx = np.where(valid)[0] if valid.size else np.arange(min(tcps.shape[0], normals.shape[0]))
                if vidx.size:
                    # sample ~frame_step_max
                    step = max(1, int(round(max(1, vidx.size // int(cfg["frame_step_max"])))))
                    sel = vidx[::step]
                    out.frames = {
                        "origins": tcps[sel],
                        "z_dirs": normals[sel],
                        "x_dirs": None,
                        "scale_mm": float(cfg["frame_scale_mm"]),
                        "step": int(step),
                    }

        except Exception:
            _LOG.exception("render_for_side normals/frames failed (%s)", side)

        # cache for potential rebuilds
        self._cache[str(side)] = {
            "cfg": cfg,
            "vis": vis,
        }

        return out
