# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab3d/overlays.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import logging
import numpy as np
import pyvista as pv

_LOG = logging.getLogger("tabs.recipe.preview.overlays")

Bounds = Tuple[float, float, float, float, float, float]


@dataclass
class OverlayOut:
    side: str

    # mask (PathBuilder output, ray start polyline)
    mask_poly: Optional[pv.PolyData] = None

    # final path (postprocessed TCP polyline)
    path_poly: Optional[pv.PolyData] = None

    # raycast visuals
    rays_hit_poly: Optional[pv.PolyData] = None
    rays_miss_poly: Optional[pv.PolyData] = None

    # tcp points (optional debug/inspection)
    tcp_poly: Optional[pv.PolyData] = None

    # normals / frames
    normals_poly: Optional[pv.PolyData] = None
    tcp_x_poly: Optional[pv.PolyData] = None
    tcp_y_poly: Optional[pv.PolyData] = None
    tcp_z_poly: Optional[pv.PolyData] = None

    visibility: Dict[str, bool] = None  # type: ignore[assignment]


def _sanitize_points(P: Any) -> np.ndarray:
    A = np.asarray(P, dtype=float).reshape(-1, 3)
    if A.size == 0:
        return np.zeros((0, 3), dtype=float)
    m = np.isfinite(A).all(axis=1)
    return A[m]


def _polyline_from_points(P: Any) -> Optional[pv.PolyData]:
    pts = _sanitize_points(P)
    if pts.shape[0] < 2:
        return None
    return pv.lines_from_points(pts, close=False)


def _points_poly(P: Any) -> Optional[pv.PolyData]:
    pts = _sanitize_points(P)
    if pts.shape[0] == 0:
        return None
    return pv.PolyData(pts)


def _unit(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if not np.isfinite(n) or n < eps:
        return np.array([1.0, 0.0, 0.0], dtype=float)
    return v / n


def _decimate_indices(n: int, max_keep: int) -> np.ndarray:
    if n <= 0:
        return np.zeros((0,), dtype=int)
    if max_keep <= 0 or n <= max_keep:
        return np.arange(n, dtype=int)
    return np.linspace(0, n - 1, num=max_keep, dtype=int)


def _segments(starts: np.ndarray, dirs: np.ndarray, length: float, max_keep: int) -> Optional[pv.PolyData]:
    S = _sanitize_points(starts)
    D = np.asarray(dirs, dtype=float).reshape(-1, 3)
    n = min(S.shape[0], D.shape[0])
    if n <= 0:
        return None

    S = S[:n]
    D = D[:n]

    idx = _decimate_indices(n, max_keep)
    S = S[idx]
    D = D[idx]

    # normalize D
    Dn = np.zeros_like(D)
    for i in range(D.shape[0]):
        Dn[i] = _unit(D[i])

    E = S + Dn * float(length)

    pts = np.vstack([S, E])
    m = S.shape[0]
    lines = np.empty((m, 3), dtype=np.int64)
    lines[:, 0] = 2
    lines[:, 1] = np.arange(0, m, dtype=np.int64)
    lines[:, 2] = np.arange(m, 2 * m, dtype=np.int64)

    poly = pv.PolyData(pts)
    poly.lines = lines.reshape(-1)
    return poly


def _tangents_from_path(P: np.ndarray) -> np.ndarray:
    """Compute per-point tangent directions for a polyline."""
    A = _sanitize_points(P)
    n = int(A.shape[0])
    if n <= 1:
        return np.zeros((0, 3), dtype=float)

    T = np.zeros((n, 3), dtype=float)
    for i in range(n):
        if i == 0:
            d = A[1] - A[0]
        elif i == n - 1:
            d = A[n - 1] - A[n - 2]
        else:
            d = A[i + 1] - A[i - 1]
        T[i] = _unit(d)
    return T


def _get_bool(cfg: Dict[str, Any], key: str, default: bool) -> bool:
    if key not in cfg:
        return bool(default)
    return bool(cfg[key])


def _get_int(cfg: Dict[str, Any], key: str, default: int) -> int:
    if key not in cfg:
        return int(default)
    return int(cfg[key])


def _get_float(cfg: Dict[str, Any], key: str, default: float) -> float:
    if key not in cfg:
        return float(default)
    return float(cfg[key])


def _normalize_overlay_cfg(cfg: Any) -> Dict[str, Any]:
    """
    STRICT keys:
      visibility: {mask,path,hits,misses,tcp,normals,tcp_axes}
      normals_len_mm, normals_max
      tcp_axes_len_mm, tcp_axes_max
    """
    if not isinstance(cfg, dict):
        cfg = {}

    vis_in = cfg.get("visibility", {})
    if not isinstance(vis_in, dict):
        vis_in = {}

    visibility = {
        "mask": _get_bool(vis_in, "mask", True),
        "path": _get_bool(vis_in, "path", True),
        "hits": _get_bool(vis_in, "hits", True),
        "misses": _get_bool(vis_in, "misses", True),
        "tcp": _get_bool(vis_in, "tcp", True),
        "normals": _get_bool(vis_in, "normals", True),
        "tcp_axes": _get_bool(vis_in, "tcp_axes", True),
    }

    return {
        "visibility": visibility,
        "normals_len_mm": _get_float(cfg, "normals_len_mm", 10.0),
        "normals_max": _get_int(cfg, "normals_max", 250),
        "tcp_axes_len_mm": _get_float(cfg, "tcp_axes_len_mm", 12.0),
        "tcp_axes_max": _get_int(cfg, "tcp_axes_max", 80),
    }


class OverlayRenderer:
    def render_for_side(
        self,
        *,
        side: str,
        mask_points_world_mm: np.ndarray,
        path_points_world_mm: Optional[np.ndarray],
        tcp_points_world_mm: Optional[np.ndarray],
        raycast_result: Any,
        hit_poly: Optional[pv.PolyData],
        miss_poly: Optional[pv.PolyData],
        overlay_cfg: Any,
    ) -> OverlayOut:
        cfg = _normalize_overlay_cfg(overlay_cfg)
        vis = dict(cfg["visibility"])

        out = OverlayOut(side=str(side), visibility=vis)

        # mask/path
        if vis["mask"]:
            out.mask_poly = _polyline_from_points(mask_points_world_mm)
        if vis["path"] and path_points_world_mm is not None:
            out.path_poly = _polyline_from_points(path_points_world_mm)

        # hits/misses (points from raycaster)
        if vis["hits"]:
            out.rays_hit_poly = hit_poly
        if vis["misses"]:
            out.rays_miss_poly = miss_poly

        # tcp points ONLY (no tcp_line; Path draws the line)
        if vis["tcp"] and tcp_points_world_mm is not None:
            out.tcp_poly = _points_poly(tcp_points_world_mm)

        # normals derived from raycast_result (hits + normals)
        if vis["normals"]:
            try:
                valid = np.asarray(getattr(raycast_result, "valid", None), dtype=bool).reshape(-1)
                hits = np.asarray(getattr(raycast_result, "hit_mm", None), dtype=float).reshape(-1, 3)
                norms = np.asarray(getattr(raycast_result, "normal", None), dtype=float).reshape(-1, 3)

                n = min(valid.shape[0], hits.shape[0], norms.shape[0])
                if n > 0:
                    valid = valid[:n]
                    hits = hits[:n]
                    norms = norms[:n]
                    idx = np.where(valid)[0]
                    if idx.size:
                        out.normals_poly = _segments(
                            hits[idx],
                            norms[idx],
                            float(cfg["normals_len_mm"]),
                            int(cfg["normals_max"]),
                        )
            except Exception:
                _LOG.exception("normals overlay failed (side=%s)", side)

        # TCP axes (local KS) at TCP points:
        # X = tangent along path, Z = surface normal, Y = Z x X
        if vis["tcp_axes"] and tcp_points_world_mm is not None:
            try:
                tcps = _sanitize_points(tcp_points_world_mm)
                if tcps.shape[0] >= 2:
                    # normals from raycast_result (prefer), else fallback
                    norms = None
                    try:
                        valid = np.asarray(getattr(raycast_result, "valid", None), dtype=bool).reshape(-1)
                        nrm = np.asarray(getattr(raycast_result, "normal", None), dtype=float).reshape(-1, 3)
                        tcp_raw = np.asarray(getattr(raycast_result, "tcp_mm", None), dtype=float).reshape(-1, 3)
                        m = min(valid.shape[0], nrm.shape[0], tcp_raw.shape[0])
                        if m > 0:
                            valid = valid[:m]
                            nrm = nrm[:m]
                            tcp_raw = tcp_raw[:m]
                            idxv = np.where(valid)[0]
                            if idxv.size:
                                norms = nrm[idxv]
                                k = min(int(tcps.shape[0]), int(norms.shape[0]))
                                tcps = tcps[:k]
                                norms = norms[:k]
                    except Exception:
                        norms = None

                    if norms is None or norms.shape[0] != tcps.shape[0]:
                        norms = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (tcps.shape[0], 1))

                    T = _tangents_from_path(tcps)
                    k = min(int(tcps.shape[0]), int(T.shape[0]), int(norms.shape[0]))
                    if k >= 2:
                        tcps = tcps[:k]
                        T = T[:k]
                        Z = np.zeros_like(norms[:k])
                        for i in range(k):
                            Z[i] = _unit(norms[i])

                        X = np.zeros_like(T)
                        Y = np.zeros_like(T)
                        for i in range(k):
                            x = _unit(T[i])
                            z = _unit(Z[i])
                            x = _unit(x - float(np.dot(x, z)) * z)
                            y = _unit(np.cross(z, x))
                            X[i], Y[i], Z[i] = x, y, z

                        out.tcp_x_poly = _segments(tcps, X, float(cfg["tcp_axes_len_mm"]), int(cfg["tcp_axes_max"]))
                        out.tcp_y_poly = _segments(tcps, Y, float(cfg["tcp_axes_len_mm"]), int(cfg["tcp_axes_max"]))
                        out.tcp_z_poly = _segments(tcps, Z, float(cfg["tcp_axes_len_mm"]), int(cfg["tcp_axes_max"]))
            except Exception:
                _LOG.exception("tcp_axes overlay failed (side=%s)", side)

        return out
