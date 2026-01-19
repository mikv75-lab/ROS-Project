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
    STRICT keys (UI):
      visibility: {mask,path,hits,misses,tcp,normals}
      normals_len_mm, normals_max
      tcp_axes_len_mm, tcp_axes_max   (axes are controlled by visibility['tcp'])
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
    }

    return {
        "visibility": visibility,
        "normals_len_mm": _get_float(cfg, "normals_len_mm", 10.0),
        "normals_max": _get_int(cfg, "normals_max", 250),
        "tcp_axes_len_mm": _get_float(cfg, "tcp_axes_len_mm", 12.0),
        "tcp_axes_max": _get_int(cfg, "tcp_axes_max", 80),
    }


def _fixed_axes_from_z(z_axis: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Build a stable right-handed local frame from Z only:
      Z = z_axis (normalized)
      X chosen deterministically from a global "up" reference (not tangent!)
      Y = Z x X
    This avoids rotation around Z for spiral/closed paths.
    """
    z = _unit(z_axis)
    up = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(up, z))) > 0.95:
        up = np.array([1.0, 0.0, 0.0], dtype=float)
    x = _unit(np.cross(up, z))
    y = _unit(np.cross(z, x))
    return x, y, z


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
        # X/Y/Z are built stably from Z only (no tangents) and are toggled by vis["tcp"].
        #
        # IMPORTANT FIX:
        #   tcp_points_world_mm may include predispense/retreat (N+2 points),
        #   while raycast normals are available only for main hits (N normals).
        #   We must NOT truncate tcps; instead pad normals to match tcps length.
        if vis["tcp"] and tcp_points_world_mm is not None:
            try:
                tcps = _sanitize_points(tcp_points_world_mm)
                if tcps.shape[0] >= 1:
                    norms: Optional[np.ndarray] = None

                    # Prefer normals from raycast_result (aligned to valid hits)
                    try:
                        valid = np.asarray(getattr(raycast_result, "valid", None), dtype=bool).reshape(-1)
                        nrm = np.asarray(getattr(raycast_result, "normal", None), dtype=float).reshape(-1, 3)

                        m = min(valid.shape[0], nrm.shape[0])
                        if m > 0:
                            valid = valid[:m]
                            nrm = nrm[:m]
                            idxv = np.where(valid)[0]
                            if idxv.size:
                                norms = nrm[idxv]
                    except Exception:
                        norms = None

                    # Align normals to tcps length WITHOUT truncating tcps.
                    if norms is not None and norms.shape[0] > 0:
                        nt = int(tcps.shape[0])
                        nn = int(norms.shape[0])

                        if nn == nt:
                            pass
                        elif nn == nt - 2:
                            # tcps includes [predispense] + main + [retreat]
                            norms = np.vstack([norms[0:1], norms, norms[-1:]])
                        elif nn < nt:
                            # pad end by repeating last normal
                            extra = nt - nn
                            norms = np.vstack([norms, np.repeat(norms[-1:, :], extra, axis=0)])
                        else:
                            norms = norms[:nt]
                    else:
                        norms = None

                    # Fallback: constant Z-up if normals missing/mismatch
                    if norms is None or norms.shape[0] != tcps.shape[0]:
                        norms = np.tile(np.array([[0.0, 0.0, 1.0]], dtype=float), (tcps.shape[0], 1))

                    k = int(tcps.shape[0])
                    X = np.zeros((k, 3), dtype=float)
                    Y = np.zeros((k, 3), dtype=float)
                    Z = np.zeros((k, 3), dtype=float)

                    for i in range(k):
                        x, y, z = _fixed_axes_from_z(norms[i])
                        X[i], Y[i], Z[i] = x, y, z

                    out.tcp_x_poly = _segments(tcps, X, float(cfg["tcp_axes_len_mm"]), int(cfg["tcp_axes_max"]))
                    out.tcp_y_poly = _segments(tcps, Y, float(cfg["tcp_axes_len_mm"]), int(cfg["tcp_axes_max"]))
                    out.tcp_z_poly = _segments(tcps, Z, float(cfg["tcp_axes_len_mm"]), int(cfg["tcp_axes_max"]))
            except Exception:
                _LOG.exception("tcp_axes overlay failed (side=%s)", side)

        return out
