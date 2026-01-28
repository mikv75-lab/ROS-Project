# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, Tuple, List
import numpy as np

from .path_data import PathData
from .path_factory import PathFactory


class PathBuilder:
    """Erzeugt Pfade (mm) aus einer Path-Definition je Side (Facade)."""

    def __new__(cls, *args: Any, **kwargs: Any):
        return super().__new__(cls)

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        pass

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
        include_airmoves: bool = False,
    ) -> PathData:
        globals_params, sample_step_mm, max_points = PathBuilder._resolve_params(
            recipe, globals_params, sample_step_mm, max_points
        )

        p = PathBuilder._extract_path_for_side(recipe, side)

        # 1) Base path (local frame)
        inst = PathFactory.from_path_dict(p)
        pd = inst.to_pathdata(step_mm=float(sample_step_mm), max_points=int(max_points))

        # 2) Fix meander U-turns / extend shorter lanes (auto-detect)
        pd = PathBuilder._maybe_fix_meander_uturns(pd, p)

        # 3) Optional air-move extension (legacy / opt-in)
        if include_airmoves:
            pd = PathBuilder._add_airmove_endpoints(pd, p)

        # 4) Meta (globals)
        return PathBuilder._with_globals_meta(pd, globals_params)

    # ---------------------------
    # Helpers
    # ---------------------------

    @staticmethod
    def _resolve_params(recipe: Any, g: Dict, step: float, max_pts: int) -> Tuple[Dict, float, int]:
        eff = dict(g or {})
        rec = (recipe.get("parameters", {}) if isinstance(recipe, dict) else getattr(recipe, "parameters", {})) or {}

        if "sample_step_mm" in rec:
            step = float(rec["sample_step_mm"])
        if "max_points" in rec:
            max_pts = int(rec["max_points"])

        for k in ("stand_off_mm", "max_angle_deg"):
            if k in rec:
                eff[k] = rec[k]

        return eff, float(step), int(max_pts)

    @staticmethod
    def _with_globals_meta(pd: PathData, g: Dict) -> PathData:
        meta = dict(pd.meta or {})
        for k in ("stand_off_mm", "max_angle_deg"):
            if k in g:
                meta[k] = float(g[k])
        return PathData(points_mm=pd.points_mm, meta=meta)

    @staticmethod
    def _extract_path_for_side(recipe: Any, side: str) -> Dict[str, Any]:
        pbs = dict(recipe.get("paths_by_side") or {}) if isinstance(recipe, dict) else dict(getattr(recipe, "paths_by_side", {}) or {})
        return dict(pbs.get(side) or {})

    @staticmethod
    def _add_airmove_endpoints(pd: PathData, p: Dict[str, Any]) -> PathData:
        P = pd.points_mm
        if P.shape[0] < 2:
            return pd

        pre = max(0.0, float(p.get("predispense_offset_mm", 0.0)))
        post = max(0.0, float(p.get("retreat_offset_mm", 0.0)))

        if pre > 0:
            v = P[0] - P[1]
            n = float(np.linalg.norm(v))
            if n > 1e-12:
                P = np.vstack([(P[0] + pre * v / n)[None, :], P])

        if post > 0:
            v = P[-1] - P[-2]
            n = float(np.linalg.norm(v))
            if n > 1e-12:
                P = np.vstack([P, (P[-1] + post * v / n)[None, :]])

        return PathData(points_mm=P, meta=pd.meta)

    # ---------------------------
    # Meander U-turn fixing
    # ---------------------------

    @staticmethod
    def _maybe_fix_meander_uturns(pd: PathData, p: Dict[str, Any]) -> PathData:
        """
        Heuristik:
        - Aktiviert nur bei meander/raster/serpentine-ähnlichen Pfaden
        - Optional über p["fix_u_turns"]=True/False steuerbar
        """
        if pd.points_mm is None:
            return pd

        P = pd.points_mm
        if P.shape[0] < 4:
            return pd

        # Explicit override
        if "fix_u_turns" in p and not bool(p["fix_u_turns"]):
            return pd

        # Auto-detect by type/kind/name
        t = str(p.get("type") or p.get("kind") or p.get("name") or "").lower()
        is_meanderish = any(k in t for k in ("meander", "raster", "zig", "zag", "serp", "lines", "line"))
        if not is_meanderish and "fix_u_turns" not in p:
            return pd

        tol_mm = float(p.get("fix_u_turns_tol_mm", 1e-3))  # 1 µm default-ish; override if needed
        ratio = float(p.get("fix_u_turns_ratio", 3.0))      # how "vertical" a lane-change must be
        return PathBuilder._fix_meander_uturns(pd, tol_mm=tol_mm, ratio=ratio)

    @staticmethod
    def _fix_meander_uturns(pd: PathData, *, tol_mm: float, ratio: float) -> PathData:
        """
        Ziel:
        - Lane-Change-Abschnitte (Sekundärachse dominiert) finden
        - In diesen Turn-Zonen Primärachse konstant auf globales min/max setzen
        - Wenn eine Lane "kürzer" endet, wird bis zum Extrem verlängert
        """
        P0 = np.asarray(pd.points_mm, dtype=float)
        n = P0.shape[0]
        if n < 4:
            return pd

        # Determine primary axis (the "lane" direction): max range (use X/Y only)
        ranges = np.ptp(P0[:, :2], axis=0)
        prim = int(np.argmax(ranges))
        sec = 1 - prim

        prim_vals = P0[:, prim]
        prim_min = float(np.min(prim_vals))
        prim_max = float(np.max(prim_vals))

        # Steps and "vertical-ish" detection
        d = P0[1:, :2] - P0[:-1, :2]
        dprim = np.abs(d[:, prim])
        dsec = np.abs(d[:, sec])

        # lane-change step: secondary dominates
        verticalish = dsec > (ratio * (dprim + 1e-12))

        # Build contiguous blocks of verticalish steps
        blocks: List[Tuple[int, int]] = []
        i = 0
        while i < verticalish.shape[0]:
            if not verticalish[i]:
                i += 1
                continue
            j = i
            while j + 1 < verticalish.shape[0] and verticalish[j + 1]:
                j += 1
            blocks.append((i, j))  # step indices
            i = j + 1

        if not blocks:
            return pd

        out: List[np.ndarray] = [P0[0].copy()]

        def _append(pt: np.ndarray) -> None:
            if np.linalg.norm(out[-1] - pt) <= 1e-12:
                return
            out.append(pt)

        ptr = 0
        for (s_step, e_step) in blocks:
            s_pt = s_step
            e_pt = e_step + 1

            # Emit points up to s_pt (inclusive)
            while ptr + 1 <= s_pt:
                ptr += 1
                _append(P0[ptr].copy())

            # Determine lane direction BEFORE block: look back for non-verticalish primary motion
            look = s_step - 1
            dir_sign = 0.0
            for k in range(look, max(-1, look - 20), -1):
                if k < 0:
                    break
                if verticalish[k]:
                    continue
                dd = P0[k + 1, prim] - P0[k, prim]
                if abs(dd) > 1e-9:
                    dir_sign = float(np.sign(dd))
                    break

            if dir_sign == 0.0:
                dd = P0[s_pt, prim] - P0[max(0, s_pt - 1), prim]
                dir_sign = float(np.sign(dd)) if abs(dd) > 1e-9 else 1.0

            target = prim_max if dir_sign >= 0 else prim_min

            # 1) Extend lane end to target if needed
            last = out[-1].copy()
            if abs(last[prim] - target) > tol_mm:
                ext = last.copy()
                ext[prim] = target
                _append(ext)

            # 2) Force all points in the turn block (and the first point after) to have prim=target
            for k in range(s_pt + 1, e_pt + 1):
                if k >= n:
                    break
                q = P0[k].copy()
                if abs(q[prim] - target) > tol_mm:
                    q[prim] = target
                _append(q)

            ptr = min(e_pt, n - 1)

        # Emit remaining tail
        while ptr + 1 < n:
            ptr += 1
            _append(P0[ptr].copy())

        P1 = np.asarray(out, dtype=float)
        return PathData(points_mm=P1, meta=pd.meta)
