# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, Iterable, List, Tuple
import numpy as np

from .path_data import PathData
from .base_path import BasePath
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

        # 2) Optional air-move extension (legacy / opt-in)
        if include_airmoves:
            pd = PathBuilder._add_airmove_endpoints(pd, p)

        # 3) Meta (globals)
        return PathBuilder._with_globals_meta(pd, globals_params)

    # ---------------------------
    # Helpers
    # ---------------------------

    @staticmethod
    def _resolve_params(recipe: Any, g: Dict, step: float, max_pts: int) -> Tuple[Dict, float, int]:
        eff = dict(g or {})
        rec = (recipe.get("parameters", {}) if isinstance(recipe, dict) else getattr(recipe, "parameters", {})) or {}
        
        if "sample_step_mm" in rec: step = float(rec["sample_step_mm"])
        if "max_points" in rec: max_pts = int(rec["max_points"])
        
        for k in ("stand_off_mm", "max_angle_deg"):
            if k in rec: eff[k] = rec[k]
            
        return eff, float(step), int(max_pts)

    @staticmethod
    def _with_globals_meta(pd: PathData, g: Dict) -> PathData:
        meta = dict(pd.meta or {})
        for k in ("stand_off_mm", "max_angle_deg"):
            if k in g: meta[k] = float(g[k])
        return PathData(points_mm=pd.points_mm, meta=meta)

    @staticmethod
    def _extract_path_for_side(recipe: Any, side: str) -> Dict[str, Any]:
        pbs = dict(recipe.get("paths_by_side") or {}) if isinstance(recipe, dict) else dict(getattr(recipe, "paths_by_side", {}) or {})
        return dict(pbs.get(side) or {})

    @staticmethod
    def _add_airmove_endpoints(pd: PathData, p: Dict[str, Any]) -> PathData:
        P = pd.points_mm
        if P.shape[0] < 2: return pd
        
        pre = max(0.0, float(p.get("predispense_offset_mm", 0.0)))
        post = max(0.0, float(p.get("retreat_offset_mm", 0.0)))
        
        if pre > 0:
            v = P[0]-P[1]
            P = np.vstack([(P[0] + pre * v/np.linalg.norm(v))[None,:], P])
        if post > 0:
            v = P[-1]-P[-2]
            P = np.vstack([P, (P[-1] + post * v/np.linalg.norm(v))[None,:]])
            
        return PathData(points_mm=P, meta=pd.meta)