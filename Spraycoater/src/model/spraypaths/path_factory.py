# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Dict, Type, Any
from .base_path import BasePath
from .path_strategies import (
    PointsPath,
    MeanderPlanePath,
    SpiralPlanePath,
    SpiralCylinderPath,
    PerimeterFollowPlanePath,
    PolyhelixPyramidPath,
    PolyhelixCubePath,
)

class PathFactory:
    _REGISTRY: Dict[str, Type[BasePath]] = {
        "path.meander.plane": MeanderPlanePath,
        "path.spiral.plane": SpiralPlanePath,
        "path.spiral.cylinder": SpiralCylinderPath,
        "path.perimeter_follow.plane": PerimeterFollowPlanePath,
        "path.polyhelix.pyramid": PolyhelixPyramidPath,
        "path.polyhelix.cube": PolyhelixCubePath,
        "points": PointsPath,
    }

    @staticmethod
    def from_path_dict(p: Dict[str, Any]) -> BasePath:
        if (p.get("points_mm") is not None) or (p.get("polyline_mm") is not None):
            return PointsPath(p)

        raw_type = str(p.get("type") or "").strip().lower()
        cls = PathFactory._REGISTRY.get(raw_type)
        if cls is None:
            raise ValueError(f"PathFactory: unknown type {raw_type!r}")
        return cls(p)