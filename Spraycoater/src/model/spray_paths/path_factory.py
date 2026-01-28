# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Dict, Type, Any, Optional, Iterable

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
    """
    Factory for path strategy instances.

    Strict contract:
      - A path dict MUST have either:
          a) points_mm / polyline_mm  -> PointsPath
          b) a non-empty 'type'       -> resolved via registry/aliases
      - Only limited legacy support is provided (explicit alias map).
      - If 'type' is missing/empty, we only infer a type if it is unambiguous.
        Otherwise we raise with a helpful error.

    This avoids silent fallbacks while still supporting old recipe formats.
    """

    _REGISTRY: Dict[str, Type[BasePath]] = {
        "path.meander.plane": MeanderPlanePath,
        "path.spiral.plane": SpiralPlanePath,
        "path.spiral.cylinder": SpiralCylinderPath,
        "path.perimeter_follow.plane": PerimeterFollowPlanePath,
        "path.polyhelix.pyramid": PolyhelixPyramidPath,
        "path.polyhelix.cube": PolyhelixCubePath,
        "points": PointsPath,
    }

    # Explicit legacy aliases -> canonical type ids
    _ALIASES: Dict[str, str] = {
        # common shortened/older names
        "meander.plane": "path.meander.plane",
        "spiral.plane": "path.spiral.plane",
        "spiral.cylinder": "path.spiral.cylinder",
        "perimeter_follow.plane": "path.perimeter_follow.plane",
        "perimeterfollow.plane": "path.perimeter_follow.plane",
        "polyhelix.pyramid": "path.polyhelix.pyramid",
        "polyhelix.cube": "path.polyhelix.cube",

        # helix naming used in older versions
        "helix.pyramid": "path.polyhelix.pyramid",
        "helix.cube": "path.polyhelix.cube",
        "path.helix.pyramid": "path.polyhelix.pyramid",
        "path.helix.cube": "path.polyhelix.cube",

        # underscore variants
        "path_polyhelix_pyramid": "path.polyhelix.pyramid",
        "path_polyhelix_cube": "path.polyhelix.cube",
        "polyhelix_pyramid": "path.polyhelix.pyramid",
        "polyhelix_cube": "path.polyhelix.cube",
    }

    @staticmethod
    def _norm_type(raw: Any) -> str:
        return str(raw or "").strip().lower()

    @staticmethod
    def _fmt_keys(p: Dict[str, Any], *, max_keys: int = 32) -> str:
        keys = sorted([str(k) for k in p.keys()])
        if len(keys) > max_keys:
            keys = keys[:max_keys] + ["..."]
        return ", ".join(keys)

    @staticmethod
    def _infer_type_if_unambiguous(p: Dict[str, Any]) -> Optional[str]:
        """
        Infer only when it is REALLY unambiguous.

        Cube helix signature (current PolyhelixCubePath):
          - edge_len_mm (cube) or base_edge_len_mm (pyramid)
          - height_mm
          - pitch_mm (usually)
          - plus various optional helix params

        Pyramid helix signature:
          - base_edge_len_mm OR base_polygon_sides
          - height_mm
          - pitch_mm (usually)
        """
        keys = set(p.keys())

        # If a path contains points/polyline, we never infer - handled earlier.
        if ("points_mm" in keys) or ("polyline_mm" in keys):
            return None

        # Strong indicators
        has_height = "height_mm" in keys
        has_pitch = "pitch_mm" in keys or "dz_mm" in keys  # dz_mm is also helix-ish
        has_cube_edge = "edge_len_mm" in keys
        has_pyr_edge = "base_edge_len_mm" in keys or "base_polygon_sides" in keys

        if has_height and (has_pitch or has_cube_edge or has_pyr_edge):
            if has_cube_edge and not has_pyr_edge:
                return "path.polyhelix.cube"
            if has_pyr_edge and not has_cube_edge:
                return "path.polyhelix.pyramid"

        # Not unambiguous -> no inference.
        return None

    @staticmethod
    def from_path_dict(p: Dict[str, Any]) -> BasePath:
        if not isinstance(p, dict):
            raise TypeError(f"PathFactory: path must be dict, got {type(p)}")

        # PointsPath shortcut
        if (p.get("points_mm") is not None) or (p.get("polyline_mm") is not None):
            return PointsPath(p)

        raw_type = PathFactory._norm_type(p.get("type"))

        # Alias resolution
        if raw_type in PathFactory._ALIASES:
            raw_type = PathFactory._ALIASES[raw_type]

        # If empty/missing, attempt *unambiguous* inference (no silent guessing)
        if raw_type == "":
            inferred = PathFactory._infer_type_if_unambiguous(p)
            if inferred:
                raw_type = inferred

        cls = PathFactory._REGISTRY.get(raw_type)
        if cls is None:
            known = ", ".join(sorted(PathFactory._REGISTRY.keys()))
            keys = PathFactory._fmt_keys(p)
            raise ValueError(
                "PathFactory: unknown or missing type.\n"
                f"  type={raw_type!r}\n"
                f"  keys=[{keys}]\n"
                f"  known_types=[{known}]\n"
                "Fix: ensure each path dict has a non-empty 'type' "
                "(e.g. 'path.polyhelix.cube' / 'path.polyhelix.pyramid')."
            )

        return cls(p)
