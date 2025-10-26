# Spraycoater/src/app/tabs/recipe/path_builder.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, Optional, Sequence, Union
import numpy as np

@dataclass
class PathData:
    points_mm: np.ndarray            # Nx3
    normals: Optional[np.ndarray]    # Nx3 oder None
    tangents: Optional[np.ndarray]   # Nx3 oder None
    meta: Dict[str, Any]

    def arclength_mm(self) -> np.ndarray:
        P = self.points_mm
        if len(P) < 2:
            return np.zeros((len(P),), dtype=float)
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

class PathBuilder:
    """
    Liest Pfadkoordinaten (mm) 1:1 aus dem Rezept oder aus direkten Arrays.
    Keine Veränderungen am Pfad, lediglich Tangentenberechnung (optional).
    Bevorzugte Felder im Rezept (in dieser Reihenfolge):
        - path.points_mm
        - path.polyline_mm
        - path.surface_points_mm (+ optional path.surface_normals)
    """

    @staticmethod
    def from_recipe(recipe: Any, *, substrate_center_mm: Sequence[float] = (0.0, 0.0, 0.0)) -> PathData:
        p = PathBuilder._extract_path_dict(recipe)
        pts = (p.get("points_mm")
               or p.get("polyline_mm")
               or p.get("surface_points_mm")
               or [])
        points_mm = np.asarray(pts, dtype=float).reshape(-1, 3)

        normals = None
        if "surface_normals" in p:
            normals = np.asarray(p["surface_normals"], dtype=float).reshape(-1, 3)
            if len(normals) != len(points_mm):
                raise ValueError("surface_normals Länge passt nicht zu den Punkten.")

        tangents = PathBuilder._tangents(points_mm) if len(points_mm) >= 2 else None
        return PathData(
            points_mm=points_mm,
            normals=normals,
            tangents=tangents,
            meta={
                "substrate_center_mm": list(map(float, substrate_center_mm)),
                "source": "recipe",
            },
        )

    @staticmethod
    def from_points(points_mm: Union[np.ndarray, Sequence[Sequence[float]]],
                    *,
                    normals: Optional[Union[np.ndarray, Sequence[Sequence[float]]]] = None,
                    substrate_center_mm: Sequence[float] = (0.0, 0.0, 0.0)) -> PathData:
        P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
        N = None if normals is None else np.asarray(normals, dtype=float).reshape(-1, 3)
        if N is not None and len(N) != len(P):
            raise ValueError("Normals-Länge passt nicht zur Punktzahl.")
        T = PathBuilder._tangents(P) if len(P) >= 2 else None
        return PathData(
            points_mm=P,
            normals=N,
            tangents=T,
            meta={"substrate_center_mm": list(map(float, substrate_center_mm)),
                  "source": "points"},
        )

    # ----- helpers -----
    @staticmethod
    def _extract_path_dict(recipe: Any) -> Dict[str, Any]:
        if hasattr(recipe, "path"):
            return dict(getattr(recipe, "path") or {})
        if isinstance(recipe, dict):
            return dict(recipe.get("path", {}) or {})
        raise TypeError("PathBuilder.from_recipe: recipe hat kein .path Feld.")

    @staticmethod
    def _tangents(P: np.ndarray) -> np.ndarray:
        if len(P) < 2:
            return np.tile(np.array([1.0, 0.0, 0.0]), (len(P), 1))
        T = np.zeros_like(P)
        T[1:-1] = P[2:] - P[:-2]
        T[0]    = P[1] - P[0]
        T[-1]   = P[-1] - P[-2]
        n = np.linalg.norm(T, axis=1, keepdims=True) + 1e-12
        return T / n
