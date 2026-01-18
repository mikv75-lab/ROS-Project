# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List
import numpy as np

@dataclass
class PathData:
    """
    Standardisiertes Ergebnis einer Pfad-Generierung (lokales Koordinatensystem, mm).
    """
    points_mm: np.ndarray  # shape: (N, 3)
    meta: Dict[str, Any]

    def arclength_mm(self) -> np.ndarray:
        P = np.asarray(self.points_mm, dtype=float).reshape(-1, 3)
        if P.shape[0] < 2:
            return np.zeros((P.shape[0],), dtype=float)
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return np.concatenate([[0.0], np.cumsum(d)])

    def as_list_mm(self) -> List[List[float]]:
        P = np.asarray(self.points_mm, dtype=float).reshape(-1, 3)
        return P.tolist()