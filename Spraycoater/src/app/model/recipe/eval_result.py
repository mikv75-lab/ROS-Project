# app/model/recipe/eval_result.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass
class EvalResult:
    """
    Bewertungs-Ergebnis (gegen Referenz/compiled_path).

    score: 0..100 (100 = perfekt)
    metrics: flache Kennzahlen (mm/deg/ratio)
    details: strukturierte Extras (per-side, per-segment, histos, etc.)
    """

    score: float = 0.0
    metrics: Dict[str, Any] = field(default_factory=dict)
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "score": float(self.score),
            "metrics": dict(self.metrics or {}),
            "details": dict(self.details or {}),
        }

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "EvalResult":
        return EvalResult(
            score=float(d.get("score", 0.0)),
            metrics=dict(d.get("metrics") or {}),
            details=dict(d.get("details") or {}),
        )
