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
    def from_dict(d: Any) -> "EvalResult":
        if not isinstance(d, dict):
            d = {}
        return EvalResult(
            score=float(d.get("score", 0.0)),
            metrics=dict(d.get("metrics") or {}),
            details=dict(d.get("details") or {}),
        )

    def to_flat_metrics(self, *, prefix: str = "") -> Dict[str, Any]:
        """
        Für RunResult.metrics (flach, UI-friendly).
        Beispiel:
          {"score": 97.1, "max_pos_err_mm": 0.8, ...}
        Optional prefix (z.B. "validate_").
        """
        p = str(prefix or "")
        out: Dict[str, Any] = {}
        out[f"{p}score" if p else "score"] = float(self.score)
        # metrics ist schon flach; details bleibt bewusst raus (kannst du separat ablegen)
        for k, v in (self.metrics or {}).items():
            out[f"{p}{k}"] = v
        return out
