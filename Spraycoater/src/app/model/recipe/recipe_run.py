# app/model/recipe/run_result.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class RunResult:
    """
    Ergebnis eines 'Runs' (Validate / Optimize / Execute).

    Minimal-Contract:
      - role: validate | optimize | execute
      - ok: bool
      - message: kurze Fehlermeldung oder Status
      - meta: beliebige Zusatzinfos (planner_id, pipeline, timings, etc.)
      - traj: optional: serialisierte Trajektorie (dict) oder pointer/id
      - metrics: optional: Eval/Metriken vom Evaluator (dict)
    """

    role: str
    ok: bool = False
    message: str = ""

    # freie Meta-Infos (Plannername, Pipeline, request params, durations, etc.)
    meta: Dict[str, Any] = field(default_factory=dict)

    # optional: raw data
    traj: Optional[Dict[str, Any]] = None

    # optional: evaluation result summary (score, metrics)
    metrics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "role": self.role,
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "meta": dict(self.meta or {}),
            "traj": self.traj,
            "metrics": dict(self.metrics or {}),
        }

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "RunResult":
        return RunResult(
            role=str(d.get("role") or ""),
            ok=bool(d.get("ok", False)),
            message=str(d.get("message") or ""),
            meta=dict(d.get("meta") or {}),
            traj=d.get("traj", None),
            metrics=dict(d.get("metrics") or {}),
        )
