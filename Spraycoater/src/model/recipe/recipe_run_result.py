# app/model/recipe/run_result.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class RunResult:
    """
    Ergebniscontainer für einen Prozesslauf.

    Design-Entscheid (nach deinen Vorgaben):
    - RunResult enthält NUR executed Trajectories (gesamt + pro Segment).
    - Eval-Ergebnisse werden in metrics abgelegt (z.B. metrics["eval_total"], metrics["eval_by_segment"]).
    """

    ROLE_VALIDATE = "validate"
    ROLE_OPTIMIZE = "optimize"
    ROLE_EXECUTE = "execute"

    role: str
    ok: bool = False
    message: str = ""

    meta: Dict[str, Any] = field(default_factory=dict)
    metrics: Dict[str, Any] = field(default_factory=dict)

    # Executed only
    executed_traj: Optional[Dict[str, Any]] = None
    executed_by_segment: Dict[str, Any] = field(default_factory=dict)

    def set_eval(
        self,
        *,
        eval_total: Optional[Dict[str, Any]] = None,
        eval_by_segment: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Convenience: Eval-Ergebnisse strukturiert in metrics ablegen.

        Erwartetes Format:
          eval_total: EvalResult.to_dict()
          eval_by_segment: {SEG: EvalResult.to_dict(), ...}
        """
        if isinstance(eval_total, dict):
            self.metrics["eval_total"] = dict(eval_total)
        if isinstance(eval_by_segment, dict):
            self.metrics["eval_by_segment"] = dict(eval_by_segment)

    def to_process_payload(self) -> Dict[str, Any]:
        """
        Serialisierbares Payload für UI/Logging/Persistenz.
        """
        out: Dict[str, Any] = {
            "role": str(self.role or ""),
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "meta": dict(self.meta or {}),
            "metrics": dict(self.metrics or {}),
        }
        if isinstance(self.executed_traj, dict):
            out["executed_traj"] = self.executed_traj
        if self.executed_by_segment:
            out["executed_by_segment"] = dict(self.executed_by_segment)
        return out
