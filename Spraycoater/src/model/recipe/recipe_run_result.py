# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class RunResult:
    """
    Ergebniscontainer für einen Prozesslauf.

    NEU (nach deiner aktuellen Policy):
      - Validate liefert zwei SegmentRunPayload v1 (joint-only):
          planned_run  (soll/replay)
          executed_run (ist/trace)

      - Optimize/Execute dürfen weiterhin "ein" Payload liefern (je nach Statemachine),
        aber RunResult kann beide aufnehmen.

    Hinweis:
      - Persistenz (welche Files) soll außerhalb passieren (ProcessTab/Repo/Bundle).
      - RunResult ist nur Transport + UI/Logging.
    """

    ROLE_VALIDATE = "validate"
    ROLE_OPTIMIZE = "optimize"
    ROLE_EXECUTE = "execute"

    role: str
    ok: bool = False
    message: str = ""

    meta: Dict[str, Any] = field(default_factory=dict)
    metrics: Dict[str, Any] = field(default_factory=dict)

    # SegmentRunPayload v1 (strict)
    planned_run: Optional[Dict[str, Any]] = None
    executed_run: Optional[Dict[str, Any]] = None

    def set_eval(
        self,
        *,
        eval_total: Optional[Dict[str, Any]] = None,
        eval_by_segment: Optional[Dict[str, Any]] = None,
        key_total: str = "eval_total",
        key_by_segment: str = "eval_by_segment",
    ) -> None:
        """
        Convenience: Eval-Ergebnisse strukturiert in metrics ablegen.

        Erwartetes Format:
          eval_total: EvalResult.to_dict()
          eval_by_segment: {SEG: EvalResult.to_dict(), ...}
        """
        if isinstance(eval_total, dict):
            self.metrics[str(key_total)] = dict(eval_total)
        if isinstance(eval_by_segment, dict):
            self.metrics[str(key_by_segment)] = dict(eval_by_segment)

    def to_process_payload(self) -> Dict[str, Any]:
        """
        Serialisierbares Payload für UI/Logging.

        Validate:
          {
            "role": "...",
            "ok": bool,
            "message": "...",
            "meta": {...},
            "metrics": {...},
            "planned_run":  <SegmentRunPayload v1>,
            "executed_run": <SegmentRunPayload v1>,
          }

        Für Optimize/Execute kann optional nur executed_run oder planned_run gesetzt sein.
        """
        out: Dict[str, Any] = {
            "role": str(self.role or ""),
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "meta": dict(self.meta or {}),
            "metrics": dict(self.metrics or {}),
        }
        if isinstance(self.planned_run, dict):
            out["planned_run"] = self.planned_run
        if isinstance(self.executed_run, dict):
            out["executed_run"] = self.executed_run
        return out
