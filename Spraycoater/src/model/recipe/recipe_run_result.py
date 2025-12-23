# app/model/recipe/run_result.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional
@dataclass
class RunResult:
    ROLE_VALIDATE = "validate"
    ROLE_OPTIMIZE = "optimize"
    ROLE_EXECUTE = "execute"

    role: str
    ok: bool = False
    message: str = ""

    meta: Dict[str, Any] = field(default_factory=dict)
    metrics: Dict[str, Any] = field(default_factory=dict)

    planned_traj: Optional[Dict[str, Any]] = None
    executed_traj: Optional[Dict[str, Any]] = None

    planned_by_segment: Dict[str, Any] = field(default_factory=dict)
    executed_by_segment: Dict[str, Any] = field(default_factory=dict)

    def to_process_payload(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "role": str(self.role or ""),
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "meta": dict(self.meta or {}),
            "metrics": dict(self.metrics or {}),
        }
        if isinstance(self.planned_traj, dict):
            out["planned_traj"] = self.planned_traj
        if isinstance(self.executed_traj, dict):
            out["executed_traj"] = self.executed_traj
        if self.planned_by_segment:
            out["planned_by_segment"] = dict(self.planned_by_segment)
        if self.executed_by_segment:
            out["executed_by_segment"] = dict(self.executed_by_segment)
        return out
