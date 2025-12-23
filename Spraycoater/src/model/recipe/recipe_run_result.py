# app/model/recipe/run_result.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional


@dataclass
class RunResult:
    """
    Ergebnis eines 'Runs' (Validate / Optimize / Execute).

    ✅ Prozess-Kompatibilität:
      Dein ProcessTab liest heute optional:
        - planned_traj: dict
        - executed_traj: dict
        - metrics: dict

      Deine BaseProcessStatemachine sammelt zusätzlich pro Segment:
        - planned_by_segment
        - executed_by_segment

    Daher enthält RunResult diese Felder direkt und bietet:
      - to_process_payload(): dict für ProcessTab (ohne dass ProcessTab RunResult kennen muss)
    """

    ROLE_VALIDATE = "validate"
    ROLE_OPTIMIZE = "optimize"
    ROLE_EXECUTE = "execute"

    role: str
    ok: bool = False
    message: str = ""

    # freie Meta-Infos (Plannername, Pipeline, request params, durations, etc.)
    meta: Dict[str, Any] = field(default_factory=dict)

    # optional: raw data (legacy/any)
    traj: Optional[Dict[str, Any]] = None

    # optional: evaluation result summary (score, metrics, etc.)
    metrics: Dict[str, Any] = field(default_factory=dict)

    # ✅ von ProcessTab erwartet (wenn vorhanden)
    planned_traj: Optional[Dict[str, Any]] = None
    executed_traj: Optional[Dict[str, Any]] = None

    # ✅ pro Segment gesammelt (deine BaseProcessStatemachine macht genau das)
    planned_by_segment: Dict[str, Any] = field(default_factory=dict)
    executed_by_segment: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "role": str(self.role or ""),
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "meta": dict(self.meta or {}),
            "traj": self.traj,
            "metrics": dict(self.metrics or {}),
            "planned_traj": self.planned_traj,
            "executed_traj": self.executed_traj,
            "planned_by_segment": dict(self.planned_by_segment or {}),
            "executed_by_segment": dict(self.executed_by_segment or {}),
        }

    @staticmethod
    def from_dict(d: Any) -> "RunResult":
        if not isinstance(d, dict):
            d = {}
        return RunResult(
            role=str(d.get("role") or ""),
            ok=bool(d.get("ok", False)),
            message=str(d.get("message") or ""),
            meta=dict(d.get("meta") or {}),
            traj=d.get("traj", None),
            metrics=dict(d.get("metrics") or {}),
            planned_traj=d.get("planned_traj", None),
            executed_traj=d.get("executed_traj", None),
            planned_by_segment=dict(d.get("planned_by_segment") or {}),
            executed_by_segment=dict(d.get("executed_by_segment") or {}),
        )

    def to_process_payload(self) -> Dict[str, Any]:
        """
        Liefert das dict-Format, das dein aktueller ProcessTab direkt versteht.
        (ProcessTab muss RunResult nicht importieren.)

        Enthält:
          planned_traj/executed_traj (falls gesetzt)
          plus Segment-Snapshots als Extras
        """
        out: Dict[str, Any] = {
            "role": str(self.role or ""),
            "ok": bool(self.ok),
            "message": str(self.message or ""),
            "meta": dict(self.meta or {}),
            "metrics": dict(self.metrics or {}),
        }

        # Primary keys für ProcessTab
        if isinstance(self.planned_traj, dict):
            out["planned_traj"] = self.planned_traj
        if isinstance(self.executed_traj, dict):
            out["executed_traj"] = self.executed_traj

        # Segment-Snapshots (optional – UI/Debug)
        if self.planned_by_segment:
            out["planned_by_segment"] = dict(self.planned_by_segment)
        if self.executed_by_segment:
            out["executed_by_segment"] = dict(self.executed_by_segment)

        # Fallback: wenn nur traj gesetzt wurde
        if "planned_traj" not in out and isinstance(self.traj, dict) and self.role != self.ROLE_EXECUTE:
            out["planned_traj"] = self.traj
        if "executed_traj" not in out and isinstance(self.traj, dict) and self.role == self.ROLE_EXECUTE:
            out["executed_traj"] = self.traj

        return out
