# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict


def _ensure_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


@dataclass
class RunResult:
    """
    Ergebniscontainer für Vergleich + Persistenz (strict, forward-compatible).

    Hard-Contract:
      - planned_run, executed_run existieren immer (dict)
      - planned_run["traj"] existiert immer (dict)   # JTBySegment YAML dict
      - executed_run["traj"] existiert immer (dict)
      - planned_run["tcp"] existiert immer (dict)    # Draft YAML dict (v1)
      - executed_run["tcp"] existiert immer (dict)
      - fk_meta existiert immer (dict)               # FK config + model hashes etc.

    Semantik:
      - BaseStatemachine füllt NUR traj.
      - ProcessTab (oder eine FK-Schicht) berechnet tcp strikt und setzt:
          planned_run["tcp"], executed_run["tcp"], fk_meta
        und persistiert anschließend über Repo/Bundle.
    """

    planned_run: Dict[str, Any] = field(default_factory=dict)
    executed_run: Dict[str, Any] = field(default_factory=dict)
    fk_meta: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        # normalize root
        self.planned_run = _ensure_dict(self.planned_run)
        self.executed_run = _ensure_dict(self.executed_run)
        self.fk_meta = _ensure_dict(self.fk_meta)

        # hard keys
        self.planned_run.setdefault("traj", {})
        self.executed_run.setdefault("traj", {})
        self.planned_run.setdefault("tcp", {})
        self.executed_run.setdefault("tcp", {})

        # normalize children
        self.planned_run["traj"] = _ensure_dict(self.planned_run["traj"])
        self.executed_run["traj"] = _ensure_dict(self.executed_run["traj"])
        self.planned_run["tcp"] = _ensure_dict(self.planned_run["tcp"])
        self.executed_run["tcp"] = _ensure_dict(self.executed_run["tcp"])

    # ---------------- Convenience setters ----------------

    def set_planned(
        self,
        *,
        traj: Dict[str, Any] | None = None,
        tcp: Dict[str, Any] | None = None,
    ) -> None:
        if traj is not None:
            self.planned_run["traj"] = _ensure_dict(traj)
        if tcp is not None:
            self.planned_run["tcp"] = _ensure_dict(tcp)

    def set_executed(
        self,
        *,
        traj: Dict[str, Any] | None = None,
        tcp: Dict[str, Any] | None = None,
    ) -> None:
        if traj is not None:
            self.executed_run["traj"] = _ensure_dict(traj)
        if tcp is not None:
            self.executed_run["tcp"] = _ensure_dict(tcp)

    def set_fk_meta(self, meta: Dict[str, Any] | None) -> None:
        self.fk_meta = _ensure_dict(meta)

    # ---------------- Stable payload for Qt/UI/logging ----------------

    def to_process_payload(self) -> Dict[str, Any]:
        """
        Stable payload schema for Qt signals and logging:

          {
            "planned_run":  {"traj": {...}, "tcp": {...}},
            "executed_run": {"traj": {...}, "tcp": {...}},
            "fk_meta": {...}
          }
        """
        return {
            "planned_run": {
                "traj": dict(_ensure_dict(self.planned_run.get("traj"))),
                "tcp": dict(_ensure_dict(self.planned_run.get("tcp"))),
            },
            "executed_run": {
                "traj": dict(_ensure_dict(self.executed_run.get("traj"))),
                "tcp": dict(_ensure_dict(self.executed_run.get("tcp"))),
            },
            "fk_meta": dict(_ensure_dict(self.fk_meta)),
        }
