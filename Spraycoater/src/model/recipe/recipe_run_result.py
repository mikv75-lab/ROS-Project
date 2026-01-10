# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_run_result.py
from __future__ import annotations

import logging
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional, Tuple

_LOG = logging.getLogger("model.recipe.run_result")


def _ensure_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _is_nonempty_dict(v: Any) -> bool:
    return isinstance(v, dict) and bool(v)


@dataclass
class RunResult:
    """Strict RunResult container used end-to-end by ProcessTab/BaseStatemachine.

    Hard contract:
      - planned_run/executed_run always dict
      - planned_run['traj'], executed_run['traj'] always dict
      - planned_run['tcp'],  executed_run['tcp']  always dict
      - fk_meta, eval always dict
      - valid always bool
      - invalid_reason always str

    Notes:
      - valid may be False for an *unevaluated* run (eval == {}).
        ProcessTab may still persist such runs if trajectories are present.
    """

    planned_run: Dict[str, Any] = field(default_factory=dict)
    executed_run: Dict[str, Any] = field(default_factory=dict)
    fk_meta: Dict[str, Any] = field(default_factory=dict)
    eval: Dict[str, Any] = field(default_factory=dict)
    valid: bool = True
    invalid_reason: str = ""

    def __post_init__(self) -> None:
        self.planned_run = _ensure_dict(self.planned_run)
        self.executed_run = _ensure_dict(self.executed_run)
        self.fk_meta = _ensure_dict(self.fk_meta)
        self.eval = _ensure_dict(self.eval)

        self.planned_run.setdefault("traj", {})
        self.executed_run.setdefault("traj", {})
        self.planned_run.setdefault("tcp", {})
        self.executed_run.setdefault("tcp", {})

        self.planned_run["traj"] = _ensure_dict(self.planned_run.get("traj"))
        self.executed_run["traj"] = _ensure_dict(self.executed_run.get("traj"))
        self.planned_run["tcp"] = _ensure_dict(self.planned_run.get("tcp"))
        self.executed_run["tcp"] = _ensure_dict(self.executed_run.get("tcp"))

        self.valid = bool(self.valid)
        self.invalid_reason = str(self.invalid_reason or "")
        if not self.valid and not self.invalid_reason:
            # Distinguish "unevaluated" from explicit failures via callers.
            self.invalid_reason = "invalid"

    # ---------------- convenience setters ----------------

    def set_planned(self, *, traj: Optional[Dict[str, Any]] = None, tcp: Optional[Dict[str, Any]] = None) -> None:
        if traj is not None:
            self.planned_run["traj"] = _ensure_dict(traj)
        if tcp is not None:
            self.planned_run["tcp"] = _ensure_dict(tcp)

    def set_executed(self, *, traj: Optional[Dict[str, Any]] = None, tcp: Optional[Dict[str, Any]] = None) -> None:
        if traj is not None:
            self.executed_run["traj"] = _ensure_dict(traj)
        if tcp is not None:
            self.executed_run["tcp"] = _ensure_dict(tcp)

    def set_fk_meta(self, meta: Optional[Dict[str, Any]]) -> None:
        self.fk_meta = _ensure_dict(meta)

    def set_eval(self, eval_dict: Optional[Dict[str, Any]]) -> None:
        self.eval = _ensure_dict(eval_dict)

    def set_valid(self, valid: bool, *, reason: str = "") -> None:
        self.valid = bool(valid)
        self.invalid_reason = "" if self.valid else str(reason or "invalid")
        _LOG.info("set_valid: valid=%s reason=%r", self.valid, self.invalid_reason)

    # ---------------- helpers used by ProcessTab ----------------

    def traj_ok(self, *, which: str) -> Tuple[bool, str]:
        which = str(which or "").strip().lower()
        if which not in ("planned", "executed"):
            return False, "which must be 'planned'|'executed'"
        run = self.planned_run if which == "planned" else self.executed_run
        traj = _ensure_dict(run.get("traj"))
        if not traj:
            return False, "traj_empty"
        try:
            ver = int(traj.get("version", 0))
        except Exception:
            ver = 0
        if ver != 1:
            return False, f"traj_version={ver}"
        segs = traj.get("segments")
        if not isinstance(segs, dict) or not segs:
            return False, "segments_empty"
        return True, ""

    def debug_summary(self, *, prefix: str = "") -> str:
        ok_p, rp = self.traj_ok(which="planned")
        ok_e, re = self.traj_ok(which="executed")
        p_tcp = _ensure_dict(self.planned_run.get("tcp"))
        e_tcp = _ensure_dict(self.executed_run.get("tcp"))
        ev = _ensure_dict(self.eval)
        return (
            f"{prefix}valid={self.valid} reason={self.invalid_reason!r} | "
            f"planned_traj_ok={ok_p}({rp}) executed_traj_ok={ok_e}({re}) | "
            f"planned_tcp_keys={len(p_tcp)} executed_tcp_keys={len(e_tcp)} | eval_keys={list(ev.keys())}"
        )

    # ---------------- stable Qt payload ----------------

    def to_process_payload(self) -> Dict[str, Any]:
        _LOG.info(
            "to_process_payload:emit | valid=%s reason=%r | planned_traj_keys=%d executed_traj_keys=%d | planned_tcp_keys=%d executed_tcp_keys=%d | eval_keys=%s",
            bool(self.valid),
            str(self.invalid_reason or ""),
            len(_ensure_dict(self.planned_run.get("traj"))),
            len(_ensure_dict(self.executed_run.get("traj"))),
            len(_ensure_dict(self.planned_run.get("tcp"))),
            len(_ensure_dict(self.executed_run.get("tcp"))),
            list(_ensure_dict(self.eval).keys()),
        )
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
            "eval": dict(_ensure_dict(self.eval)),
            "valid": bool(self.valid),
            "invalid_reason": str(self.invalid_reason or ""),
        }

    @classmethod
    def from_process_payload(cls, payload: Any) -> "RunResult":
        if isinstance(payload, cls):
            return payload
        if not isinstance(payload, dict):
            raise TypeError(f"RunResult payload must be dict, got {type(payload)}")
        return cls(
            planned_run=_ensure_dict(payload.get("planned_run")),
            executed_run=_ensure_dict(payload.get("executed_run")),
            fk_meta=_ensure_dict(payload.get("fk_meta")),
            eval=_ensure_dict(payload.get("eval")),
            valid=bool(payload.get("valid", False)),
            invalid_reason=str(payload.get("invalid_reason", "") or ""),
        )
