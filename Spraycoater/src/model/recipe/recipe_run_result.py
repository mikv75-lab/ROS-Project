# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Mapping


def _ensure_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


@dataclass
class RunResult:
    """
    Ergebniscontainer (strict) für:
      - planned/executed JointTrajectory (JTBySegment YAML dict)
      - planned/executed TCP (Draft YAML dict v1) via FK
      - Eval (dict) + valid/invalid

    Hard-Contract:
      - planned_run, executed_run existieren immer (dict)
      - planned_run["traj"] existiert immer (dict)
      - executed_run["traj"] existiert immer (dict)
      - planned_run["tcp"] existiert immer (dict)
      - executed_run["tcp"] existiert immer (dict)
      - fk_meta existiert immer (dict)
      - eval existiert immer (dict)
      - valid existiert immer (bool)
      - invalid_reason existiert immer (str)

    Semantik:
      - BaseStatemachine füllt mind. planned/executed traj.
      - Danach kann RunResult selbst FK + Eval berechnen (oder ProcessTab ruft diese Methoden auf).
      - valid/invalid wird aus Eval abgeleitet (oder explizit gesetzt).
    """

    planned_run: Dict[str, Any] = field(default_factory=dict)
    executed_run: Dict[str, Any] = field(default_factory=dict)

    fk_meta: Dict[str, Any] = field(default_factory=dict)
    eval: Dict[str, Any] = field(default_factory=dict)

    valid: bool = False
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

        self.planned_run["traj"] = _ensure_dict(self.planned_run["traj"])
        self.executed_run["traj"] = _ensure_dict(self.executed_run["traj"])
        self.planned_run["tcp"] = _ensure_dict(self.planned_run["tcp"])
        self.executed_run["tcp"] = _ensure_dict(self.executed_run["tcp"])

        # strict primitives
        self.valid = bool(self.valid)
        self.invalid_reason = str(self.invalid_reason or "")

    # ---------------- Convenience setters ----------------

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

    # ---------------- FK + Eval helpers (optional ownership in RunResult) ----------------

    def compute_fk_tcp(
        self,
        *,
        fk_builder: Any,
        robot_model: Any,
        fk_cfg: Any,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
        drop_duplicate_boundary: bool = True,
        fk_meta: Optional[Mapping[str, Any]] = None,
    ) -> None:
        """
        Berechnet TCP Draft (v1) aus planned/executed traj (JTBySegment YAML dict) und setzt:

          planned_run["tcp"], executed_run["tcp"], fk_meta

        fk_builder: typ. TrajFkBuilder (aus traj_fk_builder.py)
        fk_cfg: typ. TrajFkConfig
        """
        planned_traj = _ensure_dict(self.planned_run.get("traj"))
        executed_traj = _ensure_dict(self.executed_run.get("traj"))

        planned_tcp = fk_builder.build_tcp_draft_yaml(
            planned_traj,
            robot_model=robot_model,
            cfg=fk_cfg,
            segment_to_side=segment_to_side,
            default_side=default_side,
            drop_duplicate_boundary=drop_duplicate_boundary,
        )
        executed_tcp = fk_builder.build_tcp_draft_yaml(
            executed_traj,
            robot_model=robot_model,
            cfg=fk_cfg,
            segment_to_side=segment_to_side,
            default_side=default_side,
            drop_duplicate_boundary=drop_duplicate_boundary,
        )

        self.planned_run["tcp"] = _ensure_dict(planned_tcp)
        self.executed_run["tcp"] = _ensure_dict(executed_tcp)

        meta_out: Dict[str, Any] = dict(fk_meta or {})
        # best-effort cfg snapshot
        try:
            meta_out.setdefault("fk_cfg", {})
            meta_out["fk_cfg"] = dict(meta_out.get("fk_cfg") or {})
            for k in ("ee_link", "meters_to_mm", "step_mm", "max_points"):
                if hasattr(fk_cfg, k):
                    meta_out["fk_cfg"][k] = getattr(fk_cfg, k)
        except Exception:
            pass

        self.fk_meta = meta_out

    def compute_eval_and_validity(
        self,
        *,
        evaluator: Any,
        side: str = "top",
        min_score: float = 80.0,
        prefer: str = "tcp",
        label: str = "run",
    ) -> None:
        """
        Eval + valid/invalid.

        prefer:
          - "tcp": nutzt planned_run["tcp"] vs executed_run["tcp"] (Draft YAML)
          - "joint": nutzt planned_run["traj"] vs executed_run["traj"] (JTBySegment YAML) -> concat -> joint eval

        Ergebnis:
          self.eval = {
            "domain": "tcp"|"joint",
            "min_score": <float>,
            "result": <EvalResult.to_dict()>,
          }
          self.valid / self.invalid_reason gesetzt.
        """
        prefer = str(prefer or "tcp").strip().lower()
        side = str(side or "top").strip() or "top"

        # --- TCP eval (Draft YAML v1)
        if prefer == "tcp":
            ref_tcp = _ensure_dict(self.planned_run.get("tcp"))
            test_tcp = _ensure_dict(self.executed_run.get("tcp"))
            try:
                res = evaluator.evaluate_traj_dict_mm(ref_traj=ref_tcp, test_traj=test_tcp, side=side, label=f"{label}/tcp")
                score = float(getattr(res, "score", 0.0))
                self.eval = {"domain": "tcp", "min_score": float(min_score), "result": res.to_dict()}
                ok = bool(evaluator.is_valid_score(score, min_score=float(min_score)))
                self.set_valid(ok, reason=f"tcp_score<{float(min_score):.1f}" if not ok else "")
                return
            except Exception as ex:
                self.eval = {"domain": "tcp", "min_score": float(min_score), "error": str(ex)}
                self.set_valid(False, reason="tcp_eval_failed")
                return

        # --- JOINT eval (JTBySegment -> concat JT dict)
        ref_traj = _ensure_dict(self.planned_run.get("traj"))
        test_traj = _ensure_dict(self.executed_run.get("traj"))

        def _concat_jt_bysegment(jt_by_seg: Dict[str, Any]) -> Dict[str, Any]:
            segs = jt_by_seg.get("segments") or {}
            if not isinstance(segs, dict):
                return {}
            joint_names = None
            points = []
            for _seg, seg_obj in segs.items():
                if not isinstance(seg_obj, dict):
                    continue
                jn = seg_obj.get("joint_names")
                pts = seg_obj.get("points")
                if not isinstance(jn, list) or not isinstance(pts, list) or not jn or not pts:
                    continue
                if joint_names is None:
                    joint_names = [str(x) for x in jn]
                if [str(x) for x in jn] != joint_names:
                    return {}
                for p in pts:
                    if isinstance(p, dict):
                        points.append(p)
            if not joint_names or not points:
                return {}
            return {"joint_names": joint_names, "points": points}

        try:
            ref_joint = _concat_jt_bysegment(ref_traj)
            test_joint = _concat_jt_bysegment(test_traj)
            res = evaluator.evaluate_joint_trajectory_dict(ref_joint=ref_joint, test_joint=test_joint, label=f"{label}/joint")
            score = float(getattr(res, "score", 0.0))
            self.eval = {"domain": "joint", "min_score": float(min_score), "result": res.to_dict()}
            ok = bool(evaluator.is_valid_score(score, min_score=float(min_score)))
            self.set_valid(ok, reason=f"joint_score<{float(min_score):.1f}" if not ok else "")
        except Exception as ex:
            self.eval = {"domain": "joint", "min_score": float(min_score), "error": str(ex)}
            self.set_valid(False, reason="joint_eval_failed")

    # ---------------- Stable payload for Qt/UI/logging ----------------

    def to_process_payload(self) -> Dict[str, Any]:
        """
        Stable payload schema (Qt-signal safe):

          {
            "planned_run":  {"traj": {...}, "tcp": {...}},
            "executed_run": {"traj": {...}, "tcp": {...}},
            "fk_meta": {...},
            "eval": {...},
            "valid": true/false,
            "invalid_reason": "..."
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
            "eval": dict(_ensure_dict(self.eval)),
            "valid": bool(self.valid),
            "invalid_reason": str(self.invalid_reason or ""),
        }
