# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_run_result.py
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple


def _dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


def _as_bool(v: Any, default: bool = False) -> bool:
    if v is None:
        return default
    return bool(v)


def _get_nested(d: Dict[str, Any], path: Sequence[str], default: Any = None) -> Any:
    cur: Any = d
    for k in path:
        if not isinstance(cur, dict):
            return default
        cur = cur.get(k)
    return cur if cur is not None else default


@dataclass
class RunResult:
    """
    STRICT run result container (SSoT).

    Responsibilities:
      - store run state
      - store fk/eval results
      - store validity + reasons
      - store robot description (URDF/SRDF XML) for deterministic FK/TCP/Eval

    Schema:
      planned_run  = {"traj": <JTBySegment YAML dict>, "tcp": <Draft YAML dict or {}>}
      executed_run = {"traj": <JTBySegment YAML dict>, "tcp": <Draft YAML dict or {}>}
    """

    # ---------------- robot description (NEW) ----------------
    # NOTE: Stored as XML strings; may be large but keeps the pipeline deterministic.
    urdf_xml: str = ""
    srdf_xml: str = ""

    # ---------------- core data ----------------
    planned_run: Dict[str, Any] = field(default_factory=dict)
    executed_run: Dict[str, Any] = field(default_factory=dict)

    # ---------------- derived data ----------------
    fk_meta: Dict[str, Any] = field(default_factory=dict)
    eval: Dict[str, Any] = field(default_factory=dict)

    # ---------------- validity ----------------
    valid: bool = True
    invalid_reason: str = ""

    # ------------------------------------------------------------

    def __post_init__(self) -> None:
        self.urdf_xml = str(self.urdf_xml or "")
        self.srdf_xml = str(self.srdf_xml or "")

        self.planned_run = _dict(self.planned_run)
        self.executed_run = _dict(self.executed_run)

        self.planned_run.setdefault("traj", {})
        self.planned_run.setdefault("tcp", {})
        self.executed_run.setdefault("traj", {})
        self.executed_run.setdefault("tcp", {})

        self.fk_meta = _dict(self.fk_meta)
        self.eval = _dict(self.eval)

        self.valid = bool(self.valid)
        self.invalid_reason = str(self.invalid_reason or "")

    # ------------------------------------------------------------
    # basic setters (explicit, no magic)
    # ------------------------------------------------------------

    def set_robot_description(self, *, urdf_xml: str, srdf_xml: str) -> None:
        self.urdf_xml = str(urdf_xml or "")
        self.srdf_xml = str(srdf_xml or "")

    def set_planned(self, *, traj: Dict[str, Any] | None = None, tcp: Dict[str, Any] | None = None) -> None:
        if traj is not None:
            self.planned_run["traj"] = _dict(traj)
        if tcp is not None:
            self.planned_run["tcp"] = _dict(tcp)

    def set_executed(self, *, traj: Dict[str, Any] | None = None, tcp: Dict[str, Any] | None = None) -> None:
        if traj is not None:
            self.executed_run["traj"] = _dict(traj)
        if tcp is not None:
            self.executed_run["tcp"] = _dict(tcp)

    def set_fk_meta(self, meta: Dict[str, Any] | None) -> None:
        self.fk_meta = _dict(meta)

    def set_eval(self, eval_dict: Dict[str, Any] | None) -> None:
        self.eval = _dict(eval_dict)

    def invalidate(self, reason: str) -> None:
        self.valid = False
        self.invalid_reason = str(reason or "invalid")

    # ------------------------------------------------------------
    # derived helpers (UI / persist)
    # ------------------------------------------------------------

    def eval_invalid_reason(self) -> str:
        if not self.eval:
            return "unevaluated"
        v = self.eval.get("valid")
        if v is True:
            return ""
        return str(self.eval.get("invalid_reason") or "eval_invalid")

    def eval_summary_line(self) -> str:
        """
        One-line summary for labels.
        Tries to be robust against changes in evaluator schema.
        """
        if not self.eval:
            return "unevaluated"

        valid = self.eval.get("valid")
        dom = self.eval.get("domain") or ""
        total = _get_nested(self.eval, ["total", "score"], None)
        thr = self.eval.get("threshold", None)

        parts = []
        if dom:
            parts.append(str(dom))
        if total is not None:
            try:
                parts.append(f"score={float(total):.3f}")
            except Exception:
                parts.append(f"score={total}")
        if thr is not None:
            try:
                parts.append(f"thr={float(thr):.3f}")
            except Exception:
                parts.append(f"thr={thr}")
        if valid is True:
            parts.append("VALID")
        elif valid is False:
            parts.append("INVALID")
        else:
            parts.append("unknown")

        return " | ".join(parts) if parts else "–"

    def format_eval_text(self, *, include_tcp: bool = True) -> str:
        """
        Human-readable compact multi-line representation for the Eval text boxes.
        Avoids dumping full YAML joints etc.
        """
        lines: list[str] = []
        lines.append(self.eval_summary_line())

        if self.eval and isinstance(self.eval, dict):
            inv = self.eval.get("invalid_reason") or ""
            if inv:
                lines.append(f"reason: {inv}")

            segs = self.eval.get("segments")
            if isinstance(segs, dict) and segs:
                lines.append("")
                lines.append("segments:")
                for seg_id, seg_v in segs.items():
                    if not isinstance(seg_v, dict):
                        continue
                    v = seg_v.get("valid")
                    score = _get_nested(seg_v, ["score"], None)
                    if score is None:
                        score = _get_nested(seg_v, ["total", "score"], None)
                    if score is not None:
                        try:
                            s_txt = f"{float(score):.3f}"
                        except Exception:
                            s_txt = str(score)
                        lines.append(f"  - {seg_id}: {'OK' if v is True else 'FAIL'} | score={s_txt}")
                    else:
                        lines.append(f"  - {seg_id}: {'OK' if v is True else 'FAIL'}")

        if include_tcp:
            pl_tcp = self.planned_run.get("tcp") if isinstance(self.planned_run, dict) else {}
            ex_tcp = self.executed_run.get("tcp") if isinstance(self.executed_run, dict) else {}

            def _count_tcp(doc: Any) -> int:
                if not isinstance(doc, dict):
                    return 0
                sides = doc.get("sides")
                if not isinstance(sides, dict):
                    return 0
                n = 0
                for _, s in sides.items():
                    if isinstance(s, dict):
                        pq = s.get("poses_quat")
                        if isinstance(pq, list):
                            n += len(pq)
                return n

            lines.append("")
            lines.append(f"tcp: planned={_count_tcp(pl_tcp)} poses | executed={_count_tcp(ex_tcp)} poses")

        if self.fk_meta:
            ee = self.fk_meta.get("ee_link") or ""
            step = self.fk_meta.get("step_mm")
            lines.append("")
            if ee:
                lines.append(f"fk: ee_link={ee}")
            if step is not None:
                try:
                    lines.append(f"fk: step_mm={float(step):.3f}")
                except Exception:
                    lines.append(f"fk: step_mm={step}")

        return "\n".join(lines).strip() if lines else "–"

    # ------------------------------------------------------------
    # payloads (ProcessThread -> UI)
    # ------------------------------------------------------------

    def to_process_payload(self) -> Dict[str, Any]:
        return {
            "robot_description": {
                "urdf_xml": self.urdf_xml,
                "srdf_xml": self.srdf_xml,
            },
            "planned_run": self.planned_run,
            "executed_run": self.executed_run,
            "fk_meta": self.fk_meta,
            "eval": self.eval,
            "valid": self.valid,
            "invalid_reason": self.invalid_reason,
        }

    @classmethod
    def from_process_payload(cls, payload: Dict[str, Any]) -> "RunResult":
        if not isinstance(payload, dict):
            raise ValueError("RunResult.from_process_payload: payload muss dict sein.")

        rd = payload.get("robot_description")
        rd = rd if isinstance(rd, dict) else {}

        return cls(
            urdf_xml=str(rd.get("urdf_xml") or ""),
            srdf_xml=str(rd.get("srdf_xml") or ""),
            planned_run=payload.get("planned_run"),
            executed_run=payload.get("executed_run"),
            fk_meta=payload.get("fk_meta"),
            eval=payload.get("eval"),
            valid=payload.get("valid", True),
            invalid_reason=payload.get("invalid_reason", ""),
        )

    # ------------------------------------------------------------
    # persistence helpers
    # ------------------------------------------------------------

    def build_persist_docs(self, *, embed_eval_into_traj: bool = True) -> Dict[str, Any]:
        """
        Returns dict with:
          - planned_traj
          - executed_traj
          - planned_tcp
          - executed_tcp

        NOTE:
          Robot description (URDF/SRDF) is NOT persisted here on purpose.
          Persist stays run-artifact focused (traj/tcp/eval/fk_meta).
        """
        run_meta = {
            "run_valid": self.valid,
            "run_invalid_reason": self.invalid_reason,
            "eval_invalid_reason": self.eval_invalid_reason(),
        }

        planned_traj = dict(self.planned_run.get("traj") or {})
        executed_traj = dict(self.executed_run.get("traj") or {})

        planned_traj["run_meta"] = dict(run_meta)
        executed_traj["run_meta"] = dict(run_meta)

        if embed_eval_into_traj:
            if self.eval:
                planned_traj["eval"] = dict(self.eval)
                executed_traj["eval"] = dict(self.eval)
            if self.fk_meta:
                planned_traj["fk_meta"] = dict(self.fk_meta)
                executed_traj["fk_meta"] = dict(self.fk_meta)

        return {
            "planned_traj": planned_traj,
            "executed_traj": executed_traj,
            "planned_tcp": dict(self.planned_run.get("tcp") or {}),
            "executed_tcp": dict(self.executed_run.get("tcp") or {}),
        }

    @classmethod
    def from_persist_docs(
        cls,
        *,
        planned_traj_doc: Dict[str, Any],
        executed_traj_doc: Dict[str, Any],
        planned_tcp_doc: Optional[Dict[str, Any]] = None,
        executed_tcp_doc: Optional[Dict[str, Any]] = None,
    ) -> "RunResult":
        if not isinstance(planned_traj_doc, dict) or not isinstance(executed_traj_doc, dict):
            raise ValueError("RunResult.from_persist_docs: planned/executed traj docs müssen dict sein.")

        rm = planned_traj_doc.get("run_meta")
        rm = rm if isinstance(rm, dict) else {}

        return cls(
            # URDF/SRDF intentionally not reconstructed from persist docs.
            planned_run={"traj": dict(planned_traj_doc), "tcp": _dict(planned_tcp_doc or {})},
            executed_run={"traj": dict(executed_traj_doc), "tcp": _dict(executed_tcp_doc or {})},
            fk_meta=_dict(planned_traj_doc.get("fk_meta")),
            eval=_dict(planned_traj_doc.get("eval")),
            valid=_as_bool(rm.get("run_valid"), True),
            invalid_reason=str(rm.get("run_invalid_reason") or ""),
        )

    # ------------------------------------------------------------
    # FK/TCP + Eval
    # ------------------------------------------------------------

    def postprocess(
        self,
        *,
        recipe: Any,
        segment_order: Tuple[str, ...] | Sequence[str],
        ee_link: str = "tcp",
        step_mm: float = 1.0,
        max_points: int = 0,
        gate_valid_on_eval: bool = False,
        require_tcp: bool = True,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
    ) -> None:
        """
        Postprocess using stored URDF/SRDF (set via constructor or set_robot_description()).
        """
        if not self.urdf_xml.strip() or not self.srdf_xml.strip():
            raise ValueError("RunResult.postprocess: urdf_xml/srdf_xml fehlt (leer).")
        self.postprocess_from_urdf_srdf(
            urdf_xml=self.urdf_xml,
            srdf_xml=self.srdf_xml,
            recipe=recipe,
            segment_order=segment_order,
            ee_link=ee_link,
            step_mm=step_mm,
            max_points=max_points,
            gate_valid_on_eval=gate_valid_on_eval,
            require_tcp=require_tcp,
            segment_to_side=segment_to_side,
            default_side=default_side,
        )

    def postprocess_from_urdf_srdf(
        self,
        *,
        urdf_xml: str,
        srdf_xml: str,
        recipe: Any,
        segment_order: Tuple[str, ...] | Sequence[str],
        ee_link: str = "tcp",
        step_mm: float = 1.0,
        max_points: int = 0,
        gate_valid_on_eval: bool = False,
        require_tcp: bool = True,
        segment_to_side: Optional[Dict[str, str]] = None,
        default_side: str = "top",
    ) -> None:
        """
        Build TCP drafts (planned+executed) via FK and compute eval, using URDF/SRDF XML.

        IMPORTANT:
          This expects TrajFkBuilder to offer:
            - build_robot_model_from_urdf_srdf(urdf_xml: str, srdf_xml: str) -> robot_model

          If you prefer a different builder API, say so and I’ll align it to your moveit bindings.
        """
        from .traj_fk_builder import TrajFkBuilder, TrajFkConfig

        urdf_xml = str(urdf_xml or "")
        srdf_xml = str(srdf_xml or "")
        if not urdf_xml.strip() or not srdf_xml.strip():
            raise ValueError("RunResult.postprocess_from_urdf_srdf: urdf_xml/srdf_xml ist leer.")

        # store for later
        self.urdf_xml = urdf_xml
        self.srdf_xml = srdf_xml

        # 1) robot model
        robot_model = TrajFkBuilder.build_robot_model_from_urdf_srdf(urdf_xml=urdf_xml, srdf_xml=srdf_xml)

        # 2) trajectories exist?
        planned_traj = _dict(self.planned_run.get("traj"))
        executed_traj = _dict(self.executed_run.get("traj"))
        if not planned_traj:
            raise ValueError("RunResult.postprocess_from_urdf_srdf: planned_run.traj ist leer.")
        if not executed_traj:
            raise ValueError("RunResult.postprocess_from_urdf_srdf: executed_run.traj ist leer.")

        # 3) FK -> TCP drafts
        cfg = TrajFkConfig(ee_link=str(ee_link or "tcp"), step_mm=float(step_mm), max_points=int(max_points or 0))

        planned_tcp = TrajFkBuilder.build_tcp_draft_yaml(
            planned_traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            drop_duplicate_boundary=True,
        )
        executed_tcp = TrajFkBuilder.build_tcp_draft_yaml(
            executed_traj,
            robot_model=robot_model,
            cfg=cfg,
            segment_to_side=segment_to_side,
            default_side=str(default_side or "top"),
            drop_duplicate_boundary=True,
        )

        self.planned_run["tcp"] = _dict(planned_tcp)
        self.executed_run["tcp"] = _dict(executed_tcp)

        # 4) fk_meta (compact + useful)
        self.fk_meta = {
            "ee_link": str(cfg.ee_link),
            "step_mm": float(cfg.step_mm),
            "max_points": int(cfg.max_points),
            "segment_order": list(segment_order),
        }

        # 5) Eval (tolerant)
        eval_dict: Dict[str, Any] = {}
        try:
            from .recipe_eval import RecipeEvaluator  # type: ignore

            evaluator = RecipeEvaluator()

            fn = getattr(evaluator, "evaluate_runs", None)
            if callable(fn):
                eval_dict = fn(
                    recipe=recipe,
                    planned_tcp=planned_tcp,
                    executed_tcp=executed_tcp,
                    segment_order=tuple(segment_order),
                )
            else:
                fn2 = getattr(evaluator, "evaluate", None)
                if callable(fn2):
                    eval_dict = fn2(recipe=recipe, planned=planned_tcp, executed=executed_tcp)
                else:
                    raise RuntimeError("RecipeEvaluator: keine evaluate_runs()/evaluate() API gefunden.")
        except Exception as e:
            eval_dict = {"valid": False, "invalid_reason": f"eval_failed: {e}"}

        self.eval = _dict(eval_dict)

        if require_tcp:
            if not _dict(self.planned_run.get("tcp")) or not _dict(self.executed_run.get("tcp")):
                raise ValueError("RunResult.postprocess_from_urdf_srdf: TCP Draft leer (require_tcp=true).")

        if gate_valid_on_eval:
            if self.eval and self.eval.get("valid") is False:
                self.invalidate(f"eval_invalid: {self.eval.get('invalid_reason') or 'invalid'}")
