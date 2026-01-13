# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_eval.py
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, List

import numpy as np


# ============================================================
# Data containers
# ============================================================

@dataclass
class EvalWeights:
    w_mean: float = 0.40
    w_p95: float = 0.40
    w_max: float = 0.20


@dataclass
class EvalResult:
    score: float = 0.0
    metrics: Dict[str, Any] = field(default_factory=dict)
    details: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "score": float(self.score),
            "metrics": dict(self.metrics or {}),
            "details": dict(self.details or {}),
        }


# ============================================================
# Evaluator (TCP vs Draft only; MOVE_RECIPE segment only)
# ============================================================

class RecipeEvaluator:
    """
    NEW evaluator (old logic intentionally removed).

    Scope (strict):
      - Compare TCP paths against compiled reference (draft) only.
      - planned vs draft (MOVE_RECIPE only)
      - executed vs draft (MOVE_RECIPE only)

    Segment filtering (best-effort):
      The evaluator uses ONLY the MOVE_RECIPE part of planned/executed tcp docs
      if the tcp doc provides segment information via either:

      A) tcp_doc["segments"][<segment_id>]["sides"][side]["poses_quat"]
      B) tcp_doc["meta"]["segment_slices"][<segment_id>][side] = [start,end)

      If neither exists -> fallback to tcp_doc["sides"] (whole tcp).
    """

    DEFAULT_RECIPE_SEGMENT = "MOVE_RECIPE"

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),   # mean, p95, max targets in mm
        tcp_resample_n: int = 600,
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]
        self.tcp_resample_n = int(max(50, tcp_resample_n))

    # ============================================================
    # basic helpers
    # ============================================================

    @staticmethod
    def is_valid_score(score: float, *, min_score: float) -> bool:
        try:
            return float(score) >= float(min_score)
        except Exception:
            return False

    @staticmethod
    def _score_from_error(err_mm: float, target_mm: float) -> float:
        """
        Score in [0..100], monoton fallend mit Fehler.
        Bei err==target -> Score 50 (Halbwertszeit).
        """
        err = max(0.0, float(err_mm))
        target = max(1e-9, float(target_mm))
        x = err / target
        s = 100.0 * math.exp(-math.log(2.0) * x)
        return float(np.clip(s, 0.0, 100.0))

    @staticmethod
    def _as_float(v: Any) -> Optional[float]:
        try:
            if v is None:
                return None
            return float(v)
        except Exception:
            return None

    # ============================================================
    # extraction: draft + tcp docs -> Nx3 (mm)
    # ============================================================

    @staticmethod
    def _extract_points_from_tcp_doc_mm(tcp_doc: Optional[Dict[str, Any]], *, side: str) -> np.ndarray:
        if not isinstance(tcp_doc, dict):
            return np.zeros((0, 3), dtype=float)

        sides = tcp_doc.get("sides")
        if not isinstance(sides, dict):
            return np.zeros((0, 3), dtype=float)

        side_obj = sides.get(side)
        if not isinstance(side_obj, dict):
            return np.zeros((0, 3), dtype=float)

        poses = side_obj.get("poses_quat")
        if not isinstance(poses, list):
            return np.zeros((0, 3), dtype=float)

        pts: List[List[float]] = []
        for p in poses:
            if not isinstance(p, dict):
                continue
            x = RecipeEvaluator._as_float(p.get("x"))
            y = RecipeEvaluator._as_float(p.get("y"))
            z = RecipeEvaluator._as_float(p.get("z"))
            if None in (x, y, z):
                continue
            pts.append([float(x), float(y), float(z)])

        if not pts:
            return np.zeros((0, 3), dtype=float)
        return np.asarray(pts, dtype=float).reshape(-1, 3)

    @staticmethod
    def _extract_points_from_recipe_draft_mm(recipe: Any, *, side: str) -> np.ndarray:
        d = getattr(recipe, "draft", None)
        if d is None:
            d = getattr(recipe, "draft_data", None)
        if d is None or not hasattr(d, "sides"):
            return np.zeros((0, 3), dtype=float)

        sides = getattr(d, "sides", None)
        if not isinstance(sides, dict):
            return np.zeros((0, 3), dtype=float)

        s = sides.get(side)
        if s is None:
            return np.zeros((0, 3), dtype=float)

        pq = getattr(s, "poses_quat", None)
        if not isinstance(pq, list) or not pq:
            return np.zeros((0, 3), dtype=float)

        pts: List[List[float]] = []
        for p in pq:
            try:
                pts.append([float(p.x), float(p.y), float(p.z)])
            except Exception:
                continue

        if not pts:
            return np.zeros((0, 3), dtype=float)
        return np.asarray(pts, dtype=float).reshape(-1, 3)

    # ============================================================
    # MOVE_RECIPE segment filter for TCP docs
    # ============================================================

    @classmethod
    def _tcp_doc_only_segment(cls, tcp_doc: Optional[Dict[str, Any]], *, segment_id: str) -> Optional[Dict[str, Any]]:
        """
        Return a tcp_doc with ONLY the selected segment.

        Supported optional schemas:

        1) Per-segment payload:
           tcp_doc["segments"][segment_id] -> dict with either:
             - {"sides": {...}}  (preferred)
             - or directly {"<side>": {...}} (tolerated)

        2) Index slices:
           tcp_doc["meta"]["segment_slices"][segment_id][side] = [start,end)

        If nothing usable exists, returns original tcp_doc (fallback).
        """
        if not isinstance(tcp_doc, dict) or not tcp_doc:
            return tcp_doc

        seg = str(segment_id or cls.DEFAULT_RECIPE_SEGMENT)

        # --- (1) segments[...] schema ---
        segs = tcp_doc.get("segments")
        if isinstance(segs, dict):
            seg_doc = segs.get(seg)
            if isinstance(seg_doc, dict):
                # prefer seg_doc["sides"]
                s = seg_doc.get("sides")
                if isinstance(s, dict):
                    return {"frame": tcp_doc.get("frame"), "sides": s, "meta": {"segment": seg}}
                # tolerate seg_doc being already sides-like
                return {"frame": tcp_doc.get("frame"), "sides": seg_doc, "meta": {"segment": seg}}

        # --- (2) meta.segment_slices schema ---
        meta = tcp_doc.get("meta")
        if isinstance(meta, dict):
            sl = meta.get("segment_slices")
            if isinstance(sl, dict):
                seg_sl = sl.get(seg)
                if isinstance(seg_sl, dict):
                    sides = tcp_doc.get("sides")
                    if isinstance(sides, dict) and sides:
                        out_sides: Dict[str, Any] = {}
                        for side, side_doc in sides.items():
                            if not isinstance(side_doc, dict):
                                continue
                            poses = side_doc.get("poses_quat")
                            if not isinstance(poses, list):
                                continue

                            rng = seg_sl.get(side)
                            if not (isinstance(rng, (list, tuple)) and len(rng) == 2):
                                continue
                            try:
                                a = int(rng[0])
                                b = int(rng[1])
                            except Exception:
                                continue

                            a = max(0, a)
                            b = max(a, b)
                            out_sides[str(side)] = {"poses_quat": list(poses[a:b])}

                        if out_sides:
                            return {"frame": tcp_doc.get("frame"), "sides": out_sides, "meta": {"segment": seg}}

        # fallback: cannot segment-filter
        return tcp_doc

    # ============================================================
    # polyline resampling
    # ============================================================

    @staticmethod
    def _resample_polyline_mm(pts: np.ndarray, n: int) -> np.ndarray:
        pts = np.asarray(pts, float).reshape(-1, 3)
        n = int(max(2, n))

        if pts.shape[0] == 0:
            return np.zeros((0, 3), dtype=float)
        if pts.shape[0] == 1:
            return np.repeat(pts[:1], repeats=n, axis=0)

        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total = float(s[-1])
        if total <= 1e-9:
            return np.repeat(pts[:1], repeats=n, axis=0)

        t = np.linspace(0.0, total, n, dtype=float)

        out = np.zeros((n, 3), dtype=float)
        j = 0
        for i, ti in enumerate(t):
            while j < len(s) - 2 and s[j + 1] < ti:
                j += 1
            s0, s1 = float(s[j]), float(s[j + 1])
            p0, p1 = pts[j], pts[j + 1]
            if s1 - s0 <= 1e-12:
                out[i] = p0
            else:
                a = (ti - s0) / (s1 - s0)
                out[i] = (1.0 - a) * p0 + a * p1
        return out

    # ============================================================
    # scoring core
    # ============================================================

    def evaluate_points_mm(
        self,
        *,
        ref_points_mm: np.ndarray,
        test_points_mm: np.ndarray,
        label: str,
    ) -> EvalResult:
        ref = np.asarray(ref_points_mm, float).reshape(-1, 3)
        test = np.asarray(test_points_mm, float).reshape(-1, 3)

        metrics: Dict[str, Any] = {
            "label": str(label),
            "num_ref": int(ref.shape[0]),
            "num_test": int(test.shape[0]),
        }

        if ref.shape[0] == 0 or test.shape[0] == 0:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "empty_ref_or_test"})

        n = min(int(ref.shape[0]), int(test.shape[0]))
        diff = ref[:n] - test[:n]
        err = np.linalg.norm(diff, axis=1)

        mean_e = float(np.mean(err))
        p95_e = float(np.percentile(err, 95.0))
        max_e = float(np.max(err))

        metrics.update({"mean_l2_mm": mean_e, "p95_l2_mm": p95_e, "max_l2_mm": max_e})

        s_mean = self._score_from_error(mean_e, self.clamp_mean)
        s_p95 = self._score_from_error(p95_e, self.clamp_p95)
        s_max = self._score_from_error(max_e, self.clamp_max)

        w = self.weights
        score = (w.w_mean * s_mean) + (w.w_p95 * s_p95) + (w.w_max * s_max)

        return EvalResult(
            score=float(score),
            metrics=metrics,
            details={"score_components": {"mean": s_mean, "p95": s_p95, "max": s_max}},
        )

    def evaluate_polyline_vs_draft_mm(
        self,
        *,
        recipe: Any,
        test_tcp_doc: Optional[Dict[str, Any]],
        side: str,
        label: str,
        n_samples: Optional[int] = None,
    ) -> EvalResult:
        ref_pts = self._extract_points_from_recipe_draft_mm(recipe, side=side)
        test_pts = self._extract_points_from_tcp_doc_mm(test_tcp_doc, side=side)

        metrics: Dict[str, Any] = {
            "label": str(label),
            "num_ref": int(ref_pts.shape[0]),
            "num_test": int(test_pts.shape[0]),
        }
        if ref_pts.shape[0] == 0 or test_pts.shape[0] == 0:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "empty_ref_or_test"})

        n = int(n_samples if n_samples is not None else self.tcp_resample_n)
        ref_r = self._resample_polyline_mm(ref_pts, n)
        test_r = self._resample_polyline_mm(test_pts, n)
        return self.evaluate_points_mm(ref_points_mm=ref_r, test_points_mm=test_r, label=label)

    # ============================================================
    # threshold
    # ============================================================

    @staticmethod
    def _get_threshold_tcp_from_recipe(*, recipe: Any, default: float = 90.0) -> float:
        try:
            params = getattr(recipe, "parameters", None)
            if not isinstance(params, dict):
                return float(default)

            for k in ("eval_threshold_tcp", "eval_threshold", "eval_min_score_tcp", "eval_min_score"):
                if k in params:
                    return float(params[k])

            blk = params.get("eval")
            if isinstance(blk, dict):
                for k in ("threshold_tcp", "threshold", "min_score_tcp", "min_score"):
                    if k in blk:
                        return float(blk[k])
        except Exception:
            return float(default)
        return float(default)

    # ============================================================
    # finalization
    # ============================================================

    @staticmethod
    def finalize_eval(
        *,
        domain: str,
        total: EvalResult,
        by_segment: Dict[str, EvalResult],
        threshold: float,
    ) -> Dict[str, Any]:
        score = float(total.score)
        valid = RecipeEvaluator.is_valid_score(score, min_score=float(threshold))

        invalid_reason = ""
        if not valid:
            invalid_reason = f"eval_below_threshold domain={domain} score={score:.3e} thr={float(threshold):.3f}"

        seg_dict = {k: v.to_dict() for k, v in (by_segment or {}).items()}

        return {
            "domain": str(domain),
            "threshold": float(threshold),
            "valid": bool(valid),
            "score": float(score),
            "invalid_reason": str(invalid_reason),
            "total": total.to_dict(),
            "by_segment": seg_dict,
            "segments": seg_dict,  # compatibility alias
        }

    # ============================================================
    # PUBLIC API: planned/executed vs Draft (MOVE_RECIPE only)
    # ============================================================

    def evaluate_tcp_against_draft(
        self,
        *,
        recipe: Any,
        planned_tcp: Optional[Dict[str, Any]],
        executed_tcp: Optional[Dict[str, Any]],
        n_samples: Optional[int] = None,
        recipe_segment_id: str = DEFAULT_RECIPE_SEGMENT,
    ) -> Dict[str, Any]:
        """
        Returns:

        {
          "version": 1,
          "domain": "tcp_vs_draft",
          "segment": "MOVE_RECIPE",
          "threshold": <thr>,
          "valid": <planned.valid AND executed.valid>,
          "invalid_reason": "...",
          "planned":  <eval dict domain=tcp_planned_vs_draft>,
          "executed": <eval dict domain=tcp_executed_vs_draft>,
        }
        """
        thr = self._get_threshold_tcp_from_recipe(recipe=recipe, default=90.0)
        seg = str(recipe_segment_id or self.DEFAULT_RECIPE_SEGMENT)

        # Segment filter planned/executed to MOVE_RECIPE (best-effort)
        planned_seg = self._tcp_doc_only_segment(planned_tcp, segment_id=seg)
        executed_seg = self._tcp_doc_only_segment(executed_tcp, segment_id=seg)

        # Determine sides from draft primarily, fallback to tcp docs
        sides: List[str] = []
        try:
            d = getattr(recipe, "draft", None) or getattr(recipe, "draft_data", None)
            if d is not None and isinstance(getattr(d, "sides", None), dict):
                for s in d.sides.keys():
                    ss = str(s)
                    if ss not in sides:
                        sides.append(ss)
        except Exception:
            pass

        for t in (planned_seg, executed_seg):
            if isinstance(t, dict) and isinstance(t.get("sides"), dict):
                for s in t["sides"].keys():
                    ss = str(s)
                    if ss not in sides:
                        sides.append(ss)

        if not sides:
            sides = ["top"]

        # ---- planned vs draft
        planned_by: Dict[str, EvalResult] = {}
        wsum = 0.0
        wtot = 0.0
        for side in sides:
            r = self.evaluate_polyline_vs_draft_mm(
                recipe=recipe,
                test_tcp_doc=planned_seg,
                side=side,
                label=f"tcp/planned_vs_draft/{side}",
                n_samples=n_samples,
            )
            planned_by[side] = r

            nref = int(r.metrics.get("num_ref", 0) or 0)
            ntest = int(r.metrics.get("num_test", 0) or 0)
            w = float(max(1, min(nref, ntest)))
            wsum += float(r.score) * w
            wtot += w

        planned_total = EvalResult(
            score=float(wsum / max(1.0, wtot)),
            metrics={"label": "tcp/planned_vs_draft/total", "sides": list(sides), "segment": seg},
            details={},
        )
        planned_eval = self.finalize_eval(
            domain="tcp_planned_vs_draft",
            total=planned_total,
            by_segment=planned_by,
            threshold=thr,
        )

        # ---- executed vs draft
        executed_by: Dict[str, EvalResult] = {}
        wsum = 0.0
        wtot = 0.0
        for side in sides:
            r = self.evaluate_polyline_vs_draft_mm(
                recipe=recipe,
                test_tcp_doc=executed_seg,
                side=side,
                label=f"tcp/executed_vs_draft/{side}",
                n_samples=n_samples,
            )
            executed_by[side] = r

            nref = int(r.metrics.get("num_ref", 0) or 0)
            ntest = int(r.metrics.get("num_test", 0) or 0)
            w = float(max(1, min(nref, ntest)))
            wsum += float(r.score) * w
            wtot += w

        executed_total = EvalResult(
            score=float(wsum / max(1.0, wtot)),
            metrics={"label": "tcp/executed_vs_draft/total", "sides": list(sides), "segment": seg},
            details={},
        )
        executed_eval = self.finalize_eval(
            domain="tcp_executed_vs_draft",
            total=executed_total,
            by_segment=executed_by,
            threshold=thr,
        )

        valid = bool(planned_eval.get("valid")) and bool(executed_eval.get("valid"))
        invalid_reason = ""
        if not valid:
            parts: List[str] = []
            if not bool(planned_eval.get("valid")):
                parts.append("planned_below_threshold")
            if not bool(executed_eval.get("valid")):
                parts.append("executed_below_threshold")
            invalid_reason = "tcp_vs_draft: " + ", ".join(parts)

        return {
            "version": 1,
            "domain": "tcp_vs_draft",
            "segment": seg,
            "threshold": float(thr),
            "valid": bool(valid),
            "invalid_reason": str(invalid_reason),
            "planned": planned_eval,
            "executed": executed_eval,
        }
