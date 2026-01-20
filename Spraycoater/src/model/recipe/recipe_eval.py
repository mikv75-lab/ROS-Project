# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_eval.py
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, List, Sequence

import numpy as np

# STRICT IMPORTS: Use the new models directly
from model.spray_paths.draft import Draft, PoseQuat
from .recipe import Recipe

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
# Evaluator (TCP vs Draft; prefer MOVE_RECIPE segment)
# ============================================================


class RecipeEvaluator:
    """
    Evaluates geometric accuracy between a Reference (Draft) and a Test Path (TCP).

    STRICT:
      - REQUIRED_FRAME: substrate
      - Prefer segment isolation for MOVE_RECIPE if tcp YAML provides segments[seg]['sides']
      - Otherwise fall back to global sides
      - Reference must be recipe-only for MOVE_RECIPE (NOT predispense/retreat)
    """

    REQUIRED_FRAME = "substrate"
    DEFAULT_SEGMENT = "MOVE_RECIPE"

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),  # mean, p95, max targets
        tcp_resample_n: int = 600,
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]
        self.tcp_resample_n = int(max(50, int(tcp_resample_n)))

    # ============================================================
    # Helpers
    # ============================================================

    @staticmethod
    def _as_str(v: Any) -> str:
        return str(v or "").strip()

    def _pick_segment(self, segment_order: Optional[Sequence[str]] = None) -> str:
        if segment_order:
            segs = [self._as_str(s) for s in segment_order if self._as_str(s)]
            if self.DEFAULT_SEGMENT in segs:
                return self.DEFAULT_SEGMENT
        return self.DEFAULT_SEGMENT

    def _require_frame(self, frame: str, *, who: str) -> None:
        fr = self._as_str(frame)
        if fr and fr != self.REQUIRED_FRAME:
            raise ValueError(f"{who} frame mismatch: {fr!r} != {self.REQUIRED_FRAME!r}")

    def _draft_from_tcp_yaml_for_segment(
        self,
        tcp_doc: Dict[str, Any],
        *,
        seg_id: str,
    ) -> Dict[str, Any]:
        """
        Returns a YAML dict shaped like Draft.from_yaml_dict expects:
          {"frame": ..., "sides": {...}}

        Prefer:  tcp_doc["segments"][seg_id]["sides"]
        Fallback: tcp_doc["sides"]

        NOTE:
          - Draft model does not necessarily carry "frame". We still validate tcp_doc["frame"]
            here to keep strictness on the TCP docs.
        """
        if not isinstance(tcp_doc, dict):
            return {}

        frame = self._as_str(tcp_doc.get("frame") or "")
        if frame:
            self._require_frame(frame, who="tcp")

        segs = tcp_doc.get("segments")
        if isinstance(segs, dict):
            seg = segs.get(seg_id)
            if isinstance(seg, dict):
                sides = seg.get("sides")
                if isinstance(sides, dict) and sides:
                    seg_frame = self._as_str(seg.get("frame") or frame or self.REQUIRED_FRAME)
                    if seg_frame:
                        self._require_frame(seg_frame, who=f"tcp.segments[{seg_id}]")
                    # Draft.from_yaml_dict ignores unknown keys; keep "frame" for forward-compat.
                    return {"frame": seg_frame or self.REQUIRED_FRAME, "sides": sides}

        sides = tcp_doc.get("sides")
        if isinstance(sides, dict) and sides:
            return {"frame": frame or self.REQUIRED_FRAME, "sides": sides}

        return {}

    def _draft_from_recipe_draft(self, recipe: Recipe) -> Draft:
        d = getattr(recipe, "draft", None)
        if d is None:
            raise ValueError("no_draft")

        # recipe.draft is already Draft in your architecture
        if isinstance(d, Draft):
            # Draft currently may not carry a frame attribute; keep this tolerant.
            fr = self._as_str(getattr(d, "frame", "") or "")
            if fr:
                self._require_frame(fr, who="recipe.draft")
            return d

        # last-resort compat if recipe.draft became dict somehow
        if isinstance(d, dict):
            fr = self._as_str(d.get("frame") or "")
            if fr:
                self._require_frame(fr, who="recipe.draft(dict)")
            return Draft.from_yaml_dict(d)

        raise ValueError(f"recipe.draft invalid type: {type(d).__name__}")

    def _ref_recipe_segment_poses(self, ref_draft: Draft, *, side: str, seg_id: str) -> List[PoseQuat]:
        """
        STRICT: Reference poses must be recipe-only for MOVE_RECIPE.

        Preferred:
          - Draft.poses_quat_segment(side, seg_id) if available.
        Fallback:
          - Draft.recipe_poses_quat(side) if available.
        Last resort:
          - Draft.poses_quat(side)
        """
        fn_seg = getattr(ref_draft, "poses_quat_segment", None)
        if callable(fn_seg):
            try:
                v = fn_seg(side, seg_id)
                return list(v or [])
            except Exception:
                pass

        fn_recipe = getattr(ref_draft, "recipe_poses_quat", None)
        if callable(fn_recipe):
            try:
                v = fn_recipe(side)
                return list(v or [])
            except Exception:
                pass

        return list(ref_draft.poses_quat(side) or [])

    # ============================================================
    # Math Helpers (Quaternions & SLERP)
    # ============================================================

    @staticmethod
    def _score_from_error(err_val: float, target_val: float) -> float:
        err = max(0.0, float(err_val))
        target = max(1e-9, float(target_val))
        x = err / target
        s = 100.0 * math.exp(-math.log(2.0) * x)
        return float(np.clip(s, 0.0, 100.0))

    @staticmethod
    def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
        n1, n2 = np.linalg.norm(q1), np.linalg.norm(q2)
        if n1 > 1e-9:
            q1 = q1 / n1
        if n2 > 1e-9:
            q2 = q2 / n2
        dot = np.abs(np.dot(q1, q2))
        dot = min(1.0, max(-1.0, dot))
        return float(np.degrees(2.0 * np.arccos(dot)))

    @staticmethod
    def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        q0 = q0 / np.linalg.norm(q0)
        q1 = q1 / np.linalg.norm(q1)
        dot = np.dot(q0, q1)
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        if dot > 0.9995:
            res = (1.0 - t) * q0 + t * q1
            return res / np.linalg.norm(res)

        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        s0 = np.cos(theta) - dot * np.sin(theta) / np.sin(theta_0)
        s1 = np.sin(theta) / np.sin(theta_0)
        res = (s0 * q0) + (s1 * q1)
        return res / np.linalg.norm(res)

    # ============================================================
    # Extraction & Resampling
    # ============================================================

    def _extract_arrays(self, poses: List[PoseQuat]) -> Tuple[np.ndarray, np.ndarray]:
        if not poses:
            return np.zeros((0, 3)), np.zeros((0, 4))
        pts = np.array([[p.x, p.y, p.z] for p in poses], dtype=float)
        quats = np.array([[p.qx, p.qy, p.qz, p.qw] for p in poses], dtype=float)
        return pts, quats

    def _trim_test_to_ref_span(
        self,
        ref_pts: np.ndarray,
        test_pts: np.ndarray,
        test_quats: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if len(ref_pts) < 2 or len(test_pts) < 2:
            return test_pts, test_quats

        a, b = ref_pts[0], ref_pts[-1]
        d_start = np.linalg.norm(test_pts - a, axis=1)
        d_end = np.linalg.norm(test_pts - b, axis=1)

        i_start = int(np.argmin(d_start))
        i_end = i_start + int(np.argmin(d_end[i_start:]))

        if i_end > i_start:
            return test_pts[i_start : i_end + 1], test_quats[i_start : i_end + 1]

        return test_pts, test_quats

    def _resample_pose_mm_quat(self, pts: np.ndarray, quats: np.ndarray, n: int) -> Tuple[np.ndarray, np.ndarray]:
        if len(pts) < 2:
            return pts, quats

        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total_len = float(s[-1])
        if total_len <= 1e-9:
            return np.repeat(pts[:1], n, axis=0), np.repeat(quats[:1], n, axis=0)

        target_s = np.linspace(0.0, total_len, n)
        out_pts = np.zeros((n, 3))
        out_quats = np.zeros((n, 4))

        for dim in range(3):
            out_pts[:, dim] = np.interp(target_s, s, pts[:, dim])

        seg_idx = 0
        max_idx = len(s) - 2
        out_quats[0] = quats[0]

        for i in range(1, n):
            val = target_s[i]
            while seg_idx < max_idx and s[seg_idx + 1] < val:
                seg_idx += 1
            len_seg = s[seg_idx + 1] - s[seg_idx]
            t = (val - s[seg_idx]) / len_seg if len_seg > 1e-9 else 0.0
            t = max(0.0, min(1.0, t))
            out_quats[i] = self._slerp(quats[seg_idx], quats[seg_idx + 1], t)

        return out_pts, out_quats

    # ============================================================
    # Core
    # ============================================================

    def evaluate_side(
        self,
        *,
        ref_poses: List[PoseQuat],
        test_poses: List[PoseQuat],
        label: str,
    ) -> EvalResult:
        ref_p, ref_q = self._extract_arrays(ref_poses)
        test_p, test_q = self._extract_arrays(test_poses)

        metrics = {"label": label, "num_ref": len(ref_p), "num_test_raw": len(test_p)}
        if len(ref_p) < 2 or len(test_p) < 2:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "not_enough_points"})

        test_p_trim, test_q_trim = self._trim_test_to_ref_span(ref_p, test_p, test_q)
        metrics["num_test_trim"] = len(test_p_trim)

        ref_rp, ref_rq = self._resample_pose_mm_quat(ref_p, ref_q, self.tcp_resample_n)
        test_rp, test_rq = self._resample_pose_mm_quat(test_p_trim, test_q_trim, self.tcp_resample_n)

        n = len(ref_rp)
        err_mm = np.linalg.norm(ref_rp - test_rp, axis=1)
        mean_mm = float(np.mean(err_mm))
        p95_mm = float(np.percentile(err_mm, 95))
        max_mm = float(np.max(err_mm))

        angle_errs = [self._quat_angle_deg(ref_rq[i], test_rq[i]) for i in range(n)]
        mean_deg = float(np.mean(angle_errs))
        max_deg = float(np.max(angle_errs))

        metrics.update(
            {
                "mean_l2_mm": mean_mm,
                "p95_l2_mm": p95_mm,
                "max_l2_mm": max_mm,
                "mean_angle_deg": mean_deg,
                "max_angle_deg": max_deg,
            }
        )

        s_mean = self._score_from_error(mean_mm, self.clamp_mean)
        s_p95 = self._score_from_error(p95_mm, self.clamp_p95)
        s_max = self._score_from_error(max_mm, self.clamp_max)

        score_pos = (self.weights.w_mean * s_mean) + (self.weights.w_p95 * s_p95) + (self.weights.w_max * s_max)
        s_rot = self._score_from_error(mean_deg, target_val=3.0)

        total_score = (0.7 * score_pos) + (0.3 * s_rot)
        return EvalResult(score=total_score, metrics=metrics)

    # ============================================================
    # Facade
    # ============================================================

    def evaluate_tcp_against_draft(
        self,
        *,
        recipe: Recipe,
        planned_tcp: Optional[Draft | Dict[str, Any]],
        executed_tcp: Optional[Draft | Dict[str, Any]],
        segment_order: Optional[Sequence[str]] = None,
        domain: str = "tcp",
    ) -> Dict[str, Any]:
        """
        Entry point used by RunResult.postprocess_from_urdf_srdf().

        IMPORTANT:
          - planned_tcp/executed_tcp are TCP YAML dicts from TrajFkBuilder (with segments)
          - recipe.draft is Draft (reference)
          - STRICT: reference is segment-sliced MOVE_RECIPE (recipe-only)
        """
        if recipe is None:
            return {"valid": False, "invalid_reason": "recipe_none"}

        seg_id = self._pick_segment(segment_order)

        # Reference draft (recipe)
        try:
            ref_draft = self._draft_from_recipe_draft(recipe)
        except Exception as e:
            return {"valid": False, "invalid_reason": f"no_draft: {e}"}

        sides = list(getattr(ref_draft, "sides", {}).keys())
        if not sides:
            return {"valid": False, "invalid_reason": "draft_empty"}

        # Normalize TCP objects: accept Draft or dict, but if dict -> select seg_id sides
        def to_tcp_draft(obj: Optional[Draft | Dict[str, Any]], *, name: str) -> Optional[Draft]:
            if obj is None:
                return None
            if isinstance(obj, Draft):
                fr = self._as_str(getattr(obj, "frame", "") or "")
                if fr:
                    self._require_frame(fr, who=name)
                return obj
            if isinstance(obj, dict):
                dd = self._draft_from_tcp_yaml_for_segment(obj, seg_id=seg_id)
                if not dd:
                    return None
                return Draft.from_yaml_dict(dd)
            return None

        pl = to_tcp_draft(planned_tcp, name="planned_tcp")
        ex = to_tcp_draft(executed_tcp, name="executed_tcp")

        # Eval per mode
        def run_mode(tcp_draft: Optional[Draft], mode_name: str) -> Dict[str, Any]:
            if not tcp_draft:
                return {"valid": False, "invalid_reason": "missing_tcp"}

            side_results: Dict[str, Any] = {}
            scores: List[float] = []

            for side in sides:
                # STRICT: reference must be recipe-only for seg_id (MOVE_RECIPE)
                ref = self._ref_recipe_segment_poses(ref_draft, side=side, seg_id=seg_id)
                test = tcp_draft.poses_quat(side)

                res = self.evaluate_side(ref_poses=ref, test_poses=test, label=f"{mode_name}/{seg_id}/{side}")
                side_results[side] = res.to_dict()
                scores.append(res.score)

            avg_score = float(np.mean(scores)) if scores else 0.0
            return {
                "valid": True,
                "score": avg_score,
                "segment": seg_id,
                "by_side": side_results,
                "metrics": side_results.get(sides[0], {}).get("metrics", {}),
            }

        p_eval = run_mode(pl, "planned")
        e_eval = run_mode(ex, "executed")

        thr = float(getattr(recipe, "parameters", {}).get("eval_threshold", 90.0))
        valid = (p_eval.get("score", 0.0) >= thr) and (e_eval.get("score", 0.0) >= thr)

        def _fmt(x: Any) -> str:
            return f"{x:.3f}" if isinstance(x, (float, int)) else "-"

        pm, em = p_eval.get("metrics", {}), e_eval.get("metrics", {})
        table = [
            {"metric": "Score", "planned": _fmt(p_eval.get("score")), "executed": _fmt(e_eval.get("score"))},
            {"metric": "Ø Pos (mm)", "planned": _fmt(pm.get("mean_l2_mm")), "executed": _fmt(em.get("mean_l2_mm"))},
            {"metric": "Max Pos (mm)", "planned": _fmt(pm.get("max_l2_mm")), "executed": _fmt(em.get("max_l2_mm"))},
            {"metric": "Ø Rot (deg)", "planned": _fmt(pm.get("mean_angle_deg")), "executed": _fmt(em.get("mean_angle_deg"))},
        ]

        return {
            "version": 4,
            "domain": str(domain or "tcp"),
            "segment": seg_id,
            "valid": bool(valid),
            "threshold": float(thr),
            "comparison": table,
            "planned": p_eval,
            "executed": e_eval,
            "invalid_reason": "" if valid else "score_below_threshold",
        }

    # Compat API for callers (RunResult uses evaluate_runs with extra kwargs)
    def evaluate_runs(self, **kwargs) -> Dict[str, Any]:
        """
        Accepts extra kwargs safely (segment_order, domain, etc.).
        """
        return self.evaluate_tcp_against_draft(
            recipe=kwargs.get("recipe"),
            planned_tcp=kwargs.get("planned_tcp"),
            executed_tcp=kwargs.get("executed_tcp"),
            segment_order=kwargs.get("segment_order"),
            domain=kwargs.get("domain", "tcp"),
        )
