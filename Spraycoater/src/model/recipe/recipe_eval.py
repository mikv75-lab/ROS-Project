# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_eval.py
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, List, Sequence

import numpy as np

from model.spray_paths.draft import Draft, PoseQuat
from .recipe import Recipe


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


class RecipeEvaluator:
    """
    STRICT TCP-vs-Draft evaluator.

    Policy (2026-01):
      - Always compare Draft vs Planned and Draft vs Executed.
      - Matching is done by resampling BOTH ref+test to N where N is derived from
        TCP sample counts (planned/executed), NOT from Draft length.
      - Draft may be shorter and will be upsampled (1000 -> 100 style matching).

    Contracts:
      - REQUIRED_FRAME = 'substrate'
      - Reference = recipe.draft (Draft) and MUST be recipe-only for MOVE_RECIPE
      - Test = TCP YAML dict from TrajFkBuilder (may contain segments)
      - Returns STRICT schema consumed by RunResult.report_text()
      - No legacy/fallback output keys

    XYZ scoring (FIX 2026-01):
      - Position score is computed per-axis (abs(dx), abs(dy), abs(dz)) with mean/p95/max,
        then averaged into score_xyz (score_x/y/z are exposed in metrics).
      - L2 metrics are still reported for debug/compat but do NOT drive score_pos anymore.
    """

    REQUIRED_FRAME = "substrate"
    DEFAULT_SEGMENT = "MOVE_RECIPE"

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),  # mean, p95, max (position targets)
        tcp_resample_n: int = 600,
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]
        self.tcp_resample_n = int(max(50, int(tcp_resample_n)))

    # ============================================================
    # Strict helpers
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

    def _draft_from_tcp_yaml_for_segment(self, tcp_doc: Dict[str, Any], *, seg_id: str) -> Dict[str, Any]:
        """
        Extract a minimal Draft-YAML dict from a TCP YAML doc for a single segment.

        Preferred:
          tcp_doc.segments[seg_id].sides
        Fallback:
          tcp_doc.sides

        Frame must be 'substrate' if present.
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
                    return {"frame": seg_frame or self.REQUIRED_FRAME, "sides": sides}

        sides = tcp_doc.get("sides")
        if isinstance(sides, dict) and sides:
            return {"frame": frame or self.REQUIRED_FRAME, "sides": sides}

        return {}

    def _draft_from_recipe_draft(self, recipe: Recipe) -> Draft:
        """
        Reference draft is always recipe.draft.
        Must be a Draft object or a Draft YAML dict.
        """
        d = getattr(recipe, "draft", None)
        if d is None:
            raise ValueError("no_draft")

        if isinstance(d, Draft):
            fr = self._as_str(getattr(d, "frame", "") or "")
            if fr:
                self._require_frame(fr, who="recipe.draft")
            return d

        if isinstance(d, dict):
            fr = self._as_str(d.get("frame") or "")
            if fr:
                self._require_frame(fr, who="recipe.draft(dict)")
            return Draft.from_yaml_dict(d)

        raise ValueError(f"recipe.draft invalid type: {type(d).__name__}")

    def _ref_recipe_segment_poses(self, ref_draft: Draft, *, side: str, seg_id: str) -> List[PoseQuat]:
        """
        STRICT: Draft.poses_quat_segment must map MOVE_RECIPE to recipe-only poses.
        """
        v = ref_draft.poses_quat_segment(side, seg_id)
        return list(v or [])

    @staticmethod
    def _threshold_from_recipe(recipe: Recipe) -> float:
        thr = None
        params = getattr(recipe, "parameters", None)
        if isinstance(params, dict):
            thr = params.get("eval_threshold", None)
        else:
            thr = getattr(params, "eval_threshold", None) if params is not None else None

        try:
            return float(thr) if thr is not None else 90.0
        except Exception:
            return 90.0

    # ============================================================
    # Scoring + quaternion math
    # ============================================================

    @staticmethod
    def _score_from_error(err_val: float, target_val: float) -> float:
        """
        Exponential half-life scoring: err==target -> 50.
        """
        err = max(0.0, float(err_val))
        target = max(1e-9, float(target_val))
        x = err / target
        s = 100.0 * math.exp(-math.log(2.0) * x)
        return float(np.clip(s, 0.0, 100.0))

    @staticmethod
    def _quat_angle_deg_vec(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """
        Quaternion angle distance (deg), vectorized.
        Handles q and -q equivalence via abs(dot).
        """
        if q1.size == 0 or q2.size == 0:
            return np.zeros((0,), dtype=float)

        q1 = np.asarray(q1, dtype=float).reshape(-1, 4)
        q2 = np.asarray(q2, dtype=float).reshape(-1, 4)

        n1 = np.linalg.norm(q1, axis=1, keepdims=True)
        n2 = np.linalg.norm(q2, axis=1, keepdims=True)
        n1 = np.where(n1 > 1e-12, n1, 1.0)
        n2 = np.where(n2 > 1e-12, n2, 1.0)
        q1n = q1 / n1
        q2n = q2 / n2

        dot = np.abs(np.sum(q1n * q2n, axis=1))
        dot = np.clip(dot, -1.0, 1.0)
        ang = np.degrees(2.0 * np.arccos(dot))
        return ang.astype(float)

    @staticmethod
    def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """
        Stable SLERP between two quaternions. Returns normalized quat.
        """
        q0 = np.asarray(q0, dtype=float).reshape(4,)
        q1 = np.asarray(q1, dtype=float).reshape(4,)

        n0 = float(np.linalg.norm(q0))
        n1 = float(np.linalg.norm(q1))
        q0 = q0 / n0 if n0 > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        q1 = q1 / n1 if n1 > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot

        if dot > 0.9995:
            res = (1.0 - t) * q0 + t * q1
            nr = float(np.linalg.norm(res))
            return res / nr if nr > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

        theta_0 = float(np.arccos(dot))
        theta = theta_0 * float(t)
        sin_theta_0 = math.sin(theta_0)
        if abs(sin_theta_0) < 1e-12:
            return q0

        s0 = math.cos(theta) - dot * math.sin(theta) / sin_theta_0
        s1 = math.sin(theta) / sin_theta_0
        res = (s0 * q0) + (s1 * q1)
        nr = float(np.linalg.norm(res))
        return res / nr if nr > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=float)

    # ============================================================
    # Extraction + resampling
    # ============================================================

    def _extract_arrays(self, poses: List[PoseQuat]) -> Tuple[np.ndarray, np.ndarray]:
        if not poses:
            return np.zeros((0, 3), dtype=float), np.zeros((0, 4), dtype=float)
        pts = np.array([[p.x, p.y, p.z] for p in poses], dtype=float)
        quats = np.array([[p.qx, p.qy, p.qz, p.qw] for p in poses], dtype=float)
        return pts, quats

    def _trim_test_to_ref_span(
        self,
        ref_pts: np.ndarray,
        test_pts: np.ndarray,
        test_quats: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        NO-OP (strict).

        Segment-Isolation is already correct from TrajFkBuilder.
        Trimming by nearest start/end can break boustrophedon / zig-zag paths.
        """
        return test_pts, test_quats

    def _resample_pose_mm_quat(self, pts: np.ndarray, quats: np.ndarray, n: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Arc-length resampling in mm + quaternion SLERP, to N samples.
        """
        if len(pts) < 2:
            return pts, quats

        n = int(max(2, int(n)))

        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total_len = float(s[-1])

        if total_len <= 1e-9:
            # degenerate path: repeat first pose
            return np.repeat(pts[:1], n, axis=0), np.repeat(quats[:1], n, axis=0)

        target_s = np.linspace(0.0, total_len, n)
        out_pts = np.zeros((n, 3), dtype=float)
        out_quats = np.zeros((n, 4), dtype=float)

        for dim in range(3):
            out_pts[:, dim] = np.interp(target_s, s, pts[:, dim])

        seg_idx = 0
        max_idx = len(s) - 2
        out_quats[0] = quats[0]
        for i in range(1, n):
            val = float(target_s[i])
            while seg_idx < max_idx and s[seg_idx + 1] < val:
                seg_idx += 1
            len_seg = float(s[seg_idx + 1] - s[seg_idx])
            t = (val - float(s[seg_idx])) / len_seg if len_seg > 1e-9 else 0.0
            t = max(0.0, min(1.0, t))
            out_quats[i] = self._slerp(quats[seg_idx], quats[seg_idx + 1], t)

        return out_pts, out_quats

    # ============================================================
    # Core per-side evaluation
    # ============================================================

    def evaluate_side(
        self,
        *,
        ref_poses: List[PoseQuat],
        test_poses: List[PoseQuat],
        label: str,
        n: int,
    ) -> EvalResult:
        """
        Evaluate one side, resampling BOTH ref+test to the given N (TCP-driven).

        IMPORTANT:
          - N is derived from TCP sizes (planned/executed), not limited by Draft length.
          - Draft may be shorter and will be upsampled.
          - All units are mm for positions.
        """
        ref_p, ref_q = self._extract_arrays(ref_poses)
        test_p, test_q = self._extract_arrays(test_poses)

        metrics: Dict[str, Any] = {
            "label": label,
            "num_ref": int(len(ref_p)),
            "num_test_raw": int(len(test_p)),
            "num_test_trim": int(len(test_p)),
            "resample_n": int(n),
        }

        if len(ref_p) < 2 or len(test_p) < 2:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "not_enough_points"})

        test_p_trim, test_q_trim = self._trim_test_to_ref_span(ref_p, test_p, test_q)
        metrics["num_test_trim"] = int(len(test_p_trim))

        if len(test_p_trim) < 2:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "not_enough_points"})

        n_use = int(max(50, min(int(n), self.tcp_resample_n)))
        metrics["resample_n"] = int(n_use)

        ref_rp, ref_rq = self._resample_pose_mm_quat(ref_p, ref_q, n_use)
        test_rp, test_rq = self._resample_pose_mm_quat(test_p_trim, test_q_trim, n_use)

        if len(ref_rp) < 2 or len(test_rp) < 2:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "not_enough_points"})

        # ------------------------------------------------------------
        # Position errors in mm (XYZ + L2 for debug)
        # ------------------------------------------------------------
        dxyz = (ref_rp - test_rp)  # (N,3) in mm
        dx = np.abs(dxyz[:, 0])
        dy = np.abs(dxyz[:, 1])
        dz = np.abs(dxyz[:, 2])

        # L2 (kept for reference/debug)
        err_mm = np.linalg.norm(dxyz, axis=1)

        # Per-axis stats
        mean_abs_dx = float(np.mean(dx))
        p95_abs_dx = float(np.percentile(dx, 95))
        max_abs_dx = float(np.max(dx))

        mean_abs_dy = float(np.mean(dy))
        p95_abs_dy = float(np.percentile(dy, 95))
        max_abs_dy = float(np.max(dy))

        mean_abs_dz = float(np.mean(dz))
        p95_abs_dz = float(np.percentile(dz, 95))
        max_abs_dz = float(np.max(dz))

        # L2 stats (old keys)
        mean_mm = float(np.mean(err_mm))
        p95_mm = float(np.percentile(err_mm, 95))
        max_mm = float(np.max(err_mm))

        # Orientation error (deg)
        angle_errs = self._quat_angle_deg_vec(ref_rq, test_rq)
        mean_deg = float(np.mean(angle_errs)) if angle_errs.size else 0.0
        p95_deg = float(np.percentile(angle_errs, 95)) if angle_errs.size else 0.0
        max_deg = float(np.max(angle_errs)) if angle_errs.size else 0.0

        # Path length delta
        ref_len = float(np.sum(np.linalg.norm(np.diff(ref_rp, axis=0), axis=1)))
        test_len = float(np.sum(np.linalg.norm(np.diff(test_rp, axis=0), axis=1)))
        delta_L_percent = float(100.0 * (test_len - ref_len) / max(ref_len, 1e-6))

        metrics.update(
            {
                # L2 (existing)
                "mean_l2_mm": mean_mm,
                "p95_l2_mm": p95_mm,
                "max_l2_mm": max_mm,

                # Rotation (existing)
                "mean_angle_deg": mean_deg,
                "p95_angle_deg": p95_deg,
                "max_angle_deg": max_deg,

                # XYZ (new, explicit)
                "mean_abs_dx_mm": mean_abs_dx,
                "p95_abs_dx_mm": p95_abs_dx,
                "max_abs_dx_mm": max_abs_dx,
                "mean_abs_dy_mm": mean_abs_dy,
                "p95_abs_dy_mm": p95_abs_dy,
                "max_abs_dy_mm": max_abs_dy,
                "mean_abs_dz_mm": mean_abs_dz,
                "p95_abs_dz_mm": p95_abs_dz,
                "max_abs_dz_mm": max_abs_dz,

                "delta_L_percent": delta_L_percent,
            }
        )

        # ------------------------------------------------------------
        # XYZ score (per-axis half-life) + L2 score (debug reference)
        # ------------------------------------------------------------
        # Per-axis score components (mean/p95/max for each axis)
        sx_mean = self._score_from_error(mean_abs_dx, self.clamp_mean)
        sx_p95 = self._score_from_error(p95_abs_dx, self.clamp_p95)
        sx_max = self._score_from_error(max_abs_dx, self.clamp_max)

        sy_mean = self._score_from_error(mean_abs_dy, self.clamp_mean)
        sy_p95 = self._score_from_error(p95_abs_dy, self.clamp_p95)
        sy_max = self._score_from_error(max_abs_dy, self.clamp_max)

        sz_mean = self._score_from_error(mean_abs_dz, self.clamp_mean)
        sz_p95 = self._score_from_error(p95_abs_dz, self.clamp_p95)
        sz_max = self._score_from_error(max_abs_dz, self.clamp_max)

        # Weighted mean/p95/max per axis
        score_x = (self.weights.w_mean * sx_mean) + (self.weights.w_p95 * sx_p95) + (self.weights.w_max * sx_max)
        score_y = (self.weights.w_mean * sy_mean) + (self.weights.w_p95 * sy_p95) + (self.weights.w_max * sy_max)
        score_z = (self.weights.w_mean * sz_mean) + (self.weights.w_p95 * sz_p95) + (self.weights.w_max * sz_max)

        # Final XYZ score: average axes (strict symmetric)
        score_xyz = (score_x + score_y + score_z) / 3.0

        # L2 score kept for debugging/monitoring only (not used for final score_pos)
        s_mean = self._score_from_error(mean_mm, self.clamp_mean)
        s_p95 = self._score_from_error(p95_mm, self.clamp_p95)
        s_max = self._score_from_error(max_mm, self.clamp_max)
        score_l2 = (self.weights.w_mean * s_mean) + (self.weights.w_p95 * s_p95) + (self.weights.w_max * s_max)

        metrics.update(
            {
                "score_xyz": float(score_xyz),
                "score_l2": float(score_l2),
                "score_x": float(score_x),
                "score_y": float(score_y),
                "score_z": float(score_z),
            }
        )

        # Position score now uses XYZ (FIX)
        score_pos = float(score_xyz)

        # Rotation score (stable): mean angle target 3 deg
        s_rot = self._score_from_error(mean_deg, target_val=3.0)

        total_score = (0.7 * score_pos) + (0.3 * s_rot)
        return EvalResult(score=float(total_score), metrics=metrics, details={})

    # ============================================================
    # Facade (RunResult calls this)
    # ============================================================

    def evaluate_tcp_against_draft(
        self,
        *,
        recipe: Recipe,
        planned_tcp: Optional[Draft | Dict[str, Any]],
        executed_tcp: Optional[Draft | Dict[str, Any]],
        segment_order: Optional[Sequence[str]] = None,
        domain: str = "tcp",
        traj_points_planned: Optional[int] = None,
        traj_points_executed: Optional[int] = None,
        tcp_poses_planned: Optional[int] = None,
        tcp_poses_executed: Optional[int] = None,
    ) -> Dict[str, Any]:
        if recipe is None:
            return {"valid": False, "invalid_reason": "recipe_none", "domain": str(domain or "tcp")}

        seg_id = self._pick_segment(segment_order)

        # reference draft (recipe)
        ref_draft = self._draft_from_recipe_draft(recipe)
        sides = list(getattr(ref_draft, "sides", {}).keys())
        if not sides:
            return {"valid": False, "invalid_reason": "draft_empty", "domain": str(domain or "tcp")}

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
            raise ValueError(f"{name} invalid type: {type(obj).__name__}")

        pl = to_tcp_draft(planned_tcp, name="planned_tcp")
        ex = to_tcp_draft(executed_tcp, name="executed_tcp")

        # ------------------------------------------------------------------
        # Per-side N is derived from TCP sample counts (NOT Draft length).
        # ------------------------------------------------------------------
        n_by_side: Dict[str, int] = {}
        for side in sides:
            pl_n = 0
            ex_n = 0
            if pl is not None:
                pl_n = int(len(list(pl.poses_quat_segment(side, seg_id) or [])))
            if ex is not None:
                ex_n = int(len(list(ex.poses_quat_segment(side, seg_id) or [])))

            if pl_n >= 2 and ex_n >= 2:
                n_common = min(pl_n, ex_n, self.tcp_resample_n)
            elif pl_n >= 2:
                n_common = min(pl_n, self.tcp_resample_n)
            elif ex_n >= 2:
                n_common = min(ex_n, self.tcp_resample_n)
            else:
                n_common = 0

            n_by_side[side] = int(n_common)

        def run_mode(tcp_draft: Optional[Draft], mode_name: str) -> Dict[str, Any]:
            if tcp_draft is None:
                return {"data_ok": False, "invalid_reason": "missing_tcp", "score": 0.0, "summary": {}}

            by_side: Dict[str, Any] = {}
            scores: List[float] = []
            first_summary: Dict[str, Any] = {}
            bad_sides: List[str] = []

            for idx, side in enumerate(sides):
                ref = self._ref_recipe_segment_poses(ref_draft, side=side, seg_id=seg_id)
                test = list(tcp_draft.poses_quat_segment(side, seg_id) or [])

                n_common = int(n_by_side.get(side, 0))
                if n_common < 2:
                    res = EvalResult(
                        score=0.0,
                        metrics={
                            "label": f"{mode_name}/{seg_id}/{side}",
                            "num_ref": int(len(ref)),
                            "num_test_raw": int(len(test)),
                            "num_test_trim": int(len(test)),
                            "resample_n": int(0),
                        },
                        details={"reason": "not_enough_points"},
                    )
                else:
                    res = self.evaluate_side(
                        ref_poses=ref,
                        test_poses=test,
                        label=f"{mode_name}/{seg_id}/{side}",
                        n=n_common,
                    )

                dct = res.to_dict()
                by_side[side] = dct
                scores.append(float(res.score))

                if idx == 0 and isinstance(dct.get("metrics"), dict):
                    first_summary = dict(dct["metrics"])

                det = dct.get("details")
                if isinstance(det, dict) and det.get("reason") == "not_enough_points":
                    bad_sides.append(str(side))

            data_ok = (len(bad_sides) == 0) and bool(scores)
            avg_score = float(np.mean(scores)) if scores else 0.0

            out = {
                "data_ok": bool(data_ok),
                "score": float(avg_score),
                "segment": str(seg_id),
                "by_side": by_side,
                "summary": first_summary,  # consumed by RunResult.report_text()
            }
            if not data_ok:
                out["invalid_reason"] = f"not_enough_points sides={bad_sides}" if bad_sides else "not_enough_points"
            return out

        p_eval = run_mode(pl, "planned")
        e_eval = run_mode(ex, "executed")

        thr = float(self._threshold_from_recipe(recipe))
        p_score = float(p_eval.get("score", 0.0))
        e_score = float(e_eval.get("score", 0.0))

        planned_ok = bool(p_eval.get("data_ok", False)) and (p_score >= thr)
        executed_ok = bool(e_eval.get("data_ok", False)) and (e_score >= thr)

        valid = bool(planned_ok and executed_ok)
        overall_score = float(min(p_score, e_score))

        invalid_reason = ""
        if not valid:
            if not bool(p_eval.get("data_ok", False)) or not bool(e_eval.get("data_ok", False)):
                invalid_reason = "evaluation failed: insufficient valid points in MOVE_RECIPE"
            else:
                invalid_reason = "evaluation failed: score below threshold"

        selection = {
            "recipe": getattr(recipe, "id", None) or getattr(recipe, "recipe_id", None),
            "tool": getattr(recipe, "tool", None),
            "substrate": getattr(recipe, "substrate", None),
            "mount": getattr(recipe, "substrate_mount", None),
        }

        out: Dict[str, Any] = {
            "version": 7,
            "domain": str(domain or "tcp"),
            "segment": str(seg_id),
            "threshold": float(thr),
            "selection": selection,
            "valid": bool(valid),
            "invalid_reason": str(invalid_reason),
            "score": float(overall_score),
            "planned_valid": bool(planned_ok),
            "executed_valid": bool(executed_ok),
            "planned_score": float(p_score),
            "executed_score": float(e_score),
            "planned": p_eval,
            "executed": e_eval,
            "counts": {
                "traj_points_planned": int(traj_points_planned) if traj_points_planned is not None else None,
                "traj_points_executed": int(traj_points_executed) if traj_points_executed is not None else None,
                "tcp_poses_planned": int(tcp_poses_planned) if tcp_poses_planned is not None else None,
                "tcp_poses_executed": int(tcp_poses_executed) if tcp_poses_executed is not None else None,
            },
            "resample_n_by_side": dict(n_by_side),
        }
        return out

    def evaluate_runs(self, **kwargs) -> Dict[str, Any]:
        return self.evaluate_tcp_against_draft(
            recipe=kwargs.get("recipe"),
            planned_tcp=kwargs.get("planned_tcp"),
            executed_tcp=kwargs.get("executed_tcp"),
            segment_order=kwargs.get("segment_order"),
            domain=kwargs.get("domain", "tcp"),
            traj_points_planned=kwargs.get("traj_points_planned"),
            traj_points_executed=kwargs.get("traj_points_executed"),
            tcp_poses_planned=kwargs.get("tcp_poses_planned"),
            tcp_poses_executed=kwargs.get("tcp_poses_executed"),
        )
