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
# Evaluator
# ============================================================

class RecipeEvaluator:
    """
    Evaluator für zwei Domänen:

    A) TCP-Punkte (mm)
    B) JointTrajectory (rad)

    WICHTIG (neuer Contract):
      - Dieser Evaluator rechnet NUR.
      - Er entscheidet NICHT über Run-Gültigkeit.
      - invalid_reason wird kanonisch über finalize_eval() erzeugt.
    """

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),
        clamp_rad: Tuple[float, float, float] = (0.01, 0.03, 0.08),
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]
        self.clamp_mean_rad, self.clamp_p95_rad, self.clamp_max_rad = [
            float(x) for x in clamp_rad
        ]

    # ============================================================
    # helpers
    # ============================================================

    @staticmethod
    def is_valid_score(score: float, *, min_score: float) -> bool:
        try:
            return float(score) >= float(min_score)
        except Exception:
            return False

    @staticmethod
    def _score_from_error(err: float, target: float) -> float:
        err = max(0.0, float(err))
        target = max(1e-9, float(target))
        x = err / target
        s = 100.0 * math.exp(-math.log(2.0) * x)
        return float(np.clip(s, 0.0, 100.0))

    # ============================================================
    # A) TCP (mm)
    # ============================================================

    @staticmethod
    def _extract_points_from_traj_dict_mm(
        traj: Optional[Dict[str, Any]], *, side: str
    ) -> np.ndarray:
        if not isinstance(traj, dict):
            return np.zeros((0, 3), dtype=float)

        sides = traj.get("sides")
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
            try:
                pts.append(
                    [
                        float(p.get("x", 0.0)),
                        float(p.get("y", 0.0)),
                        float(p.get("z", 0.0)),
                    ]
                )
            except Exception:
                continue

        if not pts:
            return np.zeros((0, 3), dtype=float)

        return np.asarray(pts, dtype=float).reshape(-1, 3)

    def evaluate_points_mm(
        self,
        *,
        ref_points_mm: np.ndarray,
        test_points_mm: np.ndarray,
        label: str = "",
    ) -> EvalResult:
        ref = np.asarray(ref_points_mm, float).reshape(-1, 3)
        test = np.asarray(test_points_mm, float).reshape(-1, 3)

        metrics: Dict[str, Any] = {
            "label": label,
            "num_ref": int(ref.shape[0]),
            "num_test": int(test.shape[0]),
        }

        if ref.shape[0] == 0 or test.shape[0] == 0:
            return EvalResult(
                score=0.0,
                metrics=metrics,
                details={"reason": "empty_ref_or_test"},
            )

        diff = ref[: min(len(ref), len(test))] - test[: min(len(ref), len(test))]
        err = np.linalg.norm(diff, axis=1)

        mean_e = float(np.mean(err))
        p95_e = float(np.percentile(err, 95.0))
        max_e = float(np.max(err))

        metrics.update(
            {
                "mean_l2_mm": mean_e,
                "p95_l2_mm": p95_e,
                "max_l2_mm": max_e,
            }
        )

        s_mean = self._score_from_error(mean_e, self.clamp_mean)
        s_p95 = self._score_from_error(p95_e, self.clamp_p95)
        s_max = self._score_from_error(max_e, self.clamp_max)

        w = self.weights
        score = (w.w_mean * s_mean) + (w.w_p95 * s_p95) + (w.w_max * s_max)

        return EvalResult(
            score=float(score),
            metrics=metrics,
            details={
                "score_components": {
                    "mean": s_mean,
                    "p95": s_p95,
                    "max": s_max,
                }
            },
        )

    def evaluate_traj_dict_mm(
        self,
        *,
        ref_traj: Optional[Dict[str, Any]],
        test_traj: Optional[Dict[str, Any]],
        side: str,
        label: str = "",
    ) -> EvalResult:
        ref_pts = self._extract_points_from_traj_dict_mm(ref_traj, side=side)
        test_pts = self._extract_points_from_traj_dict_mm(test_traj, side=side)
        return self.evaluate_points_mm(
            ref_points_mm=ref_pts,
            test_points_mm=test_pts,
            label=label,
        )

    # ============================================================
    # B) JointTrajectory (rad)
    # ============================================================

    @staticmethod
    def _extract_joint_traj_dict(j: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(j, dict):
            return None
        if not isinstance(j.get("joint_names"), list):
            return None
        if not isinstance(j.get("points"), list):
            return None
        if not j["joint_names"] or not j["points"]:
            return None
        return j

    @classmethod
    def _positions_matrix(cls, jt: Dict[str, Any]) -> np.ndarray:
        Q: List[List[float]] = []
        for p in jt.get("points", []):
            pos = p.get("positions")
            if isinstance(pos, list):
                try:
                    Q.append([float(x) for x in pos])
                except Exception:
                    continue
        return np.asarray(Q, dtype=float)

    def evaluate_joint_trajectory_dict(
        self,
        *,
        ref_joint: Optional[Dict[str, Any]],
        test_joint: Optional[Dict[str, Any]],
        label: str = "joint",
    ) -> EvalResult:
        ref_joint = self._extract_joint_traj_dict(ref_joint)
        test_joint = self._extract_joint_traj_dict(test_joint)

        metrics: Dict[str, Any] = {"label": label}

        if ref_joint is None or test_joint is None:
            return EvalResult(
                score=0.0,
                metrics=metrics,
                details={"reason": "missing_ref_or_test"},
            )

        if ref_joint["joint_names"] != test_joint["joint_names"]:
            return EvalResult(
                score=0.0,
                metrics=metrics,
                details={"reason": "joint_names_mismatch"},
            )

        Qr = self._positions_matrix(ref_joint)
        Qt = self._positions_matrix(test_joint)

        n = min(len(Qr), len(Qt))
        if n <= 0:
            return EvalResult(
                score=0.0,
                metrics=metrics,
                details={"reason": "empty_positions"},
            )

        err = np.linalg.norm(Qt[:n] - Qr[:n], axis=1)

        mean_e = float(np.mean(err))
        p95_e = float(np.percentile(err, 95.0))
        max_e = float(np.max(err))

        metrics.update(
            {
                "num_ref": int(len(Qr)),
                "num_test": int(len(Qt)),
                "num_compared": int(n),
                "mean_joint_l2_rad": mean_e,
                "p95_joint_l2_rad": p95_e,
                "max_joint_l2_rad": max_e,
            }
        )

        s_mean = self._score_from_error(mean_e, self.clamp_mean_rad)
        s_p95 = self._score_from_error(p95_e, self.clamp_p95_rad)
        s_max = self._score_from_error(max_e, self.clamp_max_rad)

        w = self.weights
        score = (w.w_mean * s_mean) + (w.w_p95 * s_p95) + (w.w_max * s_max)

        return EvalResult(
            score=float(score),
            metrics=metrics,
            details={
                "score_components": {
                    "mean": s_mean,
                    "p95": s_p95,
                    "max": s_max,
                }
            },
        )

    # ============================================================
    # FINALIZATION (NEU – WICHTIG)
    # ============================================================

    @staticmethod
    def finalize_eval(
        *,
        domain: str,
        total: EvalResult,
        by_segment: Optional[Dict[str, EvalResult]] = None,
        threshold: float,
    ) -> Dict[str, Any]:
        score = float(total.score)
        valid = RecipeEvaluator.is_valid_score(score, min_score=threshold)

        invalid_reason = ""
        if not valid:
            invalid_reason = (
                f"eval_below_threshold domain={domain} "
                f"score={score:.3e} thr={threshold:.3f}"
            )

        seg_dict = {k: v.to_dict() for k, v in (by_segment or {}).items()}

        return {
            "domain": domain,
            "threshold": float(threshold),
            "valid": bool(valid),
            "score": score,
            "invalid_reason": invalid_reason,
            "total": total.to_dict(),
            # Canonical key (new)
            "by_segment": seg_dict,
            # Compatibility alias for UI/RunResult.format_eval_text() expecting eval["segments"]
            "segments": seg_dict,
        }
