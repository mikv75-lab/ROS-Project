# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_eval.py
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, List

import numpy as np


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
    Evaluator für zwei Domänen:

    A) TCP-Punkte (mm) – legacy/visualization use:
       - evaluate_points_mm()
       - evaluate_traj_dict_mm()
       - evaluate_segments_and_total()

    B) JointTrajectory (rad) – NEU für Validate-Replay (timestamped):
       - evaluate_joint_trajectory_dict()
       - evaluate_joint_run_payload()

    Wichtig:
      - Für "Validate -> Replay 1:1" ist primär die Persistenz der JointTrajectory relevant.
      - Joint-Eval ist bewusst "leichtgewichtig" (kein Zeit-Resampling); er vergleicht
        Punkte index-basiert bis min(lenA,lenB) und berücksichtigt time_from_start nur als Metrik.
    """

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),
        clamp_rad: Tuple[float, float, float] = (0.01, 0.03, 0.08),  # mean, p95, max
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]
        self.clamp_mean_rad, self.clamp_p95_rad, self.clamp_max_rad = [float(x) for x in clamp_rad]

    # -------------------------
    # low level math helpers
    # -------------------------

    @staticmethod
    def _path_length_mm(P: np.ndarray) -> float:
        P = np.asarray(P, float).reshape(-1, 3)
        if P.shape[0] < 2:
            return 0.0
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return float(d.sum())

    @staticmethod
    def _step_stats_mm(P: np.ndarray) -> Dict[str, float]:
        P = np.asarray(P, float).reshape(-1, 3)
        if P.shape[0] < 2:
            return {"mean_step_mm": 0.0, "max_step_mm": 0.0}
        d = np.linalg.norm(P[1:] - P[:-1], axis=1)
        return {"mean_step_mm": float(np.mean(d)), "max_step_mm": float(np.max(d))}

    @staticmethod
    def _nearest_neighbor_distances(A: np.ndarray, B: np.ndarray) -> np.ndarray:
        A = np.asarray(A, float).reshape(-1, 3)
        B = np.asarray(B, float).reshape(-1, 3)
        if A.shape[0] == 0 or B.shape[0] == 0:
            return np.zeros((0,), dtype=float)
        diff = A[:, None, :] - B[None, :, :]
        dist = np.linalg.norm(diff, axis=2)
        return np.min(dist, axis=1)

    @staticmethod
    def _percentile(x: np.ndarray, p: float) -> float:
        x = np.asarray(x, float).reshape(-1)
        if x.size == 0:
            return 0.0
        return float(np.percentile(x, p))

    @staticmethod
    def _score_from_error(err: float, target: float) -> float:
        err = max(0.0, float(err))
        target = max(1e-9, float(target))
        x = err / target
        s = 100.0 * math.exp(-math.log(2.0) * x)
        return float(np.clip(s, 0.0, 100.0))

    @staticmethod
    def is_valid_score(score: float, *, min_score: float) -> bool:
        try:
            return float(score) >= float(min_score)
        except Exception:
            return False

    # -------------------------
    # A) TCP points evaluation (legacy)
    # -------------------------

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
            "path_length_ref_mm": self._path_length_mm(ref),
            "path_length_test_mm": self._path_length_mm(test),
        }
        metrics.update({f"ref_{k}": v for k, v in self._step_stats_mm(ref).items()})
        metrics.update({f"test_{k}": v for k, v in self._step_stats_mm(test).items()})

        if ref.shape[0] == 0 or test.shape[0] == 0:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "empty_ref_or_test"})

        nn_rt = self._nearest_neighbor_distances(ref, test)
        nn_tr = self._nearest_neighbor_distances(test, ref)

        max_sym = max(float(np.max(nn_rt)), float(np.max(nn_tr)))
        mean_sym = float(0.5 * (np.mean(nn_rt) + np.mean(nn_tr)))
        p95_sym = float(0.5 * (self._percentile(nn_rt, 95.0) + self._percentile(nn_tr, 95.0)))

        metrics.update({"mean_nn_mm": mean_sym, "p95_nn_mm": p95_sym, "max_nn_mm": max_sym})

        s_mean = self._score_from_error(mean_sym, self.clamp_mean)
        s_p95 = self._score_from_error(p95_sym, self.clamp_p95)
        s_max = self._score_from_error(max_sym, self.clamp_max)

        w = self.weights
        score = (w.w_mean * s_mean) + (w.w_p95 * s_p95) + (w.w_max * s_max)

        details = {
            "score_components": {"mean": s_mean, "p95": s_p95, "max": s_max},
            "weights": {"mean": w.w_mean, "p95": w.w_p95, "max": w.w_max},
        }

        return EvalResult(score=float(np.clip(score, 0.0, 100.0)), metrics=metrics, details=details)

    @staticmethod
    def _extract_points_from_traj_dict_mm(traj: Optional[Dict[str, Any]], *, side: str) -> np.ndarray:
        if not isinstance(traj, dict):
            return np.zeros((0, 3), dtype=float)

        sides = traj.get("sides") or {}
        if not isinstance(sides, dict):
            return np.zeros((0, 3), dtype=float)

        side_obj = sides.get(side) or {}
        if not isinstance(side_obj, dict):
            return np.zeros((0, 3), dtype=float)

        poses = side_obj.get("poses_quat") or []
        if not isinstance(poses, list):
            return np.zeros((0, 3), dtype=float)

        pts: List[List[float]] = []
        for p in poses:
            if not isinstance(p, dict):
                continue
            try:
                pts.append([float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("z", 0.0))])
            except Exception:
                continue

        if not pts:
            return np.zeros((0, 3), dtype=float)
        return np.asarray(pts, dtype=float).reshape(-1, 3)

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
        return self.evaluate_points_mm(ref_points_mm=ref_pts, test_points_mm=test_pts, label=label)

    def evaluate_segments_and_total(
        self,
        *,
        executed_by_segment: Dict[str, Any],
        executed_total: Optional[Dict[str, Any]] = None,
        existing_reference_by_segment: Optional[Dict[str, Any]] = None,
        existing_reference_total: Optional[Dict[str, Any]] = None,
        compiled_reference_by_segment: Optional[Dict[str, Any]] = None,
        compiled_reference_total: Optional[Dict[str, Any]] = None,
        side: str = "top",
        label_base: str = "traj",
    ) -> Tuple[EvalResult, Dict[str, EvalResult]]:
        ref_by_seg = existing_reference_by_segment if isinstance(existing_reference_by_segment, dict) else None
        ref_total = existing_reference_total if isinstance(existing_reference_total, dict) else None

        if ref_by_seg is None and isinstance(compiled_reference_by_segment, dict):
            ref_by_seg = compiled_reference_by_segment
        if ref_total is None and isinstance(compiled_reference_total, dict):
            ref_total = compiled_reference_total

        if executed_total is None:
            pts_all: List[np.ndarray] = []
            for _seg, traj_seg in (executed_by_segment or {}).items():
                pts = self._extract_points_from_traj_dict_mm(traj_seg, side=side)
                if pts.shape[0] > 0:
                    pts_all.append(pts)
            test_total_pts = np.vstack(pts_all) if pts_all else np.zeros((0, 3), dtype=float)
        else:
            test_total_pts = self._extract_points_from_traj_dict_mm(executed_total, side=side)

        ref_total_pts = (
            self._extract_points_from_traj_dict_mm(ref_total, side=side)
            if isinstance(ref_total, dict)
            else np.zeros((0, 3), dtype=float)
        )
        eval_total = self.evaluate_points_mm(
            ref_points_mm=ref_total_pts,
            test_points_mm=test_total_pts,
            label=f"{label_base}/total",
        )

        eval_by_segment: Dict[str, EvalResult] = {}
        for seg, test_traj_seg in (executed_by_segment or {}).items():
            if not isinstance(test_traj_seg, dict):
                continue
            ref_traj_seg = None
            if isinstance(ref_by_seg, dict):
                cand = ref_by_seg.get(seg)
                if isinstance(cand, dict):
                    ref_traj_seg = cand
            if not isinstance(ref_traj_seg, dict):
                continue

            ref_pts = self._extract_points_from_traj_dict_mm(ref_traj_seg, side=side)
            test_pts = self._extract_points_from_traj_dict_mm(test_traj_seg, side=side)
            if ref_pts.shape[0] == 0 or test_pts.shape[0] == 0:
                continue

            eval_by_segment[seg] = self.evaluate_points_mm(
                ref_points_mm=ref_pts,
                test_points_mm=test_pts,
                label=f"{label_base}/{seg}",
            )

        return eval_total, eval_by_segment

    # -------------------------
    # B) JointTrajectory evaluation (NEW)
    # -------------------------

    @staticmethod
    def _duration_to_seconds(d: Any) -> float:
        if not isinstance(d, dict):
            return 0.0
        try:
            sec = float(d.get("sec", 0))
            nsec = float(d.get("nanosec", 0))
            return float(sec + (nsec * 1e-9))
        except Exception:
            return 0.0

    @staticmethod
    def _extract_joint_traj_dict(j: Any) -> Optional[Dict[str, Any]]:
        """
        Erwartet STRICT JointTrajectory-dict:
          {"joint_names":[...], "points":[{"positions":[...], ... , "time_from_start":{"sec":..,"nanosec":..}}, ...]}
        """
        if not isinstance(j, dict):
            return None
        if not isinstance(j.get("joint_names"), list):
            return None
        if not isinstance(j.get("points"), list):
            return None
        if not j.get("joint_names") or not j.get("points"):
            return None
        return j

    @classmethod
    def _positions_matrix(cls, jt: Dict[str, Any]) -> np.ndarray:
        pts = jt.get("points") or []
        Q: List[List[float]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            if not isinstance(pos, list):
                continue
            try:
                Q.append([float(x) for x in pos])
            except Exception:
                continue
        if not Q:
            return np.zeros((0, 0), dtype=float)
        return np.asarray(Q, dtype=float)

    @classmethod
    def _times_seconds(cls, jt: Dict[str, Any]) -> np.ndarray:
        pts = jt.get("points") or []
        t: List[float] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            t.append(cls._duration_to_seconds(p.get("time_from_start") or {}))
        return np.asarray(t, dtype=float).reshape(-1)

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
            metrics.update(
                {
                    "num_ref": int(len((ref_joint or {}).get("points") or [])),
                    "num_test": int(len((test_joint or {}).get("points") or [])),
                }
            )
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "missing_ref_or_test"})

        jn_ref = [str(x) for x in (ref_joint.get("joint_names") or [])]
        jn_test = [str(x) for x in (test_joint.get("joint_names") or [])]
        metrics.update({"joint_names_ref": jn_ref, "joint_names_test": jn_test})

        if jn_ref != jn_test:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "joint_names_mismatch"})

        Qr = self._positions_matrix(ref_joint)
        Qt = self._positions_matrix(test_joint)

        n = int(min(Qr.shape[0], Qt.shape[0]))
        if n <= 0 or Qr.shape[1] == 0:
            metrics.update({"num_ref": int(Qr.shape[0]), "num_test": int(Qt.shape[0])})
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "empty_positions"})

        Qr = Qr[:n, :]
        Qt = Qt[:n, :]

        err = np.abs(Qt - Qr)  # rad
        err_per_point = np.linalg.norm(err, axis=1)

        mean_e = float(np.mean(err_per_point)) if err_per_point.size else 0.0
        p95_e = float(np.percentile(err_per_point, 95.0)) if err_per_point.size else 0.0
        max_e = float(np.max(err_per_point)) if err_per_point.size else 0.0

        metrics.update(
            {
                "num_ref": int(len(ref_joint.get("points") or [])),
                "num_test": int(len(test_joint.get("points") or [])),
                "num_compared": int(n),
                "mean_joint_l2_rad": mean_e,
                "p95_joint_l2_rad": p95_e,
                "max_joint_l2_rad": max_e,
            }
        )

        tr = self._times_seconds(ref_joint)
        tt = self._times_seconds(test_joint)
        metrics.update(
            {
                "t_end_ref_s": float(tr[-1]) if tr.size else 0.0,
                "t_end_test_s": float(tt[-1]) if tt.size else 0.0,
            }
        )

        s_mean = self._score_from_error(mean_e, self.clamp_mean_rad)
        s_p95 = self._score_from_error(p95_e, self.clamp_p95_rad)
        s_max = self._score_from_error(max_e, self.clamp_max_rad)

        w = self.weights
        score = (w.w_mean * s_mean) + (w.w_p95 * s_p95) + (w.w_max * s_max)

        details = {
            "score_components": {"mean": s_mean, "p95": s_p95, "max": s_max},
            "weights": {"mean": w.w_mean, "p95": w.w_p95, "max": w.w_max},
        }

        return EvalResult(score=float(np.clip(score, 0.0, 100.0)), metrics=metrics, details=details)
