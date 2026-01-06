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
    Vergleicht Test-Pfad gegen Referenz (mm).
    O(N*M) NN ist bewusst simpel; reicht für <= ~2000 Punkte gut.

    Erweiterungen (für deine Anforderungen):
    - evaluate_traj_dict_mm(): evaluiert zwei Traj-Dicts (dein YAML-Schema).
    - evaluate_segments_and_total(): evaluiert pro Segment UND gesamt.
      Referenzquelle wählbar: existing_reference (wenn gesetzt) sonst compiled_reference.
    """

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]

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

    # -------------------------
    # core point evaluation
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

    # -------------------------
    # traj dict extraction helpers
    # -------------------------

    @staticmethod
    def _extract_points_from_traj_dict_mm(traj: Optional[Dict[str, Any]], *, side: str) -> np.ndarray:
        """
        Erwartet dein YAML/Dict-Schema:
          traj["sides"][side]["poses_quat"] = [{x,y,z,qx,qy,qz,qw}, ...]

        Gibt Nx3 float array (mm) zurück.
        """
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
        """
        Convenience: evaluiert 2 Traj-Dicts (dein YAML-Schema) für eine Side.
        """
        ref_pts = self._extract_points_from_traj_dict_mm(ref_traj, side=side)
        test_pts = self._extract_points_from_traj_dict_mm(test_traj, side=side)
        return self.evaluate_points_mm(ref_points_mm=ref_pts, test_points_mm=test_pts, label=label)

    # -------------------------
    # segments + total evaluation (your main requirement)
    # -------------------------

    def evaluate_segments_and_total(
        self,
        *,
        executed_by_segment: Dict[str, Any],
        executed_total: Optional[Dict[str, Any]] = None,
        # reference selection: prefer existing_reference_* if provided, else compiled_reference_*
        existing_reference_by_segment: Optional[Dict[str, Any]] = None,
        existing_reference_total: Optional[Dict[str, Any]] = None,
        compiled_reference_by_segment: Optional[Dict[str, Any]] = None,
        compiled_reference_total: Optional[Dict[str, Any]] = None,
        side: str = "top",
        label_base: str = "traj",
    ) -> Tuple[EvalResult, Dict[str, EvalResult]]:
        """
        Liefert:
          (eval_total, eval_by_segment)

        Regeln:
          - Referenz: existing_reference_* (wenn vorhanden) sonst compiled_reference_*.
          - Segment-Eval: nur wenn es für Segment sowohl ref als auch test Daten gibt (>=1 Punkt).
          - Total-Eval: gegen reference_total; falls executed_total nicht geliefert, wird aus Segmenten concat gebaut.
        """
        # choose reference source
        ref_by_seg = existing_reference_by_segment if isinstance(existing_reference_by_segment, dict) else None
        ref_total = existing_reference_total if isinstance(existing_reference_total, dict) else None

        if ref_by_seg is None and isinstance(compiled_reference_by_segment, dict):
            ref_by_seg = compiled_reference_by_segment
        if ref_total is None and isinstance(compiled_reference_total, dict):
            ref_total = compiled_reference_total

        # build executed_total if needed
        if executed_total is None:
            # concatenate in dict insertion order (caller should pass ordered dict if needed)
            pts_all: List[np.ndarray] = []
            for seg, traj_seg in (executed_by_segment or {}).items():
                pts = self._extract_points_from_traj_dict_mm(traj_seg, side=side)
                if pts.shape[0] > 0:
                    pts_all.append(pts)
            test_total_pts = np.vstack(pts_all) if pts_all else np.zeros((0, 3), dtype=float)
        else:
            test_total_pts = self._extract_points_from_traj_dict_mm(executed_total, side=side)

        # total reference points
        ref_total_pts = self._extract_points_from_traj_dict_mm(ref_total, side=side) if isinstance(ref_total, dict) else np.zeros((0, 3), dtype=float)
        eval_total = self.evaluate_points_mm(
            ref_points_mm=ref_total_pts,
            test_points_mm=test_total_pts,
            label=f"{label_base}/total",
        )

        # per segment
        eval_by_segment: Dict[str, EvalResult] = {}
        for seg, test_traj_seg in (executed_by_segment or {}).items():
            if not isinstance(test_traj_seg, dict):
                continue

            # segment reference: prefer ref_by_seg[seg], else no eval
            ref_traj_seg = None
            if isinstance(ref_by_seg, dict):
                cand = ref_by_seg.get(seg)
                if isinstance(cand, dict):
                    ref_traj_seg = cand

            if not isinstance(ref_traj_seg, dict):
                # no reference for this segment -> skip (explicit, avoids misleading scores)
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
