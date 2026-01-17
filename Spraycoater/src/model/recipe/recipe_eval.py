# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_eval.py
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple, List, Sequence

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
# Evaluator (TCP vs Draft; MOVE_RECIPE segment only)
# ============================================================

class RecipeEvaluator:
    """
    STRICT evaluator:
    - Checks SPATIAL accuracy (XYZ via LERP) and ROTATIONAL accuracy (Quat via SLERP).
    - Returns a 'comparison' table for direct UI display (Planned vs Executed).
    
    IMPORTANT (STRICT FRAME):
      - recipe.draft is in 'substrate' frame (confirmed).
      - planned_tcp/executed_tcp MUST be in 'substrate' frame as well.

    Segment filtering (STRICT, TCP only):
      The evaluator isolates MOVE_RECIPE from planned/executed tcp docs.

    Length mismatch handling:
      - Trims TCP to the Draft span based on Euclidean distance.
      - Resamples both Position (Linear) and Rotation (Spherical) synchronously.
    """

    DEFAULT_RECIPE_SEGMENT = "MOVE_RECIPE"
    REQUIRED_FRAME = "substrate"

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),   # mean, p95, max targets in mm
        tcp_resample_n: int = 600,
        conservative_total: str = "min",  # "min" | "mean"
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]
        self.tcp_resample_n = int(max(50, tcp_resample_n))

        mode = str(conservative_total or "min").strip().lower()
        if mode not in ("min", "mean"):
            mode = "min"
        self.conservative_total = mode

    # ============================================================
    # Math Helpers (Quaternions & SLERP)
    # ============================================================

    @staticmethod
    def is_valid_score(score: float, *, min_score: float) -> bool:
        try:
            return float(score) >= float(min_score)
        except Exception:
            return False

    @staticmethod
    def _score_from_error(err_val: float, target_val: float) -> float:
        """Score in [0..100], exponentially decaying."""
        err = max(0.0, float(err_val))
        target = max(1e-9, float(target_val))
        x = err / target
        s = 100.0 * math.exp(-math.log(2.0) * x)
        return float(np.clip(s, 0.0, 100.0))

    @staticmethod
    def _as_float(v: Any, default: float = 0.0) -> float:
        try:
            if v is None: return default
            return float(v)
        except Exception:
            return default

    @staticmethod
    def _dict(v: Any) -> Dict[str, Any]:
        return v if isinstance(v, dict) else {}

    @staticmethod
    def _fmt(v: Any, ndigits: int = 3) -> str:
        """Format helper for comparison table."""
        if v is None: return "-"
        try:
            return f"{float(v):.{ndigits}f}"
        except Exception:
            return str(v)

    @staticmethod
    def _quat_angle_deg(q1: np.ndarray, q2: np.ndarray) -> float:
        """Calculates minimal rotation difference in degrees."""
        n1 = np.linalg.norm(q1)
        n2 = np.linalg.norm(q2)
        if n1 > 1e-9: q1 = q1 / n1
        if n2 > 1e-9: q2 = q2 / n2
        dot = np.abs(np.dot(q1, q2))
        dot = min(1.0, max(-1.0, dot))
        angle_rad = 2.0 * np.arccos(dot)
        return float(np.degrees(angle_rad))

    @staticmethod
    def _slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
        """Spherical Linear Interpolation."""
        q0 = q0 / np.linalg.norm(q0)
        q1 = q1 / np.linalg.norm(q1)
        dot = np.dot(q0, q1)
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        if dot > 0.9995:
            # Linear fallback for very small angles
            res = (1.0 - t) * q0 + t * q1
            return res / np.linalg.norm(res)
        
        theta_0 = np.arccos(dot)
        theta = theta_0 * t
        s0 = np.cos(theta) - dot * np.sin(theta) / np.sin(theta_0)
        s1 = np.sin(theta) / np.sin(theta_0)
        res = (s0 * q0) + (s1 * q1)
        return res / np.linalg.norm(res)

    @classmethod
    def _require_tcp_frame(cls, tcp_doc: Optional[Dict[str, Any]], *, name: str) -> None:
        if not isinstance(tcp_doc, dict) or not tcp_doc:
            raise ValueError(f"{name}: tcp_doc missing/empty")
        fr = str(tcp_doc.get("frame") or "").strip()
        if not fr:
            raise ValueError(f"{name}: tcp_doc.frame missing/empty (required: {cls.REQUIRED_FRAME!r})")
        if fr != cls.REQUIRED_FRAME:
            raise ValueError(f"{name}: frame mismatch tcp_doc.frame={fr!r} expected={cls.REQUIRED_FRAME!r}")

    # ============================================================
    # Extraction: tcp/draft -> (Nx3 pts, Nx4 quats)
    # ============================================================

    @staticmethod
    def _extract_pose_from_tcp_doc(tcp_doc: Optional[Dict[str, Any]], *, side: str) -> Tuple[np.ndarray, np.ndarray]:
        empty = (np.zeros((0, 3), dtype=float), np.zeros((0, 4), dtype=float))
        if not isinstance(tcp_doc, dict): return empty
        
        sides = tcp_doc.get("sides")
        if not isinstance(sides, dict): return empty
        side_obj = sides.get(side)
        if not isinstance(side_obj, dict): return empty
        poses = side_obj.get("poses_quat")
        if not isinstance(poses, list): return empty

        pts, quats = [], []
        for p in poses:
            if not isinstance(p, dict): continue
            # Extract XYZ
            x = RecipeEvaluator._as_float(p.get("x"))
            y = RecipeEvaluator._as_float(p.get("y"))
            z = RecipeEvaluator._as_float(p.get("z"))
            # Extract Quat (default to identity 0,0,0,1)
            qx = RecipeEvaluator._as_float(p.get("qx"), 0.0)
            qy = RecipeEvaluator._as_float(p.get("qy"), 0.0)
            qz = RecipeEvaluator._as_float(p.get("qz"), 0.0)
            w  = RecipeEvaluator._as_float(p.get("w"), 1.0)
            
            pts.append([x, y, z])
            quats.append([qx, qy, qz, w])

        if not pts: return empty
        return (np.asarray(pts, dtype=float).reshape(-1, 3), np.asarray(quats, dtype=float).reshape(-1, 4))

    @classmethod
    def _extract_pose_from_draft(cls, recipe: Any, *, side: str) -> Tuple[np.ndarray, np.ndarray]:
        empty = (np.zeros((0, 3), dtype=float), np.zeros((0, 4), dtype=float))
        d = getattr(recipe, "draft", None) or getattr(recipe, "draft_data", None)
        if d is None: return empty
        
        # Check frame
        try:
            fr = str(getattr(d, "frame", "") or "").strip()
            if fr and fr != cls.REQUIRED_FRAME:
                raise ValueError(f"recipe.draft.frame={fr!r} expected={cls.REQUIRED_FRAME!r}")
        except Exception: pass

        sides = getattr(d, "sides", None)
        if not isinstance(sides, dict): return empty
        s = sides.get(side)
        if s is None: return empty
        pq = getattr(s, "poses_quat", None)
        if not isinstance(pq, list) or not pq: return empty

        pts, quats = [], []
        for p in pq:
            try:
                pts.append([float(getattr(p, "x", 0)), float(getattr(p, "y", 0)), float(getattr(p, "z", 0))])
                quats.append([float(getattr(p, "qx", 0)), float(getattr(p, "qy", 0)), float(getattr(p, "qz", 0)), float(getattr(p, "w", 1))])
            except Exception: continue
        
        if not pts: return empty
        return (np.asarray(pts, dtype=float).reshape(-1, 3), np.asarray(quats, dtype=float).reshape(-1, 4))

    @staticmethod
    def _trim_test_to_ref_span(ref_pts: np.ndarray, test_pts: np.ndarray, test_quats: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Trims TCP polyline to match Draft span."""
        ref_pts = np.asarray(ref_pts, float).reshape(-1, 3)
        test_pts = np.asarray(test_pts, float).reshape(-1, 3)
        test_quats = np.asarray(test_quats, float).reshape(-1, 4)

        if ref_pts.shape[0] < 2 or test_pts.shape[0] < 2:
            return test_pts, test_quats

        a, b = ref_pts[0], ref_pts[-1]
        
        # Primary direction
        d0 = np.linalg.norm(test_pts - a, axis=1)
        i = int(np.argmin(d0))
        d1 = np.linalg.norm(test_pts - b, axis=1)
        j = i + int(np.argmin(d1[i:]))

        if j > i:
            return test_pts[i : j + 1], test_quats[i : j + 1]

        # Reversed direction
        d0r = np.linalg.norm(test_pts - b, axis=1)
        ir = int(np.argmin(d0r))
        d1r = np.linalg.norm(test_pts - a, axis=1)
        jr = ir + int(np.argmin(d1r[ir:]))

        if jr > ir:
            sl_p = test_pts[ir : jr + 1]
            sl_q = test_quats[ir : jr + 1]
            return sl_p[::-1].copy(), sl_q[::-1].copy()
            
        return test_pts, test_quats

    @classmethod
    def _tcp_doc_only_segment_strict(cls, tcp_doc: Optional[Dict[str, Any]], *, segment_id: str) -> Dict[str, Any]:
        if not isinstance(tcp_doc, dict) or not tcp_doc: raise ValueError("tcp_doc missing/empty")
        seg = str(segment_id or cls.DEFAULT_RECIPE_SEGMENT).strip() or cls.DEFAULT_RECIPE_SEGMENT
        
        # 1. segments schema
        segs = tcp_doc.get("segments")
        if isinstance(segs, dict):
            seg_doc = segs.get(seg)
            if isinstance(seg_doc, dict):
                sides = seg_doc.get("sides")
                if isinstance(sides, dict) and sides:
                    return {"frame": tcp_doc.get("frame"), "sides": sides, "meta": {"segment": seg}}
        
        # 2. meta.segment_slices schema
        meta = tcp_doc.get("meta")
        if isinstance(meta, dict):
            sl = meta.get("segment_slices", {})
            if isinstance(sl, dict):
                seg_sl = sl.get(seg)
                if isinstance(seg_sl, dict):
                    sides = tcp_doc.get("sides")
                    if isinstance(sides, dict) and sides:
                        out_sides = {}
                        for side, side_doc in sides.items():
                            if not isinstance(side_doc, dict): continue
                            poses = side_doc.get("poses_quat")
                            if not isinstance(poses, list): continue

                            rng = seg_sl.get(side)
                            if isinstance(rng, (list, tuple)) and len(rng) == 2:
                                try:
                                    a, b = max(0, int(rng[0])), max(0, int(rng[1]))
                                    if b > a: 
                                        out_sides[str(side)] = {"poses_quat": list(poses[a:b])}
                                except: pass
                        
                        if out_sides: 
                            return {"frame": tcp_doc.get("frame"), "sides": out_sides, "meta": {"segment": seg}}

        raise ValueError(f"tcp_doc missing segment isolation for {seg!r}")

    # ============================================================
    # Resampling (Synchronized XYZ + Quat)
    # ============================================================

    @staticmethod
    def _resample_pose_mm_quat(pts: np.ndarray, quats: np.ndarray, n: int) -> Tuple[np.ndarray, np.ndarray]:
        """
        Resamples Path & Orientation synchronously based on arc length.
        XYZ -> LERP
        Quat -> SLERP
        """
        pts = np.asarray(pts, float).reshape(-1, 3)
        quats = np.asarray(quats, float).reshape(-1, 4)
        n = int(max(2, n))
        if pts.shape[0] < 2: return pts, quats

        d = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total_len = float(s[-1])
        if total_len <= 1e-9: return np.repeat(pts[:1], n, axis=0), np.repeat(quats[:1], n, axis=0)

        target_s = np.linspace(0.0, total_len, n)
        out_pts, out_quats = np.zeros((n, 3)), np.zeros((n, 4))
        out_pts[0], out_quats[0] = pts[0], quats[0]
        
        seg_idx, max_idx = 0, len(s) - 2
        for i in range(1, n):
            val = target_s[i]
            while seg_idx < max_idx and s[seg_idx + 1] < val: seg_idx += 1
            len_seg = s[seg_idx + 1] - s[seg_idx]
            t = (val - s[seg_idx]) / len_seg if len_seg > 1e-9 else 0.0
            t = max(0.0, min(1.0, t))
            
            p0, p1 = pts[seg_idx], pts[seg_idx + 1]
            out_pts[i] = (1.0 - t) * p0 + t * p1
            
            q0, q1 = quats[seg_idx], quats[seg_idx + 1]
            out_quats[i] = RecipeEvaluator._slerp(q0, q1, t)

        return out_pts, out_quats

    # ============================================================
    # Scoring
    # ============================================================

    def evaluate_points_mm_deg(self, *, ref_pts: np.ndarray, ref_quats: np.ndarray, test_pts: np.ndarray, test_quats: np.ndarray, label: str) -> EvalResult:
        ref_p, test_p = np.asarray(ref_pts).reshape(-1, 3), np.asarray(test_pts).reshape(-1, 3)
        ref_q, test_q = np.asarray(ref_quats).reshape(-1, 4), np.asarray(test_quats).reshape(-1, 4)
        
        metrics = {"label": str(label), "num_ref": int(ref_p.shape[0]), "num_test": int(test_p.shape[0])}
        if ref_p.shape[0] == 0 or test_p.shape[0] == 0:
            return EvalResult(score=0.0, metrics=metrics, details={"reason": "empty"})

        n = min(ref_p.shape[0], test_p.shape[0])
        
        # 1. Position Error
        err_mm = np.linalg.norm(ref_p[:n] - test_p[:n], axis=1)
        mean_mm, p95_mm, max_mm = float(np.mean(err_mm)), float(np.percentile(err_mm, 95)), float(np.max(err_mm))

        # 2. Rotation Error
        angle_errs = np.array([self._quat_angle_deg(ref_q[i], test_q[i]) for i in range(n)])
        mean_deg, max_deg = float(np.mean(angle_errs)), float(np.max(angle_errs))

        metrics.update({
            "mean_l2_mm": mean_mm, "p95_l2_mm": p95_mm, "max_l2_mm": max_mm,
            "mean_angle_deg": mean_deg, "max_angle_deg": max_deg
        })

        # 3. Score Calc
        s_mean, s_p95, s_max = [self._score_from_error(v, c) for v, c in [(mean_mm, self.clamp_mean), (p95_mm, self.clamp_p95), (max_mm, self.clamp_max)]]
        score_pos = (self.weights.w_mean * s_mean) + (self.weights.w_p95 * s_p95) + (self.weights.w_max * s_max)
        
        # Rot Score (Fixed weight: target 3.0 deg = 50% score)
        s_rot = self._score_from_error(mean_deg, target_val=3.0)
        
        total_score = (0.7 * score_pos) + (0.3 * s_rot)

        return EvalResult(score=total_score, metrics=metrics, details={"components": {"pos": score_pos, "rot": s_rot}})

    def evaluate_polyline_vs_draft_pose(self, *, recipe: Any, test_tcp_doc: Optional[Dict[str, Any]], side: str, label: str, segment_id: str, n_samples: Optional[int] = None) -> EvalResult:
        ref_pts, ref_quats = self._extract_pose_from_draft(recipe, side=side)
        test_pts, test_quats = self._extract_pose_from_tcp_doc(test_tcp_doc, side=side)
        
        metrics = {"label": label, "num_ref": len(ref_pts), "num_test": len(test_pts), "segment": segment_id}
        if len(ref_pts) == 0 or len(test_pts) == 0: return EvalResult(score=0.0, metrics=metrics)

        test_pts_trim, test_quats_trim = self._trim_test_to_ref_span(ref_pts, test_pts, test_quats)
        metrics["num_test_trim"] = len(test_pts_trim)

        n = int(n_samples if n_samples else self.tcp_resample_n)
        ref_r_p, ref_r_q = self._resample_pose_mm_quat(ref_pts, ref_quats, n)
        test_r_p, test_r_q = self._resample_pose_mm_quat(test_pts_trim, test_quats_trim, n)

        r = self.evaluate_points_mm_deg(ref_pts=ref_r_p, ref_quats=ref_r_q, test_pts=test_r_p, test_quats=test_r_q, label=label)
        r.metrics.update(metrics)
        return r

    # ============================================================
    # Comparison Table Builder
    # ============================================================

    def _build_comparison(self, planned: Dict[str, Any], executed: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Creates table: Metric | Planned | Executed"""
        def get_m(root, key):
            # Safe access to total->metrics->key
            return root.get("total", {}).get("metrics", {}).get(key)

        p_score = self._as_float(planned.get("total", {}).get("score"))
        e_score = self._as_float(executed.get("total", {}).get("score"))

        rows = [
            ("Score (0-100)", p_score, e_score),
            ("Ø Pos Error (mm)", get_m(planned, "mean_l2_mm"), get_m(executed, "mean_l2_mm")),
            ("Max Pos Error (mm)", get_m(planned, "max_l2_mm"), get_m(executed, "max_l2_mm")),
            ("Ø Rot Error (deg)", get_m(planned, "mean_angle_deg"), get_m(executed, "mean_angle_deg")),
            ("Max Rot Error (deg)", get_m(planned, "max_angle_deg"), get_m(executed, "max_angle_deg")),
        ]

        table = []
        for label, p_val, e_val in rows:
            table.append({
                "metric": label,
                "planned": self._fmt(p_val),
                "executed": self._fmt(e_val),
            })
        return table

    @staticmethod
    def finalize_eval(*, domain: str, total: EvalResult, by_segment: Dict[str, EvalResult], threshold: float) -> Dict[str, Any]:
        score = float(total.score)
        valid = RecipeEvaluator.is_valid_score(score, min_score=float(threshold))
        invalid_reason = "" if valid else f"eval_below_threshold domain={domain} score={score:.3e} thr={threshold:.3f}"
        seg_dict = {k: v.to_dict() for k, v in (by_segment or {}).items()}
        return {
            "domain": domain, "threshold": threshold, "valid": valid, "score": score,
            "invalid_reason": invalid_reason, "total": total.to_dict(),
            "by_segment": seg_dict, "segments": seg_dict
        }

    # ============================================================
    # PUBLIC API
    # ============================================================

    def _get_threshold_tcp_from_recipe(self, *, recipe: Any, default: float = 90.0) -> float:
        params = getattr(recipe, "parameters", {}) or {}
        for k in ("eval_threshold_tcp", "eval_threshold"):
            if k in params: return float(params[k])
        return float(default)

    def evaluate_tcp_against_draft(self, *, recipe: Any, planned_tcp: Optional[Dict[str, Any]], executed_tcp: Optional[Dict[str, Any]], n_samples: Optional[int] = None, recipe_segment_id: str = DEFAULT_RECIPE_SEGMENT) -> Dict[str, Any]:
        thr = self._get_threshold_tcp_from_recipe(recipe=recipe, default=90.0)
        seg = str(recipe_segment_id or self.DEFAULT_RECIPE_SEGMENT).strip() or self.DEFAULT_RECIPE_SEGMENT

        self._require_tcp_frame(planned_tcp, name="planned_tcp")
        self._require_tcp_frame(executed_tcp, name="executed_tcp")
        
        planned_seg = self._tcp_doc_only_segment_strict(planned_tcp, segment_id=seg)
        executed_seg = self._tcp_doc_only_segment_strict(executed_tcp, segment_id=seg)

        # Get Sides from Draft or TCP
        sides = []
        try:
            d = getattr(recipe, "draft", None) or getattr(recipe, "draft_data", None)
            if d and isinstance(d.sides, dict): sides = list(map(str, d.sides.keys()))
        except: pass
        if not sides: sides = ["top"]

        # Run Eval Logic
        def run(tcp_doc, mode):
            res_map, wsum, wtot = {}, 0.0, 0.0
            for side in sides:
                r = self.evaluate_polyline_vs_draft_pose(
                    recipe=recipe, test_tcp_doc=tcp_doc, side=side, label=f"tcp/{mode}_vs_draft/{side}",
                    segment_id=seg, n_samples=n_samples
                )
                res_map[side] = r
                w = max(1, min(r.metrics.get("num_ref", 0), r.metrics.get("num_test_trim", 0)))
                wsum += r.score * w
                wtot += w
            total = EvalResult(score=float(wsum / max(1, wtot)), metrics={"label": f"tcp/{mode}/total", "sides": sides})
            return self.finalize_eval(domain=f"tcp_{mode}_vs_draft", total=total, by_segment=res_map, threshold=thr)

        p_eval = run(planned_seg, "planned")
        e_eval = run(executed_seg, "executed")

        # Build Comparison Table
        comparison_table = self._build_comparison(p_eval, e_eval)

        valid = p_eval["valid"] and e_eval["valid"]
        invalid_reason = "" if valid else "tcp_vs_draft: below_threshold"

        return {
            "version": 3,
            "domain": "tcp_vs_draft",
            "segment": seg,
            "threshold": thr,
            "valid": valid,
            "invalid_reason": invalid_reason,
            "planned": p_eval,
            "executed": e_eval,
            "comparison": comparison_table
        }

    def evaluate_runs(self, *, recipe: Any, planned_tcp: Optional[Dict[str, Any]], executed_tcp: Optional[Dict[str, Any]], **kwargs) -> Dict[str, Any]:
        """Compat API"""
        try:
            return self.evaluate_tcp_against_draft(recipe=recipe, planned_tcp=planned_tcp, executed_tcp=executed_tcp)
        except Exception as e:
            return {"valid": False, "invalid_reason": str(e), "score": 0.0, "version": 3}

    def evaluate(self, *, recipe: Any, planned: Optional[Dict[str, Any]], executed: Optional[Dict[str, Any]]) -> Dict[str, Any]:
        return self.evaluate_runs(recipe=recipe, planned_tcp=planned, executed_tcp=executed)