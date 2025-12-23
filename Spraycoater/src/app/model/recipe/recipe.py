# -*- coding: utf-8 -*-
# app/model/recipe/recipe.py
from __future__ import annotations

import os
import math
from dataclasses import dataclass, field
from typing import Any, Dict, Iterable, List, Optional, Tuple

import yaml
import numpy as np


# ============================================================
# Eval
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


class RecipeEvaluator:
    """
    Vergleicht Test-Pfad gegen Referenz (mm).
    O(N*M) NN ist bewusst simpel; reicht für <= ~2000 Punkte gut.
    """

    def __init__(
        self,
        *,
        weights: Optional[EvalWeights] = None,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),
    ) -> None:
        self.weights = weights or EvalWeights()
        self.clamp_mean, self.clamp_p95, self.clamp_max = [float(x) for x in clamp_mm]

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

        metrics.update(
            {
                "mean_nn_mm": mean_sym,
                "p95_nn_mm": p95_sym,
                "max_nn_mm": max_sym,
            }
        )

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


# ============================================================
# Recipe
# ============================================================

@dataclass
class Recipe:
    """
    Persistentes Rezeptmodell (SSoT).

    Gespeichert:
      - paths_compiled
      - trajectories["traj"]          (Soll, Geometrie + Score)
      - trajectories["executed_traj"] (Ist,  Geometrie + Score)

    Keine History. Kein validate/optimize/editor.
    """

    # Keys
    TRAJ_COMPILED: str = "compiled_path"     # Alias auf paths_compiled
    TRAJ_TRAJ: str = "traj"                  # geplant (Soll)
    TRAJ_EXECUTED: str = "executed_traj"     # ausgeführt (Ist)

    # Meta
    id: str = "recipe"
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrate_mount: Optional[str] = None

    parameters: Dict[str, Any] = field(default_factory=dict)
    planner: Dict[str, Any] = field(default_factory=dict)
    paths_by_side: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    _paths_cache: Dict[str, np.ndarray] = field(default_factory=dict, repr=False, compare=False)

    paths_compiled: Dict[str, Any] = field(default_factory=dict)
    info: Dict[str, Any] = field(default_factory=dict)

    trajectories: Dict[str, Any] = field(default_factory=dict)

    # ---------- YAML ----------

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        traj = d.get("trajectories") or {}
        if not isinstance(traj, dict):
            raise TypeError("Recipe: trajectories must be a dict")

        return Recipe(
            id=str(d.get("id") or "recipe"),
            description=str(d.get("description") or ""),
            tool=d.get("tool"),
            substrate=d.get("substrate"),
            substrate_mount=d.get("substrate_mount"),
            parameters=dict(d.get("parameters") or {}),
            planner=dict(d.get("planner") or {}),
            paths_by_side=dict(d.get("paths_by_side") or {}),
            paths_compiled=dict(d.get("paths_compiled") or {}),
            info=dict(d.get("info") or {}),
            trajectories=dict(traj),
        )

    @staticmethod
    def _to_plain(obj: Any) -> Any:
        if isinstance(obj, np.generic):
            return obj.item()
        if isinstance(obj, (str, bool, int, float)) or obj is None:
            return obj
        if isinstance(obj, dict):
            return {str(k): Recipe._to_plain(v) for k, v in obj.items()}
        if isinstance(obj, (list, tuple)):
            return [Recipe._to_plain(x) for x in obj]
        if isinstance(obj, np.ndarray):
            return [Recipe._to_plain(x) for x in obj.tolist()]
        return str(obj)

    def to_dict(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrate_mount": self.substrate_mount,
            "parameters": Recipe._to_plain(self.parameters or {}),
            "planner": Recipe._to_plain(self.planner or {}),
            "paths_by_side": Recipe._to_plain(self.paths_by_side or {}),
        }

        if self.paths_compiled:
            out["paths_compiled"] = Recipe._to_plain(self.paths_compiled)
        if self.info:
            out["info"] = Recipe._to_plain(self.info)

        traj_out: Dict[str, Any] = {}
        for k in (self.TRAJ_TRAJ, self.TRAJ_EXECUTED):
            if isinstance(self.trajectories.get(k), dict):
                traj_out[k] = Recipe._to_plain(self.trajectories[k])
        if traj_out:
            out["trajectories"] = traj_out

        return out

    @staticmethod
    def load_yaml(path: str) -> "Recipe":
        with open(path, "r", encoding="utf-8") as f:
            return Recipe.from_dict(yaml.safe_load(f) or {})

    def save_yaml(self, path: str) -> None:
        if self.paths_compiled:
            self._recompute_info_from_compiled()
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True, sort_keys=False)

    # ---------- Helpers ----------

    def _assert_valid_traj_key(self, traj_key: str) -> str:
        if traj_key == self.TRAJ_COMPILED:
            raise ValueError("compiled_path ist Alias auf paths_compiled")
        if traj_key not in (self.TRAJ_TRAJ, self.TRAJ_EXECUTED):
            raise ValueError(f"Invalid traj_key '{traj_key}'")
        return traj_key

    # ---------- Points ----------

    def compiled_points_mm_for_side(self, side: str) -> Optional[np.ndarray]:
        sdata = (self.paths_compiled.get("sides") or {}).get(str(side))
        if not isinstance(sdata, dict):
            return None
        poses = sdata.get("poses_quat") or []
        if not poses:
            return None
        return np.array([[p["x"], p["y"], p["z"]] for p in poses], dtype=float)

    def trajectory_points_mm_for_side(self, traj_key: str, side: str) -> Optional[np.ndarray]:
        if traj_key == self.TRAJ_COMPILED:
            return self.compiled_points_mm_for_side(side)
        t = self.trajectories.get(traj_key)
        if not isinstance(t, dict):
            return None
        sdata = (t.get("sides") or {}).get(str(side))
        if not isinstance(sdata, dict):
            return None
        poses = sdata.get("poses_quat") or []
        if not poses:
            return None
        return np.array([[p["x"], p["y"], p["z"]] for p in poses], dtype=float)

    # ---------- Setters ----------

    def set_trajectory_points_mm(
        self,
        *,
        traj_key: str,
        side: str,
        points_mm: np.ndarray,
        frame: str = "scene",
        tool_frame: Optional[str] = None,
        meta: Optional[Dict[str, Any]] = None,
    ) -> None:
        traj_key = self._assert_valid_traj_key(traj_key)
        P = np.asarray(points_mm, float).reshape(-1, 3)

        t = self.trajectories.setdefault(traj_key, {})
        t["frame"] = frame
        t["tool_frame"] = tool_frame or self.planner.get("tool_frame", "tool_mount")
        t.setdefault("sides", {})[str(side)] = {
            "poses_quat": [
                {"x": float(x), "y": float(y), "z": float(z), "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
                for x, y, z in P
            ]
        }
        if meta:
            t["meta"] = dict(meta)

    # ---------- Eval ----------

    def evaluate_trajectory_against_compiled(
        self,
        *,
        traj_key: str,
        side: str,
        clamp_mm: Tuple[float, float, float] = (1.0, 3.0, 8.0),
    ) -> Optional[Dict[str, Any]]:
        traj_key = self._assert_valid_traj_key(traj_key)

        ref = self.compiled_points_mm_for_side(side)
        test = self.trajectory_points_mm_for_side(traj_key, side)
        if ref is None or test is None:
            return None

        ev = RecipeEvaluator(clamp_mm=clamp_mm)
        res = ev.evaluate_points_mm(
            ref_points_mm=ref,
            test_points_mm=test,
            label=f"{traj_key}/{side}",
        ).to_dict()

        self.trajectories.setdefault(traj_key, {}).setdefault("sides", {}).setdefault(str(side), {})["eval"] = res
        return res

    # ---------- Info ----------

    def _recompute_info_from_compiled(self) -> None:
        sides = self.paths_compiled.get("sides") or {}
        total_points = 0
        total_length = 0.0
        sides_info: Dict[str, Any] = {}

        for side, sdata in sides.items():
            poses = (sdata or {}).get("poses_quat") or []
            P = np.array([[p["x"], p["y"], p["z"]] for p in poses], dtype=float)
            num = int(P.shape[0])
            length = float(np.linalg.norm(P[1:] - P[:-1], axis=1).sum()) if num >= 2 else 0.0
            sides_info[str(side)] = {"num_points": num, "length_mm": length}
            total_points += num
            total_length += length

        self.info = {
            "total_points": total_points,
            "total_length_mm": total_length,
            "sides": sides_info,
        }
