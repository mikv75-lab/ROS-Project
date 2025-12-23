# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe.py
from __future__ import annotations

import os
import hashlib
import json
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Tuple

import yaml
import numpy as np

from model.recipe.recipe_eval import RecipeEvaluator


# ============================================================
# Recipe
# ============================================================

@dataclass
class Recipe:
    """
    Persistentes Rezeptmodell (SSoT).

    Bundle-Philosophie:
      - draft.yaml          -> Editorzustand (paths_by_side etc.)
      - compiled_path.yaml  -> Referenz (paths_compiled + info)
      - traj.yaml           -> geplanter Pfad (trajectories["traj"])
      - executed_traj.yaml  -> letzter Ist-Pfad (trajectories["executed_traj"])

    Keine History. Immer nur "letzter Stand".
    """

    # Keys
    TRAJ_COMPILED: str = "compiled_path"   # Alias auf paths_compiled (nicht in trajectories)
    TRAJ_TRAJ: str = "traj"                # geplant (Soll)
    TRAJ_EXECUTED: str = "executed_traj"   # ausgeführt (Ist)

    # Meta
    id: str = "recipe"
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None
    substrate_mount: Optional[str] = None

    parameters: Dict[str, Any] = field(default_factory=dict)
    planner: Dict[str, Any] = field(default_factory=dict)
    paths_by_side: Dict[str, Dict[str, Any]] = field(default_factory=dict)

    # Hashes etc. (vom Bundle-Manager gepflegt)
    meta: Dict[str, Any] = field(default_factory=dict)

    # compiled reference
    paths_compiled: Dict[str, Any] = field(default_factory=dict)
    info: Dict[str, Any] = field(default_factory=dict)

    # last planned/executed
    trajectories: Dict[str, Any] = field(default_factory=dict)

    # ---------- YAML ----------

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        traj = d.get("trajectories") or {}
        if traj and not isinstance(traj, dict):
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
            meta=dict(d.get("meta") or {}),
            paths_compiled=dict(d.get("paths_compiled") or {}),
            info=dict(d.get("info") or {}),
            trajectories=dict(traj or {}),
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
        if self.meta:
            out["meta"] = Recipe._to_plain(self.meta or {})

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

    # ---------- Hashing (bundle uses it) ----------

    @staticmethod
    def _stable_hash(obj: Any) -> str:
        """
        Stabiler Hash über "plain" JSON (sort_keys) – geeignet für
        change detection (paths changed? compiled changed?).
        """
        plain = Recipe._to_plain(obj)
        data = json.dumps(plain, sort_keys=True, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        return hashlib.sha256(data).hexdigest()

    def compute_paths_hash(self) -> str:
        return self._stable_hash(self.paths_by_side or {})

    def compute_compiled_hash(self) -> str:
        return self._stable_hash(self.paths_compiled or {})

    # ---------- Helpers ----------

    def _assert_valid_traj_key(self, traj_key: str) -> str:
        if traj_key == self.TRAJ_COMPILED:
            raise ValueError("compiled_path ist Alias auf paths_compiled")
        if traj_key not in (self.TRAJ_TRAJ, self.TRAJ_EXECUTED):
            raise ValueError(f"Invalid traj_key '{traj_key}'")
        return traj_key

    def clear_trajectories(self) -> None:
        """Löscht geplante + ausgeführte Traj aus dem Modell (in-memory)."""
        self.trajectories.pop(self.TRAJ_TRAJ, None)
        self.trajectories.pop(self.TRAJ_EXECUTED, None)

    def clear_compiled(self) -> None:
        """Löscht compiled Reference (in-memory)."""
        self.paths_compiled.clear()
        self.info.clear()

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

        self.info = {"total_points": total_points, "total_length_mm": total_length, "sides": sides_info}
