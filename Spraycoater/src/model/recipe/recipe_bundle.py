# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_bundle.py
from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass
from typing import Optional, Dict, Any, Literal

from model.recipe.recipe import Recipe


def _safe_name(name: str) -> str:
    name = (name or "").strip()
    if not name:
        raise ValueError("Recipe name ist leer.")
    name = re.sub(r"[^a-zA-Z0-9_\-\.]+", "_", name)
    return name


def _hash_paths_by_side(paths_by_side: dict) -> str:
    try:
        payload = json.dumps(paths_by_side or {}, sort_keys=True, separators=(",", ":")).encode("utf-8")
    except Exception:
        payload = repr(paths_by_side).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


@dataclass(frozen=True)
class RecipePaths:
    root_dir: str
    recipe_id: str
    recipe_name: str

    @property
    def folder(self) -> str:
        return os.path.join(self.root_dir, self.recipe_id, self.recipe_name)

    @property
    def editor_yaml(self) -> str:
        return os.path.join(self.folder, "recipe.yaml")

    @property
    def compiled_yaml(self) -> str:
        return os.path.join(self.folder, "compiled.yaml")

    # NEW: planned + executed split (JOINT-only runs)
    @property
    def planned_yaml(self) -> str:
        return os.path.join(self.folder, "planned_traj.yaml")

    @property
    def executed_yaml(self) -> str:
        return os.path.join(self.folder, "executed_traj.yaml")

    # LEGACY alias (older code may still call it)
    @property
    def traj_yaml(self) -> str:
        # keep compatibility: "traj.yaml" is now "planned_traj.yaml"
        return self.planned_yaml


# NOTE: accept legacy "traj" as alias for "planned"
RunKind = Literal["planned", "executed", "traj"]


class RecipeBundle:
    """
    STRICT storage (JOINT-only), deterministic replay ready.

    Files:
      - planned_traj.yaml   -> trajectories[Recipe.TRAJ_TRAJ]       = SegmentRunPayload (JOINT-only)
      - executed_traj.yaml  -> trajectories[Recipe.TRAJ_EXECUTED]   = SegmentRunPayload (JOINT-only)

    SegmentRunPayload (JOINT-only):
      {
        "version": 1,
        "meta": {...},
        "segments": {
          "<STATE>": {
             "joint": {...},        # required for persisted segments (JointTrajectory dict)
             "eval": {...},         # optional
             "meta": {...},         # optional
          },
          ...
        }
      }

    Wichtig:
      - TCP wird NICHT gespeichert (kein "tcp" key erlaubt).
      - Timestamps sind Teil der JointTrajectory points[*].time_from_start (sec/nanosec).
    """

    def __init__(self, *, recipes_root_dir: str) -> None:
        self.root_dir = os.path.abspath(recipes_root_dir)

    # -------------------- key helpers -------------------- #

    @staticmethod
    def split_key(key: str) -> tuple[str, str]:
        key = (key or "").strip()
        if not key:
            raise ValueError("Recipe key ist leer.")
        if "/" not in key:
            raise ValueError("Recipe key muss 'recipe_id/recipe_name' sein (strict).")
        rid, rname = key.split("/", 1)
        rid = _safe_name(rid)
        rname = _safe_name(rname)
        return rid, rname

    def make_key(self, recipe_id: str, recipe_name: str) -> str:
        return f"{_safe_name(recipe_id)}/{_safe_name(recipe_name)}"

    def paths(self, key: str) -> RecipePaths:
        rid, rname = self.split_key(key)
        return RecipePaths(self.root_dir, rid, rname)

    # -------------------- listing -------------------- #

    def list_names(self) -> list[str]:
        if not os.path.isdir(self.root_dir):
            return []
        out: list[str] = []
        for rid in os.listdir(self.root_dir):
            p_rid = os.path.join(self.root_dir, rid)
            if not os.path.isdir(p_rid):
                continue
            for rname in os.listdir(p_rid):
                p = os.path.join(p_rid, rname)
                if os.path.isdir(p) and os.path.isfile(os.path.join(p, "recipe.yaml")):
                    out.append(f"{rid}/{rname}")
        out.sort()
        return out

    def exists(self, key: str) -> bool:
        p = self.paths(key)
        return os.path.isfile(p.editor_yaml)

    # -------------------- create/delete -------------------- #

    def create_new(self, key: str, *, base: Optional[Recipe] = None, overwrite: bool = False) -> Recipe:
        p = self.paths(key)
        os.makedirs(p.folder, exist_ok=True)
        if (not overwrite) and os.path.exists(p.editor_yaml):
            raise FileExistsError(f"Recipe '{key}' existiert bereits.")

        rid, rname = self.split_key(key)
        r = base if isinstance(base, Recipe) else Recipe(id=rid)
        r.id = rid
        r.meta = dict(r.meta or {})
        r.meta["recipe_name"] = rname
        r.meta["recipe_id"] = rid

        r.trajectories = {}
        r.paths_compiled = {}
        r.info = {}
        r.save_yaml(p.editor_yaml)

        self._try_remove(p.compiled_yaml)
        self._try_remove(p.planned_yaml)
        self._try_remove(p.executed_yaml)

        # remove historic legacy name if it exists somewhere
        self._try_remove(os.path.join(p.folder, "traj.yaml"))

        return r

    def delete(self, key: str) -> None:
        p = self.paths(key)
        if not os.path.isdir(p.folder):
            return

        # remove known files (incl. legacy "traj.yaml")
        for fn in (p.editor_yaml, p.compiled_yaml, p.planned_yaml, p.executed_yaml, os.path.join(p.folder, "traj.yaml")):
            self._try_remove(fn)

        for e in os.listdir(p.folder):
            self._try_remove(os.path.join(p.folder, e))
        try:
            os.rmdir(p.folder)
        except OSError:
            for root, dirs, files in os.walk(p.folder, topdown=False):
                for f in files:
                    self._try_remove(os.path.join(root, f))
                for d in dirs:
                    try:
                        os.rmdir(os.path.join(root, d))
                    except OSError:
                        pass
            try:
                os.rmdir(p.folder)
            except OSError:
                pass

    @staticmethod
    def _try_remove(path: str) -> None:
        try:
            if os.path.exists(path):
                os.remove(path)
        except Exception:
            pass

    # -------------------- load -------------------- #

    def load_draft(self, key: str) -> Recipe:
        p = self.paths(key)
        if not os.path.isfile(p.editor_yaml):
            raise FileNotFoundError(f"Recipe '{key}' nicht gefunden.")
        r = Recipe.load_yaml(p.editor_yaml)

        rid, rname = self.split_key(key)
        r.id = rid
        r.meta = dict(r.meta or {})
        r.meta["recipe_name"] = rname
        r.meta["recipe_id"] = rid
        return r

    def load_full(self, key: str) -> Recipe:
        """
        Lädt draft + optional compiled + optional planned + optional executed.
        STRICT: run payload muss SegmentRunPayload (JOINT-only) sein.
        """
        p = self.paths(key)
        r = self.load_draft(key)

        if os.path.isfile(p.compiled_yaml):
            c = Recipe.load_yaml(p.compiled_yaml)
            r.paths_compiled = c.paths_compiled or {}

        # planned (preferred new filename)
        planned_path = p.planned_yaml
        legacy_traj_path = os.path.join(p.folder, "traj.yaml")  # older deployments might still have this file
        if os.path.isfile(planned_path):
            t = Recipe.load_yaml(planned_path)
            payload = (t.trajectories.get(Recipe.TRAJ_TRAJ) or {})
            self._validate_run_payload(payload, context=f"{planned_path}:{Recipe.TRAJ_TRAJ}")
            r.trajectories[Recipe.TRAJ_TRAJ] = payload
        elif os.path.isfile(legacy_traj_path):
            # optional legacy import
            t = Recipe.load_yaml(legacy_traj_path)
            payload = (t.trajectories.get(Recipe.TRAJ_TRAJ) or {})
            self._validate_run_payload(payload, context=f"{legacy_traj_path}:{Recipe.TRAJ_TRAJ}")
            r.trajectories[Recipe.TRAJ_TRAJ] = payload

        # executed
        if os.path.isfile(p.executed_yaml):
            e = Recipe.load_yaml(p.executed_yaml)
            payload = (e.trajectories.get(Recipe.TRAJ_EXECUTED) or {})
            self._validate_run_payload(payload, context=f"{p.executed_yaml}:{Recipe.TRAJ_EXECUTED}")
            r.trajectories[Recipe.TRAJ_EXECUTED] = payload

        if r.paths_compiled:
            try:
                r._recompute_info_from_compiled()
            except Exception:
                pass

        return r

    # -------------------- validation -------------------- #

    @staticmethod
    def _require_dict(x: Any, name: str, context: str) -> Dict[str, Any]:
        if not isinstance(x, dict):
            raise ValueError(f"{context}: '{name}' muss dict sein, ist {type(x).__name__}")
        return x

    @staticmethod
    def _require_int(x: Any, name: str, context: str) -> int:
        if not isinstance(x, int):
            raise ValueError(f"{context}: '{name}' muss int sein, ist {type(x).__name__}")
        return x

    @staticmethod
    def _require_list(x: Any, name: str, context: str) -> list:
        if not isinstance(x, list):
            raise ValueError(f"{context}: '{name}' muss list sein, ist {type(x).__name__}")
        return x

    def _validate_joint_trajectory(self, jt: Any, *, context: str) -> None:
        jt = self._require_dict(jt, "joint", context)

        joint_names = jt.get("joint_names", None)
        points = jt.get("points", None)

        self._require_list(joint_names, "joint.joint_names", context)
        self._require_list(points, "joint.points", context)

        if not joint_names:
            raise ValueError(f"{context}: joint.joint_names darf nicht leer sein")
        if not points:
            raise ValueError(f"{context}: joint.points darf nicht leer sein")

        # minimal checks for replay: time_from_start present and numeric
        for i, p in enumerate(points):
            if not isinstance(p, dict):
                raise ValueError(f"{context}: joint.points[{i}] muss dict sein")
            tfs = p.get("time_from_start", None)
            if not isinstance(tfs, dict):
                raise ValueError(f"{context}: joint.points[{i}].time_from_start muss dict sein")
            if "sec" not in tfs or "nanosec" not in tfs:
                raise ValueError(f"{context}: joint.points[{i}].time_from_start braucht sec/nanosec")
            try:
                int(tfs.get("sec"))
                int(tfs.get("nanosec"))
            except Exception:
                raise ValueError(f"{context}: joint.points[{i}].time_from_start sec/nanosec müssen int sein")

    def _validate_run_payload(self, payload: Any, *, context: str) -> None:
        payload = self._require_dict(payload, "payload", context)

        ver = self._require_int(payload.get("version", None), "version", context)
        if ver != 1:
            raise ValueError(f"{context}: version muss 1 sein, ist {ver}")

        meta = payload.get("meta", {})
        self._require_dict(meta, "meta", context)

        segments = payload.get("segments", None)
        segments = self._require_dict(segments, "segments", context)
        if not segments:
            raise ValueError(f"{context}: segments darf nicht leer sein")

        # JOINT-ONLY strict: tcp is forbidden
        for seg_key, seg_val in segments.items():
            if not isinstance(seg_key, str) or not seg_key.strip():
                raise ValueError(f"{context}: segments key ungültig: {repr(seg_key)}")
            seg_val = self._require_dict(seg_val, f"segments['{seg_key}']", context)

            allowed = {"joint", "eval", "meta"}
            extra = set(seg_val.keys()) - allowed
            if extra:
                raise ValueError(f"{context}: segments['{seg_key}'] hat unbekannte keys: {sorted(extra)}")

            if "joint" not in seg_val:
                raise ValueError(f"{context}: segments['{seg_key}'] muss 'joint' enthalten (joint-only)")
            self._validate_joint_trajectory(seg_val["joint"], context=f"{context}:segments['{seg_key}'].joint")

            if "eval" in seg_val:
                self._require_dict(seg_val["eval"], f"segments['{seg_key}'].eval", context)
            if "meta" in seg_val:
                self._require_dict(seg_val["meta"], f"segments['{seg_key}'].meta", context)

        # forbid old/other top-level keys to keep schema clean
        forbidden = {
            "by_segment",
            "executed_by_segment",
            "traj",
            "executed_traj",
            "eval_total",
            "eval_by_segment",
            "tcp",
        }
        bad = forbidden.intersection(set(payload.keys()))
        if bad:
            raise ValueError(f"{context}: verbotene top-level Felder: {sorted(bad)}")

    # -------------------- save policies -------------------- #

    def save_editor(
        self,
        key: str,
        *,
        draft: Recipe,
        compiled: Optional[dict] = None,
        delete_compiled_on_hash_change: bool = True,
    ) -> None:
        p = self.paths(key)
        os.makedirs(p.folder, exist_ok=True)

        rid, rname = self.split_key(key)

        draft.id = rid
        draft.meta = dict(draft.meta or {})
        draft.meta["recipe_name"] = rname
        draft.meta["recipe_id"] = rid

        old_hash = str((draft.info or {}).get("paths_hash") or "")
        new_hash = _hash_paths_by_side(draft.paths_by_side or {})
        draft.info = dict(draft.info or {})
        draft.info["paths_hash"] = new_hash

        hash_changed = bool(old_hash) and (old_hash != new_hash)

        # editor save invalidates runs (planned + executed, plus legacy name)
        self._try_remove(p.planned_yaml)
        self._try_remove(p.executed_yaml)
        self._try_remove(os.path.join(p.folder, "traj.yaml"))

        if hash_changed and delete_compiled_on_hash_change and compiled is None:
            self._try_remove(p.compiled_yaml)

        draft.trajectories = {}
        draft.paths_compiled = {}
        draft.save_yaml(p.editor_yaml)

        if isinstance(compiled, dict):
            tmp = Recipe(id=rid)
            tmp.meta = {"recipe_name": rname, "recipe_id": rid}
            tmp.paths_compiled = compiled
            tmp.save_yaml(p.compiled_yaml)

    def save_run(self, key: str, *, kind: RunKind, run: dict) -> None:
        """
        Einziger Writer für runs.

        kind='planned' (oder legacy 'traj') schreibt planned_traj.yaml / trajectories[Recipe.TRAJ_TRAJ]
        kind='executed' schreibt executed_traj.yaml / trajectories[Recipe.TRAJ_EXECUTED]
        """
        p = self.paths(key)
        os.makedirs(p.folder, exist_ok=True)
        rid, rname = self.split_key(key)

        if kind == "traj":
            kind = "planned"

        self._validate_run_payload(run, context=f"save_run({key}, kind={kind})")

        if kind == "planned":
            file_path = p.planned_yaml
            traj_key = Recipe.TRAJ_TRAJ
        elif kind == "executed":
            file_path = p.executed_yaml
            traj_key = Recipe.TRAJ_EXECUTED
        else:
            raise ValueError(f"save_run: unknown kind={kind}")

        r = Recipe(id=rid)
        r.meta = {"recipe_name": rname, "recipe_id": rid}
        r.trajectories = {traj_key: run}
        r.save_yaml(file_path)

    def clear_runs(self, key: str) -> None:
        p = self.paths(key)
        self._try_remove(p.planned_yaml)
        self._try_remove(p.executed_yaml)
        self._try_remove(os.path.join(p.folder, "traj.yaml"))

    def clear_compiled(self, key: str) -> None:
        p = self.paths(key)
        self._try_remove(p.compiled_yaml)
