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

    @property
    def traj_yaml(self) -> str:
        return os.path.join(self.folder, "traj.yaml")

    @property
    def executed_yaml(self) -> str:
        return os.path.join(self.folder, "executed_traj.yaml")


RunKind = Literal["traj", "executed"]


class RecipeBundle:
    """
    STRICT storage, segment-only. Traj und Executed sind identisch, nur anderer Speicherort.

    Files:
      - traj.yaml          -> trajectories[Recipe.TRAJ_TRAJ]       = SegmentRunPayload
      - executed_traj.yaml -> trajectories[Recipe.TRAJ_EXECUTED]   = SegmentRunPayload

    SegmentRunPayload:
      {
        "version": 1,
        "meta": {...},
        "segments": {
          "<STATE>": {
             "tcp": {...},          # optional
             "joint": {...},        # optional  (JointTrajectory als dict)
             "eval": {...},         # optional
          },
          ...
        }
      }

    Kein total. Kein legacy. Total wird aus segments zusammengesetzt.
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
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)
        return r

    def delete(self, key: str) -> None:
        p = self.paths(key)
        if not os.path.isdir(p.folder):
            return
        for fn in (p.editor_yaml, p.compiled_yaml, p.traj_yaml, p.executed_yaml):
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
        Lädt draft + optional compiled + optional traj + optional executed.
        STRICT: run payload muss SegmentRunPayload sein.
        """
        p = self.paths(key)
        r = self.load_draft(key)

        if os.path.isfile(p.compiled_yaml):
            c = Recipe.load_yaml(p.compiled_yaml)
            r.paths_compiled = c.paths_compiled or {}

        if os.path.isfile(p.traj_yaml):
            t = Recipe.load_yaml(p.traj_yaml)
            payload = (t.trajectories.get(Recipe.TRAJ_TRAJ) or {})
            self._validate_run_payload(payload, context=f"{p.traj_yaml}:{Recipe.TRAJ_TRAJ}")
            r.trajectories[Recipe.TRAJ_TRAJ] = payload

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

        # segment content checks (minimal strict)
        for seg_key, seg_val in segments.items():
            if not isinstance(seg_key, str) or not seg_key.strip():
                raise ValueError(f"{context}: segments key ungültig: {repr(seg_key)}")
            seg_val = self._require_dict(seg_val, f"segments['{seg_key}']", context)

            allowed = {"tcp", "joint", "eval", "meta"}
            extra = set(seg_val.keys()) - allowed
            if extra:
                raise ValueError(f"{context}: segments['{seg_key}'] hat unbekannte keys: {sorted(extra)}")

            if "tcp" in seg_val:
                self._require_dict(seg_val["tcp"], f"segments['{seg_key}'].tcp", context)
            if "joint" in seg_val:
                self._require_dict(seg_val["joint"], f"segments['{seg_key}'].joint", context)
            if "eval" in seg_val:
                self._require_dict(seg_val["eval"], f"segments['{seg_key}'].eval", context)
            if "meta" in seg_val:
                self._require_dict(seg_val["meta"], f"segments['{seg_key}'].meta", context)

        # forbid old/other top-level keys to keep schema clean
        forbidden = {"by_segment", "executed_by_segment", "traj", "executed_traj", "eval_total", "eval_by_segment"}
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

        # editor save invalidates runs
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)

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
        kind='traj' schreibt traj.yaml / trajectories[Recipe.TRAJ_TRAJ]
        kind='executed' schreibt executed_traj.yaml / trajectories[Recipe.TRAJ_EXECUTED]
        """
        p = self.paths(key)
        os.makedirs(p.folder, exist_ok=True)
        rid, rname = self.split_key(key)

        self._validate_run_payload(run, context=f"save_run({key}, kind={kind})")

        if kind == "traj":
            file_path = p.traj_yaml
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
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)

    def clear_compiled(self, key: str) -> None:
        p = self.paths(key)
        self._try_remove(p.compiled_yaml)
