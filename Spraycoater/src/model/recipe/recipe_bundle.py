# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_bundle.py
from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass
from typing import Optional, Tuple

from model.recipe.recipe import Recipe


def _safe_name(name: str) -> str:
    name = (name or "").strip()
    if not name:
        raise ValueError("Recipe name ist leer.")
    # windows+linux friendly
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
    name: str

    @property
    def folder(self) -> str:
        return os.path.join(self.root_dir, self.name)

    @property
    def editor_yaml(self) -> str:
        # Draft/Editor source of truth (no traj/executed stored here)
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


class RecipeBundle:
    """
    Storage Layout:
      data/recipes/<name>/
        recipe.yaml         (editor draft: meta, parameters, paths_by_side, planner...)
        compiled.yaml       (paths_compiled)
        traj.yaml           (trajectories['traj'])
        executed_traj.yaml  (trajectories['executed_traj'])

    Policy:
      - Editor save löscht IMMER traj + executed.
      - Wenn paths_hash changed: optional zusätzlich compiled löschen (default: True).
    """

    def __init__(self, *, recipes_root_dir: str) -> None:
        self.root_dir = os.path.abspath(recipes_root_dir)

    # -------------------- filesystem helpers -------------------- #

    def paths(self, name: str) -> RecipePaths:
        return RecipePaths(self.root_dir, _safe_name(name))

    def list_names(self) -> list[str]:
        if not os.path.isdir(self.root_dir):
            return []
        out: list[str] = []
        for e in os.listdir(self.root_dir):
            p = os.path.join(self.root_dir, e)
            if os.path.isdir(p) and os.path.isfile(os.path.join(p, "recipe.yaml")):
                out.append(e)
        out.sort()
        return out

    def exists(self, name: str) -> bool:
        p = self.paths(name)
        return os.path.isfile(p.editor_yaml)

    def create_new(self, name: str, *, base: Optional[Recipe] = None, overwrite: bool = False) -> Recipe:
        p = self.paths(name)
        os.makedirs(p.folder, exist_ok=True)
        if (not overwrite) and os.path.exists(p.editor_yaml):
            raise FileExistsError(f"Recipe '{name}' existiert bereits.")

        r = base if isinstance(base, Recipe) else Recipe(id=_safe_name(name))
        r.id = _safe_name(name)
        r.trajectories = {}  # editor draft has no traj/executed persisted
        r.paths_compiled = {}
        r.info = {}
        r.save_yaml(p.editor_yaml)
        # hard-clear optional extras
        self._try_remove(p.compiled_yaml)
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)
        return r

    def delete(self, name: str) -> None:
        p = self.paths(name)
        if not os.path.isdir(p.folder):
            return
        # remove all files in folder then folder
        for fn in (p.editor_yaml, p.compiled_yaml, p.traj_yaml, p.executed_yaml):
            self._try_remove(fn)
        # also remove unknown leftovers
        for e in os.listdir(p.folder):
            self._try_remove(os.path.join(p.folder, e))
        try:
            os.rmdir(p.folder)
        except OSError:
            # folder not empty (maybe subdirs) -> do a conservative walk
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

    def load_draft(self, name: str) -> Recipe:
        """
        Editor-Load: nur recipe.yaml (draft).
        """
        p = self.paths(name)
        if not os.path.isfile(p.editor_yaml):
            raise FileNotFoundError(f"Recipe '{name}' nicht gefunden.")
        r = Recipe.load_yaml(p.editor_yaml)
        # ensure id is folder name (SSoT)
        r.id = _safe_name(name)
        return r

    def load_full(self, name: str) -> Recipe:
        """
        Process-Load: draft + compiled + traj + executed, alles merged in eine Recipe-Instanz.
        """
        p = self.paths(name)
        r = self.load_draft(name)

        if os.path.isfile(p.compiled_yaml):
            c = Recipe.load_yaml(p.compiled_yaml)
            r.paths_compiled = c.paths_compiled or {}

        if os.path.isfile(p.traj_yaml):
            t = Recipe.load_yaml(p.traj_yaml)
            r.trajectories.setdefault(Recipe.TRAJ_TRAJ, {})
            r.trajectories[Recipe.TRAJ_TRAJ] = (t.trajectories.get(Recipe.TRAJ_TRAJ) or {})

        if os.path.isfile(p.executed_yaml):
            e = Recipe.load_yaml(p.executed_yaml)
            r.trajectories.setdefault(Recipe.TRAJ_EXECUTED, {})
            r.trajectories[Recipe.TRAJ_EXECUTED] = (e.trajectories.get(Recipe.TRAJ_EXECUTED) or {})

        # update info if we have compiled
        if r.paths_compiled:
            # uses internal helper
            try:
                r._recompute_info_from_compiled()
            except Exception:
                pass

        return r

    # -------------------- save policies -------------------- #

    def save_editor(
        self,
        name: str,
        *,
        draft: Recipe,
        compiled: Optional[dict] = None,
        delete_compiled_on_hash_change: bool = True,
    ) -> None:
        """
        Editor save:
          - writes recipe.yaml (draft)
          - optionally writes compiled.yaml (paths_compiled)
          - ALWAYS deletes traj.yaml + executed_traj.yaml (old runs become invalid)
          - if paths_hash changed: optionally delete compiled.yaml too (if no new compiled provided or if you want hard reset)
        """
        p = self.paths(name)
        os.makedirs(p.folder, exist_ok=True)

        # normalize id
        draft.id = _safe_name(name)

        old_hash = str((draft.info or {}).get("paths_hash") or "")
        new_hash = _hash_paths_by_side(draft.paths_by_side or {})
        draft.info = dict(draft.info or {})
        draft.info["paths_hash"] = new_hash

        hash_changed = bool(old_hash) and (old_hash != new_hash)

        # policy: any editor save invalidates old traj/executed
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)

        if hash_changed and delete_compiled_on_hash_change and compiled is None:
            # editor changed paths but didn't provide new compiled -> compiled is now stale
            self._try_remove(p.compiled_yaml)

        # write draft
        # ensure draft itself does not persist traj/executed
        draft.trajectories = {}
        draft.paths_compiled = {}
        draft.save_yaml(p.editor_yaml)

        # write compiled if provided
        if isinstance(compiled, dict):
            tmp = Recipe(id=draft.id)
            tmp.paths_compiled = compiled
            tmp.save_yaml(p.compiled_yaml)

    def save_traj(self, name: str, *, traj: dict) -> None:
        """
        Writes traj.yaml as trajectories['traj'].
        """
        p = self.paths(name)
        os.makedirs(p.folder, exist_ok=True)
        r = Recipe(id=_safe_name(name))
        r.trajectories = {Recipe.TRAJ_TRAJ: traj}
        r.save_yaml(p.traj_yaml)

    def save_executed(self, name: str, *, executed: dict) -> None:
        """
        Writes executed_traj.yaml as trajectories['executed_traj'].
        """
        p = self.paths(name)
        os.makedirs(p.folder, exist_ok=True)
        r = Recipe(id=_safe_name(name))
        r.trajectories = {Recipe.TRAJ_EXECUTED: executed}
        r.save_yaml(p.executed_yaml)

    def clear_runs(self, name: str) -> None:
        """
        Manuell: löscht traj + executed (z.B. "Reset run").
        """
        p = self.paths(name)
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)

    def clear_compiled(self, name: str) -> None:
        p = self.paths(name)
        self._try_remove(p.compiled_yaml)
