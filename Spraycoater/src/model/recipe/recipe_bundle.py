# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_bundle.py
from __future__ import annotations

import hashlib
import json
import os
import re
from dataclasses import dataclass
from typing import Optional

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
    recipe_id: str
    recipe_name: str

    @property
    def folder(self) -> str:
        # NEW layout: data/recipes/<recipe_id>/<recipe_name>/
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


class RecipeBundle:
    """
    Storage Layout (NEU):
      data/recipes/<recipe_id>/<recipe_name>/
        recipe.yaml         (editor draft: parameters, paths_by_side, planner...)
        compiled.yaml       (paths_compiled)
        traj.yaml           (trajectories['traj'])
        executed_traj.yaml  (trajectories['executed_traj'])

    Hinweis:
      - "key" ist ab jetzt "<recipe_id>/<recipe_name>"
      - damit kannst du pro recipe_id mehrere Rezepte speichern
    """

    def __init__(self, *, recipes_root_dir: str) -> None:
        self.root_dir = os.path.abspath(recipes_root_dir)

    # -------------------- key helpers -------------------- #

    @staticmethod
    def split_key(key: str) -> tuple[str, str]:
        """
        Erwartet "recipe_id/recipe_name".
        Legacy:
          - wenn kein "/" drin: interpretieren als recipe_id UND recipe_name identisch
        """
        key = (key or "").strip()
        if not key:
            raise ValueError("Recipe key ist leer.")
        if "/" not in key:
            rid = _safe_name(key)
            return rid, rid
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
        """
        Gibt Keys "recipe_id/recipe_name" zurück.
        """
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

        # Draft-Recipe: ID bleibt im YAML dein "id" (typisch recipe_id)
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

        # optional extras entfernen
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
        # Ordner löschen
        try:
            os.rmdir(p.folder)
        except OSError:
            # fallback walk
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
        p = self.paths(key)
        r = self.load_draft(key)

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

        if r.paths_compiled:
            try:
                r._recompute_info_from_compiled()
            except Exception:
                pass

        return r

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

        # normalize meta/id
        draft.id = rid
        draft.meta = dict(draft.meta or {})
        draft.meta["recipe_name"] = rname
        draft.meta["recipe_id"] = rid

        old_hash = str((draft.info or {}).get("paths_hash") or "")
        new_hash = _hash_paths_by_side(draft.paths_by_side or {})
        draft.info = dict(draft.info or {})
        draft.info["paths_hash"] = new_hash

        hash_changed = bool(old_hash) and (old_hash != new_hash)

        # policy: editor save invalidates old traj/executed
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)

        if hash_changed and delete_compiled_on_hash_change and compiled is None:
            self._try_remove(p.compiled_yaml)

        # write draft (no traj/compiled inside)
        draft.trajectories = {}
        draft.paths_compiled = {}
        draft.save_yaml(p.editor_yaml)

        # write compiled if provided
        if isinstance(compiled, dict):
            tmp = Recipe(id=rid)
            tmp.meta = {"recipe_name": rname, "recipe_id": rid}
            tmp.paths_compiled = compiled
            tmp.save_yaml(p.compiled_yaml)

    def save_traj(self, key: str, *, traj: dict) -> None:
        p = self.paths(key)
        os.makedirs(p.folder, exist_ok=True)
        rid, rname = self.split_key(key)
        r = Recipe(id=rid)
        r.meta = {"recipe_name": rname, "recipe_id": rid}
        r.trajectories = {Recipe.TRAJ_TRAJ: traj}
        r.save_yaml(p.traj_yaml)

    def save_executed(self, key: str, *, executed: dict) -> None:
        p = self.paths(key)
        os.makedirs(p.folder, exist_ok=True)
        rid, rname = self.split_key(key)
        r = Recipe(id=rid)
        r.meta = {"recipe_name": rname, "recipe_id": rid}
        r.trajectories = {Recipe.TRAJ_EXECUTED: executed}
        r.save_yaml(p.executed_yaml)

    def clear_runs(self, key: str) -> None:
        p = self.paths(key)
        self._try_remove(p.traj_yaml)
        self._try_remove(p.executed_yaml)

    def clear_compiled(self, key: str) -> None:
        p = self.paths(key)
        self._try_remove(p.compiled_yaml)
