# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_bundle.py
from __future__ import annotations

import io
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple, List

import yaml

from .recipe import Draft, JTBySegment, Recipe


def _err(msg: str) -> None:
    raise ValueError(msg)


def _norm(p: str) -> str:
    return os.path.abspath(os.path.normpath(os.path.expanduser(p)))


def _ensure_dir(d: str) -> None:
    d = _norm(d)
    os.makedirs(d, exist_ok=True)


def _require_dir(d: str) -> str:
    d = _norm(d)
    if not os.path.isdir(d):
        _err(f"Verzeichnis existiert nicht: {d!r}")
    return d


def _load_yaml(path: str) -> Dict[str, Any]:
    path = _norm(path)
    if not os.path.isfile(path):
        _err(f"Datei existiert nicht: {path!r}")
    with io.open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        data = {}
    if not isinstance(data, dict):
        _err(f"YAML root muss dict sein: {path!r} -> {type(data).__name__}")
    return data


def _save_yaml(path: str, data: Dict[str, Any]) -> None:
    path = _norm(path)
    _ensure_dir(os.path.dirname(path) or ".")
    with io.open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(
            data,
            f,
            sort_keys=False,
            allow_unicode=True,
            default_flow_style=False,
        )


@dataclass(frozen=True)
class RecipePaths:
    recipe_dir: str
    params_yaml: str
    draft_yaml: str
    planned_traj_yaml: str
    executed_traj_yaml: str


class RecipeBundle:
    """
    Legacy persistence layout (SSoT):

      <recipes_root_dir>/
        <recipe_id>/
          params.yaml
          draft.yaml
          planned_traj.yaml
          executed_traj.yaml

    UI-compatibility:
      - UI may use keys "<rid>/<name>"
      - persistence is rid-only; name is ignored (treated as "default")
    """

    def __init__(
        self,
        *,
        recipes_root_dir: Optional[str] = None,
        root_dir: Optional[str] = None,
        **_ignored: Any,
    ):
        base = recipes_root_dir if recipes_root_dir is not None else root_dir
        if not base or not isinstance(base, str):
            _err("RecipeBundle: recipes_root_dir/root_dir muss ein nicht-leerer String sein.")
        self.recipes_root_dir = _require_dir(base)

        # aliases some code probes for
        self.root_dir = self.recipes_root_dir
        self.base_dir = self.recipes_root_dir
        self.recipe_dir = self.recipes_root_dir  # legacy name used in some places

    # ------------------------------------------------------------
    # UI key helpers
    # ------------------------------------------------------------

    def split_key(self, key: str) -> Tuple[str, str]:
        """
        Accept:
          - "rid"        -> ("rid", "default")
          - "rid/name"   -> ("rid", "name")   # name is ignored by persistence
        """
        key = str(key or "").strip()
        if not key:
            return "", ""
        if "/" in key:
            rid, name = key.split("/", 1)
            rid = str(rid or "").strip()
            name = str(name or "").strip() or "default"
            return rid, name
        return key, "default"

    def make_key(self, rid: str, name: str) -> str:
        """
        Persistence is rid-only; ignore name.
        """
        rid = str(rid or "").strip()
        if not rid:
            _err("make_key: rid leer")
        return rid

    def join_key(self, rid: str, name: str) -> str:
        return self.make_key(rid, name)

    # ------------------------------------------------------------
    # paths (legacy)
    # ------------------------------------------------------------

    def _recipe_dir(self, recipe_id: str) -> str:
        rid = str(recipe_id).strip()
        if not rid:
            _err("recipe_id muss ein nicht-leerer String sein.")
        return _norm(os.path.join(self.recipes_root_dir, rid))

    def paths(self, recipe_id: str) -> RecipePaths:
        d = self._recipe_dir(recipe_id)
        return RecipePaths(
            recipe_dir=d,
            params_yaml=os.path.join(d, "params.yaml"),
            draft_yaml=os.path.join(d, "draft.yaml"),
            planned_traj_yaml=os.path.join(d, "planned_traj.yaml"),
            executed_traj_yaml=os.path.join(d, "executed_traj.yaml"),
        )

    # ------------------------------------------------------------
    # lifecycle (legacy)
    # ------------------------------------------------------------

    def create_new(self, recipe_id: str, *, overwrite: bool = False) -> RecipePaths:
        p = self.paths(recipe_id)
        if os.path.exists(p.recipe_dir) and not overwrite:
            _err(f"Recipe existiert bereits: {p.recipe_dir}")
        _ensure_dir(p.recipe_dir)
        return p

    def delete(self, recipe_id: str) -> None:
        p = self.paths(recipe_id)
        if not os.path.isdir(p.recipe_dir):
            return
        for fn in (p.params_yaml, p.draft_yaml, p.planned_traj_yaml, p.executed_traj_yaml):
            if os.path.exists(fn):
                try:
                    os.remove(fn)
                except OSError:
                    pass
        try:
            os.rmdir(p.recipe_dir)
        except OSError:
            pass

    # ------------------------------------------------------------
    # params.yaml (strict)
    # ------------------------------------------------------------

    def load_params(self, recipe_id: str) -> Recipe:
        p = self.paths(recipe_id)
        data = _load_yaml(p.params_yaml)
        return Recipe.from_params_dict(data)

    def save_params(self, recipe: Recipe) -> None:
        p = self.paths(recipe.id)
        _ensure_dir(p.recipe_dir)
        _save_yaml(p.params_yaml, recipe.to_params_dict())

    # ------------------------------------------------------------
    # draft.yaml (strict)
    # ------------------------------------------------------------

    def load_draft(self, recipe_id: str) -> Draft:
        p = self.paths(recipe_id)
        data = _load_yaml(p.draft_yaml)
        return Draft.from_yaml_dict(data)

    def save_draft(self, recipe_id: str, draft: Draft) -> None:
        p = self.paths(recipe_id)
        _ensure_dir(p.recipe_dir)
        _save_yaml(p.draft_yaml, draft.to_yaml_dict())

    # ------------------------------------------------------------
    # planned_traj.yaml / executed_traj.yaml (strict)
    # ------------------------------------------------------------

    def load_planned_traj(self, recipe_id: str) -> Optional[JTBySegment]:
        p = self.paths(recipe_id)
        if not os.path.exists(p.planned_traj_yaml):
            return None
        data = _load_yaml(p.planned_traj_yaml)
        return JTBySegment.from_yaml_dict(data)

    def save_planned_traj(self, recipe_id: str, traj: JTBySegment) -> None:
        p = self.paths(recipe_id)
        _ensure_dir(p.recipe_dir)
        _save_yaml(p.planned_traj_yaml, traj.to_yaml_dict())

    def load_executed_traj(self, recipe_id: str) -> Optional[JTBySegment]:
        p = self.paths(recipe_id)
        if not os.path.exists(p.executed_traj_yaml):
            return None
        data = _load_yaml(p.executed_traj_yaml)
        return JTBySegment.from_yaml_dict(data)

    def save_executed_traj(self, recipe_id: str, traj: JTBySegment) -> None:
        p = self.paths(recipe_id)
        _ensure_dir(p.recipe_dir)
        _save_yaml(p.executed_traj_yaml, traj.to_yaml_dict())

    def clear_traj(self, recipe_id: str) -> None:
        p = self.paths(recipe_id)
        for fp in (p.planned_traj_yaml, p.executed_traj_yaml):
            if os.path.exists(fp):
                try:
                    os.remove(fp)
                except OSError:
                    pass

    # ------------------------------------------------------------
    # UI compatibility API (key-based) mapped to legacy
    # ------------------------------------------------------------

    def list_recipes(self) -> List[str]:
        """
        Returns a list of recipe_ids (keys for UI).
        """
        root = self.recipes_root_dir
        if not os.path.isdir(root):
            return []
        out: List[str] = []
        for rid in sorted(os.listdir(root)):
            d = os.path.join(root, rid)
            if not os.path.isdir(d):
                continue
            # Consider it a "recipe" if params.yaml exists (authoritative)
            if os.path.isfile(os.path.join(d, "params.yaml")):
                out.append(str(rid))
        return out

    def load_for_editor(self, key: str) -> Recipe:
        """
        UI editor load: params + (optional) draft.
        """
        rid, _name = self.split_key(key)
        rid = rid.strip()
        if not rid:
            _err(f"load_for_editor: invalid key={key!r}")

        recipe = self.load_params(rid)

        # draft optional for editor (UI may generate preview without it)
        p = self.paths(rid)
        if os.path.isfile(p.draft_yaml):
            recipe.draft = self.load_draft(rid)

        return recipe

    def load_for_process(self, key: str) -> Recipe:
        """
        Processing load: params + draft + optional trajectories.
        """
        rid, _name = self.split_key(key)
        rid = rid.strip()
        if not rid:
            _err(f"load_for_process: invalid key={key!r}")

        recipe = self.load_for_editor(rid)

        # planned/executed optional
        planned = self.load_planned_traj(rid)
        if planned is not None:
            recipe.planned_traj = planned
        executed = self.load_executed_traj(rid)
        if executed is not None:
            recipe.executed_traj = executed

        return recipe

    def save_from_editor(
        self,
        key: str,
        *,
        draft: Recipe,
        compiled: Optional[Dict[str, Any]] = None,
        delete_compiled_on_hash_change: bool = True,
    ) -> None:
        """
        Legacy-only save:
          - params.yaml always (authoritative)
          - draft.yaml if draft.draft exists AND is Draft
        compiled/delete_compiled_on_hash_change are ignored (no compiled.yaml in legacy).
        """
        rid, _name = self.split_key(key)
        rid = rid.strip()
        if not rid:
            _err("save_from_editor: rid/key leer")

        # Ensure recipe.id consistency
        draft.id = rid

        # params.yaml always
        self.save_params(draft)

        # draft.yaml optional (only if UI/preview has produced Draft)
        if isinstance(draft.draft, Draft):
            self.save_draft(rid, draft.draft)

    def delete_by_key(self, key: str) -> None:
        rid, _name = self.split_key(key)
        rid = rid.strip()
        if not rid:
            return
        self.delete(rid)
