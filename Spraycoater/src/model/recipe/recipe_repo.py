# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_repo.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Tuple, Optional

from model.recipe.recipe import Recipe
from model.recipe.recipe_bundle import RecipeBundle


@dataclass
class RecipeRepo:
    """
    Thin repo wrapper around RecipeBundle.

    Legacy SSoT (rid-only):
      <recipes_root_dir>/<recipe_id>/
        params.yaml
        draft.yaml
        planned_traj.yaml
        executed_traj.yaml

    UI contract (rid-only):
      - list_recipes() -> List[recipe_id]
      - load_for_editor(recipe_id) -> Recipe
      - load_for_process(recipe_id) -> Recipe
      - save_from_editor(recipe_id, draft=Recipe) -> None
      - delete(recipe_id) -> None

    Compatibility helpers:
      - make_key(rid, name) -> rid  (name ignored)
      - split_key(key) -> (rid, "")  (legacy)
    """

    bundle: RecipeBundle

    def __init__(self, *, bundle: RecipeBundle, **_ignored: Any) -> None:
        if bundle is None or not isinstance(bundle, RecipeBundle):
            raise TypeError(f"RecipeRepo: bundle invalid: {type(bundle)}")
        self.bundle = bundle

    # ------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------

    @staticmethod
    def _norm_rid(key_or_rid: str) -> str:
        """
        Normalizes incoming identifiers to rid-only.

        Accepts:
          - "rid"
          - "rid/" or "rid/something"  (we take the first path token)
        """
        s = str(key_or_rid or "").strip().strip("/")
        if not s:
            return ""
        return s.split("/")[0].strip()

    # ------------------------------------------------------------
    # Compatibility key helpers (rid-only)
    # ------------------------------------------------------------

    def split_key(self, key: str) -> Tuple[str, str]:
        rid = self._norm_rid(key)
        return rid, ""

    def make_key(self, rid: str, name: str = "") -> str:
        _ = name  # ignored (legacy rid-only)
        return self._norm_rid(rid)

    # ------------------------------------------------------------
    # Listing / Loading (rid-only)
    # ------------------------------------------------------------

    def list_recipes(self) -> List[str]:
        # Prefer bundle's native listing; normalize to rid-only
        keys = self.bundle.list_recipes() if hasattr(self.bundle, "list_recipes") else []
        out: List[str] = []
        for k in (keys or []):
            rid = self._norm_rid(k)
            if rid and rid not in out:
                out.append(rid)
        return sorted(out)

    def load_for_editor(self, recipe_id: str) -> Recipe:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("load_for_editor: recipe_id leer")
        return self.bundle.load_for_editor(rid)

    def load_for_process(self, recipe_id: str) -> Recipe:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("load_for_process: recipe_id leer")
        return self.bundle.load_for_process(rid)

    # ------------------------------------------------------------
    # Saving (rid-only)
    # ------------------------------------------------------------

    def save_from_editor(self, recipe_id: str, *, draft: Recipe) -> None:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("save_from_editor: recipe_id leer")
        if draft is None or not isinstance(draft, Recipe):
            raise TypeError(f"save_from_editor: draft invalid: {type(draft)}")
        # Enforce rid on model
        draft.id = rid
        return self.bundle.save_from_editor(rid, draft=draft)

    # ------------------------------------------------------------
    # Delete (rid-only)
    # ------------------------------------------------------------

    def delete(self, recipe_id: str) -> None:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("delete: recipe_id leer")

        # Support both possible bundle APIs
        if hasattr(self.bundle, "delete_by_key"):
            return self.bundle.delete_by_key(rid)
        if hasattr(self.bundle, "delete"):
            return self.bundle.delete(rid)

        raise AttributeError("RecipeBundle hat weder delete_by_key() noch delete().")
