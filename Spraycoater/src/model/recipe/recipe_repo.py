# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_repo.py
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Optional

from model.recipe.recipe import Recipe
from model.recipe.recipe_bundle import RecipeBundle


@dataclass
class RecipeRepo:
    """
    Gemeinsamer Wrapper für Editor + ProcessTab.

    Kernidee:
      - beide Tabs nutzen denselben Repo
      - aber laden in 2 Modi
      - Process bekommt immer eine eigene Instanz (deepcopy), damit Editor-Save nix "wegbläst"
    """
    bundle: RecipeBundle

    # ---------- list/create/delete ----------

    def list_recipes(self) -> list[str]:
        return self.bundle.list_names()

    def create_new(self, name: str, *, base: Optional[Recipe] = None, overwrite: bool = False) -> Recipe:
        return self.bundle.create_new(name, base=base, overwrite=overwrite)

    def delete(self, name: str) -> None:
        self.bundle.delete(name)

    # ---------- load modes ----------

    def load_for_editor(self, name: str) -> Recipe:
        # editor arbeitet nur auf dem draft
        return self.bundle.load_draft(name)

    def load_for_process(self, name: str) -> Recipe:
        # process braucht compiled + last runs
        # deepcopy schützt gegen shared-state zwischen tabs
        return deepcopy(self.bundle.load_full(name))

    # ---------- save policies ----------

    def save_from_editor(
        self,
        name: str,
        *,
        draft: Recipe,
        compiled: Optional[dict] = None,
        delete_compiled_on_hash_change: bool = True,
    ) -> None:
        self.bundle.save_editor(
            name,
            draft=draft,
            compiled=compiled,
            delete_compiled_on_hash_change=delete_compiled_on_hash_change,
        )

    def save_traj_run(self, name: str, *, traj: dict) -> None:
        self.bundle.save_traj(name, traj=traj)

    def save_executed_run(self, name: str, *, executed: dict) -> None:
        self.bundle.save_executed(name, executed=executed)

    def clear_runs(self, name: str) -> None:
        self.bundle.clear_runs(name)
