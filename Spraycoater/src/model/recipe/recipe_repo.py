# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_repo.py
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from typing import Optional, Dict, Any

from model.recipe.recipe import Recipe
from model.recipe.recipe_bundle import RecipeBundle


@dataclass
class RecipeRepo:
    """
    Wrapper für Editor + ProcessTab.

    key-Format:
      "<recipe_id>/<recipe_name>"
    """
    bundle: RecipeBundle

    # ---------- list/create/delete ----------

    def list_recipes(self) -> list[str]:
        return self.bundle.list_names()

    def make_key(self, recipe_id: str, recipe_name: str) -> str:
        return self.bundle.make_key(recipe_id, recipe_name)

    def create_new(self, key: str, *, base: Optional[Recipe] = None, overwrite: bool = False) -> Recipe:
        return self.bundle.create_new(key, base=base, overwrite=overwrite)

    def delete(self, key: str) -> None:
        self.bundle.delete(key)

    # ---------- load modes ----------

    def load_for_editor(self, key: str) -> Recipe:
        return self.bundle.load_draft(key)

    def load_for_process(self, key: str) -> Recipe:
        return deepcopy(self.bundle.load_full(key))

    # ---------- save policies ----------

    def save_from_editor(
        self,
        key: str,
        *,
        draft: Recipe,
        compiled: Optional[dict] = None,
        delete_compiled_on_hash_change: bool = True,
    ) -> None:
        self.bundle.save_editor(
            key,
            draft=draft,
            compiled=compiled,
            delete_compiled_on_hash_change=delete_compiled_on_hash_change,
        )

    def save_planned_run(
        self,
        key: str,
        *,
        run: Optional[Dict[str, Any]] = None,
        planned: Optional[Dict[str, Any]] = None,
        traj: Optional[Dict[str, Any]] = None,  # legacy alias
    ) -> None:
        """
        Speichert run in planned_traj.yaml (Schema: SegmentRunPayload JOINT-only).

        Robustness:
          - offizielles Keyword: run=... oder planned=...
          - legacy alias: traj=... (wird auf planned gemappt)
        """
        payload = (
            run
            if isinstance(run, dict)
            else (planned if isinstance(planned, dict) else (traj if isinstance(traj, dict) else None))
        )
        if payload is None:
            raise TypeError("save_planned_run: missing payload (use run=... or planned=... or traj=...)")
        self.bundle.save_run(key, kind="planned", run=payload)

    def save_executed_run(
        self,
        key: str,
        *,
        run: Optional[Dict[str, Any]] = None,
        executed: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Speichert run in executed_traj.yaml (Schema: SegmentRunPayload JOINT-only).

        Robustness:
          - offizielles Keyword: run=...
          - legacy/alias: executed=... (wird auf run gemappt)
        """
        payload = run if isinstance(run, dict) else (executed if isinstance(executed, dict) else None)
        if payload is None:
            raise TypeError("save_executed_run: missing payload (use run=... or executed=...)")
        self.bundle.save_run(key, kind="executed", run=payload)

    # Backwards-compatible names (older call-sites)
    def save_traj_run(self, key: str, *, run: Optional[Dict[str, Any]] = None, traj: Optional[Dict[str, Any]] = None) -> None:
        """
        LEGACY: Speichert run als "planned" (planned_traj.yaml).
        """
        self.save_planned_run(key, run=run, traj=traj)

    def clear_runs(self, key: str) -> None:
        self.bundle.clear_runs(key)
