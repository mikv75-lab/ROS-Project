# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_repo.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Tuple, Optional

import os
import yaml

# --- UPDATED IMPORTS ---
from model.recipe.recipe import Recipe
from model.spray_paths.draft import Draft
from model.spray_paths.trajectory import JTBySegment
from model.recipe.recipe_bundle import RecipeBundle
from model.recipe.recipe_run_result import RunResult


@dataclass
class RecipeRepo:
    """
    Thin repo wrapper around RecipeBundle.

    SSoT (rid-only):
      <recipes_root_dir>/<recipe_id>/
        params.yaml
        draft.yaml
        planned_traj.yaml
        executed_traj.yaml
        planned_tcp.yaml
        executed_tcp.yaml
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
        s = str(key_or_rid or "").strip().strip("/")
        if not s:
            return ""
        return s.split("/")[0].strip()

    def _paths(self, rid: str):
        if not hasattr(self.bundle, "paths"):
            raise AttributeError("RecipeBundle hat keine paths(rid) API.")
        return self.bundle.paths(rid)

    @staticmethod
    def _read_yaml_file(path: str) -> Optional[dict]:
        if not path or not os.path.isfile(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return data if isinstance(data, dict) else None
        except Exception:
            return None

    def _hydrate_optional_attachments(self, rid: str, recipe: Recipe) -> Recipe:
        """
        Best-effort load optional attachments if files exist on disk.
        MUST NOT raise due to missing/invalid optional files.
        """
        try:
            p = self._paths(rid)
        except Exception:
            return recipe

        # ---- draft (Strict Type: Draft) ----
        try:
            draft_path = str(getattr(p, "draft_yaml", "") or "")
            if getattr(recipe, "draft", None) is None:
                d = self._read_yaml_file(draft_path)
                if d is not None:
                    recipe.draft = Draft.from_yaml_dict(d)
        except Exception:
            pass

        # ---- planned_traj (Strict Type: JTBySegment) ----
        try:
            planned_path = str(getattr(p, "planned_traj_yaml", "") or getattr(p, "planned_yaml", "") or "")
            if getattr(recipe, "planned_traj", None) is None:
                d = self._read_yaml_file(planned_path)
                if d is not None:
                    recipe.planned_traj = JTBySegment.from_yaml_dict(d)
        except Exception:
            pass

        # ---- executed_traj (Strict Type: JTBySegment) ----
        try:
            executed_path = str(getattr(p, "executed_traj_yaml", "") or getattr(p, "executed_yaml", "") or "")
            if getattr(recipe, "executed_traj", None) is None:
                d = self._read_yaml_file(executed_path)
                if d is not None:
                    recipe.executed_traj = JTBySegment.from_yaml_dict(d)
        except Exception:
            pass

        # ---- planned_tcp (Strict Type: Draft) ----
        try:
            planned_tcp_path = str(getattr(p, "planned_tcp_yaml", "") or "")
            if getattr(recipe, "planned_tcp", None) is None:
                d = self._read_yaml_file(planned_tcp_path)
                if d is not None:
                    recipe.planned_tcp = Draft.from_yaml_dict(d)
        except Exception:
            pass

        # ---- executed_tcp (Strict Type: Draft) ----
        try:
            executed_tcp_path = str(getattr(p, "executed_tcp_yaml", "") or "")
            if getattr(recipe, "executed_tcp", None) is None:
                d = self._read_yaml_file(executed_tcp_path)
                if d is not None:
                    recipe.executed_tcp = Draft.from_yaml_dict(d)
        except Exception:
            pass

        return recipe

    # ------------------------------------------------------------
    # Compatibility key helpers (rid-only)
    # ------------------------------------------------------------

    def split_key(self, key: str) -> Tuple[str, str]:
        rid = self._norm_rid(key)
        return rid, ""

    def make_key(self, rid: str, name: str = "") -> str:
        _ = name
        return self._norm_rid(rid)

    # ------------------------------------------------------------
    # Listing / Loading (rid-only)
    # ------------------------------------------------------------

    def list_recipes(self) -> List[str]:
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
        r = self.bundle.load_for_editor(rid)
        return self._hydrate_optional_attachments(rid, r)

    def load_for_process(self, recipe_id: str) -> Recipe:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("load_for_process: recipe_id leer")
        r = self.bundle.load_for_process(rid)
        return self._hydrate_optional_attachments(rid, r)

    # ------------------------------------------------------------
    # Saving (rid-only)
    # ------------------------------------------------------------

    def save_from_editor(self, recipe_id: str, *, draft: Recipe, compiled: Optional[dict] = None) -> None:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("save_from_editor: recipe_id leer")
        if draft is None or not isinstance(draft, Recipe):
            raise TypeError(f"save_from_editor: draft invalid: {type(draft)}")

        draft.id = rid

        # compiled = optional UI artifact. Bundle-Versionen unterscheiden sich in der Signatur.
        if compiled is not None and hasattr(self.bundle, "save_from_editor"):
            try:
                return self.bundle.save_from_editor(rid, draft=draft, compiled=compiled)  # type: ignore[arg-type]
            except TypeError:
                # älterer Bundle: compiled nicht unterstützt
                return self.bundle.save_from_editor(rid, draft=draft)

        return self.bundle.save_from_editor(rid, draft=draft)

    # ------------------------------------------------------------
    # Option A: persist run artifacts (traj + tcp)
    # ------------------------------------------------------------

    def save_run_artifacts(
        self,
        recipe_id: str,
        *,
        planned_traj: Optional[JTBySegment] = None,
        executed_traj: Optional[JTBySegment] = None,
        planned_tcp: Optional[Draft] = None,
        executed_tcp: Optional[Draft] = None,
    ) -> None:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("save_run_artifacts: recipe_id leer")

        # Prefer explicit batch method on bundle
        if hasattr(self.bundle, "save_run_artifacts"):
            return self.bundle.save_run_artifacts(
                rid,
                planned_traj=planned_traj,
                executed_traj=executed_traj,
                planned_tcp=planned_tcp,
                executed_tcp=executed_tcp,
            )

        # Fallback to individual saves (if bundle API is older)
        if planned_traj is not None and hasattr(self.bundle, "save_planned_traj"):
            self.bundle.save_planned_traj(rid, planned_traj)
        if executed_traj is not None and hasattr(self.bundle, "save_executed_traj"):
            self.bundle.save_executed_traj(rid, executed_traj)
        if planned_tcp is not None and hasattr(self.bundle, "save_planned_tcp"):
            self.bundle.save_planned_tcp(rid, planned_tcp)
        if executed_tcp is not None and hasattr(self.bundle, "save_executed_tcp"):
            self.bundle.save_executed_tcp(rid, executed_tcp)

    # ------------------------------------------------------------
    # NEW: persist from RunResult (ONLY if valid)
    # ------------------------------------------------------------

    def save_run_result_if_valid(
        self,
        recipe_id: str,
        *,
        run_result: RunResult,
    ) -> bool:
        """
        Saves run artifacts derived from RunResult ONLY if run_result.valid is True.
        Automatically converts dicts to strict Objects (JTBySegment/Draft).
        """
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("save_run_result_if_valid: recipe_id leer")
        if run_result is None or not isinstance(run_result, RunResult):
            raise TypeError(f"save_run_result_if_valid: run_result invalid: {type(run_result)}")

        if not bool(getattr(run_result, "valid", False)):
            return False

        # Extract Raw Dicts
        p_traj_dict = dict((run_result.planned_run or {}).get("traj") or {})
        e_traj_dict = dict((run_result.executed_run or {}).get("traj") or {})
        p_tcp_dict = dict((run_result.planned_run or {}).get("tcp") or {})
        e_tcp_dict = dict((run_result.executed_run or {}).get("tcp") or {})

        # Convert to Strict Objects (Bundle expects objects with .to_yaml_dict())
        planned_traj = JTBySegment.from_yaml_dict(p_traj_dict) if p_traj_dict else None
        executed_traj = JTBySegment.from_yaml_dict(e_traj_dict) if e_traj_dict else None
        planned_tcp = Draft.from_yaml_dict(p_tcp_dict) if p_tcp_dict else None
        executed_tcp = Draft.from_yaml_dict(e_tcp_dict) if e_tcp_dict else None

        self.save_run_artifacts(
            rid,
            planned_traj=planned_traj,
            executed_traj=executed_traj,
            planned_tcp=planned_tcp,
            executed_tcp=executed_tcp,
        )
        return True

    # ------------------------------------------------------------
    # Delete (rid-only)
    # ------------------------------------------------------------

    def delete(self, recipe_id: str) -> None:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("delete: recipe_id leer")

        if hasattr(self.bundle, "delete_by_key"):
            return self.bundle.delete_by_key(rid)
        if hasattr(self.bundle, "delete"):
            return self.bundle.delete(rid)

        raise AttributeError("RecipeBundle hat weder delete_by_key() noch delete().")