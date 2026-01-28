# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_repo.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, List, Tuple, Optional

import os
import yaml

from model.recipe.recipe import Recipe
from model.recipe.recipe_bundle import RecipeBundle
from model.recipe.recipe_run_result import RunResult
from model.spray_paths.draft import Draft
from model.spray_paths.trajectory import JTBySegment


@dataclass
class RecipeRepo:
    """
    STRICT repo wrapper around RecipeBundle.

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

    def __init__(self, *, bundle: RecipeBundle) -> None:
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
    def _read_yaml_dict_or_none(path: str) -> Optional[dict]:
        if not path:
            return None
        p = os.path.abspath(os.path.expanduser(path))
        if not os.path.isfile(p):
            return None
        with open(p, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data is None:
            return None
        if not isinstance(data, dict):
            raise ValueError(f"YAML file must contain dict, got {type(data).__name__}: {p}")
        return data

    def _hydrate_optional_attachments(self, rid: str, recipe: Recipe) -> Recipe:
        """
        STRICT:
          - Optional files are loaded if present.
          - If a file exists but is invalid/unparseable, we raise (no silent fallback).
          - If a file is missing, we simply leave the attachment unset.
        """
        p = self._paths(rid)

        # draft
        if getattr(recipe, "draft", None) is None:
            d = self._read_yaml_dict_or_none(str(getattr(p, "draft_yaml", "") or ""))
            if d is not None:
                recipe.draft = Draft.from_yaml_dict(d)

        # planned_traj
        if getattr(recipe, "planned_traj", None) is None:
            planned_path = str(getattr(p, "planned_traj_yaml", "") or getattr(p, "planned_yaml", "") or "")
            d = self._read_yaml_dict_or_none(planned_path)
            if d is not None:
                recipe.planned_traj = JTBySegment.from_yaml_dict(d)

        # executed_traj
        if getattr(recipe, "executed_traj", None) is None:
            executed_path = str(getattr(p, "executed_traj_yaml", "") or getattr(p, "executed_yaml", "") or "")
            d = self._read_yaml_dict_or_none(executed_path)
            if d is not None:
                recipe.executed_traj = JTBySegment.from_yaml_dict(d)

        # planned_tcp
        if getattr(recipe, "planned_tcp", None) is None:
            d = self._read_yaml_dict_or_none(str(getattr(p, "planned_tcp_yaml", "") or ""))
            if d is not None:
                recipe.planned_tcp = Draft.from_yaml_dict(d)

        # executed_tcp
        if getattr(recipe, "executed_tcp", None) is None:
            d = self._read_yaml_dict_or_none(str(getattr(p, "executed_tcp_yaml", "") or ""))
            if d is not None:
                recipe.executed_tcp = Draft.from_yaml_dict(d)

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
        if not hasattr(self.bundle, "list_recipes"):
            raise AttributeError("RecipeBundle hat keine list_recipes() API.")
        keys = self.bundle.list_recipes()
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
        if not hasattr(self.bundle, "load_for_editor"):
            raise AttributeError("RecipeBundle hat keine load_for_editor(rid) API.")
        r = self.bundle.load_for_editor(rid)
        if r is None or not isinstance(r, Recipe):
            raise TypeError(f"bundle.load_for_editor() returned invalid: {type(r)}")
        return self._hydrate_optional_attachments(rid, r)

    def load_for_process(self, recipe_id: str) -> Recipe:
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("load_for_process: recipe_id leer")
        if not hasattr(self.bundle, "load_for_process"):
            raise AttributeError("RecipeBundle hat keine load_for_process(rid) API.")
        r = self.bundle.load_for_process(rid)
        if r is None or not isinstance(r, Recipe):
            raise TypeError(f"bundle.load_for_process() returned invalid: {type(r)}")
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

        if not hasattr(self.bundle, "save_from_editor"):
            raise AttributeError("RecipeBundle hat keine save_from_editor(rid, ...) API.")

        # STRICT: call bundle API directly; if signature mismatches, let it raise.
        if compiled is None:
            return self.bundle.save_from_editor(rid, draft=draft)
        return self.bundle.save_from_editor(rid, draft=draft, compiled=compiled)  # type: ignore[arg-type]

    # ------------------------------------------------------------
    # Persist run artifacts (traj + tcp)
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

        if not hasattr(self.bundle, "save_run_artifacts"):
            raise AttributeError("RecipeBundle hat keine save_run_artifacts(rid, ...) API.")

        return self.bundle.save_run_artifacts(
            rid,
            planned_traj=planned_traj,
            executed_traj=executed_traj,
            planned_tcp=planned_tcp,
            executed_tcp=executed_tcp,
        )

    def save_run_result_if_valid(self, recipe_id: str, *, run_result: RunResult) -> bool:
        """
        STRICT + MERGE (Keep-on-missing):

        - saves ONLY if run_result.valid is True
        - converts dicts to strict objects (JTBySegment/Draft)
        - IMPORTANT POLICY:
            * if a new artifact is present -> overwrite
            * if a new artifact is missing -> keep existing on disk (do NOT delete)
            This matches:
            - executed_* overwrites executed_* when execute produced it
            - planned_* overwrites planned_* when validate/optimize produced it
            - missing parts never delete previous artifacts
        """
        rid = self._norm_rid(recipe_id)
        if not rid:
            raise KeyError("save_run_result_if_valid: recipe_id leer")
        if run_result is None or not isinstance(run_result, RunResult):
            raise TypeError(f"save_run_result_if_valid: run_result invalid: {type(run_result)}")

        if not bool(getattr(run_result, "valid", False)):
            return False

        # --- build "new" artifacts from RunResult (strict parsing) ---
        p_traj_dict = dict((run_result.planned_run or {}).get("traj") or {})
        e_traj_dict = dict((run_result.executed_run or {}).get("traj") or {})
        p_tcp_dict = dict((run_result.planned_run or {}).get("tcp") or {})
        e_tcp_dict = dict((run_result.executed_run or {}).get("tcp") or {})

        new_planned_traj = JTBySegment.from_yaml_dict(p_traj_dict) if p_traj_dict else None
        new_executed_traj = JTBySegment.from_yaml_dict(e_traj_dict) if e_traj_dict else None
        new_planned_tcp = Draft.from_yaml_dict(p_tcp_dict) if p_tcp_dict else None
        new_executed_tcp = Draft.from_yaml_dict(e_tcp_dict) if e_tcp_dict else None

        # --- MERGE/KEEP: if new is None -> keep existing on disk ---
        p = self._paths(rid)

        def _keep_traj(path: str) -> Optional[JTBySegment]:
            d = self._read_yaml_dict_or_none(path)
            return JTBySegment.from_yaml_dict(d) if d else None

        def _keep_tcp(path: str) -> Optional[Draft]:
            d = self._read_yaml_dict_or_none(path)
            return Draft.from_yaml_dict(d) if d else None

        planned_traj = new_planned_traj if new_planned_traj is not None else _keep_traj(str(getattr(p, "planned_traj_yaml", "") or ""))
        executed_traj = new_executed_traj if new_executed_traj is not None else _keep_traj(str(getattr(p, "executed_traj_yaml", "") or ""))
        planned_tcp = new_planned_tcp if new_planned_tcp is not None else _keep_tcp(str(getattr(p, "planned_tcp_yaml", "") or ""))
        executed_tcp = new_executed_tcp if new_executed_tcp is not None else _keep_tcp(str(getattr(p, "executed_tcp_yaml", "") or ""))

        # Now save_run_artifacts will NOT delete anything unintentionally,
        # because missing parts were filled with existing disk values.
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
