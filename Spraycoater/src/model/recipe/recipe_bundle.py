# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_bundle.py
from __future__ import annotations

import io
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, List

import yaml

# STRICT Imports
from .recipe import Recipe

from model.spray_paths.draft import Draft
from model.spray_paths.trajectory import JTBySegment


def _err(msg: str) -> None:
    raise ValueError(msg)


def _norm(p: str) -> str:
    return os.path.abspath(os.path.normpath(os.path.expanduser(p)))


def _ensure_dir(d: str) -> None:
    os.makedirs(_norm(d), exist_ok=True)


def _load_yaml(path: str) -> Optional[Dict[str, Any]]:
    path = _norm(path)
    if not os.path.isfile(path):
        return None
    with io.open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else None


def _save_yaml(path: str, data: Dict[str, Any]) -> None:
    path = _norm(path)
    _ensure_dir(os.path.dirname(path))
    with io.open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


def _delete_file(path: str) -> None:
    p = _norm(path)
    if os.path.exists(p):
        os.remove(p)


@dataclass(frozen=True)
class RecipePaths:
    """Canonical paths for one recipe id."""
    rid: str
    recipe_dir: str
    params_yaml: str
    draft_yaml: str
    planned_traj_yaml: str
    executed_traj_yaml: str
    planned_tcp_yaml: str
    executed_tcp_yaml: str


@dataclass
class RecipeBundle:
    """
    Verwaltet die Dateistruktur auf der Festplatte.
    Strikte Trennung der Dateien.

    Layout:
      <recipes_root>/<rid>/
        params.yaml
        draft.yaml
        planned_traj.yaml
        executed_traj.yaml
        planned_tcp.yaml
        executed_tcp.yaml
    """
    recipes_root: str

    def __post_init__(self) -> None:
        if not isinstance(self.recipes_root, str) or not self.recipes_root.strip():
            _err("RecipeBundle: recipes_root ist leer")
        self.recipes_root = _norm(self.recipes_root)
        _ensure_dir(self.recipes_root)

    def _path(self, rid: str, filename: str) -> str:
        rid = str(rid or "").strip().strip("/")
        if not rid:
            _err("RecipeBundle._path: rid ist leer")
        return os.path.join(self.recipes_root, rid, filename)

    def paths(self, rid: str) -> RecipePaths:
        rid = str(rid or "").strip().strip("/")
        if not rid:
            _err("RecipeBundle.paths: rid ist leer")
        rdir = os.path.join(self.recipes_root, rid)
        return RecipePaths(
            rid=rid,
            recipe_dir=rdir,
            params_yaml=os.path.join(rdir, "params.yaml"),
            draft_yaml=os.path.join(rdir, "draft.yaml"),
            planned_traj_yaml=os.path.join(rdir, "planned_traj.yaml"),
            executed_traj_yaml=os.path.join(rdir, "executed_traj.yaml"),
            planned_tcp_yaml=os.path.join(rdir, "planned_tcp.yaml"),
            executed_tcp_yaml=os.path.join(rdir, "executed_tcp.yaml"),
        )

    # --------------------------------------------------------
    # Listing
    # --------------------------------------------------------
    def list_recipes(self) -> List[str]:
        if not os.path.isdir(self.recipes_root):
            return []
        out: List[str] = []
        for name in sorted(os.listdir(self.recipes_root)):
            rdir = os.path.join(self.recipes_root, name)
            if not os.path.isdir(rdir):
                continue
            # minimal criterion: params.yaml exists
            if os.path.isfile(os.path.join(rdir, "params.yaml")):
                out.append(name)
        return out

    # --------------------------------------------------------
    # Params (params.yaml)
    # --------------------------------------------------------
    def load_params(self, rid: str) -> Optional[Recipe]:
        data = _load_yaml(self._path(rid, "params.yaml"))
        if not data:
            return None
        data["id"] = str(rid)  # ID sicherstellen
        return Recipe.from_params_dict(data)

    def save_params(self, recipe: Recipe) -> None:
        if recipe is None or not isinstance(recipe, Recipe):
            _err(f"RecipeBundle.save_params: recipe invalid: {type(recipe)}")
        _save_yaml(self._path(recipe.id, "params.yaml"), recipe.to_params_dict())

    # --------------------------------------------------------
    # Draft (draft.yaml)
    # --------------------------------------------------------
    def load_draft(self, rid: str) -> Optional[Draft]:
        data = _load_yaml(self._path(rid, "draft.yaml"))
        return Draft.from_yaml_dict(data) if data else None

    def save_draft(self, rid: str, draft: Draft) -> None:
        if draft is None or not isinstance(draft, Draft):
            _err(f"RecipeBundle.save_draft: draft invalid: {type(draft)}")
        _save_yaml(self._path(rid, "draft.yaml"), draft.to_yaml_dict())

    # --------------------------------------------------------
    # Trajectories (planned_traj.yaml, executed_traj.yaml)
    # --------------------------------------------------------
    def load_planned_traj(self, rid: str) -> Optional[JTBySegment]:
        data = _load_yaml(self._path(rid, "planned_traj.yaml"))
        return JTBySegment.from_yaml_dict(data) if data else None

    def load_executed_traj(self, rid: str) -> Optional[JTBySegment]:
        data = _load_yaml(self._path(rid, "executed_traj.yaml"))
        return JTBySegment.from_yaml_dict(data) if data else None

    def save_planned_traj(self, rid: str, traj: JTBySegment) -> None:
        if traj is None or not isinstance(traj, JTBySegment):
            _err(f"RecipeBundle.save_planned_traj: traj invalid: {type(traj)}")
        _save_yaml(self._path(rid, "planned_traj.yaml"), traj.to_yaml_dict())

    def save_executed_traj(self, rid: str, traj: JTBySegment) -> None:
        if traj is None or not isinstance(traj, JTBySegment):
            _err(f"RecipeBundle.save_executed_traj: traj invalid: {type(traj)}")
        _save_yaml(self._path(rid, "executed_traj.yaml"), traj.to_yaml_dict())

    # --------------------------------------------------------
    # TCP Geometry (planned_tcp.yaml, executed_tcp.yaml)
    # --------------------------------------------------------
    def load_planned_tcp(self, rid: str) -> Optional[Draft]:
        data = _load_yaml(self._path(rid, "planned_tcp.yaml"))
        return Draft.from_yaml_dict(data) if data else None

    def load_executed_tcp(self, rid: str) -> Optional[Draft]:
        data = _load_yaml(self._path(rid, "executed_tcp.yaml"))
        return Draft.from_yaml_dict(data) if data else None

    def save_planned_tcp(self, rid: str, tcp: Draft) -> None:
        if tcp is None or not isinstance(tcp, Draft):
            _err(f"RecipeBundle.save_planned_tcp: tcp invalid: {type(tcp)}")
        _save_yaml(self._path(rid, "planned_tcp.yaml"), tcp.to_yaml_dict())

    def save_executed_tcp(self, rid: str, tcp: Draft) -> None:
        if tcp is None or not isinstance(tcp, Draft):
            _err(f"RecipeBundle.save_executed_tcp: tcp invalid: {type(tcp)}")
        _save_yaml(self._path(rid, "executed_tcp.yaml"), tcp.to_yaml_dict())

    # --------------------------------------------------------
    # Full Loading / Saving Facade
    # --------------------------------------------------------
    def load_full_recipe(self, rid: str) -> Optional[Recipe]:
        recipe = self.load_params(rid)
        if recipe is None:
            return None

        recipe.draft = self.load_draft(rid)
        recipe.planned_traj = self.load_planned_traj(rid)
        recipe.executed_traj = self.load_executed_traj(rid)
        recipe.planned_tcp = self.load_planned_tcp(rid)
        recipe.executed_tcp = self.load_executed_tcp(rid)
        return recipe

    # --- Editor/Process facade names (Repo/UI compatibility) ---
    def load_for_editor(self, rid: str) -> Recipe:
        r = self.load_full_recipe(rid)
        if r is None:
            raise KeyError(f"RecipeBundle.load_for_editor: recipe not found: {rid}")
        return r

    def load_for_process(self, rid: str) -> Recipe:
        r = self.load_full_recipe(rid)
        if r is None:
            raise KeyError(f"RecipeBundle.load_for_process: recipe not found: {rid}")
        return r

    def save_from_editor(self, rid: str, *, draft: Recipe, compiled: Optional[dict] = None) -> None:
        # compiled ist ein UI-Artefakt; wird hier bewusst ignoriert (SSoT = params/draft)
        _ = compiled
        if draft is None or not isinstance(draft, Recipe):
            _err(f"RecipeBundle.save_from_editor: draft invalid: {type(draft)}")
        draft.id = str(rid)
        self.save_params(draft)
        if getattr(draft, "draft", None) is not None:
            self.save_draft(rid, draft.draft)

    def save_run_artifacts(
        self,
        rid: str,
        *,
        planned_traj: Optional[JTBySegment] = None,
        executed_traj: Optional[JTBySegment] = None,
        planned_tcp: Optional[Draft] = None,
        executed_tcp: Optional[Draft] = None,
    ) -> None:
        """Saves runtime artifacts. Removes files if data is None."""
        p_pt = self._path(rid, "planned_traj.yaml")
        if planned_traj is not None:
            _save_yaml(p_pt, planned_traj.to_yaml_dict())
        else:
            _delete_file(p_pt)

        p_et = self._path(rid, "executed_traj.yaml")
        if executed_traj is not None:
            _save_yaml(p_et, executed_traj.to_yaml_dict())
        else:
            _delete_file(p_et)

        p_ptcp = self._path(rid, "planned_tcp.yaml")
        if planned_tcp is not None:
            _save_yaml(p_ptcp, planned_tcp.to_yaml_dict())
        else:
            _delete_file(p_ptcp)

        p_etcp = self._path(rid, "executed_tcp.yaml")
        if executed_tcp is not None:
            _save_yaml(p_etcp, executed_tcp.to_yaml_dict())
        else:
            _delete_file(p_etcp)

    def delete_recipe(self, rid: str) -> None:
        import shutil
        rid = str(rid or "").strip().strip("/")
        if not rid:
            _err("RecipeBundle.delete_recipe: rid ist leer")
        path = os.path.join(self.recipes_root, rid)
        if os.path.exists(path):
            shutil.rmtree(path)

    # Repo compatibility aliases (optional)
    def delete_by_key(self, key: str) -> None:
        self.delete_recipe(key)

    def delete(self, rid: str) -> None:
        self.delete_recipe(rid)
