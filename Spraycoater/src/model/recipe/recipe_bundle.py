# -*- coding: utf-8 -*-
# File: src/model/recipe/recipe_bundle.py
from __future__ import annotations

import io
import os
from dataclasses import dataclass
from typing import Any, Dict, Optional

import yaml

# STRICT Imports
from .recipe import Recipe
from model.spraypaths.draft import Draft
from ..spraypaths.trajectory import JTBySegment

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
    if not isinstance(data, dict):
        return None
    return data

def _save_yaml(path: str, data: Dict[str, Any]) -> None:
    path = _norm(path)
    _ensure_dir(os.path.dirname(path))
    with io.open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)

def _delete_file(path: str) -> None:
    p = _norm(path)
    if os.path.exists(p):
        os.remove(p)

@dataclass
class RecipeBundle:
    """
    Verwaltet die Dateistruktur auf der Festplatte.
    Strikte Trennung der Dateien.
    """
    recipes_root: str

    def __post_init__(self):
        if not self.recipes_root:
            _err("RecipeBundle: recipes_root ist leer")

    def _path(self, rid: str, filename: str) -> str:
        return os.path.join(self.recipes_root, rid, filename)

    # --------------------------------------------------------
    # Params (recipe.yaml / params.yaml)
    # --------------------------------------------------------
    def load_params(self, rid: str) -> Optional[Recipe]:
        # STRICT: Nur params.yaml
        data = _load_yaml(self._path(rid, "params.yaml"))
        if not data:
            return None
        # ID sicherstellen
        data["id"] = rid
        return Recipe.from_params_dict(data)

    def save_params(self, recipe: Recipe) -> None:
        _save_yaml(self._path(recipe.id, "params.yaml"), recipe.to_params_dict())

    # --------------------------------------------------------
    # Draft (draft.yaml)
    # --------------------------------------------------------
    def load_draft(self, rid: str) -> Optional[Draft]:
        data = _load_yaml(self._path(rid, "draft.yaml"))
        if not data: return None
        return Draft.from_yaml_dict(data)

    def save_draft(self, rid: str, draft: Draft) -> None:
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

    # --------------------------------------------------------
    # TCP Geometry (planned_tcp.yaml, executed_tcp.yaml)
    # --------------------------------------------------------
    def load_planned_tcp(self, rid: str) -> Optional[Draft]:
        data = _load_yaml(self._path(rid, "planned_tcp.yaml"))
        return Draft.from_yaml_dict(data) if data else None

    def load_executed_tcp(self, rid: str) -> Optional[Draft]:
        data = _load_yaml(self._path(rid, "executed_tcp.yaml"))
        return Draft.from_yaml_dict(data) if data else None

    # --------------------------------------------------------
    # Full Loading / Saving Facade
    # --------------------------------------------------------
    def load_full_recipe(self, rid: str) -> Optional[Recipe]:
        recipe = self.load_params(rid)
        if not recipe: return None

        recipe.draft = self.load_draft(rid)
        recipe.planned_traj = self.load_planned_traj(rid)
        recipe.executed_traj = self.load_executed_traj(rid)
        recipe.planned_tcp = self.load_planned_tcp(rid)
        recipe.executed_tcp = self.load_executed_tcp(rid)
        
        return recipe

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
        if planned_traj: _save_yaml(p_pt, planned_traj.to_yaml_dict())
        else: _delete_file(p_pt)

        p_et = self._path(rid, "executed_traj.yaml")
        if executed_traj: _save_yaml(p_et, executed_traj.to_yaml_dict())
        else: _delete_file(p_et)

        p_ptcp = self._path(rid, "planned_tcp.yaml")
        if planned_tcp: _save_yaml(p_ptcp, planned_tcp.to_yaml_dict())
        else: _delete_file(p_ptcp)

        p_etcp = self._path(rid, "executed_tcp.yaml")
        if executed_tcp: _save_yaml(p_etcp, executed_tcp.to_yaml_dict())
        else: _delete_file(p_etcp)

    def delete_recipe(self, rid: str) -> None:
        import shutil
        path = os.path.join(self.recipes_root, rid)
        if os.path.exists(path):
            shutil.rmtree(path)