# Spraycoater/src/app/tabs/recipe/recipe_model.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from dataclasses import dataclass, field, replace
from typing import Any, Dict, List, Optional
import os, yaml, logging
from .recipe_validator import RecipeValidator

_LOG = logging.getLogger("app.tabs.recipe.model")

@dataclass(frozen=True)
class Recipe:
    id: str
    description: str = ""
    tool: Optional[str] = None
    substrate: Optional[str] = None          # <-- genau ein Substrat
    mount: Optional[str] = None
    side: Optional[str] = None
    parameters: Dict[str, Any] = field(default_factory=dict)
    path: Dict[str, Any] = field(default_factory=dict)

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "Recipe":
        if not isinstance(d, dict):
            raise ValueError("Recipe.from_dict: input ist kein Mapping.")
        # kompatibel: substrates (Liste) -> erstes nehmen
        sub = d.get("substrate")
        if sub is None:
            lst = d.get("substrates") or []
            if isinstance(lst, list) and lst:
                sub = lst[0]
        return Recipe(
            id=str(d.get("id", "")),
            description=str(d.get("description", "")),
            tool=d.get("tool"),
            substrate=sub,
            mount=d.get("mount"),
            side=d.get("side"),
            parameters=dict(d.get("parameters", {}) or {}),
            path=dict(d.get("path", {}) or {}),
        )

    def to_dict(self) -> Dict[str, Any]:
        # beides ausgeben, um alte Konsumenten nicht zu brechen
        out = {
            "id": self.id,
            "description": self.description,
            "tool": self.tool,
            "substrate": self.substrate,
            "substrates": [self.substrate] if self.substrate else [],
            "mount": self.mount,
            "side": self.side,
            "parameters": dict(self.parameters),
            "path": dict(self.path),
        }
        return out

    # ---------- IO ----------
    def save_yaml(self, dir_path: str, filename: Optional[str] = None) -> str:
        if not dir_path:
            raise ValueError("save_yaml: dir_path fehlt.")
        target_dir = os.path.abspath(os.path.normpath(dir_path))
        if not os.path.isdir(target_dir):
            raise FileNotFoundError(f"save_yaml: Verzeichnis existiert nicht: {target_dir}")
        fname = filename or f"{self.id or 'recipe'}.yaml"
        target = os.path.join(target_dir, fname)
        with open(target, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, allow_unicode=True, sort_keys=False)
        _LOG.info("Recipe gespeichert: %s", target)
        return target

    @staticmethod
    def load_yaml(file_path: str) -> "Recipe":
        fp = os.path.abspath(os.path.normpath(file_path))
        if not os.path.exists(fp):
            raise FileNotFoundError(fp)
        with open(fp, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return Recipe.from_dict(d)

    def delete_yaml(self, dir_path: str, filename: Optional[str] = None) -> bool:
        fname = filename or f"{self.id or 'recipe'}.yaml"
        fp = os.path.join(os.path.abspath(dir_path), fname)
        try:
            os.remove(fp)
            _LOG.info("Recipe gelöscht: %s", fp)
            return True
        except FileNotFoundError:
            return False

    # ---------- Validation / Optimize ----------
    def validate_syntax(self, recipe_params: Dict[str, Any]) -> List[str]:
        return RecipeValidator(recipe_params).validate_recipe(self.to_dict())

    def validate_bridge(self, bridge, *, timeout: float = 0.0, syntactic_only: bool = False):
        if bridge is None:
            return {"ok": True, "message": "Kein Bridge-Check (bridge=None)."}
        try:
            res = bridge.validate(self.to_dict(), syntactic_only=syntactic_only, timeout=timeout)
            return {"ok": bool(res.ok), "message": res.message, "data": res.data}
        except Exception as e:
            return {"ok": False, "message": str(e)}

    def optimize(self, bridge, *, timeout: float = 0.0):
        if bridge is None:
            return {"ok": False, "message": "optimize: bridge fehlt."}
        try:
            res = bridge.optimize(self.to_dict(), timeout=timeout)
            return {"ok": bool(res.ok), "message": res.message, "data": res.data}
        except Exception as e:
            return {"ok": False, "message": str(e)}
