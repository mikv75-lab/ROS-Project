# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, Optional, List


class RecipeStore:
    """Liest die YAML-Struktur aus ctx.recipes_yaml und bietet Zugriff."""
    def __init__(self, data: Dict[str, Any]):
        self.data = data or {}
        self.defaults = (self.data.get("spraycoater") or {}).get("defaults") or {}
        self.params_schema = (self.data.get("recipe_params") or {})
        self.substrate_types = (self.data.get("substrate_types") or {})
        self.recipes: List[Dict[str, Any]] = list((self.data.get("recipes") or []))

    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        return RecipeStore(getattr(ctx, "recipes_yaml", {}) or {})

    def get_recipe_def(self, rid: str) -> Optional[Dict[str, Any]]:
        for r in self.recipes:
            if str(r.get("id")) == str(rid):
                return r
        return None

    def collect_global_defaults(self) -> Dict[str, Any]:
        params = dict(self.defaults)
        globals_schema = (self.params_schema or {}).get("globals", {}) or {}
        for k, spec in globals_schema.items():
            if k not in params and isinstance(spec, dict) and "default" in spec:
                params[k] = spec["default"]
        return params

    def sides_for_substrate_type(self, stype: str) -> Dict[str, Any]:
        """substrate_types.<stype>.sides"""
        return ((self.substrate_types.get(stype) or {}).get("sides") or {})

    def allowed_and_default_for(self, stype: str, side: str) -> Dict[str, Any]:
        sides = self.sides_for_substrate_type(stype)
        return dict(sides.get(side) or {})
