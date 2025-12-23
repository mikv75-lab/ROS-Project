# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_store.py
from __future__ import annotations

import io
import os
from copy import deepcopy
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import yaml


def _err(msg: str) -> None:
    raise ValueError(msg)


def _load_yaml(path: str, *, strict: bool = True) -> Optional[Dict[str, Any]]:
    try:
        path = os.path.abspath(os.path.normpath(path))
        if not os.path.exists(path):
            if strict:
                _err(f"YAML nicht gefunden: {path}")
            return None
        with io.open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data is None:
            if strict:
                _err(f"YAML ist leer: {path}")
            return None
        if not isinstance(data, dict):
            if strict:
                _err(f"YAML ist kein Mapping (Dict): {path}")
            return None
        return data
    except Exception as e:
        if strict:
            _err(str(e))
        return None


@dataclass(frozen=True)
class RecipeStorePaths:
    recipe_params_file: str
    planner_catalog_file: str
    recipe_catalog_file: str


class RecipeStore:
    """
    Lädt und hält die 3 YAMLs:
      - recipe_params.yaml    -> recipe_params
      - planner_catalog.yaml  -> planner_catalog
      - recipe_catalog.yaml   -> recipes list

    Optional: erzeugt legacy 'recipes_yaml' View, wenn Alt-Code sie erwartet.
    """

    def __init__(
        self,
        *,
        paths: RecipeStorePaths,
        recipe_params_yaml: Dict[str, Any],
        planner_catalog_yaml: Dict[str, Any],
        recipe_catalog_yaml: Dict[str, Any],
    ):
        self.paths = paths

        # raw YAML roots
        self.recipe_params_yaml = recipe_params_yaml or {}
        self.planner_catalog_yaml = planner_catalog_yaml or {}
        self.recipe_catalog_yaml = recipe_catalog_yaml or {}

        # parsed payloads
        self.params_schema: Dict[str, Any] = self._require_dict(self.recipe_params_yaml, "recipe_params", "recipe_params.yaml")
        self.planner_catalog: Dict[str, Any] = self._require_dict(self.planner_catalog_yaml, "planner_catalog", "planner_catalog.yaml")
        self.recipes: List[Dict[str, Any]] = self._require_list_of_dicts(self.recipe_catalog_yaml, "recipes", "recipe_catalog.yaml")

        # optional legacy combined view (für Übergang)
        self.recipes_yaml: Dict[str, Any] = {
            "version": 1,
            "recipe_params": self.params_schema,
            "planner_catalog": self.planner_catalog,
            "recipes": self.recipes,
        }

    # ---------- factories ----------

    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        """
        ctx kommt aus startup.load_startup() und liefert nur Pfade:
          ctx.paths.recipe_params_file
          ctx.paths.planner_catalog_file
          ctx.paths.recipe_catalog_file
        """
        p = getattr(ctx, "paths", None)
        if p is None:
            _err("RecipeStore.from_ctx: ctx.paths fehlt.")

        rp_file = getattr(p, "recipe_params_file", None)
        pc_file = getattr(p, "planner_catalog_file", None)
        rc_file = getattr(p, "recipe_catalog_file", None)
        if not rp_file or not pc_file or not rc_file:
            _err("RecipeStore.from_ctx: ctx.paths.*_file fehlen (recipe_params/planner_catalog/recipe_catalog).")

        rp_y = _load_yaml(rp_file, strict=True) or {}
        pc_y = _load_yaml(pc_file, strict=True) or {}
        rc_y = _load_yaml(rc_file, strict=True) or {}

        return RecipeStore(
            paths=RecipeStorePaths(
                recipe_params_file=str(rp_file),
                planner_catalog_file=str(pc_file),
                recipe_catalog_file=str(rc_file),
            ),
            recipe_params_yaml=rp_y,
            planner_catalog_yaml=pc_y,
            recipe_catalog_yaml=rc_y,
        )

    # ---------- internal validators ----------

    @staticmethod
    def _require_dict(root: Dict[str, Any], key: str, label: str) -> Dict[str, Any]:
        node = (root or {}).get(key)
        if not isinstance(node, dict) or not node:
            _err(f"{label}: '{key}' fehlt oder ist leer.")
        return dict(node)

    @staticmethod
    def _require_list_of_dicts(root: Dict[str, Any], key: str, label: str) -> List[Dict[str, Any]]:
        node = (root or {}).get(key)
        if not isinstance(node, list) or not node:
            _err(f"{label}: '{key}' fehlt oder ist leer.")
        out: List[Dict[str, Any]] = []
        for i, e in enumerate(node):
            if not isinstance(e, dict):
                _err(f"{label}: {key}[{i}] ist kein Dict.")
            out.append(dict(e))
        return out

    @staticmethod
    def _as_str_list(val: Any) -> List[str]:
        if isinstance(val, (list, tuple)):
            return [str(x) for x in val]
        if isinstance(val, str):
            return [val]
        return []

    # ---------------- recipes ----------------

    def recipe_ids(self) -> List[str]:
        return [str(r.get("id")) for r in self.recipes if r.get("id")]

    def get_recipe_def(self, rid: str) -> Optional[Dict[str, Any]]:
        for r in self.recipes:
            if str(r.get("id")) == str(rid):
                return r
        return None

    def tools_for_recipe(self, rec_def: Dict[str, Any]) -> List[str]:
        return [str(t) for t in (rec_def.get("tools") or [])]

    def substrates_for_recipe(self, rec_def: Dict[str, Any]) -> List[str]:
        return [str(s) for s in (rec_def.get("substrates") or [])]

    def mounts_for_recipe(self, rec_def: Dict[str, Any]) -> List[str]:
        return [str(m) for m in (rec_def.get("substrate_mounts") or [])]

    # ---------------- recipe params ----------------

    def _get_params_node_key(self, key: str) -> Optional[Dict[str, Any]]:
        ps = self.params_schema or {}
        node = ps.get(key)
        return dict(node) if isinstance(node, dict) else None

    def globals_schema(self) -> Dict[str, Any]:
        gs = self._get_params_node_key("globals")
        return gs if isinstance(gs, dict) else {}

    def collect_global_defaults(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for key, spec in self.globals_schema().items():
            if isinstance(spec, dict) and "default" in spec:
                out[key] = spec["default"]
        return out

    # ---------------- planner catalog (NEU) ----------------

    def planner_defaults(self, *, role: str | None = None, pipeline: str | None = None) -> Dict[str, Any]:
        """
        Erwartetes Schema (empfohlen):
          planner_catalog:
            defaults:
              common: {...}
              roles: { validate_move: {...}, ... }
              pipelines: { ompl: {...}, ... }
        """
        cat = self.planner_catalog or {}
        droot = cat.get("defaults") or {}
        out: Dict[str, Any] = {}

        common = droot.get("common") or {}
        if isinstance(common, dict):
            out.update(common)

        if role:
            rnode = (droot.get("roles") or {}).get(role) or {}
            if isinstance(rnode, dict):
                out.update(rnode)

        if pipeline:
            pnode = (droot.get("pipelines") or {}).get(pipeline) or {}
            if isinstance(pnode, dict):
                out.update(pnode)

        return out

    def allowed_planners(self, *, pipeline: str) -> List[str]:
        """
        Erwartetes Schema:
          planner_catalog:
            pipelines:
              ompl:
                planners:
                  RRTConnectkConfigDefault: {}
        """
        cat = self.planner_catalog or {}
        pnode = (cat.get("pipelines") or {}).get(str(pipeline)) or {}
        planners = pnode.get("planners") or {}
        if isinstance(planners, dict):
            return [str(k) for k in planners.keys()]
        return []

    # ---------------- sides/paths schema (wie gehabt) ----------------

    def sides_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Any]:
        sides = rec_def.get("sides") or {}
        return dict(sides) if isinstance(sides, dict) else {}

    def allowed_and_default_for(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        sides = self.sides_for_recipe(rec_def)
        if side not in sides:
            raise KeyError(f"Recipe side '{side}' ist nicht definiert (rezept='{rec_def.get('id')}').")
        return deepcopy(sides[side] or {})

    def schema_for_type_strict(self, ptype: str) -> Dict[str, Any]:
        key = str(ptype).strip()
        if not key:
            raise KeyError("schema_for_type_strict: leerer Path-Typ.")

        node = self._get_params_node_key(key)
        if not isinstance(node, dict):
            available = ", ".join(
                [
                    k for k in (
                        self.params_schema.keys()
                        if isinstance(self.params_schema, dict)
                        else []
                    )
                    if str(k).startswith("path.")
                ]
            )
            raise KeyError(
                f"Kein Schema gefunden für type '{ptype}' unter recipe_params['{key}']. "
                f"Verfügbare path.* Keys: [{available}]"
            )
        return node
