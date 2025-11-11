# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_store.py
from __future__ import annotations
from copy import deepcopy
from typing import Any, Dict, Optional, List


class RecipeStore:
    """
    Liest die YAML-Struktur aus ctx.recipes_yaml und bietet Zugriffe für UI/Preview.

    Designziele:
    - Keine hartcodierten Defaults im Code – alles kommt aus der YAML.
    - Sides hängen direkt an jedem Rezept unter recipes[i].sides.
    - Globale Defaults: recipe_params.globals.*.default
    - Planner-Defaults: recipe_params.planner.*.default
    - Path-Schemata: recipe_params.path.*
    """

    # Mapping vom internen "type" → Schema-Schlüssel unter recipe_params.*
    _TYPE_TO_SCHEMA_KEY: Dict[str, str] = {
        "meander_plane":          "path.meander.plane",
        "spiral_plane":           "path.spiral.plane",
        "spiral_cylinder":        "path.spiral.cylinder",
        "perimeter_follow_plane": "path.perimeter_follow.plane",
        "polyhelix_cube":         "path.polyhelix.cube",
        "polyhelix_pyramid":      "path.polyhelix.pyramid",
    }

    # ------------------------------------------------------------------ #
    # Konstruktion
    # ------------------------------------------------------------------ #
    def __init__(self, data: Dict[str, Any]):
        # Gesamter YAML-Inhalt (dict)
        self.data = data or {}
        # recipe_params-Knoten (Schema/Defaults)
        self.params_schema: Dict[str, Any] = (self.data.get("recipe_params") or {})
        # Liste aller Rezeptdefinitionen
        self.recipes: List[Dict[str, Any]] = list((self.data.get("recipes") or []))

    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        """Factory: liest ctx.recipes_yaml (bereits geparstes YAML-Dict)."""
        return RecipeStore(getattr(ctx, "recipes_yaml", {}) or {})

    # ------------------------------------------------------------------ #
    # interne Helfer
    # ------------------------------------------------------------------ #
    @staticmethod
    def _nested_get(data: dict, dotted: str):
        cur = data
        for part in dotted.split("."):
            if not isinstance(cur, dict) or part not in cur:
                return None
            cur = cur[part]
        return cur

    @staticmethod
    def _get_params_node(ps: dict, key: str):
        if not isinstance(ps, dict):
            return None

        # 1) direkter Key
        node = ps.get(key)
        if isinstance(node, dict):
            return dict(node)

        # 2) dotted traversal
        cur = ps
        ok = True
        for part in key.split("."):
            if not isinstance(cur, dict) or part not in cur:
                ok = False
                break
            cur = cur[part]
        if ok and isinstance(cur, dict):
            return dict(cur)

        # 3) heuristisch: '_' <-> '.' und lower()
        k_l = key.replace("_", ".").lower()
        for k, v in ps.items():
            if isinstance(v, dict) and k.replace("_", ".").lower() == k_l:
                return dict(v)

        return None

    def _get_params_node_key(self, key: str) -> Optional[Dict[str, Any]]:
        return self._get_params_node(self.params_schema or {}, key)

    # ------------------------------------------------------------------ #
    # Recipes (IDs, Lookup, Listen)
    # ------------------------------------------------------------------ #
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

    # ------------------------------------------------------------------ #
    # Globals (Schema/Defaults)
    # ------------------------------------------------------------------ #
    def globals_schema(self) -> Dict[str, Any]:
        gs = (self.params_schema or {}).get("globals")
        return dict(gs) if isinstance(gs, dict) else {}

    def collect_global_defaults(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for key, spec in self.globals_schema().items():
            if isinstance(spec, dict) and "default" in spec:
                out[key] = spec["default"]
        return out

    # ------------------------------------------------------------------ #
    # Planner (Schema/Defaults)
    # ------------------------------------------------------------------ #
    def planner_schema(self) -> Dict[str, Any]:
        node = (self.params_schema or {}).get("planner") or {}
        return dict(node) if isinstance(node, dict) else {}

    def collect_planner_defaults(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for key, spec in (self.planner_schema() or {}).items():
            if isinstance(spec, dict) and "default" in spec:
                out[key] = spec["default"]
        return out

    # ------------------------------------------------------------------ #
    # Sides/Paths (aus Rezepten)
    # ------------------------------------------------------------------ #
    def sides_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Any]:
        sides = rec_def.get("sides") or {}
        return dict(sides) if isinstance(sides, dict) else {}

    def allowed_and_default_for(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        sides = self.sides_for_recipe(rec_def)
        return deepcopy(sides.get(side) or {})

    def build_default_paths_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        out: Dict[str, Dict[str, Any]] = {}
        for side, scfg in self.sides_for_recipe(rec_def).items():
            dp = dict((scfg.get("default_path") or {}))
            if not dp:
                dp = {"type": ""}  # UI setzt später explizit
            out[str(side)] = dp
        return out

    # ------------------------------------------------------------------ #
    # Path-Schema Auflösung (robust)
    # ------------------------------------------------------------------ #
    def schema_for_type_strict(self, ptype: str) -> Dict[str, Any]:
        p = str(ptype).strip()
        key = self._TYPE_TO_SCHEMA_KEY.get(p)
        if not key:
            raise KeyError(f"Unknown path type '{ptype}' – no schema key mapping.")

        ps = self.params_schema or {}
        node = self._get_params_node(ps, key)
        if not isinstance(node, dict):
            for k, v in (ps.items() if isinstance(ps, dict) else []):
                if isinstance(v, dict) and str(k).startswith("path.") and p in str(k):
                    return dict(v)
            available = ", ".join(
                [k for k in (ps.keys() if isinstance(ps, dict) else []) if str(k).startswith("path.")]
            )
            raise KeyError(
                f"Schema not found at recipe_params.{key} for type '{ptype}'. "
                f"Available path.* keys: [{available}]"
            )
        return node

    # ------------------------------------------------------------------ #
    # Runtime-Normalisierung einer Side (strict w.r.t. Schema)
    # ------------------------------------------------------------------ #
    def build_side_runtime_cfg_strict(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        scfg = self.allowed_and_default_for(rec_def, side) or {}

        # --- allowed_path_types (liberal gelesen) ---
        allowed_keys = ["allowed_path_types", "allowed_types", "allowed", "types", "path_types"]
        allowed: List[str] = []
        for k in allowed_keys:
            v = scfg.get(k)
            if isinstance(v, (list, tuple)):
                allowed = [str(x) for x in v]
                break

        # --- default_path (liberal: 'default_path' oder 'default') ---
        default_path = dict(scfg.get("default_path") or scfg.get("default") or {})
        if "type" not in default_path or not str(default_path["type"]).strip():
            if allowed:
                default_path["type"] = allowed[0]
            else:
                raise ValueError(
                    f"Recipe side '{side}' besitzt weder default_path.type noch allowed_path_types."
                )

        ptype = str(default_path["type"]).strip()

        # --- Schema des Default-Typs ---
        schema_default = self.schema_for_type_strict(ptype)

        # --- strict: Default-Params = Schema-Defaults, dann Rezept-Overrides ---
        norm: Dict[str, Any] = {"type": ptype}
        for key, spec in (schema_default or {}).items():
            if isinstance(spec, dict) and "default" in spec:
                norm[key] = spec["default"]
        for k, v in default_path.items():
            if k != "type" and k in schema_default:
                norm[k] = v

        # --- schemas: für alle erlaubten Typen (inkl. Default) bereitstellen ---
        types_to_resolve = list(dict.fromkeys([ptype] + list(allowed)))
        schemas: Dict[str, Dict[str, Any]] = {}
        resolved_allowed: List[str] = []

        for t in types_to_resolve:
            try:
                sch = self.schema_for_type_strict(t)
            except KeyError:
                continue
            schemas[t] = sch
            if t in allowed and t not in resolved_allowed:
                resolved_allowed.append(t)

        if not resolved_allowed:
            resolved_allowed = [ptype]

        return {
            "allowed_path_types": resolved_allowed,
            "default_path": norm,
            "schemas": schemas,
        }

    # ------------------------------------------------------------------ #
    # UI-Enum-Helper (optional)
    # ------------------------------------------------------------------ #
    def spiral_plane_enums(self) -> Dict[str, List[str]]:
        out: Dict[str, List[str]] = {}
        node = self._get_params_node_key("path.spiral.plane") or {}
        if isinstance(node, dict):
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["direction"] = [str(v) for v in dir_spec["values"]]
        return out

    def spiral_cylinder_enums(self) -> Dict[str, List[str]]:
        out: Dict[str, List[str]] = {}
        node = self._get_params_node_key("path.spiral.cylinder") or {}
        if isinstance(node, dict):
            sf_spec = node.get("start_from")
            if isinstance(sf_spec, dict) and isinstance(sf_spec.get("values"), list):
                out["start_froms"] = [str(v) for v in sf_spec["values"]]
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["directions"] = [str(v) for v in dir_spec["values"]]
        return out
