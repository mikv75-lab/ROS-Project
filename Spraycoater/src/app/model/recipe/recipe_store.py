# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_store.py
from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, List, Optional


class RecipeStore:
    """
    Liest die YAML-Struktur (als Dict) und bietet Zugriffe für UI/Preview.

    Erwartete Top-Level Keys (typisch):
      - recipe_params
      - planner_settings
      - planner_pipeline
      - recipes
    """

    # ------------------------------------------------------------------ #
    # Konstruktion
    # ------------------------------------------------------------------ #
    def __init__(self, data: Dict[str, Any]):
        self.data = data or {}
        self.params_schema: Dict[str, Any] = self.data.get("recipe_params") or {}
        self.recipes: List[Dict[str, Any]] = list(self.data.get("recipes") or [])
        self.planner_settings: Dict[str, Any] = self.data.get("planner_settings") or {}
        self.planner_pipeline: Dict[str, Any] = self.data.get("planner_pipeline") or {}

    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        """Factory: liest ctx.recipes_yaml (bereits geparstes YAML-Dict)."""
        return RecipeStore(getattr(ctx, "recipes_yaml", {}) or {})

    # ------------------------------------------------------------------ #
    # interne Helfer
    # ------------------------------------------------------------------ #
    def _get_params_node_key(self, key: str) -> Optional[Dict[str, Any]]:
        """Liest recipe_params[key] und gibt es als Dict zurück (oder None)."""
        ps = self.params_schema or {}
        node = ps.get(key)
        return dict(node) if isinstance(node, dict) else None

    @staticmethod
    def _as_str_list(val: Any) -> List[str]:
        if isinstance(val, (list, tuple)):
            return [str(x) for x in val]
        if isinstance(val, str):
            return [val]
        return []

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
        gs = self._get_params_node_key("globals")
        return gs if isinstance(gs, dict) else {}

    def collect_global_defaults(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for key, spec in self.globals_schema().items():
            if isinstance(spec, dict) and "default" in spec:
                out[key] = spec["default"]
        return out

    # ------------------------ PLC-Globals ------------------------------ #
    def plc_globals_schema(self) -> Dict[str, Any]:
        """
        Filtert globals-Schema auf Parameter mit `plc: true`.
        """
        out: Dict[str, Any] = {}
        for name, spec in self.globals_schema().items():
            if not isinstance(spec, dict):
                continue
            if bool(spec.get("plc", False)):
                out[name] = spec
        return out

    def plc_param_names(self) -> List[str]:
        """Namen der PLC-relevanten Global-Parameter."""
        return list(self.plc_globals_schema().keys())

    def collect_plc_defaults(self) -> Dict[str, Any]:
        """Default-Werte nur für PLC-relevante Parameter (falls vorhanden)."""
        out: Dict[str, Any] = {}
        for name, spec in self.plc_globals_schema().items():
            if isinstance(spec, dict) and "default" in spec:
                out[name] = spec["default"]
        return out

    # ------------------------------------------------------------------ #
    # Planner (historisch)
    # ------------------------------------------------------------------ #
    def planner_schema(self) -> Dict[str, Any]:
        """
        In aktuellen YAMLs nicht mehr genutzt.
        Wird als leeres Dict zurückgegeben.
        """
        return {}

    def collect_planner_defaults(self) -> Dict[str, Any]:
        """Siehe planner_schema()."""
        return {}

    # ------------------------------------------------------------------ #
    # Planner-Definitionen & -Listen
    # ------------------------------------------------------------------ #
    def planner_defs(self) -> Dict[str, Any]:
        """Alle Planner-Definitionen aus planner_pipeline (pipeline → planner → spec)."""
        return dict(self.planner_pipeline) if isinstance(self.planner_pipeline, dict) else {}

    def planner_lists_global(self) -> Dict[str, List[str]]:
        """
        Rollenlisten aus planner_settings (alle Keys außer 'path_types').
        """
        ps = self.planner_settings or {}
        out: Dict[str, List[str]] = {}
        if not isinstance(ps, dict):
            return out

        for role, vals in ps.items():
            if role == "path_types":
                continue
            out[str(role)] = self._as_str_list(vals)
        return out

    def planner_lists_for_side(
        self,
        rec_def: Dict[str, Any],
        side: str,
        path_type: Optional[str] = None,
    ) -> Dict[str, List[str]]:
        """
        Liefert Planner-Listen pro Rolle für eine Side.

        Basis:
          - planner_lists_global()
        Optional:
          - overrides über planner_settings.path_types[<path_type>][role]
        """
        base = self.planner_lists_global()
        out: Dict[str, List[str]] = {role: list(vals) for role, vals in base.items()}

        if path_type is None:
            sides = self.sides_for_recipe(rec_def)
            side_cfg = sides.get(side)
            if not isinstance(side_cfg, dict):
                raise KeyError(f"Recipe side '{side}' ist nicht definiert (rezept='{rec_def.get('id')}').")

            dp = side_cfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe side '{side}' hat keine 'default_path'-Definition (rezept='{rec_def.get('id')}').")

            ptype = dp.get("type")
            if not isinstance(ptype, str) or not ptype.strip():
                raise KeyError(f"Recipe side '{side}' default_path.type ist leer/ungültig (rezept='{rec_def.get('id')}').")

            path_type = ptype.strip()

        path_types_node = (self.planner_settings or {}).get("path_types") or {}
        overrides = path_types_node.get(str(path_type)) or {}
        if isinstance(overrides, dict):
            for role, vals in overrides.items():
                out[str(role)] = self._as_str_list(vals)

        return out

    def selected_planners_for_side(
        self,
        rec_def: Dict[str, Any],
        side: str,
        path_type: Optional[str] = None,
    ) -> Dict[str, Optional[str]]:
        """
        Wählt genau einen Planner pro Rolle.

        Priorität:
          1) side.planner_selected.<role>
          2) recipe.planner_selected.<role>
          3) erster Eintrag aus planner_lists_for_side(...)[role]
          4) None
        """
        lists = self.planner_lists_for_side(rec_def, side, path_type)

        sides = self.sides_for_recipe(rec_def)
        side_cfg = sides.get(side, {}) or {}
        side_sel = side_cfg.get("planner_selected") or {}
        rec_sel = rec_def.get("planner_selected") or {}

        all_roles = set(lists.keys()) | {"validate_move", "validate_path", "refine", "optimize", "service"}

        result: Dict[str, Optional[str]] = {}
        for role in all_roles:
            candidates = lists.get(role, [])
            chosen = side_sel.get(role) or rec_sel.get(role)
            if chosen in candidates:
                result[role] = chosen
            elif candidates:
                result[role] = candidates[0]
            else:
                result[role] = None
        return result

    # ------------------------------------------------------------------ #
    # Sides/Paths (aus Rezepten)
    # ------------------------------------------------------------------ #
    def sides_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Any]:
        sides = rec_def.get("sides") or {}
        return dict(sides) if isinstance(sides, dict) else {}

    def allowed_and_default_for(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        """
        Gibt die Side-Definition als deepcopy zurück.
        Erwartet üblicherweise 'allowed_path_types' und 'default_path'.
        """
        sides = self.sides_for_recipe(rec_def)
        if side not in sides:
            raise KeyError(f"Recipe side '{side}' ist nicht definiert (rezept='{rec_def.get('id')}').")
        return deepcopy(sides[side] or {})

    def build_default_paths_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        """Extrahiert für jede Side die default_path-Definition."""
        out: Dict[str, Dict[str, Any]] = {}
        for side, scfg in self.sides_for_recipe(rec_def).items():
            if not isinstance(scfg, dict):
                raise TypeError(f"Recipe side '{side}' ist kein Dict (rezept='{rec_def.get('id')}').")
            dp = scfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe side '{side}' hat kein 'default_path' (rezept='{rec_def.get('id')}').")
            out[str(side)] = dict(dp)
        return out

    # ------------------------------------------------------------------ #
    # Path-Schema Auflösung
    # ------------------------------------------------------------------ #
    def schema_for_type_strict(self, ptype: str) -> Dict[str, Any]:
        """
        Liefert das Schema (Spec-Dict) für einen Path-Typ aus recipe_params[ptype].
        """
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

    # ------------------------------------------------------------------ #
    # Runtime-Normalisierung einer Side
    # ------------------------------------------------------------------ #
    def build_side_runtime_cfg_strict(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        """
        Baut eine Laufzeit-Konfiguration für eine Side:

          {
            "allowed_path_types": [...],
            "default_path": { "type": "...", ... },
            "schemas": { "<type>": { ... }, ... }
          }

        - default_path wird mit Schema-Defaults befüllt und dann mit den Werten
          aus default_path überschrieben.
        - Unbekannte Keys in default_path (nicht im Schema) lösen einen Fehler aus.
        """
        scfg = self.allowed_and_default_for(rec_def, side) or {}

        allowed_raw = scfg.get("allowed_path_types")
        if not isinstance(allowed_raw, (list, tuple)) or not allowed_raw:
            raise ValueError(f"Recipe side '{side}' benötigt 'allowed_path_types' (rezept='{rec_def.get('id')}').")
        allowed: List[str] = [str(x) for x in allowed_raw]

        default_path = scfg.get("default_path")
        if not isinstance(default_path, dict):
            raise ValueError(f"Recipe side '{side}' benötigt 'default_path' als Dict (rezept='{rec_def.get('id')}').")
        if "type" not in default_path:
            raise ValueError(f"Recipe side '{side}' benötigt 'default_path.type' (rezept='{rec_def.get('id')}').")

        ptype = str(default_path["type"]).strip()
        if not ptype:
            raise ValueError(f"Recipe side '{side}' hat leeren 'default_path.type' (rezept='{rec_def.get('id')}').")

        schema_default = self.schema_for_type_strict(ptype)

        # Defaults aus Schema + Overrides aus default_path
        norm: Dict[str, Any] = {"type": ptype}
        for key, spec in (schema_default or {}).items():
            if isinstance(spec, dict) and "default" in spec:
                norm[key] = spec["default"]

        for k, v in default_path.items():
            if k == "type":
                continue
            if k not in schema_default:
                raise KeyError(
                    f"Unbekannter Parameter '{k}' in default_path für type '{ptype}' "
                    f"(side='{side}', rezept='{rec_def.get('id')}')."
                )
            norm[k] = v

        # Schemas für erlaubte Typen + Default-Typ bereitstellen
        types_to_resolve = list(dict.fromkeys([ptype] + list(allowed)))
        schemas: Dict[str, Dict[str, Any]] = {}
        resolved_allowed: List[str] = []

        for t in types_to_resolve:
            sch = self.schema_for_type_strict(t)
            schemas[t] = sch
            if t in allowed and t not in resolved_allowed:
                resolved_allowed.append(t)

        if not resolved_allowed:
            raise ValueError(
                f"Keine auflösbaren allowed_path_types für side='{side}' (rezept='{rec_def.get('id')}')."
            )

        return {
            "allowed_path_types": resolved_allowed,
            "default_path": norm,
            "schemas": schemas,
        }

    # ------------------------------------------------------------------ #
    # UI-Enum-Helper
    # ------------------------------------------------------------------ #
    def spiral_plane_enums(self) -> Dict[str, List[str]]:
        """Liest Enum-Werte für path.spiral.plane aus dem Schema (z.B. direction)."""
        out: Dict[str, List[str]] = {}
        node = self._get_params_node_key("path.spiral.plane") or {}
        if isinstance(node, dict):
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["direction"] = [str(v) for v in dir_spec["values"]]
        return out

    def spiral_cylinder_enums(self) -> Dict[str, List[str]]:
        """Liest Enum-Werte für path.spiral.cylinder aus dem Schema (start_from, direction)."""
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
