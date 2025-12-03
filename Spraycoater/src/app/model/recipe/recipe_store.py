# -*- coding: utf-8 -*-
# File: app/model/recipe/recipe_store.py
from __future__ import annotations
from copy import deepcopy
from typing import Any, Dict, Optional, List


class RecipeStore:
    """
    Liest die YAML-Struktur aus ctx.recipes_yaml und bietet Zugriffe für UI/Preview.

    Erwartete YAML-Struktur (vereinfacht):

        recipe_params:
          globals: ...
          path.meander.plane: ...
          path.spiral.plane: ...
          path.spiral.cylinder: ...
          ...

        planner_settings:
          validate_move: [ "RRTConnect", ... ]
          validate_path: [ ... ]
          refine:        [ ... ]
          optimize:      [ ... ]
          service:       [ ... ]
          path_types:
            path.meander.plane:
              validate_move: [ ... ]
              validate_path: [ ... ]
              ...
            path.spiral.cylinder:
              ...

        planner_pipeline:
          ompl:
            RRTConnect:
              description: ...
              params: ...
            ...
          pilz:
            PTP:
              ...
          chomp:
            ...
          stomp:
            ...
          post:
            TimeOptimalTrajectoryGeneration:
              ...

        recipes:
          - id: "wafer"
            ...
            sides:
              top:
                allowed_path_types:
                  - "path.meander.plane"
                  - "path.spiral.plane"
                default_path:
                  type: "path.spiral.plane"
                  pitch_mm: 4.0
                  ...

    Design:
    - Keine Legacy-Felder, keine alternativen Keys.
    - Sides hängen direkt an jedem Rezept unter recipes[i].sides.
    - Globale Defaults: recipe_params.globals.*.default
    - Path-Schemata:    recipe_params["path.*"]
    - Planner-Rollen + Path-Typ-Mapping: planner_settings.*
    - Planner-Definitionen (Beschreibung/Parameter): planner_pipeline.*
    """

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
        # Planner-Settings (Rollen + path_types)
        self.planner_settings: Dict[str, Any] = (self.data.get("planner_settings") or {})
        # Planner-Pipelines (ompl/pilz/chomp/stomp/post → planner → spec)
        self.planner_pipeline: Dict[str, Any] = (self.data.get("planner_pipeline") or {})

    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        """Factory: liest ctx.recipes_yaml (bereits geparstes YAML-Dict)."""
        return RecipeStore(getattr(ctx, "recipes_yaml", {}) or {})

    # ------------------------------------------------------------------ #
    # interne Helfer
    # ------------------------------------------------------------------ #
    def _get_params_node_key(self, key: str) -> Optional[Dict[str, Any]]:
        """
        Strikt: erwartet, dass recipe_params[key] ein dict ist.
        z.B. key = "globals" oder "path.spiral.plane".
        """
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

    # ------------------------------------------------------------------ #
    # Planner (instanzbezogene Werte aus recipe_params.planner) – nicht mehr genutzt
    # ------------------------------------------------------------------ #
    def planner_schema(self) -> Dict[str, Any]:
        """
        Historisch war hier recipe_params.planner vorgesehen.
        In der aktuellen YAML-Version gibt es das nicht mehr.

        Wir geben daher einfach ein leeres Dict zurück.
        """
        return {}

    def collect_planner_defaults(self) -> Dict[str, Any]:
        """
        Da es kein recipe_params.planner mehr gibt, sind die Planner-Instanz-
        defaults leer. Alles, was planner-spezifisch ist, steckt in
        planner_pipeline.*.params und kann UI-seitig direkt genutzt werden.
        """
        return {}

    # ------------------------------------------------------------------ #
    # Planner-Definitionen & -Listen
    # ------------------------------------------------------------------ #
    def planner_defs(self) -> Dict[str, Any]:
        """
        Alle Planner-Definitionen, gruppiert nach Pipeline, wie in planner_pipeline
        definiert, z.B.:

            {
              "ompl": {
                "RRTConnect": {...},
                ...
              },
              "pilz": { ... },
              "chomp": { ... },
              "stomp": { ... },
              "post": { ... },
            }
        """
        return dict(self.planner_pipeline) if isinstance(self.planner_pipeline, dict) else {}

    def planner_lists_global(self) -> Dict[str, List[str]]:
        """
        Globale Rollen-Listen, direkt aus planner_settings:

            planner_settings:
              validate_move: [ "RRTConnect", ... ]
              validate_path: [ "RRTstarkConfigDefault", ... ]
              refine:        [ "CHOMP", ... ]
              optimize:      [ "TimeOptimalTrajectoryGeneration", ... ]
              service:       [ "PTP", "LIN", ... ]
              path_types:
                ...

        Hier werden alle Keys außer 'path_types' als Rollen interpretiert.
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
        Liefert die erlaubten Planner pro Rolle für eine bestimmte Side.

        Basis:
          - planner_lists_global() = globale Rollen-Listen
          - overrides über planner_settings.path_types[<path_type>][role]
        """
        # 1) Basis-Listen kopieren
        base = self.planner_lists_global()
        out: Dict[str, List[str]] = {role: list(vals) for role, vals in base.items()}

        # 2) Path-Typ bestimmen (falls nicht übergeben)
        if path_type is None:
            sides = self.sides_for_recipe(rec_def)
            side_cfg = sides.get(side)
            if not isinstance(side_cfg, dict):
                raise KeyError(f"Recipe side '{side}' not defined für Rezept '{rec_def.get('id')}'.")
            dp = side_cfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe side '{side}' hat keine 'default_path'-Definition.")
            ptype = dp.get("type")
            if not isinstance(ptype, str) or not ptype.strip():
                raise KeyError(f"Recipe side '{side}' default_path.type ist leer/ungültig.")
            path_type = ptype.strip()

        # 3) Path-spezifische Overrides anwenden
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
        Liefert genau EINEN ausgewählten Planner pro Rolle:

            {
              "validate_move": "...",
              "validate_path": "...",
              "refine":        "...",
              "optimize":      "...",
              "service":       "..."
            }

        Priorität:
          1) side.planner_selected.<role>
          2) recipe.planner_selected.<role>
          3) erster Eintrag aus planner_lists_for_side(...)[role]
          4) None (falls nichts erlaubt)

        YAML definiert aktuell keine 'planner_selected', das ist optional
        und kann später ergänzt werden.
        """
        # 1) Kandidaten je Rolle
        lists = self.planner_lists_for_side(rec_def, side, path_type)

        # 2) Auswahl: Side > Rezept
        sides = self.sides_for_recipe(rec_def)
        side_cfg = sides.get(side, {}) or {}
        side_sel = side_cfg.get("planner_selected") or {}
        rec_sel = rec_def.get("planner_selected") or {}

        # Alle möglichen Rollen die wir kennen (inkl. 'refine' und 'service')
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
        Strikt: gibt die Side-Definition 1:1 zurück (deepcopy).
        Erwartet, dass dort 'allowed_path_types' und 'default_path' existieren.
        """
        sides = self.sides_for_recipe(rec_def)
        if side not in sides:
            raise KeyError(f"Recipe side '{side}' nicht definiert für Rezept '{rec_def.get('id')}'.")
        return deepcopy(sides[side] or {})

    def build_default_paths_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        """
        Baut für jede Side die Default-Path-Definition, so wie sie im YAML
        in sides[side].default_path angegeben ist.
        """
        out: Dict[str, Dict[str, Any]] = {}
        for side, scfg in self.sides_for_recipe(rec_def).items():
            if not isinstance(scfg, dict):
                raise TypeError(f"Recipe side '{side}' in '{rec_def.get('id')}' ist kein Dict.")
            dp = scfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe side '{side}' in '{rec_def.get('id')}' hat kein 'default_path'.")
            # Wir kopieren strikt das, was im YAML steht.
            out[str(side)] = dict(dp)
        return out

    # ------------------------------------------------------------------ #
    # Path-Schema Auflösung (strict)
    # ------------------------------------------------------------------ #
    def schema_for_type_strict(self, ptype: str) -> Dict[str, Any]:
        """
        Liefert das Schema (Spec-Dict) für einen Path-Typ.

        Erwartet, dass ptype direkt ein Key in recipe_params ist,
        z.B. "path.spiral.plane" oder "path.polyhelix.cube".

        Keine Heuristiken, kein Mapping.
        """
        key = str(ptype).strip()
        if not key:
            raise KeyError("schema_for_type_strict: leerer Path-Typ.")

        node = self._get_params_node_key(key)
        if not isinstance(node, dict):
            available = ", ".join(
                [k for k in (self.params_schema.keys() if isinstance(self.params_schema, dict) else [])
                 if str(k).startswith("path.")]
            )
            raise KeyError(
                f"Schema not found at recipe_params['{key}'] für type '{ptype}'. "
                f"Verfügbare path.* Keys: [{available}]"
            )
        return node

    # ------------------------------------------------------------------ #
    # Runtime-Normalisierung einer Side (strict w.r.t. Schema)
    # ------------------------------------------------------------------ #
    def build_side_runtime_cfg_strict(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        """
        Baut eine Laufzeit-Konfiguration für eine Side, streng nach Schema:

          {
            "allowed_path_types": [ "path.meander.plane", ... ],
            "default_path": { "type": "path.spiral.plane", ... },
            "schemas": {
              "path.meander.plane": { ... },
              "path.spiral.plane":  { ... },
              ...
            }
          }

        Regeln:
        - 'allowed_path_types' MUSS existieren und eine Liste von Strings sein.
        - 'default_path' MUSS existieren, 'default_path.type' MUSS ein
          gültiger Path-Typ-String sein.
        - Für alle erlaubten Typen + den Default-Typ wird das Schema unter
          recipe_params[ptype] geladen.
        - default_path wird mit Schema-Defaults befüllt (Schema.default, dann
          Overrides aus default_path).
        """
        scfg = self.allowed_and_default_for(rec_def, side) or {}

        # --- allowed_path_types (strikt) ---
        allowed_raw = scfg.get("allowed_path_types")
        if not isinstance(allowed_raw, (list, tuple)) or not allowed_raw:
            raise ValueError(
                f"Recipe side '{side}' von '{rec_def.get('id')}' muss 'allowed_path_types' haben."
            )
        allowed: List[str] = [str(x) for x in allowed_raw]

        # --- default_path (strikt) ---
        default_path = scfg.get("default_path")
        if not isinstance(default_path, dict):
            raise ValueError(
                f"Recipe side '{side}' von '{rec_def.get('id')}' muss 'default_path' als Dict haben."
            )
        if "type" not in default_path:
            raise ValueError(
                f"Recipe side '{side}' von '{rec_def.get('id')}' muss 'default_path.type' besitzen."
            )
        ptype = str(default_path["type"]).strip()
        if not ptype:
            raise ValueError(
                f"Recipe side '{side}' von '{rec_def.get('id')}' hat leeren 'default_path.type'."
            )

        # --- Schema des Default-Typs ---
        schema_default = self.schema_for_type_strict(ptype)

        # --- strict: Default-Params = Schema-Defaults, dann Rezept-Overrides ---
        norm: Dict[str, Any] = {"type": ptype}
        for key, spec in (schema_default or {}).items():
            if isinstance(spec, dict) and "default" in spec:
                norm[key] = spec["default"]
        for k, v in default_path.items():
            if k == "type":
                continue
            if k not in schema_default:
                # strikt: unbekannte Keys sind Fehler
                raise KeyError(
                    f"Unknown default_path-Parameter '{k}' für type '{ptype}' "
                    f"in Recipe side '{side}' von '{rec_def.get('id')}'."
                )
            norm[k] = v

        # --- schemas: für alle erlaubten Typen (inkl. Default) bereitstellen ---
        types_to_resolve = list(dict.fromkeys([ptype] + list(allowed)))
        schemas: Dict[str, Dict[str, Any]] = {}
        resolved_allowed: List[str] = []

        for t in types_to_resolve:
            sch = self.schema_for_type_strict(t)
            schemas[t] = sch
            if t in allowed and t not in resolved_allowed:
                resolved_allowed.append(t)

        # allowed_path_types sind exakt resolved_allowed (Reihenfolge wie im YAML, minus ungültige)
        if not resolved_allowed:
            raise ValueError(
                f"Keiner der allowed_path_types für Recipe side '{side}' von '{rec_def.get('id')}' "
                f"konnte im recipe_params.* Schema aufgelöst werden."
            )

        return {
            "allowed_path_types": resolved_allowed,
            "default_path": norm,
            "schemas": schemas,
        }

    # ------------------------------------------------------------------ #
    # UI-Enum-Helper (optional)
    # ------------------------------------------------------------------ #
    def spiral_plane_enums(self) -> Dict[str, List[str]]:
        """
        Liest Enum-Werte für path.spiral.plane (z.B. 'direction') direkt aus dem Schema.
        """
        out: Dict[str, List[str]] = {}
        node = self._get_params_node_key("path.spiral.plane") or {}
        if isinstance(node, dict):
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["direction"] = [str(v) for v in dir_spec["values"]]
        return out

    def spiral_cylinder_enums(self) -> Dict[str, List[str]]:
        """
        Liest Enum-Werte für path.spiral.cylinder (start_from, direction) direkt aus dem Schema.
        """
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
