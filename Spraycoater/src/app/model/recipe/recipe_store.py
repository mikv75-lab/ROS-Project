# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Optional, List


class RecipeStore:
    """
    Liest die YAML-Struktur aus ctx.recipes_yaml und bietet Zugriffe für UI/Preview.

    Wichtige Punkte:
    - Kein spraycoater.defaults mehr.
    - Keine substrate_types mehr.
    - Sides hängen direkt an jedem Rezept unter recipes[i].sides.
    - Globale Defaults kommen ausschließlich aus recipe_params.globals.*.default.
    - Schemas werden STRIKT aus recipe_params.path.* gelesen (keine Hidden-Fallbacks).
    """

    # Mapping von UI-Path-Typen zu Schema-Keys in recipe_params
    _TYPE_TO_SCHEMA_KEY: Dict[str, str] = {
        "meander_plane":          "path.meander.plane",
        "spiral_plane":           "path.spiral.plane",
        "spiral_cylinder":        "path.spiral.cylinder",
        "perimeter_follow_plane": "path.perimeter_follow.plane",
        "polyhelix_cube":         "path.polyhelix.cube",
        "polyhelix_pyramid":      "path.polyhelix.pyramid",
    }

    def __init__(self, data: Dict[str, Any]):
        self.data = data or {}
        # Achtung: recipe_params ist eine flache Map mit punktigen Keys (z. B. "path.meander.plane")
        self.params_schema: Dict[str, Any] = (self.data.get("recipe_params") or {})
        self.recipes: List[Dict[str, Any]] = list((self.data.get("recipes") or []))

    # -------------------- Factory --------------------
    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        return RecipeStore(getattr(ctx, "recipes_yaml", {}) or {})

    # -------------------- Hilfsfunktionen --------------------
    @staticmethod
    def _nested_get(data: dict, dotted: str):
        """Versucht, 'a.b.c' verschachtelt zu lesen; gibt None falls nicht möglich."""
        cur = data
        for part in dotted.split("."):
            if not isinstance(cur, dict) or part not in cur:
                return None
            cur = cur[part]
        return cur

    def _get_params_node(self, key: str) -> Optional[Dict[str, Any]]:
        """
        Holt einen Knoten aus recipe_params – zuerst flach (exakter Key),
        dann optional verschachtelt (falls Struktur später geändert wird).
        """
        ps = self.params_schema or {}
        # 1) flacher Zugriff (wie in deiner aktuellen YAML)
        if key in ps and isinstance(ps[key], dict):
            return dict(ps[key])
        # 2) verschachtelte Auflösung als Absicherung gegen spätere Strukturänderungen
        node = self._nested_get(ps, key)
        return dict(node) if isinstance(node, dict) else None

    # -------------------- Queries: Rezepte --------------------
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
        # YAML-Feld heißt "substrate_mounts"
        return [str(m) for m in (rec_def.get("substrate_mounts") or [])]

    # -------------------- Globals --------------------
    def globals_schema(self) -> Dict[str, Any]:
        """
        Liefert das Schema-Objekt unter recipe_params.globals (direkt aus YAML).
        Erwartet eine Dict-Struktur: { <param>: {type:..., default:..., ...}, ... }
        """
        gs = (self.params_schema or {}).get("globals")
        return dict(gs) if isinstance(gs, dict) else {}

    def collect_global_defaults(self) -> Dict[str, Any]:
        """
        Kombiniert nur recipe_params.globals.*.default.
        (spraycoater.defaults existiert nicht mehr)
        """
        out: Dict[str, Any] = {}
        for key, spec in self.globals_schema().items():
            if isinstance(spec, dict) and "default" in spec:
                out[key] = spec["default"]
        return out

    # -------------------- Sides direkt aus dem Rezept --------------------
    def sides_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Any]:
        """recipes[i].sides → Dict[side_name, side_cfg]"""
        sides = rec_def.get("sides") or {}
        return dict(sides) if isinstance(sides, dict) else {}

    def allowed_and_default_for(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        """
        Gibt die Side-Config aus dem Rezept zurück (allowed_path_types, default_path, evtl. weitere Felder).
        KEINE Schema-Ergänzung: alles kommt direkt aus YAML.
        """
        sides = self.sides_for_recipe(rec_def)
        return deepcopy(sides.get(side) or {})

    def build_default_paths_for_recipe(self, rec_def: Dict[str, Any]) -> Dict[str, Dict[str, Any]]:
        """
        Erzeugt ein {side: default_path}-Dict für ein Rezept.
        Falls default_path fehlt, wird eine Minimalstruktur gesetzt (nur 'type' leer).
        """
        out: Dict[str, Dict[str, Any]] = {}
        for side, scfg in self.sides_for_recipe(rec_def).items():
            dp = dict((scfg.get("default_path") or {}))
            if not dp:
                dp = {"type": ""}  # UI kann das dann setzen
            out[str(side)] = dp
        return out

    # -------------------- Strikte Path-Schemata --------------------
    def schema_for_type_strict(self, ptype: str) -> Dict[str, Any]:
        """
        Liefert das Schema zu einem Path-Typ direkt aus recipe_params.path.*.
        Wirft KeyError, wenn nichts gefunden wird (strikt, kein Fallback).
        """
        p = str(ptype).strip()
        key = self._TYPE_TO_SCHEMA_KEY.get(p)
        if not key:
            raise KeyError(f"Unknown path type '{ptype}' – no schema key mapping.")
        node = self._get_params_node(key)
        if not isinstance(node, dict):
            # hilfreiche Fehlermeldung mit vorhandenen Schlüsseln:
            available = ", ".join([k for k in (self.params_schema or {}).keys() if str(k).startsWith("path.") or str(k).startswith("path.")])
            raise KeyError(
                f"Schema not found at recipe_params.{key} for type '{ptype}'. "
                f"Available path.* keys: [{available}]"
            )
        return node

    def build_side_runtime_cfg_strict(self, rec_def: Dict[str, Any], side: str) -> Dict[str, Any]:
        """
        Baut eine Runtime-Config pro Side ausschließlich aus dem Rezept + den strikten recipe_params-Schemata:
          {
            'allowed_path_types': [...],
            'default_path': {...},
            'schemas': { <ptype>: <schema-dict>, ... }
          }
        """
        sides = self.sides_for_recipe(rec_def)
        scfg = sides.get(side) or {}
        if not isinstance(scfg, dict):
            scfg = {}

        allowed: List[str] = list(scfg.get("allowed_path_types") or [])
        default_path: Dict[str, Any] = dict(scfg.get("default_path") or {})

        schemas: Dict[str, Dict[str, Any]] = {}
        for p in allowed:
            schemas[p] = self.schema_for_type_strict(p)

        return {
            "allowed_path_types": allowed,
            "default_path": default_path,
            "schemas": schemas,
        }

    # -------------------- Enums direkt aus recipe_params (UI-Helfer) --------------------
    def spiral_plane_enums(self) -> Dict[str, List[str]]:
        """
        Liest z. B. direction=['cw','ccw'] für spiral_plane aus recipe_params.
        Gibt Dict mit evtl. vorhandenen Keys (z. B. {'direction': [...]}) zurück.
        (Keine Fallback-Logik; Quelle ist allein recipe_params.)
        """
        out: Dict[str, List[str]] = {}
        node = self._get_params_node("path.spiral.plane") or {}
        if isinstance(node, dict):
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["direction"] = [str(v) for v in dir_spec["values"]]
        return out

    def spiral_cylinder_enums(self) -> Dict[str, List[str]]:
        """
        Liest start_from / direction für spiral_cylinder aus recipe_params.
        (Keine Fallback-Logik; Quelle ist allein recipe_params.)
        """
        out: Dict[str, List[str]] = {}
        node = self._get_params_node("path.spiral.cylinder") or {}
        if isinstance(node, dict):
            sf_spec = node.get("start_from")
            if isinstance(sf_spec, dict) and isinstance(sf_spec.get("values"), list):
                out["start_froms"] = [str(v) for v in sf_spec["values"]]
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["directions"] = [str(v) for v in dir_spec["values"]]
        return out
