# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, Optional, List


class RecipeStore:
    """Liest die YAML-Struktur aus ctx.recipes_yaml und bietet bequeme Zugriffe für UI/Preview."""
    def __init__(self, data: Dict[str, Any]):
        self.data = data or {}
        self.defaults = (self.data.get("spraycoater") or {}).get("defaults") or {}
        self.params_schema = (self.data.get("recipe_params") or {})
        self.substrate_types = (self.data.get("substrate_types") or {})
        self.recipes: List[Dict[str, Any]] = list((self.data.get("recipes") or []))

    # -------------------- Factory --------------------
    @staticmethod
    def from_ctx(ctx) -> "RecipeStore":
        return RecipeStore(getattr(ctx, "recipes_yaml", {}) or {})

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

    # -------------------- Defaults / Schema --------------------
    def collect_global_defaults(self) -> Dict[str, Any]:
        """Kombiniert spraycoater.defaults + recipe_params.globals.*.default."""
        params = dict(self.defaults)
        globals_schema = (self.params_schema or {}).get("globals", {}) or {}
        for k, spec in globals_schema.items():
            if k not in params and isinstance(spec, dict) and "default" in spec:
                params[k] = spec["default"]
        return params

    # -------------------- Substrate Types / Sides --------------------
    def sides_for_substrate_type(self, stype: str) -> Dict[str, Any]:
        """substrate_types.<stype>.sides → Dict[side_name, side_cfg]."""
        return ((self.substrate_types.get(stype) or {}).get("sides") or {})

    def allowed_and_default_for(self, stype: str, side: str) -> Dict[str, Any]:
        """
        Gibt die komplette Side-Config zurück (allowed_path_types, default_path, ggf. Enums).
        Ergänzt fehlende Enums aus recipe_params (Fallback), falls notwendig.
        """
        sides = self.sides_for_substrate_type(stype)
        cfg = dict(sides.get(side) or {})
        return self.sanitize_side_config_with_schema(cfg)

    # -------------------- Enums aus recipe_params (Fallback) --------------------
    def spiral_plane_enums(self) -> Dict[str, List[str]]:
        """
        Liest z. B. direction=['cw','ccw'] für spiral_plane aus recipe_params.
        Gibt Dict mit evtl. vorhandenen Keys (z. B. {'direction': [...]}) zurück.
        """
        out: Dict[str, List[str]] = {}
        node = (self.params_schema.get("path.spiral.plane") if self.params_schema else None) or {}
        if isinstance(node, dict):
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["direction"] = [str(v) for v in dir_spec["values"]]
        return out

    def spiral_cylinder_enums(self) -> Dict[str, List[str]]:
        """
        Liest start_from / direction für spiral_cylinder aus recipe_params.
        """
        out: Dict[str, List[str]] = {}
        node = (self.params_schema.get("path.spiral.cylinder") if self.params_schema else None) or {}
        if isinstance(node, dict):
            sf_spec = node.get("start_from")
            if isinstance(sf_spec, dict) and isinstance(sf_spec.get("values"), list):
                out["start_froms"] = [str(v) for v in sf_spec["values"]]
            dir_spec = node.get("direction")
            if isinstance(dir_spec, dict) and isinstance(dir_spec.get("values"), list):
                out["directions"] = [str(v) for v in dir_spec["values"]]
        return out

    def sanitize_side_config_with_schema(self, side_cfg: Dict[str, Any]) -> Dict[str, Any]:
        """
        Ergänzt fehlende Enum-Listen anhand recipe_params-Fallbacks.
        - Für spiral_cylinder: start_froms / directions
        - Für spiral_plane: direction
        Gibt eine neue Dict-Kopie zurück.
        """
        cfg = dict(side_cfg or {})
        default_path = dict((cfg.get("default_path") or {}))
        ptype = str(default_path.get("type", "")).strip()

        # helix enums
        if ptype == "spiral_cylinder":
            # wenn side_cfg selbst keine Listen hat, aus schema nehmen
            if "start_froms" not in cfg or not cfg.get("start_froms"):
                helix = self.spiral_cylinder_enums()
                if helix.get("start_froms"):
                    cfg["start_froms"] = list(helix["start_froms"])
            if "directions" not in cfg or not cfg.get("directions"):
                helix = self.spiral_cylinder_enums()
                if helix.get("directions"):
                    cfg["directions"] = list(helix["directions"])

        # spiral_plane direction
        if ptype == "spiral_plane":
            # kein Feld im side_cfg nötig – UI kann aus recipe_params lesen, falls benötigt
            # Hier fügen wir nichts in cfg ein, da die Page selbst den Enum aus schema nutzen kann.
            pass

        return cfg
