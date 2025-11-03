# -*- coding: utf-8 -*-
from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, Optional, List


class RecipeStore:
    """
    Liest die YAML-Struktur aus ctx.recipes_yaml und bietet Zugriffe für UI/Preview.

    NEU:
    - Kein spraycoater.defaults mehr.
    - Keine substrate_types mehr.
    - Sides hängen direkt an jedem Rezept unter recipes[i].sides.
    - Globale Defaults kommen ausschließlich aus recipe_params.globals.*.default.
    """
    def __init__(self, data: Dict[str, Any]):
        self.data = data or {}
        self.params_schema: Dict[str, Any] = (self.data.get("recipe_params") or {})
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
        """
        Kombiniert nur recipe_params.globals.*.default.
        (spraycoater.defaults existiert nicht mehr)
        """
        out: Dict[str, Any] = {}
        globals_schema = (self.params_schema or {}).get("globals", {}) or {}
        for key, spec in globals_schema.items():
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
        Gibt die komplette Side-Config zurück (allowed_path_types, default_path, ggf. Enums),
        ergänzt fehlende Enums aus recipe_params (Fallback), falls notwendig.
        """
        sides = self.sides_for_recipe(rec_def)
        cfg = deepcopy(sides.get(side) or {})
        return self._sanitize_side_config_with_schema(cfg)

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
        """Liest start_from / direction für spiral_cylinder aus recipe_params."""
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

    def _sanitize_side_config_with_schema(self, side_cfg: Dict[str, Any]) -> Dict[str, Any]:
        """
        Ergänzt fehlende Enum-Listen anhand recipe_params-Fallbacks.
        - Für spiral_cylinder: start_froms / directions
        - Für spiral_plane: UI liest 'direction' direkt aus recipe_params, daher kein Inject nötig.
        Gibt eine neue Dict-Kopie zurück.
        """
        cfg = deepcopy(side_cfg or {})
        default_path = dict((cfg.get("default_path") or {}))
        ptype = str(default_path.get("type", "")).strip()

        # helix enums (nur wenn nötig)
        if ptype == "spiral_cylinder":
            helix = self.spiral_cylinder_enums()
            if not cfg.get("start_froms") and helix.get("start_froms"):
                cfg["start_froms"] = list(helix["start_froms"])
            if not cfg.get("directions") and helix.get("directions"):
                cfg["directions"] = list(helix["directions"])

        return cfg
