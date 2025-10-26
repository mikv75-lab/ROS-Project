# -*- coding: utf-8 -*-
"""
RecipeValidator – strikter Validator NUR für Laden/Speichern.
Prüft:
- Pfad-Schema (meander/spiral/explicit)
- Min/Max/Enums/Typen gemäß recipe_params
- Top/Helix-Regeln für spray_angle_deg
- Unknown keys in recipe.parameters

Verwendung:
    import yaml
    from app.recipe_validator import RecipeValidator

    data = yaml.safe_load(open("config/recipes.yaml", "r", encoding="utf-8"))
    specs = data["recipe_params"]

    validator = RecipeValidator(specs)

    # Einzelnes Rezept prüfen (beim Laden oder vor dem Speichern eines Plans)
    errors = validator.validate_recipe(recipe_dict)
    if errors:
        raise ValueError("Recipe invalid:\n" + "\n".join(f"- {e}" for e in errors))

    # Alle in recipes.yaml
    for rec in data.get("recipes", []):
        errs = validator.validate_recipe(rec)
        if errs:
            print(rec.get("id", "<no-id>"), "INVALID:")
            for e in errs: print("  -", e)

Hinweis:
- Dieser Validator setzt KEINE Defaults und ändert nichts – er prüft nur.
- recipe_params: erwartet Abschnitte:
    globals, path.spiral.plane, path.spiral.cylinder, path.meander.plane, path.explicit
"""

from __future__ import annotations
from typing import Any, Dict, List, Tuple, Optional


def _dot_get(d: dict, dotted: str, *, missing=object()) -> Any:
    cur = d
    for part in dotted.split("."):
        if not isinstance(cur, dict) or part not in cur:
            return missing
        cur = cur[part]
    return cur

def _is_number(x: Any) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool)

class RecipeValidator:
    # Pflichtfelder für Pfade (Schema-Level, unabhängig von Specs)
    REQUIRED_SPIRAL_PLANE = ("center_xy_mm", "r_start_mm", "r_end_mm", "pitch_mm", "z_mm")
    REQUIRED_SPIRAL_CYL   = ("pitch_mm", "outside_mm", "margin_top_mm", "margin_bottom_mm")
    REQUIRED_MEANDER_BASE = ("pitch_mm", "angle_deg")
    REQUIRED_EXPLICIT     = ("points",)

    def __init__(self, recipe_params: Dict[str, Any]) -> None:
        self.specs = recipe_params or {}
        if "globals" not in self.specs:
            raise ValueError("recipe_params.globals fehlt.")

    # ---------------- PUBLIC ---------------- #

    def validate_recipe(self, recipe: Dict[str, Any]) -> List[str]:
        """Liefert eine Liste von Fehlermeldungen (leer = OK)."""
        errors: List[str] = []
        rid = recipe.get("id", "<no-id>")

        # 1) Grundstruktur
        for fld in ("tool", "mount", "side", "path"):
            if fld not in recipe:
                errors.append(f"[{rid}] Feld '{fld}' fehlt.")
        if "substrates" not in recipe or not isinstance(recipe["substrates"], list) or not recipe["substrates"]:
            errors.append(f"[{rid}] 'substrates' muss eine nicht-leere Liste sein.")

        # 2) Top/Helix-Regeln (nur Parameter-Prüfung, keine Defaults)
        params = recipe.get("parameters", {})
        side = recipe.get("side")
        if side == "top":
            if "spray_angle_deg" in params and float(params["spray_angle_deg"]) != 0.0:
                errors.append(f"[{rid}] side='top' erzwingt parameters.spray_angle_deg = 0.")
        # helix-Erfordernis prüfen später, wenn spiral.cylinder erkannt wurde

        # 3) Pfad prüfen
        path = recipe.get("path", {})
        ptype = path.get("type")
        if ptype not in ("spiral", "meander", "explicit"):
            errors.append(f"[{rid}] path.type muss 'spiral' | 'meander' | 'explicit' sein.")
            return errors  # ohne gültigen Typ sind weitere Prüfungen sinnlos

        if ptype == "spiral":
            mode = path.get("mode")
            if mode not in ("plane", "cylinder"):
                errors.append(f"[{rid}] spiral.mode muss 'plane' oder 'cylinder' sein.")
                return errors
            if mode == "plane":
                self._require_fields(path, self.REQUIRED_SPIRAL_PLANE, rid, "path.spiral.plane", errors)
                self._validate_section_specs(path, "path.spiral.plane", rid, errors)
            else:
                self._require_fields(path, self.REQUIRED_SPIRAL_CYL, rid, "path.spiral.cylinder", errors)
                self._validate_section_specs(path, "path.spiral.cylinder", rid, errors)
                # Helix: spray_angle_deg MUSS vorhanden sein (in globals/parameters)
                angle_val = params.get("spray_angle_deg", None)
                if angle_val is None:
                    errors.append(f"[{rid}] helix: parameters.spray_angle_deg fehlt (erforderlich für spiral.cylinder).")
                else:
                    self._validate_single(("globals", "spray_angle_deg"), angle_val, rid, "parameters.spray_angle_deg", errors)

        elif ptype == "meander":
            mode = path.get("mode")
            if mode != "plane":
                errors.append(f"[{rid}] meander.mode muss 'plane' sein.")
            area = path.get("area", {})
            shape = area.get("shape")
            if shape not in ("circle", "rect"):
                errors.append(f"[{rid}] meander.area.shape muss 'circle' oder 'rect' sein.")
            if "center_xy_mm" not in area:
                errors.append(f"[{rid}] meander.area.center_xy_mm fehlt.")
            # radius vs size
            if shape == "circle":
                if "radius_mm" not in area:
                    errors.append(f"[{rid}] meander.area.radius_mm fehlt (bei shape=circle).")
            elif shape == "rect":
                if "size_mm" not in area:
                    errors.append(f"[{rid}] meander.area.size_mm fehlt (bei shape=rect).")
            self._require_fields(path, self.REQUIRED_MEANDER_BASE, rid, "path.meander", errors)
            self._validate_section_specs(path, "path.meander.plane", rid, errors)

        else:  # explicit
            self._require_fields(path, self.REQUIRED_EXPLICIT, rid, "path.explicit", errors)
            # Minimalprüfung der Punktliste
            pts = path.get("points", [])
            if not isinstance(pts, list) or len(pts) == 0:
                errors.append(f"[{rid}] path.explicit.points muss nicht-leere Liste sein.")
            else:
                needed = ("x","y","z","qx","qy","qz","qw")
                for i, pt in enumerate(pts):
                    if not isinstance(pt, dict):
                        errors.append(f"[{rid}] points[{i}] muss dict sein.")
                        continue
                    for k in needed:
                        if k not in pt:
                            errors.append(f"[{rid}] points[{i}].{k} fehlt.")

            # optional: Abschnitts-Specs prüfen, falls vorhanden
            if "path.explicit" in self.specs:
                self._validate_section_specs(path, "path.explicit", rid, errors)

        # 4) Globale Parameter min/max/enums prüfen (nur vorhandene)
        self._validate_parameters_map(params, rid, errors)

        # 5) Unbekannte Parameter-Keys fangen
        self._check_unknown_params(params, rid, errors)

        return errors

    # ---------------- INTERNALS ---------------- #

    def _require_fields(self, d: dict, fields: Tuple[str, ...], rid: str, prefix: str, errors: List[str]) -> None:
        for f in fields:
            if f not in d:
                errors.append(f"[{rid}] {prefix}: Feld '{f}' fehlt.")

    def _validate_parameters_map(self, params: dict, rid: str, errors: List[str]) -> None:
        spec = self.specs.get("globals", {})
        for key, val in params.items():
            if key not in spec:
                errors.append(f"[{rid}] parameters: unbekannter Schlüssel '{key}'.")
                continue
            self._validate_single(("globals", key), val, rid, f"parameters.{key}", errors)

    def _check_unknown_params(self, params: dict, rid: str, errors: List[str]) -> None:
        # Bereits in _validate_parameters_map abgedeckt (unbekannte Keys -> Fehler)
        return

    def _validate_section_specs(self, path_obj: dict, spec_key: str, rid: str, errors: List[str]) -> None:
        """
        Prüft Werte im 'path' gegen recipe_params[spec_key].
        Berücksichtigt visible_if (dot-keys) und Typen (number/enum/boolean/vec2).
        """
        section = self.specs.get(spec_key, {})
        if not section:
            return
        # Kontext für visible_if & dot-Keys: kompletter path-Objektbaum
        ctx = path_obj

        for dotted_key, meta in section.items():
            # Sichtbarkeit prüfen
            if "visible_if" in meta:
                cond_key, cond_val = next(iter(meta["visible_if"].items()))
                cur = _dot_get(ctx, cond_key)
                if cur != cond_val:
                    continue  # nicht sichtbar -> nicht prüfen

            val = _dot_get(ctx, dotted_key, missing=None)
            if val is None:
                # Nicht zwingend vorhanden: Pflichtfelder werden separat geprüft.
                continue

            # Validierung dieses Wertes
            self._validate_value(val, meta, rid, f"path.{dotted_key}", errors)

    def _validate_single(self, spec_addr: Tuple[str, str], val: Any, rid: str, label: str, errors: List[str]) -> None:
        spec_section, key = spec_addr
        meta = self.specs.get(spec_section, {}).get(key)
        if not meta:
            # Kein Spec -> keine Rangeprüfung
            return
        self._validate_value(val, meta, rid, label, errors)

    def _validate_value(self, val: Any, meta: dict, rid: str, label: str, errors: List[str]) -> None:
        t = meta.get("type")
        if t == "number":
            if not _is_number(val):
                errors.append(f"[{rid}] {label}: muss Zahl sein.")
                return
            lo = meta.get("min"); hi = meta.get("max")
            if lo is not None and float(val) < float(lo):
                errors.append(f"[{rid}] {label}: {val} < min {lo}.")
            if hi is not None and float(val) > float(hi):
                errors.append(f"[{rid}] {label}: {val} > max {hi}.")
        elif t == "boolean":
            if not isinstance(val, bool):
                errors.append(f"[{rid}] {label}: muss boolean sein.")
        elif t == "enum":
            values = meta.get("values", [])
            if val not in values:
                errors.append(f"[{rid}] {label}: '{val}' nicht in {values}.")
        elif t == "vec2":
            if (not isinstance(val, (list, tuple))) or len(val) != 2 or not all(_is_number(v) for v in val):
                errors.append(f"[{rid}] {label}: muss Vektor[2] aus Zahlen sein.")
                return
            minv = meta.get("min"); maxv = meta.get("max")
            if isinstance(minv, (list, tuple)) and isinstance(maxv, (list, tuple)) and len(minv)==2 and len(maxv)==2:
                for i in range(2):
                    if float(val[i]) < float(minv[i]) or float(val[i]) > float(maxv[i]):
                        errors.append(f"[{rid}] {label}[{i}]: {val[i]} außerhalb [{minv[i]}, {maxv[i]}].")
        else:
            # unbekannter Typ -> nicht prüfen
            pass
