# Spraycoater/src/app/tabs/recipe/recipe_io.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import yaml
from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple, Optional

from .recipe_validator import RecipeValidator


@dataclass
class RecipeIoResult:
    ok: bool
    message: str = ""
    data: Any = None
    errors: List[str] = field(default_factory=list)


# ---------------------------- Load ----------------------------

def load_all(recipes_yaml_path: str, *, validate_all: bool = True) -> Tuple[Dict[str, Any], List[str]]:
    """
    Lädt die komplette recipes.yaml strikt.
    Gibt (yaml_dict, fehler_liste) zurück. Bei validate_all=False bleibt fehler_liste leer.
    """
    with open(recipes_yaml_path, "r", encoding="utf-8") as f:
        ry = yaml.safe_load(f)

    if not isinstance(ry, dict):
        raise ValueError("recipes.yaml ungültig (kein Mapping).")
    if "recipes" not in ry or "recipe_params" not in ry:
        raise ValueError("recipes.yaml: Abschnitte 'recipes' und/oder 'recipe_params' fehlen.")

    if not validate_all:
        return ry, []

    validator = RecipeValidator(ry["recipe_params"])
    errs: List[str] = []
    for rec in ry["recipes"]:
        es = validator.validate_recipe(rec)
        if es:
            errs.extend([f"[{rec.get('id','?')}] {m}" for m in es])

    return ry, errs


def load_single(recipes_yaml_path: str, recipe_id: str, *, validate_one: bool = False) -> Tuple[Dict[str, Any], Optional[List[str]]]:
    """
    Holt ein einzelnes Rezept (per ID) aus recipes.yaml.
    Returns: (recipe, errors|None). Wenn validate_one=True, werden Fehler dieses Rezepts mitgegeben.
    """
    ry, _ = load_all(recipes_yaml_path, validate_all=False)
    recipes = ry.get("recipes", [])
    for rec in recipes:
        if str(rec.get("id")) == str(recipe_id):
            if not validate_one:
                return rec, None
            es = RecipeValidator(ry["recipe_params"]).validate_recipe(rec)
            return rec, es or []
    raise KeyError(f"Rezept '{recipe_id}' nicht gefunden.")


# ---------------------------- Save / Delete (einzelne Dateien im recipe_dir) ----------------------------

def save_recipe_file(target_dir: str, recipe: Dict[str, Any], *,
                     specs: Optional[Dict[str, Any]] = None,
                     validate_first: bool = True) -> RecipeIoResult:
    """
    Speichert EIN Rezept als eigene YAML-Datei <id>.yaml in target_dir.
    Optional syntaktische Validierung gegen 'specs' (recipe_params).
    """
    rid = recipe.get("id") or "recipe"
    if validate_first:
        if not specs:
            return RecipeIoResult(False, "Specs (recipe_params) fehlen für Validierung.")
        es = RecipeValidator(specs).validate_recipe(recipe)
        if es:
            return RecipeIoResult(False, "Rezept fehlerhaft.", errors=es)

    os.makedirs(target_dir, exist_ok=True)
    path = os.path.join(target_dir, f"{rid}.yaml")
    try:
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(recipe, f, allow_unicode=True, sort_keys=False)
        return RecipeIoResult(True, f"Gespeichert: {path}", data={"path": path})
    except Exception as e:
        return RecipeIoResult(False, f"Speicherfehler: {e}")


def delete_recipe_file(target_dir: str, recipe_id: str) -> RecipeIoResult:
    """
    Löscht die Datei <id>.yaml aus target_dir (falls vorhanden).
    """
    path = os.path.join(target_dir, f"{recipe_id}.yaml")
    if not os.path.exists(path):
        return RecipeIoResult(False, f"Datei nicht gefunden: {path}")
    try:
        os.remove(path)
        return RecipeIoResult(True, f"Gelöscht: {path}", data={"path": path})
    except Exception as e:
        return RecipeIoResult(False, f"Löschen fehlgeschlagen: {e}")


# ---------------------------- Validate / Optimize ----------------------------

def validate_recipe(recipe: Dict[str, Any],
                    specs: Dict[str, Any],
                    *,
                    bridge: Any = None,
                    syntactic_only: bool = False,
                    timeout: float = 0.0) -> RecipeIoResult:
    """
    Führt IMMER zuerst die syntaktische Validierung lokal aus.
    Wenn syntactic_only=True ODER keine Bridge verbunden -> nur lokal.
    Sonst (bei Bridge): ruft zusätzlich bridge.validate(recipe, timeout=...) auf.

    Erwartete Bridge-Signatur (deine UIBridge):
        - property: is_connected
        - validate(recipe: dict, syntactic_only: bool = False, timeout: float = 0.0) -> BridgeResult
          (BridgeResult besitzt 'ok', 'message', 'data')
    """
    # 1) syntaktisch (immer)
    es = RecipeValidator(specs).validate_recipe(recipe)
    if es:
        return RecipeIoResult(False, "Rezept fehlerhaft.", errors=es)

    # 2) optional ROS (live)
    if syntactic_only or bridge is None or not getattr(bridge, "is_connected", False):
        msg = "Syntaktische Prüfung OK (ROS optional/nicht verbunden)."
        return RecipeIoResult(True, msg)

    try:
        br = bridge.validate(recipe, syntactic_only=False, timeout=timeout)
        # br kann bereits eine ROS-seitige Validierung geliefert haben
        return RecipeIoResult(bool(br.ok), str(br.message), data=getattr(br, "data", None))
    except Exception as e:
        return RecipeIoResult(False, f"ROS-Validate fehlgeschlagen: {e}")


def optimize_recipe(recipe: Dict[str, Any], *, bridge: Any = None, timeout: float = 0.0) -> RecipeIoResult:
    """
    Stößt eine Optimierung via Bridge an (ROS-Seite).
    Lokal wird NICHT „gefaked“. Ohne Bridge oder Verbindung: Fehler.
    Erwartete Bridge-Signatur:
        - property: is_connected
        - optimize(recipe: dict, timeout: float = 0.0) -> BridgeResult
    """
    if bridge is None or not getattr(bridge, "is_connected", False):
        return RecipeIoResult(False, "Keine ROS-Verbindung für optimize().")

    try:
        br = bridge.optimize(recipe, timeout=timeout)
        return RecipeIoResult(bool(br.ok), str(br.message), data=getattr(br, "data", None))
    except Exception as e:
        return RecipeIoResult(False, f"ROS-Optimize fehlgeschlagen: {e}")
