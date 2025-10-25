# Spraycoater/src/config/startup.py
# -*- coding: utf-8 -*-
"""
Strict startup loader (keine Fallbacks):
- Erwartet in startup.yaml (liegt unter Spraycoater/resource/config/):
    paths.recipe_file : YAML mit recipes + recipe_params (z. B. "recipes.yaml")
    paths.recipe_dir  : Zielordner für Pläne (muss existieren)  -> z. B. "../../data/recipes"
    paths.log_dir     : Log-Ordner (muss existieren)            -> z. B. "../../data/logs"
    paths.bringup_log : Datei-Pfad für Bringup-Log              -> z. B. "../../data/logs/bringup.log"
    ros.launch_ros    : bool
    ros.sim_robot     : bool   (True = Simulation, False = echter Roboter)

- Lädt recipes.yaml strikt ('units' == 'mm', 'recipe_params', 'recipes')
- Keine Auto-Validierung; nur explizite Wrapper:
    validate_recipes_for_load(recipes_yaml)
    validate_recipe_for_save(recipe, recipe_params)
"""

from __future__ import annotations
import os
import io
import yaml
from dataclasses import dataclass
from typing import Any, Dict, List

# Muss existieren (Achtung: darf NICHT wieder config.startup importieren!)
from app.recipe_validator import RecipeValidator


def _err(msg: str) -> None:
    raise ValueError(msg)


def resolve_package_uri(uri: str) -> str:
    """Strikte ROS2 package:// Auflösung (harte Fehler bei Ungültigkeit)."""
    if not uri.startswith("package://"):
        return uri
    from ament_index_python.packages import get_package_share_directory
    try:
        pkg, rel = uri[len("package://"):].split("/", 1)
    except ValueError:
        _err(f"Ungültige package-URI: '{uri}' (erwartet package://<pkg>/relpath)")
    base = get_package_share_directory(pkg)
    return os.path.join(base, rel)


def _abspath_rel_to(base_dir: str, p: str) -> str:
    if not p:
        _err("Leerer Pfad übergeben.")
    if p.startswith("package://"):
        path = resolve_package_uri(p)
    else:
        path = p if os.path.isabs(p) else os.path.join(base_dir, p)
    return os.path.abspath(os.path.normpath(path))


def _load_yaml_strict(path_or_uri: str) -> Dict[str, Any]:
    """YAML strikt laden (Datei existiert, Mapping, nicht leer)."""
    path = resolve_package_uri(path_or_uri) if path_or_uri.startswith("package://") else path_or_uri
    path = os.path.abspath(os.path.normpath(path))
    if not os.path.exists(path):
        _err(f"YAML nicht gefunden: {path}")
    with io.open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None or not isinstance(data, dict) or not data:
        _err(f"YAML leer oder ungültig (kein Mapping): {path}")
    return data


# ---------------- Datamodel ----------------

@dataclass(frozen=True)
class AppPaths:
    recipe_file: str
    recipe_dir: str
    log_dir: str
    bringup_log: str

@dataclass(frozen=True)
class ROSConfig:
    launch_ros: bool
    sim_robot: bool  # True = sim, False = real

@dataclass(frozen=True)
class AppContext:
    paths: AppPaths
    ros: ROSConfig
    recipes_yaml: Dict[str, Any]
    recipes: List[Dict[str, Any]]
    recipe_params: Dict[str, Any]
    units: str


# ---------------- Loader ----------------

def load_startup(startup_yaml_path: str) -> AppContext:
    if not startup_yaml_path:
        _err("startup_yaml_path ist leer.")
    startup_yaml_path = os.path.abspath(os.path.normpath(startup_yaml_path))
    if not os.path.exists(startup_yaml_path):
        _err(f"startup.yaml nicht gefunden: {startup_yaml_path}")

    su = _load_yaml_strict(startup_yaml_path)

    # paths
    if "paths" not in su or not isinstance(su["paths"], dict):
        _err("startup.yaml: Abschnitt 'paths' fehlt oder ist ungültig.")
    p = su["paths"]
    for key in ("recipe_file", "recipe_dir", "log_dir", "bringup_log"):
        if key not in p:
            _err(f"startup.yaml: 'paths.{key}' fehlt.")

    base = os.path.dirname(startup_yaml_path)
    recipe_file_abs = _abspath_rel_to(base, p["recipe_file"])
    recipe_dir_abs  = _abspath_rel_to(base, p["recipe_dir"])
    log_dir_abs     = _abspath_rel_to(base, p["log_dir"])
    bringup_log_abs = _abspath_rel_to(base, p["bringup_log"])

    # Existenz prüfen (keine Erstellung hier!)
    if not os.path.isdir(recipe_dir_abs):
        _err(f"paths.recipe_dir existiert nicht als Verzeichnis: {recipe_dir_abs}")
    if not os.path.isdir(log_dir_abs):
        _err(f"paths.log_dir existiert nicht als Verzeichnis: {log_dir_abs}")
    bl_parent = os.path.dirname(bringup_log_abs) or "."
    if not os.path.isdir(bl_parent):
        _err(f"Verzeichnis für paths.bringup_log existiert nicht: {bl_parent}")

    # recipes.yaml strikt laden
    ry = _load_yaml_strict(recipe_file_abs)
    if "units" not in ry or not isinstance(ry["units"], str):
        _err("recipes.yaml: Feld 'units' fehlt oder ist ungültig.")
    if ry["units"] != "mm":
        _err(f"recipes.yaml: units='{ry['units']}' nicht unterstützt (erwartet 'mm').")
    if "recipe_params" not in ry or not isinstance(ry["recipe_params"], dict) or not ry["recipe_params"]:
        _err("recipes.yaml: Abschnitt 'recipe_params' fehlt oder ist leer.")
    if "recipes" not in ry or not isinstance(ry["recipes"], list) or not ry["recipes"]:
        _err("recipes.yaml: Abschnitt 'recipes' fehlt oder ist leer.")

    # ros
    if "ros" not in su or not isinstance(su["ros"], dict):
        _err("startup.yaml: Abschnitt 'ros' fehlt oder ist ungültig.")
    r = su["ros"]
    if "launch_ros" not in r or not isinstance(r["launch_ros"], bool):
        _err("startup.yaml: 'ros.launch_ros' fehlt oder ist kein Bool.")
    if "sim_robot" not in r or not isinstance(r["sim_robot"], bool):
        _err("startup.yaml: 'ros.sim_robot' fehlt oder ist kein Bool.")

    return AppContext(
        paths=AppPaths(
            recipe_file=recipe_file_abs,
            recipe_dir=recipe_dir_abs,
            log_dir=log_dir_abs,
            bringup_log=bringup_log_abs,
        ),
        ros=ROSConfig(launch_ros=r["launch_ros"], sim_robot=r["sim_robot"]),
        recipes_yaml=ry,
        recipes=ry["recipes"],
        recipe_params=ry["recipe_params"],
        units=ry["units"],
    )


# ---------------- explizite Checks NUR für Load/Save ----------------

def validate_recipes_for_load(recipes_yaml: Dict[str, Any]) -> List[str]:
    """Alle Rezepte aus recipes.yaml prüfen (nur auf Abruf)."""
    if "recipe_params" not in recipes_yaml or "recipes" not in recipes_yaml:
        _err("validate_recipes_for_load: 'recipe_params' oder 'recipes' fehlt.")
    v = RecipeValidator(recipes_yaml["recipe_params"])
    errors: List[str] = []
    for rec in recipes_yaml["recipes"]:
        errs = v.validate_recipe(rec)
        if errs:
            errors.extend(errs)
    return errors


def validate_recipe_for_save(recipe: Dict[str, Any], recipe_params: Dict[str, Any]) -> List[str]:
    """Genau ein Rezept prüfen (vor Speichern/Export)."""
    if not isinstance(recipe, dict) or not recipe:
        _err("validate_recipe_for_save: 'recipe' ist leer oder kein Mapping.")
    if not isinstance(recipe_params, dict) or not recipe_params:
        _err("validate_recipe_for_save: 'recipe_params' ist leer oder kein Mapping.")
    return RecipeValidator(recipe_params).validate_recipe(recipe)
