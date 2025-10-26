# -*- coding: utf-8 -*-
"""
Strict startup loader (keine Fallbacks)

Erwartet in startup.yaml (unter Spraycoater/resource/config/):
  paths.recipe_file : YAML mit 'units', 'recipe_params', 'recipes'
  paths.recipe_dir  : Zielordner für Pläne (existiert)
  paths.log_dir     : Log-Ordner (existiert)
  paths.bringup_log : Datei-Pfad für Bringup-Log (Parent existiert)
  paths.ros_setup   : OPTIONAL Pfad zu /opt/ros/.../setup.bash
  paths.ws_setup    : OPTIONAL Pfad zu <ws>/install/setup.bash

  rviz.live_config   : OPTIONAL Pfad zur RViz-Config (falls gesetzt: muss existieren)
  rviz.shadow_config : OPTIONAL Pfad zur RViz-Config (falls gesetzt: muss existieren)

  ros.launch_ros    : bool
  ros.sim_robot     : bool
"""

from __future__ import annotations
import os
import io
import yaml
from dataclasses import dataclass
from typing import Any, Dict, List, Optional



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
    p = os.path.expanduser(p)  # ~ unterstützen
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
    # optional
    ros_setup: Optional[str] = None
    ws_setup: Optional[str] = None

@dataclass(frozen=True)
class RVizConfig:
    live_config: Optional[str] = None
    shadow_config: Optional[str] = None

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
    rviz: RVizConfig = RVizConfig()  # optionales Feld


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

    # OPTIONAL: ROS-Setups
    ros_setup_abs: Optional[str] = None
    ws_setup_abs: Optional[str] = None
    if "ros_setup" in p and p["ros_setup"]:
        ros_setup_abs = _abspath_rel_to(base, p["ros_setup"])
        if not os.path.exists(ros_setup_abs):
            _err(f"paths.ros_setup nicht gefunden: {ros_setup_abs}")
    if "ws_setup" in p and p["ws_setup"]:
        ws_setup_abs = _abspath_rel_to(base, p["ws_setup"])
        if not os.path.exists(ws_setup_abs):
            _err(f"paths.ws_setup nicht gefunden: {ws_setup_abs}")

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

    # OPTIONAL: rviz
    rviz_cfg = RVizConfig()
    if "rviz" in su:
        if not isinstance(su["rviz"], dict):
            _err("startup.yaml: Abschnitt 'rviz' ist ungültig (kein Mapping).")
        rv = su["rviz"]
        if "live_config" in rv and rv["live_config"]:
            live_abs = _abspath_rel_to(base, rv["live_config"])
            if not os.path.exists(live_abs):
                _err(f"rviz.live_config nicht gefunden: {live_abs}")
            rviz_cfg = RVizConfig(live_config=live_abs, shadow_config=rviz_cfg.shadow_config)
        if "shadow_config" in rv and rv["shadow_config"]:
            shadow_abs = _abspath_rel_to(base, rv["shadow_config"])
            if not os.path.exists(shadow_abs):
                _err(f"rviz.shadow_config nicht gefunden: {shadow_abs}")
            rviz_cfg = RVizConfig(live_config=rviz_cfg.live_config, shadow_config=shadow_abs)

    return AppContext(
        paths=AppPaths(
            recipe_file=recipe_file_abs,
            recipe_dir=recipe_dir_abs,
            log_dir=log_dir_abs,
            bringup_log=bringup_log_abs,
            ros_setup=ros_setup_abs,
            ws_setup=ws_setup_abs,
        ),
        ros=ROSConfig(launch_ros=r["launch_ros"], sim_robot=r["sim_robot"]),
        recipes_yaml=ry,
        recipes=ry["recipes"],
        recipe_params=ry["recipe_params"],
        units=ry["units"],
        rviz=rviz_cfg,
    )