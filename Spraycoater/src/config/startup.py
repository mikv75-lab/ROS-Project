# -*- coding: utf-8 -*-
# Minimaler Startup-Loader: liest nur die Configs, keine harten Fallbacks / Checks
from __future__ import annotations
import os
import io
import yaml
import warnings
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple


# ---------- kleine Helpers ----------

def _err(msg: str) -> None:
    raise ValueError(msg)

def resolve_package_uri(uri: str) -> str:
    """Einfache ROS2 package:// Auflösung. Wenn kein package://, unverändert zurück."""
    if not isinstance(uri, str) or not uri.startswith("package://"):
        return uri
    try:
        from ament_index_python.packages import get_package_share_directory
    except Exception as e:
        _err(f"ament_index_python nicht verfügbar für package-URI '{uri}': {e}")
    try:
        pkg, rel = uri[len("package://"):].split("/", 1)
    except ValueError:
        _err(f"Ungültige package-URI: '{uri}' (erwartet package://<pkg>/relpath)")
    base = get_package_share_directory(pkg)
    return os.path.join(base, rel)

def _abspath_rel_to(base_dir: str, p: str) -> str:
    if not p:
        _err("Leerer Pfad übergeben.")
    p = os.path.expanduser(p)
    if p.startswith("package://"):
        path = resolve_package_uri(p)
    else:
        path = p if os.path.isabs(p) else os.path.join(base_dir, p)
    return os.path.abspath(os.path.normpath(path))

def _load_yaml(path_or_uri: str, *, strict: bool = True) -> Optional[Dict[str, Any]]:
    """YAML laden. Bei strict=True -> harte Fehler; sonst: None bei Problemen."""
    try:
        path = resolve_package_uri(path_or_uri) if isinstance(path_or_uri, str) and path_or_uri.startswith("package://") else path_or_uri
        path = os.path.abspath(os.path.normpath(path))
        if not os.path.exists(path):
            if strict:
                _err(f"YAML nicht gefunden: {path}")
            return None
        with io.open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if data is None or not isinstance(data, dict) or not data:
            if strict:
                _err(f"YAML leer oder ungültig (kein Mapping): {path}")
            return None
        return data
    except Exception as e:
        if strict:
            _err(str(e))
        warnings.warn(f"YAML konnte nicht geladen werden ({path_or_uri}): {e}")
        return None


# ---------- Datenstrukturen ----------

@dataclass(frozen=True)
class AppPaths:
    recipe_file: str
    recipe_dir: str
    log_dir: str
    bringup_log: str
    # optional genutzt:
    substrate_mounts_dir: Optional[str] = None
    substrate_mounts_file: Optional[str] = None

@dataclass(frozen=True)
class TFWorldToMecaMount:
    xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy_deg: Tuple[float, float, float] = (0.0, 0.0, 0.0)

@dataclass(frozen=True)
class ROSConfig:
    launch_ros: bool
    sim_robot: bool

@dataclass(frozen=True)
class AppContext:
    paths: AppPaths
    ros: ROSConfig
    recipes_yaml: Dict[str, Any]
    recipes: List[Dict[str, Any]]
    recipe_params: Dict[str, Any]
    units: str
    tf_world_to_meca_mount: TFWorldToMecaMount
    # optional, als dict belassen für in-place Updates:
    mounts_yaml: Optional[Dict[str, Any]] = None


# ---------- Loader ----------

def load_startup(startup_yaml_path: str) -> AppContext:
    if not startup_yaml_path:
        _err("startup_yaml_path ist leer.")
    startup_yaml_path = os.path.abspath(os.path.normpath(startup_yaml_path))
    if not os.path.exists(startup_yaml_path):
        _err(f"startup.yaml nicht gefunden: {startup_yaml_path}")

    su = _load_yaml(startup_yaml_path, strict=True)

    # Pflicht: paths
    p = su.get("paths") or {}
    if not isinstance(p, dict):
        _err("startup.yaml: Abschnitt 'paths' fehlt oder ist ungültig.")

    required_path_keys = ("recipe_file", "recipe_dir", "log_dir", "bringup_log")
    for k in required_path_keys:
        if k not in p:
            _err(f"startup.yaml: 'paths.{k}' fehlt.")

    base = os.path.dirname(startup_yaml_path)
    recipe_file_abs = _abspath_rel_to(base, p["recipe_file"])
    recipe_dir_abs  = _abspath_rel_to(base, p["recipe_dir"])
    log_dir_abs     = _abspath_rel_to(base, p["log_dir"])
    bringup_log_abs = _abspath_rel_to(base, p["bringup_log"])

    # Existenz: nur was wir direkt brauchen
    if not os.path.isdir(recipe_dir_abs):
        _err(f"paths.recipe_dir existiert nicht als Verzeichnis: {recipe_dir_abs}")
    if not os.path.isdir(log_dir_abs):
        _err(f"paths.log_dir existiert nicht als Verzeichnis: {log_dir_abs}")
    bl_parent = os.path.dirname(bringup_log_abs) or "."
    if not os.path.isdir(bl_parent):
        _err(f"Verzeichnis für paths.bringup_log existiert nicht: {bl_parent}")

    # Optional: weitere Pfade (keine harten Checks)
    mounts_dir_abs  = _abspath_rel_to(base, p["substrate_mounts_dir"])  if p.get("substrate_mounts_dir")  else None
    mounts_file_abs = _abspath_rel_to(base, p["substrate_mounts_file"]) if p.get("substrate_mounts_file") else None

    # Pflicht: ros (nur Flags)
    r = su.get("ros") or {}
    if not isinstance(r, dict):
        _err("startup.yaml: Abschnitt 'ros' fehlt oder ist ungültig.")
    if "launch_ros" not in r or "sim_robot" not in r:
        _err("startup.yaml: 'ros.launch_ros' und 'ros.sim_robot' müssen vorhanden sein.")
    ros_cfg = ROSConfig(bool(r["launch_ros"]), bool(r["sim_robot"]))

    # Optional: tf.world_to_meca_mount
    tf_cfg = TFWorldToMecaMount()
    tf_node = (su.get("tf") or {}).get("world_to_meca_mount") if isinstance(su.get("tf"), dict) else None
    if isinstance(tf_node, dict):
        xyz = tf_node.get("xyz") or [0.0, 0.0, 0.0]
        rpy = tf_node.get("rpy_deg") or [0.0, 0.0, 0.0]
        try:
            tf_cfg = TFWorldToMecaMount(
                xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                rpy_deg=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
            )
        except Exception:
            warnings.warn("tf.world_to_meca_mount fehlerhaft – verwende Defaults (0,0,0).")

    # recipes.yaml laden (strict)
    ry = _load_yaml(recipe_file_abs, strict=True)
    units = ry.get("units")
    if units != "mm":
        _err("recipes.yaml: 'units' muss 'mm' sein.")
    recipe_params = ry.get("recipe_params")
    recipes_list  = ry.get("recipes")
    if not isinstance(recipe_params, dict) or not recipe_params:
        _err("recipes.yaml: 'recipe_params' fehlt oder ist leer.")
    if not isinstance(recipes_list, list) or not recipes_list:
        _err("recipes.yaml: 'recipes' fehlt oder ist leer.")

    # Optional: substrate_mounts.yaml (nicht strict)
    mounts_yaml = None
    if mounts_file_abs:
        mounts_yaml = _load_yaml(mounts_file_abs, strict=False)
        if mounts_yaml and not isinstance(mounts_yaml.get("mounts"), dict):
            warnings.warn("substrate_mounts.yaml: 'mounts' fehlt/ungültig – wird ignoriert.")

    paths = AppPaths(
        recipe_file=recipe_file_abs,
        recipe_dir=recipe_dir_abs,
        log_dir=log_dir_abs,
        bringup_log=bringup_log_abs,
        substrate_mounts_dir=mounts_dir_abs,
        substrate_mounts_file=mounts_file_abs,
    )

    return AppContext(
        paths=paths,
        ros=ros_cfg,
        recipes_yaml=ry,
        recipes=recipes_list,
        recipe_params=recipe_params,
        units=units,
        tf_world_to_meca_mount=tf_cfg,
        mounts_yaml=mounts_yaml,
    )
