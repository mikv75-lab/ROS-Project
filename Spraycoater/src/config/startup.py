# -*- coding: utf-8 -*-
# /root/Spraycoater/src/config/startup.py
# Minimaler Startup-Loader (deine Basis) + AppContent (Frames/QoS/Topics)
from __future__ import annotations
import os
import io
import yaml
import warnings
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

# ============================================================
# ===============  Deine bestehende Basis  ===================
# ============================================================

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

# ---------- Datenstrukturen (bestehend) ----------

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
    mounts_yaml: Optional[Dict[str, Any]] = None

# ---------- Loader (bestehend) ----------

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

    # recipes.yaml laden (strict)
    ry = _load_yaml(recipe_file_abs, strict=True)
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
        mounts_yaml=mounts_yaml,
    )

# ============================================================
# =======  Erweiterung: Frames / QoS / Topics Manager  =======
# ============================================================

# ROS-Abhängigkeiten erst hier (damit das Modul auch ohne ROS importierbar bleibt)
try:
    from rosidl_runtime_py.utilities import get_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
except Exception:
    # Platzhalter, falls dieses Modul in einer Nicht-ROS-Umgebung importiert wird
    QoSProfile = object  # type: ignore
    ReliabilityPolicy = DurabilityPolicy = HistoryPolicy = object  # type: ignore
    def get_message(_s: str):  # type: ignore
        raise RuntimeError("rosidl_runtime_py nicht verfügbar – AppContent resolve_type nicht nutzbar.")

@dataclass(frozen=True)
class TopicSpec:
    id: str
    name: str
    type_str: str
    qos_key: str

    def resolve_type(self):
        """Erwartet 'pkg/msg/MsgName', z.B. 'geometry_msgs/msg/PoseStamped'."""
        if "/msg/" not in self.type_str:
            raise ValueError(f"Ungültiger ROS Msg-Typ: {self.type_str!r}")
        # WICHTIG: get_message erwartet genau 'pkg/msg/Type'
        return get_message(self.type_str)

@dataclass(frozen=True)
class PathsLite:
    """Kleine, schreibgeschützte Hülle – nur was die Bridge braucht."""
    substrate_mounts_dir: Optional[str] = None

class AppContent:
    """
    Lädt und verwaltet frames.yaml, qos.yaml, topics.yaml anhand startup.yaml (configs.*).
    Zusätzlich werden die (optionalen) Mount-Pfade aus startup.yaml gelesen, damit
    Bridge-Nodes (z. B. Scene) Verfügbarkeitslisten bilden können, ohne AppContext zu kennen.

    API:
      frame(name) -> str
      qos(key) -> QoSProfile
      topics(group, dir) -> list[TopicSpec]
      topic_by_id(group, dir, id) -> TopicSpec
      create_publisher_from_id(node, group, topic_id)
      create_subscription_from_id(node, group, topic_id, callback)

      # kompatible Zusatzinfos:
      paths: PathsLite (nur substrate_mounts_dir)
      mounts_yaml: Optional[Dict]
    """

    def __init__(self, startup_yaml_path: str):
        su = _load_yaml(startup_yaml_path, strict=True) or {}
        cfg = su.get("configs") or {}
        if not isinstance(cfg, dict) or not cfg:
            raise ValueError("startup.yaml: Abschnitt 'configs' fehlt oder ist leer (frames/qos/topics).")

        base = os.path.dirname(os.path.abspath(startup_yaml_path))

        # Dateien laden
        self._frames = _load_yaml(cfg.get("frames_file", ""), strict=True) or {}
        self._qos    = _load_yaml(cfg.get("qos_file", ""),    strict=True) or {}
        self._topics = _load_yaml(cfg.get("topics_file", ""), strict=True) or {}

        # Frames
        self._frames_map: Dict[str, str] = (self._frames.get("frames") or {})
        if not isinstance(self._frames_map, dict) or not self._frames_map:
            raise ValueError("frames.yaml: 'frames' fehlt oder ist ungültig.")

        # QoS
        self._qos_profiles: Dict[str, QoSProfile] = self._build_qos_profiles(self._qos)

        # Topics
        self._topics_root: Dict[str, Any] = self._topics.get("topics") or {}
        if not isinstance(self._topics_root, dict) or not self._topics_root:
            raise ValueError("topics.yaml: 'topics' fehlt oder ist ungültig.")

        # Optional: Namespace-Präfix, falls du später relative Namen erlaubst
        self._root_ns: str = (self._topics.get("root_ns") or "").rstrip("/")

        # ---- kompatible Mount-Infos aus startup.yaml (optional) ----
        p = su.get("paths") or {}
        mounts_dir_abs  = _abspath_rel_to(base, p["substrate_mounts_dir"])  if p.get("substrate_mounts_dir")  else None
        mounts_file_abs = _abspath_rel_to(base, p["substrate_mounts_file"]) if p.get("substrate_mounts_file") else None

        self.paths = PathsLite(substrate_mounts_dir=mounts_dir_abs)

        self.mounts_yaml: Optional[Dict[str, Any]] = None
        if mounts_file_abs:
            my = _load_yaml(mounts_file_abs, strict=False)
            if my and not isinstance(my.get("mounts"), dict):
                warnings.warn("substrate_mounts.yaml: 'mounts' fehlt/ungültig – wird ignoriert.")
            else:
                self.mounts_yaml = my

    # ----- Frames -----
    def frame(self, name: str) -> str:
        if name not in self._frames_map:
            raise KeyError(f"Frame '{name}' nicht in frames.yaml gefunden")
        return self._frames_map[name]

    # ----- QoS -----
    @staticmethod
    def _str_to_history(s: str):
        s = (s or "").upper()
        return getattr(HistoryPolicy, s, HistoryPolicy.KEEP_LAST)

    @staticmethod
    def _str_to_reliability(s: str):
        s = (s or "").upper()
        return getattr(ReliabilityPolicy, s, ReliabilityPolicy.RELIABLE)

    @staticmethod
    def _str_to_durability(s: str):
        s = (s or "").upper()
        return getattr(DurabilityPolicy, s, DurabilityPolicy.VOLATILE)

    def _build_qos_profiles(self, qos_yaml: Dict[str, Any]) -> Dict[str, QoSProfile]:
        profiles = qos_yaml.get("profiles") or {}
        if not isinstance(profiles, dict) or not profiles:
            raise ValueError("qos.yaml: 'profiles' fehlt oder ist leer.")

        out: Dict[str, QoSProfile] = {}
        for key, spec in profiles.items():
            if not isinstance(spec, dict):
                raise ValueError(f"qos.yaml: Profil '{key}' ist kein Mapping.")
            history = self._str_to_history(spec.get("history", "KEEP_LAST"))
            depth = int(spec.get("depth", 10))
            reliability = self._str_to_reliability(spec.get("reliability", "RELIABLE"))
            durability = self._str_to_durability(spec.get("durability", "VOLATILE"))

            qos = QoSProfile(history=history, depth=depth,
                             reliability=reliability, durability=durability)

            # Optionalfelder (deadline, lifespan, liveliness, lease) könntest du später ergänzen.
            out[key] = qos
        return out

    def qos(self, key: str):
        if key not in self._qos_profiles:
            raise KeyError(f"QoS-Profil '{key}' nicht gefunden")
        return self._qos_profiles[key]

    # ----- Topics -----
    def _with_root_ns(self, name: str) -> str:
        if name.startswith("/") or not self._root_ns:
            return name
        return f"{self._root_ns}/{name.lstrip('/')}"

    def topics(self, group: str, direction: str) -> List[TopicSpec]:
        if group not in self._topics_root:
            raise KeyError(f"Topic-Gruppe '{group}' nicht gefunden")
        block = self._topics_root[group].get(direction) or []
        if not isinstance(block, list):
            raise ValueError(f"topics[{group}][{direction}] ist kein Array")
        out: List[TopicSpec] = []
        for e in block:
            if not isinstance(e, dict):
                raise ValueError(f"Ungültiger Topic-Eintrag in topics[{group}][{direction}]: {e!r}")
            try:
                out.append(TopicSpec(
                    id=str(e["id"]),
                    name=self._with_root_ns(str(e["name"])),
                    type_str=str(e["type"]),
                    qos_key=str(e.get("qos", "default")),
                ))
            except Exception as ex:
                raise ValueError(f"Ungültiger Topic-Eintrag (Pflichtfelder id/name/type): {e!r} ({ex})")
        return out

    def topic_by_id(self, group: str, direction: str, topic_id: str) -> TopicSpec:
        for t in self.topics(group, direction):
            if t.id == topic_id:
                return t
        raise KeyError(f"Topic id '{topic_id}' nicht gefunden in topics[{group}][{direction}]")

    # ----- Node-Helpers -----
    def create_publisher_from_id(self, node, group: str, topic_id: str):
        spec = self.topic_by_id(group, "publish", topic_id)
        msg_type = spec.resolve_type()
        return node.create_publisher(msg_type, spec.name, self.qos(spec.qos_key))

    def create_subscription_from_id(self, node, group: str, topic_id: str, callback):
        spec = self.topic_by_id(group, "subscribe", topic_id)
        msg_type = spec.resolve_type()
        return node.create_subscription(msg_type, spec.name, callback, self.qos(spec.qos_key))
