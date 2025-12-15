# -*- coding: utf-8 -*-
# /root/Spraycoater/src/config/startup.py
# Startup-Loader + AppContent (Frames/QoS/Topics)
from __future__ import annotations

import os
import io
import yaml
import warnings
from dataclasses import dataclass
from typing import Any, Dict, List, Optional


# ============================================================
# ===============  Basis-Helfer  =============================
# ============================================================

def _err(msg: str) -> None:
    raise ValueError(msg)


def resolve_package_uri(uri: str) -> str:
    """ROS2 package:// Pfade auflösen."""
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
    """YAML laden. strict=True → Fehler; sonst: None."""
    try:
        path = (
            resolve_package_uri(path_or_uri)
            if isinstance(path_or_uri, str) and path_or_uri.startswith("package://")
            else path_or_uri
        )
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


# ============================================================
# =================== Datenstrukturen ========================
# ============================================================

@dataclass(frozen=True)
class AppPaths:
    recipe_file: str
    recipe_dir: str
    log_dir: str
    bringup_log: str
    substrate_mounts_dir: Optional[str] = None
    substrate_mounts_file: Optional[str] = None


@dataclass(frozen=True)
class ROSConfigPaths:
    topics_file: str
    qos_file: str
    frames_file: str
    scene_file: str
    robot_file: str
    servo_file: str
    poses_file: str
    tools_file: str


@dataclass(frozen=True)
class ROSRoleConfig:
    enabled: bool


@dataclass(frozen=True)
class ROSLiveEndpoint:
    ip: str
    port: int


@dataclass(frozen=True)
class ROSLiveOmronConfig:
    mode: str  # "omron" | "emulator"
    omron: ROSLiveEndpoint
    emulator: ROSLiveEndpoint


@dataclass(frozen=True)
class ROSLiveConfig(ROSRoleConfig):
    mode: str
    omron: ROSLiveOmronConfig


@dataclass(frozen=True)
class ROSConfig:
    launch_ros: bool
    bringup_launch: str
    namespace: str               # z.B. "spraycoater" (Info, aber NICHT für Topic-Prefixing!)
    use_sim_time: bool           # global
    robot_type: str
    configs: ROSConfigPaths
    shadow: ROSRoleConfig
    live: ROSLiveConfig


@dataclass(frozen=True)
class PlcEndpoint:
    ams_net_id: Optional[str]
    ip: Optional[str]
    port: Optional[int]


@dataclass(frozen=True)
class PlcConfig:
    sim: bool
    mode: str                   # "ads" oder "umrt"
    ads: Optional[PlcEndpoint]
    umrt: Optional[PlcEndpoint]
    spec_file: str
    spec: Optional[Dict[str, Any]] = None


@dataclass(frozen=True)
class AppContext:
    paths: AppPaths
    ros: ROSConfig
    plc: PlcConfig

    recipes_yaml: Dict[str, Any]
    recipes: List[Dict[str, Any]]
    recipe_params: Dict[str, Any]
    mounts_yaml: Optional[Dict[str, Any]] = None
    content: Optional["AppContent"] = None


# ============================================================
# ==================  Loader: load_startup  ==================
# ============================================================

def load_startup(startup_yaml_path: str) -> AppContext:
    if not startup_yaml_path:
        _err("startup_yaml_path ist leer.")
    startup_yaml_path = os.path.abspath(os.path.normpath(startup_yaml_path))
    if not os.path.exists(startup_yaml_path):
        _err(f"startup.yaml nicht gefunden: {startup_yaml_path}")

    su = _load_yaml(startup_yaml_path, strict=True) or {}
    base = os.path.dirname(startup_yaml_path)

    # ----- PATHS -----
    p = su.get("paths") or {}
    if not isinstance(p, dict):
        _err("startup.yaml: Abschnitt 'paths' fehlt oder ist ungültig.")

    for key in ("recipe_file", "recipe_dir", "log_dir", "bringup_log"):
        if key not in p:
            _err(f"startup.yaml: 'paths.{key}' fehlt.")

    recipe_file_abs = _abspath_rel_to(base, p["recipe_file"])
    recipe_dir_abs  = _abspath_rel_to(base, p["recipe_dir"])
    log_dir_abs     = _abspath_rel_to(base, p["log_dir"])
    bringup_log_abs = _abspath_rel_to(base, p["bringup_log"])

    if not os.path.isdir(recipe_dir_abs):
        _err(f"paths.recipe_dir existiert nicht: {recipe_dir_abs}")
    if not os.path.isdir(log_dir_abs):
        _err(f"paths.log_dir existiert nicht: {log_dir_abs}")

    bl_parent = os.path.dirname(bringup_log_abs) or "."
    if not os.path.isdir(bl_parent):
        _err(f"Verzeichnis für bringup_log existiert nicht: {bl_parent}")

    mounts_dir_abs = _abspath_rel_to(base, p["substrate_mounts_dir"]) if p.get("substrate_mounts_dir") else None
    mounts_file_abs = _abspath_rel_to(base, p["substrate_mounts_file"]) if p.get("substrate_mounts_file") else None

    paths = AppPaths(
        recipe_file=recipe_file_abs,
        recipe_dir=recipe_dir_abs,
        log_dir=log_dir_abs,
        bringup_log=bringup_log_abs,
        substrate_mounts_dir=mounts_dir_abs,
        substrate_mounts_file=mounts_file_abs,
    )

    # ----- ROS -----
    r = su.get("ros") or {}
    if not isinstance(r, dict):
        _err("startup.yaml: Abschnitt 'ros' fehlt oder ist ungültig.")

    if "launch_ros" not in r:
        _err("startup.yaml: ros.launch_ros ist Pflichtfeld.")

    launch_ros = bool(r["launch_ros"])
    bringup_launch = str(r.get("bringup_launch", "")).strip()
    namespace = str(r.get("namespace", "")).strip()
    use_sim_time = bool(r.get("use_sim_time", False))

    if not bringup_launch:
        _err("startup.yaml: ros.bringup_launch fehlt oder ist leer.")
    if not namespace:
        _err("startup.yaml: ros.namespace fehlt oder ist leer.")

    robot_type = str(r.get("robot_type", "")).strip()
    if not robot_type:
        _err("startup.yaml: ros.robot_type fehlt oder ist leer.")

    rcfg = r.get("configs") or {}
    if not isinstance(rcfg, dict):
        _err("startup.yaml: ros.configs fehlt oder ist ungültig.")

    for key in ("topics_file", "qos_file", "frames_file", "scene_file", "robot_file", "servo_file", "poses_file", "tools_file"):
        if key not in rcfg:
            _err(f"startup.yaml: ros.configs.{key} fehlt.")

    ros_paths = ROSConfigPaths(
        topics_file=_abspath_rel_to(base, rcfg["topics_file"]),
        qos_file=_abspath_rel_to(base, rcfg["qos_file"]),
        frames_file=_abspath_rel_to(base, rcfg["frames_file"]),
        scene_file=_abspath_rel_to(base, rcfg["scene_file"]),
        robot_file=_abspath_rel_to(base, rcfg["robot_file"]),
        servo_file=_abspath_rel_to(base, rcfg["servo_file"]),
        poses_file=_abspath_rel_to(base, rcfg["poses_file"]),
        tools_file=_abspath_rel_to(base, rcfg["tools_file"]),
    )

    shadow_yaml = r.get("shadow") or {}
    if not isinstance(shadow_yaml, dict):
        _err("startup.yaml: ros.shadow fehlt oder ist ungültig.")
    shadow_cfg = ROSRoleConfig(enabled=bool(shadow_yaml.get("enabled", False)))

    live_yaml = r.get("live") or {}
    if not isinstance(live_yaml, dict):
        _err("startup.yaml: ros.live fehlt oder ist ungültig.")
    live_enabled = bool(live_yaml.get("enabled", False))

    mode = str(live_yaml.get("mode", "")).strip().lower()
    if mode not in ("omron", "emulator"):
        _err(f"startup.yaml: ros.live.mode muss 'omron' oder 'emulator' sein (ist {mode!r}).")

    omron_yaml = live_yaml.get("omron")
    if not isinstance(omron_yaml, dict):
        _err("startup.yaml: ros.live.omron fehlt oder ist ungültig.")

    def _endpoint(name: str) -> ROSLiveEndpoint:
        b = omron_yaml.get(name)
        if not isinstance(b, dict):
            _err(f"startup.yaml: ros.live.omron.{name} fehlt oder ist ungültig.")
        ip = str(b.get("ip", "")).strip()
        port = b.get("port", None)
        if not ip:
            _err(f"startup.yaml: ros.live.omron.{name}.ip fehlt/leer.")
        if port is None:
            _err(f"startup.yaml: ros.live.omron.{name}.port fehlt.")
        return ROSLiveEndpoint(ip=ip, port=int(port))

    omron_cfg = ROSLiveOmronConfig(
        mode=mode,
        omron=_endpoint("omron"),
        emulator=_endpoint("emulator"),
    )

    live_cfg = ROSLiveConfig(
        enabled=live_enabled,
        mode=mode,
        omron=omron_cfg,
    )

    ros_cfg = ROSConfig(
        launch_ros=launch_ros,
        bringup_launch=bringup_launch,
        namespace=namespace,
        use_sim_time=use_sim_time,
        robot_type=robot_type,
        configs=ros_paths,
        shadow=shadow_cfg,
        live=live_cfg,
    )

    # ----- PLC -----
    plc_yaml = su.get("plc") or {}
    if not isinstance(plc_yaml, dict):
        _err("startup.yaml: Abschnitt 'plc' fehlt oder ist ungültig.")

    plc_sim = bool(plc_yaml.get("sim", True))
    plc_mode = str(plc_yaml.get("mode", "ads")).strip().lower()
    if plc_mode not in ("ads", "umrt"):
        _err(f"startup.yaml: plc.mode muss 'ads' oder 'umrt' sein (ist {plc_mode!r}).")

    ads_cfg = plc_yaml.get("ads") or {}
    umrt_cfg = plc_yaml.get("umrt") or {}

    ads_ep = PlcEndpoint(
        ams_net_id=ads_cfg.get("ams_net_id"),
        ip=ads_cfg.get("ip"),
        port=ads_cfg.get("port"),
    ) if ads_cfg else None

    umrt_ep = PlcEndpoint(
        ams_net_id=umrt_cfg.get("ams_net_id"),
        ip=umrt_cfg.get("ip"),
        port=umrt_cfg.get("port"),
    ) if umrt_cfg else None

    spec_rel = plc_yaml.get("spec_file", "plc.yaml")
    spec_abs = _abspath_rel_to(base, spec_rel)
    plc_spec = _load_yaml(spec_abs, strict=False)

    plc_cfg = PlcConfig(
        sim=plc_sim,
        mode=plc_mode,
        ads=ads_ep,
        umrt=umrt_ep,
        spec_file=spec_abs,
        spec=plc_spec,
    )

    # ----- recipes.yaml -----
    ry = _load_yaml(recipe_file_abs, strict=True) or {}
    recipe_params = ry.get("recipe_params")
    recipes_list = ry.get("recipes")

    if not isinstance(recipe_params, dict) or not recipe_params:
        _err("recipes.yaml: recipe_params fehlt oder ist leer.")
    if not isinstance(recipes_list, list) or not recipes_list:
        _err("recipes.yaml: recipes fehlt oder ist leer.")

    # ----- substrate_mounts.yaml optional -----
    mounts_yaml = None
    if mounts_file_abs:
        mounts_yaml = _load_yaml(mounts_file_abs, strict=False)
        if mounts_yaml and not isinstance(mounts_yaml.get("mounts"), dict):
            warnings.warn("substrate_mounts.yaml ungültig → ignoriert.")
            mounts_yaml = None

    # ----- AppContent erzeugen -----
    content = AppContent(startup_yaml_path)

    return AppContext(
        paths=paths,
        ros=ros_cfg,
        plc=plc_cfg,
        recipes_yaml=ry,
        recipes=recipes_list,
        recipe_params=recipe_params,
        mounts_yaml=mounts_yaml,
        content=content,
    )


# ============================================================
# =================== AppContent (Frames/QoS/Topics) =========
# ============================================================

try:
    from rosidl_runtime_py.utilities import get_message
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
except Exception:
    QoSProfile = object
    ReliabilityPolicy = DurabilityPolicy = HistoryPolicy = object

    def get_message(_s: str):
        raise RuntimeError("rosidl_runtime_py nicht verfügbar.")


@dataclass(frozen=True)
class TopicSpec:
    id: str
    name: str
    type_str: str
    qos_key: str

    def resolve_type(self):
        if "/msg/" not in self.type_str:
            raise ValueError(f"Ungültiger Msg-Typ: {self.type_str}")
        return get_message(self.type_str)


@dataclass(frozen=True)
class PathsLite:
    substrate_mounts_dir: Optional[str] = None


class AppContent:
    """
    Frames / QoS / Topics aus startup.yaml.ros.configs.* laden.

    WICHTIG für deinen Fall:
      - topics.yaml enthält bereits "spraycoater/..."
      - daher wird HIER NICHTS geprefixt (kein root_ns)
      - Node-Namespace (shadow/live) kommt allein aus RosBridge / rclpy Node
    """

    def __init__(self, startup_yaml_path: str):
        su = _load_yaml(startup_yaml_path, strict=True) or {}
        base = os.path.dirname(os.path.abspath(startup_yaml_path))

        r = su.get("ros") or {}
        if not isinstance(r, dict):
            raise ValueError("startup.yaml: Abschnitt 'ros' fehlt oder ist ungültig.")
        cfg = r.get("configs") or {}
        if not isinstance(cfg, dict) or not cfg:
            raise ValueError("startup.yaml: ros.configs fehlt oder ist ungültig.")

        # ✅ configs sauber auflösen (package:// + relativ)
        frames_path = _abspath_rel_to(base, cfg.get("frames_file", ""))
        qos_path    = _abspath_rel_to(base, cfg.get("qos_file", ""))
        topics_path = _abspath_rel_to(base, cfg.get("topics_file", ""))

        self._frames = _load_yaml(frames_path, strict=True) or {}
        self._qos    = _load_yaml(qos_path, strict=True) or {}
        self._topics = _load_yaml(topics_path, strict=True) or {}

        # ---- Frames ----
        self._frames_map = self._frames.get("frames") or {}
        if not isinstance(self._frames_map, dict) or not self._frames_map:
            raise ValueError("frames.yaml: frames fehlt.")

        # ---- QoS ----
        self._qos_profiles = self._build_qos_profiles(self._qos)

        # ---- Topics ----
        self._topics_root = self._topics.get("topics") or {}
        if not isinstance(self._topics_root, dict):
            raise ValueError("topics.yaml: topics fehlt.")

        # ❗ Für deinen Fall: kein root_ns, kein Prefixing
        self._root_ns = ""

        # optional Mounts:
        p = su.get("paths") or {}
        mounts_dir_abs = _abspath_rel_to(base, p["substrate_mounts_dir"]) if p.get("substrate_mounts_dir") else None
        mounts_file_abs = _abspath_rel_to(base, p["substrate_mounts_file"]) if p.get("substrate_mounts_file") else None

        self.paths = PathsLite(substrate_mounts_dir=mounts_dir_abs)

        self.mounts_yaml = None
        if mounts_file_abs:
            my = _load_yaml(mounts_file_abs, strict=False)
            if my and isinstance(my.get("mounts"), dict):
                self.mounts_yaml = my

    # ---------------- Frames ----------------
    def frame(self, name: str) -> str:
        if name not in self._frames_map:
            raise KeyError(f"Frame '{name}' nicht gefunden.")
        return self._frames_map[name]

    # ---------------- QoS -------------------
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
            raise ValueError("qos.yaml: profiles fehlt.")

        out: Dict[str, QoSProfile] = {}
        for key, spec in profiles.items():
            if not isinstance(spec, dict):
                raise ValueError(f"Ungültiges QoS-Profil: {key}")

            history = self._str_to_history(spec.get("history", "KEEP_LAST"))
            depth = int(spec.get("depth", 10))
            reliability = self._str_to_reliability(spec.get("reliability", "RELIABLE"))
            durability = self._str_to_durability(spec.get("durability", "VOLATILE"))

            out[key] = QoSProfile(
                history=history,
                depth=depth,
                reliability=reliability,
                durability=durability,
            )
        return out

    def qos(self, key: str):
        if key not in self._qos_profiles:
            raise KeyError(f"QoS-Profil '{key}' fehlt.")
        return self._qos_profiles[key]

    # ---------------- Topics -----------------
    def _with_root_ns(self, name: str) -> str:
        # Für deinen Fall: NICHT prefixen, aber absolute Topics respektieren
        return name

    def topics(self, group: str, direction: str) -> List[TopicSpec]:
        if group not in self._topics_root:
            raise KeyError(f"Topic-Gruppe '{group}' fehlt.")
        block = self._topics_root[group].get(direction) or []
        if not isinstance(block, list):
            raise ValueError(f"topics[{group}][{direction}] ist kein Array.")

        out: List[TopicSpec] = []
        for e in block:
            name = str(e["name"])
            if name.startswith("/"):
                raise ValueError(
                    f"topics.yaml: Topic '{name}' darf NICHT mit '/' anfangen "
                    f"(sonst ignoriert ROS den Node-Namespace)."
                )
            out.append(
                TopicSpec(
                    id=str(e["id"]),
                    name=self._with_root_ns(name),
                    type_str=str(e["type"]),
                    qos_key=str(e.get("qos", "default")),
                )
            )
        return out

    def topic_by_id(self, group: str, direction: str, topic_id: str) -> TopicSpec:
        for t in self.topics(group, direction):
            if t.id == topic_id:
                return t
        raise KeyError(f"Topic id '{topic_id}' in {group}/{direction} nicht gefunden.")

    # Node-Helpers
    def create_publisher_from_id(self, node, group: str, topic_id: str):
        spec = self.topic_by_id(group, "publish", topic_id)
        msg_type = spec.resolve_type()
        return node.create_publisher(msg_type, spec.name, self.qos(spec.qos_key))

    def create_subscription_from_id(self, node, group: str, topic_id: str, callback):
        spec = self.topic_by_id(group, "subscribe", topic_id)
        msg_type = spec.resolve_type()
        return node.create_subscription(msg_type, spec.name, callback, self.qos(spec.qos_key))
