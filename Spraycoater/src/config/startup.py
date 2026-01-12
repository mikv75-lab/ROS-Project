# -*- coding: utf-8 -*-
# File: src/config/startup.py
from __future__ import annotations

import io
import os
import subprocess
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import yaml


# ============================================================
# Helpers (strict, no silent fallbacks)
# ============================================================

def _err(msg: str) -> None:
    raise ValueError(msg)


def require_env_dir(var_name: str) -> str:
    """
    Require an environment variable pointing to an existing directory.

    No fallbacks: if unset or invalid -> error.
    """
    val = os.environ.get(var_name, "").strip()
    if not val:
        _err(f"Environment variable '{var_name}' ist nicht gesetzt (Pflicht).")
    val = os.path.abspath(os.path.normpath(os.path.expanduser(val)))
    if not os.path.isdir(val):
        _err(f"Environment variable '{var_name}' zeigt nicht auf ein Verzeichnis: {val!r}")
    return val


def _ros_overlay_available() -> bool:
    """
    Best-effort check whether ROS overlay is present in this process.
    This is critical for resolving package:// URIs via ament_index_python.
    """
    if not os.environ.get("AMENT_PREFIX_PATH", "").strip():
        return False
    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
        _ = get_package_share_directory  # noqa
        return True
    except Exception:
        return False


def require_ros_overlay(reason: str) -> None:
    if not _ros_overlay_available():
        _err(
            "ROS overlay ist in diesem Prozess nicht verfügbar.\n"
            f"Grund: {reason}\n\n"
            "Bitte stelle sicher, dass du VOR dem Start der UI folgendes gesourced hast:\n"
            "  source /root/ws_moveit/install/setup.bash\n"
        )


def resolve_package_uri(uri: str) -> str:
    """Resolve ROS2 package://<pkg>/<relpath> to an absolute filesystem path."""
    if not isinstance(uri, str) or not uri.startswith("package://"):
        return uri

    require_ros_overlay(f"package:// URI muss aufgelöst werden: {uri}")

    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
    except Exception as e:
        _err(f"ament_index_python nicht verfügbar für package-URI '{uri}': {e}")

    try:
        pkg, rel = uri[len("package://") :].split("/", 1)
    except ValueError:
        _err(f"Ungültige package-URI: '{uri}' (erwartet package://<pkg>/relpath)")

    base = get_package_share_directory(pkg)
    return os.path.join(base, rel)


def resolve_path(base_dir: str, p: str) -> str:
    """
    Resolve relative/absolute/package:// paths to an absolute normalized path.

    Strict: empty -> error.
    """
    if not isinstance(p, str) or not p.strip():
        _err("Leerer Pfad übergeben.")
    p = os.path.expanduser(p.strip())

    if p.startswith("package://"):
        path = resolve_package_uri(p)
    else:
        path = p if os.path.isabs(p) else os.path.join(base_dir, p)

    return os.path.abspath(os.path.normpath(path))


def load_yaml_abs(path: str) -> Dict[str, Any]:
    """
    Load YAML from an absolute filesystem path.

    Strict: must exist; must be a non-empty mapping.
    """
    if not isinstance(path, str) or not path.strip():
        _err("YAML Pfad ist leer.")
    path = os.path.abspath(os.path.normpath(path.strip()))

    if not os.path.exists(path):
        _err(f"YAML nicht gefunden: {path}")

    with io.open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if data is None or not isinstance(data, dict) or not data:
        _err(f"YAML leer oder ungültig (kein Mapping): {path}")

    return data


def load_yaml_path(base_dir: str, path_or_uri: str) -> Dict[str, Any]:
    """
    Load YAML from relative/absolute/package://

    Strict: must exist; must be a non-empty mapping.
    """
    abs_path = resolve_path(base_dir, path_or_uri)
    return load_yaml_abs(abs_path)


def load_text_abs(path: str) -> str:
    """
    Load a text file from an absolute filesystem path.

    Strict: must exist; must be non-empty after stripping.
    """
    if not isinstance(path, str) or not path.strip():
        _err("Text Pfad ist leer.")
    path = os.path.abspath(os.path.normpath(path.strip()))
    if not os.path.exists(path):
        _err(f"Datei nicht gefunden: {path}")
    with io.open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    if not isinstance(txt, str) or not txt.strip():
        _err(f"Datei ist leer: {path}")
    return txt


def write_text_abs(path: str, txt: str) -> None:
    """
    Write a text file to an absolute filesystem path.

    Strict:
      - parent dir must exist
      - content must be non-empty
    """
    if not isinstance(path, str) or not path.strip():
        _err("write_text_abs: path ist leer.")
    path = os.path.abspath(os.path.normpath(path.strip()))
    parent = os.path.dirname(path) or "."
    if not os.path.isdir(parent):
        _err(f"write_text_abs: Verzeichnis existiert nicht: {parent}")
    if not isinstance(txt, str) or not txt.strip():
        _err(f"write_text_abs: Text ist leer für {path}")
    with io.open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(txt)


def _looks_like_urdf(xml: str) -> bool:
    x = (xml or "").strip()
    if "<robot" not in x:
        return False
    return ("<link" in x) or ("<joint" in x)


def _looks_like_srdf(xml: str) -> bool:
    x = (xml or "").strip()
    if "<robot" not in x:
        return False
    srdf_markers = ("<group", "<end_effector", "<virtual_joint", "<disable_collisions")
    return any(m in x for m in srdf_markers)


# ============================================================
# Data structures
# ============================================================

@dataclass(frozen=True)
class AppPaths:
    recipe_dir: str
    log_dir: str
    bringup_log: str
    recipe_params_file: str
    planner_catalog_file: str
    recipe_catalog_file: str


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
    mode: str
    omron: ROSLiveEndpoint
    emulator: ROSLiveEndpoint


@dataclass(frozen=True)
class ROSLiveConfig(ROSRoleConfig):
    mode: str
    omron: ROSLiveOmronConfig


@dataclass(frozen=True)
class ROSConfig:
    launch_ros: bool
    bringup_package: str
    bringup_file: str
    namespace: str
    use_sim_time: bool
    robot_type: str
    substrate_mounts_dir: str
    substrate_mounts_file: str
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
    mode: str
    ads: Optional[PlcEndpoint]
    umrt: Optional[PlcEndpoint]
    spec_file: str
    spec: Dict[str, Any]


# ============================================================
# Robot Description (URDF / SRDF)
# ============================================================

@dataclass(frozen=True)
class RobotDescriptionSource:
    urdf: Dict[str, Any]
    srdf: Dict[str, Any]


@dataclass(frozen=True)
class RobotDescription:
    urdf_xml: str
    srdf_xml: str
    urdf_path: str
    srdf_path: str
    source: RobotDescriptionSource


@dataclass
class AppContext:
    """
    Single Source of Truth (naming):
      - ctx.store  : RecipeStore
      - ctx.repo   : RecipeRepo
      - ctx.content: AppContent
    """
    paths: AppPaths
    ros: ROSConfig
    plc: PlcConfig

    recipe_params_yaml: Dict[str, Any]
    planner_catalog_yaml: Dict[str, Any]
    recipe_catalog_yaml: Dict[str, Any]
    mounts_yaml: Dict[str, Any]

    robot_description: Optional[RobotDescription]

    content: "AppContent"
    store: Any
    repo: Any


# ============================================================
# Robot description loader (URDF/SRDF via package + xacro)
# ============================================================

def load_robot_description(*, robot_yaml: Dict[str, Any], out_dir: str, robot_type: str) -> RobotDescription:
    """
    Load URDF (xacro) and SRDF from the ROS workspace using package share paths.

    Also persists URDF/SRDF to stable files in out_dir to support consumers that
    interpret provided strings as filenames (initFile pitfall).
    """
    require_ros_overlay("URDF/SRDF laden (package share + xacro).")

    if not isinstance(out_dir, str) or not out_dir.strip():
        _err("load_robot_description: out_dir ist leer.")
    out_dir = os.path.abspath(os.path.normpath(out_dir))
    if not os.path.isdir(out_dir):
        _err(f"load_robot_description: out_dir existiert nicht: {out_dir!r}")

    robot_type = str(robot_type or "").strip()
    if not robot_type:
        _err("load_robot_description: robot_type ist leer (SSoT ros.robot_type).")

    desc = robot_yaml.get("description")
    if not isinstance(desc, dict):
        _err("startup.yaml: robot.description fehlt oder ist ungültig.")

    urdf_cfg = desc.get("urdf")
    srdf_cfg = desc.get("srdf")
    if not isinstance(urdf_cfg, dict):
        _err("startup.yaml: robot.description.urdf fehlt oder ist ungültig.")
    if not isinstance(srdf_cfg, dict):
        _err("startup.yaml: robot.description.srdf fehlt oder ist ungültig.")

    try:
        from ament_index_python.packages import get_package_share_directory  # type: ignore
    except Exception as e:
        _err(f"ament_index_python nicht verfügbar (URDF/SRDF): {e}")

    # ---- URDF via xacro ----
    urdf_pkg = str(urdf_cfg.get("package", "")).strip()
    urdf_xacro_rel = str(urdf_cfg.get("xacro", "")).strip()
    urdf_args = urdf_cfg.get("args") or {}
    if not isinstance(urdf_args, dict):
        _err("startup.yaml: robot.description.urdf.args muss Mapping sein (oder leer).")

    if not urdf_pkg or not urdf_xacro_rel:
        _err("startup.yaml: robot.description.urdf.package oder xacro fehlt/leer.")

    # SSoT enforcement: override/insert robot_type from ros.robot_type
    urdf_args = dict(urdf_args)
    urdf_args["robot_type"] = robot_type

    urdf_xacro_abs = os.path.join(get_package_share_directory(urdf_pkg), urdf_xacro_rel)
    urdf_xacro_abs = os.path.abspath(os.path.normpath(urdf_xacro_abs))
    if not os.path.exists(urdf_xacro_abs):
        _err(f"URDF xacro nicht gefunden: {urdf_xacro_abs}")

    cmd = ["xacro", urdf_xacro_abs]
    for k, v in urdf_args.items():
        cmd.append(f"{str(k)}:={str(v)}")

    try:
        urdf_xml = subprocess.check_output(cmd, text=True)
    except Exception as e:
        _err(f"xacro fehlgeschlagen ({urdf_xacro_abs}): {e}")

    if not isinstance(urdf_xml, str) or "<robot" not in urdf_xml:
        _err("xacro output sieht nicht nach URDF aus (kein '<robot').")
    if not _looks_like_urdf(urdf_xml):
        _err("URDF sanity-check fehlgeschlagen: URDF enthält keine <link>/<joint> Marker (vertauscht?).")

    # ---- SRDF plain file ----
    srdf_pkg = str(srdf_cfg.get("package", "")).strip()
    srdf_file_rel = str(srdf_cfg.get("file", "")).strip()
    if not srdf_pkg or not srdf_file_rel:
        _err("startup.yaml: robot.description.srdf.package oder file fehlt/leer.")

    srdf_abs = os.path.join(get_package_share_directory(srdf_pkg), srdf_file_rel)
    srdf_abs = os.path.abspath(os.path.normpath(srdf_abs))
    srdf_xml = load_text_abs(srdf_abs)

    if not _looks_like_srdf(srdf_xml):
        _err(
            "SRDF sanity-check fehlgeschlagen: SRDF enthält keine <group>/<end_effector>/<virtual_joint> Marker "
            "(falsche Datei oder leer?)."
        )

    # ---- Persist to stable files ----
    urdf_out = os.path.join(out_dir, f"robot_description__{robot_type}.urdf")
    srdf_out = os.path.join(out_dir, f"robot_description__{robot_type}.srdf")
    write_text_abs(urdf_out, urdf_xml)
    write_text_abs(srdf_out, srdf_xml)

    return RobotDescription(
        urdf_xml=urdf_xml,
        srdf_xml=srdf_xml,
        urdf_path=urdf_out,
        srdf_path=srdf_out,
        source=RobotDescriptionSource(urdf=dict(urdf_cfg), srdf=dict(srdf_cfg)),
    )


# ============================================================
# AppContent (ROS content)
# ============================================================

try:
    from rosidl_runtime_py.utilities import get_message  # type: ignore
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy  # type: ignore
except Exception:
    get_message = None  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    ReliabilityPolicy = None  # type: ignore[assignment]
    DurabilityPolicy = None  # type: ignore[assignment]
    HistoryPolicy = None  # type: ignore[assignment]


@dataclass(frozen=True)
class TopicSpec:
    id: str
    name: str
    type_str: str
    qos_key: str

    def resolve_type(self):
        if "/msg/" not in self.type_str:
            _err(f"Ungültiger Msg-Typ: {self.type_str}")
        require_ros_overlay("TopicSpec.resolve_type benötigt ROS (rosidl_runtime_py).")
        if get_message is None:
            _err("rosidl_runtime_py.utilities.get_message ist nicht verfügbar (ROS overlay fehlt?).")
        return get_message(self.type_str)


class AppContent:
    """
    Lightweight access to:
      - frames.yaml
      - qos.yaml
      - topics.yaml
      - scene.yaml   (NEW: loaded + resolved)
      - robot.yaml   (NEW: loaded + resolved)
      - substrate_mounts.yaml (required)
      - robot_description_xml() / robot_description_paths()
    """

    def __init__(
        self,
        *,
        base_dir: str,
        ros_cfg_paths: ROSConfigPaths,
        substrate_mounts_dir: str,
        substrate_mounts_file: str,
        robot_description: Optional[RobotDescription] = None,
    ):
        require_ros_overlay("AppContent benötigt ROS (rosidl_runtime_py + rclpy QoS).")
        if QoSProfile is None or ReliabilityPolicy is None or DurabilityPolicy is None or HistoryPolicy is None:
            _err("rclpy QoS Klassen sind nicht verfügbar (ROS overlay fehlt?).")

        self._robot_description = robot_description

        # Existing (already resolved by load_yaml_path)
        self._frames = load_yaml_path(base_dir, ros_cfg_paths.frames_file)
        self._qos = load_yaml_path(base_dir, ros_cfg_paths.qos_file)
        self._topics = load_yaml_path(base_dir, ros_cfg_paths.topics_file)

        # NEW: scene.yaml + robot.yaml (resolved + loaded)
        self._scene_yaml_path = resolve_path(base_dir, ros_cfg_paths.scene_file)
        self._robot_yaml_path = resolve_path(base_dir, ros_cfg_paths.robot_file)
        self._scene = load_yaml_abs(self._scene_yaml_path)
        self._robot = load_yaml_abs(self._robot_yaml_path)

        self._frames_map = self._frames.get("frames")
        if not isinstance(self._frames_map, dict) or not self._frames_map:
            _err("frames.yaml: 'frames' fehlt oder ist leer.")

        self._topics_root = self._topics.get("topics")
        if not isinstance(self._topics_root, dict):
            _err("topics.yaml: 'topics' fehlt oder ist ungültig.")

        self._qos_profiles = self._build_qos_profiles(self._qos)

        # mounts required
        self.substrate_mounts_dir = resolve_path(base_dir, substrate_mounts_dir)
        self.mounts_yaml = load_yaml_path(base_dir, substrate_mounts_file)
        mounts = self.mounts_yaml.get("mounts")
        if not isinstance(mounts, dict) or not mounts:
            _err("substrate_mounts.yaml: 'mounts' fehlt oder ist leer.")

    # ---------------- NEW: expose resolved paths + loaded docs ----------------

    def scene_yaml_path(self) -> str:
        return str(self._scene_yaml_path or "")

    def robot_yaml_path(self) -> str:
        return str(self._robot_yaml_path or "")

    def scene_yaml(self) -> Dict[str, Any]:
        return dict(self._scene or {})

    def robot_yaml(self) -> Dict[str, Any]:
        return dict(self._robot or {})

    # ---------------- existing API ----------------

    def has_robot_description(self) -> bool:
        return self._robot_description is not None

    def robot_description_xml(self) -> Tuple[str, str]:
        if self._robot_description is None:
            _err("RobotDescription fehlt (startup.yaml: robot.description ...).")
        return (self._robot_description.urdf_xml, self._robot_description.srdf_xml)

    def robot_description_paths(self) -> Tuple[str, str]:
        if self._robot_description is None:
            _err("RobotDescription fehlt (startup.yaml: robot.description ...).")
        return (self._robot_description.urdf_path, self._robot_description.srdf_path)

    def frame(self, name: str) -> str:
        if name not in self._frames_map:
            raise KeyError(f"Frame '{name}' nicht gefunden.")
        return str(self._frames_map[name])

    @staticmethod
    def _str_to_history(s: str):
        if HistoryPolicy is None:
            _err("HistoryPolicy ist nicht verfügbar (ROS overlay fehlt?).")
        s = (s or "").upper()
        if not hasattr(HistoryPolicy, s):
            _err(f"Ungültige HistoryPolicy: {s!r}")
        return getattr(HistoryPolicy, s)

    @staticmethod
    def _str_to_reliability(s: str):
        if ReliabilityPolicy is None:
            _err("ReliabilityPolicy ist nicht verfügbar (ROS overlay fehlt?).")
        s = (s or "").upper()
        if not hasattr(ReliabilityPolicy, s):
            _err(f"Ungültige ReliabilityPolicy: {s!r}")
        return getattr(ReliabilityPolicy, s)

    @staticmethod
    def _str_to_durability(s: str):
        if DurabilityPolicy is None:
            _err("DurabilityPolicy ist nicht verfügbar (ROS overlay fehlt?).")
        s = (s or "").upper()
        if not hasattr(DurabilityPolicy, s):
            _err(f"Ungültige DurabilityPolicy: {s!r}")
        return getattr(DurabilityPolicy, s)

    def _build_qos_profiles(self, qos_yaml: Dict[str, Any]) -> Dict[str, Any]:
        if QoSProfile is None:
            _err("QoSProfile ist nicht verfügbar (ROS overlay fehlt?).")

        profiles = qos_yaml.get("profiles")
        if not isinstance(profiles, dict) or not profiles:
            _err("qos.yaml: 'profiles' fehlt oder ist leer.")

        out: Dict[str, Any] = {}
        for key, spec in profiles.items():
            if not isinstance(spec, dict):
                _err(f"Ungültiges QoS-Profil: {key!r} (kein Mapping)")

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

    def topics(self, group: str, direction: str) -> List[TopicSpec]:
        if group not in self._topics_root:
            raise KeyError(f"Topic-Gruppe '{group}' fehlt.")
        block = self._topics_root[group].get(direction)
        if not isinstance(block, list):
            _err(f"topics[{group}][{direction}] ist nicht vom Typ Array.")

        out: List[TopicSpec] = []
        for e in block:
            if not isinstance(e, dict):
                _err("topics.yaml: Topic-Eintrag ist kein Mapping.")
            name = str(e["name"])
            if name.startswith("/"):
                _err(
                    f"topics.yaml: Topic '{name}' darf NICHT mit '/' anfangen "
                    f"(sonst ignoriert ROS den Node-Namespace)."
                )
            out.append(
                TopicSpec(
                    id=str(e["id"]),
                    name=name,
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

    def create_publisher_from_id(self, node, group: str, topic_id: str):
        spec = self.topic_by_id(group, "publish", topic_id)
        msg_type = spec.resolve_type()
        return node.create_publisher(msg_type, spec.name, self.qos(spec.qos_key))

    def create_subscription_from_id(self, node, group: str, topic_id: str, callback):
        spec = self.topic_by_id(group, "subscribe", topic_id)
        msg_type = spec.resolve_type()
        return node.create_subscription(msg_type, spec.name, callback, self.qos(spec.qos_key))


# ============================================================
# Loader (strict, no legacy)
# ============================================================

def load_startup(startup_yaml_path: str) -> AppContext:
    if not isinstance(startup_yaml_path, str) or not startup_yaml_path.strip():
        _err("startup_yaml_path ist leer.")
    startup_yaml_path = os.path.abspath(os.path.normpath(startup_yaml_path.strip()))
    if not os.path.exists(startup_yaml_path):
        _err(f"startup.yaml nicht gefunden: {startup_yaml_path}")

    base = require_env_dir("SC_PROJECT_ROOT")
    su = load_yaml_abs(startup_yaml_path)

    # ---------------- PATHS ----------------
    p = su.get("paths")
    if not isinstance(p, dict):
        _err("startup.yaml: Abschnitt 'paths' fehlt oder ist ungültig.")

    required_paths = (
        "recipe_dir",
        "log_dir",
        "bringup_log",
        "recipe_params_file",
        "planner_catalog_file",
        "recipe_catalog_file",
    )
    for key in required_paths:
        if key not in p:
            _err(f"startup.yaml: 'paths.{key}' fehlt.")

    recipe_dir_abs = resolve_path(base, p["recipe_dir"])
    log_dir_abs = resolve_path(base, p["log_dir"])
    bringup_log_abs = resolve_path(base, p["bringup_log"])

    recipe_params_abs = resolve_path(base, p["recipe_params_file"])
    planner_catalog_abs = resolve_path(base, p["planner_catalog_file"])
    recipe_catalog_abs = resolve_path(base, p["recipe_catalog_file"])

    if not os.path.isdir(recipe_dir_abs):
        _err(f"paths.recipe_dir existiert nicht: {recipe_dir_abs}")
    if not os.path.isdir(log_dir_abs):
        _err(f"paths.log_dir existiert nicht: {log_dir_abs}")

    bl_parent = os.path.dirname(bringup_log_abs) or "."
    if not os.path.isdir(bl_parent):
        _err(f"Verzeichnis für bringup_log existiert nicht: {bl_parent}")

    paths = AppPaths(
        recipe_dir=recipe_dir_abs,
        log_dir=log_dir_abs,
        bringup_log=bringup_log_abs,
        recipe_params_file=recipe_params_abs,
        planner_catalog_file=planner_catalog_abs,
        recipe_catalog_file=recipe_catalog_abs,
    )

    # ---------------- ROS ----------------
    r = su.get("ros")
    if not isinstance(r, dict):
        _err("startup.yaml: Abschnitt 'ros' fehlt oder ist ungültig.")

    if "launch_ros" not in r:
        _err("startup.yaml: ros.launch_ros ist Pflichtfeld.")
    launch_ros = bool(r["launch_ros"])

    namespace = str(r.get("namespace", "")).strip()
    if not namespace:
        _err("startup.yaml: ros.namespace fehlt oder ist leer.")

    use_sim_time = bool(r.get("use_sim_time", False))

    robot_type = str(r.get("robot_type", "")).strip()
    if not robot_type:
        _err("startup.yaml: ros.robot_type fehlt oder ist leer.")

    bringup_pkg = str(r.get("bringup_package", "")).strip()
    bringup_file = str(r.get("bringup_file", "")).strip()
    if launch_ros:
        if not bringup_pkg:
            _err("startup.yaml: ros.bringup_package fehlt oder ist leer.")
        if not bringup_file:
            _err("startup.yaml: ros.bringup_file fehlt oder ist leer.")
        require_ros_overlay("ros.launch_ros=true (Bringup/Bridges benötigen ROS overlay).")

    if "substrate_mounts_file" not in r:
        _err("startup.yaml: ros.substrate_mounts_file fehlt.")
    if "substrate_mounts_dir" not in r:
        _err("startup.yaml: ros.substrate_mounts_dir fehlt.")

    substrate_mounts_file = str(r["substrate_mounts_file"]).strip()
    substrate_mounts_dir = str(r["substrate_mounts_dir"]).strip()

    rcfg = r.get("configs")
    if not isinstance(rcfg, dict):
        _err("startup.yaml: ros.configs fehlt oder ist ungültig.")

    required_ros_cfg = (
        "topics_file",
        "qos_file",
        "frames_file",
        "scene_file",   # REQUIRED (now used by AppContent)
        "robot_file",   # REQUIRED (now used by AppContent)
        "servo_file",
        "poses_file",
        "tools_file",
    )
    for key in required_ros_cfg:
        if key not in rcfg:
            _err(f"startup.yaml: ros.configs.{key} fehlt.")

    ros_paths = ROSConfigPaths(
        topics_file=str(rcfg["topics_file"]),
        qos_file=str(rcfg["qos_file"]),
        frames_file=str(rcfg["frames_file"]),
        scene_file=str(rcfg["scene_file"]),
        robot_file=str(rcfg["robot_file"]),
        servo_file=str(rcfg["servo_file"]),
        poses_file=str(rcfg["poses_file"]),
        tools_file=str(rcfg["tools_file"]),
    )

    shadow_yaml = r.get("shadow")
    if not isinstance(shadow_yaml, dict):
        _err("startup.yaml: ros.shadow fehlt oder ist ungültig.")
    shadow_cfg = ROSRoleConfig(enabled=bool(shadow_yaml.get("enabled", False)))

    live_yaml = r.get("live")
    if not isinstance(live_yaml, dict):
        _err("startup.yaml: ros.live fehlt oder ist ungültig.")
    live_enabled = bool(live_yaml.get("enabled", False))

    mode = str(live_yaml.get("mode", "")).strip().lower()
    if live_enabled:
        if mode not in ("omron", "emulator"):
            _err(f"startup.yaml: ros.live.mode muss 'omron' oder 'emulator' sein (ist {mode!r}).")
    else:
        if mode not in ("omron", "emulator"):
            mode = "omron"

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
        bringup_package=bringup_pkg,
        bringup_file=bringup_file,
        namespace=namespace,
        use_sim_time=use_sim_time,
        robot_type=robot_type,
        substrate_mounts_dir=substrate_mounts_dir,
        substrate_mounts_file=substrate_mounts_file,
        configs=ros_paths,
        shadow=shadow_cfg,
        live=live_cfg,
    )

    # ---------------- ROBOT (URDF/SRDF) ----------------
    robot_description: Optional[RobotDescription] = None
    robot_yaml = su.get("robot")
    if robot_yaml is not None:
        if not isinstance(robot_yaml, dict):
            _err("startup.yaml: Abschnitt 'robot' ist ungültig (muss Mapping sein).")
        robot_description = load_robot_description(
            robot_yaml=robot_yaml,
            out_dir=paths.log_dir,
            robot_type=ros_cfg.robot_type,
        )

    # ---------------- PLC ----------------
    plc_yaml = su.get("plc")
    if not isinstance(plc_yaml, dict):
        _err("startup.yaml: Abschnitt 'plc' fehlt oder ist ungültig.")

    plc_sim = bool(plc_yaml.get("sim", True))
    plc_mode = str(plc_yaml.get("mode", "ads")).strip().lower()
    if plc_mode not in ("ads", "umrt"):
        _err(f"startup.yaml: plc.mode muss 'ads' oder 'umrt' sein (ist {plc_mode!r}).")

    ads_cfg = plc_yaml.get("ads") or {}
    umrt_cfg = plc_yaml.get("umrt") or {}

    ads_ep = (
        PlcEndpoint(
            ams_net_id=ads_cfg.get("ams_net_id"),
            ip=ads_cfg.get("ip"),
            port=ads_cfg.get("port"),
        )
        if isinstance(ads_cfg, dict) and ads_cfg
        else None
    )
    umrt_ep = (
        PlcEndpoint(
            ams_net_id=umrt_cfg.get("ams_net_id"),
            ip=umrt_cfg.get("ip"),
            port=umrt_cfg.get("port"),
        )
        if isinstance(umrt_cfg, dict) and umrt_cfg
        else None
    )

    if "spec_file" not in plc_yaml:
        _err("startup.yaml: plc.spec_file fehlt.")
    spec_abs = resolve_path(base, str(plc_yaml["spec_file"]))
    plc_spec = load_yaml_abs(spec_abs)

    plc_cfg = PlcConfig(
        sim=plc_sim,
        mode=plc_mode,
        ads=ads_ep,
        umrt=umrt_ep,
        spec_file=spec_abs,
        spec=plc_spec,
    )

    # ---------------- Recipe YAMLs ----------------
    rp_y = load_yaml_abs(paths.recipe_params_file)
    pc_y = load_yaml_abs(paths.planner_catalog_file)
    rc_y = load_yaml_abs(paths.recipe_catalog_file)

    recipe_params = rp_y.get("recipe_params")
    if not isinstance(recipe_params, dict) or not recipe_params:
        _err("recipe_params.yaml: recipe_params fehlt oder ist leer.")

    planner_catalog = pc_y.get("planner_catalog")
    if not isinstance(planner_catalog, dict) or not planner_catalog:
        _err("planner_catalog.yaml: planner_catalog fehlt oder ist leer.")

    recipes_list = rc_y.get("recipes")
    if not isinstance(recipes_list, list) or not recipes_list:
        _err("recipe_catalog.yaml: recipes fehlt oder ist leer.")

    mounts_yaml = load_yaml_path(base, substrate_mounts_file)
    mounts = mounts_yaml.get("mounts")
    if not isinstance(mounts, dict) or not mounts:
        _err("substrate_mounts.yaml: mounts fehlt oder ist leer.")

    # NOTE: AppContent now loads scene.yaml + robot.yaml too (resolved from package://)
    content = AppContent(
        base_dir=base,
        ros_cfg_paths=ros_paths,
        substrate_mounts_dir=substrate_mounts_dir,
        substrate_mounts_file=substrate_mounts_file,
        robot_description=robot_description,
    )

    from model.recipe.recipe_store import RecipeStore, RecipeStorePaths
    from model.recipe.recipe_bundle import RecipeBundle
    from model.recipe.recipe_repo import RecipeRepo

    store_paths = RecipeStorePaths(
        recipe_params_file=paths.recipe_params_file,
        planner_catalog_file=paths.planner_catalog_file,
        recipe_catalog_file=paths.recipe_catalog_file,
    )
    store = RecipeStore(
        paths=store_paths,
        recipe_params_yaml=rp_y,
        planner_catalog_yaml=pc_y,
        recipe_catalog_yaml=rc_y,
    )

    repo = RecipeRepo(bundle=RecipeBundle(recipes_root_dir=paths.recipe_dir))

    return AppContext(
        paths=paths,
        ros=ros_cfg,
        plc=plc_cfg,
        recipe_params_yaml=rp_y,
        planner_catalog_yaml=pc_y,
        recipe_catalog_yaml=rc_y,
        mounts_yaml=mounts_yaml,
        robot_description=robot_description,
        content=content,
        store=store,
        repo=repo,
    )
