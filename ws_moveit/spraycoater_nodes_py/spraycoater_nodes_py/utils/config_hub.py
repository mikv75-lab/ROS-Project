# -*- coding: utf-8 -*-
# spraycoater_nodes_py/utils/config_hub.py
from __future__ import annotations
import os, threading, yaml
from dataclasses import dataclass
from typing import Optional, Dict, List, Any

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

# =========================
# Prozessweiter Config-Hub
# =========================
_LOCK = threading.RLock()

# Paket/Unterordner können wahlweise via ENV überschrieben werden (vor Erstnutzung):
#   SC_CFG_PKG=spraycoater_bringup
#   SC_CFG_SUBDIR=config
_PKG: str = os.environ.get("SC_CFG_PKG", "spraycoater_bringup")
_SUBDIR: str = os.environ.get("SC_CFG_SUBDIR", "config")

_TOPICS: Optional["TopicsLoader"] = None
_FRAMES: Optional["Frames"] = None
_POSES: Optional[Dict[str, Any]] = None

def set_package(pkg: str, subdir: str = "config") -> None:
    """Optional: vor erster Nutzung anderes Paket/Unterordner setzen."""
    global _PKG, _SUBDIR
    with _LOCK:
        if any(x is not None for x in (_TOPICS, _FRAMES, _POSES)):
            raise RuntimeError("config_hub already initialized (set_package must be called before first use)")
        _PKG, _SUBDIR = pkg, subdir

def _share_dir(pkg: Optional[str] = None) -> str:
    return get_package_share_directory(pkg or _PKG)

def _cfg_root(pkg: Optional[str] = None, subdir: Optional[str] = None) -> str:
    return os.path.join(_share_dir(pkg), subdir or _SUBDIR)

def _cfg_path(*names: str) -> str:
    base  = _cfg_root()
    return os.path.join(base, *names)

def _need(path: str) -> None:
    if not os.path.exists(path):
        raise FileNotFoundError(f"Config file not found: {path}")

# =========================
# Frames (inkl. load_frames)
# =========================
class Frames:
    """
    Kleiner Resolver für Frame-Aliasse aus frames.yaml (flache Map):
      frames:
        world: "world"
        robot_mount: "robot_mount"
        tcp: "tcp"
        # ... weitere Aliasse erlaubt
    """
    def __init__(self, mapping: Dict[str, str]):
        self._m = dict(mapping or {})

    def get(self, key: str, default: Optional[str] = None) -> Optional[str]:
        return self._m.get(key, default)

    def resolve(self, name_or_alias: str) -> str:
        if not name_or_alias:
            raise ValueError("leerer Frame-Name/Alias")
        return self._m.get(name_or_alias, name_or_alias)

    def as_dict(self) -> Dict[str, str]:
        return dict(self._m)

def _validate_frames_mapping(mapping: Dict[str, str], ctx_label: str) -> None:
    if not isinstance(mapping, dict) or not mapping:
        raise ValueError(f"{ctx_label} ist leer/ungültig")
    vals = list(mapping.values())
    dups = {v for v in vals if vals.count(v) > 1}
    if dups:
        raise ValueError(f"Doppelte Frame-IDs in {ctx_label}: {sorted(dups)}")
    # Mindestmenge — weitere Frames sind ok
    for req in ("world", "robot_mount", "tcp"):
        if req not in mapping:
            raise ValueError(f"{ctx_label} fehlt Schlüssel '{req}'")

def load_frames(yaml_path: str) -> Frames:
    _need(yaml_path)
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    frames_root = data.get("frames")
    if not isinstance(frames_root, dict):
        raise ValueError(f"'frames' fehlt/ist kein Mapping in {yaml_path}")
    if any(isinstance(v, dict) for v in frames_root.values()):
        raise ValueError(f"{yaml_path} enthält gruppierte 'frames' – erwartet flache Map.")
    mapping = {str(k): str(v) for k, v in frames_root.items()}
    _validate_frames_mapping(mapping, "frames")
    return Frames(mapping)

# =========================
# Topics (TopicsLoader + QoS)
# =========================
@dataclass(frozen=True)
class TopicSpec:
    id: str           # stabile Kurz-ID
    name: str         # absoluter ROS-Topic-Name
    type: str         # z.B. "geometry_msgs/msg/TwistStamped"
    qos: str          # Profilname aus qos.yaml

class TopicsLoader:
    """
    Lädt topics.yaml + qos.yaml und stellt ID-basierte Abfragen bereit.
    (API-kompatibel zu deiner bisherigen Klasse.)
    """
    def __init__(self, topics_yaml_path: str, qos_yaml_path: str) -> None:
        self._topics_yaml_path = topics_yaml_path
        self._qos_yaml_path = qos_yaml_path

        with open(topics_yaml_path, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}
        if "topics" not in root or not isinstance(root["topics"], dict):
            raise RuntimeError(f"'topics' mapping missing in {topics_yaml_path}")

        self._pubs_by_id: Dict[str, Dict[str, TopicSpec]] = {}
        self._subs_by_id: Dict[str, Dict[str, TopicSpec]] = {}
        self._publish_specs: Dict[str, List[TopicSpec]] = {}
        self._subscribe_specs: Dict[str, List[TopicSpec]] = {}

        troot = root["topics"]
        for node_key, section in troot.items():
            if not isinstance(section, dict):
                continue

            subs = section.get("subscribe", []) or []
            for s in subs:
                spec = TopicSpec(
                    id=s.get("id", s["name"]),
                    name=s["name"],
                    type=s["type"],
                    qos=s["qos"],
                )
                self._subs_by_id.setdefault(node_key, {})[spec.id] = spec
                self._subscribe_specs.setdefault(node_key, []).append(spec)

            pubs = section.get("publish", []) or []
            for s in pubs:
                spec = TopicSpec(
                    id=s.get("id", s["name"]),
                    name=s["name"],
                    type=s["type"],
                    qos=s["qos"],
                )
                self._pubs_by_id.setdefault(node_key, {})[spec.id] = spec
                self._publish_specs.setdefault(node_key, []).append(spec)

        with open(qos_yaml_path, "r", encoding="utf-8") as f:
            self._qos_root = yaml.safe_load(f) or {}
        if "profiles" not in self._qos_root or not isinstance(self._qos_root["profiles"], dict):
            raise RuntimeError(f"'profiles' missing in {qos_yaml_path}")

    # ---- Nur-ID-API ----
    def publish_topic(self, node_key: str, id_: str) -> str:
        spec = self._must_find(self._pubs_by_id, node_key, id_, "publish")
        return spec.name

    def subscribe_topic(self, node_key: str, id_: str) -> str:
        spec = self._must_find(self._subs_by_id, node_key, id_, "subscribe")
        return spec.name

    def qos_by_id(self, direction: str, node_key: str, id_: str) -> QoSProfile:
        direction = direction.lower().strip()
        if direction == "publish":
            spec = self._must_find(self._pubs_by_id, node_key, id_, "publish")
        elif direction == "subscribe":
            spec = self._must_find(self._subs_by_id, node_key, id_, "subscribe")
        else:
            raise ValueError("direction must be 'publish' or 'subscribe'")
        return self._qos_profile(spec.qos)

    # optional
    def publish_specs(self, node_key: str) -> List[TopicSpec]:
        return list(self._publish_specs.get(node_key, []))

    def subscribe_specs(self, node_key: str) -> List[TopicSpec]:
        return list(self._subscribe_specs.get(node_key, []))

    # ---- intern ----
    def _qos_profile(self, profile_name: str) -> QoSProfile:
        profiles = self._qos_root["profiles"]
        if profile_name not in profiles:
            raise RuntimeError(f"QoS profile not found: {profile_name}")
        p = profiles[profile_name] or {}

        history_s = str(p.get("history", "KEEP_LAST")).upper()
        depth = int(p.get("depth", 10))
        qos = QoSProfile(depth=depth if history_s != "KEEP_ALL" else 1)
        qos.history = HistoryPolicy.KEEP_ALL if history_s == "KEEP_ALL" else HistoryPolicy.KEEP_LAST

        reliability_s = str(p.get("reliability", "RELIABLE")).upper()
        qos.reliability = ReliabilityPolicy.BEST_EFFORT if reliability_s == "BEST_EFFORT" else ReliabilityPolicy.RELIABLE

        durability_s = str(p.get("durability", "VOLATILE")).upper()
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL if durability_s == "TRANSIENT_LOCAL" else DurabilityPolicy.VOLATILE

        lifespan_ms = int(p.get("lifespan_ms", 0))
        if lifespan_ms > 0:
            qos.lifespan = Duration(nanoseconds=lifespan_ms * 1_000_000)
        return qos

    @staticmethod
    def _must_find(by_dir: Dict[str, Dict[str, TopicSpec]], node_key: str, id_: str, dir_label: str) -> TopicSpec:
        node_map = by_dir.get(node_key)
        if not node_map:
            raise RuntimeError(f"node_key '{node_key}' not found in topics ({dir_label})")
        spec = node_map.get(id_)
        if not spec:
            raise RuntimeError(f"topic id '{id_}' not found for node '{node_key}' ({dir_label})")
        return spec

# =========================
# Lazy-Singletons & Aliase
# =========================
def topics() -> TopicsLoader:
    """Topics/QoS aus <pkg>/config (lazy, Prozess-scope)."""
    global _TOPICS
    with _LOCK:
        if _TOPICS is None:
            ty, qy = _cfg_path("topics.yaml"), _cfg_path("qos.yaml")
            _need(ty); _need(qy)
            _TOPICS = TopicsLoader(ty, qy)
        return _TOPICS

def frames() -> Frames:
    """Frames aus <pkg>/config (lazy, Prozess-scope)."""
    global _FRAMES
    with _LOCK:
        if _FRAMES is None:
            fy = _cfg_path("frames.yaml")
            _need(fy)
            _FRAMES = load_frames(fy)
        return _FRAMES

def poses() -> Dict[str, Any]:
    """Optionale benannte Posen aus <pkg>/config/poses.yaml (falls vorhanden)."""
    global _POSES
    with _LOCK:
        if _POSES is None:
            py = _cfg_path("poses.yaml")
            if os.path.exists(py):
                with open(py, "r", encoding="utf-8") as f:
                    data = yaml.safe_load(f) or {}
                    if not isinstance(data, dict):
                        raise ValueError("poses.yaml muss ein Mapping sein")
                    _POSES = data
            else:
                _POSES = {}
        return _POSES

# ======= Komfort-Helper =======
def get_config_origin() -> str:
    """Absoluter Pfad zu <share(_PKG)>/<_SUBDIR>."""
    return _cfg_root()

def config_path(name: str) -> str:
    """Absoluter Pfad zu einer Datei in <share(_PKG)>/<_SUBDIR>."""
    p = _cfg_path(name)
    _need(p)
    return p

def load_yaml(name: str) -> Any:
    """YAML aus <share(_PKG)>/<_SUBDIR>/<name> laden."""
    p = config_path(name)
    with open(p, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}

def resolve_resource_dirs(entries: List[str]) -> List[str]:
    """
    Einträge aus robot.yaml (cage_dirs/mount_dirs/substrate_dirs) in absolute Pfade auflösen:
      - package://<pkg>/<rel>
      - absoluter Pfad
      - relativer Pfad relativ zu <share(_PKG)>
    Nicht existente Verzeichnisse werden stillschweigend übersprungen.
    """
    out: List[str] = []
    share_this = _share_dir()
    for e in entries or []:
        if not e:
            continue
        if e.startswith("package://"):
            try:
                pkg, rel = e[len("package://"):].split("/", 1)
                base = _share_dir(pkg)
                cand = os.path.join(base, rel)
            except Exception:
                continue
        elif os.path.isabs(e):
            cand = e
        else:
            cand = os.path.join(share_this, e)
        cand = os.path.normpath(cand)
        if os.path.isdir(cand):
            out.append(cand)
    return out

# Komfort-Aliase (Drop-in)
def frame(name: str) -> str:
    return frames().resolve(name)
