# topics_loader.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List
import yaml

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.duration import Duration

# Optional: zum dynamischen Laden von Msg-Typen aus Strings ("geometry_msgs/msg/TwistStamped")
try:
    from rosidl_runtime_py.utilities import get_message as resolve_msg_type  # noqa: F401
except Exception:  # rosidl_runtime_py ist optional
    resolve_msg_type = None  # type: ignore[misc]


@dataclass(frozen=True)
class TopicSpec:
    id: str           # stabile Kurz-ID
    name: str         # absoluter ROS-Topic-Name
    type: str         # z.B. "geometry_msgs/msg/TwistStamped"
    qos: str          # Profilname aus qos.yaml


class TopicsLoader:
    """
    Erwartete YAML-Struktur:

    # topics.yaml
    topics:
      servo:
        subscribe:
          - { id: twist_in, name: /servo/cartesian_mm, type: geometry_msgs/msg/TwistStamped, qos: default }
        publish:
          - { id: state,    name: /servo/state,        type: std_msgs/msg/String,            qos: latched }

    # qos.yaml
    profiles:
      default:        { history: KEEP_LAST, depth: 10, reliability: RELIABLE, durability: VOLATILE }
      latched:        { history: KEEP_LAST, depth: 1,  reliability: RELIABLE, durability: TRANSIENT_LOCAL, lifespan_ms: 0 }
    """

    def __init__(self, topics_yaml_path: str, qos_yaml_path: str) -> None:
        self._topics_yaml_path = topics_yaml_path
        self._qos_yaml_path = qos_yaml_path

        with open(topics_yaml_path, "r", encoding="utf-8") as f:
            root = yaml.safe_load(f) or {}

        if "topics" not in root or not isinstance(root["topics"], dict):
            raise RuntimeError("'topics' mapping missing in topics.yaml")

        self._pubs_by_id: Dict[str, Dict[str, TopicSpec]] = {}
        self._subs_by_id: Dict[str, Dict[str, TopicSpec]] = {}
        self._publish_specs: Dict[str, List[TopicSpec]] = {}
        self._subscribe_specs: Dict[str, List[TopicSpec]] = {}

        troot = root["topics"]
        for node_key, section in troot.items():
            if not isinstance(section, dict):
                continue

            # subscribe
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

            # publish
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
            raise RuntimeError("'profiles' missing in qos.yaml")

    # ---- Nur-ID-API ---------------------------------------------------------

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

    # optional für Einsicht/Debug
    def publish_specs(self, node_key: str) -> List[TopicSpec]:
        return list(self._publish_specs.get(node_key, []))

    def subscribe_specs(self, node_key: str) -> List[TopicSpec]:
        return list(self._subscribe_specs.get(node_key, []))

    # ---- intern -------------------------------------------------------------

    def _qos_profile(self, profile_name: str) -> QoSProfile:
        profiles = self._qos_root["profiles"]
        if profile_name not in profiles:
            raise RuntimeError(f"QoS profile not found: {profile_name}")
        p = profiles[profile_name] or {}

        history_s = str(p.get("history", "KEEP_LAST")).upper()
        depth = int(p.get("depth", 10))

        # Basisprofil
        qos = QoSProfile(depth=depth if history_s != "KEEP_ALL" else 1)
        qos.history = HistoryPolicy.KEEP_ALL if history_s == "KEEP_ALL" else HistoryPolicy.KEEP_LAST

        reliability_s = str(p.get("reliability", "RELIABLE")).upper()
        if reliability_s == "BEST_EFFORT":
            qos.reliability = ReliabilityPolicy.BEST_EFFORT
        else:
            qos.reliability = ReliabilityPolicy.RELIABLE

        durability_s = str(p.get("durability", "VOLATILE")).upper()
        if durability_s == "TRANSIENT_LOCAL":
            qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        else:
            qos.durability = DurabilityPolicy.VOLATILE

        lifespan_ms = int(p.get("lifespan_ms", 0))
        if lifespan_ms > 0:
            qos.lifespan = Duration(nanoseconds=lifespan_ms * 1_000_000)

        return qos

    @staticmethod
    def _must_find(
        by_dir: Dict[str, Dict[str, TopicSpec]],
        node_key: str,
        id_: str,
        dir_label: str,
    ) -> TopicSpec:
        node_map = by_dir.get(node_key)
        if not node_map:
            raise RuntimeError(f"node_key '{node_key}' not found in topics ({dir_label})")
        spec = node_map.get(id_)
        if not spec:
            raise RuntimeError(f"topic id '{id_}' not found for node '{node_key}' ({dir_label})")
        return spec
