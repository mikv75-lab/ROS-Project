# topics_loader.py
"""
TopicsLoader for ROS 2 (Python)

- Loads topics.yaml (supports global 'namespace' and per-node sections)
- Loads qos.yaml (profiles: default, latched, sensor_data, ...)
- Resolves relative topic names against the effective namespace
- Creates publishers/subscribers with the correct QoS

Usage:
    from topics_loader import TopicsLoader
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("servo")

    tl = TopicsLoader(
        node=node,
        topics_yaml="/path/to/topics.yaml",
        qos_yaml="/path/to/qos.yaml",
        default_namespace="",   # optional fallback if not in YAML
    )

    pubs = tl.create_publishers("servo")
    subs = tl.create_subscribers("servo", {
        "servo_jog": lambda msg: node.get_logger().info(f"vx={msg.data}")
    })

    # resolve a topic by short name (respects namespace):
    resolved = tl.resolve("servo", "servo_server/delta_twist_cmds")
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Any, Callable, Optional, List
import importlib
import pathlib
import yaml

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, LivelinessPolicy
import rclpy.duration

def _resolve(ns: str, name: str) -> str:
    if name.startswith('/'):
        return name
    if not ns:
        return name
    ns = ns.rstrip('/')
    name = name.lstrip('/')
    return f"{ns}/{name}"

def _py_type_from_string(ros_type: str):
    # 'pkg/msg/Type' -> python class
    try:
        pkg, kind, cls = ros_type.split('/')
    except ValueError as e:
        raise ValueError(f"Invalid ROS type string: '{ros_type}' (expected 'pkg/msg/Type')") from e
    if kind != "msg":
        raise ValueError(f"Only 'msg' types are supported, got '{ros_type}'")
    module = importlib.import_module(f"{pkg}.msg")
    return getattr(module, cls)

# --- QoS loader ---
_STR2HIST = {
    "KEEP_LAST": HistoryPolicy.KEEP_LAST,
    "KEEP_ALL": HistoryPolicy.KEEP_ALL,
    "SYSTEM_DEFAULT": HistoryPolicy.SYSTEM_DEFAULT,
}
_STR2RELIAB = {
    "RELIABLE": ReliabilityPolicy.RELIABLE,
    "BEST_EFFORT": ReliabilityPolicy.BEST_EFFORT,
    "SYSTEM_DEFAULT": ReliabilityPolicy.SYSTEM_DEFAULT,
}
_STR2DUR = {
    "VOLATILE": DurabilityPolicy.VOLATILE,
    "TRANSIENT_LOCAL": DurabilityPolicy.TRANSIENT_LOCAL,
    "SYSTEM_DEFAULT": DurabilityPolicy.SYSTEM_DEFAULT,
}
_STR2LIVE = {
    "SYSTEM_DEFAULT": LivelinessPolicy.SYSTEM_DEFAULT,
    "AUTOMATIC": LivelinessPolicy.AUTOMATIC,
    "MANUAL_BY_TOPIC": LivelinessPolicy.MANUAL_BY_TOPIC,
}

def load_qos_profile(name: str, qos_yaml_path: str) -> QoSProfile:
    p = pathlib.Path(qos_yaml_path)
    if not p.is_file():
        raise FileNotFoundError(f"qos.yaml not found: {p}")
    data = yaml.safe_load(p.read_text()) or {}
    profiles = data.get("profiles") or {}
    if name not in profiles:
        raise KeyError(f"QoS profile not found: '{name}' in {p}")
    prof = profiles[name]

    history = _STR2HIST.get(str(prof.get("history", "KEEP_LAST")).upper(), HistoryPolicy.KEEP_LAST)
    depth = int(prof.get("depth", 10)) if history == HistoryPolicy.KEEP_LAST else 0
    reliability = _STR2RELIAB.get(str(prof.get("reliability", "RELIABLE")).upper(), ReliabilityPolicy.RELIABLE)
    durability = _STR2DUR.get(str(prof.get("durability", "VOLATILE")).upper(), DurabilityPolicy.VOLATILE)

    qos = QoSProfile(
        history=history,
        depth=depth,
        reliability=reliability,
        durability=durability,
    )

    # Optional fields
    if "deadline_ms" in prof:
        qos.deadline = rclpy.duration.Duration(milliseconds=int(prof["deadline_ms"]))
    if "lifespan_ms" in prof:
        qos.lifespan = rclpy.duration.Duration(milliseconds=int(prof["lifespan_ms"]))
    if "liveliness" in prof:
        qos.liveliness = _STR2LIVE[str(prof["liveliness"]).upper()]
    if "liveliness_lease_ms" in prof:
        qos.liveliness_lease_duration = rclpy.duration.Duration(milliseconds=int(prof["liveliness_lease_ms"]))
    if "avoid_ros_namespace_conventions" in prof:
        qos.avoid_ros_namespace_conventions = bool(prof["avoid_ros_namespace_conventions"])
    return qos

@dataclass
class TopicSpec:
    name: str
    type: str
    qos: str
    resolved: str

class TopicsLoader:
    def __init__(
        self,
        node,
        topics_yaml: str,
        qos_yaml: str,
        default_namespace: str = ""
    ) -> None:
        self._node = node
        self._topics_yaml = pathlib.Path(topics_yaml)
        self._qos_yaml = pathlib.Path(qos_yaml)
        if not self._topics_yaml.is_file():
            raise FileNotFoundError(f"topics.yaml not found: {self._topics_yaml}")
        if not self._qos_yaml.is_file():
            raise FileNotFoundError(f"qos.yaml not found: {self._qos_yaml}")

        raw = yaml.safe_load(self._topics_yaml.read_text()) or {}
        topics_root = raw.get("topics") or {}
        self._global_ns = topics_root.get("namespace", default_namespace) or ""

        self._pub_specs: Dict[str, List[TopicSpec]] = {}
        self._sub_specs: Dict[str, List[TopicSpec]] = {}
        for node_key, section in topics_root.items():
            if node_key == "namespace":
                continue
            if not isinstance(section, dict):
                continue
            node_ns = section.get("namespace", self._global_ns)
            subs = []
            for t in section.get("subscribe", []) or []:
                name = t["name"]
                subs.append(TopicSpec(
                    name=name,
                    type=t["type"],
                    qos=t["qos"],
                    resolved=_resolve(node_ns, name),
                ))
            pubs = []
            for t in section.get("publish", []) or []:
                name = t["name"]
                pubs.append(TopicSpec(
                    name=name,
                    type=t["type"],
                    qos=t["qos"],
                    resolved=_resolve(node_ns, name),
                ))
            self._sub_specs[node_key] = subs
            self._pub_specs[node_key] = pubs

    def list_publishers(self, node_key: str) -> List[TopicSpec]:
        return list(self._pub_specs.get(node_key, []))

    def list_subscribers(self, node_key: str) -> List[TopicSpec]:
        return list(self._sub_specs.get(node_key, []))

    def qos(self, name: str) -> QoSProfile:
        return load_qos_profile(name, str(self._qos_yaml))

    def resolve(self, node_key: str, topic_name: str) -> str:
        # Try to find the resolved entry by short name
        for spec in self.list_publishers(node_key) + self.list_subscribers(node_key):
            if spec.name == topic_name or spec.resolved == topic_name:
                return spec.resolved
        # Fallback to using global namespace
        return _resolve(self._global_ns, topic_name)

    def create_publishers(self, node_key: str) -> Dict[str, Any]:
        pubs = {}
        for spec in self.list_publishers(node_key):
            MsgT = _py_type_from_string(spec.type)
            qos = self.qos(spec.qos)
            pubs[spec.resolved] = self._node.create_publisher(MsgT, spec.resolved, qos)
            self._node.get_logger().info(f"[topics] publisher: {spec.resolved} ({spec.type}, qos={spec.qos})")
        return pubs

    def create_subscribers(self, node_key: str, callbacks: Dict[str, Callable]) -> Dict[str, Any]:
        subs = {}
        for spec in self.list_subscribers(node_key):
            cb = callbacks.get(spec.name) or callbacks.get(spec.resolved)
            if cb is None:
                raise KeyError(f"No callback provided for topic '{spec.name}' (resolved '{spec.resolved}')")
            MsgT = _py_type_from_string(spec.type)
            qos = self.qos(spec.qos)
            subs[spec.resolved] = self._node.create_subscription(MsgT, spec.resolved, cb, qos)
            self._node.get_logger().info(f"[topics] subscriber: {spec.resolved} ({spec.type}, qos={spec.qos})")
        return subs
