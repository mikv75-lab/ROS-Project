# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, Type, List

from PyQt6 import QtCore

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import String as MsgString

from config.startup import AppContent
from .base_bridge import BaseBridge


class SceneSignals(QtCore.QObject):
    """
    Qt-Signale für Scene-Widget.

    Node -> UI (Strings):
      - cage_list, mount_list, substrate_list   (comma-separated)
      - cage_current, mount_current, substrate_current

    UI -> Node:
      - setCageRequested(name)
      - setMountRequested(name)
      - setSubstrateRequested(name)
    """

    cageListChanged = QtCore.pyqtSignal(list)
    mountListChanged = QtCore.pyqtSignal(list)
    substrateListChanged = QtCore.pyqtSignal(list)

    cageCurrentChanged = QtCore.pyqtSignal(str)
    mountCurrentChanged = QtCore.pyqtSignal(str)
    substrateCurrentChanged = QtCore.pyqtSignal(str)

    setCageRequested = QtCore.pyqtSignal(str)
    setMountRequested = QtCore.pyqtSignal(str)
    setSubstrateRequested = QtCore.pyqtSignal(str)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        # Bridge macht das (weil Cache dort liegt)
        return


class SceneBridge(BaseBridge):
    GROUP = "scene"

    def __init__(self, content: AppContent, *, namespace: str = ""):
        self.signals = SceneSignals()

        # UI -> Node pubs (scene/subscribe)
        self._ui_to_node_pubs: Dict[str, Any] = {}

        # Node -> UI subs (scene/publish)
        self._node_to_ui_subs: Dict[str, Any] = {}

        # Cache
        self._cage_list: List[str] = []
        self._mount_list: List[str] = []
        self._substrate_list: List[str] = []
        self._cage_current: str = ""
        self._mount_current: str = ""
        self._substrate_current: str = ""

        super().__init__("scene_bridge", content, namespace=namespace)

        s = self.signals
        s.setCageRequested.connect(self.set_cage)
        s.setMountRequested.connect(self.set_mount)
        s.setSubstrateRequested.connect(self.set_substrate)

        # ---- UI -> Node publishers (IDs wie in topics.yaml: scene/subscribe) ----
        self._ensure_pub("cage_set", MsgString)
        self._ensure_pub("mount_set", MsgString)
        self._ensure_pub("substrate_set", MsgString)

        # ---- Node -> UI subscriptions (IDs wie in topics.yaml: scene/publish) ----
        self._ensure_sub("cage_list", MsgString, self._on_cage_list)
        self._ensure_sub("mount_list", MsgString, self._on_mount_list)
        self._ensure_sub("substrate_list", MsgString, self._on_substrate_list)

        self._ensure_sub("cage_current", MsgString, self._on_cage_current)
        self._ensure_sub("mount_current", MsgString, self._on_mount_current)
        self._ensure_sub("substrate_current", MsgString, self._on_substrate_current)

        self.get_logger().info(
            f"[scene] SceneBridge bereit (ns='{self.namespace or '/'}')"
        )

    # ─────────────────────────────────────────────────────────────
    # YAML helpers (AppContent)
    # ─────────────────────────────────────────────────────────────

    def _resolve_ns_topic(self, name: str) -> str:
        name = (name or "").strip()
        if not name:
            return name
        if name.startswith("/"):
            return name
        ns = (self.namespace or "").strip().strip("/")
        if not ns:
            return "/" + name
        return f"/{ns}/{name}".replace("//", "/")

    def _qos_from_spec(self, spec: Any) -> QoSProfile:
        qos_id = getattr(spec, "qos", None) or getattr(spec, "qos_id", None) or "default"
        qos_id = str(qos_id).lower()

        if qos_id == "latched":
            return QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

        if qos_id == "sensor_data":
            # minimal sensor-data profile (BEST_EFFORT) – reicht für Strings auch
            return QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )

        return QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _spec(self, direction: str, topic_id: str) -> Any:
        return self._content.topic_by_id(self.GROUP, direction, topic_id)

    def _topic(self, direction: str, topic_id: str) -> str:
        spec = self._spec(direction, topic_id)
        raw = getattr(spec, "name", None) or getattr(spec, "topic", None) or ""
        return self._resolve_ns_topic(str(raw))

    def _ensure_pub(self, topic_id: str, msg_type: Type) -> None:
        if topic_id in self._ui_to_node_pubs:
            return
        spec = self._spec("subscribe", topic_id)
        topic = self._topic("subscribe", topic_id)
        qos = self._qos_from_spec(spec)
        self._ui_to_node_pubs[topic_id] = self.create_publisher(msg_type, topic, qos)
        self.get_logger().info(f"[scene] PUB ui->node id={topic_id} topic={topic}")

    def _ensure_sub(self, topic_id: str, msg_type: Type, cb) -> None:
        if topic_id in self._node_to_ui_subs:
            return
        spec = self._spec("publish", topic_id)
        topic = self._topic("publish", topic_id)
        qos = self._qos_from_spec(spec)
        self._node_to_ui_subs[topic_id] = self.create_subscription(msg_type, topic, cb, qos)
        self.get_logger().info(f"[scene] SUB node->ui id={topic_id} topic={topic}")

    def _pub(self, topic_id: str):
        p = self._ui_to_node_pubs.get(topic_id)
        if p is None:
            raise KeyError(f"[scene] Publisher '{topic_id}' fehlt")
        return p

    # ─────────────────────────────────────────────────────────────
    # Public API
    # ─────────────────────────────────────────────────────────────

    def set_cage(self, name: str) -> None:
        self._pub("cage_set").publish(MsgString(data=(name or "").strip()))
        self.get_logger().debug(f"[scene_bridge] PUB cage_set -> {name!r}")

    def set_mount(self, name: str) -> None:
        self._pub("mount_set").publish(MsgString(data=(name or "").strip()))
        self.get_logger().debug(f"[scene_bridge] PUB mount_set -> {name!r}")

    def set_substrate(self, name: str) -> None:
        self._pub("substrate_set").publish(MsgString(data=(name or "").strip()))
        self.get_logger().debug(f"[scene_bridge] PUB substrate_set -> {name!r}")

    def reemit_cached(self) -> None:
        s = self.signals
        s.cageListChanged.emit(list(self._cage_list))
        s.mountListChanged.emit(list(self._mount_list))
        s.substrateListChanged.emit(list(self._substrate_list))
        s.cageCurrentChanged.emit(self._cage_current)
        s.mountCurrentChanged.emit(self._mount_current)
        s.substrateCurrentChanged.emit(self._substrate_current)

    # ─────────────────────────────────────────────────────────────
    # Node -> UI callbacks
    # ─────────────────────────────────────────────────────────────

    @staticmethod
    def _parse_list(csv: str) -> List[str]:
        csv = (csv or "").strip()
        if not csv:
            return []
        return [x.strip() for x in csv.split(",") if x.strip()]

    def _on_cage_list(self, msg: MsgString) -> None:
        self._cage_list = self._parse_list(msg.data)
        self.signals.cageListChanged.emit(list(self._cage_list))

    def _on_mount_list(self, msg: MsgString) -> None:
        self._mount_list = self._parse_list(msg.data)
        self.signals.mountListChanged.emit(list(self._mount_list))

    def _on_substrate_list(self, msg: MsgString) -> None:
        self._substrate_list = self._parse_list(msg.data)
        self.signals.substrateListChanged.emit(list(self._substrate_list))

    def _on_cage_current(self, msg: MsgString) -> None:
        self._cage_current = (msg.data or "").strip()
        self.signals.cageCurrentChanged.emit(self._cage_current)

    def _on_mount_current(self, msg: MsgString) -> None:
        self._mount_current = (msg.data or "").strip()
        self.signals.mountCurrentChanged.emit(self._mount_current)

    def _on_substrate_current(self, msg: MsgString) -> None:
        self._substrate_current = (msg.data or "").strip()
        self.signals.substrateCurrentChanged.emit(self._substrate_current)
