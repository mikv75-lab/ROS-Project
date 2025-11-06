# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional, Any, List, Dict

from PyQt6 import QtCore
from rosidl_runtime_py.utilities import get_message

from .runner import RosBridge

_LOG = logging.getLogger("ros.ui_bridge")


# ---------- Qt-Signale: inbound (ROS -> UI) + outbound (UI -> ROS) ----------

class BridgeSignals(QtCore.QObject):
    # Inbound (ROS -> UI)
    cageListChanged = QtCore.pyqtSignal(list)
    mountListChanged = QtCore.pyqtSignal(list)
    substrateListChanged = QtCore.pyqtSignal(list)

    cageCurrentChanged = QtCore.pyqtSignal(str)
    mountCurrentChanged = QtCore.pyqtSignal(str)
    substrateCurrentChanged = QtCore.pyqtSignal(str)

    # Outbound (UI -> ROS)
    setCageRequested = QtCore.pyqtSignal(str)
    setMountRequested = QtCore.pyqtSignal(str)
    setSubstrateRequested = QtCore.pyqtSignal(str)


class UIBridge:
    """
    Sehr dünne UI-Bridge:
      - Start/Stop der RosBridge (Executor + Bridge-Nodes)
      - Stellt nur Qt-Signale bereit (bridge.scene.<Signals>)
      - Mapped Set-Requests (UI) -> Publish auf */set Topics
      - Subscribed selbst auf *_list / *_current und emittiert Signale
    """
    T_STRING = "std_msgs/msg/String"

    def __init__(self, startup_yaml_path: Optional[str] = None):
        self._startup_yaml = startup_yaml_path or os.environ.get("SC_STARTUP_YAML") or ""
        self._bridge: Optional[RosBridge] = None
        self._ephemeral_subs: List[Any] = []

        # Ein einziges Signal-Objekt für Scene (Alias: bridge.scene)
        self.signals = BridgeSignals()
        self.scene = self.signals

        # Topic-Mapping (muss zu deiner topics.yaml passen)
        self._scene_topics: Dict[str, Dict[str, str]] = {
            "cage": {
                "set": "/spraycoater/scene/cage/set",
                "current": "/spraycoater/scene/cage/current",
                "list": "/spraycoater/scene/cage/list",
            },
            "mount": {
                "set": "/spraycoater/scene/mount/set",
                "current": "/spraycoater/scene/mount/current",
                "list": "/spraycoater/scene/mount/list",
            },
            "substrate": {
                "set": "/spraycoater/scene/substrate/set",
                "current": "/spraycoater/scene/substrate/current",
                "list": "/spraycoater/scene/substrate/list",
            },
        }

        # UI -> ROS: Setter-Signale verdrahten
        self._wire_setters()

    # ---------- Lifecycle ----------

    @property
    def is_connected(self) -> bool:
        return self._bridge is not None

    @property
    def node_name(self) -> str:
        if self._bridge and self._bridge.primary_node:
            try:
                return self._bridge.primary_node.get_name()  # type: ignore[attr-defined]
            except Exception:
                pass
        return "ui_bridge"

    def connect(self) -> None:
        if self._bridge is not None:
            return
        if not self._startup_yaml:
            raise RuntimeError("SC_STARTUP_YAML nicht gesetzt/übergeben.")
        self._bridge = RosBridge(self._startup_yaml)
        self._bridge.start()
        _LOG.info("UIBridge connected (node=%s)", self.node_name)

        # ROS -> UI Subscriptions (latched -> sollten direkt kommen)
        self._sub_string(self._scene_topics["cage"]["list"],      lambda m: self._emit_list("cage", m))
        self._sub_string(self._scene_topics["mount"]["list"],     lambda m: self._emit_list("mount", m))
        self._sub_string(self._scene_topics["substrate"]["list"], lambda m: self._emit_list("substrate", m))

        self._sub_string(self._scene_topics["cage"]["current"],      lambda m: self._emit_current("cage", m))
        self._sub_string(self._scene_topics["mount"]["current"],     lambda m: self._emit_current("mount", m))
        self._sub_string(self._scene_topics["substrate"]["current"], lambda m: self._emit_current("substrate", m))

        _LOG.debug("UIBridge subscriptions ready (cage/mount/substrate: list + current)")

    def disconnect(self) -> None:
        _LOG.info("UIBridge disconnect()")
        try:
            if self._bridge and self._bridge.primary_node:
                node = self._bridge.primary_node
                for sub in self._ephemeral_subs:
                    try:
                        node.destroy_subscription(sub)
                    except Exception:
                        pass
        finally:
            self._ephemeral_subs.clear()

        if self._bridge is None:
            return
        try:
            self._bridge.stop()
        finally:
            self._bridge = None

    # Alias für MainWindow.closeEvent
    def shutdown(self) -> None:
        self.disconnect()

    # ---------- UI -> ROS (Publish) ----------

    def _wire_setters(self) -> None:
        self.signals.setCageRequested.connect(lambda v: self._scene_set("cage", v))
        self.signals.setMountRequested.connect(lambda v: self._scene_set("mount", v))
        self.signals.setSubstrateRequested.connect(lambda v: self._scene_set("substrate", v))

    def _scene_set(self, kind: str, value: str) -> None:
        if self._bridge is None or self._bridge.primary_node is None:
            _LOG.error("Publish %s.set: Bridge nicht verbunden.", kind)
            return
        value = (value or "").strip()
        topic = self._scene_topics[kind]["set"]
        _LOG.info("→ set %s: %s   (topic=%s)", kind, value or "''", topic)
        self._publish_string(topic, value)

    def _publish_string(self, topic_name: str, value: str, qos_depth: int = 10) -> None:
        node = self._require_node()
        Msg = get_message(self.T_STRING)
        pub = node.create_publisher(Msg, topic_name, qos_depth)
        try:
            msg = Msg()
            msg.data = value
            pub.publish(msg)
            _LOG.debug("published %s: %r", topic_name, value)
        finally:
            node.destroy_publisher(pub)

    # ---------- ROS -> UI (Subscribe + Emit) ----------

    @staticmethod
    def _csv_to_list(s: str) -> List[str]:
        if not s:
            return []
        return [x.strip() for x in s.split(",") if x.strip()]

    def _emit_list(self, kind: str, msg) -> None:
        items = self._csv_to_list(getattr(msg, "data", ""))
        _LOG.info("[scene] %s_list (%d items)", kind, len(items))
        if kind == "cage":
            self.signals.cageListChanged.emit(items)
        elif kind == "mount":
            self.signals.mountListChanged.emit(items)
        elif kind == "substrate":
            self.signals.substrateListChanged.emit(items)

    def _emit_current(self, kind: str, msg) -> None:
        val = (getattr(msg, "data", "") or "").strip()
        _LOG.info("[scene] %s_current: %s", kind, val or "-")
        if kind == "cage":
            self.signals.cageCurrentChanged.emit(val)
        elif kind == "mount":
            self.signals.mountCurrentChanged.emit(val)
        elif kind == "substrate":
            self.signals.substrateCurrentChanged.emit(val)

    def _sub_string(self, topic_name: str, cb, qos_depth: int = 10):
        node = self._require_node()
        Msg = get_message(self.T_STRING)
        sub = node.create_subscription(Msg, topic_name, cb, qos_depth)
        self._ephemeral_subs.append(sub)
        _LOG.debug("subscribed: %s", topic_name)
        return sub

    def _require_node(self):
        if self._bridge is None or self._bridge.primary_node is None:
            raise RuntimeError("Bridge nicht verbunden.")
        return self._bridge.primary_node


def get_ui_bridge(_ctx) -> UIBridge:
    # Factory für die Startup-FSM
    return UIBridge()
