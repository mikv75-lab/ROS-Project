# -*- coding: utf-8 -*-
# src/ros/bridge/scene_bridge.py
from __future__ import annotations

from typing import Optional, List

from PyQt6 import QtCore
from std_msgs.msg import String as MsgString

from config.startup import AppContent
from .base_bridge import BaseBridge


class SceneSignals(QtCore.QObject):
    """
    Qt-Signalträger für die Scene-Bridge.

    Inbound (ROS -> UI):
      - cageListChanged(list[str])
      - mountListChanged(list[str])
      - substrateListChanged(list[str])
      - cageCurrentChanged(str)
      - mountCurrentChanged(str)
      - substrateCurrentChanged(str)

    Outbound (UI -> ROS):
      - setCageRequested(str)
      - setMountRequested(str)
      - setSubstrateRequested(str)

    Cache:
      - last_* Felder werden bei jedem ROS-Update gesetzt
      - reemit_cached() sendet den zuletzt bekannten Stand erneut
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

        self.last_cage_list: List[str] = []
        self.last_mount_list: List[str] = []
        self.last_substrate_list: List[str] = []

        self.last_cage_current: str = ""
        self.last_mount_current: str = ""
        self.last_substrate_current: str = ""

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        self.cageListChanged.emit(list(self.last_cage_list))
        self.mountListChanged.emit(list(self.last_mount_list))
        self.substrateListChanged.emit(list(self.last_substrate_list))

        self.cageCurrentChanged.emit(self.last_cage_current)
        self.mountCurrentChanged.emit(self.last_mount_current)
        self.substrateCurrentChanged.emit(self.last_substrate_current)


class SceneBridge(BaseBridge):
    """
    UI-Bridge für 'scene' (im PosesBridge-Stil).

    Subscribt (ROS -> UI), aus topics.yaml: scene/publish:
      - cage_list, mount_list, substrate_list   (std_msgs/String, CSV)
      - cage_current, mount_current, substrate_current (std_msgs/String)

    Publisht (UI -> ROS), aus topics.yaml: scene/subscribe:
      - cage_set, mount_set, substrate_set (std_msgs/String)

    Wichtig:
      - QoS kommt aus spec.qos_profile (latched muss hier TRANSIENT_LOCAL sein!)
      - Wir erstellen die Subscriptions explizit, damit garantiert ALLE Topics aktiv sind.
    """

    GROUP = "scene"

    def __init__(self, content: AppContent, namespace: str = "") -> None:
        self.signals = SceneSignals()
        super().__init__("scene_bridge", content, namespace=namespace)

        # ---------- Publishers (UI -> ROS) ----------
        self._pub_cage_set = self._make_pub("cage_set")
        self._pub_mount_set = self._make_pub("mount_set")
        self._pub_substrate_set = self._make_pub("substrate_set")

        # ---------- Subscriptions (ROS -> UI) ----------
        self._sub_cage_list = self._make_sub("cage_list", self._on_cage_list)
        self._sub_mount_list = self._make_sub("mount_list", self._on_mount_list)
        self._sub_substrate_list = self._make_sub("substrate_list", self._on_substrate_list)

        self._sub_cage_current = self._make_sub("cage_current", self._on_cage_current)
        self._sub_mount_current = self._make_sub("mount_current", self._on_mount_current)
        self._sub_substrate_current = self._make_sub("substrate_current", self._on_substrate_current)

        # ---------- Qt outbound ----------
        s = self.signals
        s.setCageRequested.connect(self.set_cage)
        s.setMountRequested.connect(self.set_mount)
        s.setSubstrateRequested.connect(self.set_substrate)

        self.get_logger().info(f"[scene_bridge] ready (ns='{self.namespace or '/'}')")

    # ------------------------------------------------------------------
    # Spec helpers (wie PosesBridge: nur spec() benutzen)
    # ------------------------------------------------------------------
    def _make_pub(self, topic_id: str):
        try:
            spec = self.spec("subscribe", topic_id)     # UI -> Node
            Msg = spec.resolve_type()
            qos = getattr(spec, "qos_profile", None)
            pub = self.create_publisher(Msg, spec.name, qos if qos is not None else 10)
            return pub
        except Exception as e:
            self.get_logger().error(f"[scene_bridge] publisher init failed id={topic_id}: {e}")
            return None

    def _make_sub(self, topic_id: str, cb):
        try:
            spec = self.spec("publish", topic_id)       # Node -> UI
            Msg = spec.resolve_type()
            qos = getattr(spec, "qos_profile", None)
            sub = self.create_subscription(Msg, spec.name, cb, qos if qos is not None else 10)
            return sub
        except Exception as e:
            self.get_logger().error(f"[scene_bridge] subscription init failed id={topic_id}: {e}")
            return None

    # ------------------------------------------------------------------
    # CSV parsing
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_csv_list(csv: str) -> List[str]:
        csv = (csv or "").strip()
        if not csv:
            return []
        return [x.strip() for x in csv.split(",") if x.strip()]

    # ------------------------------------------------------------------
    # ROS -> UI callbacks
    # ------------------------------------------------------------------
    def _on_cage_list(self, msg: MsgString) -> None:
        items = self._parse_csv_list(msg.data)
        self.signals.last_cage_list = list(items)
        self.signals.cageListChanged.emit(list(items))

    def _on_mount_list(self, msg: MsgString) -> None:
        items = self._parse_csv_list(msg.data)
        self.signals.last_mount_list = list(items)
        self.signals.mountListChanged.emit(list(items))

    def _on_substrate_list(self, msg: MsgString) -> None:
        items = self._parse_csv_list(msg.data)
        self.signals.last_substrate_list = list(items)
        self.signals.substrateListChanged.emit(list(items))

    def _on_cage_current(self, msg: MsgString) -> None:
        v = (msg.data or "").strip()
        self.signals.last_cage_current = v
        self.signals.cageCurrentChanged.emit(v)

    def _on_mount_current(self, msg: MsgString) -> None:
        v = (msg.data or "").strip()
        self.signals.last_mount_current = v
        self.signals.mountCurrentChanged.emit(v)

    def _on_substrate_current(self, msg: MsgString) -> None:
        v = (msg.data or "").strip()
        self.signals.last_substrate_current = v
        self.signals.substrateCurrentChanged.emit(v)

    # ------------------------------------------------------------------
    # UI -> ROS public API
    # ------------------------------------------------------------------
    def set_cage(self, name: str) -> None:
        if self._pub_cage_set is None:
            self.get_logger().error("[scene_bridge] cage_set publish failed: publisher not initialized")
            return
        self._pub_cage_set.publish(MsgString(data=(name or "").strip()))

    def set_mount(self, name: str) -> None:
        if self._pub_mount_set is None:
            self.get_logger().error("[scene_bridge] mount_set publish failed: publisher not initialized")
            return
        self._pub_mount_set.publish(MsgString(data=(name or "").strip()))

    def set_substrate(self, name: str) -> None:
        if self._pub_substrate_set is None:
            self.get_logger().error("[scene_bridge] substrate_set publish failed: publisher not initialized")
            return
        self._pub_substrate_set.publish(MsgString(data=(name or "").strip()))
