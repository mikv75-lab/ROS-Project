# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import List, Optional, Callable

import rclpy  # (optional nutzbar für Logs etc.)
from std_msgs.msg import String
from PyQt6 import QtCore  # Qt-Signale direkt im Bridge-Node

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


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

    Zusätzlich spiegeln wir den aktuellen UI-State als Attribute auf diesem
    QObject (cage_list, mount_list, substrate_list, cage_current, mount_current,
    substrate_current), damit die UI initial synchronisieren kann, ohne auf
    neue ROS-Nachrichten warten zu müssen.
    """
    # Inbound
    cageListChanged = QtCore.pyqtSignal(list)
    mountListChanged = QtCore.pyqtSignal(list)
    substrateListChanged = QtCore.pyqtSignal(list)

    cageCurrentChanged = QtCore.pyqtSignal(str)
    mountCurrentChanged = QtCore.pyqtSignal(str)
    substrateCurrentChanged = QtCore.pyqtSignal(str)

    # Outbound
    setCageRequested = QtCore.pyqtSignal(str)
    setMountRequested = QtCore.pyqtSignal(str)
    setSubstrateRequested = QtCore.pyqtSignal(str)


class SceneBridge(BaseBridge):
    """
    UI-Bridge für 'scene' mit eingebauten Qt-Signalen.

    - Abonniert (vom ROS-Scene-Node publiziert): *_list, *_current
      -> aktualisiert internen State + emittiert Qt-Signale

    - Publiziert (vom ROS-Scene-Node abonniert): *_set
      -> via set_cage/mount/substrate oder über die Qt-Outbound-Signale
    """
    GROUP = "scene"

    def __init__(self, content: AppContent):
        # WICHTIG: Signale *vor* super().__init__() anlegen, damit bereits
        # empfangene latched-Nachrichten in den Handlern sicher Signale besitzen.
        self.signals = SceneSignals()

        # Interner UI-State
        self.cage_list: List[str] = []
        self.mount_list: List[str] = []
        self.substrate_list: List[str] = []
        self.cage_current: str = ""
        self.mount_current: str = ""
        self.substrate_current: str = ""

        # Spiegel-Attribute auf dem Signals-Objekt (für initialen UI-Sync)
        # (QObject erlaubt dynamische Attribute in Python.)
        self.signals.cage_list = []
        self.signals.mount_list = []
        self.signals.substrate_list = []
        self.signals.cage_current = ""
        self.signals.mount_current = ""
        self.signals.substrate_current = ""

        # Jetzt BaseBridge initialisieren (Publisher/Subscriber gemäß topics.yaml)
        super().__init__("scene_bridge", content)

        # Outbound Qt -> ROS wiring (UI drückt Buttons im ServiceTab)
        self.signals.setCageRequested.connect(self.set_cage)
        self.signals.setMountRequested.connect(self.set_mount)
        self.signals.setSubstrateRequested.connect(self.set_substrate)

    # -------- eingehende Nachrichten (vom ROS-Node 'publish') --------

    @sub_handler("scene", "cage_list")
    def _on_cage_list(self, msg: String):
        self.cage_list = self._parse_csv(getattr(msg, "data", ""))
        # State auch in den Signals spiegeln, damit UI ihn pullen kann
        self.signals.cage_list = list(self.cage_list)
        # Event emittieren (UI kann push-basiert reagieren)
        self.signals.cageListChanged.emit(self.cage_list)
        self.get_logger().info(f"[scene] cage_list: {len(self.cage_list)} items")

    @sub_handler("scene", "mount_list")
    def _on_mount_list(self, msg: String):
        self.mount_list = self._parse_csv(getattr(msg, "data", ""))
        self.signals.mount_list = list(self.mount_list)
        self.signals.mountListChanged.emit(self.mount_list)
        self.get_logger().info(f"[scene] mount_list: {len(self.mount_list)} items")

    @sub_handler("scene", "substrate_list")
    def _on_substrate_list(self, msg: String):
        self.substrate_list = self._parse_csv(getattr(msg, "data", ""))
        self.signals.substrate_list = list(self.substrate_list)
        self.signals.substrateListChanged.emit(self.substrate_list)
        self.get_logger().info(f"[scene] substrate_list: {len(self.substrate_list)} items")

    @sub_handler("scene", "cage_current")
    def _on_cage_current(self, msg: String):
        self.cage_current = (getattr(msg, "data", "") or "").strip()
        self.signals.cage_current = self.cage_current
        self.signals.cageCurrentChanged.emit(self.cage_current)
        self.get_logger().info(f"[scene] cage_current: {self.cage_current or '-'}")

    @sub_handler("scene", "mount_current")
    def _on_mount_current(self, msg: String):
        self.mount_current = (getattr(msg, "data", "") or "").strip()
        self.signals.mount_current = self.mount_current
        self.signals.mountCurrentChanged.emit(self.mount_current)
        self.get_logger().info(f"[scene] mount_current: {self.mount_current or '-'}")

    @sub_handler("scene", "substrate_current")
    def _on_substrate_current(self, msg: String):
        self.substrate_current = (getattr(msg, "data", "") or "").strip()
        self.signals.substrate_current = self.substrate_current
        self.signals.substrateCurrentChanged.emit(self.substrate_current)
        self.get_logger().info(f"[scene] substrate_current: {self.substrate_current or '-'}")

    # -------- Publish-API (UI -> ROS-Node 'subscribe') --------

    def set_cage(self, name: str) -> None:
        self._publish_set("cage_set", name)

    def set_mount(self, name: str) -> None:
        self._publish_set("mount_set", name)

    def set_substrate(self, name: str) -> None:
        self._publish_set("substrate_set", name)

    # -------- intern --------

    def _publish_set(self, topic_id: str, value: str) -> None:
        """
        Publisht eine std_msgs/String-Nachricht auf das Topic, das im Node
        (Scene) als 'subscribe' definiert ist (invertierte Perspektive).
        """
        try:
            Msg = self.spec("subscribe", topic_id).resolve_type()
            pub = self.pub(topic_id)
            msg = Msg()
            if hasattr(msg, "data"):
                msg.data = str(value)
            self.get_logger().info(f"[scene] -> {topic_id}: {value}")
            pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[scene] publish {topic_id} failed: {e}")

    @staticmethod
    def _parse_csv(s: str) -> List[str]:
        if not s:
            return []
        return [x.strip() for x in s.split(",") if x.strip()]
