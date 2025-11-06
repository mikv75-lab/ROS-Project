# src/ros/bridge/poses_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler
from spraycoater_nodes_py.utils.config_hub import frames  # Frame-Resolver


class PosesSignals(QtCore.QObject):
    """
    Qt-Signalträger für die Poses-Bridge.

    Inbound (ROS -> UI):
      - homePoseChanged(PoseStamped)
      - servicePoseChanged(PoseStamped)

    Outbound (UI -> ROS):
      - setPoseRequested(str)   # erwartet exakt "home" oder "service"

    Zusätzlich spiegeln wir den aktuellen UI-State als Attribute:
      - home_pose: PoseStamped | None
      - service_pose: PoseStamped | None
    """
    # Inbound
    homePoseChanged = QtCore.pyqtSignal(object)     # PoseStamped
    servicePoseChanged = QtCore.pyqtSignal(object)  # PoseStamped

    # Outbound
    setPoseRequested = QtCore.pyqtSignal(str)


class PosesBridge(BaseBridge):
    """
    UI-Bridge für 'poses' mit eingebauten Qt-Signalen.

    Abonniert:
      - poses.home_pose (PoseStamped, latched)
      - poses.service_pose (PoseStamped, latched)

    Publiziert:
      - poses.pose_set (PoseStamped) via header.frame_id = "<parent>#<name>"
        (Parent wird aus frames.yaml aufgelöst; Default: world)
    """
    GROUP = "poses"

    def __init__(self, content: AppContent):
        self.signals = PosesSignals()

        # Interner State (roh, ohne Defaults/Fallbacks)
        self.home_pose: Optional[PoseStamped] = None
        self.service_pose: Optional[PoseStamped] = None

        # Spiegeln auf das Signals-Objekt für initialen UI-Pull
        self.signals.home_pose = None
        self.signals.service_pose = None

        # Frames-Resolver (für parent in '<parent>#<name>')
        self._frames = frames()

        super().__init__("poses_bridge", content)

        # Qt Outbound -> ROS
        self.signals.setPoseRequested.connect(self.set_pose_by_name)

    # -------- eingehend (ROS -> UI) --------

    @sub_handler("poses", "home_pose")
    def _on_home_pose(self, msg: PoseStamped):
        self.home_pose = msg
        self.signals.home_pose = msg
        self.signals.homePoseChanged.emit(msg)
        self.get_logger().info("[poses] home_pose received")

    @sub_handler("poses", "service_pose")
    def _on_service_pose(self, msg: PoseStamped):
        self.service_pose = msg
        self.signals.service_pose = msg
        self.signals.servicePoseChanged.emit(msg)
        self.get_logger().info("[poses] service_pose received")

    # -------- ausgehend (UI -> ROS) --------

    def set_pose_by_name(self, name: str) -> None:
        """
        Publisht eine PoseStamped auf poses.pose_set.
        Wir nutzen das Namensschema im Header: header.frame_id = '<parent>#<name>'.
        Parent wählen wir default 'world' (oder was in frames.yaml als world definiert ist).
        Orientierung neutral (w=1), Position (0,0,0).
        """
        try:
            topic_id = "pose_set"
            Msg = self.spec("subscribe", topic_id).resolve_type()  # -> PoseStamped
            pub = self.pub(topic_id)

            parent = self._frames.get("world", "world")  # sauber aufgelöst
            msg = Msg()
            msg.header.frame_id = f"{parent}#{name.strip().lower()}"
            msg.pose.orientation.w = 1.0  # neutral

            self.get_logger().info(f"[poses] -> {topic_id}: {msg.header.frame_id}")
            pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[poses] publish pose_set failed: {e}")

    # Bequeme Shortcuts
    def set_home(self) -> None:
        self.set_pose_by_name("home")

    def set_service(self) -> None:
        self.set_pose_by_name("service")
