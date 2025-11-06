# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


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
      - poses.pose_set_name (String)  # "home" | "service"
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
        Publisht exakt den übergebenen Namen (z.B. "home" oder "service")
        auf poses.pose_set_name (String).
        """
        try:
            topic_id = "pose_set_name"
            Msg = self.spec("subscribe", topic_id).resolve_type()
            pub = self.pub(topic_id)
            msg = Msg()
            msg.data = str(name)
            self.get_logger().info(f"[poses] -> {topic_id}: {msg.data}")
            pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"[poses] publish pose_set_name failed: {e}")

    # Bequeme Shortcuts (optional, nutzen exakt dieselbe Pipe)
    def set_home(self) -> None:
        self.set_pose_by_name("home")

    def set_service(self) -> None:
        self.set_pose_by_name("service")
