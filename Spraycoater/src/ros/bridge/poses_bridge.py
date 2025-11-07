# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class PosesSignals(QtCore.QObject):
    """
    Qt-Signalträger für die Poses-Bridge.

    Inbound (ROS -> UI):
      - homePoseChanged(PoseStamped)
      - servicePoseChanged(PoseStamped)

    Outbound (UI -> ROS):
      - setHomeRequested()
      - setServiceRequested()
    """
    homePoseChanged = QtCore.pyqtSignal(object)     # PoseStamped
    servicePoseChanged = QtCore.pyqtSignal(object)  # PoseStamped

    setHomeRequested = QtCore.pyqtSignal()
    setServiceRequested = QtCore.pyqtSignal()


class PosesBridge(BaseBridge):
    """
    UI-Bridge für 'poses'.

    Abonniert:
      - poses.home_pose (PoseStamped, latched)
      - poses.service_pose (PoseStamped, latched)

    Publiziert (aus subscribe-Spec abgeleitet, keine Fallbacks):
      - poses.set_home (std_msgs/Empty)
      - poses.set_service (std_msgs/Empty)
    """
    GROUP = "poses"

    def __init__(self, content: AppContent):
        # Qt-Signale zuerst
        self.signals = PosesSignals()

        # Interner State
        self.home_pose: Optional[PoseStamped] = None
        self.service_pose: Optional[PoseStamped] = None

        super().__init__("poses_bridge", content)

        # Publisher für set_home / set_service aus subscribe-Spec ableiten (ohne Fallbacks)
        self._pub_set_home = None
        self._pub_set_service = None
        try:
            spec_home = self.spec("subscribe", "set_home")
            MsgHome = spec_home.resolve_type()
            qos_home = getattr(spec_home, "qos_profile", None)
            self._pub_set_home = self.create_publisher(MsgHome, spec_home.name, qos_home if qos_home is not None else 10)
        except Exception as e:
            self.get_logger().error(f"[poses] set_home publisher init failed: {e}")
            self._pub_set_home = None

        try:
            spec_serv = self.spec("subscribe", "set_service")
            MsgServ = spec_serv.resolve_type()
            qos_serv = getattr(spec_serv, "qos_profile", None)
            self._pub_set_service = self.create_publisher(MsgServ, spec_serv.name, qos_serv if qos_serv is not None else 10)
        except Exception as e:
            self.get_logger().error(f"[poses] set_service publisher init failed: {e}")
            self._pub_set_service = None

        # Outbound UI -> ROS
        self.signals.setHomeRequested.connect(self.set_home)
        self.signals.setServiceRequested.connect(self.set_service)

    # ---------------- eingehend (ROS -> UI) ----------------

    @sub_handler("poses", "home_pose")
    def _on_home_pose(self, msg: PoseStamped):
        self.home_pose = msg
        self.signals.homePoseChanged.emit(msg)
        self.get_logger().info("[poses] home_pose received")

    @sub_handler("poses", "service_pose")
    def _on_service_pose(self, msg: PoseStamped):
        self.service_pose = msg
        self.signals.servicePoseChanged.emit(msg)
        self.get_logger().info("[poses] service_pose received")

    # ---------------- ausgehend (UI -> ROS) ----------------

    def set_home(self) -> None:
        if self._pub_set_home is None:
            self.get_logger().error("[poses] publish set_home failed: publisher not initialized")
            return
        self._pub_set_home.publish(Empty())
        self.get_logger().info("[poses] -> set_home (Empty)")

    def set_service(self) -> None:
        if self._pub_set_service is None:
            self.get_logger().error("[poses] publish set_service failed: publisher not initialized")
            return
        self._pub_set_service.publish(Empty())
        self.get_logger().info("[poses] -> set_service (Empty)")

    # ---------------- Initialwert-Getter ----------------

    def get_last_home_pose(self) -> Optional[PoseStamped]:
        return self.home_pose

    def get_last_service_pose(self) -> Optional[PoseStamped]:
        return self.service_pose
