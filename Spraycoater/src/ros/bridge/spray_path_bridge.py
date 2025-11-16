# src/ros/bridge/spray_path_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional

from PyQt6 import QtCore
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import MarkerArray

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class SprayPathSignals(QtCore.QObject):
    """
    Qt-Signalträger für die SprayPath-Bridge.

    Inbound (ROS -> UI):
      - currentNameChanged(str)       # latched Name
      - posesChanged(PoseArray)       # aktueller Pfad als PoseArray

    Outbound (UI -> ROS):
      - setRequested(MarkerArray)     # UI erzeugt MarkerArray für SprayPath
    """

    # Inbound
    currentNameChanged = QtCore.pyqtSignal(str)
    posesChanged = QtCore.pyqtSignal(object)   # PoseArray

    # Outbound
    setRequested = QtCore.pyqtSignal(object)   # MarkerArray


class SprayPathBridge(BaseBridge):
    """
    UI-Bridge für 'spray_path' mit eingebauten Qt-Signalen.

    Abonniert:
      - spray_path.current (String, latched)
      - spray_path.poses   (PoseArray)

    Publiziert:
      - spray_path.set (MarkerArray)
    """
    GROUP = "spray_path"

    def __init__(self, content: AppContent):
        self.signals = SprayPathSignals()

        # Interner State (roh, ohne Defaults/Fallbacks)
        self.current_name: str = ""
        self.poses: Optional[PoseArray] = None

        # Spiegeln auf Signals-Objekt für initialen UI-Pull
        self.signals.current_name = ""
        self.signals.poses = None

        super().__init__("spray_path_bridge", content)

        # Qt Outbound -> ROS
        self.signals.setRequested.connect(self.publish_set)

    # -------- eingehend (ROS -> UI) --------

    @sub_handler("spray_path", "current")
    def _on_current(self, msg: String):
        self.current_name = (msg.data or "")
        self.signals.current_name = self.current_name
        self.signals.currentNameChanged.emit(self.current_name)
        self.get_logger().info(f"[spray_path] current: {self.current_name or '-'}")

    @sub_handler("spray_path", "poses")
    def _on_poses(self, msg: PoseArray):
        self.poses = msg
        self.signals.poses = msg
        self.signals.posesChanged.emit(msg)
        self.get_logger().info(f"[spray_path] poses received: {len(msg.poses)} poses")

    # -------- ausgehend (UI -> ROS) --------

    def publish_set(self, marker_array: MarkerArray) -> None:
        """
        Publisht das übergebene MarkerArray unverändert auf spray_path.set.
        UI kann dieses Signal mit einem MarkerArray speisen (z.B. aus dem Recipe-Editor).
        """
        try:
            topic_id = "set"
            Msg = self.spec("subscribe", topic_id).resolve_type()
            if not isinstance(marker_array, Msg):
                # bewusst *keine* Konvertierung/Fallbacks – Typ muss stimmen
                raise TypeError(
                    f"spray_path.set erwartet {Msg.__name__}, bekommen: {type(marker_array).__name__}"
                )
            pub = self.pub(topic_id)
            self.get_logger().info(
                f"[spray_path] -> {topic_id}: publishing MarkerArray with {len(marker_array.markers)} markers"
            )
            pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publish set failed: {e}")
