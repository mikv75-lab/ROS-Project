# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional

from PyQt6 import QtCore
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from visualization_msgs.msg import Marker, MarkerArray

from config.startup import AppContent
from .base_bridge import BaseBridge, sub_handler


class SprayPathSignals(QtCore.QObject):
    """
    Qt-Signalträger für die SprayPath-Bridge.

    Inbound (ROS -> UI):
      - currentNameChanged(str)       # latched Name
      - posesChanged(PoseArray)       # aktueller Pfad als PoseArray
      - markerChanged(Marker)         # LINE_STRIP Marker

    Outbound (UI -> ROS):
      - setRequested(object)          # erwartet MarkerArray
    """
    # Inbound
    currentNameChanged = QtCore.pyqtSignal(str)
    posesChanged = QtCore.pyqtSignal(object)   # PoseArray
    markerChanged = QtCore.pyqtSignal(object)  # Marker

    # Outbound
    setRequested = QtCore.pyqtSignal(object)   # MarkerArray


class SprayPathBridge(BaseBridge):
    """
    UI-Bridge für 'spraypath' mit eingebauten Qt-Signalen.

    Abonniert:
      - spraypath.current (String, latched)
      - spraypath.poses   (PoseArray)
      - spraypath.markers (Marker)

    Publiziert:
      - spraypath.set (MarkerArray)
    """
    GROUP = "spraypath"

    def __init__(self, content: AppContent):
        self.signals = SprayPathSignals()

        # Interner State (roh, ohne Defaults/Fallbacks)
        self.current_name: str = ""
        self.poses: Optional[PoseArray] = None
        self.marker: Optional[Marker] = None

        # Spiegeln auf Signals-Objekt für initialen UI-Pull
        self.signals.current_name = ""
        self.signals.poses = None
        self.signals.marker = None

        super().__init__("spray_path_bridge", content)

        # Qt Outbound -> ROS
        self.signals.setRequested.connect(self.publish_set)

    # -------- eingehend (ROS -> UI) --------

    @sub_handler("spraypath", "current")
    def _on_current(self, msg: String):
        self.current_name = (msg.data or "")
        self.signals.current_name = self.current_name
        self.signals.currentNameChanged.emit(self.current_name)
        self.get_logger().info(f"[spraypath] current: {self.current_name or '-'}")

    @sub_handler("spraypath", "poses")
    def _on_poses(self, msg: PoseArray):
        self.poses = msg
        self.signals.poses = msg
        self.signals.posesChanged.emit(msg)
        self.get_logger().info(f"[spraypath] poses received: {len(msg.poses)} poses")

    @sub_handler("spraypath", "markers")
    def _on_markers(self, msg: Marker):
        self.marker = msg
        self.signals.marker = msg
        self.signals.markerChanged.emit(msg)
        self.get_logger().info("[spraypath] marker received")

    # -------- ausgehend (UI -> ROS) --------

    def publish_set(self, marker_array: MarkerArray) -> None:
        """
        Publisht das übergebene MarkerArray unverändert auf spraypath.set.
        UI kann dieses Signal mit einem MarkerArray speisen (z.B. aus einem Editor).
        """
        try:
            topic_id = "set"
            Msg = self.spec("subscribe", topic_id).resolve_type()
            if not isinstance(marker_array, Msg):
                # bewusst *keine* Konvertierung/Fallbacks – Typ muss stimmen
                raise TypeError(f"spraypath.set erwartet {Msg.__name__}, bekommen: {type(marker_array).__name__}")
            pub = self.pub(topic_id)
            self.get_logger().info(f"[spraypath] -> {topic_id}: publishing MarkerArray with {len(marker_array.markers)} markers")
            pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"[spraypath] publish set failed: {e}")
