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
      - setRequested(MarkerArray)         # UI erzeugt MarkerArray für SprayPath (Sollpfad)
      - executedPathRequested(PoseArray)  # UI gibt gefahrenen Pfad als PoseArray (Istpfad)

    Zusätzlich:
      - reemit_cached(): emittiert gecachte Werte erneut (für UIBridge._try_reemit_cached)
    """

    # Inbound
    currentNameChanged = QtCore.pyqtSignal(str)
    posesChanged = QtCore.pyqtSignal(object)   # PoseArray

    # Outbound
    setRequested = QtCore.pyqtSignal(object)          # MarkerArray (Rezeptpfad)
    executedPathRequested = QtCore.pyqtSignal(object) # PoseArray (gefahrener Pfad)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # Cache für Re-Emit (wird von Bridge gepflegt)
        self.current_name: str = ""
        self.poses: Optional[PoseArray] = None
        self.executed_poses: Optional[PoseArray] = None

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        """
        Erneutes Aussenden der letzten bekannten Werte.
        Wird von UIBridge._try_reemit_cached() verwendet, wenn sich Widgets neu verbinden.
        """
        if self.current_name:
            self.currentNameChanged.emit(self.current_name)
        if self.poses is not None:
            self.posesChanged.emit(self.poses)
        # executed_poses hat aktuell kein eigenes Qt-Signal – nur interner Cache.
        # Falls später nötig, kann hier ein weiterer Signaltyp ergänzt werden.
        # (z.B. executedPosesChanged)


class SprayPathBridge(BaseBridge):
    """
    UI-Bridge für 'spray_path' mit eingebauten Qt-Signalen.

    Abonniert (ROS -> UI, topics.spray_path.publish.*):
      - spray_path.current        (String, latched)
      - spray_path.poses          (PoseArray)
      - spray_path.executed_poses (PoseArray)   [optional für spätere UI-Nutzung]

    Publiziert (UI -> ROS, topics.spray_path.subscribe.*):
      - spray_path.set            (MarkerArray)  [Sollpfad]
      - spray_path.executed_poses (PoseArray)   [Istpfad vom ProcessTab]
    """
    GROUP = "spray_path"

    def __init__(self, content: AppContent):
        # Signale anlegen (inkl. Cachefelder)
        self.signals = SprayPathSignals()

        # Interner State (roh, ohne Defaults/Fallbacks)
        self.current_name: str = ""
        self.poses: Optional[PoseArray] = None
        self.executed_poses: Optional[PoseArray] = None

        super().__init__("spray_path_bridge", content)

        # Qt Outbound -> ROS
        self.signals.setRequested.connect(self.publish_set)
        self.signals.executedPathRequested.connect(self.publish_executed_path)

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

    @sub_handler("spray_path", "executed_poses")
    def _on_executed_poses(self, msg: PoseArray):
        """
        Optionaler Callback, falls die UI den gefahrenen Pfad auch wieder aus ROS
        zurücklesen möchte. Aktuell nur im Bridge-internen State und im Signals-Cache.
        """
        self.executed_poses = msg
        self.signals.executed_poses = msg
        self.get_logger().info(f"[spray_path] executed_poses received: {len(msg.poses)} poses")

    # -------- ausgehend (UI -> ROS) --------

    def publish_set(self, marker_array: MarkerArray) -> None:
        """
        Publisht das übergebene MarkerArray unverändert auf spray_path.set.
        UI kann dieses Signal mit einem MarkerArray speisen (z.B. aus dem Recipe-Editor).
        """
        try:
            topic_id = "set"
            # Typ aus subscribe-Konfiguration (UI -> Node)
            Msg = self.spec("subscribe", topic_id).resolve_type()
            if not isinstance(marker_array, Msg):
                # bewusst *keine* Konvertierung/Fallbacks – Typ muss stimmen
                raise TypeError(
                    f"spray_path.set erwartet {Msg.__name__}, bekommen: {type(marker_array).__name__}"
                )
            pub = self.pub(topic_id)
            self.get_logger().info(
                f"[spray_path] -> {topic_id}: publishing MarkerArray with "
                f"{len(marker_array.markers)} markers"
            )
            pub.publish(marker_array)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publish set failed: {e}")

    def publish_executed_path(self, pose_array: PoseArray) -> None:
        """
        Publisht den gefahrenen Pfad als PoseArray auf spray_path.executed_poses.
        Wird typischerweise vom ProcessTab nach erfolgreichem Lauf aufgerufen.

        Der SprayPath-Node erzeugt daraus wiederum MarkerArray für RViz,
        analog zum Rezeptpfad.
        """
        try:
            topic_id = "executed_poses"
            # Typ aus subscribe-Konfiguration (UI -> Node)
            Msg = self.spec("subscribe", topic_id).resolve_type()
            if not isinstance(pose_array, Msg):
                raise TypeError(
                    f"spray_path.executed_poses erwartet {Msg.__name__}, bekommen: {type(pose_array).__name__}"
                )
            pub = self.pub(topic_id)
            self.get_logger().info(
                f"[spray_path] -> {topic_id}: publishing executed PoseArray with "
                f"{len(pose_array.poses)} poses"
            )
            pub.publish(pose_array)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publish_executed_path failed: {e}")
