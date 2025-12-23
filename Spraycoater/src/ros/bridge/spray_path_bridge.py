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
    Qt-Signalträger für SprayPath.

    Inbound (ROS -> UI):
      - currentChanged(str)
      - posesChanged(PoseArray)
      - markersChanged(MarkerArray)
      - executedPosesChanged(PoseArray)
      - executedMarkersChanged(MarkerArray)

    Outbound (UI -> ROS):
      - setViewRequested(str)
      - compiledPosesRequested(PoseArray)
      - compiledMarkersRequested(MarkerArray)
      - trajPosesRequested(PoseArray)
      - trajMarkersRequested(MarkerArray)
      - executedPosesRequested(PoseArray)
      - executedMarkersRequested(MarkerArray)
    """

    # Inbound
    currentChanged = QtCore.pyqtSignal(str)
    posesChanged = QtCore.pyqtSignal(object)           # PoseArray
    markersChanged = QtCore.pyqtSignal(object)         # MarkerArray
    executedPosesChanged = QtCore.pyqtSignal(object)   # PoseArray
    executedMarkersChanged = QtCore.pyqtSignal(object) # MarkerArray

    # Outbound
    setViewRequested = QtCore.pyqtSignal(object)          # str (object for Qt)
    compiledPosesRequested = QtCore.pyqtSignal(object)    # PoseArray
    compiledMarkersRequested = QtCore.pyqtSignal(object)  # MarkerArray
    trajPosesRequested = QtCore.pyqtSignal(object)        # PoseArray
    trajMarkersRequested = QtCore.pyqtSignal(object)      # MarkerArray
    executedPosesRequested = QtCore.pyqtSignal(object)    # PoseArray
    executedMarkersRequested = QtCore.pyqtSignal(object)  # MarkerArray

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # Cache for re-emit
        self.current: str = ""
        self.poses: Optional[PoseArray] = None
        self.markers: Optional[MarkerArray] = None
        self.executed_poses: Optional[PoseArray] = None
        self.executed_markers: Optional[MarkerArray] = None

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        if self.current:
            self.currentChanged.emit(self.current)
        if self.poses is not None:
            self.posesChanged.emit(self.poses)
        if self.markers is not None:
            self.markersChanged.emit(self.markers)
        if self.executed_poses is not None:
            self.executedPosesChanged.emit(self.executed_poses)
        if self.executed_markers is not None:
            self.executedMarkersChanged.emit(self.executed_markers)


class SprayPathBridge(BaseBridge):
    """
    UI-Bridge für 'spray_path' (compiled + traj + executed overlay).

    SUB (ROS -> UI, topics.spray_path.publish.*):
      - current
      - poses
      - markers
      - executed_poses
      - executed_markers

    PUB (UI -> ROS, topics.spray_path.subscribe.*):
      - set_view
      - compiled_poses_in / compiled_markers_in
      - traj_poses_in     / traj_markers_in
      - executed_poses_in / executed_markers_in
    """

    GROUP = "spray_path"

    def __init__(self, content: AppContent, namespace: str = ""):
        self.signals = SprayPathSignals()

        # Local state
        self.current: str = ""
        self.poses: Optional[PoseArray] = None
        self.markers: Optional[MarkerArray] = None
        self.executed_poses: Optional[PoseArray] = None
        self.executed_markers: Optional[MarkerArray] = None

        super().__init__("spray_path_bridge", content, namespace=namespace)

        # Qt Outbound -> ROS
        self.signals.setViewRequested.connect(self.publish_set_view)
        self.signals.compiledPosesRequested.connect(self.publish_compiled_poses)
        self.signals.compiledMarkersRequested.connect(self.publish_compiled_markers)
        self.signals.trajPosesRequested.connect(self.publish_traj_poses)
        self.signals.trajMarkersRequested.connect(self.publish_traj_markers)
        self.signals.executedPosesRequested.connect(self.publish_executed_poses)
        self.signals.executedMarkersRequested.connect(self.publish_executed_markers)

    # ---------------- ROS -> UI ----------------

    @sub_handler("spray_path", "current")
    def _on_current(self, msg: String):
        self.current = (msg.data or "")
        self.signals.current = self.current
        self.signals.currentChanged.emit(self.current)

    @sub_handler("spray_path", "poses")
    def _on_poses(self, msg: PoseArray):
        self.poses = msg
        self.signals.poses = msg
        self.signals.posesChanged.emit(msg)

    @sub_handler("spray_path", "markers")
    def _on_markers(self, msg: MarkerArray):
        self.markers = msg
        self.signals.markers = msg
        self.signals.markersChanged.emit(msg)

    @sub_handler("spray_path", "executed_poses")
    def _on_executed_poses(self, msg: PoseArray):
        self.executed_poses = msg
        self.signals.executed_poses = msg
        self.signals.executedPosesChanged.emit(msg)

    @sub_handler("spray_path", "executed_markers")
    def _on_executed_markers(self, msg: MarkerArray):
        self.executed_markers = msg
        self.signals.executed_markers = msg
        self.signals.executedMarkersChanged.emit(msg)

    # ---------------- UI -> ROS ----------------

    def publish_set_view(self, view_key: str) -> None:
        try:
            topic_id = "set_view"
            Msg = self.spec("subscribe", topic_id).resolve_type()
            m = Msg()
            m.data = str(view_key or "")
            self.pub(topic_id).publish(m)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publish_set_view failed: {e}")

    def publish_compiled_poses(self, pose_array: PoseArray) -> None:
        self._pub_checked("compiled_poses_in", pose_array)

    def publish_compiled_markers(self, marker_array: MarkerArray) -> None:
        self._pub_checked("compiled_markers_in", marker_array)

    def publish_traj_poses(self, pose_array: PoseArray) -> None:
        self._pub_checked("traj_poses_in", pose_array)

    def publish_traj_markers(self, marker_array: MarkerArray) -> None:
        self._pub_checked("traj_markers_in", marker_array)

    def publish_executed_poses(self, pose_array: PoseArray) -> None:
        self._pub_checked("executed_poses_in", pose_array)

    def publish_executed_markers(self, marker_array: MarkerArray) -> None:
        self._pub_checked("executed_markers_in", marker_array)

    # ---------------- internal helper ----------------

    def _pub_checked(self, topic_id: str, msg_obj) -> None:
        try:
            Msg = self.spec("subscribe", topic_id).resolve_type()
            if not isinstance(msg_obj, Msg):
                raise TypeError(f"{topic_id} expects {Msg.__name__}, got {type(msg_obj).__name__}")
            self.pub(topic_id).publish(msg_obj)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publish {topic_id} failed: {e}")
