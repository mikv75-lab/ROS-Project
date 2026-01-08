# -*- coding: utf-8 -*-
# File: ros/bridge/spray_path_bridge.py

from __future__ import annotations

import logging
from typing import Any, Mapping, Optional

from PyQt6 import QtCore

from std_msgs.msg import Bool as MsgBool
from std_msgs.msg import Empty as MsgEmpty
from geometry_msgs.msg import PoseArray

from config.startup import AppContent
from .base_bridge import BaseBridge

try:
    from visualization_msgs.msg import MarkerArray
except Exception:  # pragma: no cover
    MarkerArray = Any  # type: ignore

_LOG = logging.getLogger("ros.bridge.spray_path_bridge")


class SprayPathSignals(QtCore.QObject):
    # ---------------- GUI -> Bridge (requests) ----------------

    showCompiledRequested = QtCore.pyqtSignal(bool)

    # Preferred semantic name
    showPlannedRequested = QtCore.pyqtSignal(bool)

    # Legacy alias used historically by ProcessTab/UI
    showTrajRequested = QtCore.pyqtSignal(bool)

    showExecutedRequested = QtCore.pyqtSignal(bool)
    clearRequested = QtCore.pyqtSignal()  # explicit clear

    # ---------------- Bridge -> GUI (availability) ----------------
    compiledAvailableChanged = QtCore.pyqtSignal(bool)

    # Preferred semantic name
    plannedAvailableChanged = QtCore.pyqtSignal(bool)

    # Legacy alias
    trajAvailableChanged = QtCore.pyqtSignal(bool)

    executedAvailableChanged = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # cached flags (optional)
        self._last_show_compiled: bool = True
        self._last_show_planned: bool = True
        self._last_show_executed: bool = True

        self._last_avail_compiled: bool = False
        self._last_avail_planned: bool = False
        self._last_avail_executed: bool = False

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        self.compiledAvailableChanged.emit(bool(self._last_avail_compiled))
        self.plannedAvailableChanged.emit(bool(self._last_avail_planned))
        self.trajAvailableChanged.emit(bool(self._last_avail_planned))  # legacy alias
        self.executedAvailableChanged.emit(bool(self._last_avail_executed))


class SprayPathBridge(BaseBridge):
    """
    UI bridge for spray_path cache node.

    Contract (matches spray_path.py + topics.yaml):
      - GUI publishes to *_poses_in and *_markers_in (latched)
      - GUI publishes toggles show_* (latched)
      - GUI publishes clear (Empty)
      - Node republishes to outputs for RViz/others

    Important:
      This bridge publishes to:
        compiled/*_in, planned/*_in, executed/*_in
      NOT to compiled_markers/planned_markers/executed_markers directly.
    """

    GROUP = "spray_path"

    def __init__(
        self,
        content: AppContent,
        namespace: str = "",
        config: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.signals = SprayPathSignals()

        # local state
        self._show_compiled = True
        self._show_planned = True
        self._show_executed = True

        self._avail_compiled = False
        self._avail_planned = False
        self._avail_executed = False

        self._last_compiled_markers: Optional[MarkerArray] = None
        self._last_planned_markers: Optional[MarkerArray] = None
        self._last_executed_markers: Optional[MarkerArray] = None

        self._last_compiled_poses: Optional[PoseArray] = None
        self._last_planned_poses: Optional[PoseArray] = None
        self._last_executed_poses: Optional[PoseArray] = None

        super().__init__("spray_path_bridge", content, namespace=namespace, config=config)

        cfg = dict(config or {})

        # ---------------- IDs (inputs to node) ----------------
        id_compiled_poses_in = str(cfg.get("id_compiled_poses_in", "compiled_poses_in"))
        id_compiled_markers_in = str(cfg.get("id_compiled_markers_in", "compiled_markers_in"))

        # Preferred IDs
        id_planned_poses_in = str(cfg.get("id_planned_poses_in", "planned_poses_in"))
        id_planned_markers_in = str(cfg.get("id_planned_markers_in", "planned_markers_in"))

        # Legacy config keys (optional) -> fallback
        id_traj_poses_in = str(cfg.get("id_traj_poses_in", "")).strip()  # legacy
        id_traj_markers_in = str(cfg.get("id_traj_markers_in", "")).strip()  # legacy
        if (not id_planned_poses_in or id_planned_poses_in.strip() == "") and id_traj_poses_in:
            id_planned_poses_in = id_traj_poses_in
        if (not id_planned_markers_in or id_planned_markers_in.strip() == "") and id_traj_markers_in:
            id_planned_markers_in = id_traj_markers_in

        id_executed_poses_in = str(cfg.get("id_executed_poses_in", "executed_poses_in"))
        id_executed_markers_in = str(cfg.get("id_executed_markers_in", "executed_markers_in"))

        # ---------------- toggle IDs (GUI -> node) ----------------
        self._id_show_compiled = str(cfg.get("id_show_compiled", "show_compiled"))

        # Preferred toggle id
        self._id_show_planned = str(cfg.get("id_show_planned", "show_planned"))

        # Legacy toggle id fallback
        legacy_show_traj = str(cfg.get("id_show_traj", "")).strip()  # legacy
        if (not self._id_show_planned or self._id_show_planned.strip() == "") and legacy_show_traj:
            self._id_show_planned = legacy_show_traj

        self._id_show_executed = str(cfg.get("id_show_executed", "show_executed"))
        self._id_clear = str(cfg.get("id_clear", "clear"))

        # ---------------- publishers (to node inputs) ----------------
        self._pub_compiled_poses_in = None
        self._pub_compiled_markers_in = None
        self._pub_planned_poses_in = None
        self._pub_planned_markers_in = None
        self._pub_executed_poses_in = None
        self._pub_executed_markers_in = None

        try:
            self._pub_compiled_poses_in = content.create_publisher_from_id(self, self.GROUP, id_compiled_poses_in)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_compiled_poses_in}): {e}")

        try:
            self._pub_compiled_markers_in = content.create_publisher_from_id(self, self.GROUP, id_compiled_markers_in)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_compiled_markers_in}): {e}")

        try:
            self._pub_planned_poses_in = content.create_publisher_from_id(self, self.GROUP, id_planned_poses_in)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_planned_poses_in}): {e}")

        try:
            self._pub_planned_markers_in = content.create_publisher_from_id(self, self.GROUP, id_planned_markers_in)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_planned_markers_in}): {e}")

        try:
            self._pub_executed_poses_in = content.create_publisher_from_id(self, self.GROUP, id_executed_poses_in)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_executed_poses_in}): {e}")

        try:
            self._pub_executed_markers_in = content.create_publisher_from_id(self, self.GROUP, id_executed_markers_in)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_executed_markers_in}): {e}")

        # ---------------- Qt wiring ----------------
        self.signals.showCompiledRequested.connect(lambda v: self._on_show_compiled(bool(v)))
        self.signals.showPlannedRequested.connect(lambda v: self._on_show_planned(bool(v)))

        # Legacy alias -> planned
        self.signals.showTrajRequested.connect(lambda v: self._on_show_planned(bool(v)))

        self.signals.showExecutedRequested.connect(lambda v: self._on_show_executed(bool(v)))
        self.signals.clearRequested.connect(self.clear)

        # publish initial toggles (latched) and push current cached inputs once
        self._publish_toggle(self._id_show_compiled, True)
        self._publish_toggle(self._id_show_planned, True)
        self._publish_toggle(self._id_show_executed, True)
        self._republish_all_inputs()

    # ---------------- toggles + clear (GUI -> node) ----------------

    def _publish_toggle(self, topic_id: str, v: bool) -> None:
        if not topic_id:
            return
        try:
            self.pub(topic_id).publish(MsgBool(data=bool(v)))
        except Exception as e:
            self.get_logger().debug(f"[spray_path] toggle publish failed id={topic_id}: {e}")

    def clear(self) -> None:
        """
        Explicit clear: tells spray_path node to reset caches + outputs.
        """
        try:
            self.pub(self._id_clear).publish(MsgEmpty())
        except Exception as e:
            self.get_logger().warning(f"[spray_path] clear publish failed id={self._id_clear}: {e}")

    def _on_show_compiled(self, v: bool) -> None:
        self._show_compiled = bool(v)
        self.signals._last_show_compiled = self._show_compiled
        self._publish_toggle(self._id_show_compiled, self._show_compiled)
        self._republish_compiled_inputs()

    def _on_show_planned(self, v: bool) -> None:
        self._show_planned = bool(v)
        self.signals._last_show_planned = self._show_planned
        self._publish_toggle(self._id_show_planned, self._show_planned)
        self._republish_planned_inputs()

    def _on_show_executed(self, v: bool) -> None:
        self._show_executed = bool(v)
        self.signals._last_show_executed = self._show_executed
        self._publish_toggle(self._id_show_executed, self._show_executed)
        self._republish_executed_inputs()

    # ---------------- public API (ProcessTab / publisher side) ----------------

    def set_compiled(self, *, poses: Optional[PoseArray] = None, markers: Optional[MarkerArray] = None) -> None:
        if poses is not None:
            self._last_compiled_poses = poses
        if markers is not None:
            self._last_compiled_markers = markers
        self._set_avail("compiled", self._has_any(self._last_compiled_poses, self._last_compiled_markers))
        self._republish_compiled_inputs()

    def set_planned(self, *, poses: Optional[PoseArray] = None, markers: Optional[MarkerArray] = None) -> None:
        if poses is not None:
            self._last_planned_poses = poses
        if markers is not None:
            self._last_planned_markers = markers
        self._set_avail("planned", self._has_any(self._last_planned_poses, self._last_planned_markers))
        self._republish_planned_inputs()

    # Legacy API alias
    def set_traj(self, *, poses: Optional[PoseArray] = None, markers: Optional[MarkerArray] = None) -> None:
        self.set_planned(poses=poses, markers=markers)

    def set_executed(self, *, poses: Optional[PoseArray] = None, markers: Optional[MarkerArray] = None) -> None:
        if poses is not None:
            self._last_executed_poses = poses
        if markers is not None:
            self._last_executed_markers = markers
        self._set_avail("executed", self._has_any(self._last_executed_poses, self._last_executed_markers))
        self._republish_executed_inputs()

    # ---------------- helpers ----------------

    def _has_any(self, poses: Any, markers: Any) -> bool:
        return self._has_poses(poses) or self._has_markers(markers)

    def _has_poses(self, msg: Any) -> bool:
        try:
            return bool(msg is not None and hasattr(msg, "poses") and isinstance(msg.poses, list) and len(msg.poses) > 0)
        except Exception:
            return False

    def _has_markers(self, msg: Any) -> bool:
        try:
            return bool(msg is not None and hasattr(msg, "markers") and isinstance(msg.markers, list) and len(msg.markers) > 0)
        except Exception:
            return False

    def _set_avail(self, which: str, v: bool) -> None:
        v = bool(v)

        if which == "compiled":
            if v != self._avail_compiled:
                self._avail_compiled = v
                self.signals._last_avail_compiled = v
                self.signals.compiledAvailableChanged.emit(v)

        elif which == "planned":
            if v != self._avail_planned:
                self._avail_planned = v
                self.signals._last_avail_planned = v
                self.signals.plannedAvailableChanged.emit(v)
                self.signals.trajAvailableChanged.emit(v)  # legacy alias

        elif which == "executed":
            if v != self._avail_executed:
                self._avail_executed = v
                self.signals._last_avail_executed = v
                self.signals.executedAvailableChanged.emit(v)

    # ---------------- (re)publish to node inputs ----------------

    def _republish_compiled_inputs(self) -> None:
        try:
            if self._pub_compiled_poses_in is not None:
                self._pub_compiled_poses_in.publish(
                    self._last_compiled_poses
                    if (self._show_compiled and self._last_compiled_poses is not None)
                    else PoseArray()
                )
        except Exception:
            _LOG.exception("SprayPathBridge: republish_compiled poses_in failed")

        try:
            if self._pub_compiled_markers_in is not None:
                self._pub_compiled_markers_in.publish(
                    self._last_compiled_markers
                    if (self._show_compiled and self._last_compiled_markers is not None)
                    else MarkerArray()
                )
        except Exception:
            _LOG.exception("SprayPathBridge: republish_compiled markers_in failed")

    def _republish_planned_inputs(self) -> None:
        try:
            if self._pub_planned_poses_in is not None:
                self._pub_planned_poses_in.publish(
                    self._last_planned_poses
                    if (self._show_planned and self._last_planned_poses is not None)
                    else PoseArray()
                )
        except Exception:
            _LOG.exception("SprayPathBridge: republish_planned poses_in failed")

        try:
            if self._pub_planned_markers_in is not None:
                self._pub_planned_markers_in.publish(
                    self._last_planned_markers
                    if (self._show_planned and self._last_planned_markers is not None)
                    else MarkerArray()
                )
        except Exception:
            _LOG.exception("SprayPathBridge: republish_planned markers_in failed")

    def _republish_executed_inputs(self) -> None:
        try:
            if self._pub_executed_poses_in is not None:
                self._pub_executed_poses_in.publish(
                    self._last_executed_poses
                    if (self._show_executed and self._last_executed_poses is not None)
                    else PoseArray()
                )
        except Exception:
            _LOG.exception("SprayPathBridge: republish_executed poses_in failed")

        try:
            if self._pub_executed_markers_in is not None:
                self._pub_executed_markers_in.publish(
                    self._last_executed_markers
                    if (self._show_executed and self._last_executed_markers is not None)
                    else MarkerArray()
                )
        except Exception:
            _LOG.exception("SprayPathBridge: republish_executed markers_in failed")

    def _republish_all_inputs(self) -> None:
        self._republish_compiled_inputs()
        self._republish_planned_inputs()
        self._republish_executed_inputs()
