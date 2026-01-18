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
    showPlannedRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)
    clearRequested = QtCore.pyqtSignal()

    # ---------------- Bridge -> GUI (availability) ----------------
    # (UI "Available"-Spalte wird rein aus GUI-Cache berechnet)
    compiledAvailableChanged = QtCore.pyqtSignal(bool)
    plannedAvailableChanged = QtCore.pyqtSignal(bool)
    executedAvailableChanged = QtCore.pyqtSignal(bool)

    # Legacy alias support (UI might still connect to this)
    trajAvailableChanged = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # cached availability for immediate UI state
        self._last_avail_compiled: bool = False
        self._last_avail_planned: bool = False
        self._last_avail_executed: bool = False

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        self.compiledAvailableChanged.emit(bool(self._last_avail_compiled))
        self.plannedAvailableChanged.emit(bool(self._last_avail_planned))
        self.trajAvailableChanged.emit(bool(self._last_avail_planned)) # Legacy alias
        self.executedAvailableChanged.emit(bool(self._last_avail_executed))


class SprayPathBridge(BaseBridge):
    """
    STRICT bridge for spray_path cache node (2026-01):

    Uses separate topics for New vs Stored (Ghosting), but unified Toggles.

      subscribe (GUI -> node):
        clear
        show_compiled, show_planned, show_executed
        
        compiled_poses_in, compiled_markers_in
        planned_new_poses_in, planned_new_markers_in, planned_stored_markers_in
        executed_new_poses_in, executed_new_markers_in, executed_stored_markers_in

    No legacy traj topics, no view setter.
    """

    GROUP = "spray_path"

    def __init__(
        self,
        content: AppContent,
        namespace: str = "",
        config: Optional[Mapping[str, Any]] = None,
    ) -> None:
        self.signals = SprayPathSignals()

        # local state (toggles)
        self._show_compiled = True
        self._show_planned = True
        self._show_executed = True

        # local state (availability)
        self._avail_compiled = False
        self._avail_planned = False
        self._avail_executed = False

        # local state (data cache for republishing)
        # Compiled
        self._last_compiled_poses: Optional[PoseArray] = None
        self._last_compiled_markers: Optional[MarkerArray] = None
        
        # Planned
        self._last_planned_new_poses: Optional[PoseArray] = None
        self._last_planned_new_markers: Optional[MarkerArray] = None
        self._last_planned_stored_markers: Optional[MarkerArray] = None
        
        # Executed
        self._last_executed_new_poses: Optional[PoseArray] = None
        self._last_executed_new_markers: Optional[MarkerArray] = None
        self._last_executed_stored_markers: Optional[MarkerArray] = None

        super().__init__("spray_path_bridge", content, namespace=namespace, config=config)

        # allow optional overriding of IDs, default to strict topics.yaml IDs
        cfg = dict(config or {})

        # Toggles + Clear
        self._id_clear = str(cfg.get("id_clear", "clear")).strip()
        self._id_show_compiled = str(cfg.get("id_show_compiled", "show_compiled")).strip()
        self._id_show_planned = str(cfg.get("id_show_planned", "show_planned")).strip()
        self._id_show_executed = str(cfg.get("id_show_executed", "show_executed")).strip()

        # Inputs (Compiled)
        self._id_cp_in = str(cfg.get("id_compiled_poses_in", "compiled_poses_in")).strip()
        self._id_cm_in = str(cfg.get("id_compiled_markers_in", "compiled_markers_in")).strip()

        # Inputs (Planned)
        self._id_pnp_in = str(cfg.get("id_planned_new_poses_in", "planned_new_poses_in")).strip()
        self._id_pnm_in = str(cfg.get("id_planned_new_markers_in", "planned_new_markers_in")).strip()
        self._id_psm_in = str(cfg.get("id_planned_stored_markers_in", "planned_stored_markers_in")).strip()

        # Inputs (Executed)
        self._id_enp_in = str(cfg.get("id_executed_new_poses_in", "executed_new_poses_in")).strip()
        self._id_enm_in = str(cfg.get("id_executed_new_markers_in", "executed_new_markers_in")).strip()
        self._id_esm_in = str(cfg.get("id_executed_stored_markers_in", "executed_stored_markers_in")).strip()

        # Qt wiring
        self.signals.showCompiledRequested.connect(lambda v: self._on_show_compiled(bool(v)))
        self.signals.showPlannedRequested.connect(lambda v: self._on_show_planned(bool(v)))
        self.signals.showExecutedRequested.connect(lambda v: self._on_show_executed(bool(v)))
        self.signals.clearRequested.connect(self.clear)

        # publish initial toggles (latched) and push cached inputs once
        self._publish_toggle(self._id_show_compiled, True)
        self._publish_toggle(self._id_show_planned, True)
        self._publish_toggle(self._id_show_executed, True)
        self._republish_all_inputs()

    # ---------------- toggles + clear (GUI -> node) ----------------

    def _publish_toggle(self, topic_id: str, v: bool) -> None:
        if not topic_id: return
        try: self.pub(topic_id).publish(MsgBool(data=bool(v)))
        except Exception: pass

    def clear(self) -> None:
        """Tell spray_path node to reset caches + outputs."""
        try: self.pub(self._id_clear).publish(MsgEmpty())
        except Exception as e:
            self.get_logger().warning(f"[spray_path] clear failed: {e}")

    def _on_show_compiled(self, v: bool) -> None:
        self._show_compiled = bool(v)
        self._publish_toggle(self._id_show_compiled, self._show_compiled)
        self._republish_compiled_inputs()

    def _on_show_planned(self, v: bool) -> None:
        self._show_planned = bool(v)
        self._publish_toggle(self._id_show_planned, self._show_planned)
        self._republish_planned_inputs()

    def _on_show_executed(self, v: bool) -> None:
        self._show_executed = bool(v)
        self._publish_toggle(self._id_show_executed, self._show_executed)
        self._republish_executed_inputs()

    # ---------------- public API (ProcessTab publisher side) ----------------

    def set_compiled(self, *, poses: Optional[PoseArray] = None, markers: Optional[MarkerArray] = None) -> None:
        if poses is not None: self._last_compiled_poses = poses
        if markers is not None: self._last_compiled_markers = markers
        self._update_avail("compiled")
        self._republish_compiled_inputs()

    def set_planned(self, *, 
                    new_poses: Optional[PoseArray] = None, 
                    new_markers: Optional[MarkerArray] = None,
                    stored_markers: Optional[MarkerArray] = None) -> None:
        """
        Set Planned Layer data.
        - new_poses/new_markers: Active run data (green/colored)
        - stored_markers: Ghost data from disk (gray)
        """
        if new_poses is not None: self._last_planned_new_poses = new_poses
        if new_markers is not None: self._last_planned_new_markers = new_markers
        if stored_markers is not None: self._last_planned_stored_markers = stored_markers
        
        self._update_avail("planned")
        self._republish_planned_inputs()

    def set_executed(self, *, 
                     new_poses: Optional[PoseArray] = None, 
                     new_markers: Optional[MarkerArray] = None,
                     stored_markers: Optional[MarkerArray] = None) -> None:
        """
        Set Executed Layer data.
        - new_poses/new_markers: Active run data (red/colored)
        - stored_markers: Ghost data from disk (gray)
        """
        if new_poses is not None: self._last_executed_new_poses = new_poses
        if new_markers is not None: self._last_executed_new_markers = new_markers
        if stored_markers is not None: self._last_executed_stored_markers = stored_markers
        
        self._update_avail("executed")
        self._republish_executed_inputs()

    # ---------------- helpers ----------------

    def _update_avail(self, which: str) -> None:
        if which == "compiled":
            v = self._has_any(self._last_compiled_poses, self._last_compiled_markers)
            if v != self._avail_compiled:
                self._avail_compiled = v
                self.signals._last_avail_compiled = v
                self.signals.compiledAvailableChanged.emit(v)

        elif which == "planned":
            # Check if ANY planned data exists (new or stored)
            v = (self._has_any(self._last_planned_new_poses, self._last_planned_new_markers) or 
                 self._has_markers(self._last_planned_stored_markers))
            if v != self._avail_planned:
                self._avail_planned = v
                self.signals._last_avail_planned = v
                self.signals.plannedAvailableChanged.emit(v)
                self.signals.trajAvailableChanged.emit(v)

        elif which == "executed":
            v = (self._has_any(self._last_executed_new_poses, self._last_executed_new_markers) or 
                 self._has_markers(self._last_executed_stored_markers))
            if v != self._avail_executed:
                self._avail_executed = v
                self.signals._last_avail_executed = v
                self.signals.executedAvailableChanged.emit(v)

    def _has_any(self, poses: Any, markers: Any) -> bool:
        return self._has_poses(poses) or self._has_markers(markers)

    def _has_poses(self, msg: Any) -> bool:
        try: return bool(msg and hasattr(msg, "poses") and len(msg.poses) > 0)
        except: return False

    def _has_markers(self, msg: Any) -> bool:
        try: return bool(msg and hasattr(msg, "markers") and len(msg.markers) > 0)
        except: return False

    # ---------------- (re)publish to node inputs ----------------

    def _pub_if_shown(self, topic_id: str, msg: Any, show_flag: bool, empty_factory):
        try:
            if topic_id:
                val = msg if (show_flag and msg is not None) else empty_factory()
                self.pub(topic_id).publish(val)
        except Exception:
            _LOG.exception(f"SprayPathBridge: publish failed for {topic_id}")

    def _republish_compiled_inputs(self) -> None:
        self._pub_if_shown(self._id_cp_in, self._last_compiled_poses, self._show_compiled, PoseArray)
        self._pub_if_shown(self._id_cm_in, self._last_compiled_markers, self._show_compiled, MarkerArray)

    def _republish_planned_inputs(self) -> None:
        self._pub_if_shown(self._id_pnp_in, self._last_planned_new_poses, self._show_planned, PoseArray)
        self._pub_if_shown(self._id_pnm_in, self._last_planned_new_markers, self._show_planned, MarkerArray)
        self._pub_if_shown(self._id_psm_in, self._last_planned_stored_markers, self._show_planned, MarkerArray)

    def _republish_executed_inputs(self) -> None:
        self._pub_if_shown(self._id_enp_in, self._last_executed_new_poses, self._show_executed, PoseArray)
        self._pub_if_shown(self._id_enm_in, self._last_executed_new_markers, self._show_executed, MarkerArray)
        self._pub_if_shown(self._id_esm_in, self._last_executed_stored_markers, self._show_executed, MarkerArray)

    def _republish_all_inputs(self) -> None:
        self._republish_compiled_inputs()
        self._republish_planned_inputs()
        self._republish_executed_inputs()