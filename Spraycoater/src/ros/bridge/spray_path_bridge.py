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
    # Compiled bleibt global (ein Toggle)
    showCompiledRequested = QtCore.pyqtSignal(bool)

    # Planned/Executed split (Run vs Stored)
    showPlannedRunRequested = QtCore.pyqtSignal(bool)
    showPlannedStoredRequested = QtCore.pyqtSignal(bool)
    showExecutedRunRequested = QtCore.pyqtSignal(bool)
    showExecutedStoredRequested = QtCore.pyqtSignal(bool)

    # Optional / backward compatible master toggles (affect both)
    showPlannedRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)

    clearRequested = QtCore.pyqtSignal()

    # ---------------- Bridge -> GUI (availability) ----------------
    # Aggregate availability (old behavior)
    compiledAvailableChanged = QtCore.pyqtSignal(bool)
    plannedAvailableChanged = QtCore.pyqtSignal(bool)
    executedAvailableChanged = QtCore.pyqtSignal(bool)

    # Split availability (new behavior)
    plannedRunAvailableChanged = QtCore.pyqtSignal(bool)
    plannedStoredAvailableChanged = QtCore.pyqtSignal(bool)
    executedRunAvailableChanged = QtCore.pyqtSignal(bool)
    executedStoredAvailableChanged = QtCore.pyqtSignal(bool)

    # Legacy alias support (planned aggregate)
    trajAvailableChanged = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # cached availability for immediate UI state
        self._last_avail_compiled: bool = False
        self._last_avail_planned: bool = False
        self._last_avail_executed: bool = False

        self._last_avail_planned_run: bool = False
        self._last_avail_planned_stored: bool = False
        self._last_avail_executed_run: bool = False
        self._last_avail_executed_stored: bool = False

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        self.compiledAvailableChanged.emit(bool(self._last_avail_compiled))

        self.plannedAvailableChanged.emit(bool(self._last_avail_planned))
        self.trajAvailableChanged.emit(bool(self._last_avail_planned))  # Legacy alias

        self.executedAvailableChanged.emit(bool(self._last_avail_executed))

        self.plannedRunAvailableChanged.emit(bool(self._last_avail_planned_run))
        self.plannedStoredAvailableChanged.emit(bool(self._last_avail_planned_stored))
        self.executedRunAvailableChanged.emit(bool(self._last_avail_executed_run))
        self.executedStoredAvailableChanged.emit(bool(self._last_avail_executed_stored))


class SprayPathBridge(BaseBridge):
    """
    STRICT bridge for spray_path cache node (2026-01):

    Uses separate topics for New (Run) vs Stored (Ghosting).
    Compiled stays global (one toggle).

      subscribe (GUI -> node):
        clear
        show_compiled

        show_planned_run, show_planned_stored
        show_executed_run, show_executed_stored

        Optional backward compatible masters:
          show_planned (sets both planned_run + planned_stored)
          show_executed (sets both executed_run + executed_stored)

        compiled_poses_in, compiled_markers_in
        planned_new_poses_in, planned_new_markers_in, planned_stored_markers_in
        executed_new_poses_in, executed_new_markers_in, executed_stored_markers_in
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

        self._show_planned_run = True
        self._show_planned_stored = True

        self._show_executed_run = True
        self._show_executed_stored = True

        # local state (availability)
        self._avail_compiled = False

        self._avail_planned = False
        self._avail_planned_run = False
        self._avail_planned_stored = False

        self._avail_executed = False
        self._avail_executed_run = False
        self._avail_executed_stored = False

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

        cfg = dict(config or {})

        # Toggles + Clear
        self._id_clear = str(cfg.get("id_clear", "clear")).strip()
        self._id_show_compiled = str(cfg.get("id_show_compiled", "show_compiled")).strip()

        # Optional master toggles (backward compatible)
        self._id_show_planned = str(cfg.get("id_show_planned", "show_planned")).strip()
        self._id_show_executed = str(cfg.get("id_show_executed", "show_executed")).strip()

        # Split toggles (new)
        self._id_show_planned_run = str(cfg.get("id_show_planned_run", "show_planned_run")).strip()
        self._id_show_planned_stored = str(cfg.get("id_show_planned_stored", "show_planned_stored")).strip()
        self._id_show_executed_run = str(cfg.get("id_show_executed_run", "show_executed_run")).strip()
        self._id_show_executed_stored = str(cfg.get("id_show_executed_stored", "show_executed_stored")).strip()

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

        # Qt wiring (new split)
        self.signals.showCompiledRequested.connect(lambda v: self._on_show_compiled(bool(v)))

        self.signals.showPlannedRunRequested.connect(lambda v: self._on_show_planned_run(bool(v)))
        self.signals.showPlannedStoredRequested.connect(lambda v: self._on_show_planned_stored(bool(v)))
        self.signals.showExecutedRunRequested.connect(lambda v: self._on_show_executed_run(bool(v)))
        self.signals.showExecutedStoredRequested.connect(lambda v: self._on_show_executed_stored(bool(v)))

        # Qt wiring (optional master)
        self.signals.showPlannedRequested.connect(lambda v: self._on_show_planned_master(bool(v)))
        self.signals.showExecutedRequested.connect(lambda v: self._on_show_executed_master(bool(v)))

        self.signals.clearRequested.connect(self.clear)

        # publish initial toggles (latched)
        self._publish_toggle(self._id_show_compiled, True)

        # publish split toggles (preferred)
        self._publish_toggle(self._id_show_planned_run, True)
        self._publish_toggle(self._id_show_planned_stored, True)
        self._publish_toggle(self._id_show_executed_run, True)
        self._publish_toggle(self._id_show_executed_stored, True)

        # optional masters set to True for backwards compat (do NOT publish masters on split changes)
        self._publish_toggle(self._id_show_planned, True)
        self._publish_toggle(self._id_show_executed, True)

        # push cached inputs once
        self._republish_all_inputs()

    # ---------------- toggles + clear (GUI -> node) ----------------

    def _publish_toggle(self, topic_id: str, v: bool) -> None:
        if not topic_id:
            return
        try:
            self.pub(topic_id).publish(MsgBool(data=bool(v)))
        except Exception:
            # bridge should not explode on missing publisher; strict behavior is handled upstream
            pass

    def clear(self) -> None:
        """Tell spray_path node to reset caches + outputs."""
        try:
            self.pub(self._id_clear).publish(MsgEmpty())
        except Exception as e:
            self.get_logger().warning(f"[spray_path] clear failed: {e}")

    def _on_show_compiled(self, v: bool) -> None:
        self._show_compiled = bool(v)
        self._publish_toggle(self._id_show_compiled, self._show_compiled)
        self._republish_compiled_inputs()

    # ---- Planned/Executed: master toggles (backward compatible) ----

    def _on_show_planned_master(self, v: bool) -> None:
        """
        Master semantics: set BOTH planned_run + planned_stored and publish master topic.
        Note: does NOT publish split topics here (UI should call split signals ideally),
        but we DO republish both inputs according to the updated local flags.
        """
        vv = bool(v)
        self._show_planned_run = vv
        self._show_planned_stored = vv
        self._publish_toggle(self._id_show_planned, vv)
        self._republish_planned_inputs()

    def _on_show_executed_master(self, v: bool) -> None:
        vv = bool(v)
        self._show_executed_run = vv
        self._show_executed_stored = vv
        self._publish_toggle(self._id_show_executed, vv)
        self._republish_executed_inputs()

    # ---- Planned: split toggles ----

    def _on_show_planned_run(self, v: bool) -> None:
        self._show_planned_run = bool(v)
        self._publish_toggle(self._id_show_planned_run, self._show_planned_run)
        # do NOT publish master here (would break split semantics in node)
        self._republish_planned_inputs()

    def _on_show_planned_stored(self, v: bool) -> None:
        self._show_planned_stored = bool(v)
        self._publish_toggle(self._id_show_planned_stored, self._show_planned_stored)
        self._republish_planned_inputs()

    # ---- Executed: split toggles ----

    def _on_show_executed_run(self, v: bool) -> None:
        self._show_executed_run = bool(v)
        self._publish_toggle(self._id_show_executed_run, self._show_executed_run)
        self._republish_executed_inputs()

    def _on_show_executed_stored(self, v: bool) -> None:
        self._show_executed_stored = bool(v)
        self._publish_toggle(self._id_show_executed_stored, self._show_executed_stored)
        self._republish_executed_inputs()

    # ---------------- public API (ProcessTab publisher side) ----------------

    def set_compiled(self, *, poses: Optional[PoseArray] = None, markers: Optional[MarkerArray] = None) -> None:
        if poses is not None:
            self._last_compiled_poses = poses
        if markers is not None:
            self._last_compiled_markers = markers
        self._update_avail()
        self._republish_compiled_inputs()

    def set_planned(
        self,
        *,
        new_poses: Optional[PoseArray] = None,
        new_markers: Optional[MarkerArray] = None,
        stored_markers: Optional[MarkerArray] = None,
    ) -> None:
        """
        Set Planned Layer data.
        - new_poses/new_markers: Run data
        - stored_markers: Ghost data from disk
        """
        if new_poses is not None:
            self._last_planned_new_poses = new_poses
        if new_markers is not None:
            self._last_planned_new_markers = new_markers
        if stored_markers is not None:
            self._last_planned_stored_markers = stored_markers

        self._update_avail()
        self._republish_planned_inputs()

    def set_executed(
        self,
        *,
        new_poses: Optional[PoseArray] = None,
        new_markers: Optional[MarkerArray] = None,
        stored_markers: Optional[MarkerArray] = None,
    ) -> None:
        """
        Set Executed Layer data.
        - new_poses/new_markers: Run data
        - stored_markers: Ghost data from disk
        """
        if new_poses is not None:
            self._last_executed_new_poses = new_poses
        if new_markers is not None:
            self._last_executed_new_markers = new_markers
        if stored_markers is not None:
            self._last_executed_stored_markers = stored_markers

        self._update_avail()
        self._republish_executed_inputs()

    # ---------------- helpers ----------------

    def _update_avail(self) -> None:
        # compiled
        v_compiled = self._has_any(self._last_compiled_poses, self._last_compiled_markers)
        if v_compiled != self._avail_compiled:
            self._avail_compiled = v_compiled
            self.signals._last_avail_compiled = v_compiled
            self.signals.compiledAvailableChanged.emit(v_compiled)

        # planned split
        v_planned_run = self._has_any(self._last_planned_new_poses, self._last_planned_new_markers)
        v_planned_stored = self._has_markers(self._last_planned_stored_markers)
        v_planned_any = bool(v_planned_run or v_planned_stored)

        if v_planned_run != self._avail_planned_run:
            self._avail_planned_run = v_planned_run
            self.signals._last_avail_planned_run = v_planned_run
            self.signals.plannedRunAvailableChanged.emit(v_planned_run)

        if v_planned_stored != self._avail_planned_stored:
            self._avail_planned_stored = v_planned_stored
            self.signals._last_avail_planned_stored = v_planned_stored
            self.signals.plannedStoredAvailableChanged.emit(v_planned_stored)

        if v_planned_any != self._avail_planned:
            self._avail_planned = v_planned_any
            self.signals._last_avail_planned = v_planned_any
            self.signals.plannedAvailableChanged.emit(v_planned_any)
            self.signals.trajAvailableChanged.emit(v_planned_any)  # legacy alias

        # executed split
        v_executed_run = self._has_any(self._last_executed_new_poses, self._last_executed_new_markers)
        v_executed_stored = self._has_markers(self._last_executed_stored_markers)
        v_executed_any = bool(v_executed_run or v_executed_stored)

        if v_executed_run != self._avail_executed_run:
            self._avail_executed_run = v_executed_run
            self.signals._last_avail_executed_run = v_executed_run
            self.signals.executedRunAvailableChanged.emit(v_executed_run)

        if v_executed_stored != self._avail_executed_stored:
            self._avail_executed_stored = v_executed_stored
            self.signals._last_avail_executed_stored = v_executed_stored
            self.signals.executedStoredAvailableChanged.emit(v_executed_stored)

        if v_executed_any != self._avail_executed:
            self._avail_executed = v_executed_any
            self.signals._last_avail_executed = v_executed_any
            self.signals.executedAvailableChanged.emit(v_executed_any)

    def _has_any(self, poses: Any, markers: Any) -> bool:
        return self._has_poses(poses) or self._has_markers(markers)

    def _has_poses(self, msg: Any) -> bool:
        try:
            return bool(msg and hasattr(msg, "poses") and len(msg.poses) > 0)
        except Exception:
            return False

    def _has_markers(self, msg: Any) -> bool:
        try:
            return bool(msg and hasattr(msg, "markers") and len(msg.markers) > 0)
        except Exception:
            return False

    # ---------------- (re)publish to node inputs ----------------

    def _pub_if(self, topic_id: str, msg: Any, enable: bool, empty_factory) -> None:
        try:
            if not topic_id:
                return
            val = msg if (enable and msg is not None) else empty_factory()
            self.pub(topic_id).publish(val)
        except Exception:
            _LOG.exception("SprayPathBridge: publish failed for %s", topic_id)

    def _republish_compiled_inputs(self) -> None:
        self._pub_if(self._id_cp_in, self._last_compiled_poses, self._show_compiled, PoseArray)
        self._pub_if(self._id_cm_in, self._last_compiled_markers, self._show_compiled, MarkerArray)

    def _republish_planned_inputs(self) -> None:
        # Run slots
        self._pub_if(self._id_pnp_in, self._last_planned_new_poses, self._show_planned_run, PoseArray)
        self._pub_if(self._id_pnm_in, self._last_planned_new_markers, self._show_planned_run, MarkerArray)

        # Stored slot
        self._pub_if(self._id_psm_in, self._last_planned_stored_markers, self._show_planned_stored, MarkerArray)

    def _republish_executed_inputs(self) -> None:
        # Run slots
        self._pub_if(self._id_enp_in, self._last_executed_new_poses, self._show_executed_run, PoseArray)
        self._pub_if(self._id_enm_in, self._last_executed_new_markers, self._show_executed_run, MarkerArray)

        # Stored slot
        self._pub_if(self._id_esm_in, self._last_executed_stored_markers, self._show_executed_stored, MarkerArray)

    def _republish_all_inputs(self) -> None:
        self._republish_compiled_inputs()
        self._republish_planned_inputs()
        self._republish_executed_inputs()
