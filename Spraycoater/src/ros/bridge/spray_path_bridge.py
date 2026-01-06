# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from typing import Any, Mapping, Optional

from PyQt6 import QtCore
from std_msgs.msg import Bool as MsgBool

from config.startup import AppContent
from .base_bridge import BaseBridge

try:
    from visualization_msgs.msg import MarkerArray
except Exception:  # pragma: no cover
    MarkerArray = Any  # type: ignore

_LOG = logging.getLogger("ros.bridge.spray_path_bridge")


class SprayPathSignals(QtCore.QObject):
    # GUI -> Bridge (requests)
    showCompiledRequested = QtCore.pyqtSignal(bool)
    showTrajRequested = QtCore.pyqtSignal(bool)
    showExecutedRequested = QtCore.pyqtSignal(bool)

    # Bridge -> GUI (optional availability)
    compiledAvailableChanged = QtCore.pyqtSignal(bool)
    trajAvailableChanged = QtCore.pyqtSignal(bool)
    executedAvailableChanged = QtCore.pyqtSignal(bool)

    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)

        # cached flags (optional)
        self._last_show_compiled: bool = True
        self._last_show_traj: bool = True
        self._last_show_executed: bool = True

        self._last_avail_compiled: bool = False
        self._last_avail_traj: bool = False
        self._last_avail_executed: bool = False

    @QtCore.pyqtSlot()
    def reemit_cached(self) -> None:
        # re-emit availability state (UI friendliness)
        self.compiledAvailableChanged.emit(bool(self._last_avail_compiled))
        self.trajAvailableChanged.emit(bool(self._last_avail_traj))
        self.executedAvailableChanged.emit(bool(self._last_avail_executed))


class SprayPathBridge(BaseBridge):
    """
    UI bridge for spray_path marker layers.

    SSoT:
      - topic IDs come from topics.yaml group 'spray_path'
      - QoS from qos.yaml

    It publishes marker layers (latched) and also publishes the show_* toggles.
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
        self._show_traj = True
        self._show_executed = True

        self._avail_compiled = False
        self._avail_traj = False
        self._avail_executed = False

        self._last_compiled: Optional[MarkerArray] = None
        self._last_traj: Optional[MarkerArray] = None
        self._last_executed: Optional[MarkerArray] = None

        super().__init__("spray_path_bridge", content, namespace=namespace, config=config)

        cfg = dict(config or {})

        # IDs from topics.yaml (defaults)
        id_compiled_markers = str(cfg.get("id_compiled_markers", "compiled_markers"))
        id_traj_markers = str(cfg.get("id_traj_markers", "traj_markers"))
        id_executed_markers = str(cfg.get("id_executed_markers", "executed_markers"))

        # Toggle IDs from topics.yaml subscribe-spec (UI -> Node)
        id_show_compiled = str(cfg.get("id_show_compiled", "show_compiled"))
        id_show_traj = str(cfg.get("id_show_traj", "show_traj"))
        id_show_executed = str(cfg.get("id_show_executed", "show_executed"))

        # Publishers for marker layers (publish-spec of spray_path group)
        self._pub_compiled = None
        self._pub_traj = None
        self._pub_executed = None

        try:
            self._pub_compiled = content.create_publisher_from_id(self, self.GROUP, id_compiled_markers)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_compiled_markers}): {e}")

        try:
            self._pub_traj = content.create_publisher_from_id(self, self.GROUP, id_traj_markers)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_traj_markers}): {e}")

        try:
            self._pub_executed = content.create_publisher_from_id(self, self.GROUP, id_executed_markers)
        except Exception as e:
            self.get_logger().error(f"[spray_path] publisher init failed ({id_executed_markers}): {e}")

        # Publishers for show toggles (these are in subscribe-spec -> BaseBridge already created pubs)
        self._id_show_compiled = id_show_compiled
        self._id_show_traj = id_show_traj
        self._id_show_executed = id_show_executed

        # Qt wiring (robust: lambda avoids signature-mismatch issues)
        self.signals.showCompiledRequested.connect(lambda v: self._on_show_compiled(bool(v)))
        self.signals.showTrajRequested.connect(lambda v: self._on_show_traj(bool(v)))
        self.signals.showExecutedRequested.connect(lambda v: self._on_show_executed(bool(v)))

        # publish initial toggles + empty markers
        self._publish_toggle(self._id_show_compiled, True)
        self._publish_toggle(self._id_show_traj, True)
        self._publish_toggle(self._id_show_executed, True)
        self._republish_all()

    # ---------------- toggles (GUI -> Bridge) ----------------

    def _publish_toggle(self, topic_id: str, v: bool) -> None:
        try:
            self.pub(topic_id).publish(MsgBool(data=bool(v)))
        except Exception as e:
            # not fatal; markers still work locally
            self.get_logger().debug(f"[spray_path] toggle publish failed id={topic_id}: {e}")

    def _on_show_compiled(self, v: bool) -> None:
        self._show_compiled = bool(v)
        self.signals._last_show_compiled = self._show_compiled
        self._publish_toggle(self._id_show_compiled, self._show_compiled)
        self._republish_compiled()

    def _on_show_traj(self, v: bool) -> None:
        self._show_traj = bool(v)
        self.signals._last_show_traj = self._show_traj
        self._publish_toggle(self._id_show_traj, self._show_traj)
        self._republish_traj()

    def _on_show_executed(self, v: bool) -> None:
        self._show_executed = bool(v)
        self.signals._last_show_executed = self._show_executed
        self._publish_toggle(self._id_show_executed, self._show_executed)
        self._republish_executed()

    # ---------------- public API ----------------

    def set_compiled(self, markers: MarkerArray) -> None:
        self._last_compiled = markers
        self._set_avail("compiled", self._has_markers(markers))
        self._republish_compiled()

    def set_traj(self, markers: MarkerArray) -> None:
        self._last_traj = markers
        self._set_avail("traj", self._has_markers(markers))
        self._republish_traj()

    def set_executed(self, markers: MarkerArray) -> None:
        self._last_executed = markers
        self._set_avail("executed", self._has_markers(markers))
        self._republish_executed()

    # ---------------- helpers ----------------

    def _has_markers(self, msg: Any) -> bool:
        try:
            m = getattr(msg, "markers", None)
            return bool(m) and isinstance(m, list)
        except Exception:
            return False

    def _set_avail(self, which: str, v: bool) -> None:
        v = bool(v)
        if which == "compiled":
            if v != self._avail_compiled:
                self._avail_compiled = v
                self.signals._last_avail_compiled = v
                self.signals.compiledAvailableChanged.emit(v)
        elif which == "traj":
            if v != self._avail_traj:
                self._avail_traj = v
                self.signals._last_avail_traj = v
                self.signals.trajAvailableChanged.emit(v)
        elif which == "executed":
            if v != self._avail_executed:
                self._avail_executed = v
                self.signals._last_avail_executed = v
                self.signals.executedAvailableChanged.emit(v)

    def _empty(self) -> MarkerArray:
        try:
            return MarkerArray()
        except Exception:
            return {}  # type: ignore[return-value]

    def _republish_compiled(self) -> None:
        if self._pub_compiled is None:
            return
        try:
            self._pub_compiled.publish(self._last_compiled if (self._show_compiled and self._last_compiled is not None) else self._empty())
        except Exception:
            _LOG.exception("SprayPathBridge: republish_compiled failed")

    def _republish_traj(self) -> None:
        if self._pub_traj is None:
            return
        try:
            self._pub_traj.publish(self._last_traj if (self._show_traj and self._last_traj is not None) else self._empty())
        except Exception:
            _LOG.exception("SprayPathBridge: republish_traj failed")

    def _republish_executed(self) -> None:
        if self._pub_executed is None:
            return
        try:
            self._pub_executed.publish(self._last_executed if (self._show_executed and self._last_executed is not None) else self._empty())
        except Exception:
            _LOG.exception("SprayPathBridge: republish_executed failed")

    def _republish_all(self) -> None:
        self._republish_compiled()
        self._republish_traj()
        self._republish_executed()
