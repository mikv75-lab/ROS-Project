# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional

from .runner import RosBridge
from .scene_bridge import SceneBridge  # für Typzugriff

_LOG = logging.getLogger("ros.ui_bridge")


class UIBridge:
    """
    Dünne UI-Bridge:
      - Start/Stop RosBridge
      - Exponiert die Qt-Signale des SceneBridge-Nodes direkt als `bridge.scene`
        => ServiceTab kann bridge.scene.cageListChanged etc. verwenden
      - Set-Requests laufen ebenfalls über diese Signals (SceneSignals.*Requested)
    """

    def __init__(self, startup_yaml_path: Optional[str] = None):
        self._startup_yaml = startup_yaml_path or os.environ.get("SC_STARTUP_YAML") or ""
        self._bridge: Optional[RosBridge] = None

        # Wird in connect() gesetzt:
        self.scene = None  # Typ: SceneSignals

    # ---------- Lifecycle ----------

    @property
    def is_connected(self) -> bool:
        return self._bridge is not None and self.scene is not None

    @property
    def node_name(self) -> str:
        if self._bridge and self._bridge.primary_node:
            try:
                return self._bridge.primary_node.get_name()  # type: ignore[attr-defined]
            except Exception:
                pass
        return "ui_bridge"

    def connect(self) -> None:
        if self._bridge is not None:
            return
        if not self._startup_yaml:
            raise RuntimeError("SC_STARTUP_YAML nicht gesetzt/übergeben.")

        self._bridge = RosBridge(self._startup_yaml)
        self._bridge.start()

        # Hole den laufenden SceneBridge-Node und exponiere seine Qt-Signale
        scene_node: Optional[SceneBridge] = self._bridge.get_node(SceneBridge)
        if scene_node is None:
            raise RuntimeError("SceneBridge nicht gefunden – wurde sie gestartet?")
        self.scene = scene_node.signals

        _LOG.info("UIBridge connected (node=%s) – scene signals ready", self.node_name)

    def disconnect(self) -> None:
        _LOG.info("UIBridge disconnect()")
        self.scene = None
        if self._bridge is None:
            return
        try:
            self._bridge.stop()
        finally:
            self._bridge = None

    # Alias für MainWindow.closeEvent
    def shutdown(self) -> None:
        self.disconnect()


def get_ui_bridge(_ctx) -> UIBridge:
    # Factory für die Startup-FSM
    return UIBridge()
