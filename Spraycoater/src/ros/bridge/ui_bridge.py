# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
import threading
from typing import Optional, List

from .runner import RosBridge
from .scene_bridge import SceneBridge  # nur für Typzugriff

_LOG = logging.getLogger("ros.ui_bridge")


class SceneState:
    """
    Leichter, signal-freier State-Container:
      - hält die zuletzt bekannten Werte aus der SceneBridge
      - thread-sicher per RLock
    """
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._cage_list: List[str] = []
        self._mount_list: List[str] = []
        self._substrate_list: List[str] = []
        self._cage_current: str = ""
        self._mount_current: str = ""
        self._substrate_current: str = ""

    # ------- Setter (intern, nur UIBridge nutzt die) -------
    def _set_cage_list(self, items: List[str]) -> None:
        with self._lock:
            self._cage_list = list(items or [])

    def _set_mount_list(self, items: List[str]) -> None:
        with self._lock:
            self._mount_list = list(items or [])

    def _set_substrate_list(self, items: List[str]) -> None:
        with self._lock:
            self._substrate_list = list(items or [])

    def _set_cage_current(self, v: str) -> None:
        with self._lock:
            self._cage_current = (v or "").strip()

    def _set_mount_current(self, v: str) -> None:
        with self._lock:
            self._mount_current = (v or "").strip()

    def _set_substrate_current(self, v: str) -> None:
        with self._lock:
            self._substrate_current = (v or "").strip()

    # ------- Getter (öffentlich) -------
    def cage_list(self) -> List[str]:
        with self._lock:
            return list(self._cage_list)

    def mount_list(self) -> List[str]:
        with self._lock:
            return list(self._mount_list)

    def substrate_list(self) -> List[str]:
        with self._lock:
            return list(self._substrate_list)

    def cage_current(self) -> str:
        with self._lock:
            return self._cage_current

    def mount_current(self) -> str:
        with self._lock:
            return self._mount_current

    def substrate_current(self) -> str:
        with self._lock:
            return self._substrate_current


class UIBridge:
    """
    Dünne UI-Bridge:
      - Start/Stop RosBridge
      - Cacht den Scene-Zustand in `self.scene` (SceneState, signal-frei)
      - Veröffentlicht Set-Methoden (set_cage/mount/substrate),
        die die ROS-Bridge (SceneBridge) nutzen
    """

    def __init__(self, startup_yaml_path: Optional[str] = None):
        self._startup_yaml = startup_yaml_path or os.environ.get("SC_STARTUP_YAML") or ""
        self._bridge: Optional[RosBridge] = None
        self.scene = SceneState()  # nur Werte; keine Qt-Signale
        self._sb: Optional[SceneBridge] = None  # gehaltene SceneBridge-Referenz

    # ---------- Lifecycle ----------

    @property
    def is_connected(self) -> bool:
        return self._bridge is not None and self._sb is not None

    @property
    def node_name(self) -> str:
        if self._bridge and self._bridge.primary_node:
            try:
                return self._bridge.primary_node.get_name()  # type: ignore[attr-defined]
            except Exception:
                pass
        return "ui_bridge"

    def ensure_connected(self) -> None:
        """Idempotent verbinden."""
        if self.is_connected:
            return
        self.connect()

    def connect(self) -> None:
        if self._bridge is not None:
            # bereits verbunden (oder im Aufbau) -> sicherstellen, dass _sb gesetzt ist
            if self._sb is None:
                self._sb = self._bridge.get_node(SceneBridge)
                if self._sb is not None:
                    self._wire_scene_into_state(self._sb)
            return

        if not self._startup_yaml:
            raise RuntimeError("SC_STARTUP_YAML nicht gesetzt/übergeben.")

        self._bridge = RosBridge(self._startup_yaml)
        self._bridge.start()

        sb: Optional[SceneBridge] = self._bridge.get_node(SceneBridge)
        if sb is None:
            raise RuntimeError("SceneBridge nicht gefunden – wurde sie gestartet?")
        self._sb = sb

        # Qt-Signale der SceneBridge -> in unseren State spiegeln
        self._wire_scene_into_state(sb)

        # Optional: initiale Werte sofort holen, wenn SceneBridge so etwas anbieten kann
        try:
            # Falls du in SceneBridge eine reemit_cached() o.ä. hast, nutze sie:
            if hasattr(sb, "signals") and hasattr(sb.signals, "reemit_cached"):
                sb.signals.reemit_cached()  # damit Combos sofort Werte haben
        except Exception:
            pass

        _LOG.info("UIBridge connected (node=%s) – scene state ready", self.node_name)

    def disconnect(self) -> None:
        _LOG.info("UIBridge disconnect()")
        self._sb = None
        if self._bridge is None:
            return
        try:
            self._bridge.stop()
        finally:
            self._bridge = None

    # Alias für MainWindow.closeEvent
    def shutdown(self) -> None:
        self.disconnect()

    # ---------- Scene: Set-API (UI -> ROS) ----------
    def set_cage(self, name: str) -> None:
        """Setzt das Cage-Mesh über die SceneBridge."""
        if not self._sb:
            _LOG.error("set_cage: SceneBridge nicht verfügbar.")
            return
        try:
            self._sb.set_cage(name)
        except Exception as e:
            _LOG.error("set_cage failed: %s", e)

    def set_mount(self, name: str) -> None:
        if not self._sb:
            _LOG.error("set_mount: SceneBridge nicht verfügbar.")
            return
        try:
            self._sb.set_mount(name)
        except Exception as e:
            _LOG.error("set_mount failed: %s", e)

    def set_substrate(self, name: str) -> None:
        if not self._sb:
            _LOG.error("set_substrate: SceneBridge nicht verfügbar.")
            return
        try:
            self._sb.set_substrate(name)
        except Exception as e:
            _LOG.error("set_substrate failed: %s", e)

    # ---------- Intern: Wiring von Qt-Signalen -> SceneState ----------
    def _wire_scene_into_state(self, sb: SceneBridge) -> None:
        """Verbindet die Qt-Signale der SceneBridge mit unserem SceneState-Cache."""
        sig = sb.signals

        # Listen
        sig.cageListChanged.connect(lambda items: self.scene._set_cage_list(items))
        sig.mountListChanged.connect(lambda items: self.scene._set_mount_list(items))
        sig.substrateListChanged.connect(lambda items: self.scene._set_substrate_list(items))

        # Currents
        sig.cageCurrentChanged.connect(lambda v: self.scene._set_cage_current(v))
        sig.mountCurrentChanged.connect(lambda v: self.scene._set_mount_current(v))
        sig.substrateCurrentChanged.connect(lambda v: self.scene._set_substrate_current(v))


def get_ui_bridge(_ctx) -> UIBridge:
    """
    Factory für die Startup-FSM:
      - Erstellt die Bridge
      - Verbindet sofort (idempotent)
    """
    b = UIBridge()
    b.ensure_connected()
    return b
