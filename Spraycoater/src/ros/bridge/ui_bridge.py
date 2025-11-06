# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
import threading
from typing import Optional, List

from .runner import RosBridge
from .scene_bridge import SceneBridge
from .poses_bridge import PosesBridge
from .spray_path_bridge import SprayPathBridge

from geometry_msgs.msg import PoseStamped, PoseArray
from visualization_msgs.msg import MarkerArray

_LOG = logging.getLogger("ros.ui_bridge")


# ------------------------------ State: Scene ------------------------------

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


# ------------------------------ State: Poses ------------------------------

class PosesState:
    """
    Signal-freier State-Container für Poses:
      - hält Home/Service (PoseStamped) exakt wie empfangen (oder None)
      - thread-sicher per RLock
    """
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._home: Optional[PoseStamped] = None
        self._service: Optional[PoseStamped] = None

    # Setter
    def _set_home(self, msg: PoseStamped) -> None:
        with self._lock:
            self._home = msg

    def _set_service(self, msg: PoseStamped) -> None:
        with self._lock:
            self._service = msg

    # Getter
    def home(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._home

    def service(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._service


# ------------------------------ State: SprayPath ------------------------------

class SprayPathState:
    """
    Signal-freier State-Container für SprayPath:
      - current_name (str), poses (PoseArray|None), marker wird im Bridge-Node gehalten
      - thread-sicher per RLock
    """
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._current_name: str = ""
        self._poses: Optional[PoseArray] = None

    # Setter
    def _set_current_name(self, name: str) -> None:
        with self._lock:
            self._current_name = name or ""

    def _set_poses(self, pa: PoseArray) -> None:
        with self._lock:
            self._poses = pa

    # Getter
    def current_name(self) -> str:
        with self._lock:
            return self._current_name

    def poses(self) -> Optional[PoseArray]:
        with self._lock:
            return self._poses


# ============================== UIBridge ==============================

class UIBridge:
    """
    Dünne UI-Bridge:
      - Start/Stop RosBridge
      - Cacht Zustände in `scene`, `poses`, `spraypath` (signal-frei)
      - Stellt Set-APIs bereit (UI -> ROS) via Bridge-Knoten
    """

    def __init__(self, startup_yaml_path: Optional[str] = None):
        self._startup_yaml = startup_yaml_path or os.environ.get("SC_STARTUP_YAML") or ""
        self._bridge: Optional[RosBridge] = None

        # States (signal-frei)
        self.scene = SceneState()
        self.poses = PosesState()
        self.spraypath = SprayPathState()

        # gehaltene Bridge-Referenzen
        self._sb: Optional[SceneBridge] = None
        self._pb: Optional[PosesBridge] = None
        self._spb: Optional[SprayPathBridge] = None

    # ---------- Lifecycle ----------

    @property
    def is_connected(self) -> bool:
        return (
            self._bridge is not None
            and self._sb is not None
            and self._pb is not None
            and self._spb is not None
        )

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
            # bereits verbunden (oder im Aufbau) -> sicherstellen, dass Bridges gesetzt sind
            self._ensure_subnodes_and_wiring()
            return

        if not self._startup_yaml:
            raise RuntimeError("SC_STARTUP_YAML nicht gesetzt/übergeben.")

        self._bridge = RosBridge(self._startup_yaml)
        self._bridge.start()

        # Subnodes holen + Signale verbinden
        self._ensure_subnodes_and_wiring()

        # Optional: gecachte Werte re-emittieren (falls vorhanden)
        self._try_reemit_cached()

        _LOG.info("UIBridge connected (node=%s) – scene/poses/spraypath states ready", self.node_name)

    def _ensure_subnodes_and_wiring(self) -> None:
        if self._bridge is None:
            raise RuntimeError("RosBridge nicht aktiv.")

        # Scene
        if self._sb is None:
            sb = self._bridge.get_node(SceneBridge)
            if sb is None:
                raise RuntimeError("SceneBridge nicht gefunden – wurde sie gestartet?")
            self._sb = sb
            self._wire_scene_into_state(sb)

        # Poses
        if self._pb is None:
            pb = self._bridge.get_node(PosesBridge)
            if pb is None:
                raise RuntimeError("PosesBridge nicht gefunden – wurde sie gestartet?")
            self._pb = pb
            self._wire_poses_into_state(pb)

        # SprayPath
        if self._spb is None:
            spb = self._bridge.get_node(SprayPathBridge)
            if spb is None:
                raise RuntimeError("SprayPathBridge nicht gefunden – wurde sie gestartet?")
            self._spb = spb
            self._wire_spraypath_into_state(spb)

    def _try_reemit_cached(self) -> None:
        # Falls die Bridges eine reemit_cached()-Hilfsfunktion besitzen, nutzen wir sie.
        for b in (self._sb, self._pb, self._spb):
            try:
                if b is not None and hasattr(b, "signals") and hasattr(b.signals, "reemit_cached"):
                    b.signals.reemit_cached()
            except Exception:
                pass

    def disconnect(self) -> None:
        _LOG.info("UIBridge disconnect()")
        self._sb = None
        self._pb = None
        self._spb = None
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

    # ---------- Poses: Set-API (UI -> ROS) ----------
    def set_pose_by_name(self, name: str) -> None:
        """Publisht exakt 'home' oder 'service' über PosesBridge."""
        if not self._pb:
            _LOG.error("set_pose_by_name: PosesBridge nicht verfügbar.")
            return
        try:
            self._pb.set_pose_by_name(name)
        except Exception as e:
            _LOG.error("set_pose_by_name failed: %s", e)

    def set_home(self) -> None:
        self.set_pose_by_name("home")

    def set_service(self) -> None:
        self.set_pose_by_name("service")

    # ---------- SprayPath: Set-API (UI -> ROS) ----------
    def set_spraypath(self, marker_array: MarkerArray) -> None:
        """Publisht ein MarkerArray unverändert auf spraypath.set."""
        if not self._spb:
            _LOG.error("set_spraypath: SprayPathBridge nicht verfügbar.")
            return
        try:
            self._spb.publish_set(marker_array)
        except Exception as e:
            _LOG.error("set_spraypath failed: %s", e)

    # ---------- Intern: Wiring von Qt-Signalen -> States ----------
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

    def _wire_poses_into_state(self, pb: PosesBridge) -> None:
        """Verbindet die Qt-Signale der PosesBridge mit unserem PosesState-Cache."""
        sig = pb.signals
        sig.homePoseChanged.connect(lambda msg: self.poses._set_home(msg))
        sig.servicePoseChanged.connect(lambda msg: self.poses._set_service(msg))

    def _wire_spraypath_into_state(self, spb: SprayPathBridge) -> None:
        """Verbindet die Qt-Signale der SprayPathBridge mit unserem SprayPathState-Cache."""
        sig = spb.signals
        sig.currentNameChanged.connect(lambda name: self.spraypath._set_current_name(name))
        sig.posesChanged.connect(lambda pa: self.spraypath._set_poses(pa))


def get_ui_bridge(_ctx) -> UIBridge:
    """
    Factory für die Startup-FSM:
      - Erstellt die Bridge
      - Verbindet sofort (idempotent)
    """
    b = UIBridge()
    b.ensure_connected()
    return b
