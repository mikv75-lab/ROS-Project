# src/ros/bridge/ui_bridge.py
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
from .servo_bridge import ServoBridge
from .robot_bridge import RobotBridge
from .moveitpy_bridge import MoveItPyBridge
from .omron_bridge import OmronBridge  # optional

from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

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
      - current_name (str)
      - poses (PoseArray|None)  [Sollpfad]
      - marker-Visualisierung passiert ausschließlich im RViz/Preview
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


# ------------------------------ State: Robot ------------------------------

class RobotState:
    """
    Signal-freier State-Container für Robot-Zustände:
      - connection/mode/initialized/moving/servo_enabled/power/estop/errors
      - tcp_pose (PoseStamped|None) im world-Frame (tool_mount in world)
      - joints (JointState|None)
      - alles thread-sicher per RLock
    """
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._connection: bool = False
        self._mode: str = "DISCONNECTED"
        self._initialized: bool = False
        self._moving: bool = False
        self._servo_enabled: bool = False
        self._power: bool = False
        self._estop: bool = False
        self._errors: str = ""
        self._tcp_pose: Optional[PoseStamped] = None
        self._joints: Optional[JointState] = None

    # Setter
    def _set_connection(self, v: bool) -> None:
        with self._lock:
            self._connection = bool(v)

    def _set_mode(self, v: str) -> None:
        with self._lock:
            self._mode = (v or "").strip() or "DISCONNECTED"

    def _set_initialized(self, v: bool) -> None:
        with self._lock:
            self._initialized = bool(v)

    def _set_moving(self, v: bool) -> None:
        with self._lock:
            self._moving = bool(v)

    def _set_servo_enabled(self, v: bool) -> None:
        with self._lock:
            self._servo_enabled = bool(v)

    def _set_power(self, v: bool) -> None:
        with self._lock:
            self._power = bool(v)

    def _set_estop(self, v: bool) -> None:
        with self._lock:
            self._estop = bool(v)

    def _set_errors(self, v: str) -> None:
        with self._lock:
            self._errors = (v or "").strip()

    def _set_tcp_pose(self, msg: Optional[PoseStamped]) -> None:
        with self._lock:
            self._tcp_pose = msg

    def _set_joints(self, msg: Optional[JointState]) -> None:
        with self._lock:
            self._joints = msg

    # Getter
    def connection(self) -> bool:
        with self._lock:
            return self._connection

    def mode(self) -> str:
        with self._lock:
            return self._mode

    def initialized(self) -> bool:
        with self._lock:
            return self._initialized

    def moving(self) -> bool:
        with self._lock:
            return self._moving

    def servo_enabled(self) -> bool:
        with self._lock:
            return self._servo_enabled

    def power(self) -> bool:
        with self._lock:
            return self._power

    def estop(self) -> bool:
        with self._lock:
            return self._estop

    def errors(self) -> str:
        with self._lock:
            return self._errors

    def tcp_pose(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._tcp_pose

    def joints(self) -> Optional[JointState]:
        with self._lock:
            return self._joints


# ------------------------------ State: Motion (Trajectory Cache) ------------------------------

class MotionState:
    """
    Signal-freier State-Container für Motion:
      - planned (RobotTrajectoryMsg|None)
      - executed (RobotTrajectoryMsg|None)
      - thread-sicher per RLock

    Dient als einfacher Cache, damit UI / ProcessTab bei Bedarf
    auf die letzte geplante/ausgeführte Trajektorie zugreifen kann.
    """
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._planned: Optional[RobotTrajectoryMsg] = None
        self._executed: Optional[RobotTrajectoryMsg] = None

    # Setter
    def _set_planned(self, msg: RobotTrajectoryMsg) -> None:
        with self._lock:
            self._planned = msg

    def _set_executed(self, msg: RobotTrajectoryMsg) -> None:
        with self._lock:
            self._executed = msg

    # Getter
    def planned(self) -> Optional[RobotTrajectoryMsg]:
        with self._lock:
            return self._planned

    def executed(self) -> Optional[RobotTrajectoryMsg]:
        with self._lock:
            return self._executed


# ============================== UIBridge ==============================

class UIBridge:
    """
    Dünne UI-Bridge:
      - Start/Stop RosBridge
      - Cacht Zustände in `scene`, `poses`, `spraypath`, `robot`, `motion` (signal-frei)
      - Stellt Set-APIs bereit (UI -> ROS) via Bridge-Knoten
      - hält zusätzlich ServoBridge in self._servo für den ServiceTab
      - hält zusätzlich RobotBridge in self._robot für Service/Status-UI
      - hält zusätzlich MoveItPyBridge in self._motion für Motion/Process-UI
      - hält zusätzlich OmronBridge in self._omron für TCP-Client-Widget (optional)
    """

    def __init__(
        self,
        startup_yaml_path: Optional[str] = None,
        namespace: str = "",
    ):
        self._startup_yaml = startup_yaml_path or os.environ.get("SC_STARTUP_YAML") or ""
        self._namespace = (namespace or "").strip().strip("/")

        self._bridge: Optional[RosBridge] = None

        # States (signal-frei)
        self.scene = SceneState()
        self.poses = PosesState()
        self.spraypath = SprayPathState()
        self.robot = RobotState()
        self.motion = MotionState()

        # gehaltene Bridge-Referenzen
        self._sb: Optional[SceneBridge] = None
        self._pb: Optional[PosesBridge] = None
        self._spb: Optional[SprayPathBridge] = None

        self._sev: Optional[ServoBridge] = None
        self._servo: Optional[ServoBridge] = None

        self._rb: Optional[RobotBridge] = None
        self._robot: Optional[RobotBridge] = None

        # MoveItPy
        self._mb: Optional[MoveItPyBridge] = None     # intern
        self._motion: Optional[MoveItPyBridge] = None # öffentliches Attribut für Widgets

        # Omron
        self._ob: Optional[OmronBridge] = None
        self._omron: Optional[OmronBridge] = None

    # ---------- Lifecycle ----------

    @property
    def namespace(self) -> str:
        """Namespace dieser UI-Bridge (z. B. 'shadow' oder 'live')."""
        return self._namespace

    @property
    def is_connected(self) -> bool:
        return (
            self._bridge is not None
            and self._sb is not None
            and self._pb is not None
            and self._spb is not None
            and self._sev is not None
            and self._rb is not None
            and self._mb is not None
        )

    @property
    def node_name(self) -> str:
        if self._bridge and self._bridge.primary_node:
            try:
                return self._bridge.primary_node.get_name()  # type: ignore[attr-defined]
            except Exception:
                pass
        ns_suffix = f"_{self._namespace}" if self._namespace else ""
        return f"ui_bridge{ns_suffix}"

    def ensure_connected(self) -> None:
        """Idempotent verbinden."""
        if self.is_connected:
            return
        self.connect()

    def connect(self) -> None:
        if self._bridge is not None:
            self._ensure_subnodes_and_wiring()
            return

        if not self._startup_yaml:
            raise RuntimeError("SC_STARTUP_YAML nicht gesetzt/übergeben.")

        self._bridge = RosBridge(
            startup_yaml_path=self._startup_yaml,
            namespace=self._namespace,
        )
        self._bridge.start()

        self._ensure_subnodes_and_wiring()
        self._try_reemit_cached()

        _LOG.info(
            "UIBridge connected (node=%s, namespace=%r) – scene/poses/spraypath/servo/robot/moveitpy/omron states ready",
            self.node_name,
            self._namespace,
        )

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

        # Servo
        if self._sev is None:
            sev = self._bridge.get_node(ServoBridge)
            if sev is None:
                raise RuntimeError("ServoBridge nicht gefunden – wurde sie gestartet?")
            self._sev = sev
            self._servo = sev

        # Robot
        if self._rb is None:
            rb = self._bridge.get_node(RobotBridge)
            if rb is None:
                raise RuntimeError("RobotBridge nicht gefunden – wurde sie gestartet?")
            self._rb = rb
            self._wire_robot_into_state(rb)
            self._robot = rb

        # MoveItPy
        if self._mb is None:
            mb = self._bridge.get_node(MoveItPyBridge)
            if mb is None:
                raise RuntimeError("MoveItPyBridge nicht gefunden – wurde sie gestartet?")
            self._mb = mb
            self._motion = mb
            self._wire_motion_into_state(mb)
            _LOG.info("UIBridge: MoveItPyBridge verdrahtet (Signals + Trajectory-Cache).")

        # Omron (optional)
        if self._ob is None:
            ob = self._bridge.get_node(OmronBridge)
            if ob is None:
                _LOG.warning("OmronBridge nicht gefunden – Omron-TCP-Widget bleibt passiv.")
            else:
                self._ob = ob
                self._omron = ob
                _LOG.info("UIBridge: OmronBridge verdrahtet (TCP-Client-Schnittstelle).")

    def _try_reemit_cached(self) -> None:
        for b in (self._sb, self._pb, self._spb, self._rb, self._mb, self._ob):
            try:
                if b is not None and hasattr(b, "signals") and hasattr(b.signals, "reemit_cached"):
                    b.signals.reemit_cached()
            except Exception:
                pass

    def disconnect(self) -> None:
        _LOG.info("UIBridge disconnect() [namespace=%r]", self._namespace)
        self._sb = None
        self._pb = None
        self._spb = None
        self._sev = None
        self._servo = None
        self._rb = None
        self._robot = None
        self._mb = None
        self._motion = None
        self._ob = None
        self._omron = None
        if self._bridge is None:
            return
        try:
            self._bridge.stop()
        finally:
            self._bridge = None

    def shutdown(self) -> None:
        self.disconnect()

    # ---------- Scene: Set-API (UI -> ROS) ----------
    def set_cage(self, name: str) -> None:
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
        if not self._spb:
            _LOG.error("set_spraypath: SprayPathBridge nicht verfügbar.")
            return
        try:
            self._spb.publish_set(marker_array)
        except Exception as e:
            _LOG.error("set_spraypath failed: %s", e)

    def set_executed_path(self, pose_array: PoseArray) -> None:
        if not self._spb:
            _LOG.error("set_executed_path: SprayPathBridge nicht verfügbar.")
            return
        try:
            self._spb.publish_executed_path(pose_array)
        except Exception as e:
            _LOG.error("set_executed_path failed: %s", e)

    # ---------- Robot: Set-API (UI -> ROS) ----------
    def robot_init(self) -> None:
        if not self._rb:
            _LOG.error("robot_init: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_init()
        except Exception as e:
            _LOG.error("robot_init failed: %s", e)

    def robot_stop(self) -> None:
        if not self._rb:
            _LOG.error("robot_stop: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_stop()
        except Exception as e:
            _LOG.error("robot_stop failed: %s", e)

    def robot_clear_error(self) -> None:
        if not self._rb:
            _LOG.error("robot_clear_error: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_clear_error()
        except Exception as e:
            _LOG.error("robot_clear_error failed: %s", e)

    def robot_power_on(self) -> None:
        if not self._rb:
            _LOG.error("robot_power_on: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_power_on()
        except Exception as e:
            _LOG.error("robot_power_on failed: %s", e)

    def robot_power_off(self) -> None:
        if not self._rb:
            _LOG.error("robot_power_off: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_power_off()
        except Exception as e:
            _LOG.error("robot_power_off failed: %s", e)

    def robot_servo_on(self) -> None:
        if not self._rb:
            _LOG.error("robot_servo_on: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_servo_on()
        except Exception as e:
            _LOG.error("robot_servo_on failed: %s", e)

    def robot_servo_off(self) -> None:
        if not self._rb:
            _LOG.error("robot_servo_off: RobotBridge nicht verfügbar.")
            return
        try:
            self._rb.do_servo_off()
        except Exception as e:
            _LOG.error("robot_servo_off failed: %s", e)

    # ---------- Servo: CommandType-API (UI -> ROS) ----------
    def servo_set_command_type(self, mode: str) -> None:
        if not self._sev:
            _LOG.error("servo_set_command_type: ServoBridge nicht verfügbar.")
            return
        try:
            self._sev.set_command_type(mode)
        except Exception as e:
            _LOG.error("servo_set_command_type failed: %s", e)

    # ---------- MoveItPy: High-Level-API (UI -> MoveItPyBridge) ----------
    def motion_move_home(self) -> None:
        if not self._mb or not getattr(self._mb, "signals", None):
            _LOG.error("motion_move_home: MoveItPyBridge nicht verfügbar.")
            return
        try:
            self._mb.signals.moveToHomeRequested.emit()
        except Exception as e:
            _LOG.error("motion_move_home failed: %s", e)

    def motion_move_service(self) -> None:
        if not self._mb or not getattr(self._mb, "signals", None):
            _LOG.error("motion_move_service: MoveItPyBridge nicht verfügbar.")
            return
        try:
            self._mb.signals.moveToServiceRequested.emit()
        except Exception as e:
            _LOG.error("motion_move_service failed: %s", e)

    def motion_move_to_pose(self, pose: PoseStamped) -> None:
        if pose is None:
            _LOG.error("motion_move_to_pose: pose is None.")
            return
        if not self._mb or not getattr(self._mb, "signals", None):
            _LOG.error("motion_move_to_pose: MoveItPyBridge nicht verfügbar.")
            return
        try:
            self._mb.signals.moveToPoseRequested.emit(pose)
        except Exception as e:
            _LOG.error("motion_move_to_pose failed: %s", e)

    def motion_last_result(self) -> str:
        if not self._mb or not getattr(self._mb, "signals", None):
            return ""
        try:
            return getattr(self._mb.signals, "last_result", "") or ""
        except Exception:
            return ""

    def motion_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self.motion.planned()

    def motion_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self.motion.executed()

    # ---------- Intern: Wiring von Qt-Signalen -> States ----------
    def _wire_scene_into_state(self, sb: SceneBridge) -> None:
        sig = sb.signals
        sig.cageListChanged.connect(lambda items: self.scene._set_cage_list(items))
        sig.mountListChanged.connect(lambda items: self.scene._set_mount_list(items))
        sig.substrateListChanged.connect(lambda items: self.scene._set_substrate_list(items))
        sig.cageCurrentChanged.connect(lambda v: self.scene._set_cage_current(v))
        sig.mountCurrentChanged.connect(lambda v: self.scene._set_mount_current(v))
        sig.substrateCurrentChanged.connect(lambda v: self.scene._set_substrate_current(v))

    def _wire_poses_into_state(self, pb: PosesBridge) -> None:
        sig = pb.signals
        sig.homePoseChanged.connect(lambda msg: self.poses._set_home(msg))
        sig.servicePoseChanged.connect(lambda msg: self.poses._set_service(msg))

    def _wire_spraypath_into_state(self, spb: SprayPathBridge) -> None:
        sig = spb.signals
        sig.currentNameChanged.connect(lambda name: self.spraypath._set_current_name(name))
        sig.posesChanged.connect(lambda pa: self.spraypath._set_poses(pa))

    def _wire_robot_into_state(self, rb: RobotBridge) -> None:
        sig = rb.signals
        sig.connectionChanged.connect(lambda v: self.robot._set_connection(v))
        sig.modeChanged.connect(lambda v: self.robot._set_mode(v))
        sig.initializedChanged.connect(lambda v: self.robot._set_initialized(v))
        sig.movingChanged.connect(lambda v: self.robot._set_moving(v))
        sig.servoEnabledChanged.connect(lambda v: self.robot._set_servo_enabled(v))
        sig.powerChanged.connect(lambda v: self.robot._set_power(v))
        sig.estopChanged.connect(lambda v: self.robot._set_estop(v))
        sig.errorsChanged.connect(lambda v: self.robot._set_errors(v))
        sig.tcpPoseChanged.connect(lambda msg: self.robot._set_tcp_pose(msg))
        sig.jointsChanged.connect(lambda msg: self.robot._set_joints(msg))

    def _wire_motion_into_state(self, mb: MoveItPyBridge) -> None:
        sig = mb.signals
        sig.plannedTrajectoryChanged.connect(lambda msg: self.motion._set_planned(msg))
        sig.executedTrajectoryChanged.connect(lambda msg: self.motion._set_executed(msg))


def get_ui_bridge(_ctx, namespace: str = "") -> UIBridge:
    """
    Factory für die Startup-FSM:
      - Erstellt die Bridge
      - Verbindet sofort (idempotent)

    Namespace:
      - ""        → kein zusätzlicher ROS-Namespace
      - "shadow"  → /shadow/...
      - "live"    → /live/...
    """
    b = UIBridge(namespace=namespace)
    b.ensure_connected()
    return b
