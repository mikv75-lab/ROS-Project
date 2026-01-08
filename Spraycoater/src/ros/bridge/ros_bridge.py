# -*- coding: utf-8 -*-
"""
src/ros/bridge/ros_bridge.py

Central ROS bridge wrapper that starts a small executor thread and hosts several
UI<->ROS bridge nodes (SceneBridge, PosesBridge, SprayPathBridge, ServoBridge, RobotBridge, MoveItPyBridge).

Key points:
- Uses config_hub topics/qos (SSoT).
- Each bridge node is a rclpy.Node and owns its own Qt signals.
- This wrapper wires inbound Qt signals from bridges into simple state objects.

NOTE (2026-01):
PosesSignals renamed signals:
  - old: homeChanged, serviceChanged
  - new: homePoseChanged, servicePoseChanged
We wire these backward-compatible to avoid crashes on mixed versions.
"""

from __future__ import annotations

import threading
import logging
from dataclasses import dataclass
from typing import Optional, List

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config.startup import AppContent, AppContext

from .scene_bridge import SceneBridge
from .poses_bridge import PosesBridge
from .spray_path_bridge import SprayPathBridge
from .servo_bridge import ServoBridge
from .robot_bridge import RobotBridge
from .moveitpy_bridge import MoveItPyBridge

_LOG = logging.getLogger(__name__)


# ---------------- State containers (UI friendly) ----------------

@dataclass
class SceneState:
    cage_list: List[str] = None
    mount_list: List[str] = None
    substrate_list: List[str] = None
    cage_current: str = ""
    mount_current: str = ""
    substrate_current: str = ""

    def __post_init__(self):
        self.cage_list = self.cage_list or []
        self.mount_list = self.mount_list or []
        self.substrate_list = self.substrate_list or []


class SceneStateAdapter:
    def __init__(self, st: SceneState):
        self.st = st

    def _set_cage_list(self, v: list) -> None:
        self.st.cage_list = list(v)

    def _set_mount_list(self, v: list) -> None:
        self.st.mount_list = list(v)

    def _set_substrate_list(self, v: list) -> None:
        self.st.substrate_list = list(v)

    def _set_cage_current(self, v: str) -> None:
        self.st.cage_current = v or ""

    def _set_mount_current(self, v: str) -> None:
        self.st.mount_current = v or ""

    def _set_substrate_current(self, v: str) -> None:
        self.st.substrate_current = v or ""


@dataclass
class PosesState:
    home: Optional[object] = None
    service: Optional[object] = None


class PosesStateAdapter:
    def __init__(self, st: PosesState):
        self.st = st

    def _set_home(self, msg) -> None:
        self.st.home = msg

    def _set_service(self, msg) -> None:
        self.st.service = msg


@dataclass
class SprayPathState:
    current: str = ""
    poses: Optional[object] = None
    markers: Optional[object] = None
    executed_poses: Optional[object] = None
    executed_markers: Optional[object] = None


class SprayPathStateAdapter:
    def __init__(self, st: SprayPathState):
        self.st = st

    def _set_current(self, v: str) -> None:
        self.st.current = v or ""

    def _set_poses(self, msg) -> None:
        self.st.poses = msg

    def _set_markers(self, msg) -> None:
        self.st.markers = msg

    def _set_executed_poses(self, msg) -> None:
        self.st.executed_poses = msg

    def _set_executed_markers(self, msg) -> None:
        self.st.executed_markers = msg


@dataclass
class RobotState:
    connection: bool = False
    mode: str = ""
    initialized: bool = False
    moving: bool = False
    servo_enabled: bool = False
    power: bool = False
    estop: bool = False
    errors: str = ""
    tcp_pose: Optional[object] = None
    joints: Optional[object] = None


class RobotStateAdapter:
    def __init__(self, st: RobotState):
        self.st = st

    def _set_connection(self, v: bool) -> None:
        self.st.connection = bool(v)

    def _set_mode(self, v: str) -> None:
        self.st.mode = v or ""

    def _set_initialized(self, v: bool) -> None:
        self.st.initialized = bool(v)

    def _set_moving(self, v: bool) -> None:
        self.st.moving = bool(v)

    def _set_servo_enabled(self, v: bool) -> None:
        self.st.servo_enabled = bool(v)

    def _set_power(self, v: bool) -> None:
        self.st.power = bool(v)

    def _set_estop(self, v: bool) -> None:
        self.st.estop = bool(v)

    def _set_errors(self, v: str) -> None:
        self.st.errors = v or ""

    def _set_tcp_pose(self, msg) -> None:
        self.st.tcp_pose = msg

    def _set_joints(self, msg) -> None:
        self.st.joints = msg


@dataclass
class MoveItState:
    planned: Optional[object] = None
    executed: Optional[object] = None
    urdf: str = ""
    srdf: str = ""


class MoveItStateAdapter:
    def __init__(self, st: MoveItState):
        self.st = st

    def _set_planned(self, msg) -> None:
        self.st.planned = msg

    def _set_executed(self, msg) -> None:
        self.st.executed = msg

    def _set_urdf(self, v: str) -> None:
        self.st.urdf = v or ""

    def _set_srdf(self, v: str) -> None:
        self.st.srdf = v or ""


# ---------------- Main RosBridge ----------------

class RosBridge:
    def __init__(self, ctx: AppContext, *, namespace: str = "") -> None:
        self._ctx = ctx
        self._namespace = (namespace or "").strip().strip("/")
        self._lock = threading.RLock()

        self._exec: Optional[SingleThreadedExecutor] = None
        self._thread: Optional[threading.Thread] = None
        self._nodes: List[object] = []
        self._running: bool = False
        self._did_init_rclpy: bool = False

        # content (topics/qos)
        self._content: AppContent = ctx.content

        # states (raw storage)
        self._scene_state = SceneState()
        self._poses_state = PosesState()
        self._spraypath_state = SprayPathState()
        self._robot_state = RobotState()
        self._moveit_state = MoveItState()

        # adapters exposed as public attributes (UI/state machines read ros.<x>_state.st)
        self.scene_state = SceneStateAdapter(self._scene_state)
        self.poses_state = PosesStateAdapter(self._poses_state)
        self.spraypath_state = SprayPathStateAdapter(self._spraypath_state)
        self.robot_state = RobotStateAdapter(self._robot_state)
        self.moveit_state = MoveItStateAdapter(self._moveit_state)

        # bridge nodes (created in start())
        self.scene: Optional[SceneBridge] = None
        self.poses: Optional[PosesBridge] = None
        self.spray: Optional[SprayPathBridge] = None
        self.servo: Optional[ServoBridge] = None
        self.robot: Optional[RobotBridge] = None
        self.moveitpy: Optional[MoveItPyBridge] = None

    @property
    def namespace(self) -> str:
        return self._namespace

    def start(self) -> None:
        with self._lock:
            if self._running:
                return

            if not rclpy.ok():
                rclpy.init()
                self._did_init_rclpy = True

            self._exec = SingleThreadedExecutor()
            self._nodes = []

            # create bridge nodes
            self.scene = SceneBridge(self._content, namespace=self._namespace)
            self.poses = PosesBridge(self._content, namespace=self._namespace)
            self.spray = SprayPathBridge(self._content, namespace=self._namespace)

            # IMPORTANT: all bridges must receive AppContent (NOT AppContext)
            self.servo = ServoBridge(self._content, namespace=self._namespace)
            self.robot = RobotBridge(self._content, namespace=self._namespace)
            self.moveitpy = MoveItPyBridge(self._content, namespace=self._namespace)

            # register nodes
            for n in [self.scene, self.poses, self.spray, self.servo, self.robot, self.moveitpy]:
                if n is None:
                    continue
                self._nodes.append(n)
                self._exec.add_node(n)

            # wire bridge signals into states
            self._wire_all_into_states()

            # reemit cached values so UI sees latched state immediately
            self._reemit_cached()

            # executor thread
            self._thread = threading.Thread(target=self._spin, daemon=True)
            self._running = True
            self._thread.start()

    def _spin(self) -> None:
        try:
            assert self._exec is not None
            self._exec.spin()
        finally:
            with self._lock:
                self._running = False

    def stop(self) -> None:
        with self._lock:
            if not self._running and self._exec is None and not self._nodes:
                return

            exec_ = self._exec
            thread = self._thread
            nodes = list(self._nodes)
            did_init = self._did_init_rclpy

            self._exec = None
            self._thread = None
            self._nodes = []
            self._running = False
            self._did_init_rclpy = False

        if exec_ is not None:
            exec_.shutdown()

        if exec_ is not None:
            for n in nodes:
                exec_.remove_node(n)

        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                _LOG.exception("destroy_node failed: %s", getattr(n, "get_name", lambda: str(n))())

        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

        if did_init and rclpy.ok():
            rclpy.shutdown()

    # ---------- Wiring: Qt-Signale -> States ----------
    def _wire_all_into_states(self) -> None:
        if self.scene is not None:
            sig = self.scene.signals
            sig.cageListChanged.connect(self.scene_state._set_cage_list)
            sig.mountListChanged.connect(self.scene_state._set_mount_list)
            sig.substrateListChanged.connect(self.scene_state._set_substrate_list)
            sig.cageCurrentChanged.connect(self.scene_state._set_cage_current)
            sig.mountCurrentChanged.connect(self.scene_state._set_mount_current)
            sig.substrateCurrentChanged.connect(self.scene_state._set_substrate_current)

        if self.poses is not None:
            sig = self.poses.signals
            if hasattr(sig, "homePoseChanged"):
                sig.homePoseChanged.connect(self.poses_state._set_home)
            elif hasattr(sig, "homeChanged"):
                sig.homeChanged.connect(self.poses_state._set_home)

            if hasattr(sig, "servicePoseChanged"):
                sig.servicePoseChanged.connect(self.poses_state._set_service)
            elif hasattr(sig, "serviceChanged"):
                sig.serviceChanged.connect(self.poses_state._set_service)

        if self.spray is not None:
            sig = self.spray.signals
            if hasattr(sig, "currentChanged"):
                sig.currentChanged.connect(self.spraypath_state._set_current)
            if hasattr(sig, "posesChanged"):
                sig.posesChanged.connect(self.spraypath_state._set_poses)
            if hasattr(sig, "markersChanged"):
                sig.markersChanged.connect(self.spraypath_state._set_markers)
            if hasattr(sig, "executedPosesChanged"):
                sig.executedPosesChanged.connect(self.spraypath_state._set_executed_poses)
            if hasattr(sig, "executedMarkersChanged"):
                sig.executedMarkersChanged.connect(self.spraypath_state._set_executed_markers)

        if self.robot is not None:
            sig = self.robot.signals
            sig.connectionChanged.connect(self.robot_state._set_connection)
            sig.modeChanged.connect(self.robot_state._set_mode)
            sig.initializedChanged.connect(self.robot_state._set_initialized)
            sig.movingChanged.connect(self.robot_state._set_moving)
            sig.servoEnabledChanged.connect(self.robot_state._set_servo_enabled)
            sig.powerChanged.connect(self.robot_state._set_power)
            sig.estopChanged.connect(self.robot_state._set_estop)
            sig.errorsChanged.connect(self.robot_state._set_errors)
            sig.tcpPoseChanged.connect(self.robot_state._set_tcp_pose)
            sig.jointsChanged.connect(self.robot_state._set_joints)

        # ✅ MoveItPyBridge (minimal + robot_description strings)
        if self.moveitpy is not None:
            sig = self.moveitpy.signals

            sig.plannedTrajectoryChanged.connect(self.moveit_state._set_planned)
            sig.executedTrajectoryChanged.connect(self.moveit_state._set_executed)
            sig.robotDescriptionChanged.connect(self.moveit_state._set_urdf)
            sig.robotDescriptionSemanticChanged.connect(self.moveit_state._set_srdf)

            # (motion_result is stored in sig.last_result; wire if needed)
            # sig.motionResultChanged.connect(lambda _txt: None)

    def _reemit_cached(self) -> None:
        try:
            if self.scene is not None:
                self.scene.signals.reemit_cached()
            if self.poses is not None:
                self.poses.signals.reemit_cached()
            if self.spray is not None:
                self.spray.signals.reemit_cached()
            if self.servo is not None:
                self.servo.signals.reemit_cached()
            if self.robot is not None:
                self.robot.signals.reemit_cached()
            if self.moveitpy is not None:
                self.moveitpy.signals.reemit_cached()
        except Exception:
            _LOG.exception("reemit_cached failed")

    # ---------------------------------------------------------------------
    # Public helpers / facade API (used by ProcessTab + state machines)
    # ---------------------------------------------------------------------

    def is_running(self) -> bool:
        """Return True if the executor thread is running."""
        try:
            thr = self._thread
            return bool(self._running) and (thr is not None) and thr.is_alive()
        except Exception:
            return bool(self._running)

    # --- SprayPath --------------------------------------------------------

    def spray_set_compiled(self, *, markers) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_compiled", None)
        if callable(fn):
            fn(markers)
            return
        sig = getattr(self.spray, "signals", None)
        for name in ("setCompiledRequested", "compiledRequested", "compiledMarkersRequested"):
            s = getattr(sig, name, None)
            if s is not None and hasattr(s, "emit"):
                s.emit(markers)
                return
        raise AttributeError("SprayPathBridge has no compiled setter")

    def spray_set_traj(self, *, markers) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_traj", None)
        if callable(fn):
            fn(markers)
            return
        sig = getattr(self.spray, "signals", None)
        for name in ("setTrajRequested", "trajRequested", "trajMarkersRequested"):
            s = getattr(sig, name, None)
            if s is not None and hasattr(s, "emit"):
                s.emit(markers)
                return
        raise AttributeError("SprayPathBridge has no traj setter")

    def spray_set_executed(self, *, markers) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_executed", None)
        if callable(fn):
            fn(markers)
            return
        sig = getattr(self.spray, "signals", None)
        for name in ("setExecutedRequested", "executedRequested", "executedMarkersRequested"):
            s = getattr(sig, name, None)
            if s is not None and hasattr(s, "emit"):
                s.emit(markers)
                return
        raise AttributeError("SprayPathBridge has no executed setter")

    def spray_set_view(self, view: str) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_view", None)
        if callable(fn):
            fn(view)
            return
        sig = getattr(self.spray, "signals", None)
        for name in ("setViewRequested", "viewRequested"):
            s = getattr(sig, name, None)
            if s is not None and hasattr(s, "emit"):
                s.emit(str(view))
                return
        raise AttributeError("SprayPathBridge has no view setter")

    # --- Poses ------------------------------------------------------------

    def set_home(self) -> None:
        if self.poses is None:
            raise RuntimeError("PosesBridge not started")
        self.poses.set_home()

    def set_service(self) -> None:
        if self.poses is None:
            raise RuntimeError("PosesBridge not started")
        self.poses.set_service()

    # --- Robot ------------------------------------------------------------

    def robot_init(self) -> None:
        if self.robot is None:
            raise RuntimeError("RobotBridge not started")
        self.robot.signals.initRequested.emit()

    def robot_stop(self) -> None:
        if self.robot is None:
            return
        self.robot.signals.stopRequested.emit()

    # --- MoveItPy ---------------------------------------------------------

    def moveit_move_home(self) -> None:
        if self.moveitpy is None:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.signals.moveToHomeRequested.emit()

    def moveit_move_service(self) -> None:
        if self.moveitpy is None:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.signals.moveToServiceRequested.emit()

    def moveit_stop(self) -> None:
        if self.moveitpy is None:
            return
        self.moveitpy.signals.stopRequested.emit()

    def stop_all(self) -> None:
        """Best-effort stop of motion + robot."""
        try:
            self.moveit_stop()
        finally:
            self.robot_stop()

    def moveit_last_result(self) -> str:
        if self.moveitpy is None:
            return ""
        return (getattr(self.moveitpy.signals, "last_result", "") or "").strip()


# ---------------- Convenience factory ----------------

def make_ros_bridge(ctx: AppContext, *, namespace: str = "") -> RosBridge:
    return RosBridge(ctx, namespace=namespace)
