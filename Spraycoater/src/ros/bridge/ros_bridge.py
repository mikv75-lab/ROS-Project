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

UPDATE (planned/executed naming):
- SprayPathState uses planned_* (not traj_*)
- Wiring listens to both plannedAvailableChanged and legacy trajAvailableChanged.
- Facade API uses spray_set_planned(...) (with legacy spray_set_traj(...) alias).
- Bridge method signatures for setters are keyword-only: set_compiled(*, poses=None, markers=None) etc.

MOVEITPY FIX (minimal, no refactor):
- Provide RosBridge.moveit_planned_trajectory() / moveit_executed_trajectory()
  so BaseProcessStatemachine can reliably capture JT/RobotTrajectory sources.
- Keep existing MoveItPy state wiring; do not change frame semantics.

MOVEITPY EXT (replay/optimize):
- Expose:
    - RosBridge.moveit_execute_trajectory(traj, segment=...)
    - RosBridge.moveit_optimize_trajectory(traj, segment=...)
  that forward to MoveItPyBridge (topics: execute_trajectory, optimize_trajectory, set_segment).
- Optimized trajectory can be optionally received via optimized_trajectory_rt (if present).

UPDATE (2026-01 signals):
- MoveItPyBridge uses:
    - plannedTrajectoryChanged / executedTrajectoryChanged / optimizedTrajectoryChanged (optional)
    - robotDescriptionChanged / robotDescriptionSemanticChanged
    - segmentChanged / executeTrajectoryRequested / optimizeTrajectoryRequested
    - trajCacheClearChanged  (NEW) boundary event from node:
        published when a new external "move call" starts (plan_pose, plan_named,
        execute_trajectory, optimize_trajectory). This ensures we do NOT reuse old
        planned/executed/optimized artifacts across segments.
  RosBridge wires these with best-effort for optional signals.
"""

from __future__ import annotations

import threading
import logging
from dataclasses import dataclass
from typing import Optional, List, Any

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

    # payloads
    compiled_poses: Optional[object] = None
    compiled_markers: Optional[object] = None
    planned_poses: Optional[object] = None
    planned_markers: Optional[object] = None
    executed_poses: Optional[object] = None
    executed_markers: Optional[object] = None

    # availability flags (for SprayPathBox "Available" column)
    compiled_available: bool = False
    planned_available: bool = False
    executed_available: bool = False


class SprayPathStateAdapter:
    def __init__(self, st: SprayPathState):
        self.st = st

    def _set_current(self, v: str) -> None:
        self.st.current = v or ""

    def _set_compiled_poses(self, msg) -> None:
        self.st.compiled_poses = msg

    def _set_compiled_markers(self, msg) -> None:
        self.st.compiled_markers = msg

    def _set_planned_poses(self, msg) -> None:
        self.st.planned_poses = msg

    def _set_planned_markers(self, msg) -> None:
        self.st.planned_markers = msg

    def _set_executed_poses(self, msg) -> None:
        self.st.executed_poses = msg

    def _set_executed_markers(self, msg) -> None:
        self.st.executed_markers = msg

    def _set_compiled_available(self, v: bool) -> None:
        self.st.compiled_available = bool(v)

    def _set_planned_available(self, v: bool) -> None:
        self.st.planned_available = bool(v)

    def _set_executed_available(self, v: bool) -> None:
        self.st.executed_available = bool(v)


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
    optimized: Optional[object] = None
    urdf: str = ""
    srdf: str = ""


class MoveItStateAdapter:
    def __init__(self, st: MoveItState):
        self.st = st

    def _set_planned(self, msg) -> None:
        self.st.planned = msg

    def _set_executed(self, msg) -> None:
        self.st.executed = msg

    def _set_optimized(self, msg) -> None:
        self.st.optimized = msg

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

            # inbound outputs
            for name, fn in (
                ("compiledPosesChanged", self.spraypath_state._set_compiled_poses),
                ("compiledMarkersChanged", self.spraypath_state._set_compiled_markers),
                ("plannedPosesChanged", self.spraypath_state._set_planned_poses),
                ("plannedMarkersChanged", self.spraypath_state._set_planned_markers),
                ("executedPosesChanged", self.spraypath_state._set_executed_poses),
                ("executedMarkersChanged", self.spraypath_state._set_executed_markers),
            ):
                if hasattr(sig, name):
                    getattr(sig, name).connect(fn)

            # Backward-compat (older SprayPathBridge may still emit posesChanged/markersChanged as "planned")
            if hasattr(sig, "posesChanged"):
                sig.posesChanged.connect(self.spraypath_state._set_planned_poses)
            if hasattr(sig, "markersChanged"):
                sig.markersChanged.connect(self.spraypath_state._set_planned_markers)

            # Availability (preferred + legacy)
            if hasattr(sig, "compiledAvailableChanged"):
                sig.compiledAvailableChanged.connect(self.spraypath_state._set_compiled_available)

            if hasattr(sig, "plannedAvailableChanged"):
                sig.plannedAvailableChanged.connect(self.spraypath_state._set_planned_available)
            elif hasattr(sig, "trajAvailableChanged"):
                sig.trajAvailableChanged.connect(self.spraypath_state._set_planned_available)

            if hasattr(sig, "executedAvailableChanged"):
                sig.executedAvailableChanged.connect(self.spraypath_state._set_executed_available)

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

        # MoveItPyBridge state wiring (correct signals per MoveItPySignals)
        if self.moveitpy is not None:
            sig = self.moveitpy.signals

            # trajectories (required)
            if hasattr(sig, "plannedTrajectoryChanged"):
                sig.plannedTrajectoryChanged.connect(self.moveit_state._set_planned)
            if hasattr(sig, "executedTrajectoryChanged"):
                sig.executedTrajectoryChanged.connect(self.moveit_state._set_executed)

            # optimized (optional; only if your bridge/node provides it)
            if hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self.moveit_state._set_optimized)

            # URDF/SRDF strings (for Offline-FK in GUI)
            if hasattr(sig, "robotDescriptionChanged"):
                sig.robotDescriptionChanged.connect(self.moveit_state._set_urdf)
            if hasattr(sig, "robotDescriptionSemanticChanged"):
                sig.robotDescriptionSemanticChanged.connect(self.moveit_state._set_srdf)

            # NEW: boundary event -> hard clear all MoveIt trajectory caches
            if hasattr(sig, "trajCacheClearChanged"):
                sig.trajCacheClearChanged.connect(self._on_moveit_traj_cache_clear)

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
    # NEW: MoveItPy boundary cache clear
    # ---------------------------------------------------------------------

    def _on_moveit_traj_cache_clear(self) -> None:
        """
        Called when MoveItPyNode publishes 'traj_cache_clear' (std_msgs/Empty).

        Purpose:
          - Prevent stale latched planned/executed/optimized trajectory artifacts from
            being interpreted as belonging to a new segment/run.
          - Ensure BaseProcessStatemachine getters see None until new artifacts arrive.

        We clear:
          - RosBridge MoveItState (planned/executed/optimized)
          - MoveItPyBridge internal synchronous caches (best-effort)
          - MoveItPySignals re-emit caches (best-effort)
        """
        with self._lock:
            # 1) clear RosBridge state
            self._moveit_state.planned = None
            self._moveit_state.executed = None
            self._moveit_state.optimized = None

            # 2) clear MoveItPyBridge caches (sync getters use these first)
            mp = self.moveitpy
            try:
                if mp is not None:
                    if hasattr(mp, "_last_planned_traj"):
                        mp._last_planned_traj = None  # type: ignore[attr-defined]
                    if hasattr(mp, "_last_executed_traj"):
                        mp._last_executed_traj = None  # type: ignore[attr-defined]
                    if hasattr(mp, "_last_optimized_traj"):
                        mp._last_optimized_traj = None  # type: ignore[attr-defined]
            except Exception:
                pass

            # 3) clear MoveItPySignals caches (UI reemit)
            try:
                if mp is not None and hasattr(mp, "signals"):
                    s = mp.signals
                    if hasattr(s, "_last_planned"):
                        s._last_planned = None  # type: ignore[attr-defined]
                    if hasattr(s, "_last_executed"):
                        s._last_executed = None  # type: ignore[attr-defined]
                    if hasattr(s, "_last_optimized"):
                        s._last_optimized = None  # type: ignore[attr-defined]
            except Exception:
                pass

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

    # --- MoveIt trajectory accessors (FIX for BaseProcessStatemachine) ----

    def moveit_planned_trajectory(self) -> Any:
        """
        Best-effort getter used by BaseProcessStatemachine.

        Prefer synchronous caches on MoveItPyBridge (set in ROS callback thread)
        to avoid Qt queued-signal races where motion_result arrives before state update.
        """
        mp = self.moveitpy

        # 0) strongest: direct bridge getter (sync, no Qt queue)
        try:
            if mp is not None:
                fn = getattr(mp, "last_planned_trajectory", None)
                if callable(fn):
                    v = fn()
                    if v is not None:
                        return v
        except Exception:
            pass

        # 0b) direct cache attribute (sync)
        try:
            if mp is not None:
                v = getattr(mp, "_last_planned_traj", None)
                if v is not None:
                    return v
        except Exception:
            pass

        # 1) UI state (may be delayed due to queued Qt signals)
        try:
            if self._moveit_state.planned is not None:
                return self._moveit_state.planned
        except Exception:
            pass

        return None

    def moveit_executed_trajectory(self) -> Any:
        """
        Best-effort getter used by BaseProcessStatemachine.

        Prefer synchronous caches on MoveItPyBridge to avoid Qt queued-signal races.
        """
        mp = self.moveitpy

        # 0) strongest: direct bridge getter (sync, no Qt queue)
        try:
            if mp is not None:
                fn = getattr(mp, "last_executed_trajectory", None)
                if callable(fn):
                    v = fn()
                    if v is not None:
                        return v
        except Exception:
            pass

        # 0b) direct cache attribute (sync)
        try:
            if mp is not None:
                v = getattr(mp, "_last_executed_traj", None)
                if v is not None:
                    return v
        except Exception:
            pass

        # 1) UI state (may be delayed)
        try:
            if self._moveit_state.executed is not None:
                return self._moveit_state.executed
        except Exception:
            pass

        return None

    def moveit_optimized_trajectory(self) -> Any:
        """
        Optional getter: last optimized trajectory if your bridge/node publishes it.

        Same race-avoidance strategy as planned/executed.
        """
        mp = self.moveitpy

        try:
            if mp is not None:
                fn = getattr(mp, "last_optimized_trajectory", None)
                if callable(fn):
                    v = fn()
                    if v is not None:
                        return v
        except Exception:
            pass

        try:
            if mp is not None:
                v = getattr(mp, "_last_optimized_traj", None)
                if v is not None:
                    return v
        except Exception:
            pass

        try:
            if self._moveit_state.optimized is not None:
                return self._moveit_state.optimized
        except Exception:
            pass

        return None

    # --- NEW: direct replay/optimize execution through MoveItPyBridge -----

    def moveit_set_segment(self, seg: str) -> None:
        """
        Optional tagging for motion_result (helps statemachine logs / replay debugging).
        Requires topics.yaml subscribe id: set_segment.
        """
        if self.moveitpy is None:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.signals.segmentChanged.emit(str(seg or ""))

    def moveit_execute_trajectory(self, traj: Any, *, segment: str = "") -> None:
        """
        Execute a given RobotTrajectory message via MoveItPyNode (no extra follow_joint node).

        Requires:
          - MoveItPyBridge bound topics.yaml subscribe id=execute_trajectory
          - optional: set_segment for tagging
        """
        if self.moveitpy is None:
            raise RuntimeError("MoveItPyBridge not started")

        if segment:
            # segmentChanged is best-effort; bridge ignores if topic missing
            try:
                self.moveitpy.signals.segmentChanged.emit(str(segment))
            except Exception:
                pass

        # ExecuteTrajectoryRequested enforces RobotTrajectoryMsg type in bridge
        self.moveitpy.signals.executeTrajectoryRequested.emit(traj)

    def moveit_optimize_trajectory(self, traj: Any, *, segment: str = "") -> None:
        """
        Trigger optimizer/retiming on a given RobotTrajectory (post pipeline etc.).

        Requires:
          - MoveItPyBridge bound topics.yaml subscribe id=optimize_trajectory
          - optional: set_segment for tagging
        """
        if self.moveitpy is None:
            raise RuntimeError("MoveItPyBridge not started")

        if segment:
            try:
                self.moveitpy.signals.segmentChanged.emit(str(segment))
            except Exception:
                pass

        self.moveitpy.signals.optimizeTrajectoryRequested.emit(traj)

    # --- SprayPath --------------------------------------------------------
    # NOTE: Bridge setter signatures are keyword-only: set_compiled(*, poses=None, markers=None)

    def spray_clear(self) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        sig = getattr(self.spray, "signals", None)
        if sig is not None and hasattr(sig, "clearRequested"):
            sig.clearRequested.emit()
            return
        fn = getattr(self.spray, "clear", None)
        if callable(fn):
            fn()
            return
        raise AttributeError("SprayPathBridge has no clear()")

    def spray_set_compiled(self, *, poses=None, markers=None) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_compiled", None)
        if callable(fn):
            fn(poses=poses, markers=markers)
            return
        raise AttributeError("SprayPathBridge has no set_compiled()")

    def spray_set_planned(self, *, poses=None, markers=None) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_planned", None)
        if callable(fn):
            fn(poses=poses, markers=markers)
            return
        fn2 = getattr(self.spray, "set_traj", None)
        if callable(fn2):
            fn2(poses=poses, markers=markers)
            return
        raise AttributeError("SprayPathBridge has no planned/traj setter")

    # legacy facade alias (keep callers working)
    def spray_set_traj(self, *, poses=None, markers=None) -> None:
        self.spray_set_planned(poses=poses, markers=markers)

    def spray_set_executed(self, *, poses=None, markers=None) -> None:
        if self.spray is None:
            raise RuntimeError("SprayPathBridge not started")
        fn = getattr(self.spray, "set_executed", None)
        if callable(fn):
            fn(poses=poses, markers=markers)
            return
        raise AttributeError("SprayPathBridge has no set_executed()")

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
        """
        Keep this facade method name stable.
        RobotInitStatemachine calls ros.robot_init().
        """
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
