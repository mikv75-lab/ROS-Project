# -*- coding: utf-8 -*-
"""
src/ros/bridge/ros_bridge.py

Central ROS bridge wrapper that starts a small executor thread and hosts several
UI<->ROS bridge nodes.

Updated for SprayPath Cache Node (STRICT, 2026-01):
- Facade methods for SprayPath updated to use set_planned(new_..., stored_...)
- SprayPathState reflects new/stored availability structure.

Updated for MoveItPy PoseArray planning (Way A2, 2026-01):
- Add facade method moveit_plan_pose_array(PoseArray, segment=...)
  to plan MOVE_RECIPE as ONE multi-waypoint trajectory inside MoveItPy.

Updated for MoveItPy NEW cache clear behavior (STRICT, 2026-01):
- MoveItPy node publishes explicit traj_cache_clear (Empty) before each move call
- Node may also clear latched planned/executed/optimized topics by publishing EMPTY RobotTrajectory
- RosBridge listens to trajCacheClearChanged and clears MoveItState consistently
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
    """
    Reflects available data in SprayPathBridge.
    """
    current: str = ""

    # availability flags (used by UI checkboxes)
    compiled_available: bool = False
    planned_available: bool = False
    executed_available: bool = False

    # Raw payloads (usually not read back by UI, but kept for consistency)
    compiled_poses: Optional[object] = None
    compiled_markers: Optional[object] = None
    planned_poses: Optional[object] = None
    executed_poses: Optional[object] = None


class SprayPathStateAdapter:
    def __init__(self, st: SprayPathState):
        self.st = st

    def _set_current(self, v: str) -> None:
        self.st.current = v or ""

    def _set_compiled_available(self, v: bool) -> None:
        self.st.compiled_available = bool(v)

    def _set_planned_available(self, v: bool) -> None:
        self.st.planned_available = bool(v)

    def _set_executed_available(self, v: bool) -> None:
        self.st.executed_available = bool(v)

    def _set_compiled_poses(self, msg) -> None:
        self.st.compiled_poses = msg

    def _set_compiled_markers(self, msg) -> None:
        self.st.compiled_markers = msg

    def _set_planned_poses(self, msg) -> None:
        self.st.planned_poses = msg

    def _set_executed_poses(self, msg) -> None:
        self.st.executed_poses = msg


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

        self._content: AppContent = ctx.content

        # states
        self._scene_state = SceneState()
        self._poses_state = PosesState()
        self._spraypath_state = SprayPathState()
        self._robot_state = RobotState()
        self._moveit_state = MoveItState()

        # adapters
        self.scene_state = SceneStateAdapter(self._scene_state)
        self.poses_state = PosesStateAdapter(self._poses_state)
        self.spraypath_state = SprayPathStateAdapter(self._spraypath_state)
        self.robot_state = RobotStateAdapter(self._robot_state)
        self.moveit_state = MoveItStateAdapter(self._moveit_state)

        # bridge nodes
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

            # reemit cached values
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
            for n in nodes:
                exec_.remove_node(n)

        for n in nodes:
            try:
                n.destroy_node()
            except Exception:
                _LOG.exception("destroy_node failed")

        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)

        if did_init and rclpy.ok():
            rclpy.shutdown()

    # ---------- Wiring ----------
    def _wire_all_into_states(self) -> None:
        if self.scene:
            sig = self.scene.signals
            sig.cageListChanged.connect(self.scene_state._set_cage_list)
            sig.mountListChanged.connect(self.scene_state._set_mount_list)
            sig.substrateListChanged.connect(self.scene_state._set_substrate_list)
            sig.cageCurrentChanged.connect(self.scene_state._set_cage_current)
            sig.mountCurrentChanged.connect(self.scene_state._set_mount_current)
            sig.substrateCurrentChanged.connect(self.scene_state._set_substrate_current)

        if self.poses:
            sig = self.poses.signals
            if hasattr(sig, "homePoseChanged"):
                sig.homePoseChanged.connect(self.poses_state._set_home)
            elif hasattr(sig, "homeChanged"):
                sig.homeChanged.connect(self.poses_state._set_home)

            if hasattr(sig, "servicePoseChanged"):
                sig.servicePoseChanged.connect(self.poses_state._set_service)
            elif hasattr(sig, "serviceChanged"):
                sig.serviceChanged.connect(self.poses_state._set_service)

        if self.spray:
            sig = self.spray.signals
            if hasattr(sig, "currentChanged"):
                sig.currentChanged.connect(self.spraypath_state._set_current)

            # availability (combined new/stored)
            if hasattr(sig, "compiledAvailableChanged"):
                sig.compiledAvailableChanged.connect(self.spraypath_state._set_compiled_available)
            if hasattr(sig, "plannedAvailableChanged"):
                sig.plannedAvailableChanged.connect(self.spraypath_state._set_planned_available)
            elif hasattr(sig, "trajAvailableChanged"):
                sig.trajAvailableChanged.connect(self.spraypath_state._set_planned_available)
            if hasattr(sig, "executedAvailableChanged"):
                sig.executedAvailableChanged.connect(self.spraypath_state._set_executed_available)

            # payloads (optional)
            if hasattr(sig, "compiledPosesChanged"):
                sig.compiledPosesChanged.connect(self.spraypath_state._set_compiled_poses)
            if hasattr(sig, "compiledMarkersChanged"):
                sig.compiledMarkersChanged.connect(self.spraypath_state._set_compiled_markers)

        if self.robot:
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

        if self.moveitpy:
            sig = self.moveitpy.signals
            if hasattr(sig, "plannedTrajectoryChanged"):
                sig.plannedTrajectoryChanged.connect(self.moveit_state._set_planned)
            if hasattr(sig, "executedTrajectoryChanged"):
                sig.executedTrajectoryChanged.connect(self.moveit_state._set_executed)
            if hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self.moveit_state._set_optimized)
            if hasattr(sig, "robotDescriptionChanged"):
                sig.robotDescriptionChanged.connect(self.moveit_state._set_urdf)
            if hasattr(sig, "robotDescriptionSemanticChanged"):
                sig.robotDescriptionSemanticChanged.connect(self.moveit_state._set_srdf)

            # NEW: cache boundary from node (Empty) and/or empty-trajectory clears
            if hasattr(sig, "trajCacheClearChanged"):
                sig.trajCacheClearChanged.connect(self._on_moveit_traj_cache_clear)

    def _reemit_cached(self) -> None:
        try:
            if self.scene:
                self.scene.signals.reemit_cached()
            if self.poses:
                self.poses.signals.reemit_cached()
            if self.spray:
                self.spray.signals.reemit_cached()
            if self.servo:
                self.servo.signals.reemit_cached()
            if self.robot:
                self.robot.signals.reemit_cached()
            if self.moveitpy:
                self.moveitpy.signals.reemit_cached()
        except Exception:
            _LOG.exception("reemit_cached failed")

    def _on_moveit_traj_cache_clear(self) -> None:
        """
        Boundary clear for MoveIt planned/executed/optimized.

        NOTE:
          - moveitpy_bridge already clears its internal caches when it receives
            traj_cache_clear OR empty latched RobotTrajectory.
          - Here we only clear RosBridge state container so UI has one clean source.
        """
        with self._lock:
            self._moveit_state.planned = None
            self._moveit_state.executed = None
            self._moveit_state.optimized = None

    # ---------------------------------------------------------------------
    # Public Facade API
    # ---------------------------------------------------------------------

    def is_running(self) -> bool:
        try:
            thr = self._thread
            return bool(self._running) and (thr is not None) and thr.is_alive()
        except Exception:
            return bool(self._running)

    # --- MoveIt ---
    def moveit_planned_trajectory(self) -> Any:
        try:
            return self.moveitpy.last_planned_trajectory()
        except Exception:
            return self._moveit_state.planned

    def moveit_executed_trajectory(self) -> Any:
        try:
            return self.moveitpy.last_executed_trajectory()
        except Exception:
            return self._moveit_state.executed

    def moveit_optimized_trajectory(self) -> Any:
        try:
            return self.moveitpy.last_optimized_trajectory()
        except Exception:
            return self._moveit_state.optimized

    def moveit_set_segment(self, seg: str) -> None:
        if self.moveitpy:
            self.moveitpy.signals.segmentChanged.emit(str(seg or ""))

    def moveit_plan_pose_array(self, pose_array: Any, *, segment: str = "") -> None:
        """
        NEW (Way A2):
        Plan a full waypoint list as ONE multi-waypoint trajectory in the MoveItPy node.

        Expected input:
          - geometry_msgs/msg/PoseArray (no header; node assumes WORLD frame)
        """
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")

        # Use the bridge public API (preferred) so it can tag segment + validate types.
        if hasattr(self.moveitpy, "publish_plan_pose_array"):
            self.moveitpy.publish_plan_pose_array(pose_array, segment=segment)
            return

        # Fallback: emit signals directly (should not happen in the updated bridge)
        if segment:
            self.moveit_set_segment(segment)

        if not hasattr(self.moveitpy.signals, "planPoseArrayRequested"):
            raise RuntimeError("MoveItPyBridge does not expose planPoseArrayRequested (update moveitpy_bridge.py).")
        self.moveitpy.signals.planPoseArrayRequested.emit(pose_array)

    def moveit_execute_trajectory(self, traj: Any, *, segment: str = "") -> None:
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")

        # prefer explicit API (tags segment and validates type)
        if hasattr(self.moveitpy, "publish_execute_trajectory"):
            self.moveitpy.publish_execute_trajectory(traj, segment=segment)
            return

        if segment:
            self.moveit_set_segment(segment)
        self.moveitpy.signals.executeTrajectoryRequested.emit(traj)

    def moveit_optimize_trajectory(self, traj: Any, *, segment: str = "") -> None:
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")

        # prefer explicit API (tags segment and validates type)
        if hasattr(self.moveitpy, "publish_optimize_trajectory"):
            self.moveitpy.publish_optimize_trajectory(traj, segment=segment)
            return

        if segment:
            self.moveit_set_segment(segment)
        self.moveitpy.signals.optimizeTrajectoryRequested.emit(traj)

    def moveit_move_home(self) -> None:
        if self.moveitpy:
            self.moveitpy.signals.moveToHomeRequested.emit()

    def moveit_move_service(self) -> None:
        if self.moveitpy:
            self.moveitpy.signals.moveToServiceRequested.emit()

    def moveit_stop(self) -> None:
        if self.moveitpy:
            self.moveitpy.signals.stopRequested.emit()

    def moveit_last_result(self) -> str:
        if not self.moveitpy:
            return ""
        return getattr(self.moveitpy.signals, "last_result", "") or ""

    # --- Robot ---
    def robot_init(self) -> None:
        if self.robot:
            self.robot.signals.initRequested.emit()

    def robot_stop(self) -> None:
        if self.robot:
            self.robot.signals.stopRequested.emit()

    def stop_all(self) -> None:
        try:
            self.moveit_stop()
        finally:
            self.robot_stop()

    # --- Poses ---
    def set_home(self) -> None:
        if self.poses:
            self.poses.set_home()

    def set_service(self) -> None:
        if self.poses:
            self.poses.set_service()

    # --- SprayPath (STRICT: compiled, planned, executed) ---
    def spray_clear(self) -> None:
        if self.spray:
            self.spray.signals.clearRequested.emit()

    def spray_set_compiled(self, *, poses=None, markers=None) -> None:
        if self.spray:
            self.spray.set_compiled(poses=poses, markers=markers)

    def spray_set_planned(self, *, new_poses=None, new_markers=None, stored_markers=None) -> None:
        if self.spray:
            self.spray.set_planned(new_poses=new_poses, new_markers=new_markers, stored_markers=stored_markers)

    def spray_set_traj(self, *, poses=None, markers=None) -> None:
        # Legacy alias -> maps to NEW planned
        self.spray_set_planned(new_poses=poses, new_markers=markers)

    def spray_set_executed(self, *, new_poses=None, new_markers=None, stored_markers=None) -> None:
        if self.spray:
            self.spray.set_executed(new_poses=new_poses, new_markers=new_markers, stored_markers=stored_markers)

    def spray_set_view(self, view: str) -> None:
        # kept for API compatibility (UI may call it)
        _ = view
        return


def make_ros_bridge(ctx: AppContext, *, namespace: str = "") -> RosBridge:
    return RosBridge(ctx, namespace=namespace)
