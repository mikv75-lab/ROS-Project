# -*- coding: utf-8 -*-
"""
src/ros/bridge/ros_bridge.py

Central ROS bridge wrapper that starts a small executor thread and hosts several
UI<->ROS bridge nodes.

STRICT (2026-01):

MoveItPy Variant A (single plan_request channel):
- RosBridge forwards ONLY via MoveItPyBridge topics:
    - publish_plan_request(dict envelope)
    - publish_stop / publish_set_speed_mm_s / publish_set_planner_cfg
    - publish_execute_trajectory / publish_optimize_trajectory
- NO plan_named API in RosBridge (removed).
- Provide *local-only* boundary reset helpers so UI statemachines can hard-clear
  stale MoveIt state (results/latched traj) even if the node does not expose a
  clear service.

MoveItPy cache-clear behavior:
- MoveItPy node may publish traj_cache_clear (Empty) and/or empty RobotTrajectory
  to clear latched trajectories. RosBridge listens and clears UI state container.

NEW (2026-01):
- tray_exec_ready (std_msgs/Bool) is forwarded via MoveItPyBridge and stored in MoveItState.
  This is "controller/action readiness" (capability), not busy/occupancy.

NO-LEGACY GUARANTEE (important for your crash):
- No code path is allowed to send JSON strings for plan_request.
- Public API:
    - moveit_plan_request(op, payload, run, req_id, segment)  # builds key + dict payload
    - moveit_publish_plan_request(key=dict, payload=dict)      # dict-only
"""

from __future__ import annotations

import threading
import logging
from dataclasses import dataclass
from typing import Optional, List, Any, Dict

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

    compiled_available: bool = False
    planned_available: bool = False
    executed_available: bool = False

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

    # Derived busy state (from MoveItPyBridge)
    busy: bool = False

    # NEW: controller/action readiness (from MoveItPyBridge tray_exec_ready)
    tray_exec_ready: bool = False


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

    def _set_busy(self, v: bool) -> None:
        self.st.busy = bool(v)

    def _set_tray_exec_ready(self, v: bool) -> None:
        self.st.tray_exec_ready = bool(v)


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

    # ---------------- Lifecycle ----------------

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

            # reemit cached values (best-effort per node)
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

    # ---------------- Wiring ----------------

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

            if hasattr(sig, "compiledAvailableChanged"):
                sig.compiledAvailableChanged.connect(self.spraypath_state._set_compiled_available)
            if hasattr(sig, "plannedAvailableChanged"):
                sig.plannedAvailableChanged.connect(self.spraypath_state._set_planned_available)
            elif hasattr(sig, "trajAvailableChanged"):
                sig.trajAvailableChanged.connect(self.spraypath_state._set_planned_available)
            if hasattr(sig, "executedAvailableChanged"):
                sig.executedAvailableChanged.connect(self.spraypath_state._set_executed_available)

            if hasattr(sig, "compiledPosesChanged"):
                sig.compiledPosesChanged.connect(self.spraypath_state._set_compiled_poses)
            if hasattr(sig, "compiledMarkersChanged"):
                sig.compiledMarkersChanged.connect(self.spraypath_state._set_compiled_markers)
            if hasattr(sig, "plannedPosesChanged"):
                sig.plannedPosesChanged.connect(self.spraypath_state._set_planned_poses)
            if hasattr(sig, "executedPosesChanged"):
                sig.executedPosesChanged.connect(self.spraypath_state._set_executed_poses)

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
            sig.plannedTrajectoryChanged.connect(self.moveit_state._set_planned)
            sig.executedTrajectoryChanged.connect(self.moveit_state._set_executed)
            sig.optimizedTrajectoryChanged.connect(self.moveit_state._set_optimized)
            sig.robotDescriptionChanged.connect(self.moveit_state._set_urdf)
            sig.robotDescriptionSemanticChanged.connect(self.moveit_state._set_srdf)

            # busy derived in MoveItPyBridge
            if hasattr(sig, "busyChanged"):
                sig.busyChanged.connect(self.moveit_state._set_busy)

            # controller/action readiness
            if hasattr(sig, "trayExecReadyChanged"):
                sig.trayExecReadyChanged.connect(self.moveit_state._set_tray_exec_ready)

            # boundary clear from node (Empty) and/or empty-trajectory clears
            sig.trajCacheClearChanged.connect(self._on_moveit_traj_cache_clear)

    def _reemit_cached(self) -> None:
        """
        Best-effort: only call reemit_cached on nodes that implement it.
        (MoveItPy TOPIC-ONLY bridge does not need cached re-emit.)
        """
        try:
            for b in [self.scene, self.poses, self.spray, self.servo, self.robot, self.moveitpy]:
                if b is None:
                    continue
                sig = getattr(b, "signals", None)
                if sig is None:
                    continue
                if hasattr(sig, "reemit_cached"):
                    sig.reemit_cached()
        except Exception:
            _LOG.exception("reemit_cached failed")

    def _on_moveit_traj_cache_clear(self) -> None:
        """
        Boundary clear for MoveIt planned/executed/optimized in RosBridge UI state.
        """
        with self._lock:
            self._moveit_state.planned = None
            self._moveit_state.executed = None
            self._moveit_state.optimized = None
            # busy is controlled by busyChanged/result
            # tray_exec_ready is capability, updated by topic

    # ---------------- Public Facade API ----------------

    def is_running(self) -> bool:
        try:
            thr = self._thread
            return bool(self._running) and (thr is not None) and thr.is_alive()
        except Exception:
            return bool(self._running)

    # --- MoveIt (Variant A) ---

    def moveit_is_busy(self) -> bool:
        try:
            return bool(self._moveit_state.busy)
        except Exception:
            return False

    def moveit_tray_exec_ready(self) -> bool:
        try:
            return bool(self._moveit_state.tray_exec_ready)
        except Exception:
            return False

    def moveit_planned_trajectory(self) -> Any:
        try:
            if self.moveitpy:
                return self.moveitpy.last_planned_trajectory()
        except Exception:
            pass
        return self._moveit_state.planned

    def moveit_executed_trajectory(self) -> Any:
        try:
            if self.moveitpy:
                return self.moveitpy.last_executed_trajectory()
        except Exception:
            pass
        return self._moveit_state.executed

    def moveit_optimized_trajectory(self) -> Any:
        try:
            if self.moveitpy:
                return self.moveitpy.last_optimized_trajectory()
        except Exception:
            pass
        return self._moveit_state.optimized

    def moveit_last_result(self) -> str:
        if not self.moveitpy:
            return ""
        return getattr(self.moveitpy.signals, "last_result", "") or ""

    # ---- boundary reset helpers (UI-side, best-effort) ----

    def moveit_clear_motion_result(self) -> None:
        if not self.moveitpy:
            return
        try:
            self.moveitpy.signals.last_result = ""
            self.moveitpy.signals.motionResultChanged.emit("")
        except Exception:
            _LOG.exception("moveit_clear_motion_result failed")

    def moveit_reset_motion_result(self) -> None:
        self.moveit_clear_motion_result()

    def moveit_clear_result(self) -> None:
        self.moveit_clear_motion_result()

    def moveit_clear_trajectory_cache(self) -> None:
        try:
            self._on_moveit_traj_cache_clear()
            if self.moveitpy:
                self.moveitpy.signals.trajCacheClearChanged.emit()
        except Exception:
            _LOG.exception("moveit_clear_trajectory_cache failed")

    def moveit_clear_traj_cache(self) -> None:
        self.moveit_clear_trajectory_cache()

    def moveit_traj_cache_clear(self) -> None:
        self.moveit_clear_trajectory_cache()

    def moveit_clear_cache(self) -> None:
        self.moveit_clear_trajectory_cache()
        self.moveit_clear_motion_result()

    # ---- planning/execution (Variant A, STRICT) ----

    @staticmethod
    def _require_seg(seg: str) -> str:
        s = (seg or "").strip()
        if not s:
            raise ValueError("MoveItPy Variant-A requires non-empty seg (key.seg) [STRICT]")
        return s

    # ----------------------------
    # NO-LEGACY public senders (dict-only)
    # ----------------------------

    def moveit_publish_plan_request(self, *, key: Dict[str, Any], payload: Dict[str, Any]) -> None:
        """
        STRICT dict-only sender for plan_request envelope.
        Callers MUST NOT pass JSON strings.
        """
        if not isinstance(key, dict):
            raise TypeError(f"moveit_publish_plan_request: key must be dict, got {type(key)!r}")
        if not isinstance(payload, dict):
            raise TypeError(f"moveit_publish_plan_request: payload must be dict, got {type(payload)!r}")
        self._moveit_publish_plan_request(key=key, payload=payload)

    def moveit_plan_request(
        self,
        *,
        op: str,
        payload: Dict[str, Any],
        run: str,
        req_id: int,
        segment: str,
    ) -> None:
        """
        STRICT helper: builds key and publishes plan_request.
        """
        seg = self._require_seg(segment)
        op_s = str(op or "").strip()
        if not op_s:
            raise ValueError("moveit_plan_request: op must be non-empty")
        if not isinstance(payload, dict):
            raise TypeError("moveit_plan_request: payload must be dict")

        key = {"run": str(run or ""), "id": int(req_id), "seg": seg, "op": op_s}
        self._moveit_publish_plan_request(key=key, payload=dict(payload))

    # ----------------------------
    # payload helpers
    # ----------------------------

    @staticmethod
    def _pose_to_dict(pose) -> Dict[str, Any]:
        return {
            "position": {"x": float(pose.position.x), "y": float(pose.position.y), "z": float(pose.position.z)},
            "orientation": {
                "x": float(pose.orientation.x),
                "y": float(pose.orientation.y),
                "z": float(pose.orientation.z),
                "w": float(pose.orientation.w),
            },
        }

    @classmethod
    def _pose_stamped_payload(cls, ps: Any) -> Dict[str, Any]:
        hdr = getattr(ps, "header", None)
        frame_id = str(getattr(hdr, "frame_id", "") or "").strip()
        if not frame_id:
            raise ValueError("PoseStamped.header.frame_id is empty (required)")
        pose = getattr(ps, "pose", None)
        if pose is None:
            raise ValueError("PoseStamped.pose missing")
        return {"frame": frame_id, "pose": cls._pose_to_dict(pose)}

    @classmethod
    def _pose_array_payload(cls, pa: Any) -> Dict[str, Any]:
        hdr = getattr(pa, "header", None)
        frame_id = str(getattr(hdr, "frame_id", "") or "").strip()
        if not frame_id:
            raise ValueError("PoseArray.header.frame_id is empty (required)")
        poses = list(getattr(pa, "poses", []) or [])
        if len(poses) < 2:
            raise ValueError(f"PoseArray.poses too short (<2): {len(poses)}")
        return {"frame": frame_id, "poses": [cls._pose_to_dict(p) for p in poses]}

    def _moveit_publish_plan_request(self, *, key: Dict[str, Any], payload: Dict[str, Any]) -> None:
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")
        req = {"key": dict(key), "payload": dict(payload)}
        self.moveitpy.publish_plan_request(req)

    def moveit_set_speed_mm_s(self, speed: float) -> None:
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.publish_set_speed_mm_s(float(speed))

    def moveit_set_planner_cfg(self, cfg: object) -> None:
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.publish_set_planner_cfg(cfg)

    # ----------------------------
    # typed convenience ops (still Variant-A plan_request)
    # ----------------------------

    def moveit_plan_pose(self, pose_stamped: Any, *, run: str, req_id: int, segment: str) -> None:
        payload = self._pose_stamped_payload(pose_stamped)
        self.moveit_plan_request(op="plan_pose", payload=payload, run=run, req_id=req_id, segment=segment)

    def moveit_plan_pose_array(self, pose_array: Any, *, run: str, req_id: int, segment: str) -> None:
        payload = self._pose_array_payload(pose_array)
        self.moveit_plan_request(op="plan_pose_array", payload=payload, run=run, req_id=req_id, segment=segment)

    def moveit_execute_last_planned(self, *, run: str, req_id: int, segment: str) -> None:
        """
        Variant-A execute op: executes LAST planned trajectory on the node.
        """
        self.moveit_plan_request(op="execute", payload={}, run=run, req_id=req_id, segment=segment)

    def moveit_execute_trajectory(self, traj: Any) -> None:
        """
        External replay: key MUST be embedded in traj.joint_trajectory.header.frame_id JSON.
        """
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.publish_execute_trajectory(traj)

    def moveit_optimize_trajectory(self, traj: Any) -> None:
        """
        External optimize: key MUST be embedded in traj.joint_trajectory.header.frame_id JSON.
        """
        if not self.moveitpy:
            raise RuntimeError("MoveItPyBridge not started")
        self.moveitpy.publish_optimize_trajectory(traj)

    def moveit_stop(self) -> None:
        if not self.moveitpy:
            return
        self.moveitpy.publish_stop()

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
        # Legacy alias -> maps to NEW planned (kept for compatibility)
        self.spray_set_planned(new_poses=poses, new_markers=markers)

    def spray_set_executed(self, *, new_poses=None, new_markers=None, stored_markers=None) -> None:
        if self.spray:
            self.spray.set_executed(new_poses=new_poses, new_markers=new_markers, stored_markers=stored_markers)

    def spray_set_view(self, view: str) -> None:
        _ = view
        return


def make_ros_bridge(ctx: AppContext, *, namespace: str = "") -> RosBridge:
    return RosBridge(ctx, namespace=namespace)
