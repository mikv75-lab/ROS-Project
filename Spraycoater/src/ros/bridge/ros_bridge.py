# src/ros/bridge/ros_bridge.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import threading
from typing import Any, List, Optional, Tuple, Type, TypeVar

import rclpy
from rclpy.executors import SingleThreadedExecutor

from config.startup import AppContext, load_startup

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
from trajectory_msgs.msg import JointTrajectory

_LOG = logging.getLogger("ros.ros_bridge")

T = TypeVar("T")


class SceneState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._cage_list: List[str] = []
        self._mount_list: List[str] = []
        self._substrate_list: List[str] = []
        self._cage_current: str = ""
        self._mount_current: str = ""
        self._substrate_current: str = ""

    def _set_cage_list(self, items: List[str]) -> None:
        with self._lock:
            self._cage_list = list(items)

    def _set_mount_list(self, items: List[str]) -> None:
        with self._lock:
            self._mount_list = list(items)

    def _set_substrate_list(self, items: List[str]) -> None:
        with self._lock:
            self._substrate_list = list(items)

    def _set_cage_current(self, v: str) -> None:
        with self._lock:
            self._cage_current = v.strip()

    def _set_mount_current(self, v: str) -> None:
        with self._lock:
            self._mount_current = v.strip()

    def _set_substrate_current(self, v: str) -> None:
        with self._lock:
            self._substrate_current = v.strip()

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


class PosesState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._home: Optional[PoseStamped] = None
        self._service: Optional[PoseStamped] = None

    def _set_home(self, msg: PoseStamped) -> None:
        with self._lock:
            self._home = msg

    def _set_service(self, msg: PoseStamped) -> None:
        with self._lock:
            self._service = msg

    def home(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._home

    def service(self) -> Optional[PoseStamped]:
        with self._lock:
            return self._service


class SprayPathState:
    """
    Vollständiger Cache für alle SprayPathBridge Inbound-Signale.
    """

    def __init__(self) -> None:
        self._lock = threading.RLock()

        self._current: str = ""

        self._poses: Optional[PoseArray] = None
        self._markers: Optional[MarkerArray] = None

        self._executed_poses: Optional[PoseArray] = None
        self._executed_markers: Optional[MarkerArray] = None

    # --- setters (wired from Qt signals) ---

    def _set_current(self, name: str) -> None:
        with self._lock:
            self._current = str(name or "")

    def _set_poses(self, pa: PoseArray) -> None:
        with self._lock:
            self._poses = pa

    def _set_markers(self, ma: MarkerArray) -> None:
        with self._lock:
            self._markers = ma

    def _set_executed_poses(self, pa: PoseArray) -> None:
        with self._lock:
            self._executed_poses = pa

    def _set_executed_markers(self, ma: MarkerArray) -> None:
        with self._lock:
            self._executed_markers = ma

    # --- getters ---

    def current(self) -> str:
        with self._lock:
            return self._current

    def poses(self) -> Optional[PoseArray]:
        with self._lock:
            return self._poses

    def markers(self) -> Optional[MarkerArray]:
        with self._lock:
            return self._markers

    def executed_poses(self) -> Optional[PoseArray]:
        with self._lock:
            return self._executed_poses

    def executed_markers(self) -> Optional[MarkerArray]:
        with self._lock:
            return self._executed_markers


class RobotState:
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

    def _set_connection(self, v: bool) -> None:
        with self._lock:
            self._connection = bool(v)

    def _set_mode(self, v: str) -> None:
        with self._lock:
            self._mode = v.strip() or "DISCONNECTED"

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
            self._errors = v.strip()

    def _set_tcp_pose(self, msg: Optional[PoseStamped]) -> None:
        with self._lock:
            self._tcp_pose = msg

    def _set_joints(self, msg: Optional[JointState]) -> None:
        with self._lock:
            self._joints = msg

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


class MoveItState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._target: Optional[JointTrajectory] = None
        self._planned: Optional[RobotTrajectoryMsg] = None
        self._executed: Optional[RobotTrajectoryMsg] = None
        self._preview: Optional[MarkerArray] = None
        self._srdf: str = ""

    def _set_target(self, msg: JointTrajectory) -> None:
        with self._lock:
            self._target = msg

    def _set_planned(self, msg: RobotTrajectoryMsg) -> None:
        with self._lock:
            self._planned = msg

    def _set_executed(self, msg: RobotTrajectoryMsg) -> None:
        with self._lock:
            self._executed = msg

    def _set_preview(self, msg: MarkerArray) -> None:
        with self._lock:
            self._preview = msg

    def _set_srdf(self, text: str) -> None:
        with self._lock:
            self._srdf = text or ""

    def target(self) -> Optional[JointTrajectory]:
        with self._lock:
            return self._target

    def planned(self) -> Optional[RobotTrajectoryMsg]:
        with self._lock:
            return self._planned

    def executed(self) -> Optional[RobotTrajectoryMsg]:
        with self._lock:
            return self._executed

    def preview(self) -> Optional[MarkerArray]:
        with self._lock:
            return self._preview

    def srdf(self) -> str:
        with self._lock:
            return self._srdf


class RosBridge:
    """
    Betreibt alle Bridge-Nodes in einem eigenen Executor-Thread.

    Wichtig:
      - zentrales Objekt ist AppContext
      - AppContent steckt in ctx.content (Frames/QoS/Topics)
    """

    def __init__(
        self,
        ctx: AppContext | None = None,
        *,
        startup_yaml_path: Optional[str] = None,
        enable_omron: bool = False,
        namespace: str | None = None,
    ):
        if ctx is None:
            if not startup_yaml_path:
                raise RuntimeError("RosBridge: ctx ist None und startup_yaml_path fehlt.")
            ctx = load_startup(startup_yaml_path)

        if ctx.content is None:
            raise RuntimeError("RosBridge: ctx.content ist None (load_startup hat kein content gebaut).")

        self._ctx: AppContext = ctx
        self._content = ctx.content

        self._enable_omron = bool(enable_omron)
        self._namespace = (namespace or "").strip().strip("/")

        self._exec: Optional[SingleThreadedExecutor] = None
        self._nodes: List[Any] = []
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._lock = threading.RLock()

        # ---------- State caches (thread-safe snapshots for UI) ----------
        self.scene_state = SceneState()
        self.poses_state = PosesState()
        self.spraypath_state = SprayPathState()
        self.robot_state = RobotState()
        self.moveit_state = MoveItState()

        # ---------- Node handles (set after start()) ----------
        self.scene: Optional[SceneBridge] = None
        self.poses: Optional[PosesBridge] = None
        self.spray: Optional[SprayPathBridge] = None
        self.servo: Optional[ServoBridge] = None
        self.robot: Optional[RobotBridge] = None
        self.moveitpy: Optional[MoveItPyBridge] = None
        self.omron: Optional[OmronBridge] = None

        # wichtig: nur shutdown() wenn WIR init gemacht haben
        self._did_init_rclpy = False

    @property
    def ctx(self) -> AppContext:
        return self._ctx

    @property
    def namespace(self) -> str:
        return self._namespace

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._running

    @property
    def primary_node(self):
        with self._lock:
            return self._nodes[0] if self._nodes else None

    @property
    def nodes(self) -> Tuple[Any, ...]:
        with self._lock:
            return tuple(self._nodes)

    def get_node(self, cls: Type[T]) -> Optional[T]:
        with self._lock:
            for n in self._nodes:
                if isinstance(n, cls):
                    return n
        return None

    def start(self) -> None:
        with self._lock:
            if self._running:
                return

            if not rclpy.ok():
                rclpy.init(args=None)
                self._did_init_rclpy = True

            self._exec = SingleThreadedExecutor(context=rclpy.get_default_context())
            ns = self._namespace

            # Bridges erzeugen (immer)
            self.scene = SceneBridge(self._content, namespace=ns)
            self.poses = PosesBridge(self._content, namespace=ns)
            self.spray = SprayPathBridge(self._content, namespace=ns)
            self.servo = ServoBridge(self._content, namespace=ns)
            self.robot = RobotBridge(self._content, namespace=ns)
            self.moveitpy = MoveItPyBridge(self._content, namespace=ns)

            self._nodes = [self.scene, self.poses, self.spray, self.servo, self.robot, self.moveitpy]

            self.omron = None
            if self._enable_omron:
                self.omron = OmronBridge(self._content, namespace=ns)
                self._nodes.append(self.omron)

            self._wire_all_into_states()
            self._reemit_cached()

            for n in self._nodes:
                self._exec.add_node(n)

            self._running = True
            self._thread = threading.Thread(
                target=self._spin,
                name=f"ros-bridge-{ns or 'root'}",
                daemon=True,
            )
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
            sig.homePoseChanged.connect(self.poses_state._set_home)
            sig.servicePoseChanged.connect(self.poses_state._set_service)

        if self.spray is not None:
            sig = self.spray.signals
            sig.currentChanged.connect(self.spraypath_state._set_current)
            sig.posesChanged.connect(self.spraypath_state._set_poses)
            sig.markersChanged.connect(self.spraypath_state._set_markers)
            sig.executedPosesChanged.connect(self.spraypath_state._set_executed_poses)
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

        if self.moveitpy is not None:
            sig = self.moveitpy.signals
            sig.targetTrajectoryChanged.connect(self.moveit_state._set_target)
            sig.plannedTrajectoryChanged.connect(self.moveit_state._set_planned)
            sig.executedTrajectoryChanged.connect(self.moveit_state._set_executed)
            sig.previewMarkersChanged.connect(self.moveit_state._set_preview)
            sig.robotDescriptionSemanticChanged.connect(self.moveit_state._set_srdf)

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
            if self.omron is not None:
                self.omron.signals.reemit_cached()
        except Exception:
            _LOG.exception("reemit_cached failed")

    def add_node(self, node: Any) -> None:
        with self._lock:
            if self._exec is None:
                raise RuntimeError("RosBridge läuft nicht (Executor ist None).")
            self._nodes.append(node)
            self._exec.add_node(node)

    # ---------- High-Level API (optional convenience) ----------
    # Scene
    def set_cage(self, name: str) -> None:
        if self.scene is not None:
            self.scene.set_cage(name)

    def set_mount(self, name: str) -> None:
        if self.scene is not None:
            self.scene.set_mount(name)

    def set_substrate(self, name: str) -> None:
        if self.scene is not None:
            self.scene.set_substrate(name)

    # Poses
    def set_home(self) -> None:
        if self.poses is not None:
            self.poses.set_home()

    def set_service(self) -> None:
        if self.poses is not None:
            self.poses.set_service()

    # SprayPath (New API)
    def spray_set_view(self, view_key: str) -> None:
        if self.spray is not None:
            self.spray.publish_set_view(view_key)

    def spray_set_compiled(self, *, poses: PoseArray | None = None, markers: MarkerArray | None = None) -> None:
        if self.spray is None:
            return
        if poses is not None:
            self.spray.publish_compiled_poses(poses)
        if markers is not None:
            self.spray.publish_compiled_markers(markers)

    def spray_set_traj(self, *, poses: PoseArray | None = None, markers: MarkerArray | None = None) -> None:
        if self.spray is None:
            return
        if poses is not None:
            self.spray.publish_traj_poses(poses)
        if markers is not None:
            self.spray.publish_traj_markers(markers)

    def spray_set_executed(self, *, poses: PoseArray | None = None, markers: MarkerArray | None = None) -> None:
        if self.spray is None:
            return
        if poses is not None:
            self.spray.publish_executed_poses(poses)
        if markers is not None:
            self.spray.publish_executed_markers(markers)

    # Legacy wrappers (optional, keep old calls working)
    def set_spraypath(self, marker_array: MarkerArray) -> None:
        self.spray_set_compiled(markers=marker_array)

    def set_executed_path(self, pose_array: PoseArray) -> None:
        self.spray_set_executed(poses=pose_array)

    # Robot
    def robot_init(self) -> None:
        if self.robot is not None:
            self.robot.do_init()

    def robot_stop(self) -> None:
        if self.robot is not None:
            self.robot.do_stop()

    def robot_clear_error(self) -> None:
        if self.robot is not None:
            self.robot.do_clear_error()

    def robot_power_on(self) -> None:
        if self.robot is not None:
            self.robot.do_power_on()

    def robot_power_off(self) -> None:
        if self.robot is not None:
            self.robot.do_power_off()

    def robot_servo_on(self) -> None:
        if self.robot is not None:
            self.robot.do_servo_on()

    def robot_servo_off(self) -> None:
        if self.robot is not None:
            self.robot.do_servo_off()

    # MoveItPy
    def moveit_stop(self) -> None:
        if self.moveitpy is not None:
            self.moveitpy.signals.stopRequested.emit()

    def stop_all(self) -> None:
        self.moveit_stop()

    def ensure_connected(self) -> bool:
        """
        Legacy-API für UI-Tabs:
        - früher: Bridge musste evtl. connecten
        - jetzt: start() startet Executor + Nodes
        Diese Methode macht das robust + rückwärtskompatibel.

        Returns True wenn Bridge läuft, sonst False.
        """
        try:
            if not self.is_running:
                self.start()
            return bool(self.is_running)
        except Exception:
            _LOG.exception("ensure_connected failed")
            return False
