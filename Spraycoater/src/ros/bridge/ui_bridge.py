# -*- coding: utf-8 -*-
# File: src/ros/bridge/ui_bridge.py
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

from std_msgs.msg import String as MsgString
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg
from trajectory_msgs.msg import JointTrajectory

_LOG = logging.getLogger("ros.ui_bridge")


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
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._current_name: str = ""
        self._poses: Optional[PoseArray] = None

    def _set_current_name(self, name: str) -> None:
        with self._lock:
            self._current_name = name

    def _set_poses(self, pa: PoseArray) -> None:
        with self._lock:
            self._poses = pa

    def current_name(self) -> str:
        with self._lock:
            return self._current_name

    def poses(self) -> Optional[PoseArray]:
        with self._lock:
            return self._poses


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


class UIBridge:
    """
    Clean Contract:
      - genau eine MoveIt-Bridge: MoveItPyBridge
      - keine Fallbacks / keine Aliase
    """

    def __init__(self, startup_yaml_path: Optional[str] = None, namespace: str = ""):
        self._startup_yaml = startup_yaml_path or os.environ["SC_STARTUP_YAML"]
        self._namespace = namespace.strip().strip("/")

        self._bridge: Optional[RosBridge] = None

        # States
        self.scene = SceneState()
        self.poses = PosesState()
        self.spraypath = SprayPathState()
        self.robot = RobotState()
        self.moveit = MoveItState()

        # Nodes
        self._scene: Optional[SceneBridge] = None
        self._poses: Optional[PosesBridge] = None
        self._spray: Optional[SprayPathBridge] = None
        self._servo: Optional[ServoBridge] = None
        self._robot: Optional[RobotBridge] = None
        self._moveitpy: Optional[MoveItPyBridge] = None

        # optional
        self._omron: Optional[OmronBridge] = None

    @property
    def namespace(self) -> str:
        return self._namespace

    @property
    def is_connected(self) -> bool:
        return (
            self._bridge is not None
            and self._scene is not None
            and self._poses is not None
            and self._spray is not None
            and self._servo is not None
            and self._robot is not None
            and self._moveitpy is not None
        )

    @property
    def scene_bridge(self) -> SceneBridge:
        assert self._scene is not None
        return self._scene

    @property
    def poses_bridge(self) -> PosesBridge:
        assert self._poses is not None
        return self._poses

    @property
    def spray_path_bridge(self) -> SprayPathBridge:
        assert self._spray is not None
        return self._spray

    @property
    def servo_bridge(self) -> ServoBridge:
        assert self._servo is not None
        return self._servo

    @property
    def robot_bridge(self) -> RobotBridge:
        assert self._robot is not None
        return self._robot

    @property
    def moveitpy_bridge(self) -> MoveItPyBridge:
        assert self._moveitpy is not None
        return self._moveitpy

    @property
    def omron_bridge(self) -> Optional[OmronBridge]:
        return self._omron

    # ---------- Lifecycle ----------

    def ensure_connected(self) -> None:
        if not self.is_connected:
            self.connect()

    def connect(self) -> None:
        if self.is_connected:
            return

        if self._bridge is None:
            self._bridge = RosBridge(startup_yaml_path=self._startup_yaml, namespace=self._namespace)
            self._bridge.start()

        def _must_get(cls, label: str):
            node = self._bridge.get_node(cls) if self._bridge is not None else None
            if node is None:
                raise RuntimeError(f"{label} nicht gefunden.")
            return node

        self._scene = _must_get(SceneBridge, "SceneBridge")
        self._poses = _must_get(PosesBridge, "PosesBridge")
        self._spray = _must_get(SprayPathBridge, "SprayPathBridge")
        self._servo = _must_get(ServoBridge, "ServoBridge")
        self._robot = _must_get(RobotBridge, "RobotBridge")
        self._moveitpy = _must_get(MoveItPyBridge, "MoveItPyBridge")

        self._wire_scene_into_state(self._scene)
        self._wire_poses_into_state(self._poses)
        self._wire_spraypath_into_state(self._spray)
        self._wire_robot_into_state(self._robot)
        self._wire_moveit_into_state(self._moveitpy)

        self._omron = self._bridge.get_node(OmronBridge)

        # Cache re-emit
        self._scene.signals.reemit_cached()
        self._poses.signals.reemit_cached()
        self._spray.signals.reemit_cached()
        self._robot.signals.reemit_cached()
        self._moveitpy.signals.reemit_cached()
        if self._omron is not None:
            self._omron.signals.reemit_cached()

        _LOG.info("UIBridge connected (namespace=%r)", self._namespace)

    def disconnect(self) -> None:
        if self._bridge is None:
            return
        self._bridge.stop()
        self._bridge = None
        self._scene = None
        self._poses = None
        self._spray = None
        self._servo = None
        self._robot = None
        self._moveitpy = None
        self._omron = None

    def shutdown(self) -> None:
        self.disconnect()

    # ---------- Scene: Set-API ----------
    def set_cage(self, name: str) -> None:
        self.scene_bridge.set_cage(name)

    def set_mount(self, name: str) -> None:
        self.scene_bridge.set_mount(name)

    def set_substrate(self, name: str) -> None:
        self.scene_bridge.set_substrate(name)

    # ---------- Poses: Set-API ----------
    def set_pose_by_name(self, name: str) -> None:
        self.poses_bridge.set_pose_by_name(name)

    def set_home(self) -> None:
        self.set_pose_by_name("home")

    def set_service(self) -> None:
        self.set_pose_by_name("service")

    # ---------- SprayPath: Set-API ----------
    def set_spraypath(self, marker_array: MarkerArray) -> None:
        self.spray_path_bridge.publish_set(marker_array)

    def set_executed_path(self, pose_array: PoseArray) -> None:
        self.spray_path_bridge.publish_executed_path(pose_array)

    # ---------- Robot: Set-API ----------
    def robot_init(self) -> None:
        self.robot_bridge.do_init()

    def robot_stop(self) -> None:
        self.robot_bridge.do_stop()

    def robot_clear_error(self) -> None:
        self.robot_bridge.do_clear_error()

    def robot_power_on(self) -> None:
        self.robot_bridge.do_power_on()

    def robot_power_off(self) -> None:
        self.robot_bridge.do_power_off()

    def robot_servo_on(self) -> None:
        self.robot_bridge.do_servo_on()

    def robot_servo_off(self) -> None:
        self.robot_bridge.do_servo_off()

    # ---------- Servo: FULL API ----------
    def servo_set_command_type(self, mode: str) -> None:
        self.servo_bridge.signals.modeChanged.emit(mode)

    def servo_set_frame(self, frame_ui: str) -> None:
        self.servo_bridge.signals.frameChanged.emit(frame_ui)

    def servo_set_params(self, cfg: dict) -> None:
        self.servo_bridge.signals.paramsChanged.emit(cfg or {})

    def servo_joint_jog(self, joint_name: str, delta_deg: float, speed_pct: Optional[float] = None) -> None:
        if speed_pct is None:
            speed_pct = 40.0
        self.servo_bridge.signals.jointJogRequested.emit(str(joint_name), float(delta_deg), float(speed_pct))

    def servo_cartesian_jog(self, axis: str, delta: float, speed: Optional[float] = None, frame_ui: str = "wrf") -> None:
        if speed is None:
            speed = 50.0
        self.servo_bridge.signals.cartesianJogRequested.emit(str(axis), float(delta), float(speed), str(frame_ui))

    # ---------- MoveItPy: High-Level-API ----------
    def moveit_move_home(self) -> None:
        self.moveitpy_bridge.signals.moveToHomeRequested.emit()

    def moveit_move_service(self) -> None:
        self.moveitpy_bridge.signals.moveToServiceRequested.emit()

    def moveit_move_to_pose(self, pose: PoseStamped) -> None:
        self.moveitpy_bridge.signals.moveToPoseRequested.emit(pose)

    def moveit_stop(self) -> None:
        """Stop an MoveItPy weiterreichen (Topic moveit_py/stop)."""
        self.moveitpy_bridge.signals.stopRequested.emit()

    def moveit_last_result(self) -> str:
        return self.moveitpy_bridge.signals.last_result

    def moveit_target_trajectory(self) -> Optional[JointTrajectory]:
        return self.moveit.target()

    def moveit_planned_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self.moveit.planned()

    def moveit_executed_trajectory(self) -> Optional[RobotTrajectoryMsg]:
        return self.moveit.executed()

    def moveit_preview_markers(self) -> Optional[MarkerArray]:
        return self.moveit.preview()

    def moveit_robot_description_semantic(self) -> str:
        return self.moveit.srdf()

    # ---------- Zentraler Stop ----------
    def stop(self) -> None:
        """Zentraler Stop für die UI: stoppt MoveItPy-Ausführung."""
        self.moveit_stop()

    # ---------- Wiring: Qt-Signale -> States ----------
    def _wire_scene_into_state(self, sb: SceneBridge) -> None:
        sig = sb.signals
        sig.cageListChanged.connect(self.scene._set_cage_list)
        sig.mountListChanged.connect(self.scene._set_mount_list)
        sig.substrateListChanged.connect(self.scene._set_substrate_list)
        sig.cageCurrentChanged.connect(self.scene._set_cage_current)
        sig.mountCurrentChanged.connect(self.scene._set_mount_current)
        sig.substrateCurrentChanged.connect(self.scene._set_substrate_current)

    def _wire_poses_into_state(self, pb: PosesBridge) -> None:
        sig = pb.signals
        sig.homePoseChanged.connect(self.poses._set_home)
        sig.servicePoseChanged.connect(self.poses._set_service)

    def _wire_spraypath_into_state(self, spb: SprayPathBridge) -> None:
        sig = spb.signals
        sig.currentNameChanged.connect(self.spraypath._set_current_name)
        sig.posesChanged.connect(self.spraypath._set_poses)

    def _wire_robot_into_state(self, rb: RobotBridge) -> None:
        sig = rb.signals
        sig.connectionChanged.connect(self.robot._set_connection)
        sig.modeChanged.connect(self.robot._set_mode)
        sig.initializedChanged.connect(self.robot._set_initialized)
        sig.movingChanged.connect(self.robot._set_moving)
        sig.servoEnabledChanged.connect(self.robot._set_servo_enabled)
        sig.powerChanged.connect(self.robot._set_power)
        sig.estopChanged.connect(self.robot._set_estop)
        sig.errorsChanged.connect(self.robot._set_errors)
        sig.tcpPoseChanged.connect(self.robot._set_tcp_pose)
        sig.jointsChanged.connect(self.robot._set_joints)

    def _wire_moveit_into_state(self, mb: MoveItPyBridge) -> None:
        sig = mb.signals
        sig.targetTrajectoryChanged.connect(self.moveit._set_target)
        sig.plannedTrajectoryChanged.connect(self.moveit._set_planned)
        sig.executedTrajectoryChanged.connect(self.moveit._set_executed)
        sig.previewMarkersChanged.connect(self.moveit._set_preview)
        sig.robotDescriptionSemanticChanged.connect(self.moveit._set_srdf)


def get_ui_bridge(_ctx, namespace: str = "") -> UIBridge:
    b = UIBridge(namespace=namespace)
    b.ensure_connected()
    return b
