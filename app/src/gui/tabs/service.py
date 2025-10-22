#!/usr/bin/env python3
from __future__ import annotations

import os
import yaml
from typing import Optional, List, Tuple

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QMessageBox

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Point, Quaternion, TwistStamped
from moveit_msgs.msg import ServoStatus

from src.ros.common.topics import Topics
from src.ros.common.frames import FRAMES
from src.ros.clients.tool_client import ToolClient
from src.ros.clients.jogging_client import JoggingClient
from src.ros.clients.motion_client import MotionClient
from src.ros.clients.poses_client import PosesClient

# -------- Pfade (anpassen falls nötig) --------
APP_ROOT            = "/root/app"
STL_MOUNTS_DIR      = os.path.join(APP_ROOT, "resource", "stl", "substrate_mounts")
STL_SUBSTRATES_DIR  = os.path.join(APP_ROOT, "resource", "stl", "substrates")
STL_ENV_DIR         = os.path.join(APP_ROOT, "resource", "stl", "environment")
RECIPES_DIR         = os.path.join(APP_ROOT, "data", "recipes")
TOOLS_YAML_DEFAULT  = "/root/ws_moveit/src/mecademic_bringup/config/tools.yaml"  # Fallback

# -------- SceneManager-kompatible Topics (direkt) --------
TOPIC_MOUNT_LOAD        = "meca/mount/load"          # std_msgs/String
TOPIC_MOUNT_CURRENT     = "meca/mount/current"       # std_msgs/String (latched, falls vorhanden)
TOPIC_SUBSTRATE_LOAD    = "meca/workspace/load"      # std_msgs/String
TOPIC_SUBSTRATE_REMOVE  = "meca/workspace/remove"    # std_msgs/Empty
TOPIC_SUBSTRATE_CURRENT = "meca/workspace/current"   # std_msgs/String (latched, falls vorhanden)
TOPIC_ENV_LOAD          = "meca/environment/load"    # std_msgs/String (optional; wenn SceneManager es unterstützt)


def _list_files(directory: str, exts: Tuple[str, ...]) -> List[str]:
    try:
        files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
        files = [f for f in files if f.lower().endswith(exts)]
        files.sort()
        return files
    except Exception:
        return []


class ServiceTab(QWidget):
    """
    Service-Panel mit reinem Topic-Flow:
      - Status (Tool, Mount, Substrate, Servo, Joints, TCP)
      - Tool-Manager (Dropdown aus tools.yaml → /meca/tool/set)
      - Mount/Substrate/Environment via Dropdowns → SceneManager-Topics
      - Jogging (Step & Hold) per MoveIt Servo
      - Move/Teach (MoveIt IK/Controller)
      - Spray Path: YAML → PoseArray → /meca/spray_path/set | clear
    """

    def __init__(self, ros_node: Node, parent=None, *, tools_yaml: Optional[str] = None) -> None:
        super().__init__(parent)

        # UI laden
        ui_path = os.path.join(APP_ROOT, "src", "gui", "ui", "service_tab.ui")
        uic.loadUi(ui_path, self)

        # ROS
        self.node: Node = ros_node
        self.topics = Topics()
        self.frames = FRAMES

        # Tool / Jog / Motion / Poses (bleiben als Clients)
        self.tool   = ToolClient(self.node, self.topics)
        self.poses  = PosesClient(self.node, self.topics)
        self.jog    = JoggingClient(self.node, self.topics, default_frame=self.frames["meca_base"])
        self.motion = MotionClient(self.node, self.topics,
                                   group_name="meca_arm_group",
                                   ee_link_candidates=("tcp", "meca_axis_6_link", "tool0", "flange", None),
                                   pose_resolver=lambda name: self.poses.get(name))

        # Publisher (direkt)
        self.pub_mount_load       = self.node.create_publisher(String, TOPIC_MOUNT_LOAD, 10)
        self.pub_substrate_load   = self.node.create_publisher(String, TOPIC_SUBSTRATE_LOAD, 10)
        self.pub_substrate_remove = self.node.create_publisher(Empty,  TOPIC_SUBSTRATE_REMOVE, 10)
        self.pub_env_load         = self.node.create_publisher(String, TOPIC_ENV_LOAD, 10)

        self.pub_spray_set        = self.node.create_publisher(PoseArray, self.topics.spray_path_set, 10)
        self.pub_spray_clear      = self.node.create_publisher(Empty,     self.topics.spray_path_clear, 10)

        # Status-Subscriber
        self.sub_tool_current      = self.node.create_subscription(String, self.topics.tool_current,        self._on_tool_current, 10)
        self.sub_mount_current     = self.node.create_subscription(String, TOPIC_MOUNT_CURRENT,             self._on_mount_current, 10)
        self.sub_substrate_current = self.node.create_subscription(String, TOPIC_SUBSTRATE_CURRENT,         self._on_substrate_current, 10)
        self.sub_servo             = self.node.create_subscription(ServoStatus, self.topics.servo_status,   self._on_servo_status, 10)
        self.sub_joints            = self.node.create_subscription(JointState,   self.topics.joint_states,  self._on_joint_states, 10)

        # TCP-Status zyklisch (aus PosesClient "tcp")
        self._status_timer = QTimer(self)
        self._status_timer.setInterval(500)
        self._status_timer.timeout.connect(self._refresh_tcp_pose)
        self._status_timer.start()

        # UI Wire-ups
        self._wire_tool()
        self._wire_mount()
        self._wire_substrate()
        self._wire_environment()
        self._wire_jogging()
        self._wire_move()
        self._wire_spray()

        # Dropdowns initial füllen
        self._populate_tools(tools_yaml or TOOLS_YAML_DEFAULT)
        self._populate_mounts()
        self._populate_substrates()
        self._populate_envs()
        self._populate_recipes()

        # Hold-Steuerung
        self._hold_axis = None
        self._hold_timer = QTimer(self)
        self._hold_timer.setInterval(80)
        self._hold_timer.timeout.connect(self._tick_hold)

    # -------------------- Populate --------------------

    def _populate_tools(self, yaml_path: str) -> None:
        tools = []
        try:
            if os.path.isfile(yaml_path):
                with open(yaml_path, "r") as f:
                    data = yaml.safe_load(f) or {}
                if isinstance(data, dict) and "tools" in data and isinstance(data["tools"], list):
                    for it in data["tools"]:
                        n = it.get("name") if isinstance(it, dict) else str(it)
                        if n:
                            tools.append(n)
        except Exception:
            pass
        self.cbTool.clear()
        self.cbTool.addItems(tools)

    def _populate_mounts(self) -> None:
        self.cbMount.clear()
        self.cbMount.addItems(_list_files(STL_MOUNTS_DIR, (".stl",)))

    def _populate_substrates(self) -> None:
        self.cbSubstrate.clear()
        self.cbSubstrate.addItems(_list_files(STL_SUBSTRATES_DIR, (".stl",)))

    def _populate_envs(self) -> None:
        self.cbEnv.clear()
        self.cbEnv.addItems(_list_files(STL_ENV_DIR, (".stl",)))

    def _populate_recipes(self) -> None:
        self.cbSpray.clear()
        self.cbSpray.addItems(_list_files(RECIPES_DIR, (".yaml",)))

    # -------------------- Tool --------------------

    def _wire_tool(self) -> None:
        self.btnToolSet.clicked.connect(self._on_tool_set)

    def _on_tool_set(self) -> None:
        name = self.cbTool.currentText().strip()
        if not name:
            return
        ok = self.tool.set_tool(name, wait_for_confirm=True, timeout=3.0)
        if not ok:
            QMessageBox.warning(self, "Tool", f"Tool '{name}' konnte nicht gesetzt werden.")

    def _on_tool_current(self, msg: String) -> None:
        self.lblTool.setText(msg.data or "—")

    # -------------------- Mount --------------------

    def _wire_mount(self) -> None:
        self.btnMountLoad.clicked.connect(self._on_mount_load)

    def _on_mount_load(self) -> None:
        fn = self.cbMount.currentText().strip()
        if not fn:
            return
        path = os.path.join(STL_MOUNTS_DIR, fn)
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Mount", f"Datei nicht gefunden:\n{path}")
            return
        self.pub_mount_load.publish(String(data=path))

    def _on_mount_current(self, msg: String) -> None:
        self.lblMount.setText(os.path.basename(msg.data) if msg.data else "—")

    # -------------------- Substrate --------------------

    def _wire_substrate(self) -> None:
        self.btnSubstrateLoad.clicked.connect(self._on_substrate_load)
        self.btnSubstrateRemove.clicked.connect(self._on_substrate_remove)

    def _on_substrate_load(self) -> None:
        fn = self.cbSubstrate.currentText().strip()
        if not fn:
            return
        path = os.path.join(STL_SUBSTRATES_DIR, fn)
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Substrate", f"Datei nicht gefunden:\n{path}")
            return
        self.pub_substrate_load.publish(String(data=path))

    def _on_substrate_remove(self) -> None:
        self.pub_substrate_remove.publish(Empty())

    def _on_substrate_current(self, msg: String) -> None:
        self.lblSubstrate.setText(os.path.basename(msg.data) if msg.data else "—")

    # -------------------- Environment --------------------

    def _wire_environment(self) -> None:
        self.btnEnvLoad.clicked.connect(self._on_env_load)

    def _on_env_load(self) -> None:
        fn = self.cbEnv.currentText().strip()
        if not fn:
            return
        path = os.path.join(STL_ENV_DIR, fn)
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Environment", f"Datei nicht gefunden:\n{path}")
            return
        # Funktioniert nur, wenn dein SceneManager dieses Topic auswertet.
        self.pub_env_load.publish(String(data=path))

    # -------------------- Jogging --------------------

    def _wire_jogging(self) -> None:
        # Linear
        self.btnXminus.clicked.connect(lambda: self._step_lin(-1, 0, 0))
        self.btnXplus .clicked.connect(lambda: self._step_lin( 1, 0, 0))
        self.btnYminus.clicked.connect(lambda: self._step_lin( 0,-1, 0))
        self.btnYplus .clicked.connect(lambda: self._step_lin( 0, 1, 0))
        self.btnZminus.clicked.connect(lambda: self._step_lin( 0, 0,-1))
        self.btnZplus .clicked.connect(lambda: self._step_lin( 0, 0, 1))

        # Rot
        self.btnRxminus.clicked.connect(lambda: self._step_rot(-1, 0, 0))
        self.btnRxplus .clicked.connect(lambda: self._step_rot( 1, 0, 0))
        self.btnRyminus.clicked.connect(lambda: self._step_rot( 0,-1, 0))
        self.btnRyplus .clicked.connect(lambda: self._step_rot( 0, 1, 0))
        self.btnRzminus.clicked.connect(lambda: self._step_rot( 0, 0,-1))
        self.btnRzplus .clicked.connect(lambda: self._step_rot( 0, 0, 1))

        # Joints
        self.btnJ1minus.clicked.connect(lambda: self._step_joint(0, -1))
        self.btnJ1plus .clicked.connect(lambda: self._step_joint(0,  1))
        self.btnJ2minus.clicked.connect(lambda: self._step_joint(1, -1))
        self.btnJ2plus .clicked.connect(lambda: self._step_joint(1,  1))
        self.btnJ3minus.clicked.connect(lambda: self._step_joint(2, -1))
        self.btnJ3plus .clicked.connect(lambda: self._step_joint(2,  1))
        self.btnJ4minus.clicked.connect(lambda: self._step_joint(3, -1))
        self.btnJ4plus .clicked.connect(lambda: self._step_joint(3,  1))
        self.btnJ5minus.clicked.connect(lambda: self._step_joint(4, -1))
        self.btnJ5plus .clicked.connect(lambda: self._step_joint(4,  1))
        self.btnJ6minus.clicked.connect(lambda: self._step_joint(5, -1))
        self.btnJ6plus .clicked.connect(lambda: self._step_joint(5,  1))

        # Hold (kontinuierlich während gedrückt)
        self.btnHoldX.pressed.connect(lambda: self._start_hold(("lin", (1, 0, 0))))
        self.btnHoldX.released.connect(self._stop_hold)
        self.btnHoldY.pressed.connect(lambda: self._start_hold(("lin", (0, 1, 0))))
        self.btnHoldY.released.connect(self._stop_hold)
        self.btnHoldZ.pressed.connect(lambda: self._start_hold(("lin", (0, 0, 1))))
        self.btnHoldZ.released.connect(self._stop_hold)

    def _step_lin(self, sx: int, sy: int, sz: int) -> None:
        v = float(self.sbStepLin.value())
        self.jog.delta_twist(linear=(sx*v, sy*v, sz*v), angular=(0.0, 0.0, 0.0), frame=self.frames["meca_base"])

    def _step_rot(self, srx: int, sry: int, srz: int) -> None:
        v = float(self.sbStepRot.value())
        self.jog.delta_twist(linear=(0.0, 0.0, 0.0), angular=(srx*v, sry*v, srz*v), frame=self.frames["meca_base"])

    def _step_joint(self, idx: int, sign: int) -> None:
        v = float(self.sbStepRot.value())  # als Joint-Step (rad) wiederverwendet
        arr = [0.0]*6
        arr[idx] = sign * v
        self.jog.delta_joint(arr)

    def _start_hold(self, axis) -> None:
        self._hold_axis = axis
        self._hold_timer.start()

    def _stop_hold(self) -> None:
        self._hold_timer.stop()
        self._hold_axis = None
        # sauber stoppen & wieder aktivieren
        self.jog.disable(send_stop=True)
        self.jog.enable()

    def _tick_hold(self) -> None:
        if self._hold_axis is None:
            return
        mode, vec = self._hold_axis
        if mode == "lin":
            v = float(self.sbStepLin.value()) * 0.5
            # kleine kontinuierliche Schritte
            self.jog.delta_twist(linear=vec, angular=(0,0,0), scale=v, frame=self.frames["meca_base"])

    # -------------------- Move / Teach --------------------

    def _wire_move(self) -> None:
        self.btnMoveHome.clicked.connect(lambda: self._move_named("home"))
        self.btnMoveWorkspace.clicked.connect(lambda: self._move_named("workspace_center"))
        self.btnMovePredispense.clicked.connect(lambda: self._move_named("predispense"))
        self.btnMoveService.clicked.connect(lambda: self._move_named("service"))
        self.btnTeachFromTcp.clicked.connect(self._teach_from_tcp)

    def _move_named(self, name: str) -> None:
        ok = self.motion.move_to_named_pose(name, use_planner=False, plan_only=False, avoid_collisions=False)
        if not ok:
            QMessageBox.warning(self, "Move", f"Konnte Pose '{name}' nicht anfahren.")

    def _teach_from_tcp(self) -> None:
        name = self.leTeachName.text().strip()
        if not name:
            QMessageBox.information(self, "Teach", "Bitte einen Pose-Namen eingeben.")
            return
        ok = self.poses.set_from_tcp(name, wait_for_update=True, timeout=2.0)
        if not ok:
            QMessageBox.warning(self, "Teach", f"Pose '{name}' konnte nicht gesetzt werden.")

    # -------------------- Spray Path --------------------

    def _wire_spray(self) -> None:
        self.btnSprayLoad.clicked.connect(self._on_spray_load)
        self.btnSprayClear.clicked.connect(self._on_spray_clear)

    def _on_spray_load(self) -> None:
        fn = self.cbSpray.currentText().strip()
        if not fn:
            return
        path = os.path.join(RECIPES_DIR, fn)
        if not (os.path.isfile(path) and path.endswith(".yaml")):
            QMessageBox.warning(self, "Spray Path", f"Ungültige Datei:\n{path}")
            return
        try:
            pa = self._pose_array_from_yaml(path, frame=self.frames["workspace_center"])
            self.pub_spray_set.publish(pa)
        except Exception as e:
            QMessageBox.critical(self, "Spray Path", f"Fehler beim Laden:\n{e}")

    def _on_spray_clear(self) -> None:
        self.pub_spray_clear.publish(Empty())

    def _pose_array_from_yaml(self, yaml_path: str, *, frame: str) -> PoseArray:
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f) or {}

        poses = data.get("poses", data)
        if not isinstance(poses, list):
            raise ValueError("YAML erwartet eine Liste unter 'poses' oder eine Liste als Wurzel.")

        pa = PoseArray()
        pa.header.frame_id = frame

        for it in poses:
            p = Pose()
            # erlaubte Schlüssel: x,y,z,qx,qy,qz,qw (oder rx,ry,rz als Grad → optional später)
            x = float(it.get("x", 0.0)); y = float(it.get("y", 0.0)); z = float(it.get("z", 0.0))
            qx = float(it.get("qx", 0.0)); qy = float(it.get("qy", 0.0)); qz = float(it.get("qz", 0.0)); qw = float(it.get("qw", 1.0))
            p.position = Point(x=x, y=y, z=z)
            p.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
            pa.poses.append(p)

        if not pa.poses:
            raise ValueError("keine Posen in YAML gefunden.")
        return pa

    # -------------------- Status --------------------

    def _on_servo_status(self, msg: ServoStatus) -> None:
        self.lblServo.setText(str(msg.code))

    def _on_joint_states(self, msg: JointState) -> None:
        if not msg.position:
            return
        vals = [f"{p:.2f}" for p in msg.position[:6]]
        self.lblJoints.setText(", ".join(vals))

    def _refresh_tcp_pose(self) -> None:
        ps: Optional[PoseStamped] = self.poses.get("tcp")
        if ps is None:
            self.lblTcpPose.setText("—")
            return
        p = ps.pose.position; q = ps.pose.orientation
        self.lblTcpPose.setText(f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} | q=({q.x:.2f},{q.y:.2f},{q.z:.2f},{q.w:.2f})")
