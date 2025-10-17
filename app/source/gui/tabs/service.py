#!/usr/bin/env python3
from __future__ import annotations

import os
import json
import yaml
from typing import Optional, List, Tuple

from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import QWidget, QMessageBox

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import ServoStatus

from ros.common.topics import Topics
from ros.common.frames import FRAMES
from ros.clients.tool_client import ToolClient
from ros.clients.substrate_client import SubstrateClient
from ros.clients.mount_client import MountClient
from ros.clients.scene_client import SceneClient, PathStyle
from ros.clients.spray_path_client import SprayPathClient
from ros.clients.jogging_client import JoggingClient
from ros.clients.motion_client import MotionClient
from ros.clients.poses_client import PosesClient

from . import ui  # ensures resources if any; not strictly required
from PyQt5 import uic


APP_ROOT = "/root/app"
STL_MOUNTS_DIR      = os.path.join(APP_ROOT, "resource", "stl", "substrate_mounts")
STL_SUBSTRATES_DIR  = os.path.join(APP_ROOT, "resource", "stl", "substrates")
STL_ENV_DIR         = os.path.join(APP_ROOT, "resource", "stl", "environment")
RECIPES_DIR         = os.path.join(APP_ROOT, "data", "recipes")
TOOLS_YAML_DEFAULT  = "/root/ws_moveit/src/mecademic_bringup/config/tools.yaml"  # fallback, wenn keine app-config


def list_files(directory: str, exts: Tuple[str, ...]) -> List[str]:
    try:
        files = [f for f in os.listdir(directory) if os.path.isfile(os.path.join(directory, f))]
        files = [f for f in files if f.lower().endswith(exts)]
        files.sort()
        return files
    except Exception:
        return []


class ServiceTab(QWidget):
    """
    Komplettes Service-Panel:
     - Status oben (Tool, Mount, Substrate, Servo, Joints, TCP)
     - Tool Manager (Dropdown aus tools.yaml)
     - Mount/Substrate/Environment (Dropdowns aus Ordnern)
     - Jogging (Step & Hold)
     - Move/Posen (Home/Workspace/Predispense/Service + Teach)
     - Spray Path (Dropdown aus .yaml, Load/Clear -> Scene)
    """
    def __init__(self, ros_node: Node, parent=None, *, tools_yaml: Optional[str] = None):
        super().__init__(parent)

        # Load UI
        ui_path = os.path.join(APP_ROOT, "source", "gui", "ui", "service_tab.ui")
        uic.loadUi(ui_path, self)

        self.node: Node = ros_node
        self.topics = Topics()
        self.frames = FRAMES

        # ROS Clients
        self.tool = ToolClient(self.node, self.topics)
        self.poses = PosesClient(self.node, self.topics)
        self.scene = SceneClient(self.node, self.topics, default_frame=self.frames["workspace_center"], style=PathStyle(ns="recipe"))
        self.substrate = SubstrateClient(self.node, self.topics, search_dirs=[STL_SUBSTRATES_DIR])
        self.mount = MountClient(self.node, self.topics, search_dirs=[STL_MOUNTS_DIR])
        self.spray = SprayPathClient(self.node, self.topics)
        self.jog = JoggingClient(self.node, self.topics, default_frame=self.frames["meca_base"])
        self.motion = MotionClient(self.node, self.topics, group_name="meca_arm_group",
                                   ee_link_candidates=("tcp", "meca_axis_6_link", "tool0", "flange", None),
                                   pose_resolver=self._pose_resolver)

        # Status subscribers
        self._sub_tool_current = self.node.create_subscription(String, self.topics.tool_current, self._on_tool_current, 10)
        # Diese Topics erwarten wir latched vom SceneManager:
        self._sub_mount_current = self.node.create_subscription(String, self.topics.mount_current, self._on_mount_current, 10)
        self._sub_substrate_current = self.node.create_subscription(String, self.topics.workspace_current, self._on_substrate_current, 10)

        self._sub_servo = self.node.create_subscription(ServoStatus, self.topics.servo_status, self._on_servo_status, 10)
        self._sub_joints = self.node.create_subscription(JointState, self.topics.joint_states, self._on_joint_states, 10)

        # TCP Pose – holen wir zyklisch über PosesClient ("tcp" Pose)
        self.status_timer = QTimer(self)
        self.status_timer.setInterval(500)
        self.status_timer.timeout.connect(self._refresh_tcp_pose)
        self.status_timer.start()

        # UI hookups
        self._wire_tool()
        self._wire_mount()
        self._wire_substrate()
        self._wire_environment()
        self._wire_jogging()
        self._wire_move()
        self._wire_spray()

        # Initial Data
        self._populate_tools(tools_yaml or TOOLS_YAML_DEFAULT)
        self._populate_mounts()
        self._populate_substrates()
        self._populate_envs()
        self._populate_recipes()

        # Internal
        self._hold_timer = QTimer(self)
        self._hold_timer.setInterval(80)
        self._hold_timer.timeout.connect(self._tick_hold)
        self._hold_axis = None  # ('lin', (x,y,z)) or ('rot', (rx,ry,rz)) or ('joint', idx, sign)

        # Seed status from latched topics fast
        # (No direct call needed; latched subs will populate labels shortly.)

    # ------------- Populate Dropdowns -------------
    def _populate_tools(self, yaml_path: str):
        tools = []
        try:
            if os.path.isfile(yaml_path):
                with open(yaml_path, "r") as f:
                    data = yaml.safe_load(f) or {}
                # Schema: tools: [{name: ...}, ...] oder dict
                if isinstance(data, dict) and "tools" in data:
                    items = data["tools"]
                    if isinstance(items, list):
                        for it in items:
                            n = it.get("name") if isinstance(it, dict) else str(it)
                            if n:
                                tools.append(n)
        except Exception:
            pass
        self.cbTool.clear()
        self.cbTool.addItems(tools)

    def _populate_mounts(self):
        files = list_files(STL_MOUNTS_DIR, (".stl",))
        self.cbMount.clear()
        self.cbMount.addItems(files)

    def _populate_substrates(self):
        files = list_files(STL_SUBSTRATES_DIR, (".stl",))
        self.cbSubstrate.clear()
        self.cbSubstrate.addItems(files)

    def _populate_envs(self):
        files = list_files(STL_ENV_DIR, (".stl",))
        self.cbEnv.clear()
        self.cbEnv.addItems(files)

    def _populate_recipes(self):
        files = list_files(RECIPES_DIR, (".yaml",))
        self.cbSpray.clear()
        self.cbSpray.addItems(files)

    # ------------- Tool Manager -------------
    def _wire_tool(self):
        self.btnToolSet.clicked.connect(self._on_tool_set)

    def _on_tool_set(self):
        name = self.cbTool.currentText().strip()
        if not name:
            return
        ok = self.tool.set_tool(name, wait_for_confirm=True, timeout=3.0)
        if not ok:
            QMessageBox.warning(self, "Tool", f"Tool '{name}' konnte nicht gesetzt werden.")
        # ToolManager publisht current -> lblTool via _on_tool_current

    def _on_tool_current(self, msg: String):
        self.lblTool.setText(msg.data or "—")

    # ------------- Mount -------------
    def _wire_mount(self):
        self.btnMountLoad.clicked.connect(self._on_mount_load)

    def _on_mount_load(self):
        file = self.cbMount.currentText().strip()
        if not file:
            return
        path = os.path.join(STL_MOUNTS_DIR, file)
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Mount", f"Datei nicht gefunden:\n{path}")
            return
        self.mount.load(path)  # SceneManager wechselt Mount & setzt workspace_center

    def _on_mount_current(self, msg: String):
        self.lblMount.setText(os.path.basename(msg.data) if msg.data else "—")

    # ------------- Substrate -------------
    def _wire_substrate(self):
        self.btnSubstrateLoad.clicked.connect(self._on_substrate_load)
        self.btnSubstrateRemove.clicked.connect(self._on_substrate_remove)

    def _on_substrate_load(self):
        file = self.cbSubstrate.currentText().strip()
        if not file:
            return
        path = os.path.join(STL_SUBSTRATES_DIR, file)
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Substrate", f"Datei nicht gefunden:\n{path}")
            return
        self.substrate.load(path, must_exist=True)

    def _on_substrate_remove(self):
        self.substrate.remove()

    def _on_substrate_current(self, msg: String):
        self.lblSubstrate.setText(os.path.basename(msg.data) if msg.data else "—")

    # ------------- Environment -------------
    def _wire_environment(self):
        self.btnEnvLoad.clicked.connect(self._on_env_load)

    def _on_env_load(self):
        file = self.cbEnv.currentText().strip()
        if not file:
            return
        path = os.path.join(STL_ENV_DIR, file)
        if not os.path.isfile(path):
            QMessageBox.warning(self, "Environment", f"Datei nicht gefunden:\n{path}")
            return
        # Environment wird im SceneManager per /meca/environment/load gesetzt
        # Nutzen wir SceneClient nicht, sondern MountClient? -> separater Client ist nicht nötig:
        # Wir senden einfach String auf Topic:
        pub = self.node.create_publisher(String, self.topics.environment_load, 10)
        pub.publish(String(data=path))

    # ------------- Jogging (Step + Hold) -------------
    def _wire_jogging(self):
        # Step linear
        self.btnXminus.clicked.connect(lambda: self._step_lin(-1, 0, 0))
        self.btnXplus.clicked.connect(lambda: self._step_lin(1, 0, 0))
        self.btnYminus.clicked.connect(lambda: self._step_lin(0, -1, 0))
        self.btnYplus.clicked.connect(lambda: self._step_lin(0, 1, 0))
        self.btnZminus.clicked.connect(lambda: self._step_lin(0, 0, -1))
        self.btnZplus.clicked.connect(lambda: self._step_lin(0, 0, 1))

        # Step rot
        self.btnRxminus.clicked.connect(lambda: self._step_rot(-1, 0, 0))
        self.btnRxplus.clicked.connect(lambda: self._step_rot(1, 0, 0))
        self.btnRyminus.clicked.connect(lambda: self._step_rot(0, -1, 0))
        self.btnRyplus.clicked.connect(lambda: self._step_rot(0, 1, 0))
        self.btnRzminus.clicked.connect(lambda: self._step_rot(0, 0, -1))
        self.btnRzplus.clicked.connect(lambda: self._step_rot(0, 0, 1))

        # Step joints
        self.btnJ1minus.clicked.connect(lambda: self._step_joint(0, -1))
        self.btnJ1plus.clicked.connect(lambda: self._step_joint(0, 1))
        self.btnJ2minus.clicked.connect(lambda: self._step_joint(1, -1))
        self.btnJ2plus.clicked.connect(lambda: self._step_joint(1, 1))
        self.btnJ3minus.clicked.connect(lambda: self._step_joint(2, -1))
        self.btnJ3plus.clicked.connect(lambda: self._step_joint(2, 1))
        self.btnJ4minus.clicked.connect(lambda: self._step_joint(3, -1))
        self.btnJ4plus.clicked.connect(lambda: self._step_joint(3, 1))
        self.btnJ5minus.clicked.connect(lambda: self._step_joint(4, -1))
        self.btnJ5plus.clicked.connect(lambda: self._step_joint(4, 1))
        self.btnJ6minus.clicked.connect(lambda: self._step_joint(5, -1))
        self.btnJ6plus.clicked.connect(lambda: self._step_joint(5, 1))

        # Hold buttons (simple: toggled while pressed)
        self.btnHoldX.pressed.connect(lambda: self._start_hold(("lin", (1, 0, 0))))
        self.btnHoldX.released.connect(self._stop_hold)
        self.btnHoldY.pressed.connect(lambda: self._start_hold(("lin", (0, 1, 0))))
        self.btnHoldY.released.connect(self._stop_hold)
        self.btnHoldZ.pressed.connect(lambda: self._start_hold(("lin", (0, 0, 1))))
        self.btnHoldZ.released.connect(self._stop_hold)

    def _step_lin(self, sx: int, sy: int, sz: int):
        v = float(self.sbStepLin.value())
        self.jog.delta_twist(linear=(sx*v, sy*v, sz*v), angular=(0.0, 0.0, 0.0), frame=self.frames["meca_base"])

    def _step_rot(self, srx: int, sry: int, srz: int):
        v = float(self.sbStepRot.value())
        self.jog.delta_twist(linear=(0.0, 0.0, 0.0), angular=(srx*v, sry*v, srz*v), frame=self.frames["meca_base"])

    def _step_joint(self, idx: int, sign: int):
        v = float(self.sbStepRot.value())  # reuse rotational step as joint step (rad)
        arr = [0.0]*6
        arr[idx] = sign * v
        self.jog.delta_joint(arr)

    def _start_hold(self, axis):
        self._hold_axis = axis
        self._hold_timer.start()

    def _stop_hold(self):
        self._hold_timer.stop()
        self._hold_axis = None
        # send stop to servo
        self.jog.disable(send_stop=True)
        self.jog.enable()

    def _tick_hold(self):
        if self._hold_axis is None:
            return
        mode, vec = self._hold_axis
        if mode == "lin":
            v = float(self.sbStepLin.value()) * 0.5  # smaller continuous
            self.jog.delta_twist(linear=vec, angular=(0.0, 0.0, 0.0), scale=v, frame=self.frames["meca_base"])
        # could add rot/joint hold similarly if needed

    # ------------- Move / Posen -------------
    def _wire_move(self):
        self.btnMoveHome.clicked.connect(lambda: self._move_named("home"))
        self.btnMoveWorkspace.clicked.connect(lambda: self._move_named("workspace_center"))
        self.btnMovePredispense.clicked.connect(lambda: self._move_named("predispense"))
        self.btnMoveService.clicked.connect(lambda: self._move_named("service"))
        self.btnTeachFromTcp.clicked.connect(self._teach_from_tcp)

    def _move_named(self, name: str):
        ok = self.motion.move_to_named_pose(name, use_planner=False, plan_only=False, avoid_collisions=False)
        if not ok:
            QMessageBox.warning(self, "Move", f"Konnte Pose '{name}' nicht anfahren.")

    def _teach_from_tcp(self):
        name = self.leTeachName.text().strip()
        if not name:
            QMessageBox.information(self, "Teach", "Bitte einen Pose-Namen eingeben.")
            return
        ok = self.poses.set_from_tcp(name, wait_for_update=True, timeout=2.0)
        if not ok:
            QMessageBox.warning(self, "Teach", f"Pose '{name}' konnte nicht gesetzt werden.")

    # ------------- Spray Path -------------
    def _wire_spray(self):
        self.btnSprayLoad.clicked.connect(self._on_spray_load)
        self.btnSprayClear.clicked.connect(self._on_spray_clear)

    def _on_spray_load(self):
        file = self.cbSpray.currentText().strip()
        if not file:
            return
        path = os.path.join(RECIPES_DIR, file)
        if not os.path.isfile(path) or not path.endswith(".yaml"):
            QMessageBox.warning(self, "Spray Path", f"Ungültige Datei:\n{path}")
            return
        # Datei lesen, PoseArray erzeugen und senden
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            # Erwartetes Schema: list von Posen [{x,y,z,qx,qy,qz,qw}, ...] oder {poses: [...]}
            poses = data.get("poses", data)
            pa = self.spray.make_pose_array_from_list(poses, frame_id=self.frames["workspace_center"])
            self.spray.set_path(pa)   # publisht /meca/spray_path/set
        except Exception as e:
            QMessageBox.critical(self, "Spray Path", f"Fehler beim Laden:\n{e}")

    def _on_spray_clear(self):
        self.spray.clear_path()

    # ------------- Status handling -------------
    def _on_servo_status(self, msg: ServoStatus):
        # Simple: show code
        self.lblServo.setText(str(msg.code))

    def _on_joint_states(self, msg: JointState):
        if not msg.position:
            return
        # Kurzformat: 6 joints auf 1 Nachkommastelle
        vals = [f"{p:.2f}" for p in msg.position[:6]]
        self.lblJoints.setText(", ".join(vals))

    def _refresh_tcp_pose(self):
        ps: Optional[PoseStamped] = self.poses.get("tcp")
        if ps is None:
            self.lblTcpPose.setText("—")
            return
        p = ps.pose.position
        q = ps.pose.orientation
        self.lblTcpPose.setText(f"x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} | q=({q.x:.2f},{q.y:.2f},{q.z:.2f},{q.w:.2f})")

    # ------------- Helpers -------------
    def _pose_resolver(self, name: str) -> Optional[PoseStamped]:
        # For MotionClient
        return self.poses.get(name)
