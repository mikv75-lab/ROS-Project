# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from typing import Optional, Dict, Any, TYPE_CHECKING

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QSizePolicy, QLabel
)

from app.model.recipe.recipe_store import RecipeStore
from app.widgets.robot_command_box import RobotCommandButtonsBox
from app.widgets.robot_status_box import RobotStatusInfoBox
from app.widgets.omron_tcp_widget import OmronTcpWidget

from .scene_box import SceneGroupBox
from .poses_box import PosesGroupBox
from .tool_box import ToolGroupBox
from .moveitpy_widget import MoveItPyWidget
from .servo_widgets import JointJogWidget, CartesianJogWidget

if TYPE_CHECKING:
    from ros.bridge.ui_bridge import UIBridge

_LOG = logging.getLogger("app.tabs.service.robot")


class ServiceRobotTab(QWidget):
    """
    Service-Tab für Roboterfunktionen (Jogging, Posen, Scene, Omron-TCP).

    Contract:
      - bridge darf None sein -> disabled UI
      - sonst: bridge ist UIBridge und connected (wir rufen ensure_connected() hier einmal)
    """

    def __init__(self, *, ctx, store: RecipeStore, bridge: Optional["UIBridge"], parent: Optional[QWidget] = None):
        super().__init__(parent)

        self.ctx = ctx
        self.store = store
        self.bridge = bridge

        self._rb = None   # RobotBridge (konkret)
        self._sig = None  # RobotBridge.signals

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # Wenn keine Bridge da ist: minimaler Hinweis + return (kein Crash)
        if self.bridge is None:
            hint = QLabel("ROS-Bridge nicht verbunden (shadow/live fehlgeschlagen). Robot-Tab ist deaktiviert.", self)
            hint.setStyleSheet("color: orange; font-weight: 600;")
            root.addWidget(hint)
            root.addStretch(1)
            return

        # harter Vertrag: connected + Nodes vorhanden
        self.bridge.ensure_connected()
        self._rb = self.bridge.robot_bridge
        self._sig = self._rb.signals

        # ================================================================
        # [0] TOP ROW: COMMANDS + STATUS + OMRON TCP
        # ================================================================
        topRow = QHBoxLayout()
        topRow.setSpacing(8)

        self.commandBox = RobotCommandButtonsBox(self, title="Commands")
        self.statusBox = RobotStatusInfoBox(self, title="Robot Status")
        self.omronWidget = OmronTcpWidget(self.bridge, self)

        for w in (self.commandBox, self.statusBox, self.omronWidget):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        topRow.addWidget(self.commandBox, 1)
        topRow.addWidget(self.statusBox, 2)
        topRow.addWidget(self.omronWidget, 2)
        root.addLayout(topRow)

        # ================================================================
        # [2] BOTTOM ROW: Poses | Tools | Scene
        # ================================================================
        self.posesBox = PosesGroupBox(self.bridge, self)
        self.toolBox = ToolGroupBox(self.bridge, self)
        self.sceneBox = SceneGroupBox(self.bridge, self)

        rowBottom = QHBoxLayout()
        rowBottom.setSpacing(8)
        rowBottom.addWidget(self.posesBox)
        rowBottom.addWidget(self.toolBox)
        rowBottom.addWidget(self.sceneBox)
        root.addLayout(rowBottom)

        # ================================================================
        # [1] SUBTABS: MoveItPy | Joint Jog | Cartesian Jog
        # ================================================================
        self.tabs = QTabWidget(self)
        root.addWidget(self.tabs)

        self.moveitpyWidget = MoveItPyWidget(store=self.store, bridge=self.bridge, parent=self.tabs)
        self.tabs.addTab(self.moveitpyWidget, "Motion")

        self.jointJogWidget = JointJogWidget(self.ctx, self.bridge, self.tabs)
        self.tabs.addTab(self.jointJogWidget, "Joint Jog")

        self.cartJogWidget = CartesianJogWidget(self.ctx, self.bridge, self.tabs)
        self.tabs.addTab(self.cartJogWidget, "Cartesian Jog")

        for w in (
            self.moveitpyWidget,
            self.jointJogWidget,
            self.cartJogWidget,
            self.posesBox,
            self.toolBox,
            self.sceneBox,
        ):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        # wiring
        self._wire_bridge_inbound()
        self._wire_outbound()

        self.tabs.currentChanged.connect(self._on_tab_changed)
        self._on_tab_changed(self.tabs.currentIndex())

    # ==================================================================
    # Bridge INBOUND: ROS → UI
    # ==================================================================
    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        if js is None or not hasattr(js, "position"):
            self.statusBox.set_joints(None)
            return
        self.statusBox.set_joints(list(js.position or []))

    def _wire_bridge_inbound(self) -> None:
        sig = self._sig
        sb = self.statusBox

        sig.connectionChanged.connect(sb.set_connection)
        sig.modeChanged.connect(sb.set_mode)
        sig.initializedChanged.connect(sb.set_initialized)
        sig.movingChanged.connect(sb.set_moving)
        sig.powerChanged.connect(sb.set_power)
        sig.servoEnabledChanged.connect(sb.set_servo_enabled)
        sig.estopChanged.connect(sb.set_estop)
        sig.errorsChanged.connect(sb.set_errors)
        sig.tcpPoseChanged.connect(sb.set_tcp_from_ps)
        sig.jointsChanged.connect(self._on_joints)

    # ==================================================================
    # Bridge OUTBOUND: UI → ROS
    # ==================================================================
    def _wire_outbound(self) -> None:
        # du kannst hier entweder direkt die Signals nutzen (wie bisher)
        # oder die UIBridge wrapper (robot_init(), robot_stop(), ...).
        # Ich lasse es im exakt gleichen Stil wie du es hattest – nur korrekt referenziert.
        sig = self._sig
        cb = self.commandBox

        cb.initRequested.connect(sig.initRequested.emit)
        cb.stopRequested.connect(sig.stopRequested.emit)
        cb.clearErrorRequested.connect(sig.clearErrorRequested.emit)
        cb.powerOnRequested.connect(sig.powerOnRequested.emit)
        cb.powerOffRequested.connect(sig.powerOffRequested.emit)
        cb.servoEnableRequested.connect(sig.servoEnableRequested.emit)
        cb.servoDisableRequested.connect(sig.servoDisableRequested.emit)

    # ==================================================================
    # Tab-Wechsel → Servo an + Command-Modus setzen
    # ==================================================================
    def _on_tab_changed(self, idx: int) -> None:
        page = self.tabs.widget(idx)

        # Hard contract: bridge ist da und connected
        self.bridge.robot_servo_on()

        if page is self.jointJogWidget:
            mode = "joint"
        elif page is self.cartJogWidget:
            mode = "cart"
        else:
            mode = "joint"

        self.bridge.servo_set_command_type(mode)

    # ==================================================================
    # Forwarder-API
    # ==================================================================
    def get_planner(self) -> str:
        return self.moveitpyWidget.get_planner()

    def get_planner_params(self) -> Dict[str, Any]:
        return self.moveitpyWidget.get_params()

    def set_planner(self, name: str) -> None:
        self.moveitpyWidget.set_planner(name)

    def set_planner_params(self, cfg: Dict[str, Any]) -> None:
        self.moveitpyWidget.set_params(cfg)

    def get_joint_jog_params(self) -> Dict[str, float]:
        return self.jointJogWidget.get_params()

    def set_joint_jog_params(self, cfg: Dict[str, float]) -> None:
        self.jointJogWidget.set_params(cfg)

    def get_cart_jog_params(self) -> Dict[str, float]:
        return self.cartJogWidget.get_params()

    def set_cart_jog_params(self, cfg: Dict[str, float]) -> None:
        self.cartJogWidget.set_params(cfg)
