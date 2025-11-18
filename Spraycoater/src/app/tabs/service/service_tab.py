# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QSizePolicy
)

# RecipeStore für planner- und defaultsgetriebene Widgets
from app.model.recipe.recipe_store import RecipeStore

# Neue, getrennte Widgets
from ...widgets.robot_command_box import RobotCommandButtonsBox
from ...widgets.robot_status_box import RobotStatusInfoBox

# Service-Widgets
from .scene_box import SceneGroupBox
from .poses_box import PosesGroupBox
from .tool_box import ToolGroupBox
from .motion_widget import MotionWidget
from .servo_widgets import JointJogWidget, CartesianJogWidget

_LOG = logging.getLogger("app.tabs.service")


class ServiceTab(QWidget):
    """
    Service-Tab Layout (nur Code, keine .ui):

      [0] Row:
            ├── RobotCommandButtonsBox (links)
            └── RobotStatusInfoBox    (rechts)
      [1] Tabs:
            ├── Motion (Planner + Motion-Speed)
            ├── Joint Jog
            └── Cartesian Jog
      [2] Row(HBox): PosesGroupBox | ToolGroupBox | SceneGroupBox
      [3] Spacer (Expanding)

    Zusätzlich:
      - Beim Tab-Wechsel wird der Servo eingeschaltet.
      - Beim Tab-Wechsel wird der Servo-Command-Type gesetzt:
          Motion      -> JOINT_JOG (Default)
          Joint Jog   -> JOINT_JOG
          Cartesian   -> TWIST
    """

    def __init__(self, *, ctx, store: RecipeStore, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bridge = bridge

        # RobotBridge + Signals besorgen
        self._rb = getattr(self.bridge, "_rb", None)
        self._sig = getattr(self._rb, "signals", None) if self._rb else None

        # Root-Layout
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- [0] Command (links) + Status (rechts) --------------------------
        topRow = QHBoxLayout()
        topRow.setContentsMargins(0, 0, 0, 0)
        topRow.setSpacing(8)

        self.commandBox = RobotCommandButtonsBox(self, title="Commands")
        self.statusBox = RobotStatusInfoBox(self, title="Robot Status")

        # Size-Policies
        for w in (self.commandBox, self.statusBox):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Preferred)
            w.setSizePolicy(sp)

        topRow.addWidget(self.commandBox, 1)  # schmaler
        topRow.addWidget(self.statusBox, 2)   # breiter
        root.addLayout(topRow)

        # --- [2] Unten: Poses | Tool | Scene nebeneinander ------------------
        self.posesBox = PosesGroupBox(self.bridge, self)
        self.toolBox  = ToolGroupBox(self.bridge, self)
        self.sceneBox = SceneGroupBox(self.bridge, self)

        rowBottom = QHBoxLayout()
        rowBottom.setSpacing(8)
        rowBottom.addWidget(self.posesBox)
        rowBottom.addWidget(self.toolBox)
        rowBottom.addWidget(self.sceneBox)
        rowBottom.setStretch(0, 1)
        rowBottom.setStretch(1, 1)
        rowBottom.setStretch(2, 1)
        root.addLayout(rowBottom)

        # --- [1] Subtabs: Motion | Joint Jog | Cartesian Jog ----------------
        self.tabs = QTabWidget(self)
        root.addWidget(self.tabs)

        # (a) Motion — store-geführt
        self.motionWidget = MotionWidget(store=self.store, bridge=self.bridge, parent=self.tabs)
        self.tabs.addTab(self.motionWidget, "Motion")

        # (b) Joint Jog
        self.jointJogWidget = JointJogWidget(self.bridge, self.tabs)
        self.tabs.addTab(self.jointJogWidget, "Joint Jog")

        # (c) Cartesian Jog
        self.cartJogWidget = CartesianJogWidget(self.bridge, self.tabs)
        self.tabs.addTab(self.cartJogWidget, "Cartesian Jog")

        for w in (self.motionWidget, self.jointJogWidget, self.cartJogWidget,
                  self.posesBox, self.toolBox, self.sceneBox):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Preferred)
            w.setSizePolicy(sp)

        # Wiring RobotBridge <-> Boxes
        self._wire_bridge_inbound()
        self._wire_outbound()

        # --- Tab-Change -> Servo / CommandType ------------------------------
        self.tabs.currentChanged.connect(self._on_tab_changed)
        self._on_tab_changed(self.tabs.currentIndex())

    # ------------------------------------------------------------------ #
    #  Bridge: inbound (ROS -> StatusBox)
    # ------------------------------------------------------------------ #
    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        if js is None or not hasattr(js, "position"):
            self.statusBox.set_joints(None)
        else:
            self.statusBox.set_joints(list(js.position or []))

    def _wire_bridge_inbound(self) -> None:
        if not self._sig:
            return

        sig = self._sig
        sb = self.statusBox

        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(sb.set_connection)
        if hasattr(sig, "modeChanged"):
            sig.modeChanged.connect(sb.set_mode)
        if hasattr(sig, "initializedChanged"):
            sig.initializedChanged.connect(sb.set_initialized)
        if hasattr(sig, "movingChanged"):
            sig.movingChanged.connect(sb.set_moving)
        if hasattr(sig, "powerChanged"):
            sig.powerChanged.connect(sb.set_power)
        if hasattr(sig, "servoEnabledChanged"):
            sig.servoEnabledChanged.connect(sb.set_servo_enabled)
        if hasattr(sig, "estopChanged"):
            sig.estopChanged.connect(sb.set_estop)
        if hasattr(sig, "errorsChanged"):
            sig.errorsChanged.connect(sb.set_errors)
        if hasattr(sig, "tcpPoseChanged"):
            sig.tcpPoseChanged.connect(sb.set_tcp_from_ps)
        if hasattr(sig, "jointsChanged"):
            sig.jointsChanged.connect(self._on_joints)

    # ------------------------------------------------------------------ #
    #  Bridge: outbound (CommandBox -> RobotBridge-Signals)
    # ------------------------------------------------------------------ #
    def _wire_outbound(self) -> None:
        if not self._sig:
            return

        sig = self._sig
        cb = self.commandBox

        cb.initRequested.connect(sig.initRequested.emit)
        cb.stopRequested.connect(sig.stopRequested.emit)
        cb.clearErrorRequested.connect(sig.clearErrorRequested.emit)
        cb.powerOnRequested.connect(sig.powerOnRequested.emit)
        cb.powerOffRequested.connect(sig.powerOffRequested.emit)
        cb.servoEnableRequested.connect(sig.servoEnableRequested.emit)
        cb.servoDisableRequested.connect(sig.servoDisableRequested.emit)

    # ------------------------------------------------------------------ #
    #  Tab-Change: Servo immer an + command_type pro Tab
    # ------------------------------------------------------------------ #
    def _on_tab_changed(self, idx: int) -> None:
        page = self.tabs.widget(idx)

        # Servo immer an – ist idempotent, falls er schon an ist.
        try:
            if hasattr(self.bridge, "robot_servo_on"):
                self.bridge.robot_servo_on()
        except Exception as e:
            _LOG.error("robot_servo_on() failed: %s", e)

        # Command-Type je nach Tab
        mode: Optional[str] = None
        if page is self.jointJogWidget:
            mode = "joint"      # -> JOINT_JOG
        elif page is self.cartJogWidget:
            mode = "cart"       # -> TWIST
        else:
            # Motion-Tab: Default ebenfalls JOINT_JOG
            mode = "joint"

        if mode and hasattr(self.bridge, "servo_set_command_type"):
            try:
                self.bridge.servo_set_command_type(mode)
            except Exception as e:
                _LOG.error("servo_set_command_type(%s) failed: %s", mode, e)

    # ------------ Forwarder (optional) ---------------------------------------
    # Planner (aus Motion-Subtab)
    def get_planner(self) -> str:
        return self.motionWidget.get_planner()

    def get_planner_params(self) -> Dict[str, Any]:
        return self.motionWidget.get_params()

    def set_planner(self, name: str) -> None:
        self.motionWidget.set_planner(name)

    def set_planner_params(self, cfg: Dict[str, Any]) -> None:
        self.motionWidget.set_params(cfg)

    # Jog-Parameter
    def get_joint_jog_params(self) -> Dict[str, float]:
        return self.jointJogWidget.get_params()

    def set_joint_jog_params(self, cfg: Dict[str, float]) -> None:
        self.jointJogWidget.set_params(cfg)

    def get_cart_jog_params(self) -> Dict[str, float]:
        return self.cartJogWidget.get_params()

    def set_cart_jog_params(self, cfg: Dict[str, float]) -> None:
        self.cartJogWidget.set_params(cfg)
