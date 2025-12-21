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
    Service-Tab für Roboterfunktionen.

    Contract:
      - bridge darf None sein -> UI deaktiviert (Hinweis wird angezeigt)
      - sonst: bridge ist UIBridge und connected (ensure_connected() wird hier 1x aufgerufen)
      - keine Fallback-Attribute / kein "try multiple names"
    """

    def __init__(
        self,
        *,
        ctx,
        store: RecipeStore,
        bridge: Optional["UIBridge"],
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)

        self.ctx = ctx
        self.store = store
        self.bridge = bridge

        self._rb = None   # RobotBridge
        self._sig = None  # RobotBridge.signals

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- No bridge: Tab bleibt sichtbar aber deaktiviert ---
        if self.bridge is None:
            hint = QLabel(
                "ROS-Bridge nicht verbunden (shadow/live fehlgeschlagen). Robot-Tab ist deaktiviert.",
                self
            )
            hint.setStyleSheet("color: orange; font-weight: 600;")
            root.addWidget(hint)
            root.addStretch(1)
            return

        # --- Hard contract: connected + Bridges vorhanden ---
        self.bridge.ensure_connected()
        self._rb = self.bridge.robot_bridge
        self._sig = self._rb.signals

        # ================================================================
        # TOP ROW: Commands | Status | Omron TCP
        # ================================================================
        top_row = QHBoxLayout()
        top_row.setSpacing(8)

        self.commandBox = RobotCommandButtonsBox(self, title="Commands")
        self.statusBox = RobotStatusInfoBox(self, title="Robot Status")
        self.omronWidget = OmronTcpWidget(self.bridge, self)

        for w in (self.commandBox, self.statusBox, self.omronWidget):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        top_row.addWidget(self.commandBox, 1)
        top_row.addWidget(self.statusBox, 2)
        top_row.addWidget(self.omronWidget, 2)
        root.addLayout(top_row)

        # ================================================================
        # BOTTOM ROW: Poses | Tools | Scene
        # ================================================================
        row_bottom = QHBoxLayout()
        row_bottom.setSpacing(8)

        self.posesBox = PosesGroupBox(self.bridge, self)
        self.toolBox = ToolGroupBox(self.bridge, self)
        self.sceneBox = SceneGroupBox(self.bridge, self)

        row_bottom.addWidget(self.posesBox)
        row_bottom.addWidget(self.toolBox)
        row_bottom.addWidget(self.sceneBox)
        root.addLayout(row_bottom)

        # ================================================================
        # SUBTABS: Motion | Joint Jog | Cartesian Jog
        # ================================================================
        self.tabs = QTabWidget(self)
        root.addWidget(self.tabs)

        self.moveitpyWidget = MoveItPyWidget(store=self.store, bridge=self.bridge, parent=self.tabs)
        self.tabs.addTab(self.moveitpyWidget, "Motion Planning")

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

        # Wiring
        self._wire_bridge_inbound()
        self._wire_outbound()

        self.tabs.currentChanged.connect(self._on_tab_changed)
        self._on_tab_changed(self.tabs.currentIndex())

    # ==================================================================
    # Bridge INBOUND: ROS → UI
    # ==================================================================

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js) -> None:
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

        # ✅ JointState -> StatusBox + JointJogWidget (Anzeige-only Slider)
        sig.jointsChanged.connect(self._on_joints)
        sig.jointsChanged.connect(self.jointJogWidget.update_from_joint_state)
    
    # ==================================================================
    # Bridge OUTBOUND: UI → ROS
    # ==================================================================

    def _wire_outbound(self) -> None:
        sig = self._sig
        cb = self.commandBox

        # Direkt Signal → Signal (kein Fallback, keine Wrapper hier)
        cb.initRequested.connect(sig.initRequested)
        cb.stopRequested.connect(sig.stopRequested)
        cb.clearErrorRequested.connect(sig.clearErrorRequested)
        cb.powerOnRequested.connect(sig.powerOnRequested)
        cb.powerOffRequested.connect(sig.powerOffRequested)
        cb.servoEnableRequested.connect(sig.servoEnableRequested)
        cb.servoDisableRequested.connect(sig.servoDisableRequested)

    # ==================================================================
    # Tab-Wechsel → Servo an + Command-Type setzen
    # ==================================================================

    def _on_tab_changed(self, idx: int) -> None:
        page = self.tabs.widget(idx)

        # Hard contract: bridge ist verbunden
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
