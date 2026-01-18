# -*- coding: utf-8 -*-
# File: src/tabs/service/robot/service_robot_tab.py
from __future__ import annotations

import logging
from typing import Optional, Dict, Any, TYPE_CHECKING

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget, QSizePolicy, QLabel
)

from model.recipe.recipe_store import RecipeStore
from widgets.robot_command_box import RobotCommandButtonsBox
from widgets.robot_status_box import RobotStatusInfoBox
from widgets.omron_tcp_widget import OmronTcpWidget

from .scene_box import SceneGroupBox
from .poses_box import PosesGroupBox
from .tool_box import ToolGroupBox
from .moveitpy_widget import MoveItPyWidget
from .servo_widgets import JointJogWidget, CartesianJogWidget

# If TYPE_CHECKING is needed for circular imports or similar
if TYPE_CHECKING:
    from ros.bridge.ros_bridge import RosBridge


_LOG = logging.getLogger("tabs.service.robot")


class ServiceRobotTab(QWidget):
    """
    Service-Tab für Roboterfunktionen.

    Contract:
      - ros darf None sein -> UI deaktiviert (Hinweis wird angezeigt)
      - sonst: ros ist RosBridge und connected
    """

    def __init__(
        self,
        *,
        ctx,
        store: RecipeStore,
        ros: Optional["RosBridge"],
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)

        self.ctx = ctx
        self.store = store
        self.ros = ros

        self._rb = None   # RobotBridge
        self._sig = None  # RobotBridge.signals

        # --- Root Layout ---
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- No bridge: Tab bleibt sichtbar aber deaktiviert ---
        if self.ros is None:
            hint = QLabel(
                "ROS-Bridge nicht verbunden (shadow/live fehlgeschlagen). Robot-Tab ist deaktiviert.",
                self
            )
            hint.setStyleSheet("color: orange; font-weight: 600;")
            root.addWidget(hint)
            root.addStretch(1)
            return

        # --- Hard contract: connected + Bridges vorhanden ---
        self._rb = self.ros.robot
        self._sig = self._rb.signals

        # ================================================================
        # TOP ROW: Commands | Status | Omron TCP
        # ================================================================
        top_row = QHBoxLayout()
        top_row.setSpacing(8)

        self.commandBox = RobotCommandButtonsBox(self, title="Commands")
        self.statusBox = RobotStatusInfoBox(self, title="Robot Status")
        self.omronWidget = OmronTcpWidget(self.ros, self)

        # Policies
        for w in (self.commandBox, self.statusBox, self.omronWidget):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        # 1. Zeile befüllen
        top_row.addWidget(self.commandBox, 1)
        top_row.addWidget(self.statusBox, 2)
        top_row.addWidget(self.omronWidget, 2)
        
        # Oben zum Root hinzufügen
        root.addLayout(top_row)


        # ================================================================
        # BOTTOM AREA: HBox( TabWidget (links), VBox(Poses, Tool, Scene) (rechts) )
        # ================================================================
        bottom_area = QHBoxLayout()
        bottom_area.setSpacing(8)

        # --- Linke Seite: Tabs (Motion, Jogging) ---
        self.tabs = QTabWidget(self)
        
        self.moveitpyWidget = MoveItPyWidget(store=self.store, ros=self.ros, parent=self.tabs)
        self.tabs.addTab(self.moveitpyWidget, "Motion Planning")

        self.jointJogWidget = JointJogWidget(self.ctx, self.ros, self.tabs)
        self.tabs.addTab(self.jointJogWidget, "Joint Jog")

        self.cartJogWidget = CartesianJogWidget(self.ctx, self.ros, self.tabs)
        self.tabs.addTab(self.cartJogWidget, "Cartesian Jog")

        # Tab Policy
        sp_tabs = self.tabs.sizePolicy()
        sp_tabs.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_tabs.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.tabs.setSizePolicy(sp_tabs)
        
        # Tabs zur Bottom Area (links, Stretch 2)
        bottom_area.addWidget(self.tabs, 2)


        # --- Rechte Seite: VBox(Poses, Tool, Scene) ---
        right_col_layout = QVBoxLayout()
        right_col_layout.setSpacing(8)
        right_col_layout.setContentsMargins(0, 0, 0, 0)

        self.posesBox = PosesGroupBox(self.ros, self)
        self.toolBox = ToolGroupBox(self.ros, self)
        self.sceneBox = SceneGroupBox(self.ros, self)

        # Policies für rechte Boxen
        for w in (self.posesBox, self.toolBox, self.sceneBox):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred) # oder Expanding, je nach Inhalt
            w.setSizePolicy(sp)

        right_col_layout.addWidget(self.posesBox)
        right_col_layout.addWidget(self.toolBox)
        right_col_layout.addWidget(self.sceneBox)
        
        # Spacer unten rechts, damit Boxen oben bleiben (optional)
        right_col_layout.addStretch(1)

        # Rechte Spalte zur Bottom Area (rechts, Stretch 1)
        bottom_area.addLayout(right_col_layout, 1)

        # Bottom Area zum Root hinzufügen
        root.addLayout(bottom_area, 1) # Stretch 1 damit es den Rest füllt


        # --- Wiring ---
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
        if hasattr(self.ros, 'robot') and self.ros.robot:
             self.ros.robot.do_servo_on()

        mode = "joint"
        if page is self.jointJogWidget:
            mode = "joint"
        elif page is self.cartJogWidget:
            mode = "cart"
        # else (Motion Planning): mode egal oder joint

        # Optional: Dem Servo-Controller den Modus mitteilen (falls implementiert)
        if hasattr(self.ros, 'servo') and self.ros.servo:
             self.ros.servo.signals.modeChanged.emit(mode)

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