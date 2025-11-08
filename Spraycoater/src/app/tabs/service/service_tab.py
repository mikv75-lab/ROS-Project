# -*- coding: utf-8 -*-
# File: tabs/service_tab.py
from __future__ import annotations
import logging
from typing import Optional, Dict, Any

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QTabWidget,
    QSpacerItem, QSizePolicy
)

# Eigenständige Widgets (verdrahten sich selbst mit der Bridge)
from ...widgets.robot_command_status_box import RobotCommandStatusWidget  # ⬅️ kombiniert: Commands (links) + Status (rechts)
from .scene_box import SceneGroupBox
from .poses_box import PosesGroupBox
from .tool_box import ToolGroupBox
from .motion_widget import MotionWidget          # enthält PlannerGroupBox + Motion-Speed
from .servo_widgets import JointJogWidget, CartesianJogWidget  # getrennte Jog-Widgets

_LOG = logging.getLogger("app.tabs.service")


class ServiceTab(QWidget):
    """
    Service-Tab Layout (nur Code, keine .ui):

      [0] Command+Status   (RobotCommandStatusWidget: links Commands, rechts Status)
      [1] Tabs:
            ├── Motion (Planner + Motion-Speed)
            ├── Joint Jog
            └── Cartesian Jog
      [2] Row(HBox): PosesGroupBox | ToolGroupBox | SceneGroupBox
      [3] Spacer (Expanding)
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # Root-Layout
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- [0] Command + Status (oben) ------------------------------------
        self.commandStatus = RobotCommandStatusWidget(self.bridge, self)
        sp = self.commandStatus.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.commandStatus.setSizePolicy(sp)
        root.addWidget(self.commandStatus)

        # --- [1] Subtabs: Motion | Joint Jog | Cartesian Jog ----------------
        self.tabs = QTabWidget(self)
        root.addWidget(self.tabs)

        # (a) Motion
        self.motionWidget = MotionWidget(self.bridge, self.tabs)
        self.tabs.addTab(self.motionWidget, "Motion")

        # (b) Joint Jog
        self.jointJogWidget = JointJogWidget(self.bridge, self.tabs)
        self.tabs.addTab(self.jointJogWidget, "Joint Jog")

        # (c) Cartesian Jog
        self.cartJogWidget = CartesianJogWidget(self.bridge, self.tabs)
        self.tabs.addTab(self.cartJogWidget, "Cartesian Jog")

        for w in (self.motionWidget, self.jointJogWidget, self.cartJogWidget):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        # --- [2] Unten: Poses | Tool | Scene nebeneinander ------------------
        self.posesBox = PosesGroupBox(self.bridge, self)
        self.toolBox  = ToolGroupBox(self.bridge, self)
        self.sceneBox = SceneGroupBox(self.bridge, self)

        rowBottom = QHBoxLayout()
        rowBottom.setSpacing(8)
        rowBottom.addWidget(self.posesBox)
        rowBottom.addWidget(self.toolBox)
        rowBottom.addWidget(self.sceneBox)
        # gleichmäßig stretchen
        rowBottom.setStretch(0, 1)
        rowBottom.setStretch(1, 1)
        rowBottom.setStretch(2, 1)
        root.addLayout(rowBottom)

        # Resize-Policies für die drei Boxen
        for w in (self.posesBox, self.toolBox, self.sceneBox):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        # --- [3] EIN Expanding Spacer am Ende -------------------------------
        root.addItem(QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

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
