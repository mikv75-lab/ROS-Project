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
from .robot_status_box import RobotStatusGroupBox
from .scene_box import SceneGroupBox
from .poses_box import PosesGroupBox
from .motion_widget import MotionWidget      # enthält PlannerGroupBox
from .servo_widgets import JointJogWidget, CartesianJogWidget  # <— neu: getrennte Jog-Widgets

_LOG = logging.getLogger("app.tabs.service")


class ServiceTab(QWidget):
    """
    Service-Tab Layout (nur Code, keine .ui):

      [0] RobotStatusGroupBox
      [1] Row(HBox): SceneGroupBox | PosesGroupBox  (nebeneinander)
      [2] Tabs:
            ├── Motion (MotionGroupBox inkl. Planner)
            ├── Joint Jog (jointJogWidget)
            └── Cartesian Jog (CartesianJogBox)
      [3] Spacer (Expanding)
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # Root-Layout: einzige VBox
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # --- [0] Status ------------------------------------------------------
        self.statusBox = RobotStatusGroupBox(self.bridge, self)
        root.addWidget(self.statusBox)

        # --- [1] Scene | Poses nebeneinander --------------------------------
        self.sceneBox = SceneGroupBox(self.bridge, self)
        self.posesBox = PosesGroupBox(self.bridge, self)

        rowScenePoses = QHBoxLayout()
        rowScenePoses.setSpacing(8)
        rowScenePoses.addWidget(self.sceneBox)
        rowScenePoses.addWidget(self.posesBox)
        rowScenePoses.setStretch(0, 1)
        rowScenePoses.setStretch(1, 1)
        root.addLayout(rowScenePoses)

        # Resize-Policies
        for w in (self.sceneBox, self.posesBox):
            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

        # --- [2] Subtabs: Motion | Joint Jog | Cartesian Jog -----------------
        self.tabs = QTabWidget(self)
        root.addWidget(self.tabs)

        # (a) Motion (mit Planner + Motion-Speed mm/s)
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

        # --- [3] EIN Expanding Spacer am Ende --------------------------------
        root.addItem(QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))

    # ------------ Forwarder (optional) ---------------------------------------
    # Planner (aus MotionGroupBox)
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
