# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations
from typing import Optional

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QSizePolicy

from widgets.robot_command_status_box import RobotCommandStatusWidget
from .process_control_widget import ProcessControlWidget


class ProcessTab(QWidget):
    """
    Process-Tab (nur Code, keine .ui):
      [0] RobotCommandStatusWidget (links Commands, rechts Status)
      [1] ProcessControlWidget (enthält Process-Steuerung + Recipe/Poses)
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # [0] Command+Status oben – vertikal so klein wie möglich
        self.cmdStatus = RobotCommandStatusWidget(bridge=self.bridge, parent=self)
        sp = self.cmdStatus.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
        self.cmdStatus.setSizePolicy(sp)
        root.addWidget(self.cmdStatus)

        # [1] Process Control – füllt den restlichen Platz
        self.procCtrl = ProcessControlWidget(ctx=self.ctx, bridge=self.bridge, parent=self)
        sp = self.procCtrl.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Minimum)
        self.procCtrl.setSizePolicy(sp)
        root.addWidget(self.procCtrl)
