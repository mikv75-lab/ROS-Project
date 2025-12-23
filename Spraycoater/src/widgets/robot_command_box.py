# -*- coding: utf-8 -*-
# File: app/widgets/robot_command_box.py
from __future__ import annotations

from typing import Optional

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QGroupBox, QVBoxLayout, QPushButton, QSizePolicy, QWidget
)

__all__ = ["RobotCommandButtonsBox"]


class RobotCommandButtonsBox(QGroupBox):
    """
    Vertikale Aktions-Buttons:
      Initialize | Stop | Clear Error | Power ON/OFF | Servo ENABLE/DISABLE
    """
    initRequested         = QtCore.pyqtSignal()
    stopRequested         = QtCore.pyqtSignal()
    clearErrorRequested   = QtCore.pyqtSignal()
    powerOnRequested      = QtCore.pyqtSignal()
    powerOffRequested     = QtCore.pyqtSignal()
    servoEnableRequested  = QtCore.pyqtSignal()
    servoDisableRequested = QtCore.pyqtSignal()

    def __init__(self, parent: Optional[QWidget] = None, title: str = "Commands"):
        super().__init__(title, parent)
        self._build_ui()
        self._apply_policies()

    def _build_ui(self) -> None:
        v = QVBoxLayout(self)
        v.setSpacing(6)
        v.setContentsMargins(8, 8, 8, 8)

        self.btnInit   = QPushButton("Initialize", self)
        self.btnStop   = QPushButton("Stop", self)
        self.btnClrErr = QPushButton("Clear Error", self)
        self.btnPwrOn  = QPushButton("Power ON", self)
        self.btnPwrOff = QPushButton("Power OFF", self)
        self.btnSrvOn  = QPushButton("Servo ENABLE", self)
        self.btnSrvOff = QPushButton("Servo DISABLE", self)

        for b in (
            self.btnInit, self.btnStop, self.btnClrErr,
            self.btnPwrOn, self.btnPwrOff, self.btnSrvOn, self.btnSrvOff
        ):
            b.setMinimumHeight(28)
            v.addWidget(b)

        v.addStretch(1)

        # Buttons -> Widget-Signale
        self.btnInit.clicked.connect(self.initRequested.emit)
        self.btnStop.clicked.connect(self.stopRequested.emit)
        self.btnClrErr.clicked.connect(self.clearErrorRequested.emit)
        self.btnPwrOn.clicked.connect(self.powerOnRequested.emit)
        self.btnPwrOff.clicked.connect(self.powerOffRequested.emit)
        self.btnSrvOn.clicked.connect(self.servoEnableRequested.emit)
        self.btnSrvOff.clicked.connect(self.servoDisableRequested.emit)

    def _apply_policies(self) -> None:
        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)


# -------- Backward-compat Aliase (falls irgendwo noch alte Namen hängen) -----
RobotCommandButtonBox = RobotCommandButtonsBox
RobotCommandsBox = RobotCommandButtonsBox
