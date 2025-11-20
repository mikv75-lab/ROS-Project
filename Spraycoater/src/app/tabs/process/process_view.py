# -*- coding: utf-8 -*-
# File: tabs/process/process_view.py
from __future__ import annotations

from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QSizePolicy, QTextEdit
)

from app.widgets.robot_status_box import RobotStatusInfoBox
from app.widgets.info_groupbox import InfoGroupBox


class ProcessTabView(QWidget):
    """
    Reine View für den Process-Tab.
    - Baut nur das Layout
    - Hält alle Widgets als Attribute
    - Keine Business-Logik, keine Threads, kein ROS
    """

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---------- TOP ROW ----------
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(8)
        root.addLayout(top_row)

        # --- Process Control ---
        self.grpProcess = QGroupBox("Process Control", self)
        vproc_outer = QVBoxLayout(self.grpProcess)
        vproc_outer.setContentsMargins(8, 8, 8, 8)
        vproc_outer.setSpacing(6)

        self.btnInit = QPushButton("Init", self.grpProcess)
        self.btnLoadRecipe = QPushButton("Load Recipe", self.grpProcess)
        self.btnStart = QPushButton("Start", self.grpProcess)
        self.btnStop = QPushButton("Stop", self.grpProcess)

        for b in (self.btnInit, self.btnLoadRecipe, self.btnStart, self.btnStop):
            b.setMinimumHeight(28)
            vproc_outer.addWidget(b)
        vproc_outer.addStretch(1)
        top_row.addWidget(self.grpProcess, 0)

        # --- Startbedingungen ---
        self.grpStatus = QGroupBox("Startbedingungen", self)
        vstat = QVBoxLayout(self.grpStatus)
        vstat.setContentsMargins(8, 8, 8, 8)
        vstat.setSpacing(4)

        self.lblStatus = QLabel("-", self.grpStatus)
        self.lblStatus.setWordWrap(True)
        vstat.addWidget(self.lblStatus)
        top_row.addWidget(self.grpStatus, 1)

        # --- Setup ---
        self.grpSetup = QGroupBox("Setup", self)
        vsetup = QVBoxLayout(self.grpSetup)
        vsetup.setContentsMargins(8, 8, 8, 8)
        vsetup.setSpacing(4)

        self.lblTool = QLabel("Tool: -", self.grpSetup)
        self.lblSubstrate = QLabel("Substrate: -", self.grpSetup)
        self.lblMount = QLabel("Mount: -", self.grpSetup)

        for lab in (self.lblTool, self.lblSubstrate, self.lblMount):
            lab.setWordWrap(True)
            vsetup.addWidget(lab)

        vsetup.addStretch(1)
        top_row.addWidget(self.grpSetup, 1)

        # --- Robot Status ---
        self.robotStatusBox = RobotStatusInfoBox(self, title="Robot Status")
        top_row.addWidget(self.robotStatusBox, 2)

        # Size Policies
        for gb in (self.grpProcess, self.grpStatus, self.grpSetup):
            sp = gb.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
            gb.setSizePolicy(sp)

        sp_rs = self.robotStatusBox.sizePolicy()
        sp_rs.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_rs.setVerticalPolicy(QSizePolicy.Policy.Fixed)
        self.robotStatusBox.setSizePolicy(sp_rs)

        # ---------- INFO BOX ----------
        self.infoBox = InfoGroupBox(self)
        sp_info = self.infoBox.sizePolicy()
        sp_info.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_info.setVerticalPolicy(QSizePolicy.Policy.Fixed)
        self.infoBox.setSizePolicy(sp_info)
        root.addWidget(self.infoBox)

        # ---------- PROCESS STATUS / LOG ----------
        self.grpProcessInfo = QGroupBox("Process Status / Log", self)
        vproc_info = QVBoxLayout(self.grpProcessInfo)
        vproc_info.setContentsMargins(8, 8, 8, 8)
        vproc_info.setSpacing(4)

        self.lblProcessState = QLabel("Kein Prozess aktiv.", self.grpProcessInfo)
        self.lblProcessState.setWordWrap(True)

        self.txtProcessLog = QTextEdit(self.grpProcessInfo)
        self.txtProcessLog.setReadOnly(True)
        self.txtProcessLog.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        vproc_info.addWidget(self.lblProcessState)
        vproc_info.addWidget(self.txtProcessLog, 1)

        sp_pinfo = self.grpProcessInfo.sizePolicy()
        sp_pinfo.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_pinfo.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.grpProcessInfo.setSizePolicy(sp_pinfo)

        root.addWidget(self.grpProcessInfo, 1)

        # ---------- RECIPE GROUP ----------
        self.grpRecipe = QGroupBox("Recipe", self)
        vrec = QHBoxLayout(self.grpRecipe)
        vrec.setContentsMargins(8, 8, 8, 8)
        vrec.setSpacing(6)

        self.txtRecipeSummary = QTextEdit(self.grpRecipe)
        self.txtRecipeSummary.setReadOnly(True)
        self.txtRecipeSummary.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        self.txtRecipePoses = QTextEdit(self.grpRecipe)
        self.txtRecipePoses.setReadOnly(True)
        self.txtRecipePoses.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        vrec.addWidget(self.txtRecipeSummary, 1)
        vrec.addWidget(self.txtRecipePoses, 1)

        sp_rec = self.grpRecipe.sizePolicy()
        sp_rec.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_rec.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.grpRecipe.setSizePolicy(sp_rec)

        root.addWidget(self.grpRecipe, 1)
