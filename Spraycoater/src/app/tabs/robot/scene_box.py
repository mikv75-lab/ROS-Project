# -*- coding: utf-8 -*-
# app/tabs/service/scene_box.py  (oder wo es liegt)
from __future__ import annotations

from typing import Optional, Sequence

from PyQt6 import QtCore
from PyQt6.QtWidgets import QGroupBox, QGridLayout, QLabel, QComboBox, QPushButton, QWidget


class SceneGroupBox(QGroupBox):
    """
    Scene-Controls (event-driven):
      - UI updated nur bei ROS publish (via SceneBridge.signals)
      - Buttons senden Signale an Bridge (SceneBridge.signals.set*Requested)
    """

    # Outbound (Widget -> außen/Bridge)
    setCageRequested = QtCore.pyqtSignal(str)
    setMountRequested = QtCore.pyqtSignal(str)
    setSubstrateRequested = QtCore.pyqtSignal(str)

    def __init__(self, bridge, parent: Optional[QWidget] = None, title: str = "Scene"):
        super().__init__(parent)
        self.setTitle(title)
        self.bridge = bridge

        # hard contract: bridge.scene_bridge muss existieren (bridge.ensure_connected() passiert im Tab)
        self._sb = self.bridge.scene_bridge
        self._sig = self._sb.signals

        self._build_ui()
        self._wire_bridge_inbound()
        self._wire_outbound()

    # ------------------------------------------------------------------ UI

    def _build_ui(self) -> None:
        g = QGridLayout(self)

        def _bold(txt: str) -> QLabel:
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            return l

        # Header
        g.addWidget(_bold("Type"), 0, 0)
        g.addWidget(_bold("Select"), 0, 1)
        g.addWidget(_bold("Set"), 0, 2)
        g.addWidget(_bold("Current"), 0, 3)
        g.addWidget(_bold("Clear"), 0, 4)

        # Cage
        g.addWidget(QLabel("Cage", self), 1, 0)
        self.cmbCage = QComboBox(self)
        g.addWidget(self.cmbCage, 1, 1)
        self.btnSetCage = QPushButton("Set", self)
        g.addWidget(self.btnSetCage, 1, 2)
        self.lblCageCur = QLabel("-", self)
        g.addWidget(self.lblCageCur, 1, 3)
        self.btnClrCage = QPushButton("Clear", self)
        g.addWidget(self.btnClrCage, 1, 4)

        # Mount
        g.addWidget(QLabel("Mount", self), 2, 0)
        self.cmbMount = QComboBox(self)
        g.addWidget(self.cmbMount, 2, 1)
        self.btnSetMount = QPushButton("Set", self)
        g.addWidget(self.btnSetMount, 2, 2)
        self.lblMountCur = QLabel("-", self)
        g.addWidget(self.lblMountCur, 2, 3)
        self.btnClrMount = QPushButton("Clear", self)
        g.addWidget(self.btnClrMount, 2, 4)

        # Substrate
        g.addWidget(QLabel("Substrate", self), 3, 0)
        self.cmbSub = QComboBox(self)
        g.addWidget(self.cmbSub, 3, 1)
        self.btnSetSub = QPushButton("Set", self)
        g.addWidget(self.btnSetSub, 3, 2)
        self.lblSubCur = QLabel("-", self)
        g.addWidget(self.lblSubCur, 3, 3)
        self.btnClrSub = QPushButton("Clear", self)
        g.addWidget(self.btnClrSub, 3, 4)

    # ------------------------------------------------------------------ Bridge Inbound (ROS -> UI)

    def _wire_bridge_inbound(self) -> None:
        sig = self._sig

        # Lists
        sig.cageListChanged.connect(lambda items: self._fill(self.cmbCage, items))
        sig.mountListChanged.connect(lambda items: self._fill(self.cmbMount, items))
        sig.substrateListChanged.connect(lambda items: self._fill(self.cmbSub, items))

        # Current
        sig.cageCurrentChanged.connect(lambda v: self._set_current(self.cmbCage, self.lblCageCur, v))
        sig.mountCurrentChanged.connect(lambda v: self._set_current(self.cmbMount, self.lblMountCur, v))
        sig.substrateCurrentChanged.connect(lambda v: self._set_current(self.cmbSub, self.lblSubCur, v))

    # ------------------------------------------------------------------ Outbound (UI -> Bridge)

    def _wire_outbound(self) -> None:
        # Buttons -> Widget-Signals
        self.btnSetCage.clicked.connect(lambda: self.setCageRequested.emit(self._sel(self.cmbCage)))
        self.btnSetMount.clicked.connect(lambda: self.setMountRequested.emit(self._sel(self.cmbMount)))
        self.btnSetSub.clicked.connect(lambda: self.setSubstrateRequested.emit(self._sel(self.cmbSub)))

        self.btnClrCage.clicked.connect(lambda: self.setCageRequested.emit(""))
        self.btnClrMount.clicked.connect(lambda: self.setMountRequested.emit(""))
        self.btnClrSub.clicked.connect(lambda: self.setSubstrateRequested.emit(""))

        # Widget-Signals -> Bridge Signals (direkt, event-driven)
        self.setCageRequested.connect(self._sig.setCageRequested)
        self.setMountRequested.connect(self._sig.setMountRequested)
        self.setSubstrateRequested.connect(self._sig.setSubstrateRequested)

    # ------------------------------------------------------------------ Helpers

    @staticmethod
    def _sel(combo: QComboBox) -> str:
        try:
            return combo.currentText().strip()
        except Exception:
            return ""

    @staticmethod
    def _fill(combo: QComboBox, items: Sequence[str]) -> None:
        combo.blockSignals(True)
        combo.clear()
        combo.addItems([str(x) for x in (items or [])])
        combo.blockSignals(False)

    @staticmethod
    def _set_current(combo: QComboBox, label: QLabel, value: str) -> None:
        v = (value or "").strip()
        label.setText(v if v else "-")
        if not v:
            return
        ix = combo.findText(v)
        if ix >= 0:
            combo.setCurrentIndex(ix)
