# -*- coding: utf-8 -*-
# SceneGroupBox – besitzt Bridge, verdrahtet sich selbst, hat eigene Qt-Signals
from __future__ import annotations
from typing import Optional, Sequence
from PyQt6 import QtCore
from PyQt6.QtWidgets import QGroupBox, QGridLayout, QLabel, QComboBox, QPushButton, QWidget

class SceneGroupBox(QGroupBox):
    # Outbound (Widget -> außen/Bridge)
    setCageRequested = QtCore.pyqtSignal(str)
    setMountRequested = QtCore.pyqtSignal(str)
    setSubstrateRequested = QtCore.pyqtSignal(str)

    def __init__(self, bridge, parent: Optional[QWidget] = None, title: str = "Scene"):
        super().__init__(parent)
        self.setTitle(title)
        self.bridge = bridge
        self._build_ui()
        self._wire_bridge_inbound()
        self._wire_outbound()

    # ---------- UI ----------
    def _build_ui(self):
        g = QGridLayout(self)

        def _bold(txt: str) -> QLabel:
            l = QLabel(txt, self); l.setStyleSheet("font-weight: 600;"); return l

        g.addWidget(_bold("Type"),    0, 0)
        g.addWidget(_bold("Select"),  0, 1)
        g.addWidget(_bold("Set"),     0, 2)
        g.addWidget(_bold("Current"), 0, 3)
        g.addWidget(_bold("Clear"),   0, 4)

        # Cage row
        g.addWidget(QLabel("Cage", self), 1, 0)
        self.cmbCage = QComboBox(self); g.addWidget(self.cmbCage, 1, 1)
        self.btnSetCage = QPushButton("Set", self); g.addWidget(self.btnSetCage, 1, 2)
        self.lblCageCur = QLabel("-", self); g.addWidget(self.lblCageCur, 1, 3)
        self.btnClrCage = QPushButton("Clear", self); g.addWidget(self.btnClrCage, 1, 4)

        # Mount row
        g.addWidget(QLabel("Mount", self), 2, 0)
        self.cmbMount = QComboBox(self); g.addWidget(self.cmbMount, 2, 1)
        self.btnSetMount = QPushButton("Set", self); g.addWidget(self.btnSetMount, 2, 2)
        self.lblMountCur = QLabel("-", self); g.addWidget(self.lblMountCur, 2, 3)
        self.btnClrMount = QPushButton("Clear", self); g.addWidget(self.btnClrMount, 2, 4)

        # Substrate row
        g.addWidget(QLabel("Substrate", self), 3, 0)
        self.cmbSub = QComboBox(self); g.addWidget(self.cmbSub, 3, 1)
        self.btnSetSub = QPushButton("Set", self); g.addWidget(self.btnSetSub, 3, 2)
        self.lblSubCur = QLabel("-", self); g.addWidget(self.lblSubCur, 3, 3)
        self.btnClrSub = QPushButton("Clear", self); g.addWidget(self.btnClrSub, 3, 4)

    # ---------- Bridge Inbound ----------
    def _wire_bridge_inbound(self):
        sb = getattr(self.bridge, "_sb", None)
        sig = getattr(sb, "signals", None) if sb else None
        if not sig:
            return

        sig.cageListChanged.connect(lambda items: self._fill(self.cmbCage, items))
        sig.mountListChanged.connect(lambda items: self._fill(self.cmbMount, items))
        sig.substrateListChanged.connect(lambda items: self._fill(self.cmbSub, items))

        sig.cageCurrentChanged.connect(lambda v: self._set_current(self.cmbCage, self.lblCageCur, v))
        sig.mountCurrentChanged.connect(lambda v: self._set_current(self.cmbMount, self.lblMountCur, v))
        sig.substrateCurrentChanged.connect(lambda v: self._set_current(self.cmbSub, self.lblSubCur, v))

        # Outbound-Through: meine eigenen Signals direkt an die Bridge hängen (falls vorhanden)
        # Das mache ich im Outbound-Teil.

    # ---------- Outbound ----------
    def _wire_outbound(self):
        # Buttons -> Widget-Signals
        self.btnSetCage.clicked.connect(lambda: self.setCageRequested.emit(self.cmbCage.currentText().strip()))
        self.btnSetMount.clicked.connect(lambda: self.setMountRequested.emit(self.cmbMount.currentText().strip()))
        self.btnSetSub.clicked.connect(lambda: self.setSubstrateRequested.emit(self.cmbSub.currentText().strip()))
        self.btnClrCage.clicked.connect(lambda: self.setCageRequested.emit(""))
        self.btnClrMount.clicked.connect(lambda: self.setMountRequested.emit(""))
        self.btnClrSub.clicked.connect(lambda: self.setSubstrateRequested.emit(""))

        # Widget-Signals -> Bridge (falls vorhanden)
        sb = getattr(self.bridge, "_sb", None)
        bsig = getattr(sb, "signals", None) if sb else None
        if bsig:
            self.setCageRequested.connect(bsig.setCageRequested.emit)
            self.setMountRequested.connect(bsig.setMountRequested.emit)
            self.setSubstrateRequested.connect(bsig.setSubstrateRequested.emit)

    # ---------- Public Helpers ----------
    def set_lists(self,
                  cages: Sequence[str] | None = None,
                  mounts: Sequence[str] | None = None,
                  substrates: Sequence[str] | None = None) -> None:
        if cages is not None: self._fill(self.cmbCage, cages)
        if mounts is not None: self._fill(self.cmbMount, mounts)
        if substrates is not None: self._fill(self.cmbSub, substrates)

    def set_current(self, cage: str | None = None, mount: str | None = None, substrate: str | None = None) -> None:
        if cage is not None: self._set_current(self.cmbCage, self.lblCageCur, cage)
        if mount is not None: self._set_current(self.cmbMount, self.lblMountCur, mount)
        if substrate is not None: self._set_current(self.cmbSub, self.lblSubCur, substrate)

    # ---------- Intern ----------
    @staticmethod
    def _fill(combo: QComboBox, items: Sequence[str]):
        combo.blockSignals(True); combo.clear(); combo.addItems(list(items)); combo.blockSignals(False)

    @staticmethod
    def _set_current(combo: QComboBox, label: QLabel, value: str):
        v = (value or "").strip()
        label.setText(v if v else "-")
        if not v: return
        ix = combo.findText(v)
        if ix >= 0: combo.setCurrentIndex(ix)
