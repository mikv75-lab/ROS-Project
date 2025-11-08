# -*- coding: utf-8 -*-
# File: widgets/process_control_widget.py
from __future__ import annotations
import os
from typing import Optional

from PyQt6 import uic, QtCore
from PyQt6.QtWidgets import QWidget, QPushButton, QLabel, QVBoxLayout, QGroupBox


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/widgets
    return os.path.abspath(os.path.join(here, "..", "..", ".."))


def _ui_path(filename: str) -> str:
    # erwartet: resource/ui/tabs/process/process_control.ui
    return os.path.join(_project_root(), "resource", "ui", "tabs", "process", filename)


class ProcessControlWidget(QWidget):
    """
    Lädt 'process_control.ui' und bindet:
      - Recipe-Box mit Anzeige des geladenen Rezeptnamens
      - Process Control: Init / Load Recipe / Start / Stop

    Outbound-Signale:
      initRequested, loadRecipeRequested, startRequested, stopRequested

    Bridge-Auto-Wiring (falls vorhanden):
      - _process.signals / _proc.signals:
          initRequested, loadRecipeRequested, startRequested, stopRequested (emit)
          recipeLoaded(name: str) ODER currentRecipeChanged(name: str) -> set_recipe_name
    """
    initRequested       = QtCore.pyqtSignal()
    loadRecipeRequested = QtCore.pyqtSignal()
    startRequested      = QtCore.pyqtSignal()
    stopRequested       = QtCore.pyqtSignal()

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("process_control.ui")
        if not os.path.exists(ui_file):
            # Fallback: UI on-the-fly (Recipe + Process Control)
            wrap = QVBoxLayout(self)
            # Recipe-Box
            self.grpRecipe = QGroupBox("Recipe", self)
            vrec = QVBoxLayout(self.grpRecipe)
            self.lblRecipeName = QLabel("-", self.grpRecipe)
            vrec.addWidget(self.lblRecipeName)
            wrap.addWidget(self.grpRecipe)
            # Process Control
            self.grpProcess = QGroupBox("Process Control", self)
            v = QVBoxLayout(self.grpProcess)
            self.btnInit = QPushButton("Init", self.grpProcess)
            self.btnLoadRecipe = QPushButton("Load Recipe", self.grpProcess)
            self.btnStart = QPushButton("Start", self.grpProcess)
            self.btnStop = QPushButton("Stop", self.grpProcess)
            for b in (self.btnInit, self.btnLoadRecipe, self.btnStart, self.btnStop):
                b.setMinimumHeight(28); v.addWidget(b)
            v.addStretch(1)
            wrap.addWidget(self.grpProcess)
            wrap.addStretch(1)
        else:
            uic.loadUi(ui_file, self)
            # Widgets aus UI
            self.lblRecipeName: QLabel = self.findChild(QLabel, "lblRecipeName")
            self.btnInit: QPushButton = self.findChild(QPushButton, "btnInit")
            self.btnLoadRecipe: QPushButton = self.findChild(QPushButton, "btnLoadRecipe")
            self.btnStart: QPushButton = self.findChild(QPushButton, "btnStart")
            self.btnStop: QPushButton = self.findChild(QPushButton, "btnStop")

        # Buttons -> Widget-Signale
        if self.btnInit:        self.btnInit.clicked.connect(self.initRequested.emit)
        if self.btnLoadRecipe:  self.btnLoadRecipe.clicked.connect(self.loadRecipeRequested.emit)
        if self.btnStart:       self.btnStart.clicked.connect(self.startRequested.emit)
        if self.btnStop:        self.btnStop.clicked.connect(self.stopRequested.emit)

        # Bridge-Durchleitung
        self._wire_outbound_to_bridge()
        self._wire_inbound_from_bridge()

    # ---------------- Public API ----------------
    def set_recipe_name(self, name: str | None) -> None:
        if not hasattr(self, "lblRecipeName") or self.lblRecipeName is None:
            return
        txt = (name or "").strip()
        self.lblRecipeName.setText(txt if txt else "-")

    # ---------------- Bridge Wiring ----------------
    def _wire_outbound_to_bridge(self) -> None:
        br = getattr(self.bridge, "_process", None) or getattr(self.bridge, "_proc", None)
        sig = getattr(br, "signals", None) if br else None
        if not sig:
            return

        if hasattr(sig, "initRequested"):
            self.initRequested.connect(sig.initRequested.emit)
        if hasattr(sig, "loadRecipeRequested"):
            self.loadRecipeRequested.connect(sig.loadRecipeRequested.emit)
        if hasattr(sig, "startRequested"):
            self.startRequested.connect(sig.startRequested.emit)
        if hasattr(sig, "stopRequested"):
            self.stopRequested.connect(sig.stopRequested.emit)

    def _wire_inbound_from_bridge(self) -> None:
        br = getattr(self.bridge, "_process", None) or getattr(self.bridge, "_proc", None) \
             or getattr(self.bridge, "_recipe", None)
        sig = getattr(br, "signals", None) if br else None
        if not sig:
            return

        # verschiedene mögliche Namensvarianten unterstützen
        if hasattr(sig, "recipeLoaded"):
            sig.recipeLoaded.connect(self.set_recipe_name)  # str
        if hasattr(sig, "currentRecipeChanged"):
            sig.currentRecipeChanged.connect(self.set_recipe_name)  # str
        if hasattr(sig, "recipeNameChanged"):
            sig.recipeNameChanged.connect(self.set_recipe_name)  # str
