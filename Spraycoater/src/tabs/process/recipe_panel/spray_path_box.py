# -*- coding: utf-8 -*-
# File: src/tabs/process/spray_path_box.py
from __future__ import annotations

import logging
from typing import Optional
from PyQt6 import QtCore
from PyQt6.QtWidgets import QGroupBox, QGridLayout, QLabel, QCheckBox, QWidget

_LOG = logging.getLogger("tabs.process.spray_path_box")

class SprayPathBox(QGroupBox):
    """UI für die Sichtbarkeits-Steuerung der Pfad-Layer (Entwurf / Geplant / Ausgeführt)."""

    def __init__(self, ros, parent: Optional[QWidget] = None, title: Optional[str] = "Spray Paths"):
        super().__init__(title or "", parent)
        self.ros = ros
        self._sig = self.ros.spray.signals # ROS Bridge Integration
        self._block = False

        self._build_ui()
        self._wire_signals()
        self.set_defaults()

    def _build_ui(self) -> None:
        g = QGridLayout(self)
        g.setContentsMargins(8, 8, 8, 8)

        def _header(txt: str, col: int):
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            g.addWidget(l, 0, col)

        _header("Layer", 0)
        _header("Show", 1)

        # Die 3 Kern-Layer des V2 Modells
        self.chkCompiled = QCheckBox(self)
        g.addWidget(QLabel("Compiled (Draft)", self), 1, 0)
        g.addWidget(self.chkCompiled, 1, 1)

        self.chkPlanned = QCheckBox(self)
        g.addWidget(QLabel("Planned (Planner)", self), 2, 0)
        g.addWidget(self.chkPlanned, 2, 1)

        self.chkExecuted = QCheckBox(self)
        g.addWidget(QLabel("Executed (Robot)", self), 3, 0)
        g.addWidget(self.chkExecuted, 3, 1)

    def _wire_signals(self) -> None:
        self.chkCompiled.toggled.connect(lambda v: self._emit_req("Compiled", v))
        self.chkPlanned.toggled.connect(lambda v: self._emit_req("Planned", v))
        self.chkExecuted.toggled.connect(lambda v: self._emit_req("Executed", v))

    def _emit_req(self, layer: str, val: bool):
        if self._block: return
        getattr(self._sig, f"show{layer}Requested").emit(val)

    def set_defaults(self, compiled=True, planned=True, executed=True):
        self._block = True
        self.chkCompiled.setChecked(compiled)
        self.chkPlanned.setChecked(planned)
        self.chkExecuted.setChecked(executed)
        self._block = False
        self.publish_current()

    def publish_current(self):
        self._emit_req("Compiled", self.chkCompiled.isChecked())
        self._emit_req("Planned", self.chkPlanned.isChecked())
        self._emit_req("Executed", self.chkExecuted.isChecked())