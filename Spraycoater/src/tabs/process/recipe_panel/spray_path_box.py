# -*- coding: utf-8 -*-
# File: src/tabs/process/spray_path_box.py
from __future__ import annotations

import logging
from typing import Optional

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QGroupBox,
    QGridLayout,
    QLabel,
    QCheckBox,
    QWidget,
    QSpacerItem,
    QSizePolicy,
)

_LOG = logging.getLogger("tabs.process.spray_path_box")


class SprayPathBox(QGroupBox):
    """
    UI für die Sichtbarkeits-Steuerung der Pfad-Layer (STRICT 2026-01):
      - Compiled (Draft): 1 Toggle
      - Planned: Run + Stored
      - Executed: Run + Stored
    => 5 Checkboxen
    """

    def __init__(self, ros, parent: Optional[QWidget] = None, title: Optional[str] = "Spray Paths"):
        super().__init__(title or "", parent)
        self.ros = ros
        self._sig = self.ros.spray.signals
        self._block = False

        self._build_ui()
        self._wire_signals()
        self.set_defaults()

    def _build_ui(self) -> None:
        g = QGridLayout(self)
        g.setContentsMargins(8, 8, 8, 8)
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(4)

        def _header(txt: str, col: int, align=None):
            l = QLabel(txt, self)
            l.setStyleSheet("font-weight: 600;")
            if align is None:
                g.addWidget(l, 0, col)
            else:
                g.addWidget(l, 0, col, alignment=align)

        _header("Layer", 0, QtCore.Qt.AlignmentFlag.AlignLeft)
        _header("Show", 1, QtCore.Qt.AlignmentFlag.AlignRight)

        # ---- Compiled (single) ----
        self.chkCompiled = QCheckBox(self)
        g.addWidget(QLabel("Compiled (Draft)", self), 1, 0)
        g.addWidget(self.chkCompiled, 1, 1, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        # ---- Planned (split) ----
        self.chkPlannedRun = QCheckBox(self)
        g.addWidget(QLabel("Planned Run (New)", self), 2, 0)
        g.addWidget(self.chkPlannedRun, 2, 1, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        self.chkPlannedStored = QCheckBox(self)
        g.addWidget(QLabel("Planned Stored (Ghost)", self), 3, 0)
        g.addWidget(self.chkPlannedStored, 3, 1, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        # ---- Executed (split) ----
        self.chkExecutedRun = QCheckBox(self)
        g.addWidget(QLabel("Executed Run (New)", self), 4, 0)
        g.addWidget(self.chkExecutedRun, 4, 1, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        self.chkExecutedStored = QCheckBox(self)
        g.addWidget(QLabel("Executed Stored (Ghost)", self), 5, 0)
        g.addWidget(self.chkExecutedStored, 5, 1, alignment=QtCore.Qt.AlignmentFlag.AlignRight)

        # Columns: labels stretch, checkbox column fixed/right
        g.setColumnStretch(0, 1)
        g.setColumnStretch(1, 0)

        # ✅ IMPORTANT: push free vertical space to the bottom (not between rows)
        g.addItem(
            QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding),
            6, 0, 1, 2
        )
        g.setRowStretch(6, 1)

    def _wire_signals(self) -> None:
        self.chkCompiled.toggled.connect(lambda v: self._emit_req("Compiled", v))
        self.chkPlannedRun.toggled.connect(lambda v: self._emit_req("PlannedRun", v))
        self.chkPlannedStored.toggled.connect(lambda v: self._emit_req("PlannedStored", v))
        self.chkExecutedRun.toggled.connect(lambda v: self._emit_req("ExecutedRun", v))
        self.chkExecutedStored.toggled.connect(lambda v: self._emit_req("ExecutedStored", v))

    def _emit_req(self, key: str, val: bool) -> None:
        if self._block:
            return
        sig_name = f"show{key}Requested"
        sig = getattr(self._sig, sig_name, None)
        if sig is None:
            _LOG.error("SprayPathBox: missing signal %s on ros.spray.signals", sig_name)
            return
        sig.emit(bool(val))

    def set_defaults(
        self,
        *,
        compiled: bool = True,
        planned_run: bool = True,
        planned_stored: bool = True,
        executed_run: bool = True,
        executed_stored: bool = True,
    ) -> None:
        self._block = True
        self.chkCompiled.setChecked(bool(compiled))
        self.chkPlannedRun.setChecked(bool(planned_run))
        self.chkPlannedStored.setChecked(bool(planned_stored))
        self.chkExecutedRun.setChecked(bool(executed_run))
        self.chkExecutedStored.setChecked(bool(executed_stored))
        self._block = False
        self.publish_current()

    def publish_current(self) -> None:
        self._emit_req("Compiled", self.chkCompiled.isChecked())
        self._emit_req("PlannedRun", self.chkPlannedRun.isChecked())
        self._emit_req("PlannedStored", self.chkPlannedStored.isChecked())
        self._emit_req("ExecutedRun", self.chkExecutedRun.isChecked())
        self._emit_req("ExecutedStored", self.chkExecutedStored.isChecked())
