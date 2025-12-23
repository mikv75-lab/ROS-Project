# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import logging
from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout,
    QGroupBox, QMessageBox, QTextEdit, QSizePolicy
)

from app.model.recipe.recipe import Recipe
from app.model.recipe import recipe_markers
from ros.bridge.ros_bridge import RosBridge

from .process_thread import ProcessThread

_LOG = logging.getLogger("app.tabs.process")


class ProcessTab(QWidget):
    """
    ProcessTab

    - Buttons-only: Validate / Optimize / Execute / Stop
    - "eine Runde zählt": jeder Klick = ein frischer Run
    - Ergebnisse können ins Recipe geschrieben werden (optional)
    """

    def __init__(
        self,
        *,
        ctx,
        ros: RosBridge,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self.ctx = ctx
        self.ros = ros

        self._recipe: Optional[Recipe] = None
        self._process_thread: Optional[ProcessThread] = None
        self._process_active: bool = False
        self._active_mode: str = ""

        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        grp_proc = QGroupBox("Process", self)
        vproc = QVBoxLayout(grp_proc)

        self.btnValidate = QPushButton("Validate", grp_proc)
        self.btnOptimize = QPushButton("Optimize", grp_proc)
        self.btnExecute  = QPushButton("Execute", grp_proc)
        self.btnStop     = QPushButton("Stop", grp_proc)

        for b in (self.btnValidate, self.btnOptimize, self.btnExecute, self.btnStop):
            b.setMinimumHeight(28)
            vproc.addWidget(b)

        root.addWidget(grp_proc)

        grp_log = QGroupBox("Process Status / Log", self)
        vlog = QVBoxLayout(grp_log)

        self.lblStatus = QLabel("Kein Prozess aktiv.", grp_log)
        self.lblStatus.setWordWrap(True)

        self.txtLog = QTextEdit(grp_log)
        self.txtLog.setReadOnly(True)
        self.txtLog.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)

        vlog.addWidget(self.lblStatus)
        vlog.addWidget(self.txtLog, 1)

        root.addWidget(grp_log, 1)

        grp_proc.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        grp_log.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

        self.btnValidate.clicked.connect(self._on_validate_clicked)
        self.btnOptimize.clicked.connect(self._on_optimize_clicked)
        self.btnExecute.clicked.connect(self._on_execute_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        self._update_buttons()

    # ---------------- Public API ----------------

    def set_recipe(self, recipe: Optional[Recipe]) -> None:
        self._recipe = recipe
        self._append_log(f"Recipe gesetzt: {getattr(recipe, 'id', None)}")
        self._update_buttons()

        if recipe:
            self._publish_recipe(recipe)

    # ---------------- Button handlers ----------------

    def _on_validate_clicked(self) -> None:
        if not self._ensure_recipe("Validate"):
            return
        self._start_process(ProcessThread.MODE_VALIDATE)

    def _on_optimize_clicked(self) -> None:
        if not self._ensure_recipe("Optimize"):
            return
        self._start_process(ProcessThread.MODE_OPTIMIZE)

    def _on_execute_clicked(self) -> None:
        if not self._ensure_recipe("Execute"):
            return
        self._start_process(ProcessThread.MODE_EXECUTE)

    def _on_stop_clicked(self) -> None:
        if self._process_thread:
            self._append_log("Stop angefordert.")
            self._process_thread.request_stop()

    # ---------------- Process control ----------------

    def _start_process(self, mode: str) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Process", "Es läuft bereits ein Prozess.")
            return

        self._cleanup_process_thread()

        self.txtLog.clear()
        self._active_mode = mode
        self._append_log(f"=== {mode.upper()} gestartet ===")

        self._process_thread = ProcessThread(
            recipe=self._recipe,
            ros=self.ros,
            mode=mode,
            parent=self,  # Ownership UI-seitig
        )

        self._process_thread.stateChanged.connect(self._on_state_changed)
        self._process_thread.logMessage.connect(self._append_log)
        self._process_thread.notifyFinished.connect(self._on_process_finished_success)
        self._process_thread.notifyError.connect(self._on_process_finished_error)
        self._process_thread.finished.connect(self._on_thread_finished)

        self._process_active = True
        self._update_buttons()
        self.lblStatus.setText(f"Prozess läuft: {mode}")

        self._process_thread.startSignal.emit()

    # ---------------- Callbacks ----------------

    @QtCore.pyqtSlot(object)
    def _on_process_finished_success(self, result: object) -> None:
        self._append_log("=== Prozess erfolgreich abgeschlossen ===")

        # optional: nur wenn dict/recipe
        if isinstance(result, dict) and self._recipe:
            planned = result.get("planned_traj")
            if isinstance(planned, dict):
                self._store_and_eval_traj("traj", planned)

            executed = result.get("executed_traj")
            if isinstance(executed, dict):
                self._store_and_eval_traj(Recipe.TRAJ_EXECUTED, executed)

            self._publish_recipe(self._recipe)

        self._process_active = False
        self._update_buttons()
        self.lblStatus.setText("Prozess abgeschlossen.")

    @QtCore.pyqtSlot(str)
    def _on_process_finished_error(self, msg: str) -> None:
        self._append_log(f"=== Prozessfehler: {msg} ===")
        QMessageBox.critical(self, "Prozessfehler", msg)

        self._process_active = False
        self._update_buttons()
        self.lblStatus.setText("Prozess abgebrochen.")

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        # ProcessThread quittet selbst – wir räumen UI-seitig nur auf
        self._cleanup_process_thread()

    @QtCore.pyqtSlot(str)
    def _on_state_changed(self, state: str) -> None:
        self.lblStatus.setText(f"{self._active_mode}: {state}")

    # ---------------- Recipe helpers ----------------

    def _store_and_eval_traj(self, traj_key: str, traj_data: Dict[str, Any]) -> None:
        if not self._recipe:
            return

        frame = traj_data.get("frame", "scene")
        sides = traj_data.get("sides") or {}

        for side, sdata in sides.items():
            points = sdata.get("points_mm")
            if points is None:
                continue

            self._recipe.set_trajectory_points_mm(
                traj_key=traj_key,
                side=str(side),
                points_mm=points,
                frame=frame,
            )
            self._recipe.evaluate_trajectory_against_compiled(
                traj_key=traj_key,
                side=str(side),
            )
            self._append_log(f"{traj_key}/{side}: gespeichert + evaluiert")

    def _publish_recipe(self, recipe: Recipe) -> None:
        ma = recipe_markers.build_marker_array_from_recipe(
            recipe,
            sides=None,
            frame_id="scene",
        )
        self.ros.set_spraypath(ma)

    # ---------------- Utils ----------------

    def _cleanup_process_thread(self) -> None:
        pt = self._process_thread
        if pt is None:
            return

        try:
            if pt.isRunning():
                pt.request_stop()
                pt.wait(2000)
        except Exception:
            pass

        try:
            pt.deleteLater()
        except Exception:
            pass

        self._process_thread = None

    def _append_log(self, msg: str) -> None:
        if msg:
            self.txtLog.append(msg)

    def _ensure_recipe(self, title: str) -> bool:
        if not self._recipe:
            QMessageBox.warning(self, title, "Kein Recipe geladen.")
            return False
        return True

    def _update_buttons(self) -> None:
        idle = not self._process_active
        has_recipe = self._recipe is not None

        self.btnValidate.setEnabled(idle and has_recipe)
        self.btnOptimize.setEnabled(idle and has_recipe)
        self.btnExecute.setEnabled(idle and has_recipe)
        self.btnStop.setEnabled(not idle)
