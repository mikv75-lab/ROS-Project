# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations
import os
from typing import Optional, List

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QSizePolicy, QFileDialog, QMessageBox, QTextEdit
)

from app.model.recipe.recipe import Recipe
from app.widgets.robot_status_box import RobotStatusInfoBox
from app.widgets.info_groupbox import InfoGroupBox
from .process_thread import ProcessThread


class ProcessTab(QWidget):
    """
    Process-Tab (nur Code, keine .ui):

      VBOX(
        HBOX(
          [GroupBox] Process Control,
          [GroupBox] Startbedingungen,
          [GroupBox] Robot Status
        ),
        [InfoGroupBox]   <-- kompakte Info aus recipe.info
        HBOX(
          [GroupBox] Recipe (Text bis '# compiled poses'),
          [GroupBox] Poses  (alles ab '# compiled poses')
        )
      )
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # RobotBridge + Signals für Statusbox
        self._rb = getattr(self.bridge, "_rb", None)
        self._sig = getattr(self._rb, "signals", None) if self._rb else None

        # -------- interner Zustand --------
        self._robot_initialized: bool = False
        self._robot_at_home: bool = False

        self._recipe_model: Optional[Recipe] = None
        self._recipe_name: str | None = None

        self._tool_ok: bool = True
        self._substrate_ok: bool = False
        self._mount_ok: bool = False

        self._process_active: bool = False
        self._process_thread: Optional[ProcessThread] = None

        # ==================================================================
        # Layout (vormals ProcessControlWidget)
        # ==================================================================
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        # ---------- TOP ROW: Process-Control, Startbedingungen, RobotStatus ----------
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

        # --- Robot Status ---
        self.robotStatusBox = RobotStatusInfoBox(self, title="Robot Status")
        sp_rs = self.robotStatusBox.sizePolicy()
        sp_rs.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_rs.setVerticalPolicy(QSizePolicy.Policy.Fixed)
        self.robotStatusBox.setSizePolicy(sp_rs)
        top_row.addWidget(self.robotStatusBox, 2)

        # ---------- INFO BOX (unter Top-Row, über Recipe/Poses) ----------
        self.infoBox = InfoGroupBox(self)
        root.addWidget(self.infoBox)

        # ---------- BOTTOM ROW: Recipe-Text & Poses ----------
        bottom_row = QHBoxLayout()
        bottom_row.setContentsMargins(0, 0, 0, 0)
        bottom_row.setSpacing(8)
        root.addLayout(bottom_row, 1)

        # Recipe (Text bis '# compiled poses')
        self.grpRecipe = QGroupBox("Recipe", self)
        vrec = QVBoxLayout(self.grpRecipe)
        vrec.setContentsMargins(8, 8, 8, 8)
        vrec.setSpacing(4)

        self.txtRecipeSummary = QTextEdit(self.grpRecipe)
        self.txtRecipeSummary.setReadOnly(True)
        self.txtRecipeSummary.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vrec.addWidget(self.txtRecipeSummary)
        bottom_row.addWidget(self.grpRecipe, 1)

        # Poses (ab '# compiled poses')
        self.grpRecipeInfo = QGroupBox("Poses", self)
        vinfo = QVBoxLayout(self.grpRecipeInfo)
        vinfo.setContentsMargins(8, 8, 8, 8)
        vinfo.setSpacing(4)

        self.txtRecipePoses = QTextEdit(self.grpRecipeInfo)
        self.txtRecipePoses.setReadOnly(True)
        self.txtRecipePoses.setLineWrapMode(QTextEdit.LineWrapMode.NoWrap)
        vinfo.addWidget(self.txtRecipePoses)
        bottom_row.addWidget(self.grpRecipeInfo, 2)

        # ---------- Size-Policies ----------
        for gb in (self.grpProcess, self.grpStatus):
            sp = gb.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
            gb.setSizePolicy(sp)

        for gb in (self.grpRecipe, self.grpRecipeInfo):
            sp = gb.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.MinimumExpanding)
            gb.setSizePolicy(sp)

        # ==================================================================
        # Buttons -> Methoden
        # ==================================================================
        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnLoadRecipe.clicked.connect(self._on_load_recipe_clicked)
        self.btnStart.clicked.connect(self._on_start_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        self._wire_scene_bridge()
        self._wire_robot_status()

        # Info initial leeren
        self.infoBox.set_values(None)

        self._update_start_conditions()

        self._condTimer = QtCore.QTimer(self)
        self._condTimer.setInterval(500)
        self._condTimer.timeout.connect(self._update_start_conditions)
        self._update_timer_state()

        root.addStretch(1)

    # =====================================================================
    # Robot-Status Wiring
    # =====================================================================

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        if js is None or not hasattr(js, "position"):
            self.robotStatusBox.set_joints(None)
        else:
            self.robotStatusBox.set_joints(list(js.position or []))

    def _wire_robot_status(self) -> None:
        if not self._sig:
            return

        sig = self._sig
        sb = self.robotStatusBox

        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(sb.set_connection)
        if hasattr(sig, "modeChanged"):
            sig.modeChanged.connect(sb.set_mode)
        if hasattr(sig, "initializedChanged"):
            sig.initializedChanged.connect(sb.set_initialized)
        if hasattr(sig, "movingChanged"):
            sig.movingChanged.connect(sb.set_moving)
        if hasattr(sig, "powerChanged"):
            sig.powerChanged.connect(sb.set_power)
        if hasattr(sig, "servoEnabledChanged"):
            sig.servoEnabledChanged.connect(sb.set_servo_enabled)
        if hasattr(sig, "estopChanged"):
            sig.estopChanged.connect(sb.set_estop)
        if hasattr(sig, "errorsChanged"):
            sig.errorsChanged.connect(sb.set_errors)
        if hasattr(sig, "tcpPoseChanged"):
            sig.tcpPoseChanged.connect(sb.set_tcp_from_ps)
        if hasattr(sig, "jointsChanged"):
            sig.jointsChanged.connect(self._on_joints)

    # =====================================================================
    # ProcessThread-Handling
    # =====================================================================

    def _cleanup_process_thread(self) -> None:
        if self._process_thread is None:
            return
        thr = self._process_thread
        if thr.isRunning():
            thr.request_stop()
            thr.wait(2000)
        thr.deleteLater()
        self._process_thread = None

    def _setup_process_thread_for_recipe(self, recipe: Recipe) -> None:
        self._cleanup_process_thread()
        thr = ProcessThread(recipe=recipe, bridge=self.bridge, parent=self)
        thr.notifyFinished.connect(self._on_process_finished_success)
        thr.notifyError.connect(self._on_process_finished_error)
        thr.finished.connect(self._on_process_thread_finished)
        self._process_thread = thr

    # =====================================================================
    # Button-Slots
    # =====================================================================

    def _on_init_clicked(self) -> None:
        try:
            robot = getattr(self.bridge, "_robot", None) or getattr(self.bridge, "robot", None)
            if robot is not None:
                if hasattr(robot, "init_robot"):
                    robot.init_robot()
                elif hasattr(robot, "init"):
                    robot.init()
        except Exception:
            pass

        try:
            robot = getattr(self.bridge, "_robot", None) or getattr(self.bridge, "robot", None)
            if robot is not None:
                if hasattr(robot, "move_home"):
                    robot.move_home()
                elif hasattr(robot, "go_home"):
                    robot.go_home()
        except Exception:
            pass

        self.set_robot_initialized(True)
        self.set_robot_at_home(True)

    def _on_load_recipe_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())

        fname, _ = QFileDialog.getOpenFileName(
            self,
            "Rezept laden (Process)",
            start_dir,
            "YAML (*.yaml *.yml)"
        )
        if not fname:
            return

        try:
            model = Recipe.load_yaml(fname)
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", f"Rezept konnte nicht geladen werden:\n{e}")
            return

        self._recipe_model = model

        name = (model.id or "").strip() or os.path.basename(fname)
        self.set_recipe_name(name)

        # Text-Felder + InfoBox aktualisieren
        self._update_recipe_info_text()

        self._setup_process_thread_for_recipe(model)
        self._evaluate_scene_match()

    def _on_start_clicked(self) -> None:
        if not self.btnStart.isEnabled():
            return

        if self._recipe_model is None or self._process_thread is None:
            QMessageBox.warning(self, "Kein Rezept", "Es ist kein Rezept geladen.")
            return

        if self._process_thread.recipe is not self._recipe_model:
            self._process_thread.set_recipe(self._recipe_model)

        self.set_process_active(True)
        self._process_thread.startSignal.emit()

    def _on_stop_clicked(self) -> None:
        if self._process_thread is not None and self._process_thread.isRunning():
            self._process_thread.stopSignal.emit()

    # ---------------------------------------------------------------------
    # Callbacks vom ProcessThread
    # ---------------------------------------------------------------------

    def _on_process_finished_success(self) -> None:
        pass

    def _on_process_finished_error(self, msg: str) -> None:
        QMessageBox.critical(self, "Prozessfehler", f"Prozess abgebrochen:\n{msg}")

    def _on_process_thread_finished(self) -> None:
        self.set_process_active(False)

    # =====================================================================
    # Public API – von außen setzbar
    # =====================================================================

    def set_recipe_name(self, name: str | None) -> None:
        txt = (name or "").strip()
        self._recipe_name = txt if txt else None
        self._update_start_conditions()

    def _update_recipe_info_text(self) -> None:
        """
        Splittet str(Recipe) in:
          - txtRecipeSummary: alles vor '# compiled poses'
          - txtRecipePoses:   ab '# compiled poses'
        UND aktualisiert InfoGroupBox mit recipe.info.
        """
        if not self._recipe_model:
            self.txtRecipeSummary.setPlainText("")
            self.txtRecipePoses.setPlainText("")
            self.infoBox.set_values(None)
            return

        try:
            full_text = str(self._recipe_model)
        except Exception:
            full_text = f"id: {self._recipe_model.id}\n(no detailed dump available)"

        lines: List[str] = full_text.splitlines()
        split_idx = None
        for i, line in enumerate(lines):
            if line.strip().startswith("# compiled poses"):
                split_idx = i
                break

        if split_idx is None:
            summary_lines = lines
            poses_lines: List[str] = []
        else:
            summary_lines = lines[:split_idx]
            poses_lines = lines[split_idx:]

        self.txtRecipeSummary.setPlainText("\n".join(summary_lines).rstrip())
        self.txtRecipeSummary.moveCursor(self.txtRecipeSummary.textCursor().Start)

        self.txtRecipePoses.setPlainText("\n".join(poses_lines).rstrip())
        self.txtRecipePoses.moveCursor(self.txtRecipePoses.textCursor().Start)

        # InfoBox direkt aus recipe.info füttern
        self.infoBox.set_values(self._recipe_model.info or {})

    def set_robot_initialized(self, flag: bool) -> None:
        self._robot_initialized = bool(flag)
        self._update_start_conditions()

    def set_robot_at_home(self, flag: bool) -> None:
        self._robot_at_home = bool(flag)
        self._update_start_conditions()

    def set_tool_ok(self, flag: bool) -> None:
        self._tool_ok = True
        self._update_start_conditions()

    def set_substrate_ok(self, flag: bool) -> None:
        self._substrate_ok = bool(flag)
        self._update_start_conditions()

    def set_mount_ok(self, flag: bool) -> None:
        self._mount_ok = bool(flag)
        self._update_start_conditions()

    def set_process_active(self, active: bool) -> None:
        self._process_active = bool(active)
        self._update_timer_state()

        if self._process_active:
            if hasattr(self, "lblStatus") and self.lblStatus is not None:
                self.lblStatus.setText(
                    "Prozess läuft.\nStartbedingungen werden während der Ausführung nicht geprüft."
                )
        else:
            self._update_start_conditions()

    # =====================================================================
    # Startbedingungen + Timer
    # =====================================================================

    def _update_start_conditions(self) -> None:
        missing: list[str] = []

        if not self._robot_initialized:
            missing.append("Roboter nicht initialisiert")

        if not self._robot_at_home:
            missing.append("Roboter nicht in Home-Position")

        if not self._recipe_name:
            missing.append("Kein Rezept geladen")

        if not self._substrate_ok:
            missing.append("Substrate-Konfiguration stimmt nicht mit dem Rezept überein")

        if not self._mount_ok:
            missing.append("Mount-Konfiguration stimmt nicht mit dem Rezept überein")

        can_start = (len(missing) == 0)

        if self.btnStart is not None:
            self.btnStart.setEnabled(can_start and not self._process_active)

        if not hasattr(self, "lblStatus") or self.lblStatus is None:
            return

        if self._process_active:
            return

        if can_start:
            txt = "Startbereit.\nAlle Startbedingungen sind erfüllt."
        else:
            msg_lines = ["Start nicht möglich.", "Fehlende Bedingungen:"]
            msg_lines += [f"- {m}" for m in missing]
            txt = "\n".join(msg_lines)

        self.lblStatus.setText(txt)

    def _update_timer_state(self) -> None:
        if not hasattr(self, "_condTimer") or self._condTimer is None:
            return

        if self._process_active:
            if self._condTimer.isActive():
                self._condTimer.stop()
        else:
            if not self._condTimer.isActive():
                self._condTimer.start()

    # =====================================================================
    # Scene-Match (substrate/mount)
    # =====================================================================

    def _wire_scene_bridge(self) -> None:
        scene_br = getattr(self.bridge, "_scene", None)
        sig = getattr(scene_br, "signals", None) if scene_br else None
        if not sig:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        if hasattr(sig, "substrateCurrentChanged"):
            sig.substrateCurrentChanged.connect(lambda _s: self._evaluate_scene_match())
        if hasattr(sig, "mountCurrentChanged"):
            sig.mountCurrentChanged.connect(lambda _m: self._evaluate_scene_match())

        self._evaluate_scene_match()

    def _evaluate_scene_match(self) -> None:
        if not self._recipe_model:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        scene_br = getattr(self.bridge, "_scene", None)
        sig = getattr(scene_br, "signals", None) if scene_br else None
        if not sig:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        cur_sub = getattr(sig, "substrate_current", "") or ""
        cur_mount = getattr(sig, "mount_current", "") or ""

        rec_sub = (self._recipe_model.substrate or "").strip()
        rec_mount = (self._recipe_model.substrate_mount or "").strip()

        sub_ok = bool(rec_sub) and (cur_sub == rec_sub)
        mount_ok = bool(rec_mount) and (cur_mount == rec_mount)

        self.set_substrate_ok(sub_ok)
        self.set_mount_ok(mount_ok)
