# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations
import os
from typing import Optional, List

from PyQt6 import QtCore
from PyQt6.sip import isdeleted
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QSizePolicy, QFileDialog, QMessageBox, QTextEdit
)

from app.model.recipe.recipe import Recipe
from app.widgets.robot_status_box import RobotStatusInfoBox
from app.widgets.info_groupbox import InfoGroupBox
from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread


class ProcessTab(QWidget):
    """
    Process-Tab (nur Code, keine .ui):

      VBOX(
        HBOX(
          [GroupBox] Process Control,
          [GroupBox] Startbedingungen,
          [GroupBox] Setup (Tool / Substrate / Mount),
          [GroupBox] Robot Status
        ),
        [InfoGroupBox]
        [GroupBox] Recipe
           ├─ QTextEdit  (Summary)
           └─ QTextEdit  (Poses)
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
        self._robot_init_thread: Optional[RobotInitThread] = None

        # ==================================================================
        # Layout
        # ==================================================================
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

        # ==================================================================
        # Threads
        # ==================================================================
        if self.bridge is not None:
            self._robot_init_thread = RobotInitThread(
                bridge=self.bridge,
                parent=self,
                init_timeout_s=10.0,
                home_timeout_s=60.0,
            )
            self._robot_init_thread.notifyFinished.connect(self._on_robot_init_finished)
            self._robot_init_thread.notifyError.connect(self._on_robot_init_error)

        # ==================================================================
        # Buttons -> Methoden
        # ==================================================================
        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnLoadRecipe.clicked.connect(self._on_load_recipe_clicked)
        self.btnStart.clicked.connect(self._on_start_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        self._wire_scene_bridge()
        self._wire_robot_status()

        self.infoBox.set_values(None)

        self._condTimer = QtCore.QTimer(self)
        self._condTimer.setInterval(500)
        self._condTimer.timeout.connect(self._update_start_conditions)

        self._update_start_conditions()
        self._update_timer_state()

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
            # Home-Flag aus Mode ableiten (HOMED)
            def _on_mode(mode: str):
                m = (mode or "").strip().lower()
                self.set_robot_at_home(m == "homed")
            sig.modeChanged.connect(_on_mode)
        if hasattr(sig, "initializedChanged"):
            sig.initializedChanged.connect(sb.set_initialized)
            sig.initializedChanged.connect(self.set_robot_initialized)
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
        """
        Startet die Init-StateMachine:

          - Robot init (RobotBridge-Status)
          - MoveToHome (MotionBridge/RobotBridge)
          - Warten bis Mode = HOMED
        """
        if self._robot_init_thread is None:
            QMessageBox.warning(self, "Robot Init", "RobotInitThread nicht verfügbar.")
            return

        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        if not isdeleted(self.btnInit):
            self.btnInit.setEnabled(False)

        self.infoBox.set_values({"process": "Robot-Init läuft (Init + Home über RobotBridge-Status)..."})

        self._robot_init_thread.startSignal.emit()

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

        self._update_recipe_info_text()
        self._setup_process_thread_for_recipe(model)
        self._evaluate_scene_match()
        self._update_setup_from_scene()

    def _on_start_clicked(self) -> None:
        if isdeleted(self) or isdeleted(self.btnStart):
            return

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
    # Callbacks Robot-Init
    # ---------------------------------------------------------------------

    def _on_robot_init_finished(self) -> None:
        self.set_robot_initialized(True)
        self.set_robot_at_home(True)
        if not isdeleted(self.btnInit):
            self.btnInit.setEnabled(True)

        cur = self.infoBox.get_values() if hasattr(self.infoBox, "get_values") else {}
        cur = dict(cur or {})
        cur["robot_init"] = "Roboter initialisiert (Status HOMED/READY)."
        self.infoBox.set_values(cur)

    def _on_robot_init_error(self, msg: str) -> None:
        self.set_robot_initialized(False)
        self.set_robot_at_home(False)
        if not isdeleted(self.btnInit):
            self.btnInit.setEnabled(True)
        QMessageBox.warning(self, "Robot Init fehlgeschlagen", msg or "Unbekannter Fehler.")

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
    # Public API
    # =====================================================================

    def set_recipe_name(self, name: str | None) -> None:
        txt = (name or "").strip()
        self._recipe_name = txt if txt else None
        self._update_start_conditions()

    def _update_recipe_info_text(self) -> None:
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

        self.infoBox.set_values(self._recipe_model.info or {})

    def set_robot_initialized(self, flag: bool) -> None:
        self._robot_initialized = bool(flag)
        self._update_start_conditions()

    def set_robot_at_home(self, flag: bool) -> None:
        self._robot_at_home = bool(flag)
        self._update_start_conditions()

    def set_tool_ok(self, flag: bool) -> None:
        self._tool_ok = bool(flag)
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
            if hasattr(self, "lblStatus") and self.lblStatus is not None and not isdeleted(self.lblStatus):
                self.lblStatus.setText(
                    "Prozess läuft.\nStartbedingungen werden während der Ausführung nicht geprüft."
                )
        else:
            self._update_start_conditions()

    # =====================================================================
    # Startbedingungen + Timer
    # =====================================================================

    def _update_start_conditions(self) -> None:
        if isdeleted(self):
            return

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

        if hasattr(self, "btnStart") and self.btnStart is not None and not isdeleted(self.btnStart):
            self.btnStart.setEnabled(can_start and not self._process_active)

        if not hasattr(self, "lblStatus") or self.lblStatus is None or isdeleted(self.lblStatus):
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
        if not hasattr(self, "_condTimer") or self._condTimer is None or isdeleted(self._condTimer):
            return

        if self._process_active:
            if self._condTimer.isActive():
                self._condTimer.stop()
        else:
            if not self._condTimer.isActive():
                self._condTimer.start()

    # =====================================================================
    # Scene-Match + Setup-Anzeige
    # =====================================================================

    def _wire_scene_bridge(self) -> None:
        scene_br = getattr(self.bridge, "_sb", None) or getattr(self.bridge, "_scene", None)
        sig = getattr(scene_br, "signals", None) if scene_br else None
        if not sig:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            self._set_setup_labels("-", "-", "-")
            return

        if hasattr(sig, "substrateCurrentChanged"):
            sig.substrateCurrentChanged.connect(self._on_scene_changed)
        if hasattr(sig, "mountCurrentChanged"):
            sig.mountCurrentChanged.connect(self._on_scene_changed)
        if hasattr(sig, "toolCurrentChanged"):
            sig.toolCurrentChanged.connect(self._on_scene_changed)

        self._update_setup_from_scene()
        self._evaluate_scene_match()

    def _on_scene_changed(self, *_args) -> None:
        self._update_setup_from_scene()
        self._evaluate_scene_match()

    def _update_setup_from_scene(self) -> None:
        tool = ""
        substrate = ""
        mount = ""

        scene_br = getattr(self.bridge, "_sb", None) or getattr(self.bridge, "_scene", None)
        sig_s = getattr(scene_br, "signals", None) if scene_br else None
        if sig_s:
            tool = getattr(sig_s, "tool_current", "") or getattr(sig_s, "tool", "") or ""
            substrate = getattr(sig_s, "substrate_current", "") or ""
            mount = getattr(sig_s, "mount_current", "") or ""

        if not tool and self._rb is not None:
            tool = getattr(self._rb, "current_tool", "") or getattr(self._rb, "tool", "")

        self._set_setup_labels(tool or "-", substrate or "-", mount or "-")

    def _set_setup_labels(self, tool: str, substrate: str, mount: str) -> None:
        self.lblTool.setText(f"Tool: {tool}")
        self.lblSubstrate.setText(f"Substrate: {substrate}")
        self.lblMount.setText(f"Mount: {mount}")

    def _evaluate_scene_match(self) -> None:
        if not self._recipe_model:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        scene_br = getattr(self.bridge, "_sb", None) or getattr(self.bridge, "_scene", None)
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
