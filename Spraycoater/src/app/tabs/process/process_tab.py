# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import os
import math
import logging
from typing import Optional, List, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QSizePolicy, QFileDialog, QMessageBox, QTextEdit
)

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

from app.model.recipe.recipe import Recipe
from app.model.recipe import recipe_markers
from app.widgets.robot_status_box import RobotStatusInfoBox
from app.widgets.info_groupbox import InfoGroupBox
from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

_LOG = logging.getLogger("app.tabs.process")


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
        [GroupBox] Process Status / Log
           ├─ QLabel    (aktueller State)
           └─ QTextEdit (Log)
        [GroupBox] Recipe
           ├─ QTextEdit  (Summary)
           └─ QTextEdit  (Poses)
      )
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # Bridges (fest gecacht)
        self._rb = bridge._rb if bridge is not None else None
        self._pb = bridge._pb if bridge is not None else None
        self._poses_state = bridge.poses if bridge is not None else None
        self._sb = bridge._sb if bridge is not None else None  # SceneBridge

        self._sig = self._rb.signals if self._rb is not None else None

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

        # Info-Box Cache
        self._info_values: Dict[str, Any] = {}

        # Reentrancy-Guard
        self._in_update_start_conditions: bool = False

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

        # ==================================================================
        # Threads
        # ==================================================================
        if self.bridge is not None:
            self._robot_init_thread = RobotInitThread(
                bridge=self.bridge,
                parent=self,
                init_timeout_s=10.0,
                home_timeout_s=60.0,
                pos_tol_mm=1.0,
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

        # Wiring
        self._wire_scene_bridge()
        self._wire_robot_status()
        self._wire_poses_bridge()

        self._info_values.clear()
        self.infoBox.set_values(None)

        self._condTimer = QtCore.QTimer(self)
        self._condTimer.setInterval(500)
        self._condTimer.timeout.connect(self._update_start_conditions)

        self._update_start_conditions()
        self._update_timer_state()

    # =====================================================================
    # Hilfsfunktionen für InfoBox / ProcessLog
    # =====================================================================

    def _info_set_all(self, values: Optional[Dict[str, Any]]) -> None:
        """Gesamten Info-Cache ersetzen und InfoBox updaten."""
        self._info_values = dict(values or {})
        try:
            if self._info_values:
                self.infoBox.set_values(dict(self._info_values))
            else:
                self.infoBox.set_values(None)
        except RuntimeError:
            pass

    def _info_update(self, key: str, value: Any | None) -> None:
        """Einzelnen Key im Info-Cache setzen/löschen und InfoBox updaten."""
        if value is None:
            self._info_values.pop(key, None)
        else:
            self._info_values[key] = value

        try:
            if self._info_values:
                self.infoBox.set_values(dict(self._info_values))
            else:
                self.infoBox.set_values(None)
        except RuntimeError:
            pass

    def _append_process_log(self, text: str) -> None:
        """Text in die Process-Logbox anhängen."""
        if not text:
            return
        try:
            self.txtProcessLog.append(text)
        except RuntimeError:
            pass

    @QtCore.pyqtSlot(str)
    def _on_process_log_message(self, msg: str) -> None:
        """Slot für ProcessThread.logMessage."""
        self._append_process_log(msg)

    # =====================================================================
    # Robot-Status Wiring
    # =====================================================================

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        try:
            if js is None or not hasattr(js, "position"):
                self.robotStatusBox.set_joints(None)
            else:
                self.robotStatusBox.set_joints(list(js.position or []))
        except RuntimeError:
            pass

    def _wire_robot_status(self) -> None:
        if self._sig is None:
            return

        sig = self._sig
        sb = self.robotStatusBox

        if hasattr(sig, "connectionChanged"):
            sig.connectionChanged.connect(sb.set_connection)
        if hasattr(sig, "modeChanged"):
            sig.modeChanged.connect(sb.set_mode)
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
            sig.tcpPoseChanged.connect(self._on_tcp_pose_changed)
        if hasattr(sig, "jointsChanged"):
            sig.jointsChanged.connect(self._on_joints)

    # =====================================================================
    # Poses-Bridge: Home-Pose beobachten
    # =====================================================================

    def _wire_poses_bridge(self) -> None:
        if self._pb is None:
            return
        sigp = getattr(self._pb, "signals", None)
        if sigp is None:
            return
        if hasattr(sigp, "homePoseChanged"):
            sigp.homePoseChanged.connect(self._on_home_pose_changed)

    @QtCore.pyqtSlot(object)
    def _on_tcp_pose_changed(self, _msg: object) -> None:
        self._recompute_robot_at_home()

    @QtCore.pyqtSlot(object)
    def _on_home_pose_changed(self, _msg: object) -> None:
        self._recompute_robot_at_home()

    # =====================================================================
    # Pose-basierter Home-Check
    # =====================================================================

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        if self._rb is None:
            return None
        pose = getattr(self._rb, "tcp_pose", None)
        if isinstance(pose, PoseStamped) or pose is None:
            return pose
        return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        if self._pb is not None:
            try:
                pose = self._pb.get_last_home_pose()
                if isinstance(pose, PoseStamped):
                    return pose
            except Exception:
                pass

        if self._poses_state is not None:
            try:
                pose2 = self._poses_state.home()
                if isinstance(pose2, PoseStamped):
                    return pose2
            except Exception:
                pass

        return None

    @staticmethod
    def _poses_close(a: PoseStamped, b: PoseStamped, pos_tol_mm: float = 1.0) -> bool:
        try:
            dx = float(a.pose.position.x) - float(b.pose.position.x)
            dy = float(a.pose.position.y) - float(b.pose.position.y)
            dz = float(a.pose.position.z) - float(b.pose.position.z)
        except Exception:
            return False

        dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        dist_mm = dist_m * 1000.0
        return dist_mm <= pos_tol_mm

    def _recompute_robot_at_home(self) -> None:
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        if cur is None or home is None:
            self.set_robot_at_home(False)
            return

        at_home = self._poses_close(cur, home, pos_tol_mm=1.0)
        self.set_robot_at_home(at_home)

    # =====================================================================
    # ProcessThread-Handling
    # =====================================================================

    def _cleanup_process_thread(self) -> None:
        thr = self._process_thread
        self._process_thread = None

        if thr is None:
            return

        try:
            if thr.isRunning():
                thr.request_stop()
                thr.wait(2000)
            thr.deleteLater()
        except RuntimeError:
            pass

    def _setup_process_thread_for_recipe(self, recipe: Recipe) -> None:
        """
        Erstellt einen neuen ProcessThread für das aktuelle Rezept.
        """
        self._cleanup_process_thread()

        thr = ProcessThread(recipe=recipe, bridge=self.bridge)
        thr.notifyFinished.connect(self._on_process_finished_success)
        thr.notifyError.connect(self._on_process_finished_error)
        thr.finished.connect(self._on_process_thread_finished)
        thr.stateChanged.connect(self._on_process_state_changed)
        thr.logMessage.connect(self._on_process_log_message)
        self._process_thread = thr

        # Beim Laden eines neuen Rezepts das Log leeren
        try:
            self.txtProcessLog.clear()
        except RuntimeError:
            pass

    # =====================================================================
    # SprayPath-Ansteuerung
    # =====================================================================

    def _send_recipe_to_spraypath(self, recipe: Recipe) -> None:
        if self.bridge is None or recipe is None:
            return

        try:
            ma: MarkerArray = recipe_markers.build_marker_array_from_recipe(
                recipe,
                sides=None,
                frame_id="scene",
            )

            if ma and isinstance(ma, MarkerArray) and ma.markers:
                try:
                    self.bridge.set_spraypath(ma)
                    _LOG.info(
                        "ProcessTab: MarkerArray mit %d Markern an SprayPath gesendet (frame=scene)",
                        len(ma.markers),
                    )
                except Exception:
                    _LOG.exception("ProcessTab: bridge.set_spraypath failed")
            else:
                _LOG.warning(
                    "ProcessTab: build_marker_array_from_recipe lieferte kein/nur leeres MarkerArray"
                )
        except Exception:
            _LOG.exception(
                "ProcessTab: Fehler beim Erzeugen/Publizieren des MarkerArray für SprayPath"
            )

    # =====================================================================
    # Button-Slots
    # =====================================================================

    def _on_init_clicked(self) -> None:
        if self._robot_init_thread is None:
            QMessageBox.warning(self, "Robot Init", "RobotInitThread nicht verfügbar.")
            return

        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        try:
            self.btnInit.setEnabled(False)
        except RuntimeError:
            pass

        self._info_update("process", "Robot-Init läuft (Init + Home via Pose-Vergleich)...")

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

        try:
            self._update_recipe_info_text()
        except RuntimeError:
            return

        self._setup_process_thread_for_recipe(model)
        # FIX: korrekter Methodenname
        self._send_recipe_to_spraypath(model)

        self._update_setup_from_scene()
        self._evaluate_scene_match()

        # Status/Log zurücksetzen
        try:
            self.lblProcessState.setText("Kein Prozess aktiv.")
        except RuntimeError:
            pass

    def _on_start_clicked(self) -> None:
        try:
            if not self.btnStart.isEnabled():
                return
        except RuntimeError:
            return

        if self._recipe_model is None or self._process_thread is None:
            QMessageBox.warning(self, "Kein Rezept", "Es ist kein Rezept geladen.")
            return

        if self._process_thread.recipe is not self._recipe_model:
            self._process_thread.set_recipe(self._recipe_model)

        self.set_process_active(True)
        self._append_process_log("=== Prozess gestartet ===")
        self._process_thread.startSignal.emit()

    def _on_stop_clicked(self) -> None:
        self._append_process_log("=== Stop angefordert ===")

        if self._process_thread is not None and self._process_thread.isRunning():
            self._process_thread.stopSignal.emit()

        if self._robot_init_thread is not None and self._robot_init_thread.isRunning():
            self._robot_init_thread.stopSignal.emit()

    # ---------------------------------------------------------------------
    # Callbacks Robot-Init
    # ---------------------------------------------------------------------

    def _on_robot_init_finished(self) -> None:
        self.set_robot_initialized(True)
        self._recompute_robot_at_home()

        try:
            self.btnInit.setEnabled(True)
        except RuntimeError:
            pass

        text = "Roboter initialisiert und Home-Pose erreicht (TCP≈Home)."
        self._info_update("robot_init", text)
        if self._recipe_name:
            self._info_update("recipe", self._recipe_name)

    def _on_robot_init_error(self, msg: str) -> None:
        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        try:
            self.btnInit.setEnabled(True)
        except RuntimeError:
            pass

        QMessageBox.warning(self, "Robot Init fehlgeschlagen", msg or "Unbekannter Fehler.")

    # ---------------------------------------------------------------------
    # Callbacks vom ProcessThread
    # ---------------------------------------------------------------------

    def _on_process_finished_success(self) -> None:
        self.set_process_active(False)
        self._info_update("process", "Prozess erfolgreich abgeschlossen.")
        self._append_process_log("=== Prozess erfolgreich abgeschlossen ===")

        try:
            self.lblProcessState.setText("Prozess erfolgreich abgeschlossen.")
        except RuntimeError:
            pass

    def _on_process_finished_error(self, msg: str) -> None:
        self.set_process_active(False)
        self._info_update("process", f"Fehler: {msg}")
        self._append_process_log(f"=== Prozessfehler: {msg} ===")

        try:
            self.lblProcessState.setText(f"Fehler: {msg}")
        except RuntimeError:
            pass

        try:
            QMessageBox.critical(self, "Prozessfehler", f"Prozess abgebrochen:\n{msg}")
        except RuntimeError:
            pass

    def _on_process_thread_finished(self) -> None:
        self.set_process_active(False)

    @QtCore.pyqtSlot(str)
    def _on_process_state_changed(self, state: str) -> None:
        pretty = {
            "MOVE_PREDISPENSE": "Predispense-Position anfahren",
            "WAIT_PREDISPENSE": "Predispense-Zeit warten",
            "MOVE_RECIPE": "Rezeptpfad fahren",
            "WAIT_POSTDISPENSE": "Postdispense-Zeit warten",
            "MOVE_RETREAT": "Retreat-Position anfahren",
            "MOVE_HOME": "Home-Position anfahren",
            "ERROR": "Fehler – Prozess abgebrochen",
        }.get(state, state)

        try:
            self.lblProcessState.setText(f"Aktueller Schritt: {pretty}")
        except RuntimeError:
            pass

        try:
            prefix = "Prozess läuft.\n" if self._process_active else ""
            self.lblStatus.setText(f"{prefix}Aktueller Schritt: {pretty}")
        except RuntimeError:
            pass

        self._info_update("process_state", pretty)
        if self._recipe_name:
            self._info_update("recipe", self._recipe_name)

    # =====================================================================
    # Public API
    # =====================================================================

    def set_recipe_name(self, name: str | None) -> None:
        txt = (name or "").strip()
        self._recipe_name = txt if txt else None
        self._update_start_conditions()

    def _update_recipe_info_text(self) -> None:
        if not self._recipe_model:
            try:
                self.txtRecipeSummary.setPlainText("")
                self.txtRecipePoses.setPlainText("")
            except RuntimeError:
                return
            self._info_set_all(None)
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

        summary_text = "\n".join(summary_lines).rstrip()
        poses_text = "\n".join(poses_lines).rstrip()

        try:
            self.txtRecipeSummary.setPlainText(summary_text)
            self.txtRecipeSummary.moveCursor(self.txtRecipeSummary.textCursor().Start)

            self.txtRecipePoses.setPlainText(poses_text)
            self.txtRecipePoses.moveCursor(self.txtRecipePoses.textCursor().Start)
        except RuntimeError:
            return

        base_info = self._recipe_model.info or {}
        self._info_set_all(base_info)

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
            try:
                self.lblStatus.setText(
                    "Prozess läuft.\nStartbedingungen werden während der Ausführung nicht geprüft."
                )
            except RuntimeError:
                pass
        else:
            self._update_start_conditions()

    # =====================================================================
    # Startbedingungen + Timer
    # =====================================================================

    @staticmethod
    def _norm_mesh_name(name: str) -> str:
        n = os.path.basename(name or "").strip()
        if n.lower().endswith(".stl"):
            n = n[:-4]
        return n

    def _update_start_conditions(self) -> None:
        if self._in_update_start_conditions:
            return

        self._in_update_start_conditions = True
        try:
            self._update_setup_from_scene()
            self._evaluate_scene_match()

            missing: list[str] = []

            if not self._robot_initialized:
                missing.append("Roboter nicht initialisiert")

            if not self._robot_at_home:
                missing.append("Roboter nicht in Home-Position (TCP ≉ Home-Pose)")

            if not self._recipe_name:
                missing.append("Kein Rezept geladen")

            if not self._substrate_ok:
                missing.append("Substrate-Konfiguration stimmt nicht mit dem Rezept überein")

            if not self._mount_ok:
                missing.append("Mount-Konfiguration stimmt nicht mit dem Rezept überein")

            can_start = (len(missing) == 0)

            try:
                self.btnStart.setEnabled(can_start and not self._process_active)
            except RuntimeError:
                pass

            if self._process_active:
                return

            if can_start:
                txt = "Startbereit.\nAlle Startbedingungen sind erfüllt."
            else:
                msg_lines = ["Start nicht möglich.", "Fehlende Bedingungen:"]
                msg_lines += [f"- {m}" for m in missing]
                txt = "\n".join(msg_lines)

            try:
                self.lblStatus.setText(txt)
            except RuntimeError:
                pass
        finally:
            self._in_update_start_conditions = False

    def _update_timer_state(self) -> None:
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
        if self._sb is None:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            self._set_setup_labels("-", "-", "-")
            return

        sig = getattr(self._sb, "signals", None)
        if sig is None:
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

    def _get_scene_currents(self) -> tuple[str, str, str]:
        if self._sb is None:
            return "", "", ""

        tool = getattr(self._sb, "tool_current", "") or getattr(self._sb, "tool", "") or ""
        substrate = getattr(self._sb, "substrate_current", "") or ""
        mount = getattr(self._sb, "mount_current", "") or ""

        return str(tool), str(substrate), str(mount)

    def _update_setup_from_scene(self) -> None:
        tool, substrate, mount = self._get_scene_currents()

        if not tool and self._rb is not None:
            tool = getattr(self._rb, "current_tool", "") or getattr(self._rb, "tool", "")

        self._set_setup_labels(tool or "-", substrate or "-", mount or "-")

    def _set_setup_labels(self, tool: str, substrate: str, mount: str) -> None:
        try:
            self.lblTool.setText(f"Tool: {tool}")
            self.lblSubstrate.setText(f"Substrate: {substrate}")
            self.lblMount.setText(f"Mount: {mount}")
        except RuntimeError:
            pass

    def _evaluate_scene_match(self) -> None:
        if not self._recipe_model:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        _, cur_sub, cur_mount = self._get_scene_currents()

        cur_sub_norm = self._norm_mesh_name(cur_sub)
        cur_mount_norm = self._norm_mesh_name(cur_mount)

        rec_sub_norm = self._norm_mesh_name(self._recipe_model.substrate or "")
        rec_mount_norm = self._norm_mesh_name(self._recipe_model.substrate_mount or "")

        sub_ok = bool(rec_sub_norm) and (cur_sub_norm == rec_sub_norm)
        mount_ok = bool(rec_mount_norm) and (cur_mount_norm == rec_mount_norm)

        self.set_substrate_ok(sub_ok)
        self.set_mount_ok(mount_ok)
