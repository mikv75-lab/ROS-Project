# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations

import os
import math
import logging
import datetime
from pathlib import Path
from typing import Optional, List, Dict, Any

import yaml

from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout,
    QGroupBox, QFileDialog, QMessageBox, QTextEdit, QSizePolicy
)

from geometry_msgs.msg import PoseStamped, PoseArray
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
    UI-Tab für den Prozessablauf:
      - Robot-Init/Home prüfen
      - Rezept/Run laden
      - Prozess starten/stoppen
      - Status/Log anzeigen
      - Trajektorie-Infos berechnen und optional als YAML speichern
    """

    def __init__(
        self,
        *,
        ctx,
        bridge,
        plc=None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self.plc = plc

        # Stellt sicher, dass die Bridge "ready" ist (Verbindungen/States/Bridges vorhanden).
        self.bridge.ensure_connected()

        # -------------------------------------------------------------
        # States: reine Datenquellen ohne Qt-Signale (werden abgefragt)
        # -------------------------------------------------------------
        self._scene_state = self.bridge.scene
        self._poses_state = self.bridge.poses
        self._robot_state = self.bridge.robot

        # -------------------------------------------------------------
        # Bridges: Qt-Signal-Wrapper (Events kommen über .signals)
        # -------------------------------------------------------------
        self._sb = self.bridge.scene_bridge
        self._pb = self.bridge.poses_bridge
        self._rb = self.bridge.robot_bridge
        self._spray = self.bridge.spray_path_bridge
        self._moveit = self.bridge.moveitpy_bridge

        # Robot signals: zentrale Signalquelle für Statusupdates
        self._sig = self._rb.signals

        # -------- interner Zustand --------
        self._robot_initialized: bool = False
        self._robot_at_home: bool = False

        self._recipe_model: Optional[Recipe] = None
        self._recipe_name: str | None = None

        # Setup/Startbedingungen (werden aus Scene + Recipe verglichen)
        self._tool_ok: bool = True
        self._substrate_ok: bool = False
        self._mount_ok: bool = False

        # Prozessverwaltung
        self._process_active: bool = False
        self._process_thread: Optional[ProcessThread] = None
        self._robot_init_thread: Optional[RobotInitThread] = None

        # Merkt sich, ob der letzte Prozess success/error geliefert hat (gegen doppelte Handler)
        self._process_outcome: Optional[str] = None

        # Cache für InfoBox-Inhalte
        self._info_values: Dict[str, Any] = {}

        # Guard, damit _update_start_conditions nicht rekursiv läuft
        self._in_update_start_conditions: bool = False

        # ==================================================================
        # View / Layout
        # ==================================================================
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        left = QWidget(self)
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(8, 8, 8, 8)
        left_layout.setSpacing(8)

        # ---------- TOP ROW ----------
        top_row = QHBoxLayout()
        top_row.setContentsMargins(0, 0, 0, 0)
        top_row.setSpacing(8)
        left_layout.addLayout(top_row)

        # --- Process Control: Buttons für Init/Load/Start/Stop ---
        self.grpProcess = QGroupBox("Process Control", left)
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

        # --- Startbedingungen: Textfeld, warum Start möglich/nicht möglich ---
        self.grpStatus = QGroupBox("Startbedingungen", left)
        vstat = QVBoxLayout(self.grpStatus)
        vstat.setContentsMargins(8, 8, 8, 8)
        vstat.setSpacing(4)

        self.lblStatus = QLabel("-", self.grpStatus)
        self.lblStatus.setWordWrap(True)
        vstat.addWidget(self.lblStatus)
        top_row.addWidget(self.grpStatus, 1)

        # --- Setup: Tool/Substrate/Mount Anzeige aus der Scene ---
        self.grpSetup = QGroupBox("Setup", left)
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

        # --- Robot Status Box: live Status aus RobotBridge (Connection/Mode/TCP/...) ---
        self.robotStatusBox = RobotStatusInfoBox(left, title="Robot Status")
        top_row.addWidget(self.robotStatusBox, 2)

        # Size Policies: GroupBoxes sollen mitwachsen, RobotStatus eher fixed in der Höhe
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
        # Zeigt Key/Value Infos (z.B. Rezeptinfos, Trajektorienmetriken)
        self.infoBox = InfoGroupBox(left)
        sp_info = self.infoBox.sizePolicy()
        sp_info.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_info.setVerticalPolicy(QSizePolicy.Policy.Fixed)
        self.infoBox.setSizePolicy(sp_info)
        left_layout.addWidget(self.infoBox)

        # ---------- PROCESS STATUS / LOG ----------
        # Laufender Schritt + Logausgaben der StateMachines
        self.grpProcessInfo = QGroupBox("Process Status / Log", left)
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

        left_layout.addWidget(self.grpProcessInfo, 1)

        # ---------- RECIPE GROUP ----------
        # Links: Rezept Summary, Rechts: (compiled poses) Block
        self.grpRecipe = QGroupBox("Recipe", left)
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

        left_layout.addWidget(self.grpRecipe, 1)

        root.addWidget(left)

        # -------------------------------------------------------------
        # Button: Run laden (zusätzlich zu Rezept laden)
        # -------------------------------------------------------------
        self.btnLoadRunRecipe = QPushButton("Run laden", self.grpProcess)
        proc_layout = self.grpProcess.layout()
        proc_layout.addWidget(self.btnLoadRunRecipe)

        # ==================================================================
        # Threads
        # ==================================================================
        # RobotInitThread führt Init + Home-Anfahren/Check aus (Timeouts + Toleranz)
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
        # Buttons
        # ==================================================================
        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnLoadRecipe.clicked.connect(self._on_load_recipe_clicked)
        self.btnLoadRunRecipe.clicked.connect(self._on_load_run_recipe_clicked)
        self.btnStart.clicked.connect(self._on_start_clicked)
        self.btnStop.clicked.connect(self._on_stop_clicked)

        # Wiring: Scene/Robot/Poses Events an UI-Funktionen koppeln
        self._wire_scene_bridge()
        self._wire_robot_status()
        self._wire_poses_bridge()

        # InfoBox initial leeren
        self._info_values.clear()
        self.infoBox.set_values(None)

        # Timer: prüft Startbedingungen periodisch, solange kein Prozess läuft
        self._condTimer = QtCore.QTimer(self)
        self._condTimer.setInterval(500)
        self._condTimer.timeout.connect(self._update_start_conditions)

        self._update_start_conditions()
        self._update_timer_state()

    # =====================================================================
    # Hilfsfunktionen InfoBox / Log
    # =====================================================================

    def _info_set_all(self, values: Optional[Dict[str, Any]]) -> None:
        # Setzt die InfoBox komplett neu (überschreibt Cache)
        self._info_values = dict(values or {})
        self.infoBox.set_values(dict(self._info_values) if self._info_values else None)

    def _info_update(self, key: str, value: Any | None) -> None:
        # Updatet einen Key; None entfernt den Key aus der Anzeige
        if value is None:
            self._info_values.pop(key, None)
        else:
            self._info_values[key] = value
        self.infoBox.set_values(dict(self._info_values) if self._info_values else None)

    def _append_process_log(self, text: str) -> None:
        # Schreibt eine Zeile ins Log-Fenster
        if text:
            self.txtProcessLog.append(text)

    @QtCore.pyqtSlot(str)
    def _on_process_log_message(self, msg: str) -> None:
        # Slot für Log-Signale aus ProcessThread/StateMachines
        self._append_process_log(msg)

    # =====================================================================
    # Delayed SprayPath publish
    # =====================================================================

    def _publish_recipe_to_spraypath_delayed(self, recipe: Recipe, *, delay_ms: int = 1000) -> None:
        # Baut MarkerArray aus dem Rezept und published es zeitverzögert in die SprayPath-Anzeige
        def _do_publish():
            ma: MarkerArray = recipe_markers.build_marker_array_from_recipe(
                recipe,
                sides=None,
                frame_id="scene",
            )
            self.bridge.set_spraypath(ma)

        QtCore.QTimer.singleShot(delay_ms, _do_publish)

    # =====================================================================
    # Trajektorien-Metriken / Score
    # =====================================================================

    @staticmethod
    def _compute_trajectory_metrics_from_ps_list(poses: List[PoseStamped]) -> Dict[str, Any]:
        # Berechnet einfache Metriken: Punktezahl, Gesamtlänge, Bounding-Box (min/max)
        metrics: Dict[str, Any] = {
            "traj_points": 0,
            "traj_length_mm": 0.0,
            "traj_min_x": None,
            "traj_min_y": None,
            "traj_min_z": None,
            "traj_max_x": None,
            "traj_max_y": None,
            "traj_max_z": None,
        }
        if not poses:
            return metrics

        x0 = float(poses[0].pose.position.x)
        y0 = float(poses[0].pose.position.y)
        z0 = float(poses[0].pose.position.z)

        min_x = max_x = x0
        min_y = max_y = y0
        min_z = max_z = z0

        total_len = 0.0
        last_x, last_y, last_z = x0, y0, z0

        for i, p in enumerate(poses):
            x = float(p.pose.position.x)
            y = float(p.pose.position.y)
            z = float(p.pose.position.z)

            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)
            min_z = min(min_z, z)
            max_z = max(max_z, z)

            if i > 0:
                dx = x - last_x
                dy = y - last_y
                dz = z - last_z
                total_len += math.sqrt(dx * dx + dy * dy + dz * dz)

            last_x, last_y, last_z = x, y, z

        metrics["traj_points"] = len(poses)
        metrics["traj_length_mm"] = total_len
        metrics["traj_min_x"] = min_x
        metrics["traj_min_y"] = min_y
        metrics["traj_min_z"] = min_z
        metrics["traj_max_x"] = max_x
        metrics["traj_max_y"] = max_y
        metrics["traj_max_z"] = max_z
        return metrics

    @staticmethod
    def _compute_trajectory_score(metrics: Dict[str, Any]) -> Optional[float]:
        # Sehr einfacher Score: kürzere Wege ergeben höhere Scores (auf 0..100 geklemmt)
        total_mm = float(metrics["traj_length_mm"])
        points = int(metrics["traj_points"])
        if points < 2 or total_mm <= 0.0:
            return None
        raw = 100000.0 / (total_mm + 1.0)
        return max(0.0, min(100.0, raw))

    def _update_info_with_trajectory_metrics(self, metrics: Dict[str, Any], score: Optional[float]) -> None:
        # Schreibt Metriken in die InfoBox
        pts = int(metrics["traj_points"])
        total_mm = float(metrics["traj_length_mm"])

        self._info_update("traj_points", pts if pts > 0 else None)
        self._info_update("traj_length_mm", f"{total_mm:.1f} mm" if total_mm > 0.0 else None)
        self._info_update("traj_score", f"{score:.1f} / 100" if score is not None else None)

        for axis in ("x", "y", "z"):
            mn = metrics[f"traj_min_{axis}"]
            mx = metrics[f"traj_max_{axis}"]
            self._info_update(f"traj_{axis}_range_mm", f"{mn:.1f} .. {mx:.1f} mm")

    def _log_trajectory_metrics(self, metrics: Dict[str, Any], score: Optional[float]) -> None:
        # Schreibt Metriken zusätzlich ins Prozess-Log
        pts = int(metrics["traj_points"])
        total_mm = float(metrics["traj_length_mm"])
        self._append_process_log(f"Trajektorie: {pts} Posen, Gesamtlänge ≈ {total_mm:.1f} mm.")
        if score is not None:
            self._append_process_log(f"Einfache Score-Bewertung: {score:.1f} / 100 (kürzer ⇒ besser).")
        self._append_process_log(
            "Trajektorie Bounding-Box:"
            f" X=[{metrics['traj_min_x']:.1f}, {metrics['traj_max_x']:.1f}] mm,"
            f" Y=[{metrics['traj_min_y']:.1f}, {metrics['traj_max_y']:.1f}] mm,"
            f" Z=[{metrics['traj_min_z']:.1f}, {metrics['traj_max_z']:.1f}] mm."
        )

    # =====================================================================
    # Run-Logging
    # =====================================================================

    def _get_runs_dir(self) -> Path:
        # Legt neben dem recipe_dir einen "runs/" Ordner an (falls nicht vorhanden)
        recipe_dir = self.ctx.paths.recipe_dir
        base_dir = Path(recipe_dir).resolve().parent
        runs_dir = base_dir / "runs"
        runs_dir.mkdir(parents=True, exist_ok=True)
        return runs_dir

    def _get_run_filename_for_recipe(self) -> Path:
        # Dateiname wird aus recipe_id oder recipe_name abgeleitet und "sicher" gemacht
        rec_id = (self._recipe_model.id or "").strip() if self._recipe_model else ""
        name = rec_id or (self._recipe_name or "unnamed_recipe")
        safe_name = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in name)
        return self._get_runs_dir() / f"{safe_name}.yaml"

    def _save_trajectory_yaml(self, poses: List[PoseStamped], metrics: Dict[str, Any], score: Optional[float]) -> None:
        # Speichert Posen + Metriken als YAML (für späteres Run-Playback/Analyse)
        frame = poses[0].header.frame_id or "scene"
        data: Dict[str, Any] = {
            "version": 1,
            "recipe_id": self._recipe_model.id if self._recipe_model else self._recipe_name,
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "frame_id": frame,
            "metrics": dict(metrics),
            "poses": [
                {
                    "x": float(p.pose.position.x),
                    "y": float(p.pose.position.y),
                    "z": float(p.pose.position.z),
                    "qx": float(p.pose.orientation.x),
                    "qy": float(p.pose.orientation.y),
                    "qz": float(p.pose.orientation.z),
                    "qw": float(p.pose.orientation.w),
                }
                for p in poses
            ],
        }
        if score is not None:
            data["metrics"]["score"] = float(score)

        out_path = self._get_run_filename_for_recipe()
        with out_path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True, default_flow_style=False)

        self._append_process_log(f"Trajektorie-Log gespeichert: {out_path.name}")
        QMessageBox.information(self, "Run gespeichert", f"Trajektorie gespeichert:\n{out_path.name}")

    # =====================================================================
    # Robot-Status Wiring (direkt)
    # =====================================================================

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js):
        # JointState -> nur Positionsliste an die RobotStatusBox weitergeben
        self.robotStatusBox.set_joints(list(js.position or []) if js is not None else None)

    def _wire_robot_status(self) -> None:
        # Verbindet RobotBridge-Signale mit der RobotStatusBox und lokalen Slots
        sig = self._sig
        sb = self.robotStatusBox

        sig.connectionChanged.connect(sb.set_connection)
        sig.modeChanged.connect(sb.set_mode)
        sig.initializedChanged.connect(sb.set_initialized)
        sig.initializedChanged.connect(self.set_robot_initialized)
        sig.movingChanged.connect(sb.set_moving)
        sig.powerChanged.connect(sb.set_power)
        sig.servoEnabledChanged.connect(sb.set_servo_enabled)
        sig.estopChanged.connect(sb.set_estop)
        sig.errorsChanged.connect(sb.set_errors)
        sig.tcpPoseChanged.connect(sb.set_tcp_from_ps)
        sig.tcpPoseChanged.connect(self._on_tcp_pose_changed)
        sig.jointsChanged.connect(self._on_joints)

    # =====================================================================
    # Poses-Bridge: Home-Pose beobachten
    # =====================================================================

    def _wire_poses_bridge(self) -> None:
        # Wenn sich die HomePose ändert, wird "robot_at_home" neu berechnet
        self._pb.signals.homePoseChanged.connect(self._on_home_pose_changed)

    @QtCore.pyqtSlot(object)
    def _on_tcp_pose_changed(self, _msg: object) -> None:
        # TCP-Pose geändert -> Home-Check neu berechnen
        self._recompute_robot_at_home()

    @QtCore.pyqtSlot(object)
    def _on_home_pose_changed(self, _msg: object) -> None:
        # Home-Pose geändert -> Home-Check neu berechnen
        self._recompute_robot_at_home()

    # =====================================================================
    # Pose-basierter Home-Check (nur via State)
    # =====================================================================

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        # Aktuelle TCP-Pose aus RobotState
        return self._robot_state.tcp_pose()

    def _get_home_pose(self) -> Optional[PoseStamped]:
        # Home-Pose aus PosesState
        return self._poses_state.home()

    @staticmethod
    def _poses_close(a: PoseStamped, b: PoseStamped, pos_tol_mm: float = 1.0) -> bool:
        # Position-only Vergleich (Distanz) gegen Toleranz
        dx = float(a.pose.position.x) - float(b.pose.position.x)
        dy = float(a.pose.position.y) - float(b.pose.position.y)
        dz = float(a.pose.position.z) - float(b.pose.position.z)
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist <= pos_tol_mm

    def _recompute_robot_at_home(self) -> None:
        # Setzt robot_at_home, wenn TCP und Home vorhanden sind und nahe genug sind
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        self.set_robot_at_home(bool(cur and home and self._poses_close(cur, home, pos_tol_mm=1.0)))

    # =====================================================================
    # ProcessThread
    # =====================================================================

    def _cleanup_process_thread(self) -> None:
        # Beendet/entfernt einen bestehenden ProcessThread sauber
        thr = self._process_thread
        self._process_thread = None
        if thr is None:
            return
        if thr.isRunning():
            thr.request_stop()
            thr.wait(2000)
        thr.deleteLater()

    def _setup_process_thread_for_recipe(self, recipe: Recipe) -> None:
        # Erstellt einen ProcessThread für das "normale" Rezept (MoveIt/Motion Setup+Run)
        self._cleanup_process_thread()
        self._process_outcome = None

        thr = ProcessThread(recipe=recipe, bridge=self.bridge)
        thr.notifyFinished.connect(lambda result_obj, self=self: self._on_process_finished_success(result_obj))
        thr.notifyError.connect(lambda msg, self=self: self._on_process_finished_error(msg))
        thr.finished.connect(self._on_process_thread_finished)
        thr.stateChanged.connect(lambda s, self=self: self._on_process_state_changed(s))
        thr.logMessage.connect(lambda m, self=self: self._on_process_log_message(m))

        self._process_thread = thr
        self.txtProcessLog.clear()

    def _setup_process_thread_for_run(self, run_yaml_path: str) -> None:
        # Erstellt einen ProcessThread für Run-Playback (Servo)
        self._cleanup_process_thread()
        self._process_outcome = None

        thr = ProcessThread.for_run(run_yaml_path=run_yaml_path, bridge=self.bridge)
        thr.notifyFinished.connect(lambda result_obj, self=self: self._on_process_finished_success(result_obj))
        thr.notifyError.connect(lambda msg, self=self: self._on_process_finished_error(msg))
        thr.finished.connect(self._on_process_thread_finished)
        thr.stateChanged.connect(lambda s, self=self: self._on_process_state_changed(s))
        thr.logMessage.connect(lambda m, self=self: self._on_process_log_message(m))

        self._process_thread = thr
        self.txtProcessLog.clear()
        self._append_process_log("Run-Recipe geladen (Servo-Playback).")

    # =====================================================================
    # Buttons
    # =====================================================================

    def _on_init_clicked(self) -> None:
        # Startet RobotInitThread (Init + Home)
        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        self.btnInit.setEnabled(False)
        self._info_update("process", "Robot-Init läuft (Init + Home via Pose-Vergleich)...")
        self._robot_init_thread.startSignal.emit()

    def _on_load_recipe_clicked(self) -> None:
        # Lädt ein Rezept-YAML und wendet es im UI an
        start_dir = self.ctx.paths.recipe_dir

        fname, _ = QFileDialog.getOpenFileName(
            self,
            "Rezept laden (Process)",
            start_dir,
            "YAML (*.yaml *.yml)"
        )
        if not fname:
            return

        model = Recipe.load_yaml(fname)
        _LOG.info("ProcessTab: Rezept '%s' geladen.", fname)
        QtCore.QTimer.singleShot(0, lambda m=model, f=fname, self=self: self._apply_loaded_recipe(m, f))

    def _on_load_run_recipe_clicked(self) -> None:
        # Lädt eine Run-Datei aus runs/ und bereitet Servo-Playback vor
        runs_dir = self._get_runs_dir()
        fname, _ = QFileDialog.getOpenFileName(self, "Run-Recipe laden", str(runs_dir), "YAML (*.yaml *.yml)")
        if not fname:
            return

        base = os.path.basename(fname)
        name_no_ext, _ = os.path.splitext(base)
        if name_no_ext:
            self.set_recipe_name(name_no_ext)

        _LOG.info("ProcessTab: Run-Recipe-Datei '%s' ausgewählt.", fname)
        self._setup_process_thread_for_run(fname)
        self.lblProcessState.setText("Kein Prozess aktiv (Run-Recipe / Servo).")
        self._append_process_log(f"Run-Recipe geladen: {base}")

    def _apply_loaded_recipe(self, model: Recipe, fname: str) -> None:
        # Setzt das geladene Rezept als aktuellen Zustand und published Marker
        self._recipe_model = model
        name = (model.id or "").strip() or os.path.basename(fname)
        self.set_recipe_name(name)

        self._update_recipe_info_text()
        self._setup_process_thread_for_recipe(model)

        # Marker verzögert publishen (damit Viewer/Scene bereit ist)
        self._publish_recipe_to_spraypath_delayed(model, delay_ms=1000)

        # Setup/Scene-Abgleich aktualisieren
        self._update_setup_from_scene()
        self._evaluate_scene_match()

        self.lblProcessState.setText("Kein Prozess aktiv.")
        _LOG.info("ProcessTab: Rezept angewendet (id=%r).", model.id)

    def _on_start_clicked(self) -> None:
        # Startet den ProcessThread (wenn vorhanden)
        if self._process_thread is None:
            QMessageBox.warning(self, "Kein Prozess", "Es ist kein Rezept/Run geladen.")
            return

        self._process_outcome = None

        # Falls Rezept gewechselt wurde: Thread auf aktuelles Rezept setzen
        if self._recipe_model is not None and getattr(self._process_thread, "recipe", None) is not self._recipe_model:
            self._process_thread.set_recipe(self._recipe_model)

        self.set_process_active(True)
        self._append_process_log("=== Prozess gestartet ===")
        self._process_thread.startSignal.emit()

    def _on_stop_clicked(self) -> None:
        # Stop anfordern (ProcessThread + RobotInitThread)
        self._append_process_log("=== Stop angefordert ===")
        thr = self._process_thread
        if thr is not None:
            thr.request_stop()
            if thr.isRunning():
                thr.stopSignal.emit()

        if self._robot_init_thread.isRunning():
            self._robot_init_thread.stopSignal.emit()

    # ---------------------------------------------------------------------
    # Callbacks Robot-Init
    # ---------------------------------------------------------------------

    def _on_robot_init_finished(self) -> None:
        # RobotInit erfolgreich -> Flags setzen + UI aktualisieren
        self.set_robot_initialized(True)
        self._recompute_robot_at_home()

        self.btnInit.setEnabled(True)
        self._info_update("robot_init", "Roboter initialisiert und Home-Pose erreicht (TCP≈Home).")
        if self._recipe_name:
            self._info_update("recipe", self._recipe_name)

    def _on_robot_init_error(self, msg: str) -> None:
        # RobotInit fehlgeschlagen -> Flags zurücksetzen + MessageBox
        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        self.btnInit.setEnabled(True)
        QMessageBox.warning(self, "Robot Init fehlgeschlagen", msg or "Unbekannter Fehler.")

    # ---------------------------------------------------------------------
    # Callbacks vom ProcessThread
    # ---------------------------------------------------------------------

    def _on_process_finished_success(self, result_obj: object) -> None:
        # Erfolgspfad (wird ggf. mehrfach getriggert -> Outcome-Guard)
        if self._process_outcome == "error":
            return
        if self._process_outcome is None:
            self._process_outcome = "success"

        poses: List[PoseStamped] = []
        planned_traj: Any = None
        executed_traj: Any = None

        # Unterstützt dict-Results (Setup/Run) und Listen (Legacy)
        if isinstance(result_obj, dict):
            raw_poses = result_obj.get("poses") or result_obj.get("tcp_poses") or []
            poses = [p for p in raw_poses if isinstance(p, PoseStamped)]
            planned_traj = result_obj.get("planned_traj")
            executed_traj = result_obj.get("executed_traj")
        elif isinstance(result_obj, list):
            poses = [p for p in result_obj if isinstance(p, PoseStamped)]

        if planned_traj is not None:
            self._append_process_log("Geplante Trajektorie empfangen.")
        if executed_traj is not None:
            self._append_process_log("Ausgeführte Trajektorie empfangen.")

        # Falls TCP-Posen vorhanden: ExecutedPath als PoseArray publishen
        if poses:
            pa = PoseArray()
            pa.header.frame_id = poses[0].header.frame_id or "scene"
            pa.poses = [p.pose for p in poses]
            self.bridge.set_executed_path(pa)

        # Metriken + Score berechnen und anzeigen
        metrics = self._compute_trajectory_metrics_from_ps_list(poses)
        score = self._compute_trajectory_score(metrics)
        self._update_info_with_trajectory_metrics(metrics, score)
        self._log_trajectory_metrics(metrics, score)

        # Wenn Posen da sind, wird zusätzlich ein YAML-Log erzeugt
        if poses and (self._recipe_model or self._recipe_name):
            self._save_trajectory_yaml(poses, metrics, score)

        self.set_process_active(False)
        self._info_update("process", "Prozess erfolgreich abgeschlossen.")
        self._append_process_log("=== Prozess erfolgreich abgeschlossen ===")
        self.lblProcessState.setText("Prozess erfolgreich abgeschlossen.")

    @QtCore.pyqtSlot(str)
    def _on_process_finished_error(self, msg: str) -> None:
        # Fehlerpfad (Outcome-Guard gegen doppelte Calls)
        if self._process_outcome == "error":
            return
        self._process_outcome = "error"

        self.set_process_active(False)
        self._info_update("process", f"Fehler: {msg}")
        self._append_process_log(f"=== Prozessfehler: {msg} ===")
        self.lblProcessState.setText(f"Fehler: {msg}")
        QMessageBox.critical(self, "Prozessfehler", f"Prozess abgebrochen:\n{msg}")

    def _on_process_thread_finished(self) -> None:
        # Qt-Thread finished -> UI Flag zurücksetzen
        self.set_process_active(False)

    @QtCore.pyqtSlot(str)
    def _on_process_state_changed(self, state: str) -> None:
        # Übersetzt interne State-Namen in UI-Text
        pretty = {
            "MOVE_PREDISPENSE": "Predispense-Position anfahren",
            "WAIT_PREDISPENSE": "Predispense-Zeit warten",
            "MOVE_RECIPE": "Rezeptpfad fahren",
            "WAIT_POSTDISPENSE": "Postdispense-Zeit warten",
            "MOVE_RETREAT": "Retreat-Position anfahren",
            "MOVE_HOME": "Home-Position anfahren",
            "ERROR": "Fehler – Prozess abgebrochen",
            "FINISHED": "Prozess abgeschlossen",
        }.get(state, state)

        self.lblProcessState.setText(f"Aktueller Schritt: {pretty}")
        prefix = "Prozess läuft.\n" if self._process_active else ""
        self.lblStatus.setText(f"{prefix}Aktueller Schritt: {pretty}")

        self._info_update("process_state", pretty)
        if self._recipe_name:
            self._info_update("recipe", self._recipe_name)

    # =====================================================================
    # Public API
    # =====================================================================

    def set_recipe_name(self, name: str | None) -> None:
        # Speichert den angezeigten Rezeptnamen und aktualisiert Startbedingungen
        txt = (name or "").strip()
        self._recipe_name = txt if txt else None
        self._update_start_conditions()

    def _update_recipe_info_text(self) -> None:
        # Füllt die Recipe-Textfelder und die InfoBox aus dem RecipeModel
        if not self._recipe_model:
            self.txtRecipeSummary.setPlainText("")
            self.txtRecipePoses.setPlainText("")
            self._info_set_all(None)
            return

        full_text = str(self._recipe_model)
        lines: List[str] = full_text.splitlines()

        # Trennt Summary und "compiled poses" Block, falls vorhanden
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

        base_info = self._recipe_model.info or {}
        self._info_set_all(base_info)

    def set_robot_initialized(self, flag: bool) -> None:
        # Setzt Flag und aktualisiert Startbedingungen
        self._robot_initialized = bool(flag)
        self._update_start_conditions()

    def set_robot_at_home(self, flag: bool) -> None:
        # Setzt Flag und aktualisiert Startbedingungen
        self._robot_at_home = bool(flag)
        self._update_start_conditions()

    def set_tool_ok(self, flag: bool) -> None:
        # Setzt Flag und aktualisiert Startbedingungen
        self._tool_ok = bool(flag)
        self._update_start_conditions()

    def set_substrate_ok(self, flag: bool) -> None:
        # Setzt Flag und aktualisiert Startbedingungen
        self._substrate_ok = bool(flag)
        self._update_start_conditions()

    def set_mount_ok(self, flag: bool) -> None:
        # Setzt Flag und aktualisiert Startbedingungen
        self._mount_ok = bool(flag)
        self._update_start_conditions()

    def set_process_active(self, active: bool) -> None:
        # Aktiv-Flag + Timersteuerung (Startbedingungen nur wenn nicht aktiv)
        self._process_active = bool(active)
        self._update_timer_state()

        if self._process_active:
            self.lblStatus.setText(
                "Prozess läuft.\nStartbedingungen werden während der Ausführung nicht geprüft."
            )
        else:
            self._update_start_conditions()

    # =====================================================================
    # Startbedingungen + Timer
    # =====================================================================

    @staticmethod
    def _norm_mesh_name(name: str) -> str:
        # Normalisiert Mesh-Namen: basename + ohne ".stl"
        n = os.path.basename(name or "").strip()
        if n.lower().endswith(".stl"):
            n = n[:-4]
        return n

    def _update_start_conditions(self) -> None:
        # Prüft, ob alle Bedingungen erfüllt sind und setzt UI entsprechend
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
                missing.append("Kein Rezept/Run geladen")
            if not self._substrate_ok:
                missing.append("Substrate-Konfiguration stimmt nicht mit dem Rezept überein")
            if not self._mount_ok:
                missing.append("Mount-Konfiguration stimmt nicht mit dem Rezept überein")

            can_start = (len(missing) == 0)
            self.btnStart.setEnabled(can_start and not self._process_active)

            # Während Prozess aktiv: Status nicht ständig überschreiben
            if self._process_active:
                return

            if can_start:
                txt = "Startbereit.\nAlle Startbedingungen sind erfüllt."
            else:
                msg_lines = ["Start nicht möglich.", "Fehlende Bedingungen:"]
                msg_lines += [f"- {m}" for m in missing]
                txt = "\n".join(msg_lines)

            self.lblStatus.setText(txt)
        finally:
            self._in_update_start_conditions = False

    def _update_timer_state(self) -> None:
        # Timer läuft nur, wenn kein Prozess aktiv ist
        if self._process_active:
            if self._condTimer.isActive():
                self._condTimer.stop()
        else:
            if not self._condTimer.isActive():
                self._condTimer.start()

    # =====================================================================
    # Scene wiring + Setup Anzeige (über SceneState)
    # =====================================================================

    def _wire_scene_bridge(self) -> None:
        # Wenn Scene-Objekte wechseln, werden Setup-Labels und Match neu berechnet
        sig = self._sb.signals
        sig.substrateCurrentChanged.connect(self._on_scene_changed)
        sig.mountCurrentChanged.connect(self._on_scene_changed)
        sig.cageCurrentChanged.connect(self._on_scene_changed)  # optional
        if hasattr(sig, "toolCurrentChanged"):
            sig.toolCurrentChanged.connect(self._on_scene_changed)

        self._update_setup_from_scene()
        self._evaluate_scene_match()

    def _on_scene_changed(self, *_args) -> None:
        # Scene-Update -> Setup und Match aktualisieren
        self._update_setup_from_scene()
        self._evaluate_scene_match()

    def _get_scene_currents(self) -> tuple[str, str, str]:
        # Liest aktuelle Scene-Auswahl (Tool ist hier noch Platzhalter)
        tool = ""  # aktuell nicht im SceneState vorhanden
        substrate = self._scene_state.substrate_current()
        mount = self._scene_state.mount_current()
        return tool, substrate, mount

    def _update_setup_from_scene(self) -> None:
        # Aktualisiert die Setup-Labels aus den aktuellen Scene-Werten
        tool, substrate, mount = self._get_scene_currents()
        self._set_setup_labels(tool or "-", substrate or "-", mount or "-")

    def _set_setup_labels(self, tool: str, substrate: str, mount: str) -> None:
        # Schreibt Text in die 3 Setup-Labels
        self.lblTool.setText(f"Tool: {tool}")
        self.lblSubstrate.setText(f"Substrate: {substrate}")
        self.lblMount.setText(f"Mount: {mount}")

    def _evaluate_scene_match(self) -> None:
        # Vergleicht aktuelles Substrate/Mount aus der Scene mit dem Rezept
        if not self._recipe_model:
            self.set_substrate_ok(False)
            self.set_mount_ok(False)
            return

        _, cur_sub, cur_mount = self._get_scene_currents()

        cur_sub_norm = self._norm_mesh_name(cur_sub)
        cur_mount_norm = self._norm_mesh_name(cur_mount)

        rec_sub_norm = self._norm_mesh_name(self._recipe_model.substrate or "")
        rec_mount_norm = self._norm_mesh_name(self._recipe_model.substrate_mount or "")

        self.set_substrate_ok(bool(rec_sub_norm) and (cur_sub_norm == rec_sub_norm))
        # Hinweis: hier wird Mount aktuell gegen sich selbst verglichen (cur_mount_norm == rec_mount_norm wäre der Match)
        self.set_mount_ok(bool(rec_mount_norm) and (cur_mount_norm == rec_mount_norm))
