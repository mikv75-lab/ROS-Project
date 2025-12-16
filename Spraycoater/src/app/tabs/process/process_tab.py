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
    Process-Tab (nur Code, keine .ui):

      VBOX(
        HBOX(
          [GroupBox] Process Control,
          [GroupBox] Startbedingungen,
          [GroupBox] Setup (Tool / Substrate / Mount),
          [RobotStatusInfoBox] Robot Status
        ),
        [InfoGroupBox]
        [GroupBox] Process Status / Log
           ├─ QLabel    (aktueller State)
           └─ QTextEdit (Log)
        [GroupBox] Recipe
           ├─ QTextEdit  (Summary)
           └─ QTextEdit  (Poses)
      )

    Zusätzlich:
      - "Rezept laden"  → ProcessThread im MODE_RECIPE (=setup),
                          Worker = ProcessSetupStatemachine (MoveIt/Motion)

      - "Run laden"     → ProcessThread im MODE_RUN (=servo),
                          Worker = ProcessRunStatemachine
                          (Servo-Run eines Run-YAML, z.B. aus data/runs)
    """

    def __init__(
        self,
        *,
        ctx,
        bridge,
        plc=None,                       # 👈 neu
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self.plc = plc

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

        # Ergebnis eines Prozesslaufs:
        #   None      -> noch nichts
        #   "success" -> notifyFinished verarbeitet
        #   "error"   -> notifyError verarbeitet (hat Vorrang)
        self._process_outcome: Optional[str] = None

        # Info-Box Cache
        self._info_values: Dict[str, Any] = {}

        # Reentrancy-Guard
        self._in_update_start_conditions: bool = False

        # ==================================================================
        # View / Layout: nur Process-UI (kein Foxglove/WebView)
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

        # --- Process Control ---
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

        # --- Startbedingungen ---
        self.grpStatus = QGroupBox("Startbedingungen", left)
        vstat = QVBoxLayout(self.grpStatus)
        vstat.setContentsMargins(8, 8, 8, 8)
        vstat.setSpacing(4)

        self.lblStatus = QLabel("-", self.grpStatus)
        self.lblStatus.setWordWrap(True)
        vstat.addWidget(self.lblStatus)
        top_row.addWidget(self.grpStatus, 1)

        # --- Setup ---
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

        # --- Robot Status ---
        self.robotStatusBox = RobotStatusInfoBox(left, title="Robot Status")
        top_row.addWidget(self.robotStatusBox, 2)

        # Size Policies für Top-Row
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
        self.infoBox = InfoGroupBox(left)
        sp_info = self.infoBox.sizePolicy()
        sp_info.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_info.setVerticalPolicy(QSizePolicy.Policy.Fixed)
        self.infoBox.setSizePolicy(sp_info)
        left_layout.addWidget(self.infoBox)

        # ---------- PROCESS STATUS / LOG ----------
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

        # Linke Seite in Root-Layout einhängen
        root.addWidget(left)

        # -------------------------------------------------------------
        # Neuer Button: Run-Trajektorie / Run-Recipe laden (Servo-Modus)
        # -------------------------------------------------------------
        self.btnLoadRunRecipe = QPushButton("Run laden", self.grpProcess)
        proc_layout = self.grpProcess.layout()
        if proc_layout is not None:
            proc_layout.addWidget(self.btnLoadRunRecipe)

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
        self.btnLoadRunRecipe.clicked.connect(self._on_load_run_recipe_clicked)
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
        self._info_values = dict(values or {})
        try:
            if self._info_values:
                self.infoBox.set_values(dict(self._info_values))
            else:
                self.infoBox.set_values(None)
        except RuntimeError:
            pass

    def _info_update(self, key: str, value: Any | None) -> None:
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
        if not text:
            return
        try:
            self.txtProcessLog.append(text)
        except RuntimeError:
            pass

    @QtCore.pyqtSlot(str)
    def _on_process_log_message(self, msg: str) -> None:
        self._append_process_log(msg)

    # =====================================================================
    # 🆕 Delayed SprayPath publish (Variante 4)
    # =====================================================================

    def _publish_recipe_to_spraypath_delayed(
        self,
        recipe: Recipe,
        *,
        delay_ms: int = 1000,
    ) -> None:
        """
        Publisht den Sollpfad absichtlich verzögert, um Race Conditions zu vermeiden.
        """
        if recipe is None or self.bridge is None:
            return

        def _do_publish():
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
                            "ProcessTab: MarkerArray mit %d Markern an SprayPath gesendet (frame=scene, delayed=%dms)",
                            len(ma.markers),
                            delay_ms,
                        )
                    except Exception:
                        _LOG.exception("ProcessTab: bridge.set_spraypath failed (delayed)")
                else:
                    _LOG.warning(
                        "ProcessTab: delayed publish: build_marker_array_from_recipe lieferte kein/nur leeres MarkerArray"
                    )
            except Exception:
                _LOG.exception(
                    "ProcessTab: Fehler beim Erzeugen/Publizieren des MarkerArray für SprayPath (delayed)"
                )

        QtCore.QTimer.singleShot(delay_ms, _do_publish)

    # =====================================================================
    # Trajektorien-Metriken / Score
    # =====================================================================

    @staticmethod
    def _compute_trajectory_metrics_from_ps_list(
        poses: List[PoseStamped]
    ) -> Dict[str, Any]:
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

        first = poses[0]
        try:
            x0 = float(first.pose.position.x)
            y0 = float(first.pose.position.y)
            z0 = float(first.pose.position.z)
        except Exception:
            x0 = y0 = z0 = 0.0

        min_x = max_x = x0
        min_y = max_y = y0
        min_z = max_z = z0

        total_len = 0.0
        last_x, last_y, last_z = x0, y0, z0

        for i, p in enumerate(poses):
            try:
                x = float(p.pose.position.x)
                y = float(p.pose.position.y)
                z = float(p.pose.position.z)
            except Exception:
                continue

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
                step = math.sqrt(dx * dx + dy * dy + dz * dz)
                total_len += step

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
        try:
            total_mm = float(metrics.get("traj_length_mm", 0.0) or 0.0)
        except Exception:
            return None

        points = int(metrics.get("traj_points", 0) or 0)
        if points < 2 or total_mm <= 0.0:
            return None

        raw = 100000.0 / (total_mm + 1.0)
        if raw < 0.0:
            raw = 0.0
        if raw > 100.0:
            raw = 100.0
        return raw

    def _update_info_with_trajectory_metrics(
        self,
        metrics: Dict[str, Any],
        score: Optional[float],
    ) -> None:
        pts = int(metrics.get("traj_points", 0) or 0)
        total_mm = float(metrics.get("traj_length_mm", 0.0) or 0.0)

        self._info_update("traj_points", pts if pts > 0 else None)
        self._info_update(
            "traj_length_mm",
            f"{total_mm:.1f} mm" if total_mm > 0.0 else None,
        )

        if score is not None:
            self._info_update("traj_score", f"{score:.1f} / 100")
        else:
            self._info_update("traj_score", None)

        for axis in ("x", "y", "z"):
            mn = metrics.get(f"traj_min_{axis}")
            mx = metrics.get(f"traj_max_{axis}")
            key = f"traj_{axis}_range_mm"
            if mn is None or mx is None:
                self._info_update(key, None)
            else:
                self._info_update(key, f"{mn:.1f} .. {mx:.1f} mm")

    def _log_trajectory_metrics(
        self,
        metrics: Dict[str, Any],
        score: Optional[float],
    ) -> None:
        pts = int(metrics.get("traj_points", 0) or 0)
        total_mm = float(metrics.get("traj_length_mm", 0.0) or 0.0)

        if pts <= 0:
            self._append_process_log("Trajektorie: keine Posen übergeben.")
            return

        self._append_process_log(
            f"Trajektorie: {pts} Posen, Gesamtlänge ≈ {total_mm:.1f} mm."
        )

        if score is not None:
            self._append_process_log(
                f"Einfache Score-Bewertung: {score:.1f} / 100 "
                "(kürzerer Weg ⇒ höherer Score)."
            )

        mn_x = metrics.get("traj_min_x")
        mx_x = metrics.get("traj_max_x")
        mn_y = metrics.get("traj_min_y")
        mx_y = metrics.get("traj_max_y")
        mn_z = metrics.get("traj_min_z")
        mx_z = metrics.get("traj_max_z")

        if None not in (mn_x, mx_x, mn_y, mx_y, mn_z, mx_z):
            self._append_process_log(
                "Trajektorie Bounding-Box:"
                f" X=[{mn_x:.1f}, {mx_x:.1f}] mm,"
                f" Y=[{mn_y:.1f}, {mx_y:.1f}] mm,"
                f" Z=[{mn_z:.1f}, {mx_z:.1f}] mm."
            )

    # =====================================================================
    # Trajektorie-Logging (YAML) – ein File pro Rezept in data/runs
    # =====================================================================

    def _get_runs_dir(self) -> Path:
        base_dir: Path | None = None

        try:
            recipe_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", None)
            if recipe_dir:
                base_dir = Path(recipe_dir).resolve().parent
        except Exception:
            base_dir = None

        if base_dir is None:
            base_dir = Path(os.getcwd())

        runs_dir = base_dir / "runs"
        runs_dir.mkdir(parents=True, exist_ok=True)
        return runs_dir

    def _get_run_filename_for_recipe(self) -> Path:
        rec_id = None
        if self._recipe_model is not None:
            rec_id = (self._recipe_model.id or "").strip()

        name = rec_id or (self._recipe_name or "unnamed_recipe")
        safe_name = "".join(
            c if c.isalnum() or c in ("-", "_") else "_"
            for c in name
        )
        runs_dir = self._get_runs_dir()
        return runs_dir / f"{safe_name}.yaml"

    def _save_trajectory_yaml(
        self,
        poses: List[PoseStamped],
        metrics: Dict[str, Any],
        score: Optional[float],
    ) -> None:
        if not poses:
            _LOG.info("ProcessTab: _save_trajectory_yaml: keine Posen -> nichts zu speichern.")
            return

        frame = poses[0].header.frame_id or "scene"

        data: Dict[str, Any] = {
            "version": 1,
            "recipe_id": self._recipe_model.id if self._recipe_model else self._recipe_name,
            "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
            "frame_id": frame,
            "metrics": dict(metrics or {}),
        }

        if score is not None:
            data["metrics"]["score"] = float(score)

        pose_list: List[Dict[str, float]] = []
        for p in poses:
            try:
                pose_list.append(
                    {
                        "x": float(p.pose.position.x),
                        "y": float(p.pose.position.y),
                        "z": float(p.pose.position.z),
                        "qx": float(p.pose.orientation.x),
                        "qy": float(p.pose.orientation.y),
                        "qz": float(p.pose.orientation.z),
                        "qw": float(p.pose.orientation.w),
                    }
                )
            except Exception:
                continue

        data["poses"] = pose_list

        try:
            out_path = self._get_run_filename_for_recipe()
            with out_path.open("w", encoding="utf-8") as f:
                yaml.safe_dump(
                    data,
                    f,
                    sort_keys=False,
                    allow_unicode=True,
                    default_flow_style=False,
                )

            _LOG.info(
                "ProcessTab: Trajektorie-Log mit %d Posen nach '%s' gespeichert (überschrieben, falls vorhanden).",
                len(pose_list),
                str(out_path),
            )
            self._append_process_log(f"Trajektorie-Log gespeichert: {out_path.name}")

            try:
                QMessageBox.information(
                    self,
                    "Run gespeichert",
                    f"Trajektorie für Rezept wurde gespeichert:\n{out_path.name}",
                )
            except RuntimeError:
                pass

        except Exception:
            _LOG.exception("ProcessTab: _save_trajectory_yaml: Fehler beim Schreiben der YAML-Datei.")

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

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        dist_mm = dist
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
        self._cleanup_process_thread()
        self._process_outcome = None

        thr = ProcessThread(recipe=recipe, bridge=self.bridge)

        thr.notifyFinished.connect(
            lambda result_obj, self=self: self._on_process_finished_success(result_obj)
        )
        thr.notifyError.connect(
            lambda msg, self=self: self._on_process_finished_error(msg)
        )
        thr.finished.connect(self._on_process_thread_finished)
        thr.stateChanged.connect(lambda s, self=self: self._on_process_state_changed(s))
        thr.logMessage.connect(lambda m, self=self: self._on_process_log_message(m))

        self._process_thread = thr

        try:
            self.txtProcessLog.clear()
        except RuntimeError:
            pass

    def _setup_process_thread_for_run(self, run_yaml_path: str) -> None:
        self._cleanup_process_thread()
        self._process_outcome = None

        thr = ProcessThread.for_run(
            run_yaml_path=run_yaml_path,
            bridge=self.bridge,
        )

        thr.notifyFinished.connect(
            lambda result_obj, self=self: self._on_process_finished_success(result_obj)
        )
        thr.notifyError.connect(
            lambda msg, self=self: self._on_process_finished_error(msg)
        )
        thr.finished.connect(self._on_process_thread_finished)
        thr.stateChanged.connect(lambda s, self=self: self._on_process_state_changed(s))
        thr.logMessage.connect(lambda m, self=self: self._on_process_log_message(m))

        self._process_thread = thr

        try:
            self.txtProcessLog.clear()
        except RuntimeError:
            pass

        self._append_process_log("Run-Recipe geladen (Servo-Playback).")

    # =====================================================================
    # SprayPath-Ansteuerung (Soll-Pfad) - alte direkte Variante bleibt drin
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

        _LOG.info("ProcessTab: Rezept '%s' geladen, wende nun auf UI/Threads an.", fname)

        QtCore.QTimer.singleShot(
            0,
            lambda m=model, f=fname, self=self: self._apply_loaded_recipe(m, f)
        )

    def _on_load_run_recipe_clicked(self) -> None:
        runs_dir = self._get_runs_dir()
        start_dir = str(runs_dir)

        fname, _ = QFileDialog.getOpenFileName(
            self,
            "Run-Recipe laden",
            start_dir,
            "YAML (*.yaml *.yml)",
        )
        if not fname:
            return

        base = os.path.basename(fname)
        name_no_ext, _ = os.path.splitext(base)
        if name_no_ext:
            self.set_recipe_name(name_no_ext)

        _LOG.info("ProcessTab: Run-Recipe-Datei '%s' ausgewählt.", fname)

        self._setup_process_thread_for_run(fname)

        try:
            self.lblProcessState.setText("Kein Prozess aktiv (Run-Recipe / Servo).")
        except RuntimeError:
            pass

        self._append_process_log(f"Run-Recipe geladen: {base}")

    def _apply_loaded_recipe(self, model: Recipe, fname: str) -> None:
        if model is None:
            return

        # 1) Model & Name
        self._recipe_model = model
        name = (model.id or "").strip() or os.path.basename(fname)
        self.set_recipe_name(name)

        # 2) Recipe-Textfelder
        try:
            self._update_recipe_info_text()
        except RuntimeError:
            return

        # 3) ProcessThread vorbereiten
        try:
            self._setup_process_thread_for_recipe(model)
        except Exception:
            _LOG.exception("ProcessTab: _setup_process_thread_for_recipe() hat eine Exception geworfen")
            return

        # ✅ VARIANTE 4: Publish strikt entkoppelt und 1s verzögert
        try:
            self._publish_recipe_to_spraypath_delayed(model, delay_ms=1000)
        except Exception:
            _LOG.exception("ProcessTab: delayed spraypath publish scheduling failed")

        # 5) Setup/Scene prüfen
        try:
            self._update_setup_from_scene()
            self._evaluate_scene_match()
        except Exception:
            _LOG.exception("ProcessTab: Szene-/Setup-Update nach Rezeptladen fehlgeschlagen")

        # 6) Status
        try:
            self.lblProcessState.setText("Kein Prozess aktiv.")
        except RuntimeError:
            pass

        _LOG.info("ProcessTab: Rezept '%s' erfolgreich angewendet (id=%r, spraypath delayed).", fname, model.id)

    def _on_start_clicked(self) -> None:
        try:
            if not self.btnStart.isEnabled():
                return
        except RuntimeError:
            return

        if self._process_thread is None:
            QMessageBox.warning(self, "Kein Prozess", "Es ist kein Rezept/Run geladen.")
            return

        self._process_outcome = None

        if self._recipe_model is not None and getattr(self._process_thread, "recipe", None) is not self._recipe_model:
            try:
                self._process_thread.set_recipe(self._recipe_model)
            except Exception:
                _LOG.exception("ProcessTab: set_recipe im ProcessThread fehlgeschlagen")
                QMessageBox.critical(
                    self,
                    "Process-Fehler",
                    "Rezept konnte nicht an den Prozess übergeben werden.",
                )
                return

        self.set_process_active(True)
        self._append_process_log("=== Prozess gestartet ===")

        try:
            self._process_thread.startSignal.emit()
        except Exception:
            _LOG.exception("ProcessTab: startSignal.emit() fehlgeschlagen")
            self.set_process_active(False)

    def _on_stop_clicked(self) -> None:
        self._append_process_log("=== Stop angefordert ===")

        thr = self._process_thread
        if thr is not None:
            try:
                thr.request_stop()
            except Exception:
                _LOG.exception("ProcessTab: request_stop() auf ProcessThread fehlgeschlagen.")

            try:
                if thr.isRunning():
                    thr.stopSignal.emit()
            except Exception:
                _LOG.exception("ProcessTab: stopSignal.emit() auf ProcessThread fehlgeschlagen.")

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

    def _on_process_finished_success(self, result_obj: object) -> None:
        if self._process_outcome == "error":
            _LOG.info("ProcessTab: notifyFinished ignoriert, da bereits ein Fehler gemeldet wurde.")
            return

        if self._process_outcome is None:
            self._process_outcome = "success"

        poses: List[PoseStamped] = []
        planned_traj: Any = None
        executed_traj: Any = None

        if isinstance(result_obj, dict):
            raw_poses = result_obj.get("poses") or result_obj.get("tcp_poses") or []
            if isinstance(raw_poses, list):
                poses = [p for p in raw_poses if isinstance(p, PoseStamped)]

            planned_traj = result_obj.get("planned_traj", None)
            executed_traj = result_obj.get("executed_traj", None)

            if planned_traj is not None:
                if isinstance(planned_traj, list):
                    self._append_process_log(
                        f"Geplante Trajektorien vom Motion-Node empfangen (Segmente: {len(planned_traj)})."
                    )
                else:
                    self._append_process_log("Geplante Trajektorie (planned_traj) vom Motion-Node empfangen.")

            if executed_traj is not None:
                if isinstance(executed_traj, list):
                    self._append_process_log(
                        f"Ausgeführte Trajektorien vom Motion-Node empfangen (Segmente: {len(executed_traj)})."
                    )
                else:
                    self._append_process_log("Ausgeführte Trajektorie (executed_traj) vom Motion-Node empfangen.")
        elif isinstance(result_obj, list):
            poses = [p for p in result_obj if isinstance(p, PoseStamped)]
        else:
            _LOG.warning(
                "ProcessTab: _on_process_finished_success: unerwarteter Result-Typ %r, "
                "erwarte Dict oder List[PoseStamped].",
                type(result_obj),
            )

        if poses:
            try:
                pa = PoseArray()
                frame = poses[0].header.frame_id or "scene"
                pa.header.frame_id = frame
                pa.poses = [p.pose for p in poses]

                if self.bridge is not None and hasattr(self.bridge, "set_executed_path"):
                    try:
                        self.bridge.set_executed_path(pa)  # type: ignore[attr-defined]
                        _LOG.info(
                            "ProcessTab: executed path mit %d Posen an SprayPath gesendet (frame=%s).",
                            len(pa.poses),
                            pa.header.frame_id,
                        )
                    except Exception:
                        _LOG.exception("ProcessTab: bridge.set_executed_path failed")
                else:
                    _LOG.info(
                        "ProcessTab: executed path vorhanden (%d Posen), "
                        "aber bridge.set_executed_path existiert (noch) nicht.",
                        len(poses),
                    )
            except Exception:
                _LOG.exception(
                    "ProcessTab: Fehler beim Erzeugen/Publizieren des executed PoseArray"
                )
        else:
            _LOG.info("ProcessTab: notifyFinished ohne gültige PoseListe oder Liste leer.")

        try:
            metrics = self._compute_trajectory_metrics_from_ps_list(poses)
            score = self._compute_trajectory_score(metrics)

            self._update_info_with_trajectory_metrics(metrics, score)
            self._log_trajectory_metrics(metrics, score)
            if self._recipe_model or self._recipe_name:
                self._save_trajectory_yaml(poses, metrics, score)
        except Exception:
            _LOG.exception("ProcessTab: Fehler bei Berechnung/Anzeige/Speicherung der Trajektorien-Metriken")

        self.set_process_active(False)
        self._info_update("process", "Prozess erfolgreich abgeschlossen.")
        self._append_process_log("=== Prozess erfolgreich abgeschlossen ===")

        try:
            self.lblProcessState.setText("Prozess erfolgreich abgeschlossen.")
        except RuntimeError:
            pass

    @QtCore.pyqtSlot(str)
    def _on_process_finished_error(self, msg: str) -> None:
        if self._process_outcome == "error":
            _LOG.info("ProcessTab: zusätzlicher Fehler-Callback ignoriert: %s", msg)
            return

        if self._process_outcome == "success":
            _LOG.warning(
                "ProcessTab: Fehler-Callback nach notifyFinished erhalten, "
                "Fehler hat Vorrang: %s", msg
            )

        self._process_outcome = "error"

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
            "FINISHED": "Prozess abgeschlossen",
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
                missing.append("Kein Rezept/Run geladen")

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
        self.set_substrate_ok(sub_ok)

        mount_ok = bool(rec_mount_norm) and (cur_mount_norm == rec_mount_norm)
        self.set_mount_ok(mount_ok)
