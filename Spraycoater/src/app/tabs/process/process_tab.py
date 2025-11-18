# -*- coding: utf-8 -*-
# File: tabs/process/process_tab.py
from __future__ import annotations
import os
import math
import logging
from typing import Optional, List

from PyQt6 import QtCore
from PyQt6.sip import isdeleted
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
        [GroupBox] Recipe
           ├─ QTextEdit  (Summary)
           └─ QTextEdit  (Poses)
      )
    """

    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # Bridges
        self._rb = getattr(self.bridge, "_rb", None)
        self._sig = getattr(self._rb, "signals", None) if self._rb else None
        # Pose-Konfiguration (Home/Service):
        #   - PosesBridge: bridge._pb
        #   - PosesState:  bridge.poses  (signal-freier Cache)
        self._pb = getattr(self.bridge, "_pb", None)
        self._poses_state = getattr(self.bridge, "poses", None)

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

        # Reentrancy-Guard für _update_start_conditions
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
            # Mode nur noch für Anzeige, Home-Check läuft über Pose
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
        """
        Home-Pose aus PosesBridge; wenn sich diese ändert,
        Robot-at-home neu auswerten.
        """
        if not self._pb:
            return
        sigp = getattr(self._pb, "signals", None)
        if not sigp:
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
        """
        TCP-Pose direkt aus der RobotBridge.
        (RobotStatusBox wird ohnehin per Signal gefüttert.)
        """
        rb = self._rb
        if rb is None:
            return None
        pose = getattr(rb, "tcp_pose", None)
        if isinstance(pose, PoseStamped) or pose is None:
            return pose
        return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        """
        Holt die Home-Pose:
          1. bevorzugt aus der PosesBridge (get_last_home_pose)
          2. Fallback: aus dem PosesState (bridge.poses.home())
        """
        # 1) PosesBridge
        if self._pb is not None:
            try:
                pose = self._pb.get_last_home_pose()
                if isinstance(pose, PoseStamped):
                    return pose
            except Exception:
                pass

        # 2) PosesState der UIBridge
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
        """
        Vorhandenen ProcessThread stoppen/aufräumen, ohne auf bereits
        gelöschte Qt-Objekte zuzugreifen.
        """
        thr = self._process_thread
        self._process_thread = None

        if thr is None or isdeleted(thr):
            return

        if thr.isRunning():
            thr.request_stop()
            thr.wait(2000)

        thr.deleteLater()

    def _setup_process_thread_for_recipe(self, recipe: Recipe) -> None:
        """
        Neuen ProcessThread für das übergebene Rezept anlegen.
        Wird NICHT ausgeführt, wenn der ProcessTab bereits zerstört ist.
        """
        if isdeleted(self):
            _LOG.warning("ProcessTab already deleted, skip _setup_process_thread_for_recipe()")
            return

        self._cleanup_process_thread()

        thr = ProcessThread(recipe=recipe, bridge=self.bridge, parent=self)
        thr.notifyFinished.connect(self._on_process_finished_success)
        thr.notifyError.connect(self._on_process_finished_error)
        thr.finished.connect(self._on_process_thread_finished)
        self._process_thread = thr

    # =====================================================================
    # SprayPath-Ansteuerung aus ProcessTab
    # =====================================================================

    def _send_recipe_to_spraypath(self, recipe: Recipe) -> None:
        """
        Baut ein MarkerArray aus dem geladenen Rezept und schickt es
        über die UIBridge an spray_path.set – analog zum RecipeTab.
        """
        if self.bridge is None or recipe is None:
            return

        try:
            # Hier keine UI-Sides, d.h. kompletter Pfad laut Rezept
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
        """
        Startet die Init-StateMachine (RobotInitThread):

          - Robot init (über RobotBridge)
          - MoveToHome (MotionBridge/RobotBridge)
          - Warten bis TCP-Pose ≈ Home-Pose
        """
        if self._robot_init_thread is None:
            QMessageBox.warning(self, "Robot Init", "RobotInitThread nicht verfügbar.")
            return

        self.set_robot_initialized(False)
        self.set_robot_at_home(False)

        if not isdeleted(self.btnInit):
            self.btnInit.setEnabled(False)

        self.infoBox.set_values({"process": "Robot-Init läuft (Init + Home via Pose-Vergleich)..."})

        self._robot_init_thread.startSignal.emit()

    def _on_load_recipe_clicked(self) -> None:
        if isdeleted(self):
            _LOG.warning("ProcessTab already deleted, skip _on_load_recipe_clicked()")
            return

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

        # Rezept auch an SprayPath schicken (wie im RecipeTab)
        self._send_recipe_to_spraypath(model)

        # einmal initial; danach hält der Timer es aktuell
        self._update_setup_from_scene()
        self._evaluate_scene_match()

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
        """
        Stop-Button:
          - stoppt die Prozess-StateMachine (ProcessThread)
          - stoppt die Robot-Init-StateMachine (RobotInitThread)
        """
        # Prozess-StateMachine stoppen
        if self._process_thread is not None and self._process_thread.isRunning():
            self._process_thread.stopSignal.emit()

        # Robot-Init-StateMachine stoppen
        if self._robot_init_thread is not None and self._robot_init_thread.isRunning():
            self._robot_init_thread.stopSignal.emit()

    # ---------------------------------------------------------------------
    # Callbacks Robot-Init
    # ---------------------------------------------------------------------

    def _on_robot_init_finished(self) -> None:
        # Init-Thread meldet Erfolg -> init=True, Home via Pose checken
        self.set_robot_initialized(True)
        self._recompute_robot_at_home()
        if not isdeleted(self.btnInit):
            self.btnInit.setEnabled(True)

        cur = self.infoBox.get_values() if hasattr(self.infoBox, "get_values") else {}
        cur = dict(cur or {})
        cur["robot_init"] = "Roboter initialisiert und Home-Pose erreicht (TCP≈Home)."
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
        if isdeleted(self):
            return

        txt_summary = getattr(self, "txtRecipeSummary", None)
        txt_poses = getattr(self, "txtRecipePoses", None)
        info_box = getattr(self, "infoBox", None)

        if (
            txt_summary is None
            or txt_poses is None
            or isdeleted(txt_summary)
            or isdeleted(txt_poses)
        ):
            return

        if not self._recipe_model:
            txt_summary.setPlainText("")
            txt_poses.setPlainText("")
            if info_box is not None and not isdeleted(info_box):
                info_box.set_values(None)
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

        txt_summary.setPlainText("\n".join(summary_lines).rstrip())
        txt_summary.moveCursor(txt_summary.textCursor().Start)

        txt_poses.setPlainText("\n".join(poses_lines).rstrip())
        txt_poses.moveCursor(txt_poses.textCursor().Start)

        if info_box is not None and not isdeleted(info_box):
            info_box.set_values(self._recipe_model.info or {})

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

    @staticmethod
    def _norm_mesh_name(name: str) -> str:
        """
        Für den Vergleich:
          - basename
          - .stl (case-insensitive) abschneiden
        """
        n = os.path.basename(name or "").strip()
        if n.lower().endswith(".stl"):
            n = n[:-4]
        return n

    def _update_start_conditions(self) -> None:
        if isdeleted(self):
            return

        # Reentrancy-Guard, um Rekursion zu verhindern
        if self._in_update_start_conditions:
            return

        self._in_update_start_conditions = True
        try:
            # JEDES MAL: aktuelle Scene holen + Match neu bewerten
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
        finally:
            self._in_update_start_conditions = False

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

    def _get_scene_currents(self) -> tuple[str, str, str]:
        """
        Liest immer direkt die aktuellen Werte von der SceneBridge:

          - tool_current / tool
          - substrate_current
          - mount_current
        """
        scene_br = getattr(self.bridge, "_sb", None) or getattr(self.bridge, "_scene", None)
        if scene_br is None:
            return "", "", ""

        tool = getattr(scene_br, "tool_current", "") or getattr(scene_br, "tool", "") or ""
        substrate = getattr(scene_br, "substrate_current", "") or ""
        mount = getattr(scene_br, "mount_current", "") or ""

        return str(tool), str(substrate), str(mount)

    def _update_setup_from_scene(self) -> None:
        """
        Setup-Anzeige im UI; basiert direkt auf den *current*-Feldern
        der SceneBridge zum Zeitpunkt des Aufrufs.
        """
        tool, substrate, mount = self._get_scene_currents()

        # Falls kein Tool, evtl. aus RobotBridge
        if not tool and self._rb is not None:
            tool = getattr(self._rb, "current_tool", "") or getattr(self._rb, "tool", "")

        self._set_setup_labels(tool or "-", substrate or "-", mount or "-")

    def _set_setup_labels(self, tool: str, substrate: str, mount: str) -> None:
        self.lblTool.setText(f"Tool: {tool}")
        self.lblSubstrate.setText(f"Substrate: {substrate}")
        self.lblMount.setText(f"Mount: {mount}")

    def _evaluate_scene_match(self) -> None:
        """
        Prüft Substrate/Mount gegen das Rezept auf Basis der aktuellen
        *current*-Felder der SceneBridge.
        Vergleich erfolgt auf basename ohne .stl.
        """
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
