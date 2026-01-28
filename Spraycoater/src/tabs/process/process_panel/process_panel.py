# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/process_panel.py
from __future__ import annotations

import logging
import os
from typing import Optional, Any, Tuple

import yaml
from PyQt6 import QtCore
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QPushButton,
    QTextEdit,
    QLabel,
    QSizePolicy,
    QMessageBox,
)

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult

from widgets.robot_status_box import RobotStatusInfoBox
from .setup_info_box import SetupInfoBox  # Sicherstellen, dass die Datei existiert

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread

_LOG = logging.getLogger("tabs.process.process_panel")


class ProcessPanel(QWidget):
    """
    ProcessPanel (STRICT, "collect-only"):
    Verantwortlich für:
      - Start/Stop RobotInitThread
      - Start/Stop ProcessThread (Validate/Optimize/Execute)
      - UI Statusanzeige (Robot & Setup)
      - Logging
    """

    sig_robot_ready_changed = QtCore.pyqtSignal(bool)
    sig_process_state = QtCore.pyqtSignal(str)
    sig_log = QtCore.pyqtSignal(str)

    sig_run_started = QtCore.pyqtSignal(str, str)
    sig_run_finished = QtCore.pyqtSignal(str, object, object)
    sig_run_error = QtCore.pyqtSignal(str, str)

    def __init__(
        self,
        *,
        ctx: Any,
        repo: Any,
        ros: RosBridge,
        plc: PlcClientBase | None = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        if ctx is None: raise RuntimeError("ProcessPanel: ctx is None")
        if repo is None: raise RuntimeError("ProcessPanel: repo is None")
        if ros is None: raise RuntimeError("ProcessPanel: ros is None")

        self.ctx = ctx
        self.repo = repo
        self.ros = ros
        self.plc = plc

        self._recipe_key: Optional[str] = None
        self._recipe: Optional[Recipe] = None

        self._robot_initialized: bool = False
        self._require_reinit: bool = False
        self._robot_ready: bool = False

        self._process_active: bool = False
        self._init_active: bool = False
        self._active_mode: str = ""

        self._process_thread: Optional[ProcessThread] = None
        self._init_thread: Optional[RobotInitThread] = None

        self._build_ui()
        self._wire_ui()
        self._wire_robot_status_inbound()
        self._setup_init_thread()

        self._refresh_robot_ready_initial()
        self._update_buttons()

    # ------------------------------------------------------------------
    # UI Layout & Wiring
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        """
        4-Spalten Layout: [Process] | [Setup] | [Robot Status] | [Log]
        """
        root = QHBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        # Spalte 1: Prozess Buttons
        self.ProzessGroupbox = QGroupBox("Process", self)
        vproc = QVBoxLayout(self.ProzessGroupbox)
        vproc.setContentsMargins(8, 8, 8, 8)
        vproc.setSpacing(6)

        self.btnInit = QPushButton("Init", self.ProzessGroupbox)
        self.btnValidate = QPushButton("Validate", self.ProzessGroupbox)
        self.btnOptimize = QPushButton("Optimize", self.ProzessGroupbox)
        self.btnExecute = QPushButton("Execute", self.ProzessGroupbox)
        self.btnStop = QPushButton("Stop", self.ProzessGroupbox)

        for b in (self.btnInit, self.btnValidate, self.btnOptimize, self.btnExecute, self.btnStop):
            b.setMinimumHeight(28)
            vproc.addWidget(b)

        self.lblInit = QLabel("Init: –", self.ProzessGroupbox)
        vproc.addWidget(self.lblInit)

        self.ProzessGroupbox.setFixedWidth(160)
        root.addWidget(self.ProzessGroupbox, 0)

        # Spalte 2: Setup Info (Tool, Substrate, Mount)
        self.SetupGrp = SetupInfoBox(self, title="Hardware Setup")
        self.SetupGrp.setFixedWidth(240)
        root.addWidget(self.SetupGrp, 0)

        # Spalte 3: Robot Status
        self.RobotStatusGrp = RobotStatusInfoBox(self, title="Robot Status")
        self.RobotStatusGrp.setMinimumWidth(360)
        self.RobotStatusGrp.setMaximumWidth(500)
        root.addWidget(self.RobotStatusGrp, 0)

        # Spalte 4: Log (nimmt restlichen Platz ein)
        self.LogGroup = QGroupBox("Log", self)
        vlog = QVBoxLayout(self.LogGroup)
        vlog.setContentsMargins(8, 8, 8, 8)

        self.txtLog = QTextEdit(self.LogGroup)
        self.txtLog.setReadOnly(True)
        vlog.addWidget(self.txtLog, 1)

        root.addWidget(self.LogGroup, 1)

    def _wire_ui(self) -> None:
        """Koppelt UI-Events an die Logik."""
        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnValidate.clicked.connect(lambda: self._start_process(ProcessThread.MODE_VALIDATE))
        self.btnOptimize.clicked.connect(lambda: self._start_process(ProcessThread.MODE_OPTIMIZE))
        self.btnExecute.clicked.connect(lambda: self._start_process(ProcessThread.MODE_EXECUTE))
        self.btnStop.clicked.connect(self.request_stop)

        self.sig_log.connect(self._append_log)

    def _wire_robot_status_inbound(self) -> None:
        """Verbindet ROS-Signale mit den Info-Boxen."""
        # 1. Robot Bridge
        rb = self.ros.robot
        sig = rb.signals
        sig.connectionChanged.connect(self.RobotStatusGrp.set_connection)
        sig.modeChanged.connect(self.RobotStatusGrp.set_mode)
        sig.initializedChanged.connect(self.RobotStatusGrp.set_initialized)
        sig.initializedChanged.connect(self._on_robot_initialized_changed)
        sig.movingChanged.connect(self.RobotStatusGrp.set_moving)
        sig.powerChanged.connect(self.RobotStatusGrp.set_power)
        sig.servoEnabledChanged.connect(self.RobotStatusGrp.set_servo_enabled)
        sig.estopChanged.connect(self.RobotStatusGrp.set_estop)
        sig.errorsChanged.connect(self.RobotStatusGrp.set_errors)
        sig.tcpPoseChanged.connect(self.RobotStatusGrp.set_tcp_from_ps)
        sig.jointsChanged.connect(self._on_joints)

        # 2. Tool & Scene Bridge (Setup)
        if hasattr(self.ros, "tool") and self.ros.tool:
            self.ros.tool.signals.currentToolChanged.connect(self.SetupGrp.set_tool)
        
        if self.ros.scene:
            sc_sig = self.ros.scene.signals
            sc_sig.substrateCurrentChanged.connect(self.SetupGrp.set_substrate)
            sc_sig.mountCurrentChanged.connect(self.SetupGrp.set_mount)

    # ------------------------------------------------------------------
    # Initialisierung & Button-Logik
    # ------------------------------------------------------------------

    def _refresh_robot_ready_initial(self) -> None:
        """Liest initiale Zustände beim Start aus der Bridge."""
        rb = getattr(self.ros, "robot", None)
        if rb:
            cur = getattr(rb, "initialized", None)
            if cur is not None:
                self._robot_initialized = bool(cur)
                self._recompute_robot_ready()

        if self.ros.scene:
            st = self.ros._scene_state 
            self.SetupGrp.set_substrate(st.substrate_current)
            self.SetupGrp.set_mount(st.mount_current)

        if hasattr(self.ros, "tool") and self.ros.tool:
            self.SetupGrp.set_tool(getattr(self.ros.tool, "current_tool_name", "-"))

    def _update_buttons(self) -> None:
        has_recipe = bool(self._recipe_key)
        running = bool(self._process_active)
        init_running = bool(self._init_active)
        can_click = (not running) and (not init_running)

        self.btnInit.setEnabled(can_click)
        self.btnStop.setEnabled(True) # Stop ist immer aktiv

        self.btnValidate.setEnabled(can_click and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled(can_click and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(can_click and has_recipe and self._robot_ready and self._plc_execute_available())

    def _recompute_robot_ready(self) -> None:
        v = bool(self._robot_initialized) and (not bool(self._require_reinit))
        if self._robot_ready == v:
            return
        self._robot_ready = v
        self.sig_robot_ready_changed.emit(v)
        self._update_buttons()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(str, object)
    def set_recipe(self, key: str, recipe: Recipe) -> None:
        if self._process_active or self._init_active:
            raise RuntimeError("ProcessPanel.set_recipe while active")
        self._recipe_key = key
        self._recipe = recipe
        self._update_buttons()

    @QtCore.pyqtSlot()
    def clear_recipe(self) -> None:
        if self._process_active or self._init_active:
            raise RuntimeError("ProcessPanel.clear_recipe while active")
        self._recipe_key = None
        self._recipe = None
        self._update_buttons()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._require_reinit = True
        self._recompute_robot_ready()

        if self._process_thread is not None:
            self._log("Stop: Anfrage an ProcessThread")
            self._process_thread.request_stop()

        if self._init_thread is not None and self._init_thread.isRunning():
            self._log("Stop: Anfrage an RobotInitThread")
            self._init_thread.request_stop()

        if not self._init_active:
            self.lblInit.setText("Init: – (required)")
        self._update_buttons()

    # ------------------------------------------------------------------
    # Prozess & Init Handhabung
    # ------------------------------------------------------------------

    def _start_process(self, mode: str) -> None:
        if self._process_active or self._init_active or not self._recipe_key:
            return

        if self._require_reinit:
            QMessageBox.warning(self, "Process", "Nach STOP muss zuerst Init ausgeführt werden.")
            return

        self._process_active = True
        self._active_mode = str(mode)
        self.txtLog.clear()
        self._log(f"=== {str(mode).upper()} gestartet ===")
        self._update_buttons()

        # Laden der aktuellen Rezept-Daten
        recipe_run = self.repo.load_for_process(str(self._recipe_key))
        
        # XML-Kontext laden
        rd = getattr(self.ctx, "robot_description", None)
        u_xml = str(getattr(rd, "urdf_xml", "") or "")
        s_xml = str(getattr(rd, "srdf_xml", "") or "")

        self._process_thread = ProcessThread(
            ctx=self.ctx, recipe=recipe_run, ros=self.ros, plc=self.plc,
            mode=str(mode), urdf_xml=u_xml, srdf_xml=s_xml
        )
        self._process_thread.stateChanged.connect(self._on_process_state)
        self._process_thread.logMessage.connect(self._log)
        self._process_thread.notifyFinished.connect(self._on_process_finished_success)
        self._process_thread.notifyError.connect(self._on_process_finished_error)
        self._process_thread.start()

    def _setup_init_thread(self) -> None:
        self._init_thread = RobotInitThread(ros=self.ros)
        self._init_thread.notifyFinished.connect(self._on_init_finished_ok)
        self._init_thread.notifyError.connect(self._on_init_finished_err)
        self._init_thread.logMessage.connect(self._log)
        self._init_thread.stateChanged.connect(self._on_init_state)

    def _on_init_clicked(self) -> None:
        if self._process_active: return
        self._init_active = True
        self.lblInit.setText("Init: START")
        self._update_buttons()
        self._init_thread.startSignal.emit()

    def _on_init_finished_ok(self) -> None:
        self._init_active = False
        self.lblInit.setText("Init: OK")
        self._require_reinit = False
        self._recompute_robot_ready()

    def _on_init_finished_err(self, msg: str) -> None:
        self._init_active = False
        self.lblInit.setText("Init: ERROR")
        self._require_reinit = True
        self._recompute_robot_ready()

    # ------------------------------------------------------------------
    # Hilfsfunktionen
    # ------------------------------------------------------------------

    def _plc_execute_available(self) -> bool:
        if self.plc is not None: return True
        cfg = getattr(self.ctx, "plc", None)
        return bool(getattr(cfg, "sim", False))

    def _append_log(self, msg: str) -> None:
        if msg: self.txtLog.append(str(msg))

    def _log(self, msg: str) -> None:
        self.sig_log.emit(str(msg))

    @QtCore.pyqtSlot(str)
    def _on_init_state(self, s: str) -> None:
        self.lblInit.setText(f"Init: {s}")

    @QtCore.pyqtSlot(bool)
    def _on_robot_initialized_changed(self, v: bool) -> None:
        self._robot_initialized = bool(v)
        self._recompute_robot_ready()

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js) -> None:
        if js is not None and hasattr(js, "position"):
            self.RobotStatusGrp.set_joints(list(js.position or []))

    @QtCore.pyqtSlot(str)
    def _on_process_state(self, s: str) -> None:
        self.sig_process_state.emit(str(s))
        self._log(f"STATE: {s}")

    def _finish_process_ui(self) -> None:
        pt = self._process_thread
        self._process_thread = None
        self._process_active = False
        self._active_mode = ""
        self._update_buttons()
        if pt: pt.deleteLater()

    @QtCore.pyqtSlot(object)
    def _on_process_finished_success(self, payload: object) -> None:
        key = str(self._recipe_key or "")
        try:
            rr = RunResult.from_process_payload(payload if isinstance(payload, dict) else {})
            self._persist_raw_runresult(key=key, rr=rr)
        except Exception as e:
            self._log(f"E: Post-Process Fehler: {e}")
        self._finish_process_ui()
        self.sig_run_finished.emit(key, payload, None) # Payload reicht für RecipePanel

    @QtCore.pyqtSlot(str)
    def _on_process_finished_error(self, msg: str) -> None:
        self._log(f"=== Prozess Fehler: {msg} ===")
        self._finish_process_ui()
        self.sig_run_error.emit(str(self._recipe_key), str(msg))

    def _persist_raw_runresult(self, *, key: str, rr: RunResult) -> None:
        p = self.repo.bundle.paths(key)
        def save(path, data):
            if path:
                os.makedirs(os.path.dirname(path), exist_ok=True)
                with open(path, "w", encoding="utf-8") as f:
                    yaml.safe_dump(data or {}, f, allow_unicode=True, sort_keys=False)

        save(getattr(p, "planned_traj_yaml", ""), rr.planned_run.get("traj"))
        save(getattr(p, "executed_traj_yaml", ""), rr.executed_run.get("traj"))
        self._log("Rohdaten gespeichert.")