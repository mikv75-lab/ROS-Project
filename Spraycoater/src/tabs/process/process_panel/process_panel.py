# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/process_panel.py
from __future__ import annotations

import logging
from typing import Optional, Any, Tuple

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

from .process_thread import ProcessThread
from .robot_init_thread import RobotInitThread
from .base_statemachine import (
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process.process_panel")


class ProcessPanel(QWidget):
    """
    ProcessPanel (STRICT):

    Responsibilities:
      - Start/stop RobotInitThread
      - Start/stop ProcessThread (validate/optimize/execute)
      - Owns button enable/disable rules + runtime checks
      - Owns log group
      - Wires RobotStatusInfoBox to RosBridge robot signals

    Explicitly NOT here (moved to RecipePanel):
      - NO postprocess / FK / TCP building
      - NO evaluation
      - NO persistence

    Public API:
      - set_recipe(key: str, recipe_model: Recipe)
      - clear_recipe()
      - request_stop()

    Signals (panel -> outer):
      - sig_robot_ready_changed(bool)
      - sig_process_state(str)
      - sig_log(str)
      - sig_run_started(str mode, str key)
      - sig_run_finished(str key, object rr)
      - sig_run_error(str key, str message)
    """

    sig_robot_ready_changed = QtCore.pyqtSignal(bool)
    sig_process_state = QtCore.pyqtSignal(str)
    sig_log = QtCore.pyqtSignal(str)

    sig_run_started = QtCore.pyqtSignal(str, str)
    sig_run_finished = QtCore.pyqtSignal(str, object)  # key, RunResult
    sig_run_error = QtCore.pyqtSignal(str, str)

    _SEG_ORDER = (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME)

    def __init__(
        self,
        *,
        ctx: Any,
        ros: RosBridge,
        plc: PlcClientBase | None = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        if ctx is None:
            raise RuntimeError("ProcessPanel: ctx is None (strict)")
        if ros is None:
            raise RuntimeError("ProcessPanel: ros is None (strict)")

        self.ctx = ctx
        self.ros = ros
        self.plc = plc

        self._recipe_key: Optional[str] = None
        self._recipe_model: Optional[Recipe] = None

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
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)

        top = QHBoxLayout()
        top.setSpacing(8)

        self.ProzessGroupbox = QGroupBox("Process", self)
        vproc = QVBoxLayout(self.ProzessGroupbox)
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

        top.addWidget(self.ProzessGroupbox, 1)

        self.RobotStatusGrp = RobotStatusInfoBox(self, title="Robot Status")
        top.addWidget(self.RobotStatusGrp, 2)

        root.addLayout(top, 0)

        self.LogGroup = QGroupBox("Log", self)
        vlog = QVBoxLayout(self.LogGroup)
        self.txtLog = QTextEdit(self.LogGroup)
        self.txtLog.setReadOnly(True)
        vlog.addWidget(self.txtLog, 1)
        root.addWidget(self.LogGroup, 1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.setSizePolicy(sp)

    def _wire_ui(self) -> None:
        self.btnInit.clicked.connect(self._on_init_clicked)
        self.btnValidate.clicked.connect(lambda: self._start_process(ProcessThread.MODE_VALIDATE))
        self.btnOptimize.clicked.connect(lambda: self._start_process(ProcessThread.MODE_OPTIMIZE))
        self.btnExecute.clicked.connect(lambda: self._start_process(ProcessThread.MODE_EXECUTE))
        self.btnStop.clicked.connect(self.request_stop)

        self.sig_log.connect(self._append_log)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(str, object)
    def set_recipe(self, key: str, recipe_model: Recipe) -> None:
        if self._process_active or self._init_active:
            raise RuntimeError("ProcessPanel.set_recipe while active (strict)")

        key = str(key or "").strip()
        if not key:
            raise ValueError("ProcessPanel.set_recipe: empty key (strict)")
        if recipe_model is None:
            raise ValueError("ProcessPanel.set_recipe: recipe_model is None (strict)")

        self._recipe_key = key
        self._recipe_model = recipe_model
        self._update_buttons()

    @QtCore.pyqtSlot()
    def clear_recipe(self) -> None:
        if self._process_active or self._init_active:
            raise RuntimeError("ProcessPanel.clear_recipe while active (strict)")
        self._recipe_key = None
        self._recipe_model = None
        self._update_buttons()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._process_thread is not None:
            self._log("Stop: request_stop() -> ProcessThread")
            self._process_thread.request_stop()

        if self._init_thread is not None and self._init_thread.isRunning():
            self._log("Stop: request_stop() -> RobotInitThread")
            self._init_thread.request_stop()

        self._update_buttons()

    # ------------------------------------------------------------------
    # Button rules (STRICT)
    # ------------------------------------------------------------------

    def _plc_execute_available(self) -> bool:
        if self.plc is not None:
            return True
        plc_cfg = getattr(self.ctx, "plc", None)
        if plc_cfg is None:
            return False
        return bool(getattr(plc_cfg, "sim", False))

    def _update_buttons(self) -> None:
        has_recipe = bool(self._recipe_key)
        running = bool(self._process_active)
        init_running = bool(self._init_active)
        can_click = (not running) and (not init_running)

        self.btnInit.setEnabled(can_click)
        self.btnStop.setEnabled(bool(running or init_running))

        self.btnValidate.setEnabled(can_click and has_recipe and self._robot_ready)
        self.btnOptimize.setEnabled(can_click and has_recipe and self._robot_ready)
        self.btnExecute.setEnabled(can_click and has_recipe and self._robot_ready and self._plc_execute_available())

    # ------------------------------------------------------------------
    # Logging
    # ------------------------------------------------------------------

    def _append_log(self, msg: str) -> None:
        if msg:
            self.txtLog.append(str(msg))

    def _log(self, msg: str) -> None:
        self.sig_log.emit(str(msg))

    # ------------------------------------------------------------------
    # Robot status wiring
    # ------------------------------------------------------------------

    def _wire_robot_status_inbound(self) -> None:
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

    def _refresh_robot_ready_initial(self) -> None:
        rb = getattr(self.ros, "robot", None)
        if rb is None:
            return
        try:
            cur = getattr(rb, "initialized", None)
            if cur is not None:
                self._set_robot_ready(bool(cur))
        except Exception:
            pass

    def _set_robot_ready(self, v: bool) -> None:
        v = bool(v)
        if self._robot_ready == v:
            return
        self._robot_ready = v
        self.sig_robot_ready_changed.emit(v)
        self._update_buttons()

    @QtCore.pyqtSlot(bool)
    def _on_robot_initialized_changed(self, v: bool) -> None:
        self._set_robot_ready(bool(v))

    @QtCore.pyqtSlot(object)
    def _on_joints(self, js) -> None:
        if js is None or not hasattr(js, "position"):
            self.RobotStatusGrp.set_joints(None)
            return
        self.RobotStatusGrp.set_joints(list(js.position or []))

    # ------------------------------------------------------------------
    # Robot Init
    # ------------------------------------------------------------------

    def _setup_init_thread(self) -> None:
        if self._init_thread is not None:
            return

        self._init_thread = RobotInitThread(ros=self.ros)
        self._init_thread.notifyFinished.connect(self._on_init_finished_ok)
        self._init_thread.notifyError.connect(self._on_init_finished_err)
        self._init_thread.logMessage.connect(self._log)
        self._init_thread.stateChanged.connect(self._on_init_state)

    @QtCore.pyqtSlot(str)
    def _on_init_state(self, s: str) -> None:
        self.lblInit.setText(f"Init: {s}")

    def _on_init_clicked(self) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Init", "Während eines laufenden Prozesses nicht möglich.")
            return
        if self._init_thread is None:
            raise RuntimeError("ProcessPanel: RobotInitThread missing (strict)")
        if self._init_thread.isRunning():
            self._log("RobotInit: already running")
            return

        self._init_active = True
        self.lblInit.setText("Init: START")
        self._log("=== Robot-Init gestartet ===")
        self._update_buttons()

        self._init_thread.startSignal.emit()

    def _on_init_finished_ok(self) -> None:
        self._init_active = False
        self.lblInit.setText("Init: OK")
        self._log("=== Robot-Init erfolgreich abgeschlossen ===")
        self._update_buttons()

    def _on_init_finished_err(self, msg: str) -> None:
        self._init_active = False
        self.lblInit.setText("Init: ERROR")
        self._log(f"=== Robot-Init Fehler: {msg} ===")
        self._update_buttons()

    # ------------------------------------------------------------------
    # Process lifecycle (STRICT: no postprocess/persist here)
    # ------------------------------------------------------------------

    def _ctx_robot_xml(self) -> Tuple[str, str]:
        rd = getattr(self.ctx, "robot_description", None)
        urdf = str(getattr(rd, "urdf_xml", "") or "")
        srdf = str(getattr(rd, "srdf_xml", "") or "")
        return urdf, srdf

    def _start_process(self, mode: str) -> None:
        if self._process_active:
            QMessageBox.warning(self, "Process", "Es läuft bereits ein Prozess.")
            return
        if self._init_active:
            QMessageBox.warning(self, "Process", "Während RobotInit läuft nicht möglich.")
            return

        if not self._recipe_key or self._recipe_model is None:
            QMessageBox.warning(self, "Process", "Kein Rezept geladen.")
            return
        if not self._robot_ready:
            QMessageBox.warning(self, "Process", "Robot nicht initialisiert / nicht ready.")
            return
        if mode == ProcessThread.MODE_EXECUTE and not self._plc_execute_available():
            QMessageBox.warning(self, "Execute", "PLC nicht verfügbar (Execute benötigt PLC oder plc.sim=true).")
            return

        # STRICT: Execute requires planned_traj baseline in the CURRENT recipe model
        if mode == ProcessThread.MODE_EXECUTE and getattr(self._recipe_model, "planned_traj", None) is None:
            QMessageBox.warning(
                self,
                "Execute",
                "Execute benötigt planned_traj.yaml (Baseline). Bitte zuerst Validate/Optimize ausführen.",
            )
            return

        key = str(self._recipe_key)
        recipe_run = self._recipe_model  # STRICT: run uses the current model (no repo reload here)

        urdf_xml, srdf_xml = self._ctx_robot_xml()

        self._process_active = True
        self._active_mode = str(mode)
        self.txtLog.clear()

        self._log(f"=== {str(mode).upper()} gestartet ===")
        self._update_buttons()
        self.sig_run_started.emit(str(mode), key)

        pt = ProcessThread(
            ctx=self.ctx,
            recipe=recipe_run,
            ros=self.ros,
            plc=self.plc,
            mode=str(mode),
            urdf_xml=urdf_xml,
            srdf_xml=srdf_xml,
        )
        pt.stateChanged.connect(self._on_process_state)
        pt.logMessage.connect(self._log)
        pt.notifyFinished.connect(self._on_process_finished_success)
        pt.notifyError.connect(self._on_process_finished_error)

        self._process_thread = pt
        pt.start()

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

        if pt is not None:
            try:
                pt.deleteLater()
            except Exception:
                pass

    @QtCore.pyqtSlot(object)
    def _on_process_finished_success(self, payload: object) -> None:
        key = str(self._recipe_key or "")

        # STRICT: Process produces RunResult only; no postprocess/eval/persist here.
        try:
            rr = RunResult.from_process_payload(payload)
        except Exception as e:
            self._log(f"E: RunResult.from_process_payload failed: {e}")
            self._finish_process_ui()
            self.sig_run_error.emit(key, f"payload_decode_failed: {e}")
            return

        self._finish_process_ui()
        self.sig_run_finished.emit(key, rr)

    @QtCore.pyqtSlot(str)
    def _on_process_finished_error(self, msg: str) -> None:
        key = str(self._recipe_key or "")
        self._log(f"=== Process ERROR: {msg} ===")
        self._finish_process_ui()
        self.sig_run_error.emit(key, str(msg or "unknown_error"))
