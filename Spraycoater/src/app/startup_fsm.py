# -*- coding: utf-8 -*-
import os
import logging
from dataclasses import dataclass
from typing import Optional, Callable, Any

from PyQt6 import QtCore, QtStateMachine  # StateMachine unter PyQt6

# Projekt-Helfer
from config.startup import load_startup
from app.utils.logging_setup import configure_logging_from_yaml
from app.utils.warnings_setup import enable_all_warnings
from ros.ros_launcher import (
    ensure_clean_graph_then_launch,
    BRINGUP_RUNNING,
    make_bringup_cmd,
)
from ros.bridge.ui_bridge import get_ui_bridge, UIBridge


@dataclass
class StartupResult:
    ctx: Any
    bridge: Optional[UIBridge]


class _StepState(QtStateMachine.QState):
    """
    Ruft 'fn' asynchron (singleShot 0 ms) auf, fängt Exceptions und triggert done/failed.
    """
    def __init__(self, name: str, machine: 'StartupMachine', fn: Callable[[], None], parent=None):
        super().__init__(parent)
        self._name = name
        self._machine = machine
        self._fn = fn

    def onEntry(self, event: QtCore.QEvent) -> None:
        self._machine.progress.emit(self._name)
        QtCore.QTimer.singleShot(0, self._run)

    def _run(self):
        try:
            self._fn()
        except Exception as e:
            logging.getLogger("app.startup").exception("%s failed", self._name)
            self._machine.warning.emit(f"{self._name}: {e}")
            if self._machine.abort_on_error:
                self._machine._failed_sig.emit()
                return
        self._machine._done_sig.emit()


class StartupMachine(QtCore.QObject):
    """
    Orchestriert den App-Start via QStateMachine (PyQt6).
    Unverändert bzgl. RViz/PyVista – MainWindow kümmert sich um Splitter & Preview.
    """
    progress = QtCore.pyqtSignal(str)             # z.B. "LoadStartup"
    warning  = QtCore.pyqtSignal(str)             # gelbe Meldung auf Splash
    error    = QtCore.pyqtSignal(str)             # optional
    ready    = QtCore.pyqtSignal(object, object)  # (ctx, bridge)

    # interne Trigger
    _done_sig   = QtCore.pyqtSignal()
    _failed_sig = QtCore.pyqtSignal()

    def __init__(self,
                 startup_yaml_path: str,
                 logging_yaml_path: str,
                 *,
                 abort_on_error: bool = False,
                 parent=None):
        super().__init__(parent)
        self.startup_yaml_path = os.path.abspath(startup_yaml_path)
        self.logging_yaml_path = os.path.abspath(logging_yaml_path)
        self.abort_on_error = abort_on_error

        self.ctx = None
        self.bridge: Optional[UIBridge] = None

        self._log = logging.getLogger("app.startup")

        # QStateMachine + States
        self.m = QtStateMachine.QStateMachine(self)

        s_init   = QtStateMachine.QState()
        s_load   = _StepState("LoadStartup",   self, self._step_load)
        s_log    = _StepState("SetupLogging",  self, self._step_logging)
        s_warn   = _StepState("SetupWarnings", self, self._step_warnings)
        s_bring  = _StepState("StartBringup",  self, self._step_bringup)
        s_bridge = _StepState("ConnectBridge", self, self._step_bridge)
        s_ready  = QtStateMachine.QFinalState()

        # transitions
        s_init.addTransition(self._done_sig,   s_load)
        s_load.addTransition(self._done_sig,   s_log)
        s_log.addTransition(self._done_sig,    s_warn)
        s_warn.addTransition(self._done_sig,   s_bring)
        s_bring.addTransition(self._done_sig,  s_bridge)
        s_bridge.addTransition(self._done_sig, s_ready)

        # abort path
        for st in (s_load, s_log, s_warn, s_bring, s_bridge):
            st.addTransition(self._failed_sig, s_ready)

        # kickoff
        def _kickoff():
            QtCore.QTimer.singleShot(0, self._done_sig.emit)
        s_init.entered.connect(_kickoff)

        # final
        self.m.finished.connect(self._emit_ready)

        self.m.addState(s_init)
        self.m.addState(s_load)
        self.m.addState(s_log)
        self.m.addState(s_warn)
        self.m.addState(s_bring)
        self.m.addState(s_bridge)
        self.m.addState(s_ready)
        self.m.setInitialState(s_init)

    # -------- Steps --------
    def _step_load(self):
        # Strict load
        self.ctx = load_startup(self.startup_yaml_path)
        if self.ctx is None:
            raise RuntimeError("load_startup() returned None")
        self._log.info("startup loaded: %s", self.startup_yaml_path)

        # SC_STARTUP_YAML für Bridge/ROS (Single Source of Truth)
        os.environ["SC_STARTUP_YAML"] = self.startup_yaml_path
        self._log.info("SC_STARTUP_YAML = %s", os.environ.get("SC_STARTUP_YAML"))

        # In Containern SHM-Transport deaktivieren (FastDDS)
        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

    def _step_logging(self):
        configure_logging_from_yaml(
            logging_yaml_path=self.logging_yaml_path,
            log_dir=self.ctx.paths.log_dir
        )
        self._log.info("logging configured")

    def _step_warnings(self):
        enable_all_warnings(qt=True, pywarnings="always")
        logging.getLogger("qt").info("Qt message handler aktiv")
        self._log.info("warnings enabled")

    def _step_bringup(self):
        # Falls ROS in startup.yaml deaktiviert: überspringen
        if not self.ctx.ros.launch_ros:
            self._log.info("ROS launch disabled in startup.yaml -> Bringup übersprungen")
            return

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

        cmd, extra = make_bringup_cmd(self.ctx.ros.sim_robot)
        env = {
            "SPRAY_LOG_DIR": self.ctx.paths.log_dir,
            "SPRAY_LOG_CFG": self.logging_yaml_path,
        }
        ensure_clean_graph_then_launch(
            cmd,
            extra_launch_args=extra,
            log_path=self.ctx.paths.bringup_log,
            env=env,
        )
        if not BRINGUP_RUNNING():
            self.warning.emit(f"Bringup nicht gestartet. Log: {self.ctx.paths.bringup_log}")

    def _step_bridge(self):
        """
        Bridge nur verbinden, wenn Bringup gewollt ist.
        """
        if not self.ctx.ros.launch_ros:
            self._log.info("ROS launch disabled -> Bridge-Verbindung übersprungen")
            self.bridge = None
            return

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

        self.bridge = get_ui_bridge(self.ctx)
        try:
            self.bridge.connect()
            logging.getLogger("ros").info(
                "UIBridge verbunden (node=%s).",
                getattr(self.bridge, "node_name", "ui_bridge"),
            )
        except Exception as e:
            self.warning.emit(f"UIBridge nicht verbunden: {e}")
            self.bridge = None  # ohne Bridge weiter

    # -------- Control --------
    def start(self):
        self.m.start()

    def _emit_ready(self):
        self._log.info("StartupMachine finished (ctx=%s, bridge=%s)", bool(self.ctx), bool(self.bridge))
        if self.ctx is None:
            self.warning.emit("Startup finished without valid ctx (ctx=None).")
        self.ready.emit(self.ctx, self.bridge)
