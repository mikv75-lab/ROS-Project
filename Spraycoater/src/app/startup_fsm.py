# -*- coding: utf-8 -*-
import os
import logging
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

# PLC – liegt bei dir unter src/plc
from plc.plc_config import PlcConfig, load_plc_config
from plc.plc_client import PlcClientBase, PlcClient  # <- Mock entfernt, konkreter Client


class _StepState(QtStateMachine.QState):
    """
    Ruft 'fn' asynchron (singleShot 0 ms) auf, fängt Exceptions und triggert done/failed.
    """

    def __init__(self, name: str, machine: "StartupMachine", fn: Callable[[], None], parent=None):
        super().__init__(parent)
        self._name = name
        self._machine = machine
        self._fn = fn

    def onEntry(self, event: QtCore.QEvent) -> None:  # type: ignore[override]
        self._machine.progress.emit(self._name)
        QtCore.QTimer.singleShot(0, self._run)

    def _run(self) -> None:
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

    Laufzeit-Ressourcen:
      - ROS-UIBridge (self.bridge)
      - PLC-Backend (self.plc)

    WICHTIG:
      - PLC wird NICHT in ctx gespeichert, sondern als eigenes Feld geführt.
      - ready-Signal: (ctx, bridge, plc)
    """

    progress = QtCore.pyqtSignal(str)                 # z.B. "LoadStartup"
    warning = QtCore.pyqtSignal(str)                  # gelbe Meldung auf Splash
    error = QtCore.pyqtSignal(str)                    # optional
    ready = QtCore.pyqtSignal(object, object, object)  # (ctx, bridge, plc)

    # interne Trigger
    _done_sig = QtCore.pyqtSignal()
    _failed_sig = QtCore.pyqtSignal()

    def __init__(
        self,
        startup_yaml_path: str,
        logging_yaml_path: str,
        *,
        abort_on_error: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.startup_yaml_path = os.path.abspath(startup_yaml_path)
        self.logging_yaml_path = os.path.abspath(logging_yaml_path)
        self.abort_on_error = abort_on_error

        self.ctx: Any = None
        self.bridge: Optional[UIBridge] = None
        self.plc: Optional[PlcClientBase] = None   # PLC separat, nicht im ctx

        self._log = logging.getLogger("app.startup")

        # QStateMachine + States
        self.m = QtStateMachine.QStateMachine(self)

        s_init = QtStateMachine.QState()
        s_load = _StepState("LoadStartup", self, self._step_load)
        s_log = _StepState("SetupLogging", self, self._step_logging)
        s_warn = _StepState("SetupWarnings", self, self._step_warnings)
        s_bring = _StepState("StartBringup", self, self._step_bringup)
        s_plc = _StepState("SetupPLC", self, self._step_plc)
        s_bridge = _StepState("ConnectBridge", self, self._step_bridge)
        s_ready = QtStateMachine.QFinalState()

        # transitions
        s_init.addTransition(self._done_sig, s_load)
        s_load.addTransition(self._done_sig, s_log)
        s_log.addTransition(self._done_sig, s_warn)
        s_warn.addTransition(self._done_sig, s_bring)
        s_bring.addTransition(self._done_sig, s_plc)       # Bringup → PLC
        s_plc.addTransition(self._done_sig, s_bridge)      # PLC → Bridge
        s_bridge.addTransition(self._done_sig, s_ready)

        # abort path
        for st in (s_load, s_log, s_warn, s_bring, s_plc, s_bridge):
            st.addTransition(self._failed_sig, s_ready)

        # kickoff
        def _kickoff() -> None:
            QtCore.QTimer.singleShot(0, self._done_sig.emit)

        s_init.entered.connect(_kickoff)

        # final
        self.m.finished.connect(self._emit_ready)

        self.m.addState(s_init)
        self.m.addState(s_load)
        self.m.addState(s_log)
        self.m.addState(s_warn)
        self.m.addState(s_bring)
        self.m.addState(s_plc)
        self.m.addState(s_bridge)
        self.m.addState(s_ready)
        self.m.setInitialState(s_init)

    # -------- Steps --------
    def _step_load(self) -> None:
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

    def _step_logging(self) -> None:
        configure_logging_from_yaml(
            logging_yaml_path=self.logging_yaml_path,
            log_dir=self.ctx.paths.log_dir,
        )
        self._log.info("logging configured")

    def _step_warnings(self) -> None:
        enable_all_warnings(qt=True, pywarnings="always")
        logging.getLogger("qt").info("Qt message handler aktiv")
        self._log.info("warnings enabled")

    def _step_bringup(self) -> None:
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

    def _step_plc(self) -> None:
        """
        Initialisiert das PLC-Backend anhand der PLC-Konfiguration (nur echter Client).

        Ergebnis:
          - self.plc = PlcClientBase-Instanz oder None
          - NICHT in ctx speichern.
        """
        try:
            # Annahme: load_plc_config() kennt intern den Pfad zur plc.yaml
            plc_cfg: PlcConfig = load_plc_config()
        except Exception as e:
            self.warning.emit(f"PLC-Konfiguration konnte nicht geladen werden: {e}")
            self._log.exception("load_plc_config() failed")
            self.plc = None
            return

        try:
            self.plc = PlcClient(plc_cfg)
            self.plc.connect()
            self._log.info(
                "PLC-Backend initialisiert (%s, connected=%s)",
                type(self.plc).__name__,
                getattr(self.plc, "is_connected", None),
            )
        except Exception as e:
            self.warning.emit(f"PLC-Backend konnte nicht initialisiert werden: {e}")
            self._log.exception("SetupPLC fehlgeschlagen")
            self.plc = None

    def _step_bridge(self) -> None:
        """
        Bridge nur verbinden, wenn Bringup gewollt ist.
        PLC ist – falls konfiguriert – bereits in self.plc verfügbar.
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
    def start(self) -> None:
        self.m.start()

    def _emit_ready(self) -> None:
        self._log.info(
            "StartupMachine finished (ctx=%s, bridge=%s, plc=%s)",
            bool(self.ctx),
            bool(self.bridge),
            bool(self.plc),
        )
        if self.ctx is None:
            self.warning.emit("Startup finished without valid ctx (ctx=None).")
        # ctx, bridge, plc an die GUI geben
        self.ready.emit(self.ctx, self.bridge, self.plc)
