# -*- coding: utf-8 -*-
import os
import logging
from typing import Optional, Callable, Any

from PyQt6 import QtCore, QtStateMachine  # StateMachine unter PyQt6

# Projekt-Helfer
from config.startup import load_startup
from app.utils.logging_setup import init_logging
from app.utils.warnings_setup import enable_all_warnings
from ros.ros_launcher import (
    ensure_clean_graph_then_launch,
    BRINGUP_RUNNING,
    make_bringup_cmd,
)
from ros.bridge.ui_bridge import get_ui_bridge, UIBridge

# PLC – liegt bei dir unter src/plc
from plc.plc_config import PlcConfig, load_plc_config
from plc.plc_client import PlcClientBase, PlcClient


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
        except Exception:
            logging.getLogger("app.startup").exception("%s failed", self._name)
            self._machine.warning.emit(f"{self._name} failed – siehe Log.")
            if self._machine.abort_on_error:
                self._machine._failed_sig.emit()
                return
        self._machine._done_sig.emit()


class StartupMachine(QtCore.QObject):
    """
    Orchestriert den App-Start via QStateMachine (PyQt6).

    Laufzeit-Ressourcen:
      - ROS-UIBridge SHADOW (self.bridge_shadow)
      - ROS-UIBridge LIVE   (self.bridge_live)
      - PLC-Backend         (self.plc)

    WICHTIG:
      - PLC wird NICHT in ctx gespeichert, sondern als eigenes Feld geführt.
      - ready-Signal: (ctx, bridge_shadow, bridge_live, plc)
    """

    progress = QtCore.pyqtSignal(str)                     # z.B. "LoadStartup"
    warning = QtCore.pyqtSignal(str)                     # gelbe Meldung auf Splash
    error = QtCore.pyqtSignal(str)                       # optional
    ready = QtCore.pyqtSignal(object, object, object, object)
    #               ctx,    bridge_shadow, bridge_live,   plc

    # interne Trigger
    _done_sig = QtCore.pyqtSignal()
    _failed_sig = QtCore.pyqtSignal()

    def __init__(
        self,
        startup_yaml_path: str,
        *,
        abort_on_error: bool = False,
        parent=None,
    ):
        super().__init__(parent)
        self.startup_yaml_path = os.path.abspath(startup_yaml_path)
        self.abort_on_error = abort_on_error

        self.ctx: Any = None
        self.bridge_shadow: Optional[UIBridge] = None
        self.bridge_live: Optional[UIBridge] = None
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
        s_plc.addTransition(self._done_sig, s_bridge)      # PLC → Bridges
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
        """
        Neues Logging-Setup ohne logging.yaml:
          - init_logging(log_dir)
          - eigenständiger File-Logger für app.startup
        """
        init_logging(self.ctx.paths.log_dir)

        # Logfile für diesen Logger direkt anlegen
        try:
            logging.add_file_logger("app.startup")  # type: ignore[attr-defined]
        except AttributeError:
            # Falls add_file_logger nicht gepatcht wurde (Safety)
            from app.utils.logging_setup import add_file_logger  # type: ignore
            add_file_logger("app.startup")

        self._log.info("logging configured (AppLogging)")

    def _step_warnings(self) -> None:
        enable_all_warnings(qt=True, pywarnings="always")
        logging.getLogger("qt").info("Qt message handler aktiv")
        self._log.info("warnings enabled")

    def _step_bringup(self) -> None:
        # Falls ROS in startup.yaml deaktiviert: überspringen
        if not getattr(self.ctx.ros, "launch_ros", False):
            self._log.info("ROS launch disabled in startup.yaml -> Bringup übersprungen")
            return

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

        # make_bringup_cmd muss intern aus ctx.ros (shadow/live) ableiten,
        # wie viele Launches (shadow/live) gestartet werden.
        cmd, extra = make_bringup_cmd(self.ctx.ros)
        env = {
            "SPRAY_LOG_DIR": self.ctx.paths.log_dir,
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
            plc_cfg: PlcConfig = load_plc_config()
        except Exception as e:
            self.warning.emit(f"PLC-Konfiguration konnte nicht geladen werden: {e}")
            self._log.exception("load_plc_config() failed")
            self.plc = None
            return

        # Wenn im startup.yaml plc.sim: true → kein Connect
        if getattr(plc_cfg, "sim", False):
            self._log.info("PLC im Simulationsmodus (sim=true) -> keine ADS-Verbindung")
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
        Bridges für shadow + live nur verbinden, wenn ROS-Launch aktiv ist.

        Erwartet, dass ctx.ros die Struktur aus startup.yaml hat:

          ros.shadow.enabled: bool
          ros.live.enabled:   bool
        """
        if not getattr(self.ctx.ros, "launch_ros", False):
            self._log.info("ROS launch disabled -> Bridge-Verbindungen übersprungen")
            self.bridge_shadow = None
            self.bridge_live = None
            return

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

        self.bridge_shadow = None
        self.bridge_live = None

        # --- SHADOW-Bridge ---
        try:
            shadow_cfg = getattr(self.ctx.ros, "shadow", None)
            shadow_enabled = bool(getattr(shadow_cfg, "enabled", False)) if shadow_cfg else False
            if shadow_enabled:
                # get_ui_bridge muss Rolle 'shadow' verstehen
                self.bridge_shadow = get_ui_bridge(self.ctx, namespace="shadow")
                self.bridge_shadow.connect()
                logging.getLogger("ros").info(
                    "UIBridge SHADOW verbunden (node=%s).",
                    getattr(self.bridge_shadow, "node_name", "ui_bridge_shadow"),
                )
            else:
                self._log.info("ROS shadow-role disabled -> keine Shadow-Bridge.")
        except Exception as e:
            self.warning.emit(f"UIBridge SHADOW nicht verbunden: {e}")
            self._log.exception("ConnectBridge[shadow] fehlgeschlagen")
            self.bridge_shadow = None

        # --- LIVE-Bridge ---
        try:
            live_cfg = getattr(self.ctx.ros, "live", None)
            live_enabled = bool(getattr(live_cfg, "enabled", False)) if live_cfg else False
            if live_enabled:
                # get_ui_bridge muss Rolle 'live' verstehen
                self.bridge_live = get_ui_bridge(self.ctx, namespace="live")
                self.bridge_live.connect()
                logging.getLogger("ros").info(
                    "UIBridge LIVE verbunden (node=%s).",
                    getattr(self.bridge_live, "node_name", "ui_bridge_live"),
                )
            else:
                self._log.info("ROS live-role disabled -> keine Live-Bridge.")
        except Exception as e:
            self.warning.emit(f"UIBridge LIVE nicht verbunden: {e}")
            self._log.exception("ConnectBridge[live] fehlgeschlagen")
            self.bridge_live = None

    # -------- Control --------
    def start(self) -> None:
        self.m.start()

    def _emit_ready(self) -> None:
        self._log.info(
            "StartupMachine finished (ctx=%s, shadow=%s, live=%s, plc=%s)",
            bool(self.ctx),
            bool(self.bridge_shadow),
            bool(self.bridge_live),
            bool(self.plc),
        )
        if self.ctx is None:
            self.warning.emit("Startup finished without valid ctx (ctx=None).")
        # ctx, bridge_shadow, bridge_live, plc an die GUI geben
        self.ready.emit(self.ctx, self.bridge_shadow, self.bridge_live, self.plc)
