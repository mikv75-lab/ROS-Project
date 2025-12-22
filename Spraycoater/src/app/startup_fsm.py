# -*- coding: utf-8 -*-
# File: src/app/startup_fsm.py
#
# Fixes:
# - Bridge ist RosBridge, Variable/Semantik konsistent
# - ready(ctx, bridge_shadow, bridge_live, plc) bleibt, aber intern sauber benannt
# - Bridge wird nur gestartet wenn ros.launch_ros true ist
# - Jede StepState fängt Exceptions, optional abort_on_error

from __future__ import annotations

import os
import logging
from typing import Optional, Callable, Any

from PyQt6 import QtCore, QtStateMachine

from config.startup import load_startup, PlcConfig
from app.utils.logging_setup import init_logging
from app.utils.warnings_setup import enable_all_warnings

from ros.ros_launcher import (
    ensure_clean_graph_then_launch,
    BRINGUP_RUNNING,
    make_bringup_cmd,
)

from ros.bridge.ros_bridge import RosBridge

from plc.plc_client import PlcClientBase, create_plc_client

_LOG = logging.getLogger("app.startup")


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
      - RosBridge SHADOW (self.bridge_shadow)
      - RosBridge LIVE   (self.bridge_live)
      - PLC-Backend      (self.plc)

    ready-Signal: (ctx, bridge_shadow, bridge_live, plc)
    """

    progress = QtCore.pyqtSignal(str)
    warning = QtCore.pyqtSignal(str)
    error = QtCore.pyqtSignal(str)
    ready = QtCore.pyqtSignal(object, object, object, object)

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
        self.bridge_shadow: Optional[RosBridge] = None
        self.bridge_live: Optional[RosBridge] = None
        self.plc: Optional[PlcClientBase] = None

        self._log = logging.getLogger("app.startup")

        self.m = QtStateMachine.QStateMachine(self)

        s_init = QtStateMachine.QState()
        s_load = _StepState("LoadStartup", self, self._step_load)
        s_log = _StepState("SetupLogging", self, self._step_logging)
        s_warn = _StepState("SetupWarnings", self, self._step_warnings)
        s_bring = _StepState("StartBringup", self, self._step_bringup)
        s_plc = _StepState("SetupPLC", self, self._step_plc)
        s_bridge = _StepState("ConnectBridge", self, self._step_bridge)
        s_ready = QtStateMachine.QFinalState()

        s_init.addTransition(self._done_sig, s_load)
        s_load.addTransition(self._done_sig, s_log)
        s_log.addTransition(self._done_sig, s_warn)
        s_warn.addTransition(self._done_sig, s_bring)
        s_bring.addTransition(self._done_sig, s_plc)
        s_plc.addTransition(self._done_sig, s_bridge)
        s_bridge.addTransition(self._done_sig, s_ready)

        for st in (s_load, s_log, s_warn, s_bring, s_plc, s_bridge):
            st.addTransition(self._failed_sig, s_ready)

        def _kickoff() -> None:
            QtCore.QTimer.singleShot(0, self._done_sig.emit)

        s_init.entered.connect(_kickoff)

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
        self.ctx = load_startup(self.startup_yaml_path)
        if self.ctx is None:
            raise RuntimeError("load_startup() returned None")
        self._log.info("startup loaded: %s", self.startup_yaml_path)

        os.environ["SC_STARTUP_YAML"] = self.startup_yaml_path
        self._log.info("SC_STARTUP_YAML = %s", os.environ.get("SC_STARTUP_YAML"))

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

    def _step_logging(self) -> None:
        # ctx.paths.log_dir ist canonical
        init_logging(self.ctx.paths.log_dir)

        try:
            logging.add_file_logger("app.startup")  # type: ignore[attr-defined]
        except AttributeError:
            from app.utils.logging_setup import add_file_logger  # type: ignore
            add_file_logger("app.startup")

        self._log.info("logging configured (AppLogging)")

    def _step_warnings(self) -> None:
        enable_all_warnings(qt=True, pywarnings="always")
        logging.getLogger("qt").info("Qt message handler aktiv")
        self._log.info("warnings enabled")

    def _step_bringup(self) -> None:
        if not getattr(self.ctx.ros, "launch_ros", False):
            self._log.info("ROS launch disabled in startup.yaml -> Bringup übersprungen")
            return

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

        cmd, extra = make_bringup_cmd(self.ctx.ros, startup_yaml_path=self.startup_yaml_path)

        env = {"SPRAY_LOG_DIR": self.ctx.paths.log_dir}
        ensure_clean_graph_then_launch(
            cmd,
            extra_launch_args=extra,
            log_path=self.ctx.paths.bringup_log,
            env=env,
        )
        if not BRINGUP_RUNNING():
            self.warning.emit(f"Bringup nicht gestartet. Log: {self.ctx.paths.bringup_log}")

    def _step_plc(self) -> None:
        if self.ctx is None:
            self.warning.emit("PLC-Setup übersprungen: ctx ist None.")
            self._log.error("SetupPLC: ctx ist None")
            self.plc = None
            return

        plc_cfg: Optional[PlcConfig] = getattr(self.ctx, "plc", None)
        if plc_cfg is None:
            self.warning.emit("PLC-Konfiguration fehlt in ctx (ctx.plc ist None).")
            self._log.error("SetupPLC: ctx.plc fehlt")
            self.plc = None
            return

        if getattr(plc_cfg, "sim", False):
            self._log.info("PLC im Simulationsmodus (sim=true) -> keine ADS-Verbindung.")
            self.plc = None
            return

        try:
            self.plc = create_plc_client(plc_cfg)
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
        Start RosBridge for shadow + live only if ROS is enabled/launched.

        Expects:
          ctx.ros.shadow.enabled
          ctx.ros.live.enabled
        """
        if self.ctx is None:
            self.warning.emit("Bridge-Setup übersprungen: ctx ist None.")
            self._log.error("ConnectBridge: ctx ist None")
            self.bridge_shadow = None
            self.bridge_live = None
            return

        if not getattr(self.ctx.ros, "launch_ros", False):
            self._log.info("ROS launch disabled -> Bridge-Verbindungen übersprungen")
            self.bridge_shadow = None
            self.bridge_live = None
            return

        os.environ.setdefault("FASTDDS_SHM_TRANSPORT_DISABLE", "1")

        self.bridge_shadow = None
        self.bridge_live = None

        # --- SHADOW ---
        try:
            shadow_cfg = getattr(self.ctx.ros, "shadow", None)
            shadow_enabled = bool(getattr(shadow_cfg, "enabled", False)) if shadow_cfg else False
            if shadow_enabled:
                b = RosBridge(ctx=self.ctx, namespace="shadow")
                b.start()
                self.bridge_shadow = b
                logging.getLogger("ros").info("RosBridge SHADOW gestartet (namespace=shadow)")
            else:
                self._log.info("ROS shadow-role disabled -> keine Shadow-Bridge.")
        except Exception as e:
            self.warning.emit(f"RosBridge SHADOW nicht gestartet: {e}")
            self._log.exception("ConnectBridge[shadow] fehlgeschlagen")
            self.bridge_shadow = None

        # --- LIVE ---
        try:
            live_cfg = getattr(self.ctx.ros, "live", None)
            live_enabled = bool(getattr(live_cfg, "enabled", False)) if live_cfg else False
            if live_enabled:
                b = RosBridge(ctx=self.ctx, namespace="live")
                b.start()
                self.bridge_live = b
                logging.getLogger("ros").info("RosBridge LIVE gestartet (namespace=live)")
            else:
                self._log.info("ROS live-role disabled -> keine Live-Bridge.")
        except Exception as e:
            self.warning.emit(f"RosBridge LIVE nicht gestartet: {e}")
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
        self.ready.emit(self.ctx, self.bridge_shadow, self.bridge_live, self.plc)
