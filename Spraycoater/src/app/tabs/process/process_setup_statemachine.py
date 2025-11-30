# -*- coding: utf-8 -*-
# File: app/tabs/process/process_setup_statemachine.py

from __future__ import annotations

import logging
from typing import Any, Optional, Dict, List

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from geometry_msgs.msg import PoseStamped, PoseArray

_LOG = logging.getLogger("app.tabs.process.setup_statemachine")


class _QtSignalHandler(logging.Handler):
    """
    Logging → Qt-Signal-Weiterleitung (für ProcessTab-Log).
    """
    def __init__(self, owner: "ProcessSetupStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:
            return
        try:
            self._owner.logMessage.emit(msg)
        except Exception:
            # beim Shutdown ggf. schon weg – ignorieren
            pass


class ProcessSetupStatemachine(QtCore.QObject):
    """
    StateMachine-Worker für den Setup-Prozess eines Rezepts.

    States:
      - PREDISPENSE  → erste Pose des SprayPath
      - RECIPE       → alle Posen der SprayPath-PoseArray nacheinander abfahren
      - RETREAT      → letzte Pose des SprayPath
      - HOME         → Roboter in Home-Position fahren
      - FINISHED     → Ende
      - ERROR        → Fehlerpfad

    Signale (kompatibel zu ProcessRunStatemachine):
      - notifyFinished(object)
      - notifyError(str)
      - stateChanged(str)
      - logMessage(str)
    """

    # Ergebnis / Fehler
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # interne Transition-Signale
    _sig_predispense_done = QtCore.pyqtSignal()
    _sig_recipe_done = QtCore.pyqtSignal()
    _sig_retreat_done = QtCore.pyqtSignal()
    _sig_home_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Any,
        ui_bridge: Any,
        parent: Optional[QtCore.QObject] = None,
        **_ignored_kwargs: Any,
    ) -> None:
        """
        Erwartet:
          - recipe: Rezept-Objekt/ID (für Logging/Result)
          - ui_bridge: Instanz von UIBridge (holt sich alles aus ihr)
        """
        super().__init__(parent)

        self._recipe = recipe
        self._ui = ui_bridge

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStateMachine] = None

        # SprayPath-Posen (aus ui_bridge.spraypath)
        self._poses: List[PoseStamped] = []
        self._recipe_index: int = 0

        # Aktueller State-Name (für Motion-Result-Handling)
        self._current_state_name: str = ""

        # MotionBridge-Signale via UIBridge
        self._motion_bridge = getattr(self._ui, "_motion", None)
        self._motion_signals = getattr(self._motion_bridge, "signals", None) if self._motion_bridge else None

        # Logging → Qt
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        formatter = logging.Formatter("%(message)s")
        self._log_handler.setFormatter(formatter)
        _LOG.addHandler(self._log_handler)

        _LOG.info(
            "ProcessSetupStatemachine init: recipe=%r, ui_bridge=%s, motion=%s",
            self._recipe,
            type(self._ui).__name__,
            type(self._motion_bridge).__name__ if self._motion_bridge is not None else "None",
        )

        if self._motion_signals is not None:
            try:
                self._motion_signals.motionResultChanged.connect(self._on_motion_result)  # type: ignore[attr-defined]
                _LOG.info("ProcessSetupStatemachine: motion_signals verbunden.")
            except Exception as ex:
                _LOG.warning("ProcessSetupStatemachine: konnte motion_signals nicht verbinden: %r", ex)

    # ------------------------------------------------------------------ #
    # Öffentliche API (vom ProcessThread aufzurufen)
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        """
        Startet den Setup-Prozess im Worker-Thread.
        """
        _LOG.info("ProcessSetupStatemachine.start: gestartet.")

        self._stop_requested = False
        self._error_msg = None

        # SprayPath-Posen aus UIBridge holen
        pa: Optional[PoseArray] = self._ui.spraypath.poses()
        if pa is None or not pa.poses:
            msg = "ProcessSetupStatemachine.start: keine Recipe-Posen vorhanden (spraypath.poses ist leer)."
            _LOG.error(msg)
            self.notifyError.emit(msg)
            return

        frame = pa.header.frame_id or "scene"
        self._poses = []

        # PoseArray -> Liste von PoseStamped
        # - Position: unverändert (wir nehmen an: schon in m)
        # - Orientierung: auf (0,0,0,1) setzen
        for idx, p in enumerate(pa.poses):
            ps = PoseStamped()
            ps.header.frame_id = frame

            # Position direkt übernehmen
            ps.pose.position.x = float(p.position.x)
            ps.pose.position.y = float(p.position.y)
            ps.pose.position.z = float(p.position.z)

            # Orientierung: RPY = 0 -> Quaternion (0,0,0,1)
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            self._poses.append(ps)

        _LOG.info(
            "ProcessSetupStatemachine.start: %d Posen aus SprayPath (%s) geladen "
            "(keine Umrechnung, quat=(0,0,0,1)).",
            len(self._poses),
            frame,
        )

        # StateMachine aufbauen
        machine = QStateMachine(self)
        self._machine = machine

        s_predisp = QState(machine)
        s_recipe = QState(machine)
        s_retreat = QState(machine)
        s_home = QState(machine)
        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        machine.setInitialState(s_predisp)

        # Transitions
        s_predisp.addTransition(self._sig_predispense_done, s_recipe)
        s_recipe.addTransition(self._sig_recipe_done, s_retreat)
        s_retreat.addTransition(self._sig_retreat_done, s_home)
        s_home.addTransition(self._sig_home_done, s_finished)

        for st in (s_predisp, s_recipe, s_retreat, s_home):
            st.addTransition(self._sig_error, s_error)

        # Callbacks
        s_predisp.entered.connect(self._on_state_predispense)
        s_recipe.entered.connect(self._on_state_recipe)
        s_retreat.entered.connect(self._on_state_retreat)
        s_home.entered.connect(self._on_state_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        # UI-State-Namen
        s_predisp.entered.connect(lambda: self._emit_state("PREDISPENSE"))
        s_recipe.entered.connect(lambda: self._emit_state("RECIPE"))
        s_retreat.entered.connect(lambda: self._emit_state("RETREAT"))
        s_home.entered.connect(lambda: self._emit_state("HOME"))

        try:
            machine.start()
            _LOG.info("ProcessSetupStatemachine: StateMachine gestartet.")
        except Exception as e:
            _LOG.exception("ProcessSetupStatemachine.start: Exception: %s", e)
            self._signal_error(str(e) or "Unbekannter Fehler beim Start der Setup-StateMachine")

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        Von außen aufgerufen (z. B. Stop-Button).
        """
        _LOG.info("ProcessSetupStatemachine: request_stop()")
        self._stop_requested = True

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        """
        Alternative Stop-API (falls ProcessThread stop() aufruft).
        """
        _LOG.info("ProcessSetupStatemachine: stop()")
        self._stop_requested = True
        self._cleanup()
        # Für den Caller ist „stop“ äquivalent zu „fertig“
        self.notifyFinished.emit({"status": "stopped"})

    # ------------------------------------------------------------------ #
    # Hilfsfunktionen
    # ------------------------------------------------------------------ #

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _emit_state(self, name: str) -> None:
        self._current_state_name = name
        _LOG.info("ProcessSetupStatemachine: State=%s", name)
        try:
            self.stateChanged.emit(name)
        except Exception:
            pass

    def _signal_error(self, msg: str) -> None:
        if not msg:
            msg = "Unbekannter Setup-Fehler."
        if not self._error_msg:
            self._error_msg = msg
        _LOG.error("ProcessSetupStatemachine: _signal_error: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _finish_state(self, success_sig: QtCore.pyqtSignal) -> None:
        """
        Gemeinsames Ende eines States:
          - Wenn Stop oder Fehler -> ERROR
          - sonst -> success_sig
        """
        if self._should_stop() and not self._error_msg:
            self._error_msg = "Setup durch Benutzer gestoppt."
        if self._error_msg:
            self._signal_error(self._error_msg)
        else:
            QtCore.QTimer.singleShot(0, success_sig.emit)

    # ------------------------------------------------------------------ #
    # State-Handler
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_state_predispense(self) -> None:
        _LOG.info("ProcessSetupStatemachine: ENTER PREDISPENSE")

        if not self._poses:
            self._signal_error("Keine Posen für PREDISPENSE vorhanden.")
            return

        if self._should_stop():
            self._finish_state(self._sig_predispense_done)
            return

        # Erste Pose anfahren
        pose = self._poses[0]
        _LOG.info(
            "ProcessSetupStatemachine: PREDISPENSE -> motion_move_to_pose (idx=0, frame=%s)",
            pose.header.frame_id,
        )
        self._ui.motion_move_to_pose(pose)

        # Weiter geht es in _on_motion_result() bei EXECUTED:OK

    @QtCore.pyqtSlot()
    def _on_state_recipe(self) -> None:
        _LOG.info("ProcessSetupStatemachine: ENTER RECIPE")

        if len(self._poses) <= 1:
            _LOG.info("ProcessSetupStatemachine: RECIPE: weniger als 2 Posen – überspringe.")
            self._finish_state(self._sig_recipe_done)
            return

        if self._should_stop():
            self._finish_state(self._sig_recipe_done)
            return

        # Posen 1..N-2 (also "mittlere" Posen) nacheinander fahren
        self._recipe_index = 1
        self._start_next_recipe_pose()

    def _start_next_recipe_pose(self) -> None:
        if self._should_stop():
            self._finish_state(self._sig_recipe_done)
            return

        if self._recipe_index >= len(self._poses) - 1:
            # letztes Segment der Posen machen wir in RETREAT
            _LOG.info(
                "ProcessSetupStatemachine: RECIPE: alle Zwischen-Posen gefahren (bis idx=%d).",
                self._recipe_index - 1,
            )
            self._finish_state(self._sig_recipe_done)
            return

        pose = self._poses[self._recipe_index]
        _LOG.info(
            "ProcessSetupStatemachine: RECIPE -> motion_move_to_pose (idx=%d/%d, frame=%s)",
            self._recipe_index,
            len(self._poses) - 1,
            pose.header.frame_id,
        )
        self._ui.motion_move_to_pose(pose)

        # Weiter geht es in _on_motion_result() bei EXECUTED:OK
        # dort wird _recipe_index erhöht und _start_next_recipe_pose() erneut aufgerufen.

    @QtCore.pyqtSlot()
    def _on_state_retreat(self) -> None:
        _LOG.info("ProcessSetupStatemachine: ENTER RETREAT")

        if len(self._poses) < 2:
            _LOG.info("ProcessSetupStatemachine: RETREAT: keine getrennte Retreat-Pose – überspringe.")
            self._finish_state(self._sig_retreat_done)
            return

        if self._should_stop():
            self._finish_state(self._sig_retreat_done)
            return

        # Letzte Pose als Retreat
        pose = self._poses[-1]
        _LOG.info(
            "ProcessSetupStatemachine: RETREAT -> motion_move_to_pose (idx=%d, frame=%s)",
            len(self._poses) - 1,
            pose.header.frame_id,
        )
        self._ui.motion_move_to_pose(pose)
        # Weiter geht es in _on_motion_result()

    @QtCore.pyqtSlot()
    def _on_state_home(self) -> None:
        _LOG.info("ProcessSetupStatemachine: ENTER HOME")

        if self._should_stop():
            self._finish_state(self._sig_home_done)
            return

        _LOG.info("ProcessSetupStatemachine: HOME -> motion_move_home()")
        self._ui.motion_move_home()
        # Weiter geht es in _on_motion_result()

    # ------------------------------------------------------------------ #
    # Motion-Ergebnis-Handler
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        """
        Callback für Motion-Ergebnis (von MotionBridge.signals.motionResultChanged).

        Nutzt den aktuellen State-Namen, um zu entscheiden,
        ob ein Schritt fertig ist oder der nächste Pose-Schritt gestartet wird.
        """
        _LOG.info("ProcessSetupStatemachine: motion_result=%s (state=%s)",
                  result, self._current_state_name)

        if not self._machine or not self._machine.isRunning():
            _LOG.debug("ProcessSetupStatemachine: motion_result ignoriert – Maschine läuft nicht.")
            return

        if result.startswith("ERROR"):
            self._signal_error(result)
            return

        if not result.startswith("EXECUTED:OK"):
            # andere Result-Texte ignorieren
            return

        # Nur EXECUTED:OK relevant
        if self._current_state_name == "PREDISPENSE":
            _LOG.info("ProcessSetupStatemachine: PREDISPENSE abgeschlossen (EXECUTED:OK).")
            self._finish_state(self._sig_predispense_done)

        elif self._current_state_name == "RECIPE":
            _LOG.info(
                "ProcessSetupStatemachine: RECIPE: Pose idx=%d EXECUTED:OK.",
                self._recipe_index,
            )
            self._recipe_index += 1
            self._start_next_recipe_pose()

        elif self._current_state_name == "RETREAT":
            _LOG.info("ProcessSetupStatemachine: RETREAT abgeschlossen (EXECUTED:OK).")
            self._finish_state(self._sig_retreat_done)

        elif self._current_state_name == "HOME":
            _LOG.info("ProcessSetupStatemachine: HOME abgeschlossen (EXECUTED:OK).")
            self._finish_state(self._sig_home_done)

    # ------------------------------------------------------------------ #
    # Final / Error
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_state_error(self) -> None:
        _LOG.info("ProcessSetupStatemachine: ENTER ERROR")
        msg = self._error_msg or "Unbekannter Fehler im Setup."
        _LOG.error("ProcessSetupStatemachine: notifyError(%s)", msg)
        self.notifyError.emit(msg)
        self._cleanup()

    @QtCore.pyqtSlot()
    def _on_state_finished(self) -> None:
        _LOG.info("ProcessSetupStatemachine: ENTER FINISHED")

        result: Dict[str, Any] = {
            # wie bei ProcessRunStatemachine – aktuell ohne echte Pose/Sampling
            "poses": [],          # kannst du später mit TCP-Log füllen
            "planned_traj": [],   # für Setup nicht aus MotionState gezogen
            "executed_traj": [],
            "status": "finished",
            "recipe": self._recipe,
        }
        self.notifyFinished.emit(result)
        self._cleanup()

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def _cleanup(self) -> None:
        m = self._machine
        if m is not None and m.isRunning():
            try:
                _LOG.info("ProcessSetupStatemachine: stoppe StateMachine.")
                m.stop()
            except Exception:
                _LOG.exception("ProcessSetupStatemachine: Fehler beim Stoppen der StateMachine.")
        self._machine = None

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None
