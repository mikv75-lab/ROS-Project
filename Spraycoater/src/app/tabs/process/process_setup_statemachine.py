# -*- coding: utf-8 -*-
# File: app/tabs/process/process_setup_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, List, Dict, Callable

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from geometry_msgs.msg import PoseStamped, PoseArray

_LOG = logging.getLogger("app.tabs.process.setup_statemachine")


# Zustandsnamen, wie sie auch an die UI gemeldet werden
STATE_PREDISPENSE = "PREDISPENSE"
STATE_RECIPE = "RECIPE"
STATE_RETREAT = "RETREAT"
STATE_HOME = "HOME"


class _QtSignalHandler(logging.Handler):
    # Leitet Log-Messages als Qt-Signal in die UI weiter.
    """Logging → Qt-Signal-Weiterleitung (für ProcessTab-Log)."""

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
            pass


class ProcessSetupStatemachine(QtCore.QObject):
    # Ergebnis/Fehler an den ProcessThread/Tab
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    # UI-Hilfssignale (Status + Log)
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # interne Transition-Signale (treiben die Qt-StateMachine weiter)
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
        super().__init__(parent)

        self._recipe = recipe
        self._ui = ui_bridge

        # Laufzeitflags/Zustand
        self._stop_requested: bool = False
        self._stopped: bool = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStateMachine] = None

        # Pose-Liste aus spraypath (PoseArray -> PoseStamped)
        self._poses: List[PoseStamped] = []
        self._recipe_index: int = 0

        # Retry-Logik nur für den RECIPE-State (bei Motion-ERROR)
        self._max_retries: int = 3
        self._retry_count: int = 0

        # Merkt sich den aktuellen UI-State-Namen für Motion-Result-Handling
        self._current_state_name: str = ""

        # MotionBridge-Signale kommen über die UIBridge
        self._motion_bridge = getattr(self._ui, "_motion", None)
        self._motion_signals = getattr(self._motion_bridge, "signals", None) if self._motion_bridge else None

        # Logging → Qt-UI
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        _LOG.addHandler(self._log_handler)

        _LOG.info(
            "ProcessSetupStatemachine init: recipe=%r, ui_bridge=%s, motion=%s",
            self._recipe,
            type(self._ui).__name__,
            type(self._motion_bridge).__name__ if self._motion_bridge is not None else "None",
        )

        # Bindet an motionResultChanged an, um State-Fortschritt über Motion-Feedback zu steuern
        if self._motion_signals is not None:
            try:
                self._motion_signals.motionResultChanged.connect(self._on_motion_result)  # type: ignore[attr-defined]
                _LOG.info("ProcessSetupStatemachine: motion_signals verbunden.")
            except Exception as ex:
                _LOG.warning("ProcessSetupStatemachine: motion_signals konnten nicht verbunden werden: %r", ex)

    # ------------------------------------------------------------------ #
    # Öffentliche API
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        # Lädt die SprayPath-Posen und startet die Qt-StateMachine.
        _LOG.info("ProcessSetupStatemachine.start: gestartet.")

        self._stop_requested = False
        self._stopped = False
        self._error_msg = None
        self._retry_count = 0

        # SprayPath aus der UI holen
        pa: Optional[PoseArray] = self._ui.spraypath.poses()
        if pa is None or not pa.poses:
            msg = "Keine Recipe-Posen vorhanden (spraypath.poses ist leer)."
            _LOG.error("ProcessSetupStatemachine.start: %s", msg)
            self.notifyError.emit(msg)
            return

        # Posen übernehmen (Position 1:1, Orientation wird auf Identität gesetzt)
        frame = pa.header.frame_id or "scene"
        self._poses = []

        for p in pa.poses:
            ps = PoseStamped()
            ps.header.frame_id = frame

            ps.pose.position.x = float(p.position.x)
            ps.pose.position.y = float(p.position.y)
            ps.pose.position.z = float(p.position.z)

            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            self._poses.append(ps)

        _LOG.info(
            "ProcessSetupStatemachine.start: %d Posen geladen (frame=%s, quat=(0,0,0,1)).",
            len(self._poses),
            frame,
        )

        # Qt-StateMachine aufbauen (4 Schritte + finished/error)
        machine = QStateMachine(self)
        self._machine = machine

        s_predisp = QState(machine)
        s_recipe = QState(machine)
        s_retreat = QState(machine)
        s_home = QState(machine)
        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        machine.setInitialState(s_predisp)

        # Übergänge: linearer Ablauf, Fehler führt jederzeit in ERROR
        s_predisp.addTransition(self._sig_predispense_done, s_recipe)
        s_recipe.addTransition(self._sig_recipe_done, s_retreat)
        s_retreat.addTransition(self._sig_retreat_done, s_home)
        s_home.addTransition(self._sig_home_done, s_finished)

        for st in (s_predisp, s_recipe, s_retreat, s_home):
            st.addTransition(self._sig_error, s_error)

        # State-Callbacks: beim Eintritt wird ein Motion-Kommando abgesetzt
        s_predisp.entered.connect(self._on_state_predispense)
        s_recipe.entered.connect(self._on_state_recipe)
        s_retreat.entered.connect(self._on_state_retreat)
        s_home.entered.connect(self._on_state_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        # UI-Statusausgabe (nur Namen)
        s_predisp.entered.connect(lambda: self._emit_state(STATE_PREDISPENSE))
        s_recipe.entered.connect(lambda: self._emit_state(STATE_RECIPE))
        s_retreat.entered.connect(lambda: self._emit_state(STATE_RETREAT))
        s_home.entered.connect(lambda: self._emit_state(STATE_HOME))

        try:
            machine.start()
            _LOG.info("ProcessSetupStatemachine: StateMachine gestartet.")
        except Exception as e:
            _LOG.exception("ProcessSetupStatemachine.start: Exception: %s", e)
            self._signal_error(str(e) or "Fehler beim Start der Setup-StateMachine.")

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        # Markiert den Prozess als "stop requested". Die nächsten States werden übersprungen.
        _LOG.info("ProcessSetupStatemachine: request_stop()")
        self._stop_requested = True
        self._stopped = True

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        """
        Stoppt den Ablauf UI-seitig:
          - setzt Stop-Flags
          - ruft optional motion_stop()
          - kickt die StateMachine in den nächsten State, damit sie bis FINISHED durchläuft
        """
        _LOG.info("ProcessSetupStatemachine: stop()")
        self.request_stop()

        # Optional: Motion sofort abbrechen (best effort)
        try:
            if hasattr(self._ui, "motion_stop"):
                self._ui.motion_stop()
        except Exception:
            pass

        # Falls die Maschine noch läuft: aktuellen State "weiterkicken"
        if self._machine is not None and self._machine.isRunning():
            QtCore.QTimer.singleShot(0, self._emit_done_for_current_state)
        else:
            # Wenn nichts läuft: direkt als stopped "fertig"
            self.notifyFinished.emit({"status": "stopped", "recipe": self._recipe})
            self._cleanup()

    # ------------------------------------------------------------------ #
    # Hilfsfunktionen
    # ------------------------------------------------------------------ #

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _emit_state(self, name: str) -> None:
        # Merkt sich den State für Motion-Result-Auswertung und informiert die UI
        self._current_state_name = name
        _LOG.info("ProcessSetupStatemachine: State=%s", name)
        try:
            self.stateChanged.emit(name)
        except Exception:
            pass

    def _signal_error(self, msg: str) -> None:
        # Merkt sich die Fehlermeldung und triggert den ERROR-Übergang
        if not msg:
            msg = "Unbekannter Setup-Fehler."
        if not self._error_msg:
            self._error_msg = msg
        _LOG.error("ProcessSetupStatemachine: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _finish_state(self, success_emit: Callable[[], None]) -> None:
        """
        Abschluss eines States:
          - bei Stop: markiert stopped und geht einfach weiter
          - bei Error: ERROR-State
          - sonst: weiter zum nächsten State
        """
        if self._should_stop():
            self._stopped = True

        if self._error_msg:
            self._signal_error(self._error_msg)
        else:
            QtCore.QTimer.singleShot(0, success_emit)

    def _emit_done_for_current_state(self) -> None:
        # Treibt die StateMachine weiter, wenn wir gerade "auf Motion warten"
        st = self._current_state_name or STATE_PREDISPENSE
        _LOG.info("ProcessSetupStatemachine: stop-kick für State=%s", st)

        if st == STATE_PREDISPENSE:
            self._sig_predispense_done.emit()
        elif st == STATE_RECIPE:
            self._sig_recipe_done.emit()
        elif st == STATE_RETREAT:
            self._sig_retreat_done.emit()
        elif st == STATE_HOME:
            self._sig_home_done.emit()
        else:
            # Fallback: Richtung FINISHED treiben
            self._sig_home_done.emit()

    # ------------------------------------------------------------------ #
    # State-Handler
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_state_predispense(self) -> None:
        # Fährt die erste Pose an
        _LOG.info("ProcessSetupStatemachine: ENTER %s", STATE_PREDISPENSE)

        if not self._poses:
            self._signal_error("Keine Posen für PREDISPENSE vorhanden.")
            return

        if self._should_stop():
            self._finish_state(self._sig_predispense_done.emit)
            return

        pose = self._poses[0]
        self._retry_count = 0
        _LOG.info(
            "ProcessSetupStatemachine: %s -> motion_move_to_pose (idx=0, frame=%s, pos=(%.3f, %.3f, %.3f))",
            STATE_PREDISPENSE,
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        self._ui.motion_move_to_pose(pose)

    @QtCore.pyqtSlot()
    def _on_state_recipe(self) -> None:
        # Fährt alle mittleren Posen nacheinander (idx 1..N-2)
        _LOG.info("ProcessSetupStatemachine: ENTER %s", STATE_RECIPE)

        if len(self._poses) <= 1:
            _LOG.info("ProcessSetupStatemachine: %s: weniger als 2 Posen – überspringe.", STATE_RECIPE)
            self._finish_state(self._sig_recipe_done.emit)
            return

        if self._should_stop():
            self._finish_state(self._sig_recipe_done.emit)
            return

        self._recipe_index = 1
        self._start_next_recipe_pose()

    def _start_next_recipe_pose(self) -> None:
        # Startet den nächsten Pose-Schritt im RECIPE-State
        if self._should_stop():
            self._finish_state(self._sig_recipe_done.emit)
            return

        if self._recipe_index >= len(self._poses) - 1:
            _LOG.info(
                "ProcessSetupStatemachine: %s: alle Zwischen-Posen gefahren (bis idx=%d).",
                STATE_RECIPE,
                self._recipe_index - 1,
            )
            self._finish_state(self._sig_recipe_done.emit)
            return

        pose = self._poses[self._recipe_index]
        self._retry_count = 0
        _LOG.info(
            "ProcessSetupStatemachine: %s -> motion_move_to_pose (idx=%d/%d, frame=%s, pos=(%.3f, %.3f, %.3f))",
            STATE_RECIPE,
            self._recipe_index,
            len(self._poses) - 1,
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        self._ui.motion_move_to_pose(pose)

    @QtCore.pyqtSlot()
    def _on_state_retreat(self) -> None:
        # Fährt die letzte Pose als Retreat an
        _LOG.info("ProcessSetupStatemachine: ENTER %s", STATE_RETREAT)

        if len(self._poses) < 2:
            _LOG.info("ProcessSetupStatemachine: %s: keine getrennte Retreat-Pose – überspringe.", STATE_RETREAT)
            self._finish_state(self._sig_retreat_done.emit)
            return

        if self._should_stop():
            self._finish_state(self._sig_retreat_done.emit)
            return

        pose = self._poses[-1]
        self._retry_count = 0
        _LOG.info(
            "ProcessSetupStatemachine: %s -> motion_move_to_pose (idx=%d, frame=%s, pos=(%.3f, %.3f, %.3f))",
            STATE_RETREAT,
            len(self._poses) - 1,
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        self._ui.motion_move_to_pose(pose)

    @QtCore.pyqtSlot()
    def _on_state_home(self) -> None:
        # Fährt Home über die UIBridge
        _LOG.info("ProcessSetupStatemachine: ENTER %s", STATE_HOME)

        if self._should_stop():
            self._finish_state(self._sig_home_done.emit)
            return

        self._retry_count = 0
        _LOG.info("ProcessSetupStatemachine: %s -> motion_move_home()", STATE_HOME)
        self._ui.motion_move_home()

    # ------------------------------------------------------------------ #
    # Motion-Ergebnis-Handler
    # ------------------------------------------------------------------ #

    def _log_recipe_pose_state(self, prefix: str, result: str) -> None:
        # Logging-Helfer, um bei RECIPE die Zielpose mit auszugeben
        pose: Optional[PoseStamped] = None
        if 0 <= self._recipe_index < len(self._poses):
            pose = self._poses[self._recipe_index]

        if pose is None:
            _LOG.error("ProcessSetupStatemachine: %s: idx=%d %s%s", STATE_RECIPE, self._recipe_index, prefix, result)
            return

        _LOG.error(
            "ProcessSetupStatemachine: %s: idx=%d %s%s (frame=%s, pos=(%.3f, %.3f, %.3f))",
            STATE_RECIPE,
            self._recipe_index,
            prefix,
            result,
            pose.header.frame_id,
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )

    def _handle_recipe_error_with_retry(self, result: str) -> None:
        # Bei ERROR im RECIPE-State: gleiche Pose bis zu _max_retries erneut anfahren
        self._log_recipe_pose_state("ERROR:", result)

        if not (0 <= self._recipe_index < len(self._poses)):
            self._signal_error(result)
            return

        if self._retry_count < self._max_retries:
            self._retry_count += 1
            pose = self._poses[self._recipe_index]
            _LOG.warning(
                "ProcessSetupStatemachine: %s: Retry %d/%d (idx=%d) wegen %s",
                STATE_RECIPE,
                self._retry_count,
                self._max_retries,
                self._recipe_index,
                result,
            )
            self._ui.motion_move_to_pose(pose)
        else:
            _LOG.error(
                "ProcessSetupStatemachine: %s: maximale Retries (%d) erreicht (idx=%d).",
                STATE_RECIPE,
                self._max_retries,
                self._recipe_index,
            )
            self._signal_error(result)

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        # Reagiert auf Motion-Ergebnis und treibt je nach State den Ablauf weiter
        _LOG.info("ProcessSetupStatemachine: motion_result=%s (state=%s)", result, self._current_state_name)

        if not self._machine or not self._machine.isRunning():
            _LOG.debug("ProcessSetupStatemachine: motion_result ignoriert – Maschine läuft nicht.")
            return

        # Wenn gestoppt: Motion-Results ignorieren (StateMachine wird per stop-kick weitergetrieben)
        if self._should_stop():
            return

        if result.startswith("ERROR"):
            if self._current_state_name == STATE_RECIPE:
                self._handle_recipe_error_with_retry(result)
            else:
                self._signal_error(result)
            return

        # Nur EXECUTED:OK wird als "fertig" gewertet
        if not result.startswith("EXECUTED:OK"):
            return

        if self._current_state_name == STATE_PREDISPENSE:
            self._finish_state(self._sig_predispense_done.emit)

        elif self._current_state_name == STATE_RECIPE:
            self._retry_count = 0
            self._recipe_index += 1
            self._start_next_recipe_pose()

        elif self._current_state_name == STATE_RETREAT:
            self._finish_state(self._sig_retreat_done.emit)

        elif self._current_state_name == STATE_HOME:
            self._finish_state(self._sig_home_done.emit)

    # ------------------------------------------------------------------ #
    # Final / Error
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_state_error(self) -> None:
        # Fehlerpfad: notifyError + cleanup
        _LOG.info("ProcessSetupStatemachine: ENTER ERROR")
        msg = self._error_msg or "Unbekannter Fehler."
        self.notifyError.emit(msg)
        self._cleanup()

    @QtCore.pyqtSlot()
    def _on_state_finished(self) -> None:
        # Erfolgsfall: notifyFinished + cleanup (Status kann finished/stopped sein)
        _LOG.info("ProcessSetupStatemachine: ENTER FINISHED")
        status = "stopped" if self._stopped else "finished"
        self.notifyFinished.emit(
            {
                "poses": [],
                "planned_traj": [],
                "executed_traj": [],
                "status": status,
                "recipe": self._recipe,
            }
        )
        self._cleanup()

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def _cleanup(self) -> None:
        # Löst Signalverbindungen, stoppt die Maschine und entfernt den Log-Handler
        if self._motion_signals is not None:
            try:
                self._motion_signals.motionResultChanged.disconnect(self._on_motion_result)  # type: ignore[attr-defined]
            except Exception:
                pass

        m = self._machine
        if m is not None and m.isRunning():
            try:
                _LOG.info("ProcessSetupStatemachine: stoppe StateMachine.")
                m.stop()
            except Exception:
                _LOG.exception("ProcessSetupStatemachine: Fehler beim Stoppen der StateMachine.")
        if m is not None:
            try:
                m.deleteLater()
            except Exception:
                pass
        self._machine = None

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None
