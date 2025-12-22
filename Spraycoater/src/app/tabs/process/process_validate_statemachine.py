# -*- coding: utf-8 -*-
# File: tabs/process/process_validate_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, List, Callable

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from geometry_msgs.msg import PoseStamped, PoseArray

_LOG = logging.getLogger("app.tabs.process.validate_statemachine")

# Einheitliche State-Namen (passen zu ProcessTab.pretty-map + Run-StateMachine)
STATE_MOVE_PREDISPENSE = "MOVE_PREDISPENSE"
STATE_MOVE_RECIPE = "MOVE_RECIPE"
STATE_MOVE_RETREAT = "MOVE_RETREAT"
STATE_MOVE_HOME = "MOVE_HOME"


class _QtSignalHandler(logging.Handler):
    """Logging → Qt-Signal-Weiterleitung (für ProcessTab-Log)."""

    def __init__(self, owner: "ProcessValidateStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        msg = self.format(record)
        self._owner.logMessage.emit(msg)


class ProcessValidateStatemachine(QtCore.QObject):
    """
    Validate-StateMachine (war Setup):

    - nimmt die SprayPath-Posen aus RosBridge.spraypath.poses()
    - fährt sie via MoveItPy ab: moveit_move_to_pose() / moveit_move_home()
    - Fortschritt basiert auf moveit_py/motion_result (EXECUTED:OK / ERROR:...)
    """

    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    _sig_predispense_done = QtCore.pyqtSignal()
    _sig_recipe_done = QtCore.pyqtSignal()
    _sig_retreat_done = QtCore.pyqtSignal()
    _sig_home_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        **_ignored_kwargs: Any,
    ) -> None:
        super().__init__(parent)

        self._recipe = recipe
        self._ros = ros

        self._stop_requested: bool = False
        self._stopped: bool = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStateMachine] = None

        self._poses: List[PoseStamped] = []
        self._recipe_index: int = 0

        self._max_retries: int = 3
        self._retry_count: int = 0

        self._current_state_name: str = ""

        # MoveItPy-Bridge (SSoT)
        self._moveitpy_bridge = self._ros.moveitpy_bridge
        self._moveitpy_signals = self._moveitpy_bridge.signals

        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        _LOG.addHandler(self._log_handler)

        _LOG.info(
            "ProcessValidateStatemachine init: recipe=%r, ros=%s, moveitpy=%s",
            self._recipe,
            type(self._ros).__name__,
            type(self._moveitpy_bridge).__name__,
        )

        self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        _LOG.info("ProcessValidateStatemachine.start")

        self._stop_requested = False
        self._stopped = False
        self._error_msg = None
        self._retry_count = 0
        self._recipe_index = 0
        self._current_state_name = ""

        pa: Optional[PoseArray] = self._ros.spraypath.poses()
        if pa is None or not pa.poses:
            msg = "Keine Recipe-Posen vorhanden (spraypath.poses ist leer)."
            _LOG.error(msg)
            self.notifyError.emit(msg)
            return

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

        _LOG.info("Validate: %d Posen geladen (frame=%s).", len(self._poses), frame)

        machine = QStateMachine(self)
        self._machine = machine

        s_predisp = QState(machine)
        s_recipe = QState(machine)
        s_retreat = QState(machine)
        s_home = QState(machine)
        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        machine.setInitialState(s_predisp)

        s_predisp.addTransition(self._sig_predispense_done, s_recipe)
        s_recipe.addTransition(self._sig_recipe_done, s_retreat)
        s_retreat.addTransition(self._sig_retreat_done, s_home)
        s_home.addTransition(self._sig_home_done, s_finished)

        for st in (s_predisp, s_recipe, s_retreat, s_home):
            st.addTransition(self._sig_error, s_error)

        s_predisp.entered.connect(self._on_state_predispense)
        s_recipe.entered.connect(self._on_state_recipe)
        s_retreat.entered.connect(self._on_state_retreat)
        s_home.entered.connect(self._on_state_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        s_predisp.entered.connect(lambda: self._emit_state(STATE_MOVE_PREDISPENSE))
        s_recipe.entered.connect(lambda: self._emit_state(STATE_MOVE_RECIPE))
        s_retreat.entered.connect(lambda: self._emit_state(STATE_MOVE_RETREAT))
        s_home.entered.connect(lambda: self._emit_state(STATE_MOVE_HOME))

        machine.start()
        _LOG.info("Validate: StateMachine gestartet.")

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        _LOG.info("Validate: request_stop()")
        self._stop_requested = True
        self._stopped = True

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        """
        Zentraler Stop:
          - Stop-Flag setzen
          - MoveIt stoppen (moveit_py/stop)
          - StateMachine weiterkicken, damit sie sauber zu FINISHED kommt
        """
        _LOG.info("Validate: stop()")
        self.request_stop()

        self._ros.moveit_stop()

        if self._machine is not None and self._machine.isRunning():
            QtCore.QTimer.singleShot(0, self._emit_done_for_current_state)
        else:
            self.notifyFinished.emit({"status": "stopped", "recipe": self._recipe})
            self._cleanup()

    # ------------------------------------------------------------------ #
    # Internals
    # ------------------------------------------------------------------ #

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _emit_state(self, name: str) -> None:
        self._current_state_name = name
        _LOG.info("Validate: State=%s", name)
        self.stateChanged.emit(name)

    def _signal_error(self, msg: str) -> None:
        if not msg:
            msg = "Unbekannter Validate-Fehler."
        if self._error_msg is None:
            self._error_msg = msg
        _LOG.error("Validate: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _finish_state(self, success_emit: Callable[[], None]) -> None:
        if self._should_stop():
            self._stopped = True

        if self._error_msg:
            self._signal_error(self._error_msg)
        else:
            QtCore.QTimer.singleShot(0, success_emit)

    def _emit_done_for_current_state(self) -> None:
        st = self._current_state_name or STATE_MOVE_PREDISPENSE
        _LOG.info("Validate: stop-kick für State=%s", st)

        if st == STATE_MOVE_PREDISPENSE:
            self._sig_predispense_done.emit()
        elif st == STATE_MOVE_RECIPE:
            self._sig_recipe_done.emit()
        elif st == STATE_MOVE_RETREAT:
            self._sig_retreat_done.emit()
        elif st == STATE_MOVE_HOME:
            self._sig_home_done.emit()
        else:
            self._sig_home_done.emit()

    # ------------------------------------------------------------------ #
    # State handlers
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_state_predispense(self) -> None:
        _LOG.info("Validate: ENTER %s", STATE_MOVE_PREDISPENSE)

        if not self._poses:
            self._signal_error("Keine Posen für Predispense vorhanden.")
            return

        if self._should_stop():
            self._finish_state(self._sig_predispense_done.emit)
            return

        self._recipe_index = 0
        self._retry_count = 0
        self._ros.moveit_move_to_pose(self._poses[0])

    @QtCore.pyqtSlot()
    def _on_state_recipe(self) -> None:
        _LOG.info("Validate: ENTER %s", STATE_MOVE_RECIPE)

        if len(self._poses) <= 1:
            _LOG.info("Validate: weniger als 2 Posen – überspringe RECIPE.")
            self._finish_state(self._sig_recipe_done.emit)
            return

        if self._should_stop():
            self._finish_state(self._sig_recipe_done.emit)
            return

        self._recipe_index = 1
        self._start_next_recipe_pose()

    def _start_next_recipe_pose(self) -> None:
        if self._should_stop():
            self._finish_state(self._sig_recipe_done.emit)
            return

        if self._recipe_index >= len(self._poses) - 1:
            _LOG.info("Validate: alle Zwischen-Posen gefahren.")
            self._finish_state(self._sig_recipe_done.emit)
            return

        self._retry_count = 0
        self._ros.moveit_move_to_pose(self._poses[self._recipe_index])

    @QtCore.pyqtSlot()
    def _on_state_retreat(self) -> None:
        _LOG.info("Validate: ENTER %s", STATE_MOVE_RETREAT)

        if len(self._poses) < 2:
            _LOG.info("Validate: keine Retreat-Pose – überspringe.")
            self._finish_state(self._sig_retreat_done.emit)
            return

        if self._should_stop():
            self._finish_state(self._sig_retreat_done.emit)
            return

        self._retry_count = 0
        self._ros.moveit_move_to_pose(self._poses[-1])

    @QtCore.pyqtSlot()
    def _on_state_home(self) -> None:
        _LOG.info("Validate: ENTER %s", STATE_MOVE_HOME)

        if self._should_stop():
            self._finish_state(self._sig_home_done.emit)
            return

        self._retry_count = 0
        self._ros.moveit_move_home()

    # ------------------------------------------------------------------ #
    # Motion result handler
    # ------------------------------------------------------------------ #

    def _handle_recipe_error_with_retry(self, result: str) -> None:
        if self._retry_count < self._max_retries:
            self._retry_count += 1
            _LOG.warning(
                "Validate: RECIPE Retry %d/%d (idx=%d) wegen %s",
                self._retry_count,
                self._max_retries,
                self._recipe_index,
                result,
            )
            self._ros.moveit_move_to_pose(self._poses[self._recipe_index])
        else:
            self._signal_error(result)

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        _LOG.info("Validate: motion_result=%s (state=%s)", result, self._current_state_name)

        if self._machine is None or not self._machine.isRunning():
            return

        if self._should_stop():
            return

        if result.startswith("ERROR"):
            if self._current_state_name == STATE_MOVE_RECIPE:
                self._handle_recipe_error_with_retry(result)
            else:
                self._signal_error(result)
            return

        if not result.startswith("EXECUTED:OK"):
            return

        if self._current_state_name == STATE_MOVE_PREDISPENSE:
            self._finish_state(self._sig_predispense_done.emit)

        elif self._current_state_name == STATE_MOVE_RECIPE:
            self._retry_count = 0
            self._recipe_index += 1
            self._start_next_recipe_pose()

        elif self._current_state_name == STATE_MOVE_RETREAT:
            self._finish_state(self._sig_retreat_done.emit)

        elif self._current_state_name == STATE_MOVE_HOME:
            self._finish_state(self._sig_home_done.emit)

    # ------------------------------------------------------------------ #
    # Final / Error
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_state_error(self) -> None:
        _LOG.info("Validate: ENTER ERROR")
        msg = self._error_msg or "Unbekannter Fehler."
        self.notifyError.emit(msg)
        self._cleanup()

    @QtCore.pyqtSlot()
    def _on_state_finished(self) -> None:
        _LOG.info("Validate: ENTER FINISHED")
        status = "stopped" if self._stopped else "finished"

        # Du bekommst hier direkt die letzten MoveIt-Trajs aus dem RosBridge-State.
        self.notifyFinished.emit(
            {
                "poses": [],
                "planned_traj": self._ros.moveit_planned_trajectory(),
                "executed_traj": self._ros.moveit_executed_trajectory(),
                "status": status,
                "recipe": self._recipe,
            }
        )
        self._cleanup()

    # ------------------------------------------------------------------ #
    # Cleanup
    # ------------------------------------------------------------------ #

    def _cleanup(self) -> None:
        try:
            self._moveitpy_signals.motionResultChanged.disconnect(self._on_motion_result)
        except Exception:
            pass

        m = self._machine
        if m is not None and m.isRunning():
            try:
                m.stop()
            except Exception:
                _LOG.exception("Validate: Fehler beim Stoppen der StateMachine.")
        if m is not None:
            m.deleteLater()
        self._machine = None

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None
