# -*- coding: utf-8 -*-
# File: tabs/process/base_statemachine.py
from __future__ import annotations

import logging
from typing import Optional, Dict, Any, Callable

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

_LOG = logging.getLogger("app.tabs.process.base_statemachine")

STATE_MOVE_PREDISPENSE = "MOVE_PREDISPENSE"
STATE_MOVE_RECIPE = "MOVE_RECIPE"
STATE_MOVE_RETREAT = "MOVE_RETREAT"
STATE_MOVE_HOME = "MOVE_HOME"

SEG_ORDER = [STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME]


class _QtSignalHandler(logging.Handler):
    """Logging → Qt-Signal (für ProcessTab-Log)."""

    def __init__(self, owner: "BaseProcessStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            self._owner.logMessage.emit(msg)
        except Exception:
            pass


class BaseProcessStatemachine(QtCore.QObject):
    """
    Base für Validate/Optimize/Execute:
      - identisches State-Gerüst (MOVE_* Segmente)
      - Stop/Retry/MotionResult-Handling zentral
      - Ergebnis: dict (planned_traj / executed_traj / meta)
    """

    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    _sig_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
    ) -> None:
        super().__init__(parent)
        self._recipe = recipe
        self._ros = ros
        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._stop_requested = False
        self._stopped = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0

        # MoveItPy signals (falls vorhanden)
        self._moveitpy_bridge = getattr(self._ros, "moveitpy", None) or getattr(self._ros, "moveitpy_bridge", None)
        self._moveitpy_signals = getattr(self._moveitpy_bridge, "signals", None)

        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        _LOG.addHandler(self._log_handler)

        if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
            try:
                self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)  # type: ignore[attr-defined]
            except Exception:
                pass

    # ---------------- Public API ----------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        """Startet die StateMachine."""
        self._stop_requested = False
        self._stopped = False
        self._error_msg = None
        self._retry_count = 0
        self._current_state = ""

        if not self._prepare_run():
            msg = self._error_msg or "Prepare failed"
            _LOG.error("Base: %s", msg)
            self.notifyError.emit(msg)
            self._cleanup()
            return

        self._machine = self._build_machine()
        self._machine.start()
        _LOG.info("Base: StateMachine gestartet (initial=%s).", self._first_segment_state() or "NONE")

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True
        self._stopped = True
        # MoveIt stoppen, falls vorhanden
        try:
            self._ros.moveit_stop()
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        self.request_stop()

    # ---------------- Hooks (override in derived) ----------------

    def _prepare_run(self) -> bool:
        """
        Derived lädt hier Daten (poses, run-yaml, trajectories, etc.)
        Muss self._error_msg setzen und False zurückgeben bei Fehler.
        """
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        """
        Derived führt hier den Segment-Start aus.
        Muss bei Fehler self._signal_error(...) rufen.
        """
        raise NotImplementedError

    def _on_segment_ok(self, seg_name: str) -> None:
        """Optional hook wenn Segment erfolgreich war."""
        pass

    def _build_result(self) -> Dict[str, Any]:
        """Derived kann hier Extra-Felder ergänzen."""
        return {
            "status": "stopped" if self._stopped else "finished",
            "planned_traj": self._safe_call("moveit_planned_trajectory"),
            "executed_traj": self._safe_call("moveit_executed_trajectory"),
            "recipe": self._recipe,
        }

    # ---------------- Machine builder ----------------

    def _build_machine(self) -> QStateMachine:
        m = QStateMachine(self)

        states: Dict[str, QState] = {name: QState(m) for name in SEG_ORDER}
        s_finished = QFinalState(m)
        s_error = QFinalState(m)

        first = self._first_segment_state()
        if first is None:
            self._error_msg = "Keine Segmente vorhanden."
            m.setInitialState(s_error)
        else:
            m.setInitialState(states[first])

        # linear transitions via _sig_done / _sig_error
        for i, name in enumerate(SEG_ORDER):
            st = states[name]
            st.addTransition(self._sig_done, states[SEG_ORDER[i + 1]] if i + 1 < len(SEG_ORDER) else s_finished)
            st.addTransition(self._sig_error, s_error)

        # entered handlers
        for name in SEG_ORDER:
            states[name].entered.connect(lambda n=name: self._on_state_enter(n))

        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        return m

    def _first_segment_state(self) -> Optional[str]:
        for name in SEG_ORDER:
            if self._skip_home and name == STATE_MOVE_HOME:
                continue
            if self._segment_exists(name):
                return name
        return None

    def _segment_exists(self, seg_name: str) -> bool:
        """Derived kann das überschreiben, default: alle existieren."""
        if self._skip_home and seg_name == STATE_MOVE_HOME:
            return False
        return True

    # ---------------- State enter / motion result ----------------

    def _emit_state(self, name: str) -> None:
        self._current_state = name
        self.stateChanged.emit(name)
        self.logMessage.emit(f"{type(self).__name__}: State={name}")

    def _on_state_enter(self, seg_name: str) -> None:
        self._emit_state(seg_name)

        if self._skip_home and seg_name == STATE_MOVE_HOME:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        if self._stop_requested:
            self._signal_error("Prozess durch Benutzer gestoppt.")
            return

        self._retry_count = 0
        self._on_enter_segment(seg_name)

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        if self._machine is None or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if not self._current_state:
            return

        _LOG.info("%s: motion_result=%s (state=%s)", type(self).__name__, result, self._current_state)

        if result.startswith("ERROR"):
            if self._retry_count < self._max_retries:
                self._retry_count += 1
                _LOG.warning("%s: Retry %d/%d für %s", type(self).__name__, self._retry_count, self._max_retries, self._current_state)
                self._on_enter_segment(self._current_state)
            else:
                self._signal_error(result)
            return

        if not result.startswith("EXECUTED:OK"):
            return

        self._on_segment_ok(self._current_state)
        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ---------------- Final ----------------

    def _signal_error(self, msg: str) -> None:
        if not msg:
            msg = "Unbekannter Fehler."
        if self._error_msg is None:
            self._error_msg = msg
        _LOG.error("%s: %s", type(self).__name__, msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    @QtCore.pyqtSlot()
    def _on_state_error(self) -> None:
        self.notifyError.emit(self._error_msg or "Unbekannter Fehler.")
        self._cleanup()

    @QtCore.pyqtSlot()
    def _on_state_finished(self) -> None:
        self.notifyFinished.emit(self._build_result())
        self._cleanup()

    # ---------------- Helpers / cleanup ----------------

    def _safe_call(self, name: str, *args, **kwargs):
        if hasattr(self._ros, name):
            try:
                return getattr(self._ros, name)(*args, **kwargs)
            except Exception:
                return None
        return None

    def _cleanup(self) -> None:
        try:
            if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
                self._moveitpy_signals.motionResultChanged.disconnect(self._on_motion_result)  # type: ignore[attr-defined]
        except Exception:
            pass

        m = self._machine
        if m is not None and m.isRunning():
            try:
                m.stop()
            except Exception:
                pass
        if m is not None:
            m.deleteLater()
        self._machine = None

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None
