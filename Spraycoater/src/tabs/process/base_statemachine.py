# -*- coding: utf-8 -*-
# File: tabs/process/base_statemachine.py
from __future__ import annotations

import logging
from typing import Optional, Dict, Any

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from ...model.recipe.run_result import RunResult  # <-- anpassen: relative Ebenen passend zu deinem Refactor

_LOG = logging.getLogger("tabs.process.base_statemachine")


STATE_MOVE_PREDISPENSE = "MOVE_PREDISPENSE"
STATE_MOVE_RECIPE = "MOVE_RECIPE"
STATE_MOVE_RETREAT = "MOVE_RETREAT"
STATE_MOVE_HOME = "MOVE_HOME"

SEG_ORDER = [
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
]


class _QtSignalHandler(logging.Handler):
    def __init__(self, owner: "BaseProcessStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._owner.logMessage.emit(self.format(record))
        except Exception:
            pass


class BaseProcessStatemachine(QtCore.QObject):
    """
    Gemeinsame Basis für Validate / Optimize / Execute.

    Ergebnis: RunResult (oder kompatibles dict via to_process_payload()).
    """

    notifyFinished = QtCore.pyqtSignal(object)  # emits RunResult oder dict-payload
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
        role: str,  # <-- NEU: "validate" | "optimize" | "execute"
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
    ) -> None:
        super().__init__(parent)

        self._recipe = recipe
        self._ros = ros
        self._role = str(role or "")

        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._stop_requested = False
        self._stopped = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0

        # per-run snapshot stores
        self._planned_by_segment: Dict[str, Any] = {}
        self._executed_by_segment: Dict[str, Any] = {}

        self._moveitpy = getattr(self._ros, "moveitpy", None)
        self._moveitpy_signals = getattr(self._moveitpy, "signals", None)

        self._log_handler = _QtSignalHandler(self)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        self._log_handler.setLevel(logging.INFO)
        _LOG.addHandler(self._log_handler)

        if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
            try:
                self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)
            except Exception:
                pass

    # ---------------- Public ----------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._stop_requested = False
        self._stopped = False
        self._retry_count = 0
        self._current_state = ""
        self._error_msg = None

        self._planned_by_segment.clear()
        self._executed_by_segment.clear()

        if not self._prepare_run():
            self.notifyError.emit(self._error_msg or "Prepare failed")
            self._cleanup()
            return

        self._machine = self._build_machine()
        self._machine.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True
        self._stopped = True
        try:
            self._ros.moveit_stop()
        except Exception:
            pass

    # ---------------- Hooks ----------------

    def _prepare_run(self) -> bool:
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        raise NotImplementedError

    def _on_segment_ok(self, seg_name: str) -> None:
        self._snapshot_trajs_for_segment(seg_name)

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        return True

    # ---------------- Result ----------------

    def _build_result(self) -> RunResult:
        # last snapshots für Kompatibilität (ProcessTab liest planned_traj/executed_traj)
        last_planned = self._safe_call("moveit_planned_trajectory")
        last_executed = self._safe_call("moveit_executed_trajectory")

        rr = RunResult(
            role=self._role,
            ok=not self._stopped and not bool(self._error_msg),
            message="stopped" if self._stopped else "finished",
        )

        rr.meta.update({
            "status": "stopped" if self._stopped else "finished",
            "planned_by_segment": dict(self._planned_by_segment),
            "executed_by_segment": dict(self._executed_by_segment),
        })

        # Diese beiden Keys versteht dein aktueller ProcessTab direkt:
        if isinstance(last_planned, dict):
            rr.planned_traj = last_planned
        if isinstance(last_executed, dict):
            rr.executed_traj = last_executed

        # optional: recipe-id etc. nur als meta (nicht als Object rumschleppen)
        rid = getattr(self._recipe, "id", None)
        if rid is not None:
            rr.meta["recipe_id"] = rid

        return rr

    # ---------------- Machine ----------------
    # (dein Code unverändert)
    def _build_machine(self) -> QStateMachine:
        m = QStateMachine(self)
        states = {n: QState(m) for n in SEG_ORDER}
        s_done = QFinalState(m)
        s_err = QFinalState(m)

        first = self._first_segment_state()
        m.setInitialState(states[first] if first else s_err)

        for i, name in enumerate(SEG_ORDER):
            st = states[name]
            st.entered.connect(lambda n=name: self._on_state_enter(n))
            st.addTransition(self._sig_done, states[SEG_ORDER[i + 1]] if i + 1 < len(SEG_ORDER) else s_done)
            st.addTransition(self._sig_error, s_err)

        s_done.entered.connect(self._on_finished)
        s_err.entered.connect(self._on_error)

        return m

    def _first_segment_state(self) -> Optional[str]:
        for s in SEG_ORDER:
            if self._skip_home and s == STATE_MOVE_HOME:
                continue
            if self._segment_exists(s):
                return s
        return None

    def _segment_exists(self, seg_name: str) -> bool:
        return not (self._skip_home and seg_name == STATE_MOVE_HOME)

    # ---------------- Motion ----------------

    def _on_state_enter(self, seg_name: str) -> None:
        self._current_state = seg_name
        self.stateChanged.emit(seg_name)
        self.logMessage.emit(seg_name)

        if self._stop_requested:
            self._signal_error("Gestoppt")
            return

        self._retry_count = 0
        self._on_enter_segment(seg_name)

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested or not self._current_state:
            return

        if result.startswith("ERROR"):
            if self._retry_count < self._max_retries:
                self._retry_count += 1
                self._on_enter_segment(self._current_state)
            else:
                self._signal_error(result)
            return

        if result.startswith("EXECUTED:OK"):
            if self._should_transition_on_ok(self._current_state, result):
                self._on_segment_ok(self._current_state)
                QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ---------------- Snapshot helpers ----------------

    def _snapshot_trajs_for_segment(self, seg_name: str) -> None:
        planned = self._safe_call("moveit_planned_trajectory")
        executed = self._safe_call("moveit_executed_trajectory")

        if planned is not None:
            self._planned_by_segment[seg_name] = planned
        if executed is not None:
            self._executed_by_segment[seg_name] = executed

    # ---------------- Final ----------------

    def _signal_error(self, msg: str) -> None:
        self._error_msg = msg
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_finished(self) -> None:
        rr = self._build_result()

        # ✅ kompatibel zu deinem aktuellen ProcessTab (dict erwartet):
        self.notifyFinished.emit(rr.to_process_payload())

        # Alternative (wenn ProcessTab RunResult akzeptiert):
        # self.notifyFinished.emit(rr)

        self._cleanup()

    def _on_error(self) -> None:
        self.notifyError.emit(self._error_msg or "Fehler")
        self._cleanup()

    def _safe_call(self, name: str):
        fn = getattr(self._ros, name, None)
        try:
            return fn() if fn else None
        except Exception:
            return None

    def _cleanup(self) -> None:
        try:
            if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
                self._moveitpy_signals.motionResultChanged.disconnect(self._on_motion_result)
        except Exception:
            pass

        if self._machine:
            try:
                self._machine.stop()
            except Exception:
                pass
            try:
                self._machine.deleteLater()
            except Exception:
                pass
            self._machine = None

        if self._log_handler:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None
