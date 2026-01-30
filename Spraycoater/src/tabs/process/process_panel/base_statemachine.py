# -*- coding: utf-8 -*-
# File: src/tabs/process/base_statemachine.py
from __future__ import annotations

import time
import logging
from typing import Optional, Dict, Any, Tuple

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from PyQt6.sip import isdeleted

# Helper aus process_serialization/contract
from .process_contract import (
    ReqKey, 
    parse_motion_result, 
    parse_key_from_traj_header
)
from .process_serialization import (
    is_empty_robot_trajectory, 
    jt_msg_to_dict_normalized, 
    concat_jt_dicts_sequential
)

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
            if not isdeleted(self._owner):
                self._owner.logMessage.emit(self.format(record))
        except Exception:
            pass

class BaseProcessStatemachine(QtCore.QObject):
    ROLE = "process"
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)
    
    _sig_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    def __init__(self, *, recipe: Any, ros: Any, run_result: Any, parent: Optional[QtCore.QObject] = None, max_retries: int = 0) -> None:
        super().__init__(parent)
        self._recipe = recipe
        self._ros = ros
        self._rr = run_result
        self._role = str(getattr(self, "ROLE", "process") or "process")

        self._stop_requested = False
        self._stop_emitted = False
        self._error_msg: Optional[str] = None
        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        
        # Shared Context
        self._run_id: str = ""
        self._side: str = "top"

        # MoveItPy signals
        self._moveitpy = None
        self._moveitpy_signals = None
        try:
            self._moveitpy = self._ros.moveitpy
            if self._moveitpy is not None:
                self._moveitpy_signals = self._moveitpy.signals
        except Exception:
            pass

        # Stashes (run, id, seg, op) -> Data
        # Hier sammeln wir ALLES. Die Subklassen müssen nix mehr selbst stashen.
        self._results: Dict[Tuple[str, int, str, str], Dict[str, Any]] = {}
        self._traj_planned: Dict[Tuple[str, int, str, str], Any] = {}
        self._traj_executed: Dict[Tuple[str, int, str, str], Any] = {}
        self._traj_optimized: Dict[Tuple[str, int, str, str], Any] = {}

        # Logger
        self._log_handler = _QtSignalHandler(self)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        self._log_handler.setLevel(logging.INFO)
        _LOG.addHandler(self._log_handler)

        self._connect_signals()
        _ = max_retries

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._stop_requested = False
        self._stop_emitted = False
        self._error_msg = None
        
        self._results.clear()
        self._traj_planned.clear()
        self._traj_executed.clear()
        self._traj_optimized.clear()

        # Shared setup
        self._prepare_run_base()

        if not self._prepare_run():
            self.notifyError.emit(self._error_msg or "Prepare failed")
            return

        self._machine = self._build_machine()
        self._machine.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._stop_requested: return
        self._stop_requested = True
        try:
            if hasattr(self._ros, "stop_all") and callable(self._ros.stop_all):
                self._ros.stop_all()
            elif hasattr(self._ros, "moveit_stop") and callable(self._ros.moveit_stop):
                self._ros.moveit_stop()
        except Exception:
            pass
        if not self._stop_emitted:
            self._stop_emitted = True
            if not self._error_msg: self._error_msg = "STOPPED"
            if self._machine is None:
                self.notifyError.emit(self._error_msg)
                self._cleanup()
                return
            QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def current_segment(self) -> str:
        return str(self._current_state or "")

    def segment_done(self) -> None:
        if self._machine is None or self._stop_requested or self._error_msg: return
        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    def segment_error(self, msg: str) -> None:
        self._signal_error(str(msg or "ERROR"))

    # --- Internal ---
    def _prepare_run_base(self) -> None:
        self._run_id = str(getattr(self._rr, "run_id", "") or "").strip()
        if not self._run_id: self._run_id = f"run_{int(time.time() * 1000)}"
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", "top") or "top")
        except Exception:
            self._side = "top"

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
            st.addTransition(self._sig_done, states[SEG_ORDER[i+1]] if i+1 < len(SEG_ORDER) else s_done)
            st.addTransition(self._sig_error, s_err)
        s_done.entered.connect(self._on_finished)
        s_err.entered.connect(self._on_error)
        return m

    def _first_segment_state(self) -> Optional[str]:
        for s in SEG_ORDER:
            if self._segment_exists(s): return s
        return None
    def _segment_exists(self, seg_name: str) -> bool: return True
    def _on_state_enter(self, seg_name: str) -> None:
        self._current_state = seg_name
        self.stateChanged.emit(seg_name)
        self.logMessage.emit(f"STATE: {seg_name}")
        if self._stop_requested:
            self._signal_error("STOPPED")
            return
        try:
            self._on_enter_segment(seg_name)
        except Exception as e:
            self._signal_error(f"enter_segment failed: {e}")

    def _connect_signals(self) -> None:
        sig = self._moveitpy_signals
        if sig is None: return
        try:
            sig.motionResultChanged.connect(self._on_motion_result)
            sig.plannedTrajectoryChanged.connect(lambda o: self._stash_traj(kind="planned", obj=o))
            sig.executedTrajectoryChanged.connect(lambda o: self._stash_traj(kind="executed", obj=o))
            sig.optimizedTrajectoryChanged.connect(lambda o: self._stash_traj(kind="optimized", obj=o))
        except Exception:
            pass

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        if self._machine is None or self._stop_requested: return
        try:
            key, status, d = parse_motion_result(text)
            self._results[key.as_tuple()] = dict(d)
        except Exception:
            pass

    def _stash_traj(self, *, kind: str, obj: Any) -> None:
        if self._machine is None or self._stop_requested or self._error_msg: return
        if is_empty_robot_trajectory(obj): return
        try:
            key = parse_key_from_traj_header(obj)
        except Exception:
            return 
        t = key.as_tuple()
        if kind == "planned": self._traj_planned[t] = obj
        elif kind == "executed": self._traj_executed[t] = obj
        else: self._traj_optimized[t] = obj

    def _deduplicate_storage(self) -> None:
        """Entfernt Duplikate (Retries), behält nur das Neuste pro (seg, op)."""
        self._results = self._filter_latest(self._results)
        self._traj_planned = self._filter_latest(self._traj_planned)
        self._traj_executed = self._filter_latest(self._traj_executed)
        self._traj_optimized = self._filter_latest(self._traj_optimized)
    
    def _filter_latest(self, source: Dict[Tuple, Any]) -> Dict[Tuple, Any]:
        if not source: return {}
        latest = {}
        for full_key, val in source.items():
            seg, op = full_key[2], full_key[3]
            latest[(seg, op)] = (full_key, val)
        return {k: v for k, v in latest.values()}

    def _build_trajectory_yaml(self, source_stash: Dict, which_op: str) -> Dict[str, Any]:
        """
        Zentraler 'Staubsauger': Baut aus den gesammelten Daten das fertige YAML-Dict.
        Sortiert und merged Zeitstempel.
        """
        segments = {}
        for seg in SEG_ORDER:
            entries = []
            for (run, rid, s, op), msg in source_stash.items():
                if str(run) == self._run_id and str(s) == seg and str(op) == which_op:
                    d = jt_msg_to_dict_normalized(msg)
                    if d: entries.append((rid, d))
            entries.sort(key=lambda x: x[0])
            dicts = [e[1] for e in entries]
            if dicts:
                merged = concat_jt_dicts_sequential(dicts)
                if merged: segments[seg] = merged
        return {"version": 1, "segments": segments}

    def _on_finished(self) -> None:
        # Standard Finish Logic (kann von Subklasse erweitert werden)
        self._deduplicate_storage()
        try:
            fn = getattr(self._rr, "attach_recipe_context", None)
            if callable(fn): fn(self._recipe)
        except Exception:
            pass
        
        try:
            payload = self._rr.to_process_payload()
        except Exception:
            payload = {"ok": True}
        self.notifyFinished.emit(payload)
        self._cleanup()

    def _signal_error(self, msg: str) -> None:
        if self._error_msg: return
        self._error_msg = str(msg or "ERROR")
        _LOG.error("Statemachine Error: %s", self._error_msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_error(self) -> None:
        self._deduplicate_storage()
        try:
            if hasattr(self._ros, "moveit_stop") and callable(self._ros.moveit_stop):
                self._ros.moveit_stop()
        except Exception:
            pass
        self.notifyError.emit(self._error_msg or "ERROR")
        self._cleanup()

    def _cleanup(self) -> None:
        if self._machine:
            try:
                self._machine.stop()
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

    def _prepare_run(self) -> bool: return True
    def _on_enter_segment(self, seg_name: str) -> None: raise NotImplementedError