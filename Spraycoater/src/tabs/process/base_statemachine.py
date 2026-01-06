# -*- coding: utf-8 -*-
# File: tabs/process/base_statemachine.py
from __future__ import annotations

import copy
import logging
from typing import Optional, Dict, Any, List

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from model.recipe.recipe_run_result import RunResult

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

    Subklassen:
      - setzen ROLE = "validate" | "optimize" | "execute"
      - implementieren _on_enter_segment(seg_name)
      - optional: _prepare_run(), _segment_exists(), _should_transition_on_ok()

    Contract:
      - StateMachine triggert Bewegungs-Calls via ros.*
      - Transition passiert über moveitpy.signals.motionResultChanged:
          - "EXECUTED:OK"  -> next / or stay in segment (override _should_transition_on_ok)
          - "ERROR..."     -> retry / fail
      - Eval/Score passiert NICHT hier (macht ProcessTab).

    Output:
      - notifyFinished emit() liefert dict payload (RunResult.to_process_payload()).
        (Robust: falls _build_result() bereits dict liefert, wird direkt emittiert.)
    """

    ROLE = "process"

    notifyFinished = QtCore.pyqtSignal(object)  # emits dict payload
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
        self._role = str(getattr(self, "ROLE", "process") or "process")

        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._stop_requested = False
        self._stopped = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0

        # per-run snapshot stores (final per segment, may already be concatenated)
        self._planned_by_segment: Dict[str, Any] = {}
        self._executed_by_segment: Dict[str, Any] = {}

        # per-step buffers (captures every EXECUTED:OK inside same segment)
        self._planned_steps_by_segment: Dict[str, List[Any]] = {}
        self._executed_steps_by_segment: Dict[str, List[Any]] = {}

        self._moveitpy = getattr(self._ros, "moveitpy", None)
        self._moveitpy_signals = getattr(self._moveitpy, "signals", None)

        # forward our internal logger to UI
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
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()

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
        # finalize segment buffers into *_by_segment
        self._snapshot_trajs_for_segment(seg_name)

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        return True

    # ---------------- Result ----------------

    def _build_result(self) -> RunResult:
        last_planned = self._safe_call("moveit_planned_trajectory")
        last_executed = self._safe_call("moveit_executed_trajectory")

        rr = RunResult(
            role=self._role,
            ok=not self._stopped and not bool(self._error_msg),
            message="stopped" if self._stopped else "finished",
        )

        rr.meta.update(
            {
                "status": "stopped" if self._stopped else "finished",
                "planned_by_segment": dict(self._planned_by_segment),
                "executed_by_segment": dict(self._executed_by_segment),
            }
        )

        if isinstance(last_planned, dict):
            rr.planned_traj = last_planned
        if isinstance(last_executed, dict):
            rr.executed_traj = last_executed

        rid = getattr(self._recipe, "id", None)
        if rid is not None:
            rr.meta["recipe_id"] = rid

        return rr

    def _result_payload(self) -> Dict[str, Any]:
        """
        Robust conversion:
          - if _build_result() returns RunResult -> use to_process_payload()
          - if something returns dict already -> pass through
        """
        rr = self._build_result()
        if hasattr(rr, "to_process_payload"):
            try:
                payload = rr.to_process_payload()
                if isinstance(payload, dict):
                    return payload
            except Exception:
                pass

        # fallback (should not normally happen)
        if isinstance(rr, dict):
            return rr
        return {"role": self._role, "ok": not self._stopped and not bool(self._error_msg), "message": "finished"}

    # ---------------- Machine ----------------

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

        # reset per-segment step buffers on entry (important for retries)
        self._planned_steps_by_segment[seg_name] = []
        self._executed_steps_by_segment[seg_name] = []

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
                # retry same state: keep buffers but append new steps as they come
                self._on_enter_segment(self._current_state)
            else:
                self._signal_error(result)
            return

        if result.startswith("EXECUTED:OK"):
            # capture per-step trajectories first (so segment logic can send next move)
            self._snapshot_step_for_segment(self._current_state)

            if self._should_transition_on_ok(self._current_state, result):
                self._on_segment_ok(self._current_state)
                QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ---------------- Snapshot helpers ----------------

    def _snapshot_step_for_segment(self, seg_name: str) -> None:
        planned = self._safe_call("moveit_planned_trajectory")
        executed = self._safe_call("moveit_executed_trajectory")

        if planned is not None:
            self._planned_steps_by_segment.setdefault(seg_name, []).append(copy.deepcopy(planned))
        if executed is not None:
            self._executed_steps_by_segment.setdefault(seg_name, []).append(copy.deepcopy(executed))

    def _snapshot_trajs_for_segment(self, seg_name: str) -> None:
        planned_steps = self._planned_steps_by_segment.get(seg_name) or []
        executed_steps = self._executed_steps_by_segment.get(seg_name) or []

        planned = self._concat_joint_traj_dicts(planned_steps) if planned_steps else self._safe_call("moveit_planned_trajectory")
        executed = (
            self._concat_joint_traj_dicts(executed_steps) if executed_steps else self._safe_call("moveit_executed_trajectory")
        )

        if planned is not None:
            self._planned_by_segment[seg_name] = planned
        if executed is not None:
            self._executed_by_segment[seg_name] = executed

    def _concat_joint_traj_dicts(self, items: List[Any]) -> Any:
        """
        Best-effort concatenation for the dict schema returned by ros.moveit_*_trajectory().

        Expected (typical) structure:
          {
            "joint_names": [...],
            "points": [ { "positions": [...], "velocities": [...], "time_from_start": <float|int|{sec,nsec}|str> }, ... ],
            ...
          }

        If schema is unknown, we fall back to:
          - return last non-empty element
        """
        dicts = [d for d in items if isinstance(d, dict) and d]
        if not dicts:
            # return last raw item if any
            for it in reversed(items):
                if it is not None:
                    return it
            return None

        base = copy.deepcopy(dicts[0])
        pts_out: List[dict] = []
        joint_names = base.get("joint_names")

        t_offset = 0.0

        def _tfs_to_float(tfs: Any) -> float:
            if tfs is None:
                return 0.0
            if isinstance(tfs, (int, float)):
                return float(tfs)
            if isinstance(tfs, str):
                try:
                    return float(tfs)
                except Exception:
                    return 0.0
            if isinstance(tfs, dict):
                # ROS-style: {"sec": int, "nanosec": int} or {"secs":...}
                sec = tfs.get("sec", tfs.get("secs", 0))
                nsec = tfs.get("nanosec", tfs.get("nsec", 0))
                try:
                    return float(sec) + float(nsec) * 1e-9
                except Exception:
                    return 0.0
            return 0.0

        def _float_to_tfs_like(example: Any, val: float) -> Any:
            if example is None:
                return val
            if isinstance(example, (int, float)):
                return float(val)
            if isinstance(example, str):
                return str(val)
            if isinstance(example, dict):
                sec = int(val)
                nsec = int(round((val - sec) * 1e9))
                # preserve key naming if possible
                if "nanosec" in example or "sec" in example:
                    return {"sec": sec, "nanosec": nsec}
                if "nsec" in example or "secs" in example:
                    return {"secs": sec, "nsec": nsec}
                return {"sec": sec, "nanosec": nsec}
            return float(val)

        for d in dicts:
            if joint_names is not None and d.get("joint_names") is not None and d.get("joint_names") != joint_names:
                # incompatible -> cannot concatenate reliably
                continue

            pts = d.get("points")
            if not isinstance(pts, list) or not pts:
                continue

            # determine last time in current output
            if pts_out:
                last_ex = pts_out[-1].get("time_from_start")
                t_offset = _tfs_to_float(last_ex)
            else:
                t_offset = 0.0

            # append points with offset, but avoid duplicating the first point if it's identical to last
            for j, p in enumerate(pts):
                if not isinstance(p, dict):
                    continue
                p2 = copy.deepcopy(p)
                ex = p2.get("time_from_start")
                if ex is not None:
                    t = _tfs_to_float(ex)
                    if pts_out:
                        t = t + t_offset
                    p2["time_from_start"] = _float_to_tfs_like(ex, t)

                # de-dup first point (common when concatenating segments)
                if pts_out and j == 0:
                    try:
                        if p2.get("positions") == pts_out[-1].get("positions"):
                            continue
                    except Exception:
                        pass

                pts_out.append(p2)

        base["points"] = pts_out
        return base

    # ---------------- Final ----------------

    def _signal_error(self, msg: str) -> None:
        self._error_msg = msg
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_finished(self) -> None:
        payload = self._result_payload()
        self.notifyFinished.emit(payload)
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
        # disconnect moveit signals
        try:
            if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
                self._moveitpy_signals.motionResultChanged.disconnect(self._on_motion_result)
        except Exception:
            pass

        # stop + delete machine
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

        # remove log handler
        if self._log_handler:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None
