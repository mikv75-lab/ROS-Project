# -*- coding: utf-8 -*-
# File: tabs/process/base_statemachine.py
from __future__ import annotations

import copy
import logging
from typing import Optional, Dict, Any, List

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from PyQt6.sip import isdeleted

from model.recipe.recipe import Recipe
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
            if not isdeleted(self._owner):
                self._owner.logMessage.emit(self.format(record))
        except Exception:
            pass


class BaseProcessStatemachine(QtCore.QObject):
    """
    Common base for Validate / Optimize / Execute.

    Deterministic pairing model:
      - Wartet auf PLANNED:OK und EXECUTED:OK eines Segments.
      - Kombiniert diese erst nach Eintreffen beider Signale zu einem committed Step.
      - Nutzt RunResult als SSoT für die Rückgabe an die UI.

    Retry model (STRICT):
      - Bei motionResult "ERROR:*" wird NICHT sofort abgebrochen.
      - Stattdessen wird bis max_retries die letzte Bewegung erneut ausgelöst
        via Hook: _on_retry_last_motion(seg_name, attempt, last_error) -> bool.
      - Nur wenn Retry nicht möglich oder max_retries überschritten => final error.
    """

    ROLE = "process"

    notifyFinished = QtCore.pyqtSignal(object)  # emits RunResult payload dict
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    _sig_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    _RETRY_DELAY_MS = 250
    _WAIT_RESULTS_TIMEOUT_MS = 1000
    _WAIT_RESULTS_TICK_MS = 15

    def __init__(
        self,
        *,
        recipe: Recipe,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 3,
    ) -> None:
        super().__init__(parent)

        if not isinstance(run_result, RunResult):
            raise TypeError("BaseProcessStatemachine: run_result muss RunResult sein.")
        if not isinstance(recipe, Recipe):
            raise TypeError("BaseProcessStatemachine: recipe muss vom Typ Recipe sein.")

        self._recipe = recipe
        self._ros = ros
        self._rr: RunResult = run_result
        self._role = str(getattr(self, "ROLE", "process") or "process")

        self._max_retries = max(0, int(max_retries))

        self._stop_requested = False
        self._stopped = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0
        self._last_error_res: str = ""

        # Snapshot Speicher für finale Trajektorien (Dicts im JointTrajectory Format)
        self._planned_by_segment: Dict[str, Any] = {}
        self._executed_by_segment: Dict[str, Any] = {}

        # Puffer für Paare (ein Segment kann aus mehreren Teilbewegungen bestehen)
        self._planned_steps_by_segment: Dict[str, List[Any]] = {}
        self._executed_steps_by_segment: Dict[str, List[Any]] = {}

        # Pending Slots für asynchrones Pairing
        self._pending_planned_step: Dict[str, Optional[Dict[str, Any]]] = {}
        self._pending_executed_step: Dict[str, Optional[Dict[str, Any]]] = {}

        # WAIT_RESULTS state
        self._wait_active: bool = False
        self._wait_seg: str = ""
        self._wait_deadline_ms: int = 0
        self._wait_last_result: str = ""

        # MoveItPy Handles
        self._moveitpy = getattr(self._ros, "moveitpy", None)
        self._moveitpy_signals = getattr(self._moveitpy, "signals", None)

        self._last_planned_any: Any = None
        self._last_executed_any: Any = None

        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        self._log_handler.setLevel(logging.INFO)
        _LOG.addHandler(self._log_handler)

        if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
            try:
                self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)
            except Exception:
                pass

        self._connect_traj_cache_signals()
        self._connect_traj_cache_clear_signal()

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
            if self._segment_exists(s):
                return s
        return None

    def _segment_exists(self, seg_name: str) -> bool:
        """Subclass prüft, ob das Segment im Rezept definiert ist."""
        return True

    # ---------------- Segment capture reset ----------------

    def _reset_segment_capture(self, seg_name: str, *, clear_steps: bool = True) -> None:
        self._last_planned_any = None
        self._last_executed_any = None
        self._pending_planned_step[seg_name] = None
        self._pending_executed_step[seg_name] = None

        if self._wait_active and self._wait_seg == seg_name:
            self._wait_active = False

        if clear_steps:
            self._planned_steps_by_segment[seg_name] = []
            self._executed_steps_by_segment[seg_name] = []

    # ---------------- Motion & Pairing ----------------

    def _on_state_enter(self, seg_name: str) -> None:
        self._current_state = seg_name
        self.stateChanged.emit(seg_name)
        self.logMessage.emit(f"STATE: {seg_name}")

        if self._stop_requested:
            self._signal_error("Gestoppt")
            return

        self._retry_count = 0
        self._last_error_res = ""
        self._reset_segment_capture(seg_name, clear_steps=True)

        try:
            fn = getattr(self._ros, "moveit_set_segment", None)
            if callable(fn):
                fn(seg_name)
        except Exception:
            pass

        self._on_enter_segment(seg_name)

    def _try_commit_pair(self, seg: str) -> bool:
        pp = self._pending_planned_step.get(seg)
        pe = self._pending_executed_step.get(seg)
        if pp is None or pe is None:
            return False

        self._planned_steps_by_segment.setdefault(seg, []).append(copy.deepcopy(pp))
        self._executed_steps_by_segment.setdefault(seg, []).append(copy.deepcopy(pe))
        self._pending_planned_step[seg] = None
        self._pending_executed_step[seg] = None
        return True

    def _start_wait_results(self, seg: str, *, result: str, timeout_ms: Optional[int] = None) -> None:
        if self._wait_active:
            self._wait_last_result = str(result or "")
            return
        t_ms = int(timeout_ms) if timeout_ms is not None else int(self._WAIT_RESULTS_TIMEOUT_MS)
        self._wait_active = True
        self._wait_seg = str(seg)
        self._wait_last_result = str(result or "")
        self._wait_deadline_ms = int(QtCore.QTime.currentTime().msecsSinceStartOfDay()) + t_ms
        QtCore.QTimer.singleShot(0, self._wait_results_tick)

    def _wait_results_tick(self) -> None:
        if not self._wait_active or self._stop_requested:
            return
        seg = self._wait_seg

        if self._pending_planned_step.get(seg) is None:
            p = self._capture_planned_step(seg)
            if p:
                self._pending_planned_step[seg] = copy.deepcopy(p)

        if self._pending_executed_step.get(seg) is None:
            e = self._capture_executed_step(seg)
            if e:
                self._pending_executed_step[seg] = copy.deepcopy(e)

        if self._try_commit_pair(seg):
            self._wait_active = False
            self._after_motion_pair_ready(seg, self._wait_last_result)
            return

        now = int(QtCore.QTime.currentTime().msecsSinceStartOfDay())
        if now >= self._wait_deadline_ms:
            self._signal_error(f"Trajectory capture timeout for segment '{seg}'")
            return

        QtCore.QTimer.singleShot(int(self._WAIT_RESULTS_TICK_MS), self._wait_results_tick)

    def _after_motion_pair_ready(self, seg: str, res: str) -> None:
        if self._error_msg:
            return
        if self._should_transition_on_ok(seg, res):
            self._on_segment_ok(seg)
            QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ---------------- Retry ----------------

    def _handle_motion_error(self, seg: str, res: str) -> None:
        if self._stop_requested or self._error_msg:
            return

        self._last_error_res = str(res or "ERROR")

        if self._wait_active and self._wait_seg == seg:
            self._wait_active = False

        if self._max_retries <= 0:
            self._signal_error(f"{self._last_error_res} seg={seg}")
            return

        if self._retry_count >= self._max_retries:
            self._signal_error(
                f"{self._last_error_res} seg={seg} (retries_exhausted={self._retry_count}/{self._max_retries})"
            )
            return

        self._retry_count += 1
        attempt = self._retry_count

        self.logMessage.emit(
            f"Retry(last-motion) {attempt}/{self._max_retries} for segment '{seg}' due to: {self._last_error_res}"
        )

        QtCore.QTimer.singleShot(int(self._RETRY_DELAY_MS), lambda: self._do_retry(seg, attempt, self._last_error_res))

    def _do_retry(self, seg: str, attempt: int, last_error: str) -> None:
        if self._stop_requested or self._error_msg:
            return
        if seg != self._current_state:
            return

        ok = False
        try:
            ok = bool(self._on_retry_last_motion(seg, attempt, last_error))
        except Exception as e:
            ok = False
            _LOG.error("Retry hook raised: %s", e)

        if not ok:
            self._signal_error(f"{last_error} seg={seg} (retry_failed attempt={attempt}/{self._max_retries})")
            return

        # reset ONLY current motion pairing slots
        self._pending_planned_step[seg] = None
        self._pending_executed_step[seg] = None
        self._wait_active = False

    # ---------------- MotionResult slot ----------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        # IMPORTANT: Do NOT gate on isRunning(); signals can arrive during transitions.
        if self._machine is None or self._stop_requested or self._error_msg:
            return

        res = (result or "").strip()
        if not res:
            return

        if self._wait_active:
            return

        seg = self._current_state

        if res.startswith("ERROR"):
            self._handle_motion_error(seg, res)
            return

        if res.startswith("PLANNED:OK"):
            p = self._capture_planned_step(seg)
            if p:
                self._pending_planned_step[seg] = copy.deepcopy(p)

        elif res.startswith("EXECUTED:OK"):
            e = self._capture_executed_step(seg)
            if e:
                self._pending_executed_step[seg] = copy.deepcopy(e)
            self._start_wait_results(seg, result=res)

    # ---------------- Trajectory extraction (FULL RECURSION) ----------------

    @classmethod
    def _jt_from_any(cls, obj: Any, *, _depth: int = 0, _max_depth: int = 8) -> Optional[Dict[str, Any]]:
        if obj is None or _depth > _max_depth:
            return None

        try:
            if hasattr(obj, "joint_names") and hasattr(obj, "points"):
                out = cls._jt_msg_to_dict(obj)
                if out:
                    return out
        except Exception:
            pass

        try:
            jt = getattr(obj, "joint_trajectory", None)
            if jt is not None:
                out = cls._jt_msg_to_dict(jt)
                if out:
                    return out
        except Exception:
            pass

        if isinstance(obj, dict) and obj:
            try:
                if isinstance(obj.get("joint_names"), list) and isinstance(obj.get("points"), list):
                    return obj
            except Exception:
                pass

            for k in ("joint_trajectory", "planned", "executed", "trajectory", "result", "plan"):
                try:
                    v = obj.get(k)
                    out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                    if out:
                        return out
                except Exception:
                    pass

            for v in obj.values():
                out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if out:
                    return out

        for attr in ("trajectory", "robot_trajectory", "planned_trajectory", "executed_trajectory", "result", "plan"):
            try:
                v = getattr(obj, attr, None)
                out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if out:
                    return out
            except Exception:
                pass

        return None

    @classmethod
    def _jt_msg_to_dict(cls, jt_msg: Any) -> Optional[Dict[str, Any]]:
        if jt_msg is None:
            return None
        try:
            jn = [str(x) for x in list(getattr(jt_msg, "joint_names") or [])]
            pts_msg = list(getattr(jt_msg, "points") or [])
            pts = []
            for p in pts_msg:
                pts.append(
                    {
                        "positions": [float(x) for x in list(getattr(p, "positions", []) or [])],
                        "time_from_start": {
                            "sec": int(getattr(p.time_from_start, "sec", 0)),
                            "nanosec": int(getattr(p.time_from_start, "nanosec", 0)),
                        },
                    }
                )
            return {"joint_names": jn, "points": pts} if jn and pts else None
        except Exception:
            return None

    # ---------------- Concatenation & YAML Packing ----------------

    def _snapshot_trajs_for_segment(self, seg_name: str) -> None:
        p_steps = self._planned_steps_by_segment.get(seg_name) or []
        e_steps = self._executed_steps_by_segment.get(seg_name) or []
        if not p_steps or not e_steps:
            return
        self._planned_by_segment[seg_name] = self._concat_joint_traj_dicts(p_steps)
        self._executed_by_segment[seg_name] = self._concat_joint_traj_dicts(e_steps)

    def _concat_joint_traj_dicts(self, dicts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not dicts:
            return None
        base_names = list(dicts[0].get("joint_names") or [])
        merged_pts = []
        t_offset_ns = 0
        last_global_ns = -1

        for d in dicts:
            pts = d.get("points") or []
            for p in pts:
                tfs = p.get("time_from_start")
                t_local_ns = int(tfs["sec"]) * 1000000000 + int(tfs["nanosec"])
                t_global_ns = t_offset_ns + t_local_ns
                if t_global_ns <= last_global_ns:
                    t_global_ns = last_global_ns + 1000000  # +1ms safety

                q = copy.deepcopy(p)
                q["time_from_start"] = {"sec": t_global_ns // 1000000000, "nanosec": t_global_ns % 1000000000}
                merged_pts.append(q)
                last_global_ns = t_global_ns
            t_offset_ns = last_global_ns
        return {"joint_names": base_names, "points": merged_pts}

    # ---------------- Finalization & Cleanup ----------------

    def _on_finished(self) -> None:
        planned = self._jt_by_segment_yaml(which="planned")
        executed = self._jt_by_segment_yaml(which="executed")

        self._rr.set_planned(traj=planned)
        self._rr.set_executed(traj=executed)

        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    def _jt_by_segment_yaml(self, *, which: str) -> Dict[str, Any]:
        src = self._planned_by_segment if which == "planned" else self._executed_by_segment
        segments = {}
        for seg in SEG_ORDER:
            jt = src.get(seg)
            if jt:
                out_pts = []
                for p in jt["points"]:
                    out_pts.append(
                        {
                            "positions": p["positions"],
                            "time_from_start": [p["time_from_start"]["sec"], p["time_from_start"]["nanosec"]],
                        }
                    )
                segments[seg] = {"joint_names": jt["joint_names"], "points": out_pts}
        return {"version": 1, "segments": segments}

    def _signal_error(self, msg: str) -> None:
        self._error_msg = msg
        _LOG.error("Statemachine Error: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_error(self) -> None:
        try:
            self._ros.moveit_stop()
        except Exception:
            pass
        self.notifyError.emit(self._error_msg or "Unbekannter Fehler")
        self._cleanup()

    def _cleanup(self) -> None:
        self._wait_active = False
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

    # ---------------- Public API ----------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._stop_requested = False
        self._error_msg = None
        if not self._prepare_run():
            self.notifyError.emit(self._error_msg or "Prepare failed")
            return
        self._machine = self._build_machine()
        self._machine.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True
        try:
            self._ros.moveit_stop()
        except Exception:
            pass

    # ---------------- Hooks for Subclasses ----------------

    def _prepare_run(self) -> bool:
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        pass

    def _on_segment_ok(self, seg_name: str) -> None:
        self._snapshot_trajs_for_segment(seg_name)

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        return True

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        """
        Subclass must re-issue the last motion command for seg_name.
        Return True if a retry was triggered, otherwise False.
        """
        return False

    def _capture_planned_step(self, seg: str):
        rp = getattr(self._ros, "moveit_planned_trajectory", None)
        return self._jt_from_any(rp() if callable(rp) else self._last_planned_any)

    def _capture_executed_step(self, seg: str):
        re = getattr(self._ros, "moveit_executed_trajectory", None)
        return self._jt_from_any(re() if callable(re) else self._last_executed_any)

    def _connect_traj_cache_signals(self):
        sig = self._moveitpy_signals
        if sig:
            if hasattr(sig, "plannedTrajectoryChanged"):
                try:
                    sig.plannedTrajectoryChanged.connect(self._on_planned_traj_changed)
                except Exception:
                    pass
            if hasattr(sig, "executedTrajectoryChanged"):
                try:
                    sig.executedTrajectoryChanged.connect(self._on_executed_traj_changed)
                except Exception:
                    pass

    def _connect_traj_cache_clear_signal(self):
        sig = self._moveitpy_signals
        if sig and hasattr(sig, "trajCacheClearChanged"):
            try:
                sig.trajCacheClearChanged.connect(self._on_traj_cache_clear)
            except Exception:
                pass

    @QtCore.pyqtSlot()
    def _on_traj_cache_clear(self):
        self._last_planned_any = None
        self._last_executed_any = None

    @QtCore.pyqtSlot(object)
    def _on_planned_traj_changed(self, obj):
        self._last_planned_any = obj

    @QtCore.pyqtSlot(object)
    def _on_executed_traj_changed(self, obj):
        self._last_executed_any = obj
