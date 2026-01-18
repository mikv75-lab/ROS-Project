# -*- coding: utf-8 -*-
# File: tabs/process/base_statemachine.py
from __future__ import annotations

import copy
import logging
from typing import Optional, Dict, Any, List, Tuple

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from PyQt6.sip import isdeleted

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
    """Common base for Validate / Optimize / Execute.

    Capture (STRICT + race-tolerant):
      - MoveItPy can publish PLANNED:OK / EXECUTED:OK before the trajectory objects
        are queryable via RosBridge / cached Qt signals.
      - Therefore we NEVER hard-fail on PLANNED:OK/EXECUTED:OK when the trajectory
        is missing *immediately*.

    Deterministic pairing model:
      - For each motion pair we wait (non-blocking) until BOTH planned + executed
        are available, then we commit exactly one pair into step lists.

    Design:
      - On PLANNED:OK: mark planned pending (best-effort capture now).
      - On EXECUTED:OK: mark executed pending (best-effort capture now) and enter
        WAIT_RESULTS loop with timeout (default 1000ms).
      - WAIT_RESULTS tick keeps polling sources until both are present; then:
          commit pair -> call _after_motion_pair_ready(seg, res)

    IMPORTANT:
      - trajCacheClearChanged must NOT clear committed steps or pending slots.
        It only clears last_* cache mirrors.
      - No dedupe logic. No blocking sleeps.
    """

    ROLE = "process"

    notifyFinished = QtCore.pyqtSignal(object)  # emits RunResult payload dict
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    _sig_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    _RETRY_DELAY_MS = 250

    # WAIT_RESULTS behavior
    _WAIT_RESULTS_TIMEOUT_MS = 1000
    _WAIT_RESULTS_TICK_MS = 15

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 3,
        skip_home: bool = False,
    ) -> None:
        super().__init__(parent)

        if not isinstance(run_result, RunResult):
            raise TypeError("BaseProcessStatemachine: run_result muss RunResult sein.")

        self._recipe = recipe
        self._ros = ros
        self._rr: RunResult = run_result
        self._role = str(getattr(self, "ROLE", "process") or "process")

        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._stop_requested = False
        self._stopped = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0

        # per-run snapshot stores (final per-segment JT dicts)
        self._planned_by_segment: Dict[str, Any] = {}
        self._executed_by_segment: Dict[str, Any] = {}

        # per-segment step buffers (paired JT dicts, one per committed pair)
        self._planned_steps_by_segment: Dict[str, List[Any]] = {}
        self._executed_steps_by_segment: Dict[str, List[Any]] = {}

        # pending slots (allow out-of-order)
        self._pending_planned_step: Dict[str, Optional[Dict[str, Any]]] = {}
        self._pending_executed_step: Dict[str, Optional[Dict[str, Any]]] = {}

        # WAIT_RESULTS state
        self._wait_active: bool = False
        self._wait_seg: str = ""
        self._wait_deadline_ms: int = 0
        self._wait_last_result: str = ""

        # --- MoveItPy handles ---
        self._moveitpy = getattr(self._ros, "moveitpy", None)
        self._moveitpy_signals = getattr(self._moveitpy, "signals", None)

        # --- cached last trajectories (Qt signal fallback) ---
        self._last_planned_any: Any = None
        self._last_executed_any: Any = None
        self._traj_sig_connected: bool = False
        self._motion_sig_connected: bool = False
        self._clear_sig_connected: bool = False

        # forward our internal logger to UI (avoid duplicates on re-instantiation)
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        self._log_handler.setLevel(logging.INFO)
        _LOG.addHandler(self._log_handler)

        # connect motion result
        if self._moveitpy_signals is not None and hasattr(self._moveitpy_signals, "motionResultChanged"):
            try:
                self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)
                self._motion_sig_connected = True
            except Exception:
                self._motion_sig_connected = False

        # connect trajectory cache signals (fallback)
        self._connect_traj_cache_signals()

        # connect boundary clear signal
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
            if self._skip_home and s == STATE_MOVE_HOME:
                continue
            if self._segment_exists(s):
                return s
        return None

    def _segment_exists(self, seg_name: str) -> bool:
        return not (self._skip_home and seg_name == STATE_MOVE_HOME)

    # ---------------- Segment capture reset ----------------

    def _reset_segment_capture(self, seg_name: str, *, clear_steps: bool = True) -> None:
        """Reset segment-local capture state.

        - On normal segment entry: clear_steps=True
        - On retry: do NOT clear steps

        NOTE: does NOT touch already committed steps unless clear_steps=True.
        """
        self._last_planned_any = None
        self._last_executed_any = None

        self._pending_planned_step[seg_name] = None
        self._pending_executed_step[seg_name] = None

        # cancel any wait loop that might still be running
        if self._wait_active and self._wait_seg == seg_name:
            self._wait_active = False
            self._wait_seg = ""
            self._wait_deadline_ms = 0
            self._wait_last_result = ""

        if clear_steps:
            self._planned_steps_by_segment[seg_name] = []
            self._executed_steps_by_segment[seg_name] = []

    # ---------------- Motion ----------------

    def _on_state_enter(self, seg_name: str) -> None:
        self._current_state = seg_name
        self.stateChanged.emit(seg_name)
        self.logMessage.emit(f"STATE: {seg_name}")

        if self._stop_requested:
            self._signal_error("Gestoppt")
            return

        self._retry_count = 0

        # STRICT: segment-local capture reset
        self._reset_segment_capture(seg_name, clear_steps=True)

        # best-effort: tag segment into node (if supported)
        try:
            fn = getattr(self._ros, "moveit_set_segment", None)
            if callable(fn):
                fn(seg_name)
        except Exception:
            pass

        self._on_enter_segment(seg_name)

    def _is_soft_error(self, res: str) -> bool:
        r = (res or "").strip()
        if not r:
            return False
        code = r.split()[0]
        return code in {"ERROR:EXEC", "ERROR:NO_TRAJ"}

    def _make_retry_exhausted_msg(self, seg: str, res: str) -> str:
        return (
            f"RETRY_EXHAUSTED: {res} seg={seg} "
            f"(attempts={int(self._retry_count)}/{int(self._max_retries)})"
        )

    def _try_commit_pair(self, seg: str) -> bool:
        """Commit exactly one planned/executed pair if both pending are present."""
        pp = self._pending_planned_step.get(seg)
        pe = self._pending_executed_step.get(seg)
        if pp is None or pe is None:
            return False

        self._planned_steps_by_segment.setdefault(seg, []).append(copy.deepcopy(pp))
        self._executed_steps_by_segment.setdefault(seg, []).append(copy.deepcopy(pe))

        self._pending_planned_step[seg] = None
        self._pending_executed_step[seg] = None
        return True

    def _now_ms(self) -> int:
        # monotonic enough for timeouts within a run
        return int(QtCore.QTime.currentTime().msecsSinceStartOfDay())

    def _start_wait_results(self, seg: str, *, result: str, timeout_ms: Optional[int] = None) -> None:
        if self._wait_active:
            # already waiting; keep the earlier deadline but update last result
            self._wait_last_result = str(result or "")
            return

        t_ms = int(timeout_ms) if timeout_ms is not None else int(self._WAIT_RESULTS_TIMEOUT_MS)
        if t_ms <= 0:
            t_ms = 1

        self._wait_active = True
        self._wait_seg = str(seg)
        self._wait_last_result = str(result or "")
        self._wait_deadline_ms = self._now_ms() + t_ms

        QtCore.QTimer.singleShot(0, self._wait_results_tick)

    def _wait_results_tick(self) -> None:
        if not self._wait_active:
            return
        if self._stop_requested or not self._machine or not self._machine.isRunning():
            self._wait_active = False
            return

        seg = self._wait_seg
        if not seg or seg != self._current_state:
            # segment changed -> abort wait
            self._wait_active = False
            self._wait_seg = ""
            return

        # best-effort capture missing pending pieces
        if self._pending_planned_step.get(seg) is None:
            p = self._capture_planned_step(seg)
            if p is not None:
                self._pending_planned_step[seg] = copy.deepcopy(p)

        if self._pending_executed_step.get(seg) is None:
            e = self._capture_executed_step(seg)
            if e is not None:
                self._pending_executed_step[seg] = copy.deepcopy(e)

        if self._try_commit_pair(seg):
            # we have exactly one pair -> continue OK-processing deterministically
            self._wait_active = False
            self._wait_seg = ""
            self._wait_deadline_ms = 0

            res = self._wait_last_result
            self._wait_last_result = ""
            self._after_motion_pair_ready(seg, res)
            return

        if self._now_ms() >= int(self._wait_deadline_ms):
            # timeout -> hard error
            self._wait_active = False
            self._wait_seg = ""

            pp = self._pending_planned_step.get(seg)
            pe = self._pending_executed_step.get(seg)
            self._signal_error(
                "Trajectory capture timeout for segment '{seg}': planned={p} executed={e} (waited {ms}ms).".format(
                    seg=str(seg),
                    p="ok" if pp is not None else "None",
                    e="ok" if pe is not None else "None",
                    ms=int(self._WAIT_RESULTS_TIMEOUT_MS),
                )
            )
            return

        QtCore.QTimer.singleShot(int(self._WAIT_RESULTS_TICK_MS), self._wait_results_tick)

    def _after_motion_pair_ready(self, seg: str, res: str) -> None:
        """Called exactly once per committed pair, after planned+executed are available.

        Default behavior:
          - if _should_transition_on_ok(...) -> snapshot for seg and advance
          - otherwise: stay in segment (subclass drives next motions)
        """
        if self._error_msg:
            return

        if self._should_transition_on_ok(seg, res):
            self._on_segment_ok(seg)
            QtCore.QTimer.singleShot(0, self._sig_done.emit)

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested or not self._current_state:
            return

        res = (result or "").strip()
        if not res:
            return

        # If we are currently waiting for trajectories, ignore additional motion results
        # until the pair is committed. This prevents overlaps.
        if self._wait_active:
            return

        if res.startswith("ERROR"):
            try:
                self._ros.moveit_stop()
            except Exception:
                pass

            seg = self._current_state
            soft = self._is_soft_error(res)

            if soft and (self._retry_count < self._max_retries) and (not self._stop_requested):
                self._retry_count += 1
                _LOG.warning(
                    "Retry(last-motion) %d/%d for segment '%s' due to: %s",
                    int(self._retry_count), int(self._max_retries), str(seg), str(res)
                )

                ok = False
                try:
                    ok = bool(self._on_retry_last_motion(seg, int(self._retry_count), str(res)))
                except Exception:
                    ok = False

                if ok:
                    QtCore.QTimer.singleShot(self._RETRY_DELAY_MS, lambda: None)
                    return

                self._signal_error(self._make_retry_exhausted_msg(seg, res))
                return

            if soft:
                self._signal_error(self._make_retry_exhausted_msg(seg, res))
            else:
                self._signal_error(f"{res} seg={seg}")
            return

        seg = self._current_state

        if res.startswith("PLANNED:OK"):
            # Best-effort capture now; but do NOT fail if missing. We will wait on EXECUTED:OK.
            planned = self._capture_planned_step(seg)
            if planned is not None:
                self._pending_planned_step[seg] = copy.deepcopy(planned)
            else:
                # Keep pending None; wait loop will attempt again.
                self._pending_planned_step.setdefault(seg, None)
            return

        if res.startswith("EXECUTED:OK"):
            executed = self._capture_executed_step(seg)
            if executed is not None:
                self._pending_executed_step[seg] = copy.deepcopy(executed)
            else:
                self._pending_executed_step.setdefault(seg, None)

            # Start deterministic wait loop to obtain BOTH planned + executed.
            self._start_wait_results(seg, result=res, timeout_ms=int(self._WAIT_RESULTS_TIMEOUT_MS))
            return

    # ---------------- Trajectory cache signals ----------------

    def _connect_traj_cache_signals(self) -> None:
        sig = self._moveitpy_signals
        if sig is None:
            self._traj_sig_connected = False
            return

        ok = False
        try:
            if hasattr(sig, "plannedTrajectoryChanged"):
                sig.plannedTrajectoryChanged.connect(self._on_planned_traj_changed)
                ok = True
        except Exception:
            pass
        try:
            if hasattr(sig, "executedTrajectoryChanged"):
                sig.executedTrajectoryChanged.connect(self._on_executed_traj_changed)
                ok = True
        except Exception:
            pass

        self._traj_sig_connected = bool(ok)

    def _connect_traj_cache_clear_signal(self) -> None:
        sig = self._moveitpy_signals
        if sig is None:
            self._clear_sig_connected = False
            return
        try:
            if hasattr(sig, "trajCacheClearChanged"):
                sig.trajCacheClearChanged.connect(self._on_traj_cache_clear)
                self._clear_sig_connected = True
                return
        except Exception:
            pass
        self._clear_sig_connected = False

    @QtCore.pyqtSlot()
    def _on_traj_cache_clear(self) -> None:
        """CRITICAL FIX:

        Do NOT clear captured steps.
        Do NOT clear pending slots.

        Only clear "last_*" cache mirrors, because those are just sources.
        """
        self._last_planned_any = None
        self._last_executed_any = None

    @QtCore.pyqtSlot(object)
    def _on_planned_traj_changed(self, obj: object) -> None:
        self._last_planned_any = obj

    @QtCore.pyqtSlot(object)
    def _on_executed_traj_changed(self, obj: object) -> None:
        self._last_executed_any = obj

    # ---------------- Snapshot helpers ----------------

    def _capture_planned_step(self, seg_name: str) -> Optional[Dict[str, Any]]:
        planned_raw, _ = self._get_step_sources()
        return self._jt_from_any(planned_raw)

    def _capture_executed_step(self, seg_name: str) -> Optional[Dict[str, Any]]:
        _, executed_raw = self._get_step_sources()
        return self._jt_from_any(executed_raw)

    def _snapshot_trajs_for_segment(self, seg_name: str) -> None:
        planned_steps = self._planned_steps_by_segment.get(seg_name) or []
        executed_steps = self._executed_steps_by_segment.get(seg_name) or []

        if not planned_steps or not executed_steps:
            self._signal_error(
                f"Segment '{seg_name}': no paired trajectory steps captured. "
                "Expected at least one PLANNED:OK+EXECUTED:OK pair."
            )
            return

        if len(planned_steps) != len(executed_steps):
            self._signal_error(
                f"Segment '{seg_name}': planned/executed step count mismatch "
                f"(planned={len(planned_steps)}, executed={len(executed_steps)}). "
                "Refusing to persist."
            )
            return

        planned = self._concat_joint_traj_dicts(planned_steps)
        executed = self._concat_joint_traj_dicts(executed_steps)

        if planned is None or executed is None:
            self._signal_error(
                f"Segment '{seg_name}': concat failed "
                f"(planned={'ok' if planned is not None else 'None'}, "
                f"executed={'ok' if executed is not None else 'None'})."
            )
            return

        self._planned_by_segment[seg_name] = planned
        self._executed_by_segment[seg_name] = executed

    def _get_step_sources(self) -> Tuple[Any, Any]:
        rp = getattr(self._ros, "moveit_planned_trajectory", None)
        re_ = getattr(self._ros, "moveit_executed_trajectory", None)

        if callable(rp) and callable(re_):
            try:
                return rp(), re_()
            except Exception as e:
                self._signal_error(f"RosBridge moveit_*_trajectory() failed: {e!r}")
                return None, None

        return self._last_planned_any, self._last_executed_any

    # ---------------- JointTrajectory extraction ----------------

    @staticmethod
    def _duration_to_dict(dur: Any) -> Dict[str, int]:
        try:
            return {"sec": int(getattr(dur, "sec")), "nanosec": int(getattr(dur, "nanosec"))}
        except Exception:
            return {"sec": 0, "nanosec": 0}

    @classmethod
    def _jt_msg_to_dict(cls, jt_msg: Any) -> Optional[Dict[str, Any]]:
        if jt_msg is None:
            return None
        if not hasattr(jt_msg, "joint_names") or not hasattr(jt_msg, "points"):
            return None

        try:
            joint_names = [str(x) for x in list(getattr(jt_msg, "joint_names") or [])]
            points_msg = list(getattr(jt_msg, "points") or [])
        except Exception:
            return None

        pts: List[Dict[str, Any]] = []
        for p in points_msg:
            try:
                pts.append(
                    {
                        "positions": [float(x) for x in list(getattr(p, "positions", []) or [])],
                        "velocities": [float(x) for x in list(getattr(p, "velocities", []) or [])],
                        "accelerations": [float(x) for x in list(getattr(p, "accelerations", []) or [])],
                        "effort": [float(x) for x in list(getattr(p, "effort", []) or [])],
                        "time_from_start": cls._duration_to_dict(getattr(p, "time_from_start", None)),
                    }
                )
            except Exception:
                continue

        if not joint_names or not pts:
            return None
        return {"joint_names": joint_names, "points": pts}

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
                except Exception:
                    v = None
                out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if out:
                    return out

            for v in obj.values():
                out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if out:
                    return out

        for attr in ("trajectory", "robot_trajectory", "planned_trajectory", "executed_trajectory", "result", "plan"):
            try:
                v = getattr(obj, attr, None)
            except Exception:
                v = None
            out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
            if out:
                return out

        return None

    # ---------------- concatenation (merged JT per segment) ----------------

    def _concat_joint_traj_dicts(self, dicts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not dicts:
            return None

        base_names = list(dicts[0].get("joint_names") or [])
        if not base_names:
            return None

        def dur_to_ns(tfs):
            if isinstance(tfs, dict):
                return int(tfs.get("sec", 0)) * 1000000000 + int(tfs.get("nanosec", 0))
            if isinstance(tfs, (list, tuple)) and len(tfs) == 2:
                return int(tfs[0]) * 1000000000 + int(tfs[1])
            return 0

        def ns_to_dur(ns):
            return {"sec": int(ns // 1000000000), "nanosec": int(ns % 1000000000)}

        merged_pts: List[Dict[str, Any]] = []
        t_offset_ns = 0
        last_global_ns = -1

        for d in dicts:
            if list(d.get("joint_names") or []) != base_names:
                return None
            pts = [p for p in (d.get("points") or []) if isinstance(p, dict)]
            if not pts:
                continue

            for p in pts:
                t_local_ns = dur_to_ns(p.get("time_from_start"))
                t_global_ns = t_offset_ns + t_local_ns
                if last_global_ns >= 0 and t_global_ns <= last_global_ns:
                    t_global_ns = last_global_ns + 1000000  # +1ms monotonic guard
                q = copy.deepcopy(p)
                q["time_from_start"] = ns_to_dur(t_global_ns)
                merged_pts.append(q)
                last_global_ns = t_global_ns
            t_offset_ns = last_global_ns

        if not merged_pts:
            return None
        return {"joint_names": base_names, "points": merged_pts}

    # ---------------- Final ----------------

    def _signal_error(self, msg: str) -> None:
        self._error_msg = msg
        _LOG.error("Statemachine Error: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()
        self._rr.set_planned(traj=planned)
        self._rr.set_executed(traj=executed)
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    def _on_error(self) -> None:
        try:
            self._ros.moveit_stop()
        except Exception:
            pass
        try:
            fn = getattr(self._ros, "moveit_set_segment", None)
            if callable(fn):
                fn("")
        except Exception:
            pass

        self.notifyError.emit(self._error_msg or "Unbekannter Fehler")
        self._cleanup()

    def _cleanup(self) -> None:
        # cancel wait
        self._wait_active = False
        self._wait_seg = ""
        self._wait_deadline_ms = 0
        self._wait_last_result = ""

        if self._machine:
            try:
                self._machine.stop()
                self._machine.deleteLater()
            except Exception:
                pass
            self._machine = None

        try:
            sig = self._moveitpy_signals
            if sig is not None:
                if self._motion_sig_connected and hasattr(sig, "motionResultChanged"):
                    try:
                        sig.motionResultChanged.disconnect(self._on_motion_result)
                    except Exception:
                        pass
                if self._traj_sig_connected:
                    if hasattr(sig, "plannedTrajectoryChanged"):
                        try:
                            sig.plannedTrajectoryChanged.disconnect(self._on_planned_traj_changed)
                        except Exception:
                            pass
                    if hasattr(sig, "executedTrajectoryChanged"):
                        try:
                            sig.executedTrajectoryChanged.disconnect(self._on_executed_traj_changed)
                        except Exception:
                            pass
                if self._clear_sig_connected and hasattr(sig, "trajCacheClearChanged"):
                    try:
                        sig.trajCacheClearChanged.disconnect(self._on_traj_cache_clear)
                    except Exception:
                        pass
        except Exception:
            pass

        self._motion_sig_connected = False
        self._traj_sig_connected = False
        self._clear_sig_connected = False

        if self._log_handler is not None:
            try:
                _LOG.removeHandler(self._log_handler)
            except Exception:
                pass
            self._log_handler = None

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
        self._pending_planned_step.clear()
        self._pending_executed_step.clear()
        self._last_planned_any = None
        self._last_executed_any = None

        self._wait_active = False
        self._wait_seg = ""
        self._wait_deadline_ms = 0
        self._wait_last_result = ""

        if not self._prepare_run():
            self.notifyError.emit(self._error_msg or "Prepare failed")
            self._cleanup()
            return

        self._machine = self._build_machine()
        self._machine.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._stop_requested:
            return
        self._stop_requested = True
        self._stopped = True

        # cancel wait
        self._wait_active = False
        self._wait_seg = ""

        try:
            self._ros.moveit_stop()
        except Exception:
            pass

        try:
            fn = getattr(self._ros, "moveit_set_segment", None)
            if callable(fn):
                fn("")
        except Exception:
            pass

        QtCore.QTimer.singleShot(0, lambda: self._signal_error("Gestoppt"))

    # ---------------- Retry hook ----------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        return False

    # ---------------- Hooks ----------------

    def _prepare_run(self) -> bool:
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        raise NotImplementedError

    def _on_segment_ok(self, seg_name: str) -> None:
        self._snapshot_trajs_for_segment(seg_name)

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        return True

    # ---------------- Traj payload ----------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        planned = self._jt_by_segment_yaml(which="planned")
        executed = self._jt_by_segment_yaml(which="executed")
        return planned, executed

    def _jt_by_segment_yaml(self, *, which: str) -> Dict[str, Any]:
        src = self._planned_by_segment if which == "planned" else self._executed_by_segment
        segments: Dict[str, Any] = {}
        for seg in SEG_ORDER:
            jt = src.get(seg)
            seg_yaml = self._jt_dict_to_segment_yaml(jt)
            if seg_yaml is not None:
                segments[seg] = seg_yaml
        return {"version": 1, "segments": segments}

    @staticmethod
    def _jt_dict_to_segment_yaml(jt: Any) -> Optional[Dict[str, Any]]:
        if not (
            isinstance(jt, dict)
            and isinstance(jt.get("joint_names"), list)
            and isinstance(jt.get("points"), list)
        ):
            return None
        jn = [str(x) for x in (jt.get("joint_names") or [])]
        if not jn:
            return None

        out_pts: List[Dict[str, Any]] = []
        for p in (jt.get("points") or []):
            if not isinstance(p, dict):
                continue
            pos = p.get("positions") or []
            if not isinstance(pos, list) or len(pos) != len(jn):
                continue

            tfs = p.get("time_from_start")
            sec, nsec = 0, 0
            if isinstance(tfs, dict):
                sec, nsec = int(tfs.get("sec", 0)), int(tfs.get("nanosec", 0))
            elif isinstance(tfs, (list, tuple)) and len(tfs) == 2:
                sec, nsec = int(tfs[0]), int(tfs[1])

            out_pts.append(
                {
                    "positions": [float(x) for x in pos],
                    "time_from_start": [int(sec), int(nsec)],
                }
            )
        if not out_pts:
            return None
        return {"joint_names": jn, "points": out_pts}
