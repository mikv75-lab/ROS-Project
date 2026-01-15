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
    """
    Gemeinsame Basis für Validate / Optimize / Execute.

    RESULT CONTRACT (RunResult payload, strict keys always present):
      - notifyFinished emits dict payload:
          {
            "planned_run":  {"traj": <JTBySegment yaml dict>, "tcp": <Draft yaml dict or {}>},
            "executed_run": {"traj": <JTBySegment yaml dict>, "tcp": <Draft yaml dict or {}>},
            "fk_meta": {...},
            "eval": {...},
            "valid": bool,
            "invalid_reason": str,
            "urdf_xml": str,
            "srdf_xml": str,
          }

    Retry (STRICT, NEW):
      - Soft execution errors (default: ERROR:EXEC ...) trigger retry of ONLY the last motion command.
      - No segment replay in Base. Subclasses must implement _on_retry_last_motion().
      - After max retries: notifyError with prefix 'RETRY_EXHAUSTED:' so UI can show Retry/Abort dialog.
    """

    ROLE = "process"

    notifyFinished = QtCore.pyqtSignal(object)  # emits RunResult payload dict
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    _sig_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    # Retry policy
    # - Soft execution errors are retried by re-sending ONLY the last motion command (no segment replay).
    # - After max_retries attempts => error with prefix 'RETRY_EXHAUSTED:' for UI dialog.
    _RETRY_DELAY_MS = 250

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

        # per-run snapshot stores
        self._planned_by_segment: Dict[str, Any] = {}
        self._executed_by_segment: Dict[str, Any] = {}

        # per-segment event buffers
        self._planned_steps_by_segment: Dict[str, List[Any]] = {}
        self._executed_steps_by_segment: Dict[str, List[Any]] = {}

        # optional: enforce ordering PLANNED -> EXECUTED within a segment
        self._seen_planned_ok: Dict[str, bool] = {}

        # --- MoveItPy handles ---
        self._moveitpy = getattr(self._ros, "moveitpy", None)
        self._moveitpy_signals = getattr(self._moveitpy, "signals", None)

        # --- cached last trajectories ---
        self._last_planned_any: Any = None
        self._last_executed_any: Any = None
        self._traj_sig_connected: bool = False
        self._motion_sig_connected: bool = False
        self._clear_sig_connected: bool = False

        # forward our internal logger to UI
        self._log_handler = _QtSignalHandler(self)
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
        """
        Reset all segment-local capture state.

        Needed for:
          - entering a segment
          - boundary clear (trajCacheClearChanged)
        NOTE: for 'retry last motion' we intentionally DO NOT clear prior steps.
        """
        self._last_planned_any = None
        self._last_executed_any = None
        self._seen_planned_ok[seg_name] = False

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

        # optional: tag segment into node
        try:
            fn = getattr(self._ros, "moveit_set_segment", None)
            if callable(fn):
                fn(seg_name)
        except Exception:
            pass

        self._on_enter_segment(seg_name)

    def _is_soft_error(self, res: str) -> bool:
        """
        Soft errors = retryable without aborting the whole run.
        Default policy (strict):
          - execution error => soft (usually start-state mismatch / controller race)
          - everything else => hard
        """
        r = (res or "").strip()
        return r.startswith("ERROR:EXEC")

    def _make_retry_exhausted_msg(self, seg: str, res: str) -> str:
        return (
            f"RETRY_EXHAUSTED: {res} seg={seg} "
            f"(attempts={int(self._retry_count)}/{int(self._max_retries)})"
        )

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested or not self._current_state:
            return

        res = (result or "").strip()
        if not res:
            return

        if res.startswith("ERROR"):
            # Best-effort: stop any in-flight execution request
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
                    # IMPORTANT: this must resend ONLY the last command (pose/joint/whatever),
                    # not restart the segment logic.
                    ok = bool(self._on_retry_last_motion(seg, int(self._retry_count), str(res)))
                except Exception:
                    ok = False

                if ok:
                    # small backoff so current RobotState can catch up
                    QtCore.QTimer.singleShot(self._RETRY_DELAY_MS, lambda: None)
                    return

                # If subclass can't retry last motion => abort deterministically
                self._signal_error(self._make_retry_exhausted_msg(seg, res))
                return

            if soft:
                # retries exhausted
                self._signal_error(self._make_retry_exhausted_msg(seg, res))
            else:
                # hard error => abort immediately
                self._signal_error(f"{res} seg={seg}")
            return

        if res.startswith("PLANNED:OK"):
            self._seen_planned_ok[self._current_state] = True
            return

        if res.startswith("EXECUTED:OK"):
            if not self._seen_planned_ok.get(self._current_state, False):
                self._signal_error(
                    f"Segment '{self._current_state}': EXECUTED:OK ohne vorheriges PLANNED:OK. "
                    "Contract violated / UI-Start mid-stream."
                )
                return

            self._snapshot_step_for_segment(self._current_state)

            if self._error_msg:
                return

            if self._should_transition_on_ok(self._current_state, res):
                self._on_segment_ok(self._current_state)
                QtCore.QTimer.singleShot(0, self._sig_done.emit)

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
        """
        Best-effort boundary clear hook.
        If MoveItPyBridge exposes trajCacheClearChanged (Empty -> Qt signal),
        we clear our caches AND per-segment step buffers for the current segment.
        """
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
        self._last_planned_any = None
        self._last_executed_any = None

        # also clear step buffers + ordering for current segment
        if self._current_state:
            self._reset_segment_capture(self._current_state, clear_steps=True)

    @QtCore.pyqtSlot(object)
    def _on_planned_traj_changed(self, obj: object) -> None:
        self._last_planned_any = obj

    @QtCore.pyqtSlot(object)
    def _on_executed_traj_changed(self, obj: object) -> None:
        self._last_executed_any = obj

    # ---------------- Snapshot helpers ----------------

    def _snapshot_step_for_segment(self, seg_name: str) -> None:
        """
        STRICT pairing with bounded wait:
        - wait briefly for executed/planned to arrive (DDS cross-topic order not guaranteed)
        - still fails deterministically if missing after timeout
        """
        import time

        def _get_pair():
            planned_raw, executed_raw = self._get_step_sources()
            return self._jt_from_any(planned_raw), self._jt_from_any(executed_raw)

        planned, executed = _get_pair()

        if executed is None or planned is None:
            deadline = time.monotonic() + 0.35
            while time.monotonic() < deadline:
                time.sleep(0.01)
                planned2, executed2 = _get_pair()
                if planned is None and planned2 is not None:
                    planned = planned2
                if executed is None and executed2 is not None:
                    executed = executed2
                if planned is not None and executed is not None:
                    break

        if planned is None or executed is None:
            self._signal_error(
                f"Trajectory capture missing for segment '{seg_name}': "
                f"planned={'ok' if planned is not None else 'None'}, "
                f"executed={'ok' if executed is not None else 'None'}. "
                "Requirement: EXECUTED:OK must have a matching last PLANNED + last EXECUTED."
            )
            return

        self._planned_steps_by_segment.setdefault(seg_name, []).append(copy.deepcopy(planned))
        self._executed_steps_by_segment.setdefault(seg_name, []).append(copy.deepcopy(executed))

    def _snapshot_trajs_for_segment(self, seg_name: str) -> None:
        """
        Build the final per-segment planned/executed trajectories.
        STRICT:
          - planned and executed are treated symmetrically.
          - We only store a segment if BOTH exist and were captured as pairs.
        """
        planned_steps = self._planned_steps_by_segment.get(seg_name) or []
        executed_steps = self._executed_steps_by_segment.get(seg_name) or []

        if not planned_steps or not executed_steps:
            self._signal_error(
                f"Segment '{seg_name}': no paired trajectory steps captured. "
                "Expected at least one EXECUTED:OK with both planned+executed cached."
            )
            return

        if len(planned_steps) != len(executed_steps):
            self._signal_error(
                f"Segment '{seg_name}': planned/executed step count mismatch "
                f"(planned={len(planned_steps)}, executed={len(executed_steps)}). "
                "This indicates asymmetrical capture; refusing to persist."
            )
            return

        planned = self._concat_joint_traj_dicts(planned_steps)
        executed = self._concat_joint_traj_dicts(executed_steps)

        if planned is None or executed is None:
            self._signal_error(
                f"Segment '{seg_name}': concat failed "
                f"(planned={'ok' if planned is not None else 'None'}, "
                f"executed={'ok' if executed is not None else 'None'}). "
                "Refusing to persist partial data."
            )
            return

        self._planned_by_segment[seg_name] = planned
        self._executed_by_segment[seg_name] = executed

    def _get_step_sources(self) -> Tuple[Any, Any]:
        """
        Preferred source of truth (race-free):
          - RosBridge.moveit_planned_trajectory()
          - RosBridge.moveit_executed_trajectory()
        Only if those methods do not exist, fall back to cached last_* from Qt signals.
        """
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

        merged_pts = []
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
        _LOG.error(f"Statemachine Error: {msg}")
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()
        self._rr.set_planned(traj=planned)
        self._rr.set_executed(traj=executed)
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    def _on_error(self) -> None:
        """
        STOP FIX: Bei Fehlern Bewegung sofort stoppen und aufräumen.
        """
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
        self._seen_planned_ok.clear()
        self._last_planned_any = None
        self._last_executed_any = None

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

    # ---------------- Retry hook (NEW) ----------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        """
        Subclass hook: resend ONLY the last motion command (the one that just failed).

        MUST NOT:
          - rebuild the whole segment
          - clear already captured per-segment steps

        Return:
          True  -> retry was triggered (command resent)
          False -> cannot retry (Base will abort with RETRY_EXHAUSTED)
        """
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
