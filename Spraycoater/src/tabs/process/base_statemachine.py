# -*- coding: utf-8 -*-
# File: tabs/process/base_statemachine.py
from __future__ import annotations

import copy
import logging
from typing import Optional, Dict, Any, List, Tuple

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from model.recipe.recipe_run_result import RunResult  # <-- adjust if needed

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

    RESULT CONTRACT (RunResult payload, strict keys always present):
      - notifyFinished emits dict payload:
          {
            "planned_run":  {"traj": <JTBySegment yaml dict>, "tcp": <Draft yaml dict or {}>},
            "executed_run": {"traj": <JTBySegment yaml dict>, "tcp": <Draft yaml dict or {}>},
            "fk_meta": {...}
          }

    WICHTIGER FIX (2026-01):
      - Trajectory Capture ist in der Praxis oft NICHT zuverlässig über "ros.moveit_planned_trajectory()"
        (je nach Bridge/Namespace/Implementation).
      - Deshalb cachen wir zusätzlich (best-effort) die letzten Trajektorien über:
          moveitpy.signals.plannedTrajectoryChanged
          moveitpy.signals.executedTrajectoryChanged
        und nutzen diese Caches bei EXECUTED:OK.
    """

    ROLE = "process"

    notifyFinished = QtCore.pyqtSignal(object)  # emits RunResult payload dict (see contract)
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

        # per-run snapshot stores (final per segment, typically merged/concat)
        self._planned_by_segment: Dict[str, Any] = {}
        self._executed_by_segment: Dict[str, Any] = {}

        # per-segment event buffers (captures every EXECUTED:OK inside same segment)
        self._planned_steps_by_segment: Dict[str, List[Any]] = {}
        self._executed_steps_by_segment: Dict[str, List[Any]] = {}

        # --- MoveItPy handles ---
        self._moveitpy = getattr(self._ros, "moveitpy", None)
        self._moveitpy_signals = getattr(self._moveitpy, "signals", None)

        # --- NEW: cached last trajectories (best-effort) ---
        self._last_planned_any: Any = None
        self._last_executed_any: Any = None
        self._traj_sig_connected: bool = False
        self._motion_sig_connected: bool = False

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

        # connect trajectory cache signals (best effort)
        self._connect_traj_cache_signals()

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

    # ---------------- Traj payload (JTBySegment YAML dicts) ----------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        planned = self._jt_by_segment_yaml(which="planned")
        executed = self._jt_by_segment_yaml(which="executed")
        return planned, executed

    def _jt_by_segment_yaml(self, *, which: str) -> Dict[str, Any]:
        if which not in ("planned", "executed"):
            raise ValueError(f"_jt_by_segment_yaml: invalid which={which}")

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
        if not (isinstance(jt, dict) and isinstance(jt.get("joint_names"), list) and isinstance(jt.get("points"), list)):
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
            sec = 0
            nsec = 0
            if isinstance(tfs, dict):
                try:
                    sec = int(tfs.get("sec", 0))
                    nsec = int(tfs.get("nanosec", 0))
                except Exception:
                    sec, nsec = 0, 0
            elif isinstance(tfs, (list, tuple)) and len(tfs) == 2:
                try:
                    sec = int(tfs[0])
                    nsec = int(tfs[1])
                except Exception:
                    sec, nsec = 0, 0

            out_pts.append(
                {
                    "positions": [float(x) for x in pos],
                    "time_from_start": [int(sec), int(nsec)],
                }
            )

        if not out_pts:
            return None

        return {"joint_names": jn, "points": out_pts}

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
                self._on_enter_segment(self._current_state)
            else:
                self._signal_error(result)
            return

        if result.startswith("EXECUTED:OK"):
            self._snapshot_step_for_segment(self._current_state)

            if self._should_transition_on_ok(self._current_state, result):
                self._on_segment_ok(self._current_state)
                QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ---------------- Trajectory cache signals (NEW) ----------------

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

    @QtCore.pyqtSlot(object)
    def _on_planned_traj_changed(self, obj: object) -> None:
        self._last_planned_any = obj

    @QtCore.pyqtSlot(object)
    def _on_executed_traj_changed(self, obj: object) -> None:
        self._last_executed_any = obj

    # ---------------- Snapshot helpers ----------------

    def _snapshot_step_for_segment(self, seg_name: str) -> None:
        planned_raw, executed_raw = self._get_step_sources()

        planned = self._jt_from_any(planned_raw)
        executed = self._jt_from_any(executed_raw)

        if planned is not None:
            self._planned_steps_by_segment.setdefault(seg_name, []).append(copy.deepcopy(planned))
        if executed is not None:
            self._executed_steps_by_segment.setdefault(seg_name, []).append(copy.deepcopy(executed))

    def _snapshot_trajs_for_segment(self, seg_name: str) -> None:
        planned_steps = self._planned_steps_by_segment.get(seg_name) or []
        executed_steps = self._executed_steps_by_segment.get(seg_name) or []

        planned = self._concat_joint_traj_dicts(planned_steps) if planned_steps else None
        executed = self._concat_joint_traj_dicts(executed_steps) if executed_steps else None

        # fallback: if no steps were captured, try to pull last-known directly
        if planned is None or executed is None:
            pr, er = self._get_step_sources()
            if planned is None:
                planned = self._jt_from_any(pr)
            if executed is None:
                executed = self._jt_from_any(er)

        if planned is not None:
            self._planned_by_segment[seg_name] = planned
        if executed is not None:
            self._executed_by_segment[seg_name] = executed

    def _get_step_sources(self) -> Tuple[Any, Any]:
        """
        Try multiple sources for planned/executed trajectory objects.

        Priority:
          0) cached signals plannedTrajectoryChanged/executedTrajectoryChanged
          1) ros.moveit_planned_trajectory() / ros.moveit_executed_trajectory()
          2) moveitpy wrapper common attributes
        """
        # 0) cached
        if self._last_planned_any is not None or self._last_executed_any is not None:
            return self._last_planned_any, self._last_executed_any

        # 1) ros facade calls
        planned = self._safe_call("moveit_planned_trajectory")
        executed = self._safe_call("moveit_executed_trajectory")
        if planned is not None or executed is not None:
            return planned, executed

        # 2) moveitpy object attributes
        mp = self._moveitpy
        for p_attr, e_attr in (
            ("last_planned", "last_executed"),
            ("planned_trajectory", "executed_trajectory"),
            ("planned", "executed"),
            ("last_plan_result", "last_exec_result"),
            ("last_plan", "last_exec"),
        ):
            try:
                p = getattr(mp, p_attr, None) if mp is not None else None
                e = getattr(mp, e_attr, None) if mp is not None else None
                if p is not None or e is not None:
                    return p, e
            except Exception:
                pass

        return None, None

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
        if obj is None:
            return None
        if _depth > _max_depth:
            return None

        # JointTrajectory msg
        try:
            if hasattr(obj, "joint_names") and hasattr(obj, "points"):
                out = cls._jt_msg_to_dict(obj)
                if out:
                    return out
        except Exception:
            pass

        # RobotTrajectory msg
        try:
            jt = getattr(obj, "joint_trajectory", None)
            if jt is not None:
                out = cls._jt_msg_to_dict(jt)
                if out:
                    return out
        except Exception:
            pass

        # dict JT
        if isinstance(obj, dict) and obj:
            try:
                if isinstance(obj.get("joint_names"), list) and isinstance(obj.get("points"), list):
                    return obj
            except Exception:
                pass

            for k in (
                "joint_trajectory",
                "jointTrajectory",
                "jt",
                "trajectory",
                "robot_trajectory",
                "robotTrajectory",
                "planned",
                "executed",
                "plan",
                "execution",
                "result",
                "data",
                "solution",
                "response",
            ):
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

        # Common attributes
        for attr in (
            "trajectory",
            "robot_trajectory",
            "robotTrajectory",
            "planned_trajectory",
            "executed_trajectory",
            "plan",
            "result",
            "data",
            "solution",
            "response",
        ):
            try:
                v = getattr(obj, attr, None)
            except Exception:
                v = None
            out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
            if out:
                return out

        # scan lists/tuples
        if isinstance(obj, (list, tuple)) and obj:
            for v in obj:
                out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if out:
                    return out

        # scan object __dict__
        try:
            d = getattr(obj, "__dict__", None)
            if isinstance(d, dict) and d:
                for v in d.values():
                    out = cls._jt_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                    if out:
                        return out
        except Exception:
            pass

        return None

    # ---------------- concatenation (merged JT per segment) ----------------

    def _concat_joint_traj_dicts(self, items: List[Any]) -> Optional[Dict[str, Any]]:
        dicts = [d for d in (items or []) if isinstance(d, dict) and d.get("joint_names") and d.get("points")]
        if not dicts:
            return None

        base_names = list(dicts[0].get("joint_names") or [])
        if not base_names:
            return None

        def dur_to_ns(d: Any) -> int:
            if d is None:
                return 0
            if isinstance(d, dict):
                try:
                    return int(d.get("sec", 0)) * 1_000_000_000 + int(d.get("nanosec", 0))
                except Exception:
                    return 0
            if isinstance(d, (int, float)):
                return int(float(d) * 1_000_000_000)
            if isinstance(d, str):
                try:
                    return int(float(d) * 1_000_000_000)
                except Exception:
                    return 0
            return 0

        def ns_to_dur(ns: int) -> Dict[str, int]:
            ns = int(max(ns, 0))
            return {"sec": ns // 1_000_000_000, "nanosec": ns % 1_000_000_000}

        merged_pts: List[Dict[str, Any]] = []
        last_global_ns = 0
        t_offset_ns = 0

        for jt in dicts:
            names = list(jt.get("joint_names") or [])
            pts = list(jt.get("points") or [])
            if not pts:
                continue
            if names != base_names:
                continue

            local_times = [dur_to_ns(p.get("time_from_start")) if isinstance(p, dict) else 0 for p in pts]
            for i, p in enumerate(pts):
                if not isinstance(p, dict):
                    continue
                t_local_ns = local_times[i] if i < len(local_times) else 0
                t_global_ns = t_offset_ns + t_local_ns

                if merged_pts and t_global_ns <= last_global_ns:
                    t_global_ns = last_global_ns + 1_000_000  # +1ms

                q = copy.deepcopy(p)
                q["time_from_start"] = ns_to_dur(t_global_ns)

                if merged_pts and i == 0:
                    try:
                        if q.get("positions") == merged_pts[-1].get("positions"):
                            last_global_ns = t_global_ns
                            continue
                    except Exception:
                        pass

                merged_pts.append(q)
                last_global_ns = t_global_ns

            t_offset_ns = last_global_ns

        if not merged_pts:
            return None

        return {"joint_names": base_names, "points": merged_pts}

    # ---------------- Final ----------------

    def _signal_error(self, msg: str) -> None:
        self._error_msg = msg
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()

        # HARD GUARD: if we captured nothing, fail here with a useful error.
        p_segs = planned.get("segments") if isinstance(planned, dict) else None
        e_segs = executed.get("segments") if isinstance(executed, dict) else None
        if not (isinstance(p_segs, dict) and p_segs) or not (isinstance(e_segs, dict) and e_segs):
            self.notifyError.emit(
                "Validate/Run finished, but trajectory capture is empty (segments={}). "
                "Check MoveItPyBridge: plannedTrajectoryChanged/executedTrajectoryChanged or ros.moveit_* accessors."
            )
            self._cleanup()
            return

        rr = RunResult()
        rr.set_planned(traj=planned)     # tcp keys exist already (empty dict)
        rr.set_executed(traj=executed)  # tcp keys exist already (empty dict)

        self.notifyFinished.emit(rr.to_process_payload())
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
            if self._moveitpy_signals is not None:
                if self._motion_sig_connected and hasattr(self._moveitpy_signals, "motionResultChanged"):
                    try:
                        self._moveitpy_signals.motionResultChanged.disconnect(self._on_motion_result)
                    except Exception:
                        pass
                if self._traj_sig_connected:
                    try:
                        if hasattr(self._moveitpy_signals, "plannedTrajectoryChanged"):
                            self._moveitpy_signals.plannedTrajectoryChanged.disconnect(self._on_planned_traj_changed)
                    except Exception:
                        pass
                    try:
                        if hasattr(self._moveitpy_signals, "executedTrajectoryChanged"):
                            self._moveitpy_signals.executedTrajectoryChanged.disconnect(self._on_executed_traj_changed)
                    except Exception:
                        pass
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
