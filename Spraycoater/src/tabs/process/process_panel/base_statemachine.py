# -*- coding: utf-8 -*-
# File: src/tabs/process/base_statemachine.py
from __future__ import annotations

import json
import logging
from dataclasses import dataclass
from typing import Optional, Dict, Any, Tuple

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState
from PyQt6.sip import isdeleted

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


# =============================================================================
# Helpers / Contracts (STRICT)
# =============================================================================

def _norm(s: Any) -> str:
    return str(s or "").strip()


def _as_dict(x: Any) -> Dict[str, Any]:
    return x if isinstance(x, dict) else {}


@dataclass(frozen=True)
class ReqKey:
    """
    Correlation key (STRICT) for MoveItPy Variant-A envelope.

    MUST match:
      - motion_result JSON: {"key": {"run": "...", "id": <int>, "seg": "...", "op": "..."}, "status": "...", ...}
      - RobotTrajectoryMsg.joint_trajectory.header.frame_id: JSON string containing at least the SAME "key" object
        (either {"key":{...}} or just {run,id,seg,op}).
    """
    run: str
    id: int
    seg: str
    op: str

    def as_tuple(self) -> Tuple[str, int, str, str]:
        return (self.run, int(self.id), self.seg, self.op)

    def token_base(self) -> str:
        return f"run={self.run}|id={int(self.id)}|seg={self.seg}|op={self.op}"


def _parse_motion_result(text: str) -> Tuple[ReqKey, str, Dict[str, Any]]:
    """
    Parse motion_result string into (ReqKey, status, full_dict).
    """
    raw = _norm(text)
    if not raw:
        raise ValueError("motion_result empty")

    try:
        d = json.loads(raw)
    except Exception as e:
        raise ValueError(f"motion_result not valid JSON: {e!r}")

    if not isinstance(d, dict):
        raise ValueError("motion_result JSON is not an object")

    key = _as_dict(d.get("key"))
    status = _norm(d.get("status"))
    if not key or not status:
        raise ValueError("motion_result missing key/status")

    run = _norm(key.get("run"))
    seg = _norm(key.get("seg"))
    op = _norm(key.get("op"))
    rid = key.get("id")

    if not run:
        raise ValueError("motion_result.key.run empty")
    if not isinstance(rid, int):
        raise ValueError(f"motion_result.key.id must be int, got {type(rid).__name__}")
    if not seg:
        raise ValueError("motion_result.key.seg empty")
    if not op:
        raise ValueError("motion_result.key.op empty")

    return ReqKey(run=run, id=int(rid), seg=seg, op=op), status, d


def _parse_key_from_traj_header(obj: Any) -> ReqKey:
    """
    Extract ReqKey from RobotTrajectoryMsg.joint_trajectory.header.frame_id (JSON).
    """
    jt = getattr(obj, "joint_trajectory", None)
    if jt is None:
        raise ValueError("trajectory has no joint_trajectory")
    hdr = getattr(jt, "header", None)
    if hdr is None:
        raise ValueError("trajectory.joint_trajectory has no header")
    frame_id = _norm(getattr(hdr, "frame_id", ""))
    if not frame_id:
        raise ValueError("trajectory header.frame_id empty")

    try:
        d = json.loads(frame_id)
    except Exception as e:
        raise ValueError(f"trajectory key JSON invalid: {e!r}")
    if not isinstance(d, dict):
        raise ValueError("trajectory key JSON is not an object")

    if "key" in d and isinstance(d.get("key"), dict):
        k = _as_dict(d.get("key"))
    else:
        k = d

    run = _norm(k.get("run"))
    seg = _norm(k.get("seg"))
    op = _norm(k.get("op"))
    rid = k.get("id")

    if not run:
        raise ValueError("trajectory key.run empty")
    if not isinstance(rid, int):
        raise ValueError(f"trajectory key.id must be int, got {type(rid).__name__}")
    if not seg:
        raise ValueError("trajectory key.seg empty")
    if not op:
        raise ValueError("trajectory key.op empty")

    return ReqKey(run=run, id=int(rid), seg=seg, op=op)


def _is_empty_robot_trajectory(obj: Any) -> bool:
    """
    Node may publish empty RobotTrajectory to clear latched state.
    Treat that as "ignore".
    """
    try:
        jt = getattr(obj, "joint_trajectory", None)
        if jt is None:
            return True
        names = list(getattr(jt, "joint_names", []) or [])
        pts = list(getattr(jt, "points", []) or [])
        return (len(names) == 0) and (len(pts) == 0)
    except Exception:
        return False


# =============================================================================
# Logging Handler -> Qt Signal
# =============================================================================

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


# =============================================================================
# BaseProcessStatemachine (COLLECT-ONLY, SegmentRunner-driven)
# =============================================================================

class BaseProcessStatemachine(QtCore.QObject):
    """
    Base class for Validate / Optimize / Execute — COLLECT-ONLY.

    Features:
      - Sequential segment state machine (SEG_ORDER).
      - Collects inbound MoveItPy signals (motion_result + trajectories).
      - **Automatic Deduplication**: On finish, ensures only the latest result/trajectory
        per logical step (Segment + Op) is retained, removing duplicates from retries.
    """

    ROLE = "process"

    notifyFinished = QtCore.pyqtSignal(object)  # emits RunResult payload dict
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
        run_result: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 0,
    ) -> None:
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

        # MoveItPy handles/signals (best-effort)
        self._moveitpy = None
        self._moveitpy_signals = None
        try:
            self._moveitpy = self._ros.moveitpy
            if self._moveitpy is not None:
                self._moveitpy_signals = self._moveitpy.signals
        except Exception:
            self._moveitpy = None
            self._moveitpy_signals = None

        # Keyed stashes (run, id, seg, op) -> Data
        # We collect ALL attempts here. Deduplication happens at the end.
        self._results: Dict[Tuple[str, int, str, str], Dict[str, Any]] = {}
        self._traj_planned: Dict[Tuple[str, int, str, str], Any] = {}
        self._traj_executed: Dict[Tuple[str, int, str, str], Any] = {}
        self._traj_optimized: Dict[Tuple[str, int, str, str], Any] = {}

        # logging handler -> Qt
        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        self._log_handler.setLevel(logging.INFO)
        _LOG.addHandler(self._log_handler)

        # connect inbound
        self._connect_motion_result_signal()
        self._connect_traj_signals()

        _ = max_retries

    # ------------------------------------------------------------------
    # Subclass control API (explicit transitions)
    # ------------------------------------------------------------------

    def current_segment(self) -> str:
        return str(self._current_state or "")

    def segment_done(self) -> None:
        if self._machine is None or self._stop_requested or self._error_msg:
            return
        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    def segment_error(self, msg: str) -> None:
        self._signal_error(str(msg or "ERROR"))

    # ------------------------------------------------------------------
    # Machine wiring
    # ------------------------------------------------------------------

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
        _ = seg_name
        return True

    # ------------------------------------------------------------------
    # Segment lifecycle
    # ------------------------------------------------------------------

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
            return

    # ------------------------------------------------------------------
    # Inbound: motion_result + trajectories (Collect ALL)
    # ------------------------------------------------------------------

    def _connect_motion_result_signal(self) -> None:
        sig = self._moveitpy_signals
        if sig is None:
            return
        try:
            sig.motionResultChanged.connect(self._on_motion_result)
        except Exception:
            pass

    def _connect_traj_signals(self) -> None:
        sig = self._moveitpy_signals
        if sig is None:
            return
        try:
            sig.plannedTrajectoryChanged.connect(self._on_planned_traj_changed)
        except Exception:
            pass
        try:
            sig.executedTrajectoryChanged.connect(self._on_executed_traj_changed)
        except Exception:
            pass
        try:
            sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj_changed)
        except Exception:
            pass

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        if self._machine is None or self._stop_requested:
            return
        raw = _norm(text)
        if not raw:
            return

        try:
            key, status, d = _parse_motion_result(raw)
        except Exception as e:
            _LOG.warning("motion_result ignored: %s", e)
            return

        # Stash everything (dedupe later)
        try:
            self._results[key.as_tuple()] = dict(d)
        except Exception:
            self._results[key.as_tuple()] = {"raw": raw, "status": str(status)}

    @QtCore.pyqtSlot(object)
    def _on_planned_traj_changed(self, obj: object) -> None:
        self._stash_traj(kind="planned", obj=obj)

    @QtCore.pyqtSlot(object)
    def _on_executed_traj_changed(self, obj: object) -> None:
        self._stash_traj(kind="executed", obj=obj)

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj_changed(self, obj: object) -> None:
        self._stash_traj(kind="optimized", obj=obj)

    def _stash_traj(self, *, kind: str, obj: Any) -> None:
        if self._machine is None or self._stop_requested or self._error_msg:
            return
        if obj is None or _is_empty_robot_trajectory(obj):
            return

        try:
            key = _parse_key_from_traj_header(obj)
        except Exception as e:
            self._signal_error(f"{kind} trajectory missing/invalid key: {e}")
            return

        if kind == "planned":
            self._traj_planned[key.as_tuple()] = obj
        elif kind == "executed":
            self._traj_executed[key.as_tuple()] = obj
        else:
            self._traj_optimized[key.as_tuple()] = obj

    # ------------------------------------------------------------------
    # Data Deduplication
    # ------------------------------------------------------------------

    def _deduplicate_storage(self) -> None:
        """
        Filters all stored dictionaries (_results, _traj_*) to keep only the
        LATEST entry for each logical step (Segment + Operation).
        
        Logic:
          1. Uses a temporary dict mapping (seg, op) -> (full_key, value).
          2. Since Python dicts preserve insertion order, the last inserted
             item for a given (seg, op) overwrites previous ones (retries).
          3. Reconstructs the class dicts with only the unique/latest items.
        """
        self._results = self._filter_latest(self._results)
        self._traj_planned = self._filter_latest(self._traj_planned)
        self._traj_executed = self._filter_latest(self._traj_executed)
        self._traj_optimized = self._filter_latest(self._traj_optimized)

    def _filter_latest(self, source: Dict[Tuple, Any]) -> Dict[Tuple, Any]:
        if not source:
            return {}
        
        # Intermediate storage: (seg, op) -> (full_key, value)
        latest_map = {}
        
        # Iteration order ensures we process older entries first, newer last.
        for full_key, val in source.items():
            # full_key is (run, id, seg, op) -> indices 2 and 3 are seg, op
            seg, op = full_key[2], full_key[3]
            latest_map[(seg, op)] = (full_key, val)
            
        # Reconstruct dict with original key structure (full_key)
        return {k: v for k, v in latest_map.values()}

    # ------------------------------------------------------------------
    # Finalization & cleanup
    # ------------------------------------------------------------------

    def _on_finished(self) -> None:
        # 1. Clean up duplicates (retries) so RunResult gets only the latest data
        self._deduplicate_storage()

        try:
            fn = getattr(self._rr, "attach_recipe_context", None)
            if callable(fn):
                fn(self._recipe)
        except Exception:
            pass

        try:
            # self._rr (RunResult) will now read the cleaner self._traj_* dicts
            payload = self._rr.to_process_payload()
        except Exception:
            payload = {"ok": True}

        self.notifyFinished.emit(payload)
        self._cleanup()

    def _signal_error(self, msg: str) -> None:
        if self._error_msg:
            return
        self._error_msg = str(msg or "ERROR")
        _LOG.error("Statemachine Error: %s", self._error_msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    def _on_error(self) -> None:
        # Even on error, we might want to dedupe results for diagnostics,
        # but usually we want full history on error? 
        # Let's dedupe to keep output clean unless debugging.
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

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._stop_requested = False
        self._stop_emitted = False
        self._error_msg = None

        # Clear previous run data
        self._results.clear()
        self._traj_planned.clear()
        self._traj_executed.clear()
        self._traj_optimized.clear()

        if not self._prepare_run():
            self.notifyError.emit(self._error_msg or "Prepare failed")
            return

        self._machine = self._build_machine()
        self._machine.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._stop_requested:
            return
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
            if not self._error_msg:
                self._error_msg = "STOPPED"

            if self._machine is None:
                self.notifyError.emit(self._error_msg)
                self._cleanup()
                return

            QtCore.QTimer.singleShot(0, self._sig_error.emit)

    # ------------------------------------------------------------------
    # Hooks for subclasses
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        _ = seg_name
        raise NotImplementedError