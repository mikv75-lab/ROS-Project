# -*- coding: utf-8 -*-
# File: tabs/process/process_run_statemachine.py
from __future__ import annotations

from typing import Optional, List, Dict, Any, Tuple
import logging

import yaml

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_LOG = logging.getLogger("app.tabs.process.run_statemachine")

SEG_ORDER = ["MOVE_PREDISPENSE", "MOVE_RECIPE", "MOVE_RETREAT", "MOVE_HOME"]


class _QtSignalHandler(logging.Handler):
    """Logging → Qt-Signal-Weiterleitung (für ProcessTab-Log)."""

    def __init__(self, owner: "ProcessExecuteStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            self._owner.logMessage.emit(msg)
        except Exception:
            pass


class ProcessExecuteStatemachine(QtCore.QObject):
    """
    Execute-StateMachine (Execute-only via MoveItPy)

    - Lädt Run-YAML (joints + segments[name+points])
    - Baut pro Segment eine JointTrajectory
    - Führt jedes Segment via MoveItPy aus (execute-only, kein move_home, keine Pose-Steps)

    Ablauf:
      MOVE_PREDISPENSE -> MOVE_RECIPE -> MOVE_RETREAT -> MOVE_HOME -> FINISHED
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
        run_yaml_path: str,
        bridge: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
    ) -> None:
        super().__init__(parent)

        self._run_yaml_path = run_yaml_path
        self._ui = bridge  # UIBridge/Bridge (wie in Validate)
        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._stop_requested: bool = False
        self._stopped: bool = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0

        self._joints: List[str] = []
        self._segments_pts: Dict[str, List[Dict[str, Any]]] = {}
        self._segments_traj: Dict[str, JointTrajectory] = {}

        # MoveItPy-Bridge/Signals wie bei Validate
        self._moveitpy_bridge = getattr(self._ui, "moveitpy_bridge", None)
        self._moveitpy_signals = getattr(self._moveitpy_bridge, "signals", None)

        if self._moveitpy_signals is None or not hasattr(self._moveitpy_signals, "motionResultChanged"):
            _LOG.warning("Run: moveitpy_bridge.signals.motionResultChanged nicht gefunden – Feedback kann fehlen.")

        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        _LOG.addHandler(self._log_handler)

        if self._moveitpy_signals is not None:
            try:
                self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)  # type: ignore[attr-defined]
            except Exception:
                pass

        _LOG.info("ProcessRunStatemachine init: run_yaml=%s", run_yaml_path)

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        _LOG.info("Run: start()")
        self._stop_requested = False
        self._stopped = False
        self._error_msg = None
        self._retry_count = 0
        self._current_state = ""
        self._segments_traj.clear()

        if not self._load_run_yaml(self._run_yaml_path):
            msg = self._error_msg or f"Run-YAML '{self._run_yaml_path}' konnte nicht geladen werden."
            _LOG.error("Run: %s", msg)
            self.notifyError.emit(msg)
            return

        self._build_all_segment_trajectories()

        machine = QStateMachine(self)
        self._machine = machine

        states: Dict[str, QState] = {}
        for name in SEG_ORDER:
            states[name] = QState(machine)

        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        # Initialstate = erster existierender/erlaubter State
        first = self._first_segment_state()
        if first is None:
            msg = "Run: Keine Segmente zum Ausführen gefunden."
            _LOG.error(msg)
            self.notifyError.emit(msg)
            return

        machine.setInitialState(states[first])

        # lineare Übergänge (done -> nächstes Segment / oder finished)
        for i, name in enumerate(SEG_ORDER):
            st = states[name]
            st.addTransition(self._sig_done, states[SEG_ORDER[i + 1]] if i + 1 < len(SEG_ORDER) else s_finished)
            st.addTransition(self._sig_error, s_error)

        # entered handlers
        for name in SEG_ORDER:
            states[name].entered.connect(lambda n=name: self._on_state_execute(n))

        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        machine.start()
        _LOG.info("Run: StateMachine gestartet (initial=%s).", first)

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        _LOG.info("Run: request_stop()")
        self._stop_requested = True
        self._stopped = True
        # MoveIt stoppen (falls vorhanden)
        self._call_ui("moveit_stop")

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        _LOG.info("Run: stop()")
        self.request_stop()

    # ------------------------------------------------------------------ #
    # YAML → Segmente
    # ------------------------------------------------------------------ #

    def _load_run_yaml(self, path: str) -> bool:
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self._error_msg = f"Run-YAML konnte nicht geladen werden: {e}"
            return False

        joints = data.get("joints") or []
        if not isinstance(joints, list) or not joints:
            self._error_msg = "Run-YAML enthält keine gültige 'joints'-Liste."
            return False

        segments_raw = data.get("segments") or []
        if not isinstance(segments_raw, list) or not segments_raw:
            self._error_msg = "Run-YAML enthält keine 'segments'-Liste."
            return False

        seg_map: Dict[str, List[Dict[str, Any]]] = {k: [] for k in SEG_ORDER}

        for seg in segments_raw:
            name = (seg.get("name") or "").strip().upper()
            pts = seg.get("points") or []
            if name in seg_map and isinstance(pts, list) and pts:
                seg_map[name] = pts

        self._joints = list(joints)
        self._segments_pts = seg_map

        _LOG.info(
            "Run: YAML geladen joints=%d, segments={%s}",
            len(self._joints),
            ", ".join(f"{k}:{len(v)}" for k, v in self._segments_pts.items()),
        )
        return True

    def _build_all_segment_trajectories(self) -> None:
        self._segments_traj.clear()
        for name in SEG_ORDER:
            pts = self._segments_pts.get(name, [])
            if not pts:
                continue
            jt = self._segment_points_to_joint_traj(name, pts)
            if jt is not None and jt.points:
                self._segments_traj[name] = jt

    def _segment_points_to_joint_traj(self, seg_name: str, pts: List[Dict[str, Any]]) -> Optional[JointTrajectory]:
        n_j = len(self._joints)
        if n_j <= 0:
            return None

        # Zeit normalisieren: t0 -> 0
        try:
            t0 = float(pts[0].get("t", 0.0))
        except Exception:
            t0 = 0.0

        traj = JointTrajectory()
        traj.joint_names = list(self._joints)

        last_t = -1.0
        for p in pts:
            positions = p.get("positions") or []
            if not isinstance(positions, list) or len(positions) != n_j:
                continue

            try:
                t = float(p.get("t", 0.0)) - t0
            except Exception:
                continue

            # ensure monotonic increasing
            if t <= last_t:
                t = last_t + 0.001
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in positions]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)

        if len(traj.points) < 2:
            _LOG.warning("Run: Segment %s hat zu wenig gültige Punkte.", seg_name)
            return None

        _LOG.info("Run: Segment %s -> JointTrajectory points=%d", seg_name, len(traj.points))
        return traj

    # ------------------------------------------------------------------ #
    # State execution
    # ------------------------------------------------------------------ #

    def _first_segment_state(self) -> Optional[str]:
        for name in SEG_ORDER:
            if self._skip_home and name == "MOVE_HOME":
                continue
            if self._segments_traj.get(name) is not None:
                return name
        return None

    def _next_segment(self, current: str) -> Optional[str]:
        try:
            idx = SEG_ORDER.index(current)
        except ValueError:
            return None
        for j in range(idx + 1, len(SEG_ORDER)):
            name = SEG_ORDER[j]
            if self._skip_home and name == "MOVE_HOME":
                continue
            if self._segments_traj.get(name) is not None:
                return name
        return None

    def _emit_state(self, name: str) -> None:
        self._current_state = name
        self.stateChanged.emit(name)
        self.logMessage.emit(f"Run: State={name}")

    def _on_state_execute(self, seg_name: str) -> None:
        # Skip wenn kein Segment
        if self._skip_home and seg_name == "MOVE_HOME":
            self._emit_state(seg_name)
            _LOG.info("Run: skip_home -> überspringe MOVE_HOME")
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        jt = self._segments_traj.get(seg_name)
        if jt is None:
            # Segment fehlt -> direkt weiter
            self._emit_state(seg_name)
            _LOG.info("Run: Segment %s leer -> überspringe", seg_name)
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        self._retry_count = 0
        self._emit_state(seg_name)

        if self._stop_requested:
            self._error_msg = "Prozess durch Benutzer gestoppt."
            QtCore.QTimer.singleShot(0, self._sig_error.emit)
            return

        ok = self._execute_joint_traj(jt)
        if not ok:
            msg = self._error_msg or f"Execute failed: {seg_name}"
            self._signal_error(msg)

    def _execute_joint_traj(self, jt: JointTrajectory) -> bool:
        """
        Erwartet eine UI-Methode, eine JointTrajectory auszuführen.
        Unterstützte Namen (probiert in Reihenfolge):
          - moveit_execute_joint_trajectory(jt)
          - moveit_execute_trajectory(jt)
          - moveit_execute(jt)
        """
        for fn in ("moveit_execute_joint_trajectory", "moveit_execute_trajectory", "moveit_execute"):
            if hasattr(self._ui, fn):
                try:
                    getattr(self._ui, fn)(jt)
                    _LOG.info("Run: execute via %s()", fn)
                    return True
                except Exception as e:
                    self._error_msg = f"{fn} failed: {e}"
                    _LOG.exception("Run: %s", self._error_msg)
                    return False

        self._error_msg = "UIBridge hat keine Execute-Methode (moveit_execute_*)."
        _LOG.error("Run: %s", self._error_msg)
        return False

    def _signal_error(self, msg: str) -> None:
        if not msg:
            msg = "Unbekannter Run-Fehler."
        if self._error_msg is None:
            self._error_msg = msg
        _LOG.error("Run: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    # ------------------------------------------------------------------ #
    # Motion result feedback
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        if self._machine is None or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if not self._current_state:
            return

        _LOG.info("Run: motion_result=%s (state=%s)", result, self._current_state)

        if result.startswith("ERROR"):
            if self._retry_count < self._max_retries:
                self._retry_count += 1
                _LOG.warning("Run: Retry %d/%d für %s", self._retry_count, self._max_retries, self._current_state)
                jt = self._segments_traj.get(self._current_state)
                if jt is not None:
                    self._execute_joint_traj(jt)
                else:
                    self._signal_error(result)
            else:
                self._signal_error(result)
            return

        if not result.startswith("EXECUTED:OK"):
            return

        # Segment ok -> nächstes Segment oder done
        nxt = self._next_segment(self._current_state)
        if nxt is None:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
        else:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ------------------------------------------------------------------ #
    # Final states
    # ------------------------------------------------------------------ #

    def _on_state_error(self) -> None:
        msg = self._error_msg or "Unbekannter Fehler."
        self.notifyError.emit(msg)
        self._cleanup()

    def _on_state_finished(self) -> None:
        status = "stopped" if self._stopped else "finished"
        self.notifyFinished.emit(
            {
                "status": status,
                "planned_traj": self._call_ui("moveit_planned_trajectory"),
                "executed_traj": self._call_ui("moveit_executed_trajectory"),
            }
        )
        self._cleanup()

    # ------------------------------------------------------------------ #
    # Helpers / cleanup
    # ------------------------------------------------------------------ #

    def _call_ui(self, name: str, *args, **kwargs):
        if hasattr(self._ui, name):
            try:
                return getattr(self._ui, name)(*args, **kwargs)
            except Exception:
                return None
        return None

    def _cleanup(self) -> None:
        try:
            if self._moveitpy_signals is not None:
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
