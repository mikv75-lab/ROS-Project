# -*- coding: utf-8 -*-
# File: tabs/process/process_optimize_statemachine.py
from __future__ import annotations

from typing import Optional, List, Dict, Any
import logging

import yaml

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_LOG = logging.getLogger("app.tabs.process.optimize_statemachine")

SEG_ORDER = ["MOVE_PREDISPENSE", "MOVE_RECIPE", "MOVE_RETREAT", "MOVE_HOME"]


class _QtSignalHandler(logging.Handler):
    """Logging → Qt-Signal-Weiterleitung (für ProcessTab-Log)."""

    def __init__(self, owner: "ProcessOptimizeStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._owner.logMessage.emit(self.format(record))
        except Exception:
            pass


class ProcessOptimizeStatemachine(QtCore.QObject):
    """
    Optimize-StateMachine (Retiming-only + Execute-only via MoveItPy)

    WICHTIG: Keine Geometrieänderung:
      - keine Waypoint-Reduction
      - kein Shortcut / Replan
      - nur Retiming (Zeit + optional Velocities neu)

    Ablauf:
      EXEC seg1 -> EXEC seg2 -> ... -> FINISHED
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
        skip_home: bool = True,
        # Retiming Parameter (später aus Recipe/Params)
        time_scale: float = 1.0,   # <1.0 = schneller, >1.0 = langsamer
        enforce_min_dt: float = 0.01,
    ) -> None:
        super().__init__(parent)

        self._run_yaml_path = run_yaml_path
        self._ros = bridge

        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._time_scale = float(time_scale)
        self._min_dt = float(enforce_min_dt)

        self._stop_requested: bool = False
        self._stopped: bool = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStateMachine] = None
        self._current_state: str = ""
        self._retry_count: int = 0

        self._joints: List[str] = []
        self._segments_pts: Dict[str, List[Dict[str, Any]]] = {}
        self._segments_traj: Dict[str, JointTrajectory] = {}

        self._moveitpy_bridge = getattr(self._ros, "moveitpy_bridge", None)
        self._moveitpy_signals = getattr(self._moveitpy_bridge, "signals", None)

        self._log_handler: Optional[_QtSignalHandler] = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        self._log_handler.setFormatter(logging.Formatter("%(message)s"))
        _LOG.addHandler(self._log_handler)

        if self._moveitpy_signals is not None:
            try:
                self._moveitpy_signals.motionResultChanged.connect(self._on_motion_result)  # type: ignore[attr-defined]
            except Exception:
                pass

        _LOG.info(
            "Optimize init: run_yaml=%s, time_scale=%.3f, skip_home=%s",
            run_yaml_path, self._time_scale, self._skip_home
        )

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def start(self) -> None:
        _LOG.info("Optimize: start()")

        self._stop_requested = False
        self._stopped = False
        self._error_msg = None
        self._retry_count = 0
        self._current_state = ""
        self._segments_traj.clear()

        if not self._load_run_yaml(self._run_yaml_path):
            msg = self._error_msg or f"Run-YAML '{self._run_yaml_path}' konnte nicht geladen werden."
            _LOG.error("Optimize: %s", msg)
            self.notifyError.emit(msg)
            return

        self._build_all_segment_trajectories()
        self._apply_retime_all()

        machine = QStateMachine(self)
        self._machine = machine

        states: Dict[str, QState] = {}
        for name in SEG_ORDER:
            states[name] = QState(machine)

        s_finished = QFinalState(machine)
        s_error = QFinalState(machine)

        first = self._first_segment_state()
        if first is None:
            msg = "Optimize: Keine Segmente zum Ausführen gefunden."
            _LOG.error(msg)
            self.notifyError.emit(msg)
            return

        machine.setInitialState(states[first])

        for i, name in enumerate(SEG_ORDER):
            st = states[name]
            st.addTransition(self._sig_done, states[SEG_ORDER[i + 1]] if i + 1 < len(SEG_ORDER) else s_finished)
            st.addTransition(self._sig_error, s_error)

        for name in SEG_ORDER:
            states[name].entered.connect(lambda n=name: self._on_state_execute(n))

        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        machine.start()
        _LOG.info("Optimize: StateMachine gestartet (initial=%s).", first)

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        _LOG.info("Optimize: request_stop()")
        self._stop_requested = True
        self._stopped = True
        self._call_ui("moveit_stop")

    @QtCore.pyqtSlot()
    def stop(self) -> None:
        _LOG.info("Optimize: stop()")
        self.request_stop()

    # ------------------------------------------------------------------ #
    # YAML / build trajectories
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
            "Optimize: YAML geladen joints=%d, segments={%s}",
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

            if t <= last_t:
                t = last_t + max(self._min_dt, 0.001)
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in positions]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)

        if len(traj.points) < 2:
            _LOG.warning("Optimize: Segment %s hat zu wenig gültige Punkte.", seg_name)
            return None
        return traj

    # ------------------------------------------------------------------ #
    # Retiming (NO geometry change)
    # ------------------------------------------------------------------ #

    def _apply_retime_all(self) -> None:
        # time_scale: <1 schneller, >1 langsamer
        s = max(0.05, float(self._time_scale))
        if abs(s - 1.0) < 1e-6:
            _LOG.info("Optimize: time_scale=1.0 -> kein Retiming angewendet.")
            return

        for name, jt in list(self._segments_traj.items()):
            self._segments_traj[name] = self._retime_scale_joint_traj(jt, scale=s)

        _LOG.info("Optimize: Retiming angewendet (scale=%.3f).", s)

    def _retime_scale_joint_traj(self, jt: JointTrajectory, *, scale: float) -> JointTrajectory:
        out = JointTrajectory()
        out.joint_names = list(jt.joint_names)

        # skaliere time_from_start
        for p in jt.points:
            t = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            t2 = max(self._min_dt, t * scale)

            q = JointTrajectoryPoint()
            q.positions = list(p.positions)
            q.time_from_start.sec = int(t2)
            q.time_from_start.nanosec = int((t2 - int(t2)) * 1e9)
            out.points.append(q)

        # optional: velocities aus finite diff (hilft Controller)
        self._fill_velocities_fd(out)

        return out

    def _fill_velocities_fd(self, jt: JointTrajectory) -> None:
        n = len(jt.points)
        if n < 2:
            return
        n_j = len(jt.joint_names)
        if n_j <= 0:
            return

        def t_of(i: int) -> float:
            p = jt.points[i]
            return float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9

        for i in range(n):
            jt.points[i].velocities = [0.0] * n_j

        for i in range(n - 1):
            p0 = jt.points[i]
            p1 = jt.points[i + 1]
            dt = max(self._min_dt, t_of(i + 1) - t_of(i))
            v = [(p1.positions[j] - p0.positions[j]) / dt for j in range(n_j)]
            p0.velocities = v

        # letztes: gleiche v wie vorletztes
        jt.points[-1].velocities = list(jt.points[-2].velocities)

    # ------------------------------------------------------------------ #
    # State exec
    # ------------------------------------------------------------------ #

    def _first_segment_state(self) -> Optional[str]:
        for name in SEG_ORDER:
            if self._skip_home and name == "MOVE_HOME":
                continue
            if self._segments_traj.get(name) is not None:
                return name
        return None

    def _emit_state(self, name: str) -> None:
        self._current_state = name
        self.stateChanged.emit(name)
        self.logMessage.emit(f"Optimize: State={name}")

    def _on_state_execute(self, seg_name: str) -> None:
        if self._skip_home and seg_name == "MOVE_HOME":
            self._emit_state(seg_name)
            _LOG.info("Optimize: skip_home -> überspringe MOVE_HOME")
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        jt = self._segments_traj.get(seg_name)
        if jt is None:
            self._emit_state(seg_name)
            _LOG.info("Optimize: Segment %s leer -> überspringe", seg_name)
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
        for fn in ("moveit_execute_joint_trajectory", "moveit_execute_trajectory", "moveit_execute"):
            if hasattr(self._ros, fn):
                try:
                    getattr(self._ros, fn)(jt)
                    _LOG.info("Optimize: execute via %s()", fn)
                    return True
                except Exception as e:
                    self._error_msg = f"{fn} failed: {e}"
                    _LOG.exception("Optimize: %s", self._error_msg)
                    return False
        self._error_msg = "RosBridge hat keine Execute-Methode (moveit_execute_*)."
        _LOG.error("Optimize: %s", self._error_msg)
        return False

    def _signal_error(self, msg: str) -> None:
        if not msg:
            msg = "Unbekannter Optimize-Fehler."
        if self._error_msg is None:
            self._error_msg = msg
        _LOG.error("Optimize: %s", msg)
        QtCore.QTimer.singleShot(0, self._sig_error.emit)

    # ------------------------------------------------------------------ #
    # Motion feedback
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        if self._machine is None or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if not self._current_state:
            return

        _LOG.info("Optimize: motion_result=%s (state=%s)", result, self._current_state)

        if result.startswith("ERROR"):
            if self._retry_count < self._max_retries:
                self._retry_count += 1
                _LOG.warning("Optimize: Retry %d/%d für %s", self._retry_count, self._max_retries, self._current_state)
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

        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ------------------------------------------------------------------ #
    # Final / cleanup
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

    def _call_ui(self, name: str, *args, **kwargs):
        if hasattr(self._ros, name):
            try:
                return getattr(self._ros, name)(*args, **kwargs)
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
