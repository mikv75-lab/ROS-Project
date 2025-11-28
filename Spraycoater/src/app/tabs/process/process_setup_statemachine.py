# -*- coding: utf-8 -*-
# File: tabs/process/process_setup_statemachine.py
from __future__ import annotations

from typing import Optional, List, Dict, Any
from queue import Queue, Empty
import logging

from PyQt6 import QtCore
from PyQt6.QtStateMachine import QStateMachine, QState, QFinalState

from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.process.setup_statemachine")


class QStatemachine(QStateMachine):
    """Nur Wrapper für lesbareren Code."""
    def __init__(self, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(parent)


class _QtSignalHandler(logging.Handler):
    """Qt-Signal-Bridge fürs Logging."""
    def __init__(self, owner: "ProcessSetupStatemachine") -> None:
        super().__init__()
        self._owner = owner

    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
        except Exception:
            return
        try:
            self._owner.logMessage.emit(msg)
        except Exception:
            pass


class ProcessSetupStatemachine(QtCore.QObject):
    """
    Setup-Prozesslauf:
        - Predispense → Rezept → Retreat → Home
        - zeichnet TCP-Stream + geplante/ausgeführte Trajs auf
    """

    # Ergebnis/Fehler
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    # UI-Infos
    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # interne StateMachine-Signale
    _sig_predispense_move_done = QtCore.pyqtSignal()
    _sig_predispense_wait_done = QtCore.pyqtSignal()
    _sig_move_recipe_done = QtCore.pyqtSignal()
    _sig_postdispense_wait_done = QtCore.pyqtSignal()
    _sig_retreat_move_done = QtCore.pyqtSignal()
    _sig_move_home_done = QtCore.pyqtSignal()
    _sig_error = QtCore.pyqtSignal()

    # Phasen
    PHASE_NONE = "NONE"
    PHASE_MOVE_PREDISPENSE = "MOVE_PREDISPENSE"
    PHASE_MOVE_RECIPE = "MOVE_RECIPE"
    PHASE_MOVE_RETREAT = "MOVE_RETREAT"
    PHASE_MOVE_HOME = "MOVE_HOME"

    def __init__(self, *, recipe: Optional[Recipe], bridge, parent=None) -> None:
        super().__init__(parent)

        self._recipe: Optional[Recipe] = recipe
        self._bridge = bridge

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None

        self._machine: Optional[QStatemachine] = None
        self._current_phase: str = self.PHASE_NONE

        # Daten
        self._recipe_poses: List[PoseStamped] = []
        self._recipe_index: int = 0
        self._executed_poses: List[PoseStamped] = []
        self._planned_traj_list: List[RobotTrajectoryMsg] = []
        self._executed_traj_list: List[RobotTrajectoryMsg] = []

        # Queues
        self._tcp_pose_queue: "Queue[PoseStamped]" = Queue()
        self._planned_traj_queue: "Queue[RobotTrajectoryMsg]" = Queue()
        self._executed_traj_queue: "Queue[RobotTrajectoryMsg]" = Queue()

        # Timer
        self._poll_timer: Optional[QtCore.QTimer] = None

        # Logging
        self._log_handler = _QtSignalHandler(self)
        self._log_handler.setLevel(logging.INFO)
        _LOG.addHandler(self._log_handler)

        # Bridges
        self._rb = getattr(bridge, "_rb", None)
        self._motion = getattr(bridge, "_motion", None)

        self._rb_signals = getattr(self._rb, "signals", None) if self._rb else None
        self._motion_signals = getattr(self._motion, "signals", None) if self._motion else None

        # RobotBridge TCP-Stream
        if self._rb_signals and hasattr(self._rb_signals, "tcpPoseChanged"):
            self._rb_signals.tcpPoseChanged.connect(self._on_tcp_stream)

        # MotionBridge Ergebnisse
        if self._motion_signals:
            if hasattr(self._motion_signals, "motionResultChanged"):
                self._motion_signals.motionResultChanged.connect(self._on_motion_result)
            if hasattr(self._motion_signals, "plannedTrajectoryChanged"):
                self._motion_signals.plannedTrajectoryChanged.connect(self._on_planned_traj)
            if hasattr(self._motion_signals, "executedTrajectoryChanged"):
                self._motion_signals.executedTrajectoryChanged.connect(self._on_executed_traj)

    # ----------------------------------------------------------------------
    # Public API
    # ----------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        if self._recipe is None:
            self._signal_error("Kein Rezept gesetzt.")
            return

        self._stop_requested = False
        self._error_msg = None
        self._current_phase = self.PHASE_NONE

        self._recipe_poses = []
        self._recipe_index = 0
        self._executed_poses = []
        self._planned_traj_list = []
        self._executed_traj_list = []

        self._drain_all_queues()

        m = QStatemachine(self)
        self._machine = m

        # States
        s_predisp = QState(m)
        s_wait_pre = QState(m)
        s_recipe = QState(m)
        s_wait_post = QState(m)
        s_retreat = QState(m)
        s_home = QState(m)
        s_finished = QFinalState(m)
        s_error = QFinalState(m)

        m.setInitialState(s_predisp)

        # Transitions
        s_predisp.addTransition(self._sig_predispense_move_done, s_wait_pre)
        s_wait_pre.addTransition(self._sig_predispense_wait_done, s_recipe)
        s_recipe.addTransition(self._sig_move_recipe_done, s_wait_post)
        s_wait_post.addTransition(self._sig_postdispense_wait_done, s_retreat)
        s_retreat.addTransition(self._sig_retreat_move_done, s_home)
        s_home.addTransition(self._sig_move_home_done, s_finished)

        for st in (s_predisp, s_wait_pre, s_recipe, s_wait_post, s_retreat, s_home):
            st.addTransition(self._sig_error, s_error)

        # Logic
        s_predisp.entered.connect(self._on_state_move_predispense)
        s_wait_pre.entered.connect(self._on_state_wait_predispense)
        s_recipe.entered.connect(self._on_state_move_recipe)
        s_wait_post.entered.connect(self._on_state_wait_postdispense)
        s_retreat.entered.connect(self._on_state_move_retreat)
        s_home.entered.connect(self._on_state_move_home)
        s_error.entered.connect(self._on_state_error)
        s_finished.entered.connect(self._on_state_finished)

        # UI states
        s_predisp.entered.connect(lambda: self.stateChanged.emit("MOVE_PREDISPENSE"))
        s_wait_pre.entered.connect(lambda: self.stateChanged.emit("WAIT_PREDISPENSE"))
        s_recipe.entered.connect(lambda: self.stateChanged.emit("MOVE_RECIPE"))
        s_wait_post.entered.connect(lambda: self.stateChanged.emit("WAIT_POSTDISPENSE"))
        s_retreat.entered.connect(lambda: self.stateChanged.emit("MOVE_RETREAT"))
        s_home.entered.connect(lambda: self.stateChanged.emit("MOVE_HOME"))

        # Polling-Timer
        self._poll_timer = QtCore.QTimer(self)
        self._poll_timer.setInterval(50)
        self._poll_timer.timeout.connect(self._poll_queues)
        self._poll_timer.start()

        m.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True

    @property
    def recipe(self) -> Optional[Recipe]:
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        self._recipe = recipe

    # ----------------------------------------------------------------------
    # Internals
    # ----------------------------------------------------------------------

    def _should_stop(self) -> bool:
        return self._stop_requested

    def _finish_state(self, sig: QtCore.pyqtSignal) -> None:
        if self._should_stop() and not self._error_msg:
            self._error_msg = "Vom Benutzer gestoppt."
        if self._error_msg:
            self._post_transition(self._sig_error)
        else:
            self._post_transition(sig)

    def _post_transition(self, sig: QtCore.pyqtSignal) -> None:
        QtCore.QTimer.singleShot(0, sig.emit)

    def _signal_error(self, msg: str) -> None:
        self._error_msg = msg
        self._post_transition(self._sig_error)

    def _disconnect_signals(self) -> None:
        try:
            if self._rb_signals and hasattr(self._rb_signals, "tcpPoseChanged"):
                self._rb_signals.tcpPoseChanged.disconnect(self._on_tcp_stream)
        except Exception:
            pass

        try:
            if self._motion_signals and hasattr(self._motion_signals, "motionResultChanged"):
                self._motion_signals.motionResultChanged.disconnect(self._on_motion_result)
        except Exception:
            pass

        try:
            if self._motion_signals and hasattr(self._motion_signals, "plannedTrajectoryChanged"):
                self._motion_signals.plannedTrajectoryChanged.disconnect(self._on_planned_traj)
        except Exception:
            pass

        try:
            if self._motion_signals and hasattr(self._motion_signals, "executedTrajectoryChanged"):
                self._motion_signals.executedTrajectoryChanged.disconnect(self._on_executed_traj)
        except Exception:
            pass

    # ----------------------------------------------------------------------
    # Result-Handling
    # ----------------------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        res = (text or "").upper()
        if res.startswith("PLANNED"):
            return

        if res.startswith("EXECUTED"):
            self._handle_motion_success()
            return

        if res.startswith(("ERROR", "FAILED", "ABORTED", "CANCELLED", "UNKNOWN")):
            self._signal_error(res)
            return

        self._signal_error(res)

    def _handle_motion_success(self) -> None:
        ph = self._current_phase

        if ph == self.PHASE_MOVE_PREDISPENSE:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_predispense_move_done)
            return

        if ph == self.PHASE_MOVE_RECIPE:
            self._recipe_index += 1
            if self._recipe_index >= len(self._recipe_poses) - 1:
                self._current_phase = self.PHASE_NONE
                self._post_transition(self._sig_move_recipe_done)
                return

            pose = self._recipe_poses[self._recipe_index]
            self._send_motion_pose(pose)
            return

        if ph == self.PHASE_MOVE_RETREAT:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_retreat_move_done)
            return

        if ph == self.PHASE_MOVE_HOME:
            self._current_phase = self.PHASE_NONE
            self._post_transition(self._sig_move_home_done)
            return

        self._current_phase = self.PHASE_NONE

    # ----------------------------------------------------------------------
    # TCP & Traj Streams
    # ----------------------------------------------------------------------

    @QtCore.pyqtSlot(object)
    def _on_tcp_stream(self, msg: object) -> None:
        if not isinstance(msg, PoseStamped):
            return
        try:
            self._tcp_pose_queue.put_nowait(msg)
        except Exception:
            pass

    @QtCore.pyqtSlot(object)
    def _on_planned_traj(self, msg: object) -> None:
        if isinstance(msg, RobotTrajectoryMsg):
            try:
                self._planned_traj_queue.put_nowait(msg)
            except Exception:
                pass

    @QtCore.pyqtSlot(object)
    def _on_executed_traj(self, msg: object) -> None:
        if isinstance(msg, RobotTrajectoryMsg):
            try:
                self._executed_traj_queue.put_nowait(msg)
            except Exception:
                pass

    def _poll_queues(self) -> None:
        if self._machine is None or not self._machine.isRunning():
            self._drain_all_queues()
            return

        if self._error_msg or self._should_stop():
            self._drain_all_queues()
            return

        active = (
            self.PHASE_MOVE_PREDISPENSE,
            self.PHASE_MOVE_RECIPE,
            self.PHASE_MOVE_RETREAT,
            self.PHASE_MOVE_HOME,
        )

        # TCP
        while True:
            try:
                ps = self._tcp_pose_queue.get_nowait()
            except Empty:
                break
            if self._current_phase in active:
                self._executed_poses.append(ps)

        # planned trajs
        while True:
            try:
                tr = self._planned_traj_queue.get_nowait()
            except Empty:
                break
            if self._current_phase in active:
                self._planned_traj_list.append(tr)

        # executed trajs
        while True:
            try:
                tr = self._executed_traj_queue.get_nowait()
            except Empty:
                break
            if self._current_phase in active:
                self._executed_traj_list.append(tr)

    def _drain_all_queues(self) -> None:
        for q in (self._tcp_pose_queue, self._planned_traj_queue, self._executed_traj_queue):
            while True:
                try:
                    q.get_nowait()
                except Empty:
                    break

    # ----------------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------------

    def _get_recipe_poses(self) -> List[PoseStamped]:
        pc = getattr(self._recipe, "paths_compiled", {}) if self._recipe else {}
        sides = pc.get("sides", {})
        if not sides:
            return []

        first = next(iter(sides.keys()))
        s = sides.get(first, {})
        poses_q = s.get("poses_quat", [])

        out = []
        frame = pc.get("frame", "scene")

        for p in poses_q:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose.position.x = p.get("x", 0)
            ps.pose.position.y = p.get("y", 0)
            ps.pose.position.z = p.get("z", 0)
            ps.pose.orientation.w = 1.0
            out.append(ps)

        return out

    def _get_home_pose(self) -> Optional[PoseStamped]:
        pb = getattr(self._bridge, "_pb", None)
        if pb:
            try:
                p = pb.get_last_home_pose()
                if isinstance(p, PoseStamped):
                    return p
            except Exception:
                pass
        return None

    def _send_motion_pose(self, pose: PoseStamped) -> None:
        if not pose:
            self._signal_error("Pose None.")
            return

        if not self._motion_signals:
            self._signal_error("MotionBridge fehlt.")
            return

        try:
            self._current_phase = (
                self.PHASE_MOVE_PREDISPENSE
                if self._current_phase == self.PHASE_NONE
                else self._current_phase
            )
            self._motion_signals.moveToPoseRequested.emit(pose)
        except Exception as e:
            self._signal_error(str(e))

    # ----------------------------------------------------------------------
    # FINAL STATES
    # ----------------------------------------------------------------------

    def _on_state_error(self) -> None:
        self.stateChanged.emit("ERROR")
        self.notifyError.emit(self._error_msg or "Fehler.")
        self._stop_machine_and_quit()

    def _on_state_finished(self) -> None:
        self.stateChanged.emit("FINISHED")

        result = {
            "poses": list(self._executed_poses),
            "planned_traj": list(self._planned_traj_list),
            "executed_traj": list(self._executed_traj_list),
        }

        self.notifyFinished.emit(result)
        self._stop_machine_and_quit()

    def _stop_machine_and_quit(self) -> None:
        if self._poll_timer:
            self._poll_timer.stop()
            self._poll_timer.deleteLater()
            self._poll_timer = None

        m = self._machine
        if m and m.isRunning():
            try:
                m.stop()
            except Exception:
                pass

        self._machine = None
