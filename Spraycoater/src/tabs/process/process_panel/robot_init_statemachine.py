# -*- coding: utf-8 -*-
# File: src/app/tabs/process/robot_init_statemachine.py
from __future__ import annotations

import json
import logging
from typing import Optional, Any, Dict, Tuple, Set

from PyQt6 import QtCore
from PyQt6.QtCore import QTimer

_LOG = logging.getLogger("tabs.process.robot_init_sm")


class RobotInitStatemachine(QtCore.QObject):
    """
    Robot Init + Home — STRICT (RosBridge facade, plan_request-only)

    Fixes (2026-01):
      ✅ tray_exec_ready is DEBOUNCED (must be stable True for N ms)
      ✅ HOME_EXEC_REQUESTED has its own gate-timeout (no infinite hang)
      ✅ Execute result op accepts aliases ("execute", "execute_last_planned", "execute_trajectory")
    """

    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    # ---------------- States ----------------
    S_WAIT_HOME_AVAILABLE = "WAIT_HOME_AVAILABLE"
    S_WAIT_MOVEIT_READY = "WAIT_MOVEIT_READY"
    S_INIT_REQUESTED = "INIT_REQUESTED"
    S_HOME_PLAN_REQUESTED = "HOME_PLAN_REQUESTED"
    S_WAIT_HOME_PLANNED = "WAIT_HOME_PLANNED"
    S_WAIT_BEFORE_EXEC = "WAIT_BEFORE_EXEC"
    S_HOME_EXEC_REQUESTED = "HOME_EXEC_REQUESTED"
    S_WAIT_HOME_EXEC = "WAIT_HOME_EXEC"
    S_FINISHED = "FINISHED"
    S_ERROR = "ERROR"
    S_STOPPED = "STOPPED"

    # ---------------- Keying ----------------
    _SEG = "ROBOT_INIT"
    _OP_PLAN_POSE = "plan_pose"

    # Execute op aliases (node/facade dependent)
    _OP_EXECUTE_ALIASES: Set[str] = {
        "execute",
        "execute_last_planned",
        "execute_trajectory",
    }

    def __init__(
        self,
        *,
        ros: Any,
        init_timeout_s: float = 10.0,
        home_timeout_s: float = 60.0,
        home_pose_timeout_s: float = 10.0,
        moveit_ready_timeout_s: float = 20.0,
        moveit_ack_timeout_s: float = 20.0,   # IMPORTANT: was too short at 5s
        pos_tol_mm: float = 1.0,
        do_execute: bool = True,
        wait_before_exec_ms: int = 500,
        max_exec_retries: int = 20,
        # NEW: execution gate timeout (tray_exec_ready/busy gate in HOME_EXEC_REQUESTED)
        exec_gate_timeout_s: float = 20.0,
        # NEW: tray_exec_ready must be stable True for this long
        tray_ready_stable_ms: int = 500,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ros = ros

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._home_pose_timeout_s = float(home_pose_timeout_s)
        self._moveit_ready_timeout_s = float(moveit_ready_timeout_s)
        self._moveit_ack_timeout_s = float(moveit_ack_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)
        self._do_execute = bool(do_execute)

        # NEW
        self._exec_gate_timeout_s = float(exec_gate_timeout_s)
        self._tray_ready_stable_ms = int(max(0, tray_ready_stable_ms))
        self._tray_ready_since_ms: int = 0
        self._exec_gate_started: bool = False

        self._stop_requested: bool = False
        self._done: bool = False

        self._state: str = self.S_WAIT_HOME_AVAILABLE
        self._deadline_ms: int = 0

        # request identity (keyed)
        self._run: str = ""
        self._req_id: int = 0

        # "send once" gates per run attempt
        self._init_sent_once: bool = False
        self._plan_sent_once: bool = False
        self._exec_sent_once: bool = False

        # retry logic
        self._exec_retry_count: int = 0
        self._max_exec_retries: int = int(max_exec_retries)

        # last motion result snapshot
        self._last_result_snapshot: str = ""

        # delay between plan and execute
        self._wait_before_exec_ms: int = int(max(0, wait_before_exec_ms))

        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------
    # Public
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._require_contract()

        self._stop_requested = False
        self._done = False

        self._init_sent_once = False
        self._plan_sent_once = False
        self._exec_sent_once = False

        self._exec_retry_count = 0
        self._run = ""
        self._req_id = 0

        # NEW: reset readiness debouncer + exec gate
        self._tray_ready_since_ms = 0
        self._exec_gate_started = False

        # snapshot baseline
        self._last_result_snapshot = self._last_motion_result()

        self._set_state(self.S_WAIT_HOME_AVAILABLE)
        self._set_deadline(self._home_pose_timeout_s)

        self._log(
            "RobotInit: start "
            f"(init_timeout={self._init_timeout_s:.1f}s, "
            f"home_timeout={self._home_timeout_s:.1f}s, "
            f"moveit_ready_timeout={self._moveit_ready_timeout_s:.1f}s, "
            f"ack_timeout={self._moveit_ack_timeout_s:.1f}s, "
            f"do_execute={self._do_execute}, "
            f"exec_gate_timeout={self._exec_gate_timeout_s:.1f}s, "
            f"tray_stable={self._tray_ready_stable_ms}ms)"
        )

        self._timer.start()
        self._tick()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        if self._done:
            return
        self._stop_requested = True
        self._log("RobotInit: stop requested -> stopping moveit + robot")
        self._ros.stop_all()

    # ------------------------------------------------------------------
    # Contract (STRICT)
    # ------------------------------------------------------------------

    def _require_contract(self) -> None:
        if self._ros is None:
            raise RuntimeError("RobotInit: ros is None")

        for fn in ("stop_all", "robot_init", "moveit_last_result", "moveit_plan_pose"):
            if not callable(getattr(self._ros, fn, None)):
                raise RuntimeError(f"RobotInit: ros.{fn} missing")

        if self._do_execute and not callable(getattr(self._ros, "moveit_execute_last_planned", None)):
            raise RuntimeError("RobotInit: do_execute=True but ros.moveit_execute_last_planned(...) missing")

        if not hasattr(self._ros, "_poses_state"):
            raise RuntimeError("RobotInit: ros._poses_state missing")
        if not hasattr(self._ros._poses_state, "home"):
            raise RuntimeError("RobotInit: ros._poses_state.home missing")

    # ------------------------------------------------------------------
    # Time / State
    # ------------------------------------------------------------------

    def _now_ms(self) -> int:
        return int(QtCore.QDateTime.currentMSecsSinceEpoch())

    def _set_deadline(self, timeout_s: float) -> None:
        self._deadline_ms = self._now_ms() + max(0, int(float(timeout_s) * 1000.0))

    def _set_deadline_ms(self, timeout_ms: int) -> None:
        self._deadline_ms = self._now_ms() + max(0, int(timeout_ms))

    def _deadline_passed(self) -> bool:
        return self._deadline_ms > 0 and self._now_ms() >= self._deadline_ms

    def _set_state(self, s: str) -> None:
        if s != self._state:
            self._state = s
            self.stateChanged.emit(s)

    def _log(self, msg: str) -> None:
        if msg:
            self.logMessage.emit(msg)
            _LOG.info(msg)

    def _finish_ok(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_FINISHED)
        self._log("RobotInit: finished OK")
        self.notifyFinished.emit()

    def _finish_err(self, msg: str) -> None:
        if self._done:
            return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_ERROR)
        self._log(f"RobotInit: ERROR: {msg}")
        self.notifyError.emit(msg)

    def _finish_stopped(self) -> None:
        if self._done:
            return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_STOPPED)
        self._log("RobotInit: STOPPED")
        self.notifyError.emit("STOPPED")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _home_pose_available(self) -> bool:
        return self._ros._poses_state.home is not None

    def _moveit_tray_exec_ready(self) -> Optional[bool]:
        fn = getattr(self._ros, "moveit_tray_exec_ready", None)
        if callable(fn):
            try:
                return bool(fn())
            except Exception:
                return None
        return None

    def _moveit_is_busy(self) -> Optional[bool]:
        fn = getattr(self._ros, "moveit_is_busy", None)
        if callable(fn):
            try:
                return bool(fn())
            except Exception:
                return None
        return None

    def _tray_exec_ready_stable(self) -> Optional[bool]:
        """
        Debounced tray_exec_ready:
          - returns True only if tray_exec_ready stayed True for _tray_ready_stable_ms
          - returns False if currently False (and resets stability timer)
          - returns None if signal not available (optional contract)
        """
        r = self._moveit_tray_exec_ready()
        if r is None:
            return None

        now = self._now_ms()
        if r:
            if self._tray_ready_since_ms <= 0:
                self._tray_ready_since_ms = now
            if self._tray_ready_stable_ms <= 0:
                return True
            return (now - self._tray_ready_since_ms) >= self._tray_ready_stable_ms
        # r == False
        self._tray_ready_since_ms = 0
        return False

    def _clear_motion_result_best_effort(self) -> None:
        """
        Local-only clear to avoid parsing stale JSON; does NOT require a ROS service.
        """
        for name in ("moveit_clear_result", "moveit_clear_motion_result", "moveit_reset_motion_result"):
            fn = getattr(self._ros, name, None)
            if callable(fn):
                try:
                    fn()
                    return
                except Exception:
                    _LOG.exception("RobotInit: %s failed", name)

    def _last_motion_result(self) -> str:
        try:
            return str(self._ros.moveit_last_result() or "").strip()
        except Exception:
            _LOG.exception("RobotInit: moveit_last_result failed")
            return ""

    @staticmethod
    def _parse_keyed_result(res: str) -> Optional[Tuple[Dict[str, Any], str]]:
        s = (res or "").strip()
        if not s or not s.startswith("{"):
            return None
        try:
            obj = json.loads(s)
            if not isinstance(obj, dict):
                return None
            key = obj.get("key")
            status = obj.get("status")
            if not isinstance(key, dict) or not isinstance(status, str):
                return None
            return key, status.strip()
        except Exception:
            return None

    @staticmethod
    def _is_error_status(status: str) -> bool:
        up = (status or "").strip().upper()
        return bool(up) and up.startswith("ERROR")

    @staticmethod
    def _is_retryable_status(status: str) -> bool:
        up = (status or "").strip().upper()
        return ("GOAL_REJECTED" in up) or ("BUSY" in up)

    def _match_key(self, key: Dict[str, Any], *, op: str) -> bool:
        try:
            return (
                key.get("run") == self._run
                and int(key.get("id")) == int(self._req_id)
                and str(key.get("seg") or "") == self._SEG
                and str(key.get("op") or "") == str(op)
            )
        except Exception:
            return False

    def _match_key_any_exec(self, key: Dict[str, Any]) -> bool:
        try:
            return (
                key.get("run") == self._run
                and int(key.get("id")) == int(self._req_id)
                and str(key.get("seg") or "") == self._SEG
                and str(key.get("op") or "") in self._OP_EXECUTE_ALIASES
            )
        except Exception:
            return False

    def _new_run_key(self) -> None:
        """
        Create a new run/id for a new attempt.
        """
        now = self._now_ms()
        self._run = f"robot_init_{now}"
        self._req_id = int(now)

    # ------------------------------------------------------------------
    # Actions (plan_request only via RosBridge facade)
    # ------------------------------------------------------------------

    def _send_robot_init_once(self) -> None:
        if self._init_sent_once:
            return
        self._init_sent_once = True
        self._log("RobotInit: sending robot_init()")
        self._ros.robot_init()

    def _request_home_plan_pose_once(self) -> None:
        if self._plan_sent_once:
            return
        self._plan_sent_once = True

        home = self._ros._poses_state.home
        if home is None:
            raise RuntimeError("RobotInit: home pose is None")

        # fresh key per plan attempt (important for retries)
        if not self._run or not self._req_id:
            self._new_run_key()

        # clear stale local result so we can detect a new one deterministically
        self._clear_motion_result_best_effort()
        self._last_result_snapshot = self._last_motion_result()

        self._log("RobotInit: request HOME plan_pose via RosBridge (plan_request)")
        self._ros.moveit_plan_pose(home, run=self._run, req_id=self._req_id, segment=self._SEG)

    def _request_execute_last_planned_once(self) -> None:
        if not self._do_execute:
            return
        if self._exec_sent_once:
            return
        self._exec_sent_once = True

        # clear stale local result
        self._clear_motion_result_best_effort()
        self._last_result_snapshot = self._last_motion_result()

        self._log("RobotInit: request execute-last-planned via RosBridge (plan_request)")
        self._ros.moveit_execute_last_planned(run=self._run, req_id=self._req_id, segment=self._SEG)

    def _reset_for_retry(self) -> None:
        """
        Reset send-once flags and allocate a fresh key so we can re-plan cleanly.
        """
        self._exec_retry_count += 1
        self._init_sent_once = True  # keep robot_init() as already sent (do not spam)
        self._plan_sent_once = False
        self._exec_sent_once = False
        self._new_run_key()
        self._last_result_snapshot = ""
        self._clear_motion_result_best_effort()

        # NEW: reset debouncer + exec gate (fresh attempt)
        self._tray_ready_since_ms = 0
        self._exec_gate_started = False

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _tick(self) -> None:
        if self._stop_requested:
            self._finish_stopped()
            return

        st = self._state

        # ---------------- WAIT_HOME_AVAILABLE ----------------
        if st == self.S_WAIT_HOME_AVAILABLE:
            if self._home_pose_available():
                self._log("RobotInit: home pose available")
                self._set_state(self.S_WAIT_MOVEIT_READY)
                self._set_deadline(self._moveit_ready_timeout_s)
            elif self._deadline_passed():
                self._finish_err(f"Home pose not available (timeout {self._home_pose_timeout_s:.1f}s)")
            return

        # ---------------- WAIT_MOVEIT_READY ----------------
        if st == self.S_WAIT_MOVEIT_READY:
            # gate: tray_exec_ready (capability) if available (DEBOUNCED)
            ready = self._tray_exec_ready_stable()
            if ready is False:
                if self._deadline_passed():
                    self._finish_err(
                        f"MoveIt tray_exec_ready not stable-true "
                        f"(timeout {self._moveit_ready_timeout_s:.1f}s)"
                    )
                return

            # gate: busy must be False if available
            busy = self._moveit_is_busy()
            if busy is True:
                if self._deadline_passed():
                    self._finish_err(f"MoveIt stayed busy too long (timeout {self._moveit_ready_timeout_s:.1f}s)")
                return

            self._log("RobotInit: moveit ready")
            self._set_state(self.S_INIT_REQUESTED)
            self._deadline_ms = 0
            return

        # ---------------- INIT_REQUESTED ----------------
        if st == self.S_INIT_REQUESTED:
            try:
                self._send_robot_init_once()
            except Exception as e:
                self._finish_err(f"robot_init() failed: {e!r}")
                return

            # fresh key for this init sequence
            self._new_run_key()

            self._set_state(self.S_HOME_PLAN_REQUESTED)
            self._deadline_ms = 0
            return

        # ---------------- HOME_PLAN_REQUESTED ----------------
        if st == self.S_HOME_PLAN_REQUESTED:
            try:
                self._request_home_plan_pose_once()
            except Exception as e:
                self._finish_err(f"HOME plan_pose request failed: {e!r}")
                return

            self._set_state(self.S_WAIT_HOME_PLANNED)
            self._set_deadline(self._moveit_ack_timeout_s)
            return

        # ---------------- WAIT_HOME_PLANNED ----------------
        if st == self.S_WAIT_HOME_PLANNED:
            cur = self._last_motion_result()

            # only react on changes (prevents spam)
            if cur and cur != self._last_result_snapshot:
                self._log(f"RobotInit: moveit result update: {cur}")
                self._last_result_snapshot = cur

                parsed = self._parse_keyed_result(cur)
                if parsed is None:
                    # non-JSON error string
                    if cur.upper().startswith("ERROR"):
                        self._finish_err(f"Home plan failed: {cur}")
                    return

                key, status = parsed
                if not self._match_key(key, op=self._OP_PLAN_POSE):
                    return

                if status == "PLANNED:OK":
                    self._log("RobotInit: home planned (motion_result)")
                    if not self._do_execute:
                        self._finish_ok()
                        return

                    # optional stability delay
                    if self._wait_before_exec_ms > 0:
                        self._set_state(self.S_WAIT_BEFORE_EXEC)
                        self._set_deadline_ms(self._wait_before_exec_ms)
                    else:
                        self._set_state(self.S_HOME_EXEC_REQUESTED)
                        self._deadline_ms = 0
                        self._exec_gate_started = False
                    return

                if self._is_error_status(status):
                    self._finish_err(f"Home plan failed: {status}")
                    return

            if self._deadline_passed():
                self._finish_err(f"Timeout waiting for PLANNED:OK (last='{cur}')")
            return

        # ---------------- WAIT_BEFORE_EXEC ----------------
        if st == self.S_WAIT_BEFORE_EXEC:
            if self._deadline_passed():
                self._set_state(self.S_HOME_EXEC_REQUESTED)
                self._deadline_ms = 0
                self._exec_gate_started = False
            return

        # ---------------- HOME_EXEC_REQUESTED ----------------
        if st == self.S_HOME_EXEC_REQUESTED:
            # NEW: explicit gate-timeout to avoid infinite hang here
            if not self._exec_gate_started:
                self._exec_gate_started = True
                self._set_deadline(self._exec_gate_timeout_s)

            # gate execute: tray_exec_ready && !busy if available
            ready = self._tray_exec_ready_stable()
            busy = self._moveit_is_busy()

            if ready is False or busy is True:
                if self._deadline_passed():
                    self._finish_err(
                        f"Execute gate not ready "
                        f"(tray_exec_ready={ready}, busy={busy}) "
                        f"(timeout {self._exec_gate_timeout_s:.1f}s)"
                    )
                return

            try:
                self._request_execute_last_planned_once()
            except Exception as e:
                self._finish_err(f"Execute request failed: {e!r}")
                return

            self._set_state(self.S_WAIT_HOME_EXEC)
            self._set_deadline(self._home_timeout_s)
            return

        # ---------------- WAIT_HOME_EXEC ----------------
        if st == self.S_WAIT_HOME_EXEC:
            cur = self._last_motion_result()

            if cur and cur != self._last_result_snapshot:
                self._log(f"RobotInit: moveit result update: {cur}")
                self._last_result_snapshot = cur

                parsed = self._parse_keyed_result(cur)

                # if parse fails, allow raw ERROR strings to abort
                if parsed is None:
                    if cur.upper().startswith("ERROR"):
                        # retry only if it looks retryable
                        if self._exec_retry_count < self._max_exec_retries and self._is_retryable_status(cur):
                            self._log(
                                f"RobotInit: retryable execute failure, retry "
                                f"{self._exec_retry_count + 1}/{self._max_exec_retries} (RE-PLAN)..."
                            )
                            self._reset_for_retry()
                            self._set_state(self.S_HOME_PLAN_REQUESTED)
                            self._deadline_ms = 0
                            return
                        self._finish_err(f"Home execute failed: {cur}")
                    return

                key, status = parsed

                # Accept execute op aliases (execute/execute_last_planned/execute_trajectory)
                if self._match_key_any_exec(key):
                    if status == "EXECUTED:OK":
                        self._log("RobotInit: home executed (motion_result)")
                        self._finish_ok()
                        return

                    if status in ("STOP:REQ", "STOPPED:USER", "STOPPED"):
                        self._finish_stopped()
                        return

                    if self._is_error_status(status):
                        if self._exec_retry_count < self._max_exec_retries and self._is_retryable_status(status):
                            self._log(
                                f"RobotInit: {status} retry "
                                f"{self._exec_retry_count + 1}/{self._max_exec_retries} (RE-PLAN)..."
                            )
                            self._reset_for_retry()
                            self._set_state(self.S_HOME_PLAN_REQUESTED)
                            self._deadline_ms = 0
                            return

                        self._finish_err(f"Home execute failed: {status}")
                        return

                # not our key -> ignore
                return

            if self._deadline_passed():
                self._finish_err(f"Timeout waiting for EXECUTED:OK (last='{cur}')")
            return

        self._finish_err(f"Unknown state '{st}'")
