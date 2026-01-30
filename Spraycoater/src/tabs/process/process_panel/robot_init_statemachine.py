# -*- coding: utf-8 -*-
# File: src/app/tabs/process/robot_init_statemachine.py
from __future__ import annotations

import json
import logging
from typing import Optional, Any, Dict, Tuple

from PyQt6 import QtCore
from PyQt6.QtCore import QTimer

_LOG = logging.getLogger("tabs.process.robot_init_sm")


class RobotInitStatemachine(QtCore.QObject):
    """
    Robot Init + Home — STRICT (RosBridge facade, topic-only)
    With Robust Retry-Logic and Delay.
    """

    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    S_WAIT_HOME_AVAILABLE = "WAIT_HOME_AVAILABLE"
    S_WAIT_MOVEIT_READY = "WAIT_MOVEIT_READY"
    S_INIT_REQUESTED = "INIT_REQUESTED"
    S_HOME_PLAN_REQUESTED = "HOME_PLAN_REQUESTED"
    S_WAIT_HOME_PLANNED = "WAIT_HOME_PLANNED"
    S_WAIT_BEFORE_EXEC = "WAIT_BEFORE_EXEC"  # NEW: Stability delay
    S_HOME_EXEC_REQUESTED = "HOME_EXEC_REQUESTED"
    S_WAIT_HOME_EXEC = "WAIT_HOME_EXEC"
    S_FINISHED = "FINISHED"
    S_ERROR = "ERROR"
    S_STOPPED = "STOPPED"

    _SEG = "ROBOT_INIT"
    _OP_PLAN_POSE = "plan_pose"
    _OP_EXECUTE = "execute"

    def __init__(
        self,
        *,
        ros: Any,
        init_timeout_s: float = 10.0,
        home_timeout_s: float = 60.0,
        home_pose_timeout_s: float = 10.0,
        moveit_ready_timeout_s: float = 10.0,
        moveit_ack_timeout_s: float = 5.0,
        pos_tol_mm: float = 1.0,
        ack_timeout_s: float = 5.0,
        do_execute: bool = True,
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

        self._stop_requested: bool = False
        self._done: bool = False

        self._state: str = self.S_WAIT_HOME_AVAILABLE
        self._deadline_ms: int = 0

        self._run: str = ""
        self._req_id: int = 0

        self._init_sent_once: bool = False
        self._plan_sent_once: bool = False
        self._exec_sent_once: bool = False

        # --- RETRY LOGIC ---
        self._exec_retry_count: int = 0
        self._max_exec_retries: int = 20  
        self._last_result_snapshot: str = ""
        
        # Stability Delay
        self._wait_before_exec_ms: int = 500  # 500ms Pause vor Execute

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

        self._last_result_snapshot = self._last_motion_result()

        self._set_state(self.S_WAIT_HOME_AVAILABLE)
        self._set_deadline(self._home_pose_timeout_s)

        self._log(
            "RobotInit: start "
            f"(init_timeout={self._init_timeout_s:.1f}s, "
            f"home_timeout={self._home_timeout_s:.1f}s, "
            f"do_execute={self._do_execute})"
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
        if not callable(self._ros.stop_all):
            raise RuntimeError("RobotInit: ros.stop_all() missing")
        if not callable(self._ros.robot_init):
            raise RuntimeError("RobotInit: ros.robot_init() missing")
        if not callable(self._ros.moveit_last_result):
            raise RuntimeError("RobotInit: ros.moveit_last_result() missing")
        if not callable(self._ros.moveit_plan_pose):
            raise RuntimeError("RobotInit: ros.moveit_plan_pose(...) missing")
        if self._do_execute:
            if not callable(self._ros.moveit_execute_last_planned):
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
        if self._done: return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_FINISHED)
        self._log("RobotInit: finished OK")
        self.notifyFinished.emit()

    def _finish_err(self, msg: str) -> None:
        if self._done: return
        self._done = True
        self._timer.stop()
        self._set_state(self.S_ERROR)
        self._log(f"RobotInit: ERROR: {msg}")
        self.notifyError.emit(msg)

    def _finish_stopped(self) -> None:
        if self._done: return
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

    def _last_motion_result(self) -> str:
        return str(self._ros.moveit_last_result() or "").strip()

    @staticmethod
    def _parse_keyed_result(res: str) -> Optional[Tuple[Dict[str, Any], str]]:
        s = (res or "").strip()
        if not s or not s.startswith("{"): return None
        try:
            obj = json.loads(s)
            if not isinstance(obj, dict): return None
            key = obj.get("key")
            status = obj.get("status")
            if not isinstance(key, dict) or not isinstance(status, str): return None
            return key, status.strip()
        except Exception:
            return None

    @staticmethod
    def _is_error_status(status: str) -> bool:
        up = (status or "").strip().upper()
        return bool(up) and up.startswith("ERROR")

    def _match_key(self, key: Dict[str, Any], *, op: str) -> bool:
        return (
            key.get("run") == self._run
            and int(key.get("id")) == int(self._req_id)
            and str(key.get("seg") or "") == self._SEG
            and str(key.get("op") or "") == str(op)
        )

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _send_robot_init_once(self) -> None:
        if self._init_sent_once: return
        self._init_sent_once = True
        self._log("RobotInit: sending robot_init()")
        self._ros.robot_init()

    def _request_home_plan_pose_once(self) -> None:
        if self._plan_sent_once: return
        self._plan_sent_once = True
        home = self._ros._poses_state.home
        if home is None: raise RuntimeError("RobotInit: home pose is None")
        now = self._now_ms()
        self._run = f"robot_init_{now}"
        self._req_id = int(now)
        self._last_result_snapshot = self._last_motion_result()
        self._log("RobotInit: request HOME plan_pose via RosBridge (plan_request)")
        self._ros.moveit_plan_pose(home, run=self._run, req_id=self._req_id, segment=self._SEG)

    def _request_execute_last_planned_once(self) -> None:
        if not self._do_execute: return
        if self._exec_sent_once: return
        self._exec_sent_once = True
        self._last_result_snapshot = self._last_motion_result()
        self._log("RobotInit: request execute-last-planned via RosBridge (plan_request)")
        self._ros.moveit_execute_last_planned(run=self._run, req_id=self._req_id, segment=self._SEG)

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
            if cur and cur != self._last_result_snapshot:
                self._log(f"RobotInit: moveit result update: {cur}")
                self._last_result_snapshot = cur
                parsed = self._parse_keyed_result(cur)
                if parsed is None:
                    if cur.upper().startswith("ERROR"): self._finish_err(f"Home plan failed: {cur}")
                    return
                key, status = parsed
                if not self._match_key(key, op=self._OP_PLAN_POSE): return

                if status == "PLANNED:OK":
                    self._log("RobotInit: home planned (motion_result)")
                    if not self._do_execute:
                        self._finish_ok()
                        return
                    
                    # NEW: Insert delay before execute
                    self._set_state(self.S_WAIT_BEFORE_EXEC)
                    self._set_deadline(0.5) # 500ms delay
                    return
                
                if self._is_error_status(status):
                    self._finish_err(f"Home plan failed: {status}")
                    return
            if self._deadline_passed():
                self._finish_err(f"Timeout waiting for PLANNED:OK (last='{cur}')")
            return

        # ---------------- WAIT_BEFORE_EXEC (NEW) ----------------
        if st == self.S_WAIT_BEFORE_EXEC:
            if self._deadline_passed():
                self._set_state(self.S_HOME_EXEC_REQUESTED)
                self._deadline_ms = 0
            return

        # ---------------- HOME_EXEC_REQUESTED ----------------
        if st == self.S_HOME_EXEC_REQUESTED:
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
                
                # --- RETRY LOGIC for GOAL_REJECTED ---
                if parsed is None or self._is_error_status(parsed[1]):
                    status_str = parsed[1] if parsed else cur
                    
                    if "GOAL_REJECTED" in status_str.upper() or "BUSY" in status_str.upper():
                        if self._exec_retry_count < self._max_exec_retries:
                            self._exec_retry_count += 1
                            self._log(f"RobotInit: GOAL_REJECTED, retry {self._exec_retry_count}/{self._max_exec_retries} (RE-PLANNING)...")
                            
                            # Reset flags to allow re-sending EVERYTHING (Plan + Exec)
                            # Often a plan becomes invalid if execution was rejected.
                            self._plan_sent_once = False
                            self._exec_sent_once = False
                            
                            # Go back to PLAN request state
                            self._set_state(self.S_HOME_PLAN_REQUESTED)
                            self._last_result_snapshot = "" 
                            return
                # -------------------------------------

                if parsed is None:
                    if cur.upper().startswith("ERROR"): self._finish_err(f"Home execute failed: {cur}")
                    return
                
                key, status = parsed
                if not self._match_key(key, op=self._OP_EXECUTE): return

                if status == "EXECUTED:OK":
                    self._log("RobotInit: home executed (motion_result)")
                    self._finish_ok()
                    return
                if status in ("STOP:REQ", "STOPPED:USER", "STOPPED"):
                    self._finish_stopped()
                    return
                if self._is_error_status(status):
                    self._finish_err(f"Home execute failed: {status}")
                    return

            if self._deadline_passed():
                self._finish_err(f"Timeout waiting for EXECUTED:OK (last='{cur}')")
            return

        self._finish_err(f"Unknown state '{st}'")