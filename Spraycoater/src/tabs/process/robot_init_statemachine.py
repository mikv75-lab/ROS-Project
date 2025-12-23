# -*- coding: utf-8 -*-
# File: src/app/tabs/process/robot_init_statemachine.py
from __future__ import annotations

import math
import logging
from typing import Optional

from PyQt6 import QtCore
from PyQt6.QtCore import QTimer

from geometry_msgs.msg import PoseStamped

_LOG = logging.getLogger("tabs.process.robot_init_sm")


class RobotInitStatemachine(QtCore.QObject):
    """
    Robot Init + Home (Run-once Worker)

    Contract:
      - nutzt NUR RosBridge High-Level API
      - liest NUR über ros.robot_state / ros.poses_state
      - STOP darf NICHT ros.stop() aufrufen (keine Node-Destroy!)
        -> best effort: motion stop + robot stop, dann Run abbrechen

    Ablauf:
      1) FETCH_HOME: ros.set_home(), kurz warten bis poses_state.home() verfügbar (timeout)
      2) INIT: falls !initialized -> ros.robot_init(), warten bis initialized True (timeout)
      3) HOME: falls TCP nicht an Home -> ros.moveit_move_home(), warten bis TCP≈Home (timeout / moveit error)
    """

    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    S_IDLE = "IDLE"
    S_FETCH_HOME = "FETCH_HOME"
    S_WAIT_HOME_AVAILABLE = "WAIT_HOME_AVAILABLE"
    S_INIT_REQUESTED = "INIT_REQUESTED"
    S_WAIT_INITIALIZED = "WAIT_INITIALIZED"
    S_HOME_REQUESTED = "HOME_REQUESTED"
    S_WAIT_HOME = "WAIT_HOME"
    S_FINISHED = "FINISHED"
    S_ERROR = "ERROR"
    S_STOPPED = "STOPPED"

    def __init__(
        self,
        *,
        ros,
        init_timeout_s: float = 10.0,
        home_timeout_s: float = 60.0,
        pos_tol_mm: float = 1.0,
        parent: Optional[QtCore.QObject] = None,
    ) -> None:
        super().__init__(parent)
        self._ros = ros

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._stop_requested: bool = False
        self._done: bool = False

        self._state: str = self.S_IDLE
        self._deadline_ms: int = 0

        # run-once flags
        self._home_requested_once = False
        self._init_sent_once = False
        self._home_sent_once = False

        # tick timer
        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------
    # Public slots
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        self._stop_requested = False
        self._done = False
        self._deadline_ms = 0

        self._home_requested_once = False
        self._init_sent_once = False
        self._home_sent_once = False

        self._set_state(self.S_FETCH_HOME)
        self._log(
            f"RobotInit: start (init_timeout={self._init_timeout_s:.1f}s, "
            f"home_timeout={self._home_timeout_s:.1f}s, tol={self._pos_tol_mm:.2f}mm)"
        )

        self._timer.start()
        self._tick()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        STOP = Run abbrechen, ohne RosBridge/Nodes zu zerstören.
        """
        if self._done:
            return

        self._stop_requested = True
        self._log("RobotInit: stop requested -> stopping motion/robot (NOT ros.stop())")

        # best effort stop: MoveIt
        try:
            if hasattr(self._ros, "stop_all"):
                self._ros.stop_all()
            elif hasattr(self._ros, "moveit_stop"):
                self._ros.moveit_stop()
        except Exception:
            _LOG.exception("RobotInit: moveit stop failed")

        # best effort stop: Robot
        try:
            if hasattr(self._ros, "robot_stop"):
                self._ros.robot_stop()
        except Exception:
            _LOG.exception("RobotInit: robot_stop failed")

    # ------------------------------------------------------------------
    # Time helpers
    # ------------------------------------------------------------------

    def _now_ms(self) -> int:
        return int(QtCore.QDateTime.currentMSecsSinceEpoch())

    def _set_deadline(self, timeout_s: float) -> None:
        self._deadline_ms = self._now_ms() + max(0, int(timeout_s * 1000.0))

    def _deadline_passed(self) -> bool:
        return self._deadline_ms > 0 and self._now_ms() >= self._deadline_ms

    # ------------------------------------------------------------------
    # State / log / finish
    # ------------------------------------------------------------------

    def _set_state(self, s: str) -> None:
        if s != self._state:
            self._state = s
            self.stateChanged.emit(s)

    def _log(self, msg: str) -> None:
        if msg:
            self.logMessage.emit(msg)

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
        self._log("RobotInit: stopped")
        self.notifyError.emit("Robot-Init abgebrochen.")

    # ------------------------------------------------------------------
    # Guards
    # ------------------------------------------------------------------

    def _bridge_alive(self) -> bool:
        try:
            return bool(getattr(self._ros, "is_running", False))
        except Exception:
            return False

    # ------------------------------------------------------------------
    # State reads
    # ------------------------------------------------------------------

    def _is_initialized(self) -> bool:
        try:
            rs = getattr(self._ros, "robot_state", None)
            return bool(rs.initialized()) if rs is not None else False
        except Exception:
            return False

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        try:
            rs = getattr(self._ros, "robot_state", None)
            p = rs.tcp_pose() if rs is not None else None
            return p if isinstance(p, PoseStamped) else None
        except Exception:
            return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        try:
            ps = getattr(self._ros, "poses_state", None)
            p = ps.home() if ps is not None else None
            return p if isinstance(p, PoseStamped) else None
        except Exception:
            return None

    def _poses_close(self, a: PoseStamped, b: PoseStamped) -> bool:
        dx = float(a.pose.position.x) - float(b.pose.position.x)
        dy = float(a.pose.position.y) - float(b.pose.position.y)
        dz = float(a.pose.position.z) - float(b.pose.position.z)
        dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        return (dist_m * 1000.0) <= self._pos_tol_mm

    def _is_at_home(self) -> bool:
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        if cur is None or home is None:
            return False
        return self._poses_close(cur, home)

    # ------------------------------------------------------------------
    # MoveIt result / error logic
    # ------------------------------------------------------------------

    def _last_motion_result(self) -> str:
        try:
            if hasattr(self._ros, "moveit_last_result"):
                return (self._ros.moveit_last_result() or "").strip()
        except Exception:
            return ""
        return ""

    def _has_motion_error(self) -> bool:
        res = self._last_motion_result().upper()
        return bool(res) and (res.startswith("ERROR") or "EXECUTE_FAILED" in res)

    # ------------------------------------------------------------------
    # Actions (run-once)
    # ------------------------------------------------------------------

    def _request_home_pose_once(self) -> None:
        if self._home_requested_once:
            return
        self._home_requested_once = True
        try:
            self._ros.set_home()
        except Exception as e:
            self._log(f"RobotInit: warning: set_home() failed: {e}")

    def _send_robot_init_once(self) -> None:
        if self._init_sent_once:
            return
        self._init_sent_once = True
        self._log("RobotInit: sending robot_init()")
        self._ros.robot_init()

    def _send_move_home_once(self) -> None:
        if self._home_sent_once:
            return
        self._home_sent_once = True
        self._log("RobotInit: sending moveit_move_home()")
        self._ros.moveit_move_home()

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _tick(self) -> None:
        if self._stop_requested:
            self._finish_stopped()
            return

        if not self._bridge_alive():
            self._finish_err("RosBridge läuft nicht (wurde gestoppt/zerstört).")
            return

        # 1) Fetch home pose (best effort, aber HOME-Fahrt braucht sie)
        if self._state in (self.S_FETCH_HOME, self.S_WAIT_HOME_AVAILABLE):
            self._request_home_pose_once()

            if self._state == self.S_FETCH_HOME:
                self._set_state(self.S_WAIT_HOME_AVAILABLE)
                self._set_deadline(2.0)

            if self._get_home_pose() is not None:
                self._log("RobotInit: home pose available")
                self._set_state(self.S_IDLE)
                self._deadline_ms = 0
            elif self._deadline_passed():
                self._log("RobotInit: warning: home pose not available yet (continuing)")
                self._set_state(self.S_IDLE)
                self._deadline_ms = 0
            else:
                return

        # 2) Init
        if not self._is_initialized():
            if self._state == self.S_IDLE:
                self._set_state(self.S_INIT_REQUESTED)
                try:
                    self._send_robot_init_once()
                except Exception as e:
                    self._finish_err(f"robot_init() fehlgeschlagen: {e}")
                    return

                self._set_state(self.S_WAIT_INITIALIZED)
                self._set_deadline(self._init_timeout_s)
                return

            if self._state == self.S_WAIT_INITIALIZED:
                if self._is_initialized():
                    self._log("RobotInit: robot initialized")
                    self._set_state(self.S_IDLE)
                    self._deadline_ms = 0
                    return

                if self._deadline_passed():
                    self._finish_err(f"Timeout nach {self._init_timeout_s:.1f}s beim Warten auf initialized.")
                    return

                return

            # converge
            self._set_state(self.S_WAIT_INITIALIZED)
            return

        # 3) Home (hier: Home-Pose muss existieren, sonst können wir nicht deterministisch vergleichen)
        if self._get_home_pose() is None:
            # wir haben init, aber keine home pose -> einmal nachfordern und kurz warten
            self._set_state(self.S_WAIT_HOME_AVAILABLE)
            self._request_home_pose_once()
            self._set_deadline(2.0)
            return

        if not self._is_at_home():
            if self._state == self.S_IDLE:
                self._set_state(self.S_HOME_REQUESTED)
                try:
                    self._send_move_home_once()
                except Exception as e:
                    self._finish_err(f"move_home() fehlgeschlagen: {e}")
                    return

                self._set_state(self.S_WAIT_HOME)
                self._set_deadline(self._home_timeout_s)
                return

            if self._state == self.S_WAIT_HOME:
                if self._has_motion_error():
                    self._finish_err(f"Home-Execute Fehler: {self._last_motion_result() or 'EXECUTE_FAILED'}")
                    return

                if self._is_at_home():
                    self._log("RobotInit: TCP is at home pose")
                    self._finish_ok()
                    return

                if self._deadline_passed():
                    self._finish_err(f"Timeout nach {self._home_timeout_s:.1f}s: TCP ≉ Home-Pose.")
                    return

                return

            self._set_state(self.S_WAIT_HOME)
            return

        # 4) Done
        self._finish_ok()
