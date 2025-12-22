# -*- coding: utf-8 -*-
# File: src/app/tabs/process/robot_init_statemachine.py
from __future__ import annotations

import math
import logging
from typing import Optional

from PyQt6 import QtCore
from PyQt6.QtCore import QTimer

from geometry_msgs.msg import PoseStamped

_LOG = logging.getLogger("app.tabs.process.robot_init_sm")


class RobotInitStatemachine(QtCore.QObject):
    """
    Worker-Statemachine für Robot Init + Home.

    Clean Contract:
      - nutzt ausschließlich die RosBridge High-Level API
      - liest ausschließlich States über ros.robot_state / ros.poses_state
      - STOP: darf NICHT ros.stop() aufrufen (das zerstört rclpy-Nodes / Destroyable)
        Stattdessen: laufende Motion/Robot stoppen und die Statemachine abbrechen.

    Ablauf:
      A) FETCH_HOME: fordert Home-Pose an (ros.set_home()) und wartet kurz,
         bis poses.home() verfügbar ist (oder timeout)
      B) INIT: falls nicht initialized -> ros.robot_init(), warten bis initialized True (oder timeout)
      C) HOME: falls TCP nicht an Home -> ros.moveit_move_home() (named),
         warten bis TCP≈Home oder timeout; bricht ab bei moveit_last_result ERROR
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
        self._b = ros

        # Bridge starten (aber niemals hier wieder stoppen/zerstören)
        try:
            if hasattr(self._b, "ensure_connected"):
                self._b.ensure_connected()
            elif hasattr(self._b, "start") and not getattr(self._b, "is_running", False):
                self._b.start()
        except Exception:
            _LOG.exception("RobotInit: ensure_connected/start failed")

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._stop_requested: bool = False
        self._state: str = self.S_IDLE
        self._deadline_ms: int = 0
        self._done: bool = False  # prevents double-finish

        # run-once guards (per start-run)
        self._home_requested_once: bool = False
        self._init_sent_once: bool = False
        self._home_sent_once: bool = False

        self._timer = QTimer(self)
        self._timer.setInterval(50)
        self._timer.timeout.connect(self._tick)

    # ------------------------------------------------------------------
    # Qt slots
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def start(self) -> None:
        # reset run state
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
        WICHTIG:
        - NICHT ros.stop() aufrufen! Das zerstört Nodes (Destroyable) und danach geht nix mehr.
        - Stattdessen: best-effort Stop von MoveIt/Robot + Statemachine abbrechen.
        """
        self._stop_requested = True
        self._log("RobotInit: stop requested -> stopping motion/robot (NOT ros.stop())")

        # Best-effort: MoveIt stoppen
        try:
            if hasattr(self._b, "stop_all"):
                self._b.stop_all()
            elif hasattr(self._b, "moveit_stop"):
                self._b.moveit_stop()
        except Exception:
            _LOG.exception("RobotInit: moveit stop failed")

        # Best-effort: Robot stoppen
        try:
            if hasattr(self._b, "robot_stop"):
                self._b.robot_stop()
        except Exception:
            _LOG.exception("RobotInit: robot_stop failed")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _now_ms(self) -> int:
        return int(QtCore.QDateTime.currentMSecsSinceEpoch())

    def _set_deadline(self, timeout_s: float) -> None:
        self._deadline_ms = self._now_ms() + max(0, int(timeout_s * 1000.0))

    def _deadline_passed(self) -> bool:
        return self._deadline_ms > 0 and self._now_ms() >= self._deadline_ms

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
    # Bridge guards
    # ------------------------------------------------------------------

    def _bridge_alive(self) -> bool:
        try:
            # RosBridge hat is_running property
            return bool(getattr(self._b, "is_running", False))
        except Exception:
            return False

    # ------------------------------------------------------------------
    # State checks
    # ------------------------------------------------------------------

    def _is_initialized(self) -> bool:
        try:
            # in deiner neuen RosBridge: robot_state.initialized()
            if hasattr(self._b, "robot_state"):
                return bool(self._b.robot_state.initialized())
            # fallback (falls du doch noch self._b.robot verwendest)
            if hasattr(self._b, "robot") and self._b.robot is not None:
                return bool(self._b.robot.initialized())
        except Exception:
            return False
        return False

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        try:
            if hasattr(self._b, "robot_state"):
                pose = self._b.robot_state.tcp_pose()
                return pose if isinstance(pose, PoseStamped) else None
            if hasattr(self._b, "robot") and self._b.robot is not None:
                pose = self._b.robot.tcp_pose()
                return pose if isinstance(pose, PoseStamped) else None
        except Exception:
            return None
        return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        try:
            # in deiner neuen RosBridge: poses_state.home()
            if hasattr(self._b, "poses_state"):
                pose = self._b.poses_state.home()
                return pose if isinstance(pose, PoseStamped) else None
            # fallback: poses.home()
            if hasattr(self._b, "poses") and self._b.poses is not None:
                pose = self._b.poses.home()
                return pose if isinstance(pose, PoseStamped) else None
        except Exception:
            return None
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
            if hasattr(self._b, "moveit_last_result"):
                return (self._b.moveit_last_result() or "").strip()
        except Exception:
            return ""
        return ""

    def _has_motion_error(self) -> bool:
        res = self._last_motion_result().upper()
        if not res:
            return False
        return res.startswith("ERROR") or ("EXECUTE_FAILED" in res)

    def _clear_motion_result_cache_if_possible(self) -> None:
        """
        Optional: wenn du in MoveItPyBridge irgendwann 'clear last_result' einbaust.
        Aktuell gibt's keine API dafür, also no-op.
        """
        return

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    def _request_home_pose_once(self) -> None:
        if self._home_requested_once:
            return
        self._home_requested_once = True

        if not self._bridge_alive():
            self._log("RobotInit: warning: RosBridge not running -> cannot set_home()")
            return

        try:
            self._b.set_home()
        except Exception as e:
            self._log(f"RobotInit: warning: set_home() failed: {e}")

    def _send_robot_init_once(self) -> None:
        if self._init_sent_once:
            return
        self._init_sent_once = True

        if not self._bridge_alive():
            raise RuntimeError("RosBridge not running (cannot robot_init)")

        self._log("RobotInit: sending robot_init()")
        self._b.robot_init()

    def _send_home_once(self) -> None:
        if self._home_sent_once:
            return
        self._home_sent_once = True

        if not self._bridge_alive():
            raise RuntimeError("RosBridge not running (cannot move home)")

        self._clear_motion_result_cache_if_possible()
        self._log("RobotInit: sending moveit_move_home() (named, via MoveItPyBridge)")
        # erwartet: ros.moveit_move_home() existiert (wie in deinem Code)
        self._b.moveit_move_home()

    # ------------------------------------------------------------------
    # Tick
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _tick(self) -> None:
        if self._stop_requested:
            self._finish_stopped()
            return

        # Wenn Bridge weg ist, sofort sauber abbrechen (statt Destroyable-Errors zu spammen)
        if not self._bridge_alive():
            self._finish_err("RosBridge läuft nicht (wurde gestoppt/zerstört).")
            return

        # A) Ensure home pose is available (so comparisons work deterministically)
        if self._state in (self.S_FETCH_HOME, self.S_WAIT_HOME_AVAILABLE):
            self._request_home_pose_once()

            if self._state == self.S_FETCH_HOME:
                self._set_state(self.S_WAIT_HOME_AVAILABLE)
                # give it a short budget to arrive (separate from init/home timeouts)
                self._set_deadline(2.0)

            if self._get_home_pose() is not None:
                self._log("RobotInit: home pose available")
                self._set_state(self.S_IDLE)
                self._deadline_ms = 0
            elif self._deadline_passed():
                # not fatal: we can still continue, but home-check will be false until pose arrives
                self._log("RobotInit: warning: home pose not available yet (continuing anyway)")
                self._set_state(self.S_IDLE)
                self._deadline_ms = 0
            else:
                return  # keep waiting a bit

        # B) Ensure initialized
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
                    self._finish_err(
                        f"Robot-Init: Timeout nach {self._init_timeout_s:.1f}s "
                        "beim Warten auf 'initialized'."
                    )
                    return
                return

            # any other state while not initialized -> converge
            self._set_state(self.S_WAIT_INITIALIZED)
            return

        # C) Ensure home pose reached
        if not self._is_at_home():
            if self._state == self.S_IDLE:
                self._set_state(self.S_HOME_REQUESTED)
                try:
                    self._send_home_once()
                except Exception as e:
                    self._finish_err(f"Home request fehlgeschlagen: {e}")
                    return

                self._set_state(self.S_WAIT_HOME)
                self._set_deadline(self._home_timeout_s)
                return

            if self._state == self.S_WAIT_HOME:
                if self._has_motion_error():
                    self._finish_err(
                        "Robot-Init: Fehler beim Anfahren der Home-Pose: "
                        f"{self._last_motion_result() or 'EXECUTE_FAILED'}"
                    )
                    return

                if self._is_at_home():
                    self._log("RobotInit: TCP is at home pose")
                    self._finish_ok()
                    return

                if self._deadline_passed():
                    if self._has_motion_error():
                        self._finish_err(
                            "Robot-Init: Fehler beim Anfahren der Home-Pose (nach Timeout): "
                            f"{self._last_motion_result() or 'EXECUTE_FAILED'}"
                        )
                    else:
                        self._finish_err(
                            f"Robot-Init: Timeout nach {self._home_timeout_s:.1f}s "
                            "beim Anfahren der Home-Pose (TCP ≉ Home-Pose)."
                        )
                    return
                return

            # any other state while not at home -> converge
            self._set_state(self.S_WAIT_HOME)
            return

        # D) All good
        self._finish_ok()
