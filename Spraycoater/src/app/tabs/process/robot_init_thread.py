# -*- coding: utf-8 -*-
# File: tabs/process/robot_init_thread.py
from __future__ import annotations

import math
from typing import Optional

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped


class RobotInitThread(QtCore.QThread):
    """
    Thread für die Robot-Initialisierung:

      1) Wenn nicht initialized: bridge.robot_init() senden und auf RobotState.initialized()==True warten.
      2) Wenn TCP nicht auf Home: bridge.motion_move_home() anfordern und warten, bis TCP≈Home ist.
         Abbruch bei Timeout, Stop-Request oder Motion-Error (bridge.motion_last_result()).
    """

    # Von außen: Start/Stop
    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    # Ergebnis-Signale nach außen
    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        bridge,
        parent: Optional[QtCore.QObject] = None,
        init_timeout_s: float = 10.0,
        home_timeout_s: float = 60.0,
        pos_tol_mm: float = 1.0,
    ) -> None:
        """
        `bridge` ist deine UIBridge-Instanz (clean contract).
        Erwartet (ohne Fallbacks):

          - bridge.ensure_connected()
          - bridge.robot_init()
          - bridge.motion_move_home()
          - bridge.motion_last_result() -> str
          - bridge.robot (RobotState)  -> .initialized(), .tcp_pose()
          - bridge.poses (PosesState)  -> .home()
        """
        super().__init__(parent)

        self._bridge = bridge
        self._bridge.ensure_connected()

        # State-Container (thread-safe via Lock in UIBridge)
        self._robot_state = self._bridge.robot
        self._poses_state = self._bridge.poses

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None

        # Start/Stop wiring
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

    # ------------------------------------------------------------------
    # Public API (Slots)
    # ------------------------------------------------------------------

    def _on_start_signal(self) -> None:
        if not self.isRunning():
            self.start()

    def request_stop(self) -> None:
        self._stop_requested = True

    def _should_stop(self) -> bool:
        return self._stop_requested

    # ------------------------------------------------------------------
    # Motion-Error Handling (über UIBridge API)
    # ------------------------------------------------------------------

    def _last_motion_result(self) -> str:
        # UIBridge garantiert: liefert String (evtl. "")
        return (self._bridge.motion_last_result() or "").strip()

    def _has_motion_error(self) -> bool:
        res = self._last_motion_result().upper()
        if not res:
            return False
        return res.startswith("ERROR:") or ("EXECUTE_FAILED" in res)

    # ------------------------------------------------------------------
    # QThread.run – Ablauf Init + Home
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._stop_requested = False
        self._error_msg = None

        try:
            # 1) INIT falls nötig
            if self._should_stop():
                return

            if not self._is_initialized():
                self._send_init()
                if not self._wait_for_initialized(self._init_timeout_s):
                    self._error_msg = (
                        f"Robot-Init: Timeout nach {self._init_timeout_s:.1f} s "
                        "beim Warten auf 'initialized'."
                    )
                    return

            if self._should_stop():
                return

            # 2) Home (Pose-Vergleich)
            if not self._is_at_home():
                self._send_home()

                if not self._wait_for_home(self._home_timeout_s):
                    if not self._error_msg:
                        self._error_msg = (
                            f"Robot-Init: Timeout nach {self._home_timeout_s:.1f} s "
                            "beim Anfahren der Home-Pose (TCP ≉ Home-Pose)."
                        )
                    return

        except Exception as e:
            self._error_msg = str(e)
        finally:
            if self._error_msg or self._should_stop():
                if not self._error_msg and self._should_stop():
                    self._error_msg = "Robot-Init abgebrochen."
                self.notifyError.emit(self._error_msg or "Unbekannter Fehler.")
            else:
                self.notifyFinished.emit()

    # ------------------------------------------------------------------
    # Schritt 1: Init
    # ------------------------------------------------------------------

    def _is_initialized(self) -> bool:
        return bool(self._robot_state.initialized())

    def _send_init(self) -> None:
        self._bridge.robot_init()

    def _wait_for_initialized(self, timeout_s: float) -> bool:
        if timeout_s <= 0.0:
            return self._is_initialized()

        step_ms = 50
        max_ms = int(timeout_s * 1000.0)
        elapsed_ms = 0

        while elapsed_ms < max_ms:
            if self._should_stop():
                return False
            if self._is_initialized():
                return True
            self.msleep(step_ms)
            elapsed_ms += step_ms

        return self._is_initialized()

    # ------------------------------------------------------------------
    # Schritt 2: Home (Pose-basierter Check)
    # ------------------------------------------------------------------

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        pose = self._robot_state.tcp_pose()
        return pose if isinstance(pose, PoseStamped) else None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        pose = self._poses_state.home()
        return pose if isinstance(pose, PoseStamped) else None

    @staticmethod
    def _poses_close(a: PoseStamped, b: PoseStamped, pos_tol_mm: float) -> bool:
        dx = float(a.pose.position.x) - float(b.pose.position.x)
        dy = float(a.pose.position.y) - float(b.pose.position.y)
        dz = float(a.pose.position.z) - float(b.pose.position.z)

        dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        dist_mm = dist_m * 1000.0
        return dist_mm <= pos_tol_mm

    def _is_at_home(self) -> bool:
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        if cur is None or home is None:
            return False
        return self._poses_close(cur, home, self._pos_tol_mm)

    def _send_home(self) -> None:
        self._bridge.motion_move_home()

    def _wait_for_home(self, timeout_s: float) -> bool:
        if timeout_s <= 0.0:
            return self._is_at_home()

        step_ms = 50
        max_ms = int(timeout_s * 1000.0)
        elapsed_ms = 0

        while elapsed_ms < max_ms:
            if self._should_stop():
                return False

            if self._has_motion_error():
                if not self._error_msg:
                    self._error_msg = (
                        "Robot-Init: Fehler beim Anfahren der Home-Pose: "
                        f"{self._last_motion_result() or 'EXECUTE_FAILED'}"
                    )
                return False

            if self._is_at_home():
                return True

            self.msleep(step_ms)
            elapsed_ms += step_ms

        if self._is_at_home():
            return True

        if self._has_motion_error() and not self._error_msg:
            self._error_msg = (
                "Robot-Init: Fehler beim Anfahren der Home-Pose (nach Timeout): "
                f"{self._last_motion_result() or 'EXECUTE_FAILED'}"
            )

        return False
