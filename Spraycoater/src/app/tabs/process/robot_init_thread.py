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

      1. Prüfen, ob Robot schon initialisiert ist (RobotBridge.initialized).
         - Falls nein: INIT-Kommando senden und auf initialized=True warten.
      2. Prüfen, ob TCP-Pose bereits auf Home-Pose liegt (Pose-Vergleich).
         - Falls nein: Home-Fahrt anfordern (MotionBridge) und warten,
           bis TCP≈Home ist.

    Abbruch bei Timeout, Motion-Error oder Stop-Request.
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
        `bridge` ist deine UIBridge-Instanz.
        Es wird erwartet, dass sie folgende Attribute besitzt:

          - bridge._rb           -> RobotBridge (mit .initialized, .signals.initRequested, tcp_pose())
          - bridge._motion       -> MotionBridge (mit .signals.moveToHomeRequested)
          - bridge.robot         -> RobotState (mit .tcp_pose())
          - bridge.poses         -> PosesState (mit .home())
        """
        super().__init__(parent)

        # Direkte Referenzen ohne getattr/hasattr
        self._bridge = bridge
        self._rb = bridge._rb                 # RobotBridge
        self._motion = bridge._motion         # MotionBridge

        # State-Container der UIBridge (signal-frei, thread-safe)
        self._robot_state = bridge.robot      # RobotState
        self._poses_state = bridge.poses      # PosesState

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None

        # letztes motion_result (für Fehlererkennung)
        self._last_motion_result: Optional[str] = None

        # Signale verdrahten
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

        # Motion-Result-Listener (direkt, ohne Fallbacks)
        if self._motion is not None:
            self._motion.signals.motionResultChanged.connect(self._on_motion_result)

    # ------------------------------------------------------------------
    # Public API (Slots)
    # ------------------------------------------------------------------

    def _on_start_signal(self) -> None:
        """Slot für startSignal – startet den Thread, falls er nicht läuft."""
        if not self.isRunning():
            self.start()

    def request_stop(self) -> None:
        """Von außen aufgerufen, um einen Abbruch anzufordern."""
        self._stop_requested = True

    def _should_stop(self) -> bool:
        return self._stop_requested

    # ------------------------------------------------------------------
    # Motion-Result Handling
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        """
        Callback für /motion_result (MotionBridge).

        Wir merken uns immer den letzten Text, damit _wait_for_home()
        sofort bei ERROR:... abbrechen kann.
        """
        self._last_motion_result = (text or "").strip()

    def _has_motion_error(self) -> bool:
        """
        True, wenn das letzte motion_result einen Fehler andeutet.
        """
        res = (self._last_motion_result or "").strip().upper()
        if not res:
            return False
        # einfache Heuristik: ERROR:* oder enthält EXECUTE_FAILED
        return res.startswith("ERROR:") or ("EXECUTE_FAILED" in res)

    # ------------------------------------------------------------------
    # QThread.run – Ablauf Init + Home
    # ------------------------------------------------------------------

    def run(self) -> None:
        # Zustand zurücksetzen
        self._stop_requested = False
        self._error_msg = None
        self._last_motion_result = None

        try:
            # 1. INIT falls nötig
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

            # 2. Home-Pose checken (Pose-Vergleich)
            if not self._is_at_home():
                if not self._send_home():
                    self._error_msg = "Robot-Init: MotionBridge für Home-Fahrt nicht verfügbar."
                    return

                if not self._wait_for_home(self._home_timeout_s):
                    # Falls noch keine spezifische Fehlermeldung gesetzt,
                    # ist es ein klassischer Timeout.
                    if not self._error_msg:
                        self._error_msg = (
                            f"Robot-Init: Timeout nach {self._home_timeout_s:.1f} s "
                            "beim Anfahren der Home-Pose (TCP ≉ Home-Pose)."
                        )
                    return

        except Exception as e:
            self._error_msg = str(e)
        finally:
            # Ergebnis signalisieren
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
        """
        Prüft initialized-Flag aus RobotBridge.
        """
        if self._rb is None:
            return False
        return bool(self._rb.initialized)

    def _send_init(self) -> None:
        """
        INIT-Kommando über RobotBridge auslösen.
        Erwartet: self._rb.signals.initRequested
        """
        if self._rb is None:
            return

        self._rb.signals.initRequested.emit()

    def _wait_for_initialized(self, timeout_s: float) -> bool:
        """
        Wartet bis initialized=True oder Timeout.
        """
        if timeout_s <= 0.0:
            return self._is_initialized()

        elapsed_ms = 0
        step_ms = 50
        max_ms = int(timeout_s * 1000.0)

        while elapsed_ms < max_ms:
            if self._should_stop():
                return False
            if self._is_initialized():
                return True
            self.msleep(step_ms)
            elapsed_ms += step_ms

        return self._is_initialized()

    # ------------------------------------------------------------------
    # Schritt 2: Home-Fahrt (Pose-basierter Check)
    # ------------------------------------------------------------------

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        """
        Holt die aktuelle TCP-Pose aus dem RobotState-Cache der UIBridge.
        """
        if self._robot_state is None:
            return None
        pose = self._robot_state.tcp_pose()
        if isinstance(pose, PoseStamped) or pose is None:
            return pose
        return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        """
        Holt die Home-Pose aus dem PosesState-Cache der UIBridge.
        """
        if self._poses_state is None:
            return None
        pose = self._poses_state.home()
        if isinstance(pose, PoseStamped) or pose is None:
            return pose
        return None

    @staticmethod
    def _poses_close(a: PoseStamped, b: PoseStamped, pos_tol_mm: float) -> bool:
        try:
            dx = float(a.pose.position.x) - float(b.pose.position.x)
            dy = float(a.pose.position.y) - float(b.pose.position.y)
            dz = float(a.pose.position.z) - float(b.pose.position.z)
        except Exception:
            return False

        dist_m = math.sqrt(dx * dx + dy * dy + dz * dz)
        dist_mm = dist_m * 1000.0
        return dist_mm <= pos_tol_mm

    def _is_at_home(self) -> bool:
        """
        Prüft, ob TCP-Pose und Home-Pose innerhalb pos_tol_mm liegen.
        """
        cur = self._get_tcp_pose()
        home = self._get_home_pose()
        if cur is None or home is None:
            return False
        return self._poses_close(cur, home, self._pos_tol_mm)

    def _send_home(self) -> bool:
        """
        Triggert eine Home-Fahrt über MotionBridge.
        Erwartet: self._motion.signals.moveToHomeRequested
        """
        if self._motion is None:
            return False

        self._motion.signals.moveToHomeRequested.emit()
        return True

    def _wait_for_home(self, timeout_s: float) -> bool:
        """
        Wartet bis TCP≈Home-Pose oder Timeout oder Motion-Error.
        """
        if timeout_s <= 0.0:
            return self._is_at_home()

        elapsed_ms = 0
        step_ms = 50
        max_ms = int(timeout_s * 1000.0)

        while elapsed_ms < max_ms:
            if self._should_stop():
                return False

            # Sofortiger Abbruch bei Motion-Error (z.B. EXECUTE_FAILED)
            if self._has_motion_error():
                if not self._error_msg:
                    self._error_msg = (
                        "Robot-Init: Fehler beim Anfahren der Home-Pose: "
                        f"{self._last_motion_result or 'EXECUTE_FAILED'}"
                    )
                return False

            if self._is_at_home():
                return True

            self.msleep(step_ms)
            elapsed_ms += step_ms

        # letzter Check nach Timeout
        if self._is_at_home():
            return True

        # Falls nach Timeout ein Motion-Error vorliegt, präzise Fehlermeldung setzen
        if self._has_motion_error() and not self._error_msg:
            self._error_msg = (
                "Robot-Init: Fehler beim Anfahren der Home-Pose (nach Timeout): "
                f"{self._last_motion_result or 'EXECUTE_FAILED'}"
            )

        return False
