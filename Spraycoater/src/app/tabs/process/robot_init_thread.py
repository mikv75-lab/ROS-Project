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

    Abbruch bei Timeout oder Stop-Request.
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
        super().__init__(parent)
        self._bridge = bridge

        # einzelne Bridges holen
        self._rb = getattr(self._bridge, "_rb", None)        # RobotBridge
        self._motion = getattr(self._bridge, "_motion", None)  # MotionBridge
        self._poses = getattr(self._bridge, "_poses", None)    # PosesBridge

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._stop_requested: bool = False
        self._error_msg: Optional[str] = None

        # Signale verdrahten
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

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
    # QThread.run – Ablauf Init + Home
    # ------------------------------------------------------------------

    def run(self) -> None:
        self._stop_requested = False
        self._error_msg = None

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
        try:
            if self._rb is not None and hasattr(self._rb, "initialized"):
                return bool(getattr(self._rb, "initialized", False))
            sig = getattr(self._rb, "signals", None) if self._rb is not None else None
            if sig is not None and hasattr(sig, "initialized"):
                return bool(getattr(sig, "initialized", False))
        except Exception:
            pass
        return False

    def _send_init(self) -> None:
        """
        INIT-Kommando über RobotBridge auslösen.
        Bevorzugt RobotSignals.initRequested, sonst RobotBridge.do_init().
        """
        if self._rb is None:
            return

        try:
            sig = getattr(self._rb, "signals", None)
            if sig is not None and hasattr(sig, "initRequested"):
                # Qt-Signal benutzen (UI-like)
                sig.initRequested.emit()
                return

            # Fallback: direkte Methode, falls vorhanden
            if hasattr(self._rb, "do_init"):
                self._rb.do_init()
        except Exception:
            # Fehler hier sind nicht kritisch, der Timeout fängt das ab
            pass

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

        # letzter Check nach Timeout
        return self._is_initialized()

    # ------------------------------------------------------------------
    # Schritt 2: Home-Fahrt (Pose-basierter Check)
    # ------------------------------------------------------------------

    def _get_tcp_pose(self) -> Optional[PoseStamped]:
        """
        Holt die aktuelle TCP-Pose aus RobotBridge (oder RobotSignals).
        """
        rb = self._rb
        if rb is None:
            return None

        try:
            if hasattr(rb, "tcp_pose") and getattr(rb, "tcp_pose", None) is not None:
                return rb.tcp_pose  # type: ignore[no-any-return]

            sig = getattr(rb, "signals", None)
            if sig is not None and hasattr(sig, "tcp_pose") and getattr(sig, "tcp_pose", None) is not None:
                return sig.tcp_pose  # type: ignore[no-any-return]
        except Exception:
            pass
        return None

    def _get_home_pose(self) -> Optional[PoseStamped]:
        """
        Holt die Home-Pose aus PosesBridge.
        Falls du stattdessen die Home-Pose aus der Scene verwenden willst,
        kannst du diese Methode anpassen.
        """
        pb = self._poses
        if pb is None:
            return None

        try:
            if hasattr(pb, "home_pose") and getattr(pb, "home_pose", None) is not None:
                return pb.home_pose  # type: ignore[no-any-return]

            sigp = getattr(pb, "signals", None)
            if sigp is not None and hasattr(sigp, "home_pose") and getattr(sigp, "home_pose", None) is not None:
                return sigp.home_pose  # type: ignore[no-any-return]
        except Exception:
            pass
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
        Bevorzugt MotionSignals.moveToHomeRequested, sonst internen Helper.
        """
        m = self._motion
        if m is None:
            return False

        try:
            sig = getattr(m, "signals", None)
            if sig is not None and hasattr(sig, "moveToHomeRequested"):
                sig.moveToHomeRequested.emit()
                return True

            # Fallback: interne Methode, falls du z.B. _do_named("home") public machst
            if hasattr(m, "_do_named"):
                m._do_named("home")  # type: ignore[attr-defined]
                return True
        except Exception:
            return False

        return False

    def _wait_for_home(self, timeout_s: float) -> bool:
        """
        Wartet bis TCP≈Home-Pose oder Timeout.
        """
        if timeout_s <= 0.0:
            return self._is_at_home()

        elapsed_ms = 0
        step_ms = 50
        max_ms = int(timeout_s * 1000.0)

        while elapsed_ms < max_ms:
            if self._should_stop():
                return False
            if self._is_at_home():
                return True
            self.msleep(step_ms)
            elapsed_ms += step_ms

        # letzter Check nach Timeout
        return self._is_at_home()
