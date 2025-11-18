# -*- coding: utf-8 -*-
# File: tabs/process/robot_init_thread.py
from __future__ import annotations

from typing import Optional

from PyQt6 import QtCore


class RobotInitThread(QtCore.QThread):
    """
    Thread für Robot-Initialisierung + Home-Fahrt basierend auf RobotBridge-Status.

    Status kommt aus RobotBridge:
      - mode:  INIT / HOMED / READY / BUSY / ERROR / ...
      - initialized: Bool
      - moving: Bool

    Ablauf:

      1. Wenn Robot bereits initialized und mode in {HOMED, READY} -> fertig.
      2. Sonst:
         - initRequested senden (oder do_init)
         - warten bis initialized=True ODER mode in {INIT, HOMED, READY, BUSY}
      3. Home:
         - wenn mode != HOMED:
             - MoveToHome auslösen (MotionBridge bevorzugt)
             - warten bis mode == HOMED

      Erfolg -> notifyFinished()
      Fehler/Timeout/Abbruch -> notifyError(msg)

    Signale (UI <-> Thread):

      - startSignal()            : von außen zum Starten
      - stopSignal()             : von außen für Abbruch
      - notifyFinished()         : Init+Home erfolgreich
      - notifyError(str message) : Fehler / Timeout / Abbruch
    """

    # Steuer-Signale von außen
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
    ):
        super().__init__(parent)

        self._bridge = bridge
        # RobotBridge
        self._rb = getattr(bridge, "_rb", None) or getattr(bridge, "robot", None)
        self._sig_rb = getattr(self._rb, "signals", None) if self._rb is not None else None
        # MotionBridge (für MoveToHome)
        self._motion = getattr(bridge, "_motion", None)

        self._stop_requested = False
        self._error_msg: Optional[str] = None

        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)

        # Außen-Signale verbinden
        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self.request_stop)

    # ------------------------------------------------------------------ #
    # Steuerung von außen
    # ------------------------------------------------------------------ #

    def _on_start_signal(self) -> None:
        if not self.isRunning():
            self.start()

    def request_stop(self) -> None:
        self._stop_requested = True

    def _should_stop(self) -> bool:
        return self._stop_requested

    # ------------------------------------------------------------------ #
    # Status-Getter aus RobotBridge
    # ------------------------------------------------------------------ #

    def _get_mode(self) -> str:
        """
        Liefert aktuellen Robot-Mode aus RobotBridge/RobotSignals.
        Erwartete Werte (Case-insensitiv):
          INIT, HOMED, READY, BUSY, ERROR, ...
        """
        m = ""
        if self._sig_rb is not None and hasattr(self._sig_rb, "mode"):
            m = getattr(self._sig_rb, "mode", "") or ""
        elif self._rb is not None and hasattr(self._rb, "mode"):
            m = getattr(self._rb, "mode", "") or ""
        return (m or "").strip()

    def _get_initialized(self) -> bool:
        """
        True, wenn:
          - RobotSignals.initialized == True ODER
          - Mode etwas "Sinnvolles" ist (INIT/HOMED/READY/BUSY).
        """
        val = False
        if self._sig_rb is not None and hasattr(self._sig_rb, "initialized"):
            val = bool(getattr(self._sig_rb, "initialized", False))
        elif self._rb is not None and hasattr(self._rb, "initialized"):
            val = bool(getattr(self._rb, "initialized", False))

        if val:
            return True

        mode = self._get_mode().lower()
        return mode in {"init", "homed", "ready", "busy"}

    def _get_moving(self) -> bool:
        if self._sig_rb is not None and hasattr(self._sig_rb, "moving"):
            return bool(getattr(self._sig_rb, "moving", False))
        if self._rb is not None and hasattr(self._rb, "moving"):
            return bool(getattr(self._rb, "moving", False))
        return False

    def _is_home_state(self) -> bool:
        """
        Liefert True, wenn der Mode "HOMED" ist.
        (Case-insensitiv, 'ready' könnte man optional auch als ok werten.)
        """
        mode = self._get_mode().lower()
        return mode == "homed"

    # ------------------------------------------------------------------ #
    # generischer Wait-Helper
    # ------------------------------------------------------------------ #

    def _wait_for(self, predicate, timeout_s: float, label: str) -> bool:
        """
        Pollt predicate() bis True oder Timeout.

        Rückgabe:
          - True  -> Bedingung erfüllt
          - False -> Timeout oder Stop; _error_msg wird gesetzt
        """
        ms_total = max(0, int(float(timeout_s) * 1000.0))
        elapsed = 0
        step = 50  # ms

        while elapsed < ms_total:
            if self._should_stop():
                self._error_msg = f"RobotInit abgebrochen während: {label}"
                return False
            try:
                if predicate():
                    return True
            except Exception:
                # Fehler im Predicate ignorieren, weiter pollen
                pass
            self.msleep(step)
            elapsed += step

        self._error_msg = f"Timeout beim Warten auf: {label}"
        return False

    # ------------------------------------------------------------------ #
    # MoveToHome-Kommando
    # ------------------------------------------------------------------ #

    def _request_move_home(self) -> bool:
        """
        Versucht, eine Home-Fahrt auszulösen.

        Preferiert:
          - MotionBridge.signals.moveToHomeRequested
        Fallback:
          - RobotBridge.move_home() oder go_home()

        Rückgabe:
          - True  -> Befehl konnte ausgelöst werden
          - False -> kein Interface vorhanden / Fehler
        """
        moved = False
        try:
            # 1) MotionBridge
            if self._motion is not None:
                sig_m = getattr(self._motion, "signals", None)
                if sig_m is not None and hasattr(sig_m, "moveToHomeRequested"):
                    sig_m.moveToHomeRequested.emit()
                    moved = True

            # 2) Fallback direkt auf RobotBridge
            if not moved and self._rb is not None:
                if hasattr(self._rb, "move_home"):
                    self._rb.move_home()
                    moved = True
                elif hasattr(self._rb, "go_home"):
                    self._rb.go_home()
                    moved = True
        except Exception as e:
            self._error_msg = f"Home-Kommando fehlgeschlagen: {e}"
            return False

        if not moved:
            self._error_msg = "Kein Interface für Move-to-Home vorhanden."
            return False

        return True

    # ------------------------------------------------------------------ #
    # Kernlogik
    # ------------------------------------------------------------------ #

    def run(self) -> None:
        self._stop_requested = False
        self._error_msg = None

        if self._rb is None:
            self._error_msg = "RobotBridge nicht verfügbar."
            self.notifyError.emit(self._error_msg)
            return

        # --- 0) Bereits fertig? ---
        if self._get_initialized() and self._is_home_state():
            # nix zu tun
            self.notifyFinished.emit()
            return

        # --- 1) INIT anfordern (falls nötig) ---
        if not self._get_initialized():
            try:
                if self._sig_rb is not None and hasattr(self._sig_rb, "initRequested"):
                    self._sig_rb.initRequested.emit()
                elif hasattr(self._rb, "do_init"):
                    self._rb.do_init()
                else:
                    self._error_msg = "RobotBridge hat kein init-Interface."
                    self.notifyError.emit(self._error_msg)
                    return
            except Exception as e:
                self._error_msg = f"Init-Kommando fehlgeschlagen: {e}"
                self.notifyError.emit(self._error_msg)
                return

            # Warten, bis "initialized" oder Mode einer der erwarteten ist
            def _init_ok():
                if self._get_initialized():
                    return True
                mode = self._get_mode().lower()
                return mode in {"init", "homed", "ready", "busy"}

            ok_init = self._wait_for(_init_ok, self._init_timeout_s, "Robot initialisiert")
            if not ok_init:
                self.notifyError.emit(self._error_msg or "Init fehlgeschlagen.")
                return

        if self._should_stop():
            self._error_msg = "RobotInit abgebrochen."
            self.notifyError.emit(self._error_msg)
            return

        # --- 2) Home-Fahrt (falls nötig) ---
        if not self._is_home_state():
            if not self._request_move_home():
                self.notifyError.emit(self._error_msg or "Move-to-Home nicht möglich.")
                return

            # Warten bis Mode = HOMED (und optional: nicht moving)
            def _home_ok():
                return self._is_home_state() and not self._get_moving()

            ok_home = self._wait_for(_home_ok, self._home_timeout_s, "Home-Fahrt")
            if not ok_home:
                self.notifyError.emit(self._error_msg or "Home-Fahrt fehlgeschlagen.")
                return

        if self._should_stop():
            self._error_msg = "RobotInit abgebrochen."
            self.notifyError.emit(self._error_msg or "RobotInit abgebrochen.")
            return

        # --- 3) Fertig ---
        self.notifyFinished.emit()
