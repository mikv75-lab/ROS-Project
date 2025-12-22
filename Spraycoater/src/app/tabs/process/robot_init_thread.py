# -*- coding: utf-8 -*-
# File: tabs/process/robot_init_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from .robot_init_statemachine import RobotInitStatemachine

_LOG = logging.getLogger("app.tabs.process.robot_init_thread")


class RobotInitThread(QtCore.QObject):
    """
    Persistent Worker + QThread (nur RosBridge).

    - QThread läuft dauerhaft (einmal gestartet)
    - Worker wird einmal erzeugt und lebt im QThread
    - startSignal triggert einen Run (wenn nicht bereits running)
    - stopSignal bricht den Run ab (ohne RosBridge zu zerstören!)

    API:
      - startSignal.emit() -> startet Run
      - stopSignal.emit()  -> stoppt Run
      - request_stop()     -> stoppt Run
      - isRunning()        -> ob Run aktiv ist
    """

    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    notifyFinished = QtCore.pyqtSignal()
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        init_timeout_s: float = 10.0,
        home_timeout_s: float = 60.0,
        pos_tol_mm: float = 1.0,
    ) -> None:
        super().__init__(parent)

        if ros is None:
            raise ValueError("RobotInitThread: ros ist None")

        self._ros = ros
        self._init_timeout_s = float(init_timeout_s)
        self._home_timeout_s = float(home_timeout_s)
        self._pos_tol_mm = float(pos_tol_mm)

        self._thread = QtCore.QThread()
        self._worker: Optional[RobotInitStatemachine] = None
        self._running: bool = False

        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self._on_stop_signal)
        self._thread.finished.connect(self._on_thread_finished)

        app = QCoreApplication.instance()
        if app is not None:
            try:
                app.aboutToQuit.connect(self._on_app_about_to_quit)
            except Exception:
                pass

        self._start_thread_if_needed()
        self._create_worker_once()

        _LOG.info(
            "RobotInitThread init (ros-only): init_timeout=%.1fs home_timeout=%.1fs tol=%.2fmm",
            self._init_timeout_s,
            self._home_timeout_s,
            self._pos_tol_mm,
        )

    # ------------------------------------------------------------------ #
    # Lifecycle
    # ------------------------------------------------------------------ #

    def _start_thread_if_needed(self) -> None:
        if not self._thread.isRunning():
            _LOG.info("RobotInitThread: starte QThread (persistent).")
            self._thread.start()

    def _create_worker_once(self) -> None:
        if self._worker is not None:
            return

        worker = RobotInitStatemachine(
            ros=self._ros,
            init_timeout_s=self._init_timeout_s,
            home_timeout_s=self._home_timeout_s,
            pos_tol_mm=self._pos_tol_mm,
        )
        worker.moveToThread(self._thread)

        # Worker -> UI
        worker.stateChanged.connect(self.stateChanged)
        worker.logMessage.connect(self.logMessage)
        worker.notifyFinished.connect(self._on_worker_finished)
        worker.notifyError.connect(self._on_worker_error)

        self._worker = worker

    # ------------------------------------------------------------------ #
    # Public API
    # ------------------------------------------------------------------ #

    def isRunning(self) -> bool:
        return self._running

    def request_stop(self) -> None:
        w = self._worker
        if w is None:
            return
        QtCore.QMetaObject.invokeMethod(
            w,
            "request_stop",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    def wait(self, timeout_ms: int = 2000) -> None:
        try:
            self._thread.wait(timeout_ms)
        except Exception:
            _LOG.exception("RobotInitThread.wait: Fehler beim Warten.")

    # ------------------------------------------------------------------ #
    # Slots start / stop
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_start_signal(self) -> None:
        self._start_thread_if_needed()
        self._create_worker_once()

        if self._running:
            _LOG.info("RobotInitThread: Start ignoriert (läuft bereits).")
            return

        if self._worker is None:
            self.notifyError.emit("RobotInitThread: Worker fehlt.")
            return

        self._running = True
        QtCore.QMetaObject.invokeMethod(
            self._worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        self.request_stop()

    # ------------------------------------------------------------------ #
    # Worker callbacks
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_worker_finished(self) -> None:
        self._running = False
        self.notifyFinished.emit()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        self._running = False
        self.notifyError.emit(msg)

    # ------------------------------------------------------------------ #
    # Thread / App shutdown
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        _LOG.info("RobotInitThread: QThread beendet.")
        try:
            self.finished.emit()
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def _on_app_about_to_quit(self) -> None:
        _LOG.info("RobotInitThread: aboutToQuit – stoppe Worker/Thread.")
        try:
            self.request_stop()
        except Exception:
            pass

        try:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("RobotInitThread._on_app_about_to_quit: Fehler beim Thread-Shutdown.")

        try:
            if self._worker is not None:
                self._worker.deleteLater()
        except Exception:
            pass
        self._worker = None

    def deleteLater(self) -> None:
        try:
            self.request_stop()
        except Exception:
            pass

        try:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("RobotInitThread.deleteLater: Fehler beim Thread-Shutdown.")

        try:
            if self._worker is not None:
                self._worker.deleteLater()
        except Exception:
            pass
        self._worker = None

        try:
            self._thread.deleteLater()
        except Exception:
            pass

        super().deleteLater()
