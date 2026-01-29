# -*- coding: utf-8 -*-
# File: tabs/process/robot_init_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any, Tuple

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from .robot_init_statemachine import RobotInitStatemachine

_LOG = logging.getLogger("tabs.process.robot_init_thread")


class RobotInitThread(QtCore.QObject):
    """
    Persistent Worker + QThread (Ros-only)

    Problem:
      - stale MoveItPy motion_result / segment leaks from previous Process runs
        into RobotInit HOME execution (ERROR:NO_TRAJ named seg=...).
      - Stop can leave UI "blocked" if worker doesn't emit callbacks promptly.

    Fix:
      - HARD boundary reset on EVERY RobotInit start (pre-run).
      - Optional boundary reset on finish/error (post-run) as extra safety.
      - request_stop() unblocks UI semantics immediately by clearing _running,
        then asks worker to stop.
      - CRITICAL: Segment context is reset best-effort (legacy compatibility),
        but Variant-A is keyed and segment is provided per request anyway.
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

        self._thread = QtCore.QThread(self)

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
            "RobotInitThread init: init_timeout=%.1fs home_timeout=%.1fs tol=%.2fmm",
            self._init_timeout_s,
            self._home_timeout_s,
            self._pos_tol_mm,
        )

    # ------------------------------------------------------------------
    # Boundary reset helpers (best-effort, ordered)
    # ------------------------------------------------------------------

    def _call_first(self, names: Tuple[str, ...]) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                try:
                    fn()
                except Exception as e:
                    try:
                        self.logMessage.emit(f"W: ros.{n}() failed: {e!r}")
                    except Exception:
                        pass
                return True
        return False

    def _call_first_with(self, names: Tuple[str, ...], *args: Any, **kwargs: Any) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                try:
                    fn(*args, **kwargs)
                except Exception as e:
                    try:
                        self.logMessage.emit(f"W: ros.{n}(*args) failed: {e!r}")
                    except Exception:
                        pass
                return True
        return False

    def _reset_segment_context(self) -> None:
        """
        Best-effort reset of legacy "segment context".

        Variant-A does NOT require a set_segment topic (segment is per request),
        but we keep this to guard older facades that may still leak state.
        """
        if self._call_first(("moveit_clear_segment", "moveit_reset_segment", "moveit_set_segment_none")):
            return

        # Fallback: minimal API
        self._call_first_with(("moveit_set_segment",), "")

        try:
            self._call_first_with(("moveit_set_segment",), None)  # type: ignore[arg-type]
        except Exception:
            pass

    def _reset_run_boundary(self) -> None:
        """
        Clear stale MoveIt/bridge state before/after RobotInit runs.

        Goal:
          - prevent RobotInit HOME using previous segment/result (e.g. MOVE_RECIPE)
          - prevent UI/state-machine from seeing old motion_result / traj snapshots

        Note:
          This is best-effort because RosBridge may not expose all clear APIs.
        """
        # 1) stop any motion
        self._call_first(("moveit_stop", "stop_moveit", "moveit_cancel", "moveit_abort"))

        # 2) clear last motion result
        self._call_first(("moveit_clear_motion_result", "moveit_reset_motion_result", "moveit_clear_result"))

        # 3) clear trajectory caches
        self._call_first(
            (
                "moveit_clear_trajectory_cache",
                "moveit_clear_traj_cache",
                "moveit_traj_cache_clear",
                "moveit_clear_cache",
            )
        )

        # 4) legacy: clear segment context
        self._reset_segment_context()

        try:
            QCoreApplication.processEvents()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Thread / worker lifecycle
    # ------------------------------------------------------------------

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

        worker.stateChanged.connect(self.stateChanged)
        worker.logMessage.connect(self.logMessage)
        worker.notifyFinished.connect(self._on_worker_finished)
        worker.notifyError.connect(self._on_worker_error)

        self._worker = worker

    # ---------------- Public API ----------------

    def isRunning(self) -> bool:
        return self._running

    def request_stop(self) -> None:
        """
        Stop RobotInit worker (best-effort) and reset MoveIt boundary.

        IMPORTANT UI semantics:
          - immediately clear `_running` so UI can re-start without waiting for
            an async callback (worker may still be shutting down).
        """
        # Allow re-start immediately
        self._running = False

        # Stop motion + clear boundary immediately
        try:
            self.logMessage.emit("RobotInitThread: boundary reset (request_stop)...")
        except Exception:
            pass
        try:
            self._reset_run_boundary()
        except Exception:
            pass

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

    # ---------------- Slots ----------------

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

        # HARD boundary reset BEFORE RobotInit starts
        try:
            self.logMessage.emit("RobotInitThread: boundary reset (pre-run, stop/clear caches)...")
            self._reset_run_boundary()
        except Exception:
            pass

        self._running = True
        QtCore.QMetaObject.invokeMethod(
            self._worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        self.request_stop()

    # ---------------- Worker callbacks ----------------

    @QtCore.pyqtSlot()
    def _on_worker_finished(self) -> None:
        self._running = False

        # Post-run reset (optional safety)
        try:
            self.logMessage.emit("RobotInitThread: boundary reset (post-run)...")
            self._reset_run_boundary()
        except Exception:
            pass

        self.notifyFinished.emit()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        self._running = False

        # Post-error reset (critical safety)
        try:
            self.logMessage.emit("RobotInitThread: boundary reset (post-error)...")
            self._reset_run_boundary()
        except Exception:
            pass

        self.notifyError.emit(msg)

    # ---------------- Shutdown ----------------

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
