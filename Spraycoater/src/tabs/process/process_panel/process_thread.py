# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any

from PyQt6 import QtCore

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult

from .validate_statemachine import ProcessValidateStatemachine
from .optimize_statemachine import ProcessOptimizeStatemachine
from .execute_statemachine import ProcessExecuteStatemachine

_LOG = logging.getLogger("tabs.process.thread")


class ProcessThread(QtCore.QObject):
    """
    Runs Validate / Optimize / Execute in a dedicated QThread.

    STRICT (collect-only):
      - The process statemachines may populate raw artifacts in RunResult (traj, logs, etc.)
      - This thread does NOT run FK/TCP postprocess and does NOT run evaluation.
      - ProcessPanel persists raw trajectories only.

    Signals:
      - stateChanged(str): current segment/state name
      - logMessage(str): textual log lines
      - notifyFinished(object): payload dict (RunResult.to_process_payload())
      - notifyError(str): error string
    """

    MODE_VALIDATE = "validate"
    MODE_OPTIMIZE = "optimize"
    MODE_EXECUTE = "execute"

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)
    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    _STOP_FORCE_QUIT_MS = 500  # safety net if worker never emits after stop

    def __init__(
        self,
        *,
        ctx: Any,
        recipe: Recipe,
        ros: RosBridge,
        plc: PlcClientBase | None,
        mode: str,
        urdf_xml: str = "",
        srdf_xml: str = "",
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 100,
    ) -> None:
        super().__init__(parent)

        if recipe is None:
            raise RuntimeError("ProcessThread: recipe is None (strict)")
        if ros is None:
            raise RuntimeError("ProcessThread: ros is None (strict)")

        self._ctx = ctx
        self._recipe = recipe
        self._ros = ros
        self._plc = plc
        self._mode = str(mode or "").strip().lower()

        # Collect-only: still embed robot_description into RunResult so RecipePanel can postprocess.
        self._urdf_xml = str(urdf_xml or "")
        self._srdf_xml = str(srdf_xml or "")

        self._max_retries = int(max_retries)

        self._thread: Optional[QtCore.QThread] = None
        self._worker: Optional[QtCore.QObject] = None
        self._stop_requested: bool = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            self.logMessage.emit("ProcessThread: start ignored (already running).")
            return

        self._stop_requested = False

        t = QtCore.QThread(self)
        t.setObjectName("ProcessThread")
        t.started.connect(self._on_thread_started)
        t.finished.connect(self._on_thread_finished)

        self._thread = t
        t.start()

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._stop_requested = True

        w = self._worker
        if w is not None and hasattr(w, "request_stop"):
            try:
                QtCore.QMetaObject.invokeMethod(
                    w,
                    "request_stop",
                    QtCore.Qt.ConnectionType.QueuedConnection,
                )
            except Exception as e:
                self.logMessage.emit(f"Stop: invokeMethod(request_stop) failed: {e!r}")

        QtCore.QTimer.singleShot(self._STOP_FORCE_QUIT_MS, self._force_quit_after_stop)

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _create_worker(self) -> QtCore.QObject:
        # Seed RunResult with robot description (strict: downstream RecipePanel postprocess needs it)
        rr = RunResult(urdf_xml=self._urdf_xml, srdf_xml=self._srdf_xml)

        side = "top"
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            side = str(params.get("active_side", side) or side)
        except Exception:
            pass

        mode = self._mode
        if mode == self.MODE_VALIDATE:
            return ProcessValidateStatemachine(
                recipe=self._recipe,
                ros=self._ros,
                run_result=rr,
                parent=None,
                max_retries=self._max_retries,
                side=side,
            )
        if mode == self.MODE_OPTIMIZE:
            return ProcessOptimizeStatemachine(
                recipe=self._recipe,
                ros=self._ros,
                run_result=rr,
                parent=None,
                max_retries=self._max_retries,
                side=side,
            )
        if mode == self.MODE_EXECUTE:
            return ProcessExecuteStatemachine(
                recipe=self._recipe,
                ros=self._ros,
                plc=self._plc,
                run_result=rr,
                parent=None,
                max_retries=self._max_retries,
                side=side,
            )

        raise RuntimeError(f"ProcessThread: unknown mode={mode!r}")

    def _wire_worker_signals(self, w: QtCore.QObject) -> None:
        if hasattr(w, "logMessage"):
            try:
                w.logMessage.connect(self.logMessage.emit)  # type: ignore[attr-defined]
            except Exception:
                pass

        if hasattr(w, "stateChanged"):
            try:
                w.stateChanged.connect(self.stateChanged.emit)  # type: ignore[attr-defined]
            except Exception:
                pass

        if hasattr(w, "notifyFinished"):
            try:
                w.notifyFinished.connect(self._on_worker_finished)  # type: ignore[attr-defined]
            except Exception:
                pass

        if hasattr(w, "notifyError"):
            try:
                w.notifyError.connect(self._on_worker_error)  # type: ignore[attr-defined]
            except Exception:
                pass

    def _start_worker_queued(self, w: QtCore.QObject) -> None:
        """
        Start worker in its own thread context.

        STRICT contract:
          - prefer worker.start() via queued invokeMethod
          - else accept worker.startSignal.emit()
          - no direct run() call
        """
        fn = getattr(w, "start", None)
        if callable(fn):
            QtCore.QMetaObject.invokeMethod(
                w,
                "start",
                QtCore.Qt.ConnectionType.QueuedConnection,
            )
            return

        sig = getattr(w, "startSignal", None)
        if sig is not None and hasattr(sig, "emit"):
            sig.emit()
            return

        raise RuntimeError("ProcessThread: worker has no start() and no startSignal")

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _on_thread_started(self) -> None:
        if self._stop_requested:
            self._shutdown_thread()
            return

        try:
            w = self._create_worker()
        except Exception as e:
            self.notifyError.emit(str(e))
            self._shutdown_thread()
            return

        self._worker = w

        if self._thread is not None:
            try:
                w.moveToThread(self._thread)
            except Exception:
                pass

        self._wire_worker_signals(w)

        try:
            self._start_worker_queued(w)
        except Exception as e:
            self.notifyError.emit(str(e))
            self._shutdown_thread()

    def _on_worker_finished(self, payload: object = None) -> None:
        self.notifyFinished.emit(payload if payload is not None else {})
        self._shutdown_thread()

    def _on_worker_error(self, msg: object = "") -> None:
        self.notifyError.emit(str(msg or "Unknown error"))
        self._shutdown_thread()

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        self._thread = None
        self._worker = None

    # ------------------------------------------------------------------
    # Stop safety net
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _force_quit_after_stop(self) -> None:
        if not self._stop_requested:
            return
        t = self._thread
        if t is None:
            return
        try:
            if t.isRunning():
                self.logMessage.emit("Stop: force quitting thread (worker unresponsive)")
                t.quit()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _shutdown_thread(self) -> None:
        w = self._worker
        if w is not None:
            try:
                w.deleteLater()
            except Exception:
                pass

        t = self._thread
        if t is not None:
            try:
                t.quit()
            except Exception:
                pass
            try:
                t.wait(2000)
            except Exception:
                pass
            try:
                t.deleteLater()
            except Exception:
                pass
