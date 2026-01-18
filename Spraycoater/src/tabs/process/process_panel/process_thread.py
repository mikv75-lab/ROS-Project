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

    STRICT fixes:
      - Do NOT reference non-existent worker attributes (e.g. worker.notify).
      - Always wire worker signals by capability detection.
      - Seed RunResult with URDF/SRDF provided by ProcessTab (ctx).
      - RosBridge remains unchanged.

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
        skip_home: bool = False,
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

        self._urdf_xml = str(urdf_xml or "")
        self._srdf_xml = str(srdf_xml or "")

        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

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
                w.request_stop()  # type: ignore[attr-defined]
                return
            except Exception as e:
                self.logMessage.emit(f"Stop: worker.request_stop failed: {e!r}")

        # best-effort: quit thread
        if self._thread is not None:
            try:
                self._thread.quit()
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _create_worker(self) -> QtCore.QObject:
        # seed RunResult (SSoT)
        rr = RunResult(urdf_xml=self._urdf_xml, srdf_xml=self._srdf_xml)

        # side param (best-effort; used by your SMs for compat)
        side = "top"
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            side = str(params.get("active_side", side) or side)
        except Exception:
            pass

        # skip_home best-effort from recipe if present
        skip_home = self._skip_home
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            if "skip_home" in params:
                skip_home = bool(params.get("skip_home"))
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
                skip_home=skip_home,
                side=side,
            )
        if mode == self.MODE_OPTIMIZE:
            return ProcessOptimizeStatemachine(
                recipe=self._recipe,
                ros=self._ros,
                run_result=rr,
                parent=None,
                max_retries=self._max_retries,
                skip_home=skip_home,
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
                skip_home=skip_home,
                side=side,
            )

        raise RuntimeError(f"ProcessThread: unknown mode={mode!r}")

    def _wire_worker_signals(self, w: QtCore.QObject) -> None:
        # log
        if hasattr(w, "logMessage"):
            try:
                w.logMessage.connect(self.logMessage.emit)  # type: ignore[attr-defined]
            except Exception:
                pass

        # state
        if hasattr(w, "stateChanged"):
            try:
                w.stateChanged.connect(self.stateChanged.emit)  # type: ignore[attr-defined]
            except Exception:
                pass

        # finished/error
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

        # also allow older naming (best-effort)
        if hasattr(w, "finished"):
            try:
                w.finished.connect(self._on_worker_finished)  # type: ignore[attr-defined]
            except Exception:
                pass

        if hasattr(w, "error"):
            try:
                w.error.connect(self._on_worker_error)  # type: ignore[attr-defined]
            except Exception:
                pass

    def _start_worker(self, w: QtCore.QObject) -> None:
        """
        Start the worker without assuming a specific API.
        """
        # Preferred: explicit start() method
        fn = getattr(w, "start", None)
        if callable(fn):
            fn()
            return

        # Alternative: startSignal pyqtSignal
        sig = getattr(w, "startSignal", None)
        if sig is not None and hasattr(sig, "emit"):
            sig.emit()
            return

        # Alternative: run() method
        fn = getattr(w, "run", None)
        if callable(fn):
            fn()
            return

        raise RuntimeError("ProcessThread: worker has no start()/startSignal/run()")

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

        # move worker into thread
        if self._thread is not None:
            try:
                w.moveToThread(self._thread)
            except Exception:
                pass

        self._wire_worker_signals(w)

        # start worker ASAP in its thread context
        try:
            QtCore.QTimer.singleShot(0, lambda: self._safe_start_worker())
        except Exception:
            # fallback: direct
            self._safe_start_worker()

    def _safe_start_worker(self) -> None:
        w = self._worker
        if w is None:
            return
        if self._stop_requested:
            self._shutdown_thread()
            return
        try:
            self._start_worker(w)
        except Exception as e:
            self._on_worker_error(f"Process start failed: {e}")

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, payload: object) -> None:
        self.notifyFinished.emit(payload)
        self._shutdown_thread()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        self.notifyError.emit(str(msg or "Unknown error"))
        self._shutdown_thread()

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        # cleanup (worker is deleted via deleteLater)
        self._thread = None
        self._worker = None

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def _shutdown_thread(self) -> None:
        # delete worker in its own thread
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
