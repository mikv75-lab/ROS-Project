# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from model.recipe.recipe import Recipe
from plc.plc_client import PlcClientBase  # nur für Execute

from .validate_statemachine import ProcessValidateStatemachine
from .optimize_statemachine import ProcessOptimizeStatemachine
from .execute_statemachine import ProcessExecuteStatemachine

_LOG = logging.getLogger("tabs.process.thread")


class ProcessThread(QtCore.QObject):
    """
    Führt Validate/Optimize/Execute in einem Worker-Thread aus.

    NEU (Contract):
      - Worker liefert ein Ergebnisobjekt zurück, typischerweise ein SegmentRunPayload (dict):
          { "version": 1, "meta": {...}, "segments": {...} }

      - Persistenz passiert NICHT hier.
      - ProcessTab übernimmt:
          - eval (segmentweise + optional gesamt)
          - UI
          - optional save (traj.yaml oder executed_traj.yaml; Inhalt identisch, nur anderer Name)
    """

    MODE_VALIDATE = "validate"
    MODE_OPTIMIZE = "optimize"
    MODE_EXECUTE = "execute"

    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    notifyFinished = QtCore.pyqtSignal(object)
    notifyError = QtCore.pyqtSignal(str)

    stateChanged = QtCore.pyqtSignal(str)
    logMessage = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        recipe: Optional[Recipe],
        ros: Any,
        plc: PlcClientBase | None = None,
        parent: Optional[QtCore.QObject] = None,
        mode: str = MODE_VALIDATE,
    ) -> None:
        super().__init__(parent)

        self._ros = ros
        self._plc: PlcClientBase | None = plc
        self._recipe: Optional[Recipe] = recipe
        self._mode: str = str(mode)

        self._thread = QtCore.QThread(self)
        self._worker: Optional[QtCore.QObject] = None

        self._thread.started.connect(self._on_thread_started)

    def start(self) -> None:
        self._thread.start()

    def request_stop(self) -> None:
        if self._worker is None:
            return
        try:
            QtCore.QMetaObject.invokeMethod(
                self._worker,
                "request_stop",
                QtCore.Qt.ConnectionType.QueuedConnection,
            )
        except Exception:
            _LOG.exception("request_stop: Fehler")

    # ----------------- Thread start -----------------

    def _on_thread_started(self) -> None:
        QCoreApplication.processEvents()

        if self._recipe is None:
            self._on_worker_error("Kein Recipe übergeben.")
            return

        if self._mode == self.MODE_VALIDATE:
            self._worker = ProcessValidateStatemachine(recipe=self._recipe, ros=self._ros)
        elif self._mode == self.MODE_OPTIMIZE:
            self._worker = ProcessOptimizeStatemachine(recipe=self._recipe, ros=self._ros)
        elif self._mode == self.MODE_EXECUTE:
            if self._plc is None:
                self._on_worker_error("PLC fehlt (Execute benötigt PLC).")
                return
            self._worker = ProcessExecuteStatemachine(recipe=self._recipe, ros=self._ros, plc=self._plc)
        else:
            self._on_worker_error(f"Unbekannter Mode: {self._mode}")
            return

        # forward signals
        self._worker.stateChanged.connect(self.stateChanged)
        self._worker.logMessage.connect(self.logMessage)
        self._worker.notifyFinished.connect(self._on_worker_finished)
        self._worker.notifyError.connect(self._on_worker_error)

        # move to thread and start
        self._worker.moveToThread(self._thread)

        try:
            QtCore.QMetaObject.invokeMethod(
                self._worker,
                "start",
                QtCore.Qt.ConnectionType.QueuedConnection,
            )
        except Exception as e:
            self._on_worker_error(f"Worker start failed: {e}")

    # ----------------- Worker callbacks -----------------

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, result_obj: object) -> None:
        self.notifyFinished.emit(result_obj)
        self._cleanup_worker()
        self._quit_thread()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        self.notifyError.emit(msg)
        self._cleanup_worker()
        self._quit_thread()

    def _quit_thread(self) -> None:
        if not self._thread.isRunning():
            return
        try:
            self._thread.quit()
            self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread._quit_thread: Fehler beim Thread-Shutdown.")

    def _cleanup_worker(self) -> None:
        if self._worker is None:
            return

        # best-effort stop
        try:
            QtCore.QMetaObject.invokeMethod(
                self._worker,
                "request_stop",
                QtCore.Qt.ConnectionType.QueuedConnection,
            )
        except Exception:
            pass

        try:
            self._worker.deleteLater()
        except Exception:
            pass
        self._worker = None
