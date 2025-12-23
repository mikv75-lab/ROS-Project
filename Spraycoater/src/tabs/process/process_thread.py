# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional
import logging

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from.model.recipe.recipe import Recipe

from .validate_statemachine import ProcessValidateStatemachine
from .optimize_statemachine import ProcessOptimizeStatemachine
from .execute_statemachine import ProcessExecuteStatemachine

_LOG = logging.getLogger("tabs.process.thread")


class ProcessThread(QtCore.QObject):
    """
    ProcessThread (Run-per-Click)

    Flow:
      Thread startet
      Worker läuft einmal (Validate/Optimize/Execute)
      Worker finished/error
      Thread wird gequitttet
      Nächster Button => neuer frischer Run
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

    finished = QtCore.pyqtSignal()

    def __init__(
        self,
        *,
        recipe: Optional[Recipe],
        ros,
        parent: Optional[QtCore.QObject] = None,
        mode: str = MODE_VALIDATE,
    ) -> None:
        super().__init__(parent)

        self._ros = ros
        self._recipe: Optional[Recipe] = recipe
        self._mode: str = str(mode)

        # Ownership sauber
        self._thread = QtCore.QThread(self)
        self._worker: Optional[QtCore.QObject] = None

        self.startSignal.connect(self._on_start_signal)
        self.stopSignal.connect(self._on_stop_signal)

        self._thread.finished.connect(self._on_thread_finished)

        app = QCoreApplication.instance()
        if app is not None:
            try:
                app.aboutToQuit.connect(self._on_app_about_to_quit)
            except Exception:
                pass

        _LOG.info("ProcessThread init (mode=%s)", self._mode)

    # ---------------- Public API ----------------

    def isRunning(self) -> bool:
        return self._thread.isRunning()

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
            _LOG.exception("ProcessThread.wait: Fehler.")

    # ---------------- Internals ----------------

    def _create_worker(self) -> QtCore.QObject:
        if self._recipe is None:
            raise RuntimeError("ProcessThread: recipe ist None")

        if self._mode == self.MODE_EXECUTE:
            worker = ProcessExecuteStatemachine(recipe=self._recipe, ros=self._ros)
        elif self._mode == self.MODE_OPTIMIZE:
            worker = ProcessOptimizeStatemachine(recipe=self._recipe, ros=self._ros)
        else:
            worker = ProcessValidateStatemachine(recipe=self._recipe, ros=self._ros)

        worker.moveToThread(self._thread)

        worker.stateChanged.connect(self.stateChanged)           # type: ignore[attr-defined]
        worker.logMessage.connect(self.logMessage)               # type: ignore[attr-defined]
        worker.notifyFinished.connect(self._on_worker_finished)  # type: ignore[attr-defined]
        worker.notifyError.connect(self._on_worker_error)        # type: ignore[attr-defined]

        self._worker = worker
        return worker

    @QtCore.pyqtSlot()
    def _on_start_signal(self) -> None:
        if self._recipe is None:
            self.notifyError.emit("Kein Rezept geladen.")
            return
        if self._thread.isRunning():
            _LOG.warning("ProcessThread: Start ignoriert – Thread läuft bereits.")
            return

        self._cleanup_worker()

        _LOG.info("ProcessThread: starte QThread.")
        self._thread.start()

        try:
            worker = self._create_worker()
        except Exception as e:
            self.notifyError.emit(str(e))
            self._quit_thread()
            return

        QtCore.QMetaObject.invokeMethod(
            worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        self.request_stop()

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

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        _LOG.info("ProcessThread: QThread beendet.")
        self.finished.emit()

    @QtCore.pyqtSlot()
    def _on_app_about_to_quit(self) -> None:
        _LOG.info("ProcessThread: aboutToQuit – stoppe Worker/Thread.")
        try:
            self._cleanup_worker()
        except Exception:
            pass
        self._quit_thread()

    def deleteLater(self) -> None:
        try:
            self._cleanup_worker()
        except Exception:
            pass
        self._quit_thread()
        try:
            self._thread.deleteLater()
        except Exception:
            pass
        super().deleteLater()
