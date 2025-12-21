# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

from typing import Optional
import logging

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from app.model.recipe.recipe import Recipe

from .process_validate_statemachine import ProcessValidateStatemachine
from .process_optimize_statemachine import ProcessOptimizeStatemachine
from .process_run_statemachine import ProcessRunStatemachine

_LOG = logging.getLogger("app.tabs.process.thread")


class ProcessThread(QtCore.QObject):
    """
    Wrapper, der in der GUI wie ein "Thread" genutzt wird, intern aber:
      - besitzt einen QThread (Eventloop im Hintergrund)
      - erzeugt pro Start einen Worker (StateMachine QObject) je Mode

    Modi:
      - validate: ProcessValidateStatemachine (Recipe Input, MoveItPy plan/execute)
      - optimize: ProcessOptimizeStatemachine (Run YAML Input, MoveItPy optimize)
      - run:      ProcessRunStatemachine (Run YAML Input, Replay)
    """

    MODE_VALIDATE = "validate"
    MODE_OPTIMIZE = "optimize"
    MODE_RUN = "run"

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
        bridge,
        parent: Optional[QtCore.QObject] = None,
        mode: str = MODE_VALIDATE,
        run_yaml_path: Optional[str] = None,
    ) -> None:
        super().__init__(parent)

        self._thread = QtCore.QThread()
        self._bridge = bridge

        self._recipe: Optional[Recipe] = recipe
        self._mode: str = mode or self.MODE_VALIDATE
        self._run_yaml_path: Optional[str] = run_yaml_path

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

        _LOG.info("ProcessThread init: mode=%s run_yaml=%r", self._mode, self._run_yaml_path)

    # ------------------------------------------------------------------ #
    # Öffentliche API
    # ------------------------------------------------------------------ #

    @property
    def recipe(self) -> Optional[Recipe]:
        return self._recipe

    def set_recipe(self, recipe: Recipe) -> None:
        if self.isRunning():
            _LOG.warning("ProcessThread.set_recipe ignoriert: Run läuft.")
            return
        self._recipe = recipe

    def isRunning(self) -> bool:
        return self._thread.isRunning()

    def request_stop(self) -> None:
        worker = self._worker
        if worker is None:
            return
        QtCore.QMetaObject.invokeMethod(
            worker,
            "request_stop",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    def wait(self, timeout_ms: int = 2000) -> None:
        try:
            self._thread.wait(timeout_ms)
        except Exception:
            _LOG.exception("ProcessThread.wait: Fehler beim Warten.")

    # ------------------------------------------------------------------ #
    # Worker
    # ------------------------------------------------------------------ #

    def _create_worker(self) -> QtCore.QObject:
        if self._mode == self.MODE_RUN:
            worker = ProcessRunStatemachine(
                run_yaml_path=self._run_yaml_path or "",
                bridge=self._bridge,
            )
            _LOG.info("Worker=ProcessRunStatemachine run_yaml=%r", self._run_yaml_path)

        elif self._mode == self.MODE_OPTIMIZE:
            worker = ProcessOptimizeStatemachine(
                run_yaml_path=self._run_yaml_path or "",
                ui_bridge=self._bridge,
            )
            _LOG.info("Worker=ProcessOptimizeStatemachine run_yaml=%r", self._run_yaml_path)

        else:
            worker = ProcessValidateStatemachine(
                recipe=self._recipe,
                ui_bridge=self._bridge,
            )
            _LOG.info("Worker=ProcessValidateStatemachine recipe=%r", getattr(self._recipe, "id", None))

        worker.moveToThread(self._thread)

        worker.stateChanged.connect(self.stateChanged)           # type: ignore[attr-defined]
        worker.logMessage.connect(self.logMessage)               # type: ignore[attr-defined]
        worker.notifyFinished.connect(self._on_worker_finished)  # type: ignore[attr-defined]
        worker.notifyError.connect(self._on_worker_error)        # type: ignore[attr-defined]

        self._worker = worker
        return worker

    def _cleanup_worker(self) -> None:
        worker = self._worker
        if worker is None:
            return
        _LOG.info("ProcessThread: cleanup Worker (mode=%s).", self._mode)
        try:
            worker.deleteLater()
        except Exception:
            pass
        self._worker = None

    # ------------------------------------------------------------------ #
    # Slots start/stop
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_start_signal(self) -> None:
        if not self._thread.isRunning():
            _LOG.info("ProcessThread: starte QThread (mode=%s).", self._mode)
            self._thread.start()

        if self._worker is not None:
            _LOG.warning("ProcessThread: alter Worker war noch vorhanden, räume auf.")
            self._cleanup_worker()

        worker = self._create_worker()

        QtCore.QMetaObject.invokeMethod(
            worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    @QtCore.pyqtSlot()
    def _on_stop_signal(self) -> None:
        self.request_stop()

    # ------------------------------------------------------------------ #
    # Worker -> Wrapper
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, result_obj: object) -> None:
        self.notifyFinished.emit(result_obj)
        self._cleanup_worker()

    @QtCore.pyqtSlot(str)
    def _on_worker_error(self, msg: str) -> None:
        self.notifyError.emit(msg)
        self._cleanup_worker()

    # ------------------------------------------------------------------ #
    # Thread/App shutdown
    # ------------------------------------------------------------------ #

    @QtCore.pyqtSlot()
    def _on_thread_finished(self) -> None:
        _LOG.info("ProcessThread: QThread beendet.")
        try:
            self.finished.emit()
        except Exception:
            pass

    @QtCore.pyqtSlot()
    def _on_app_about_to_quit(self) -> None:
        _LOG.info("ProcessThread: aboutToQuit – stoppe Worker/Thread.")
        try:
            self.request_stop()
        except Exception:
            pass

        try:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread._on_app_about_to_quit: Fehler beim Thread-Shutdown.")

        self._cleanup_worker()

    def deleteLater(self) -> None:
        try:
            if self._thread.isRunning():
                try:
                    self.request_stop()
                except Exception:
                    pass
                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            _LOG.exception("ProcessThread.deleteLater: Fehler beim Thread-Shutdown.")

        self._cleanup_worker()

        try:
            self._thread.deleteLater()
        except Exception:
            pass

        super().deleteLater()
