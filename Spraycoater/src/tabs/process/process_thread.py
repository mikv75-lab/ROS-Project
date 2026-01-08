# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, Tuple

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from model.recipe.recipe import Recipe
from plc.plc_client import PlcClientBase  # nur für Execute

from .validate_statemachine import ProcessValidateStatemachine
from .optimize_statemachine import ProcessOptimizeStatemachine
from .execute_statemachine import ProcessExecuteStatemachine

_LOG = logging.getLogger("tabs.process.thread")


def _ensure_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


class ProcessThread(QtCore.QObject):
    """
    Führt Validate/Optimize/Execute in einem Worker-Thread aus.

    Ergebnis-Contract (STRICT, RunResult payload):
      Jede Statemachine (via BaseProcessStatemachine) emittiert IMMER:

        {
          "planned_run":  {"traj": <JTBySegment v1 yaml dict>, "tcp": <Draft v1 yaml dict or {}>},
          "executed_run": {"traj": <JTBySegment v1 yaml dict>, "tcp": <Draft v1 yaml dict or {}>},
          "fk_meta": {...}   # kann {} sein, wird typischerweise im ProcessTab nach FK gesetzt
        }

    Persistenz passiert NICHT hier.
    ProcessTab übernimmt:
      - FK (TrajFkBuilder) -> planned/executed tcp
      - Persistenz via Repo/Bundle (traj + tcp)
      - UI/Marker
    """

    MODE_VALIDATE = "validate"
    MODE_OPTIMIZE = "optimize"
    MODE_EXECUTE = "execute"

    startSignal = QtCore.pyqtSignal()
    stopSignal = QtCore.pyqtSignal()

    notifyFinished = QtCore.pyqtSignal(object)  # emits strict RunResult payload dict
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
        # optional run params
        side: str = "top",
        max_retries: int = 2,
        skip_home: bool = False,
    ) -> None:
        super().__init__(parent)

        self._ros = ros
        self._plc: PlcClientBase | None = plc
        self._recipe: Optional[Recipe] = recipe
        self._mode: str = str(mode)

        self._side = str(side or "top")
        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

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

    # ----------------- Contract validation -----------------

    @staticmethod
    def _normalize_runresult_payload(obj: Any) -> Dict[str, Any]:
        """
        Ensures hard keys exist; never raises.
        (Strict validation/decisions can be done in ProcessTab.)
        """
        root = _ensure_dict(obj)

        planned = _ensure_dict(root.get("planned_run"))
        executed = _ensure_dict(root.get("executed_run"))
        fk_meta = _ensure_dict(root.get("fk_meta"))

        planned.setdefault("traj", {})
        planned.setdefault("tcp", {})
        executed.setdefault("traj", {})
        executed.setdefault("tcp", {})

        planned["traj"] = _ensure_dict(planned.get("traj"))
        planned["tcp"] = _ensure_dict(planned.get("tcp"))
        executed["traj"] = _ensure_dict(executed.get("traj"))
        executed["tcp"] = _ensure_dict(executed.get("tcp"))

        return {"planned_run": planned, "executed_run": executed, "fk_meta": fk_meta}

    @staticmethod
    def _looks_like_runresult_payload(obj: Any) -> bool:
        if not isinstance(obj, dict):
            return False
        if "planned_run" not in obj or "executed_run" not in obj:
            return False
        pr = obj.get("planned_run")
        er = obj.get("executed_run")
        if not isinstance(pr, dict) or not isinstance(er, dict):
            return False
        # keys should exist; allow empty dict values
        return ("traj" in pr) and ("traj" in er) and ("tcp" in pr) and ("tcp" in er)

    # ----------------- Thread start -----------------

    def _on_thread_started(self) -> None:
        QCoreApplication.processEvents()

        if self._recipe is None:
            self._on_worker_error("Kein Recipe übergeben.")
            return

        try:
            if self._mode == self.MODE_VALIDATE:
                self._worker = ProcessValidateStatemachine(
                    recipe=self._recipe,
                    ros=self._ros,
                    side=self._side,
                    max_retries=self._max_retries,
                    skip_home=self._skip_home,
                )

            elif self._mode == self.MODE_OPTIMIZE:
                self._worker = ProcessOptimizeStatemachine(
                    recipe=self._recipe,
                    ros=self._ros,
                    side=self._side,
                    max_retries=self._max_retries,
                    skip_home=self._skip_home,
                )

            elif self._mode == self.MODE_EXECUTE:
                if self._plc is None:
                    self._on_worker_error("PLC fehlt (Execute benötigt PLC).")
                    return
                self._worker = ProcessExecuteStatemachine(
                    recipe=self._recipe,
                    ros=self._ros,
                    plc=self._plc,
                    side=self._side,
                    max_retries=self._max_retries,
                    skip_home=self._skip_home,
                )

            else:
                self._on_worker_error(f"Unbekannter Mode: {self._mode}")
                return

        except Exception as e:
            self._on_worker_error(f"Worker init failed: {e}")
            return

        # forward signals
        try:
            self._worker.stateChanged.connect(self.stateChanged)
        except Exception:
            pass
        try:
            self._worker.logMessage.connect(self.logMessage)
        except Exception:
            pass
        try:
            self._worker.notifyFinished.connect(self._on_worker_finished)
        except Exception:
            pass
        try:
            self._worker.notifyError.connect(self._on_worker_error)
        except Exception:
            pass

        # move to thread and start
        try:
            self._worker.moveToThread(self._thread)
        except Exception as e:
            self._on_worker_error(f"moveToThread failed: {e}")
            return

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
        """
        Worker returns RunResult payload dict (strict keys).
        We normalize to guarantee keys exist even if a worker misbehaves.
        """
        payload = self._normalize_runresult_payload(result_obj)

        # If it doesn't even resemble the contract, treat as error (strict boundary here).
        if not self._looks_like_runresult_payload(payload):
            self.notifyError.emit("Worker returned invalid RunResult payload (missing planned/executed traj/tcp keys).")
            self._cleanup_worker()
            self._quit_thread()
            return

        self.notifyFinished.emit(payload)
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
