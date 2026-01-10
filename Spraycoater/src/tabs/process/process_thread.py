# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict

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
    Führt Validate / Optimize / Execute in einem Worker-Thread aus.

    STRICT Ergebnis-Contract (ein Format, keine Legacy):

      notifyFinished(payload) wird NUR emittiert, wenn payload ein dict ist mit:

        payload = {
          "planned_run":  {"traj": <JTBySegment YAML v1 dict>, "tcp": <Draft YAML dict oder {}>},
          "executed_run": {"traj": <JTBySegment YAML v1 dict>, "tcp": <Draft YAML dict oder {}>},
          "fk_meta": {...},              # dict
          "eval": {...},                 # dict
          "valid": true/false,           # bool
          "invalid_reason": "..."        # str
        }

    Alles andere ist ein Fehler.
    """

    MODE_VALIDATE = "validate"
    MODE_OPTIMIZE = "optimize"
    MODE_EXECUTE = "execute"

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
        side: str = "top",
        max_retries: int = 2,
        skip_home: bool = False,
    ) -> None:
        super().__init__(parent)

        self._ros = ros
        self._plc = plc
        self._recipe = recipe
        self._mode = str(mode)

        self._side = str(side or "top")
        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._thread = QtCore.QThread(self)
        self._worker: Optional[QtCore.QObject] = None

        self._thread.started.connect(self._on_thread_started)

    # ------------------------------------------------------------------

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
            _LOG.exception("request_stop failed")

    # ------------------------------------------------------------------
    # Payload validation (STRICT)
    # ------------------------------------------------------------------

    @staticmethod
    def _is_jtbysegment_yaml_v1(obj: Any) -> bool:
        if not isinstance(obj, dict):
            return False
        try:
            ver = int(obj.get("version", 0))
        except Exception:
            return False
        if ver != 1:
            return False
        segs = obj.get("segments", None)
        return isinstance(segs, dict) and bool(segs)

    @staticmethod
    def _normalize_payload(obj: Any) -> Dict[str, Any]:
        """
        Normalize only types/shapes. Does NOT do schema conversion.
        Missing keys stay missing and will fail validation.
        """
        root = _ensure_dict(obj)

        planned = _ensure_dict(root.get("planned_run"))
        executed = _ensure_dict(root.get("executed_run"))

        # keep exactly these keys; do not invent trajectories
        planned_traj = planned.get("traj", None)
        executed_traj = executed.get("traj", None)

        planned_tcp = planned.get("tcp", {})
        executed_tcp = executed.get("tcp", {})

        out = {
            "planned_run": {"traj": planned_traj, "tcp": _ensure_dict(planned_tcp)},
            "executed_run": {"traj": executed_traj, "tcp": _ensure_dict(executed_tcp)},
            "fk_meta": _ensure_dict(root.get("fk_meta")),
            "eval": _ensure_dict(root.get("eval")),
            "valid": bool(root.get("valid", False)),
            "invalid_reason": str(root.get("invalid_reason", "") or ""),
        }
        return out

    @classmethod
    def _validate_payload_strict(cls, payload: Dict[str, Any]) -> Optional[str]:
        if not isinstance(payload, dict):
            return "payload ist kein dict."

        pr = payload.get("planned_run")
        er = payload.get("executed_run")
        if not isinstance(pr, dict) or not isinstance(er, dict):
            return "planned_run/executed_run fehlen oder sind ungültig."

        if "traj" not in pr or "traj" not in er:
            return "planned_run['traj'] oder executed_run['traj'] fehlt."

        if not cls._is_jtbysegment_yaml_v1(pr.get("traj")):
            return "planned_run['traj'] ist nicht JTBySegment YAML v1 (segments/version)."
        if not cls._is_jtbysegment_yaml_v1(er.get("traj")):
            return "executed_run['traj'] ist nicht JTBySegment YAML v1 (segments/version)."

        if not isinstance(payload.get("fk_meta"), dict):
            return "fk_meta ist kein dict."
        if not isinstance(payload.get("eval"), dict):
            return "eval ist kein dict."
        if not isinstance(payload.get("valid"), bool):
            return "valid ist kein bool."
        if not isinstance(payload.get("invalid_reason"), str):
            return "invalid_reason ist kein str."

        return None

    # ------------------------------------------------------------------
    # Thread start
    # ------------------------------------------------------------------

    def _on_thread_started(self) -> None:
        QCoreApplication.processEvents()

        if self._recipe is None:
            self._emit_error("Kein Recipe übergeben.")
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
                    self._emit_error("PLC fehlt (Execute benötigt PLC).")
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
                self._emit_error(f"Unbekannter Mode: {self._mode}")
                return

        except Exception as e:
            self._emit_error(f"Worker init failed: {e}")
            return

        # wire signals
        for sig in ("stateChanged", "logMessage"):
            try:
                getattr(self._worker, sig).connect(getattr(self, sig))
            except Exception:
                pass

        self._worker.notifyFinished.connect(self._on_worker_finished)
        self._worker.notifyError.connect(self._emit_error)

        self._worker.moveToThread(self._thread)

        QtCore.QMetaObject.invokeMethod(
            self._worker,
            "start",
            QtCore.Qt.ConnectionType.QueuedConnection,
        )

    # ------------------------------------------------------------------
    # Worker callbacks
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, obj: object) -> None:
        payload = self._normalize_payload(obj)

        err = self._validate_payload_strict(payload)
        if err:
            self._emit_error(f"Worker finished with invalid payload (strict): {err}")
            return

        self.notifyFinished.emit(payload)
        self._shutdown()

    @QtCore.pyqtSlot(str)
    def _emit_error(self, msg: str) -> None:
        self.notifyError.emit(str(msg))
        self._shutdown()

    # ------------------------------------------------------------------

    def _shutdown(self) -> None:
        if self._worker is not None:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self._worker,
                    "request_stop",
                    QtCore.Qt.ConnectionType.QueuedConnection,
                )
            except Exception:
                pass
            self._worker.deleteLater()
            self._worker = None

        if self._thread.isRunning():
            self._thread.quit()
            self._thread.wait(5000)
