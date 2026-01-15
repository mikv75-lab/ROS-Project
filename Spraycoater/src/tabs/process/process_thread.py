# -*- coding: utf-8 -*-
# File: tabs/process/process_thread.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, Tuple

from PyQt6 import QtCore
from PyQt6.QtCore import QCoreApplication

from model.recipe.recipe import Recipe
from model.recipe.recipe_run_result import RunResult
from plc.plc_client import PlcClientBase

from .validate_statemachine import ProcessValidateStatemachine
from .optimize_statemachine import ProcessOptimizeStatemachine
from .execute_statemachine import ProcessExecuteStatemachine

_LOG = logging.getLogger("tabs.process.thread")


def _ensure_dict(v: Any) -> Dict[str, Any]:
    return v if isinstance(v, dict) else {}


class ProcessThread(QtCore.QObject):
    """
    Führt Validate / Optimize / Execute in einem Worker-Thread aus.

    Ziel: "mehrfach Start" stabil, keine stale MoveItPy Results/Segmente/Traj-Caches.

    Fixes:
      - Boundary reset vor JEDEM Run (vor Worker.start()).
      - Boundary reset nach JEDEM Run (success + error).
      - Shutdown ist BLOCKING: request_stop() MUSS laufen, bevor QThread beendet wird.
      - Reset von self._started am Ende, damit ein "Re-Init" (erneuter Start) möglich ist.
      - CRITICAL: Segment-Kontext wird hart zurückgesetzt.
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
        max_retries: int = 100,
        skip_home: bool = False,
        urdf_xml: str = "",
        srdf_xml: str = "",
    ) -> None:
        super().__init__(parent)

        self._ros = ros
        self._plc = plc
        self._recipe = recipe
        self._mode = str(mode)

        self._side = str(side or "top")
        self._max_retries = int(max_retries)
        self._skip_home = bool(skip_home)

        self._urdf_xml = str(urdf_xml or "")
        self._srdf_xml = str(srdf_xml or "")

        self._thread: Optional[QtCore.QThread] = None
        self._worker: Optional[QtCore.QObject] = None

        self._started = False
        self._shutting_down = False

    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._started:
            _LOG.warning("ProcessThread: Start ignoriert, Thread läuft bereits.")
            return
        
        self._started = True
        self._shutting_down = False
        
        self._thread = QtCore.QThread()
        self._thread.started.connect(self._on_thread_started)
        self._thread.start()

    def request_stop(self) -> None:
        """Externer Stopp-Aufruf (nicht blockierend)."""
        try:
            fn = getattr(self._ros, "moveit_stop", None)
            if callable(fn):
                fn()
        except Exception:
            pass

        if self._worker is not None:
            try:
                QtCore.QMetaObject.invokeMethod(
                    self._worker,
                    "request_stop",
                    QtCore.Qt.ConnectionType.QueuedConnection,
                )
            except Exception:
                _LOG.exception("request_stop invoke failed")

    # ------------------------------------------------------------------
    # Run-boundary reset
    # ------------------------------------------------------------------

    def _call_first(self, names: Tuple[str, ...]) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                try:
                    fn()
                except Exception as e:
                    self.logMessage.emit(f"W: ros.{n}() failed: {e!r}")
                return True
        return False

    def _call_first_with(self, names: Tuple[str, ...], *args: Any, **kwargs: Any) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                try:
                    fn(*args, **kwargs)
                except Exception as e:
                    self.logMessage.emit(f"W: ros.{n}(*args) failed: {e!r}")
                return True
        return False

    def _reset_segment_context(self) -> None:
        if self._call_first(("moveit_clear_segment", "moveit_reset_segment", "moveit_set_segment_none")):
            return
        self._call_first_with(("moveit_set_segment",), "")
        try:
            self._call_first_with(("moveit_set_segment",), None)  # type: ignore
        except Exception:
            pass

    def _reset_run_boundary(self) -> None:
        """Löscht alle Zustände, die zwischen Läufen hängen bleiben könnten."""
        self._call_first(("moveit_stop", "stop_moveit", "moveit_cancel", "moveit_abort"))
        self._call_first(("moveit_clear_motion_result", "moveit_reset_motion_result", "moveit_clear_result"))
        self._call_first(
            (
                "moveit_clear_trajectory_cache",
                "moveit_clear_traj_cache",
                "moveit_traj_cache_clear",
                "moveit_clear_cache",
            )
        )
        self._reset_segment_context()
        QCoreApplication.processEvents()

    # ------------------------------------------------------------------
    # Payload validation
    # ------------------------------------------------------------------

    @staticmethod
    def _is_jtbysegment_yaml_v1(obj: Any) -> bool:
        if not isinstance(obj, dict): return False
        try:
            return int(obj.get("version", 0)) == 1 and isinstance(obj.get("segments"), dict)
        except Exception: return False

    @staticmethod
    def _normalize_payload(obj: Any) -> Dict[str, Any]:
        root = _ensure_dict(obj)
        planned = _ensure_dict(root.get("planned_run"))
        executed = _ensure_dict(root.get("executed_run"))
        rd = _ensure_dict(root.get("robot_description"))

        out = {
            "robot_description": {
                "urdf_xml": str(rd.get("urdf_xml") or root.get("urdf_xml") or ""),
                "srdf_xml": str(rd.get("srdf_xml") or root.get("srdf_xml") or ""),
            },
            "planned_run": {"traj": planned.get("traj"), "tcp": _ensure_dict(planned.get("tcp"))},
            "executed_run": {"traj": executed.get("traj"), "tcp": _ensure_dict(executed.get("tcp"))},
            "fk_meta": _ensure_dict(root.get("fk_meta")),
            "eval": _ensure_dict(root.get("eval")),
            "valid": bool(root.get("valid", False)),
            "invalid_reason": str(root.get("invalid_reason", "") or ""),
        }
        return out

    @classmethod
    def _validate_payload_strict(cls, payload: Dict[str, Any]) -> Optional[str]:
        if not isinstance(payload, dict): return "payload ist kein dict."
        pr = payload.get("planned_run")
        er = payload.get("executed_run")
        if not isinstance(pr, dict) or not isinstance(er, dict): return "planned/executed fehlen."
        if not cls._is_jtbysegment_yaml_v1(pr.get("traj")): return "planned_run invalid."
        if not cls._is_jtbysegment_yaml_v1(er.get("traj")): return "executed_run invalid."
        return None

    # ------------------------------------------------------------------
    # Thread logic
    # ------------------------------------------------------------------

    def _on_thread_started(self) -> None:
        if self._recipe is None:
            self._emit_error("Kein Recipe übergeben.")
            return

        # Reset vor dem Start
        try:
            self.logMessage.emit("ProcessThread: boundary reset (pre-run)...")
            self._reset_run_boundary()
        except Exception: pass

        rr = RunResult(urdf_xml=self._urdf_xml, srdf_xml=self._srdf_xml, valid=True)

        try:
            if self._mode == self.MODE_VALIDATE:
                self._worker = ProcessValidateStatemachine(
                    recipe=self._recipe, ros=self._ros, run_result=rr, 
                    side=self._side, max_retries=self._max_retries, skip_home=self._skip_home
                )
            elif self._mode == self.MODE_OPTIMIZE:
                self._worker = ProcessOptimizeStatemachine(
                    recipe=self._recipe, ros=self._ros, run_result=rr,
                    side=self._side, max_retries=self._max_retries, skip_home=self._skip_home
                )
            elif self._mode == self.MODE_EXECUTE:
                self._worker = ProcessExecuteStatemachine(
                    recipe=self._recipe, ros=self._ros, plc=self._plc, run_result=rr,
                    side=self._side, max_retries=self._max_retries, skip_home=self._skip_home
                )
            else:
                self._emit_error(f"Unbekannter Mode: {self._mode}")
                return
        except Exception as e:
            self._emit_error(f"Worker init failed: {e}")
            return

        # Signale verbinden
        self._worker.stateChanged.connect(self.stateChanged)
        self._worker.logMessage.connect(self.logMessage)
        self._worker.notifyFinished.connect(self._on_worker_finished)
        self._worker.notifyError.connect(self._emit_error)

        self._worker.moveToThread(self._thread)
        QtCore.QMetaObject.invokeMethod(self._worker, "start", QtCore.Qt.ConnectionType.QueuedConnection)

    @QtCore.pyqtSlot(object)
    def _on_worker_finished(self, obj: object) -> None:
        payload = self._normalize_payload(obj)
        err = self._validate_payload_strict(payload)
        if err:
            self._emit_error(f"Worker finished with invalid payload: {err}")
            return

        self.notifyFinished.emit(payload)
        self.logMessage.emit("ProcessThread: boundary reset (post-run)...")
        self._reset_run_boundary()
        self._shutdown()

    @QtCore.pyqtSlot(str)
    def _emit_error(self, msg: str) -> None:
        """
        Fehlerbehandlung: Meldet den Fehler und sorgt für einen sauberen Stopp.
        """
        self.logMessage.emit(f"=== Process ERROR: {msg} ===")
        self.notifyError.emit(str(msg))

        try:
            self.logMessage.emit("ProcessThread: boundary reset (post-error)...")
            self._reset_run_boundary()
        except Exception: pass

        self._shutdown()

    def _shutdown(self) -> None:
        """
        Stoppt den Thread und den Worker blockierend. 
        Setzt Flags zurück, damit ein Re-Init möglich ist.
        """
        if self._shutting_down:
            return
        self._shutting_down = True

        w = self._worker
        self._worker = None

        if w is not None:
            # Blockierender Stopp des Workers
            QtCore.QMetaObject.invokeMethod(
                w, "request_stop", QtCore.Qt.ConnectionType.BlockingQueuedConnection
            )
            w.deleteLater()

        if self._thread is not None:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(2000)
            self._thread.deleteLater()
            self._thread = None

        # WICHTIG: Flag zurücksetzen für den nächsten Start-Versuch
        self._started = False
        _LOG.info("ProcessThread: Shutdown abgeschlossen, bereit für Re-Init.")