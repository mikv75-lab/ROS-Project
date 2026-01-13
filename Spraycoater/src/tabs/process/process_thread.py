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
      - Worker wird erst nach blocking stop + thread.wait() gelöscht.
      - CRITICAL: Segment-Kontext wird hart zurückgesetzt (auch wenn ros nur moveit_set_segment hat).
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

        # IMPORTANT: thread without parent -> deterministic lifecycle
        self._thread = QtCore.QThread()
        self._worker: Optional[QtCore.QObject] = None

        self._started = False
        self._shutting_down = False

        self._thread.started.connect(self._on_thread_started)

    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._started:
            return
        self._started = True
        self._thread.start()

    def request_stop(self) -> None:
        """
        Best-effort stop: stop motion + ask worker to stop.
        Non-blocking external call; _shutdown() will do the blocking stop.
        """
        try:
            fn = getattr(self._ros, "moveit_stop", None)
            if callable(fn):
                fn()
        except Exception:
            pass

        if self._worker is None:
            return

        try:
            QtCore.QMetaObject.invokeMethod(
                self._worker,
                "request_stop",
                QtCore.Qt.ConnectionType.QueuedConnection,
            )
        except Exception:
            _LOG.exception("request_stop invoke failed")

    # ------------------------------------------------------------------
    # Run-boundary reset (best-effort but ordered)
    # ------------------------------------------------------------------

    def _call_first(self, names: Tuple[str, ...]) -> bool:
        """
        Call the first existing method on ros from the given names.
        Returns True if a method existed (even if it errored).
        """
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
        """
        Call the first existing method on ros from the given names, with args/kwargs.
        Returns True if a method existed (even if it errored).
        """
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
        HARD reset of segment context to avoid leaking seg=... into subsequent runs
        (notably RobotInit/Home-Execute).

        Supports both styles:
          - moveit_clear_segment()/moveit_reset_segment()/moveit_set_segment_none()
          - moveit_set_segment(seg: str)  -> we call with "" and best-effort None
        """
        if self._call_first(("moveit_clear_segment", "moveit_reset_segment", "moveit_set_segment_none")):
            return

        # Fallback: minimal API found in many facades
        self._call_first_with(("moveit_set_segment",), "")

        # Some implementations accept None -> best-effort
        try:
            self._call_first_with(("moveit_set_segment",), None)  # type: ignore[arg-type]
        except Exception:
            pass

    def _reset_run_boundary(self) -> None:
        """
        Clear stale MoveItPy bridge state that can leak across runs:
          - stop motion
          - clear last motion_result
          - clear planned/executed caches
          - clear segment context (named seg=... leaks)
        """
        # 1) stop any motion
        self._call_first(("moveit_stop", "stop_moveit", "moveit_cancel", "moveit_abort"))

        # 2) clear last motion result (so no stale OK/ERROR leaks into next run)
        self._call_first(("moveit_clear_motion_result", "moveit_reset_motion_result", "moveit_clear_result"))

        # 3) clear traj caches (planned/executed)
        self._call_first(
            (
                "moveit_clear_trajectory_cache",
                "moveit_clear_traj_cache",
                "moveit_traj_cache_clear",
                "moveit_clear_cache",
            )
        )

        # 4) CRITICAL: clear segment context (also handles moveit_set_segment-only facades)
        self._reset_segment_context()

        QCoreApplication.processEvents()

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
        root = _ensure_dict(obj)

        planned = _ensure_dict(root.get("planned_run"))
        executed = _ensure_dict(root.get("executed_run"))

        planned_traj = planned.get("traj", None)
        executed_traj = executed.get("traj", None)

        planned_tcp = _ensure_dict(planned.get("tcp", {}))
        executed_tcp = _ensure_dict(executed.get("tcp", {}))

        rd = root.get("robot_description")
        rd = rd if isinstance(rd, dict) else {}

        urdf_xml = str((rd.get("urdf_xml") or root.get("urdf_xml") or "") or "")
        srdf_xml = str((rd.get("srdf_xml") or root.get("srdf_xml") or "") or "")

        out = {
            "robot_description": {"urdf_xml": urdf_xml, "srdf_xml": srdf_xml},
            "planned_run": {"traj": planned_traj, "tcp": planned_tcp},
            "executed_run": {"traj": executed_traj, "tcp": executed_tcp},
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

        rd = payload.get("robot_description")
        if not isinstance(rd, dict):
            return "robot_description fehlt oder ist ungültig."
        if not isinstance(rd.get("urdf_xml", ""), str):
            return "robot_description.urdf_xml ist kein str."
        if not isinstance(rd.get("srdf_xml", ""), str):
            return "robot_description.srdf_xml ist kein str."

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

        # HARD boundary reset BEFORE worker starts
        try:
            self.logMessage.emit("ProcessThread: boundary reset (pre-run, stop/clear caches)...")
            self._reset_run_boundary()
        except Exception as e:
            try:
                self.logMessage.emit(f"W: boundary reset failed: {e!r}")
            except Exception:
                pass

        rr = RunResult(
            urdf_xml=self._urdf_xml,
            srdf_xml=self._srdf_xml,
            valid=True,
            invalid_reason="",
        )

        if rr.urdf_xml.strip() and rr.srdf_xml.strip():
            self.logMessage.emit("ProcessThread: URDF/SRDF injected into RunResult (non-empty).")
        else:
            self.logMessage.emit("ProcessThread: URDF/SRDF injection is empty (FK/Eval will be skipped later).")

        try:
            if self._mode == self.MODE_VALIDATE:
                self._worker = ProcessValidateStatemachine(
                    recipe=self._recipe,
                    ros=self._ros,
                    side=self._side,
                    max_retries=self._max_retries,
                    skip_home=self._skip_home,
                    run_result=rr,
                )
            elif self._mode == self.MODE_OPTIMIZE:
                self._worker = ProcessOptimizeStatemachine(
                    recipe=self._recipe,
                    ros=self._ros,
                    side=self._side,
                    max_retries=self._max_retries,
                    skip_home=self._skip_home,
                    run_result=rr,
                )
            elif self._mode == self.MODE_EXECUTE:
                self._worker = ProcessExecuteStatemachine(
                    recipe=self._recipe,
                    ros=self._ros,
                    plc=self._plc,
                    side=self._side,
                    max_retries=self._max_retries,
                    skip_home=self._skip_home,
                    run_result=rr,
                )
            else:
                self._emit_error(f"Unbekannter Mode: {self._mode}")
                return
        except Exception as e:
            self._emit_error(f"Worker init failed: {e}")
            return

        # wire signals (best-effort)
        for sig in ("stateChanged", "logMessage"):
            try:
                getattr(self._worker, sig).connect(getattr(self, sig))
            except Exception:
                pass

        self._worker.notifyFinished.connect(self._on_worker_finished)
        self._worker.notifyError.connect(self._emit_error)

        # move worker to thread and start
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

        # re-impose injected XML if missing
        try:
            rd = payload.get("robot_description")
            rd = rd if isinstance(rd, dict) else {}
            if not str(rd.get("urdf_xml") or "").strip() and self._urdf_xml.strip():
                rd["urdf_xml"] = self._urdf_xml
            if not str(rd.get("srdf_xml") or "").strip() and self._srdf_xml.strip():
                rd["srdf_xml"] = self._srdf_xml
            payload["robot_description"] = rd
        except Exception:
            pass

        err = self._validate_payload_strict(payload)
        if err:
            self._emit_error(f"Worker finished with invalid payload (strict): {err}")
            return

        self.notifyFinished.emit(payload)

        # Boundary reset AFTER run (important if MoveItPy keeps last msgs/caches)
        try:
            self.logMessage.emit("ProcessThread: boundary reset (post-run)...")
            self._reset_run_boundary()
        except Exception:
            pass

        self._shutdown()

    @QtCore.pyqtSlot(str)
    def _emit_error(self, msg: str) -> None:
        self.notifyError.emit(str(msg))

        # Boundary reset AFTER error (prevents stale seg/result leaking into RobotInit)
        try:
            self.logMessage.emit("ProcessThread: boundary reset (post-error)...")
            self._reset_run_boundary()
        except Exception:
            pass

        self._shutdown()

    # ------------------------------------------------------------------
    # Deterministic shutdown (BLOCKING)
    # ------------------------------------------------------------------

    def _shutdown(self) -> None:
        if self._shutting_down:
            return
        self._shutting_down = True

        w = self._worker
        self._worker = None

        # 1) BLOCKING stop on worker thread (ensures stop logic runs before thread quits)
        if w is not None:
            try:
                QtCore.QMetaObject.invokeMethod(
                    w,
                    "request_stop",
                    QtCore.Qt.ConnectionType.BlockingQueuedConnection,
                )
            except Exception:
                # if request_stop isn't a slot or thread is already stopping, ignore
                pass

            try:
                w.notifyFinished.disconnect()
            except Exception:
                pass
            try:
                w.notifyError.disconnect()
            except Exception:
                pass

            try:
                w.deleteLater()
            except Exception:
                pass

        # 2) stop thread
        try:
            if self._thread.isRunning():
                self._thread.quit()
                self._thread.wait(5000)
        except Exception:
            pass

        # 3) delete thread object (now safe)
        try:
            self._thread.deleteLater()
        except Exception:
            pass