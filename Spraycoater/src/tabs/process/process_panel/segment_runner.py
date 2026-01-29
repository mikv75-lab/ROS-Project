# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/segmentrunner.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional, Callable

from PyQt6 import QtCore

from .base_statemachine import (
    ReqKey,
    _parse_motion_result,
    _parse_key_from_traj_header,
    _is_empty_robot_trajectory,
)

_LOG = logging.getLogger("tabs.process.segmentrunner")


# =============================================================================
# Step specification
# =============================================================================

@dataclass(frozen=True)
class StepSpec:
    """
    One atomic MoveIt action that must complete before the next step starts.

    - send(): invoked once when step starts
    - expect_result_key: ReqKey for motion_result
    - expect_status: exact match (STRICT)
    - expect_traj_kind: one of: None | "planned" | "executed" | "optimized"
    - expect_traj_key: ReqKey for trajectory header.frame_id
    """
    label: str
    send: Callable[[], None]

    expect_result_key: ReqKey
    expect_status: str  # "PLANNED:OK" / "EXECUTED:OK" / "OPTIMIZED:OK"

    expect_traj_kind: Optional[str] = None
    expect_traj_key: Optional[ReqKey] = None


# =============================================================================
# Convenience builders (INTENTIONALLY UNBOUND)
# =============================================================================
# These exist only to keep old imports compiling; they must never be used.
# Use SegmentRunner.make_*_step_bound(...) instead (needs ros instance).
# =============================================================================

def make_plan_pose_step(*, run: str, req_id: int, seg: str, pose: Any, label: str = "plan_pose") -> StepSpec:
    raise RuntimeError("Use SegmentRunner.make_plan_pose_step_bound(...) instead (needs ros).")


def make_execute_last_planned_step(*, run: str, req_id: int, seg: str, label: str = "execute") -> StepSpec:
    raise RuntimeError("Use SegmentRunner.make_execute_last_planned_step_bound(...) instead (needs ros).")


def make_execute_trajectory_step(*, run: str, req_id: int, seg: str, traj: Any, label: str = "execute_trajectory") -> StepSpec:
    raise RuntimeError("Use SegmentRunner.make_execute_trajectory_step_bound(...) instead (needs ros).")


def make_optimize_trajectory_step(*, run: str, req_id: int, seg: str, traj: Any, label: str = "optimize_trajectory") -> StepSpec:
    raise RuntimeError("Use SegmentRunner.make_optimize_trajectory_step_bound(...) instead (needs ros).")


# =============================================================================
# Step runner (wait for result + traj) - no timeouts
# =============================================================================

class StepRunner(QtCore.QObject):
    finished = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_goal_rejected_retries: int = 100,
        goal_rejected_retry_delay_ms: int = 0,
        max_exec_error_retries: int = 1000,
        exec_error_retry_delay_ms: int = 100,
    ) -> None:
        super().__init__(parent)
        self._ros = ros

        self._moveitpy_signals = None
        try:
            mp = self._ros.moveitpy
            self._moveitpy_signals = mp.signals if mp is not None else None
        except Exception:
            self._moveitpy_signals = None

        self._active: bool = False
        self._stop: bool = False

        self._spec: Optional[StepSpec] = None
        self._got_result: bool = False
        self._got_traj: bool = False

        # Retry policy (STRICT): only for ERROR:GOAL_REJECTED.
        self._max_goal_rejected_retries: int = int(max_goal_rejected_retries)
        self._goal_rejected_retry_delay_ms: int = int(goal_rejected_retry_delay_ms)
        self._goal_rejected_attempts: int = 0

        self._max_exec_error_retries: int = int(max_exec_error_retries)
        self._exec_error_retry_delay_ms: int = int(exec_error_retry_delay_ms)
        self._exec_error_attempts: int = 0

        self._connect()

    def _connect(self) -> None:
        sig = self._moveitpy_signals
        if sig is None:
            return
        try:
            sig.motionResultChanged.connect(self._on_motion_result)
        except Exception:
            pass
        try:
            sig.plannedTrajectoryChanged.connect(self._on_planned_traj)
        except Exception:
            pass
        try:
            sig.executedTrajectoryChanged.connect(self._on_executed_traj)
        except Exception:
            pass
        try:
            sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj)
        except Exception:
            pass

    def _emit_log(self, msg: str) -> None:
        """Emit a UI log line.

        Order of delivery:
          1) SegmentRunner.logLine (local subscribers)
          2) parent.logMessage (if present; BaseProcessStatemachine)
        """
        text = str(msg or "").strip()
        if not text:
            return
        try:
            self.logLine.emit(text)
        except Exception:
            pass
        try:
            p = self.parent()
            if p is not None and hasattr(p, "logMessage"):
                # BaseProcessStatemachine.logMessage: pyqtSignal(str)
                getattr(p, "logMessage").emit(text)
        except Exception:
            pass


    def _ui_log(self, msg: str) -> None:
        """Forward a log line to parent SegmentRunner (and further to UI)."""
        text = str(msg or "").strip()
        if not text:
            return
        try:
            p = self.parent()
            if p is not None and hasattr(p, "_emit_log"):
                p._emit_log(text)  # type: ignore[attr-defined]
                return
            if p is not None and hasattr(p, "logLine"):
                getattr(p, "logLine").emit(text)
        except Exception:
            pass

    def request_stop(self) -> None:
        self._stop = True

    def start(self, spec: StepSpec) -> None:
        if self._active:
            self.error.emit("StepRunner: already active")
            return
        self._active = True
        self._stop = False
        self._spec = spec
        self._got_result = False
        self._got_traj = (spec.expect_traj_kind is None)
        self._goal_rejected_attempts = 0  # reset per step  # if no traj expected, already satisfied

        self._ui_log(f"{spec.label}: send")

        try:
            spec.send()
        except Exception as e:
            self._active = False
            self.error.emit(f"{spec.label}: send failed: {e}")
            return

        self._maybe_finish()

    @staticmethod
    def _match_key(a: ReqKey, b: ReqKey) -> bool:
        return a.as_tuple() == b.as_tuple()

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        if not self._active or self._stop or self._spec is None:
            return
        try:
            key, status, d = _parse_motion_result(str(text or ""))
        except Exception:
            return

        spec = self._spec
        if not self._match_key(key, spec.expect_result_key):
            return

        up = str(status or "").strip()
        if up.upper().startswith("ERROR"):
            msg = ""
            try:
                msg = str(d.get("msg") or d.get("error") or up)
            except Exception:
                msg = up

            # Retry only for GOAL_REJECTED (common transient when controller/action server is not ready).
            up_u = up.upper()
            msg_u = str(msg).upper()
            is_goal_rejected = up_u.startswith("ERROR:GOAL_REJECTED") or ("GOAL_REJECTED" in msg_u)
            if is_goal_rejected and (self._goal_rejected_attempts < self._max_goal_rejected_retries):
                self._goal_rejected_attempts += 1
                self._ui_log(f"{spec.label}: GOAL_REJECTED retry {self._goal_rejected_attempts}/{self._max_goal_rejected_retries}")
                _LOG.warning(
                    "StepRunner: %s -> GOAL_REJECTED retry %d/%d (key=%s)",
                    spec.label,
                    self._goal_rejected_attempts,
                    self._max_goal_rejected_retries,
                    spec.expect_result_key,
                )

                # Reset completion flags and resend the SAME request (same key) after optional delay.
                self._got_result = False
                self._got_traj = (spec.expect_traj_kind is None)

                def _resend() -> None:
                    if not self._active or self._stop or self._spec is None:
                        return
                    try:
                        spec.send()
                    except Exception as e:
                        self._active = False
                        self.error.emit(f"{spec.label}: resend failed: {e}")

                QtCore.QTimer.singleShot(max(0, int(self._goal_rejected_retry_delay_ms)), _resend)
                return

            # Retry only for ERROR:EXEC (often transient during controller switching).
            is_exec_error = up_u.startswith("ERROR:EXEC") or ("ERROR:EXEC" in msg_u)
            if is_exec_error and (self._exec_error_attempts < self._max_exec_error_retries):
                self._exec_error_attempts += 1
                self._ui_log(f"{spec.label}: EXEC retry {self._exec_error_attempts}/{self._max_exec_error_retries} ({msg})")
                _LOG.warning(
                    "StepRunner: %s -> EXEC error retry %d/%d (key=%s, msg=%s)",
                    spec.label,
                    self._exec_error_attempts,
                    self._max_exec_error_retries,
                    spec.expect_result_key,
                    msg,
                )

                self._got_result = False
                self._got_traj = (spec.expect_traj_kind is None)

                def _resend_exec() -> None:
                    if not self._active or self._stop or self._spec is None:
                        return
                    try:
                        spec.send()
                    except Exception as e:
                        self._active = False
                        self.error.emit(f"{spec.label}: resend failed: {e}")

                QtCore.QTimer.singleShot(max(0, int(self._exec_error_retry_delay_ms)), _resend_exec)
                return

            self._ui_log(f"{spec.label}: ERROR final: {msg}")
            self._active = False
            self.error.emit(f"{spec.label}: {msg}")
            return

        if up != str(spec.expect_status):
            # strict: ignore other statuses for same key
            return

        self._ui_log(f"{spec.label}: result OK ({spec.expect_status})")
        self._got_result = True
        self._maybe_finish()

    def _consume_traj(self, *, kind: str, obj: Any) -> None:
        if not self._active or self._stop or self._spec is None:
            return
        spec = self._spec
        if spec.expect_traj_kind is None:
            return
        if str(kind) != str(spec.expect_traj_kind):
            return
        if obj is None or _is_empty_robot_trajectory(obj):
            return
        try:
            key = _parse_key_from_traj_header(obj)
        except Exception:
            return
        if spec.expect_traj_key is None:
            return
        if not self._match_key(key, spec.expect_traj_key):
            return

        self._ui_log(f"{spec.label}: traj OK ({kind})")
        self._got_traj = True
        self._maybe_finish()

    @QtCore.pyqtSlot(object)
    def _on_planned_traj(self, obj: object) -> None:
        self._consume_traj(kind="planned", obj=obj)

    @QtCore.pyqtSlot(object)
    def _on_executed_traj(self, obj: object) -> None:
        self._consume_traj(kind="executed", obj=obj)

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj(self, obj: object) -> None:
        self._consume_traj(kind="optimized", obj=obj)

    def _maybe_finish(self) -> None:
        if not self._active or self._stop:
            return
        if self._got_result and self._got_traj:
            self._ui_log(f"{self._spec.label}: DONE")
            self._active = False
            self._emit_log("segment: DONE")
            QtCore.QTimer.singleShot(50, self.finished.emit)


# =============================================================================
# SegmentRunner (sequential chaining)
# =============================================================================

class SegmentRunner(QtCore.QObject):
    finished = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(str)
    logLine = QtCore.pyqtSignal(str)  # UI-friendly progress/trace lines

    def __init__(
        self,
        *,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_goal_rejected_retries: int = 100,
        goal_rejected_retry_delay_ms: int = 0,
    ) -> None:
        super().__init__(parent)
        self._ros = ros

        self._steps: list[StepSpec] = []
        self._idx: int = 0
        self._stop: bool = False

        self._step_runner = StepRunner(
            ros=self._ros,
            parent=self,
            max_goal_rejected_retries=int(max_goal_rejected_retries),
            goal_rejected_retry_delay_ms=int(goal_rejected_retry_delay_ms),
        )
        self._step_runner.finished.connect(self._on_step_finished)
        self._step_runner.error.connect(self._on_step_error)

        # Optional: forward runner log lines into parent Statemachine logMessage.
        # Parent is typically BaseProcessStatemachine (has .logMessage signal).


    def _emit_log(self, text: str) -> None:
        """Emit a UI-friendly log line and forward into parent Statemachine if available."""
        msg = str(text or "")
        try:
            self.logLine.emit(msg)
        except Exception:
            pass

        p = self.parent()
        if p is None:
            return
        # Forward to BaseProcessStatemachine.logMessage if present.
        try:
            sig = getattr(p, "logMessage", None)
            if sig is not None:
                sig.emit(msg)  # type: ignore[attr-defined]
        except Exception:
            pass


    def request_stop(self) -> None:
        self._stop = True
        try:
            self._step_runner.request_stop()
        except Exception:
            pass
        self._steps = []
        self._idx = 0

    # ----- bound step builders (need ros) -----

    def make_plan_pose_step_bound(self, *, run: str, req_id: int, seg: str, pose: Any, label: str) -> StepSpec:
        key = ReqKey(run=str(run), id=int(req_id), seg=str(seg), op="plan_pose")

        def _send() -> None:
            self._ros.moveit_plan_pose(pose, run=key.run, req_id=key.id, segment=key.seg)

        return StepSpec(
            label=str(label),
            send=_send,
            expect_result_key=key,
            expect_status="PLANNED:OK",
            expect_traj_kind="planned",
            expect_traj_key=key,
        )

    def make_execute_last_planned_step_bound(self, *, run: str, req_id: int, seg: str, label: str) -> StepSpec:
        key = ReqKey(run=str(run), id=int(req_id), seg=str(seg), op="execute")

        def _send() -> None:
            self._ros.moveit_execute_last_planned(run=key.run, req_id=key.id, segment=key.seg)

        return StepSpec(
            label=str(label),
            send=_send,
            expect_result_key=key,
            expect_status="EXECUTED:OK",
            expect_traj_kind="executed",
            expect_traj_key=key,
        )

    def make_execute_trajectory_step_bound(self, *, run: str, req_id: int, seg: str, traj: Any, label: str) -> StepSpec:
        # NOTE: RosBridge.moveit_execute_trajectory(...) may NOT accept (run, req_id).
        # The keyed contract for execute_trajectory is carried in traj.joint_trajectory.header.frame_id.
        # Therefore we derive the expected key from the trajectory itself.
        key = _parse_key_from_traj_header(traj)

        # Be tolerant: caller still passes seg/run/req_id (legacy call sites), but we do not rely on them.
        def _send() -> None:
            # Most bridges accept: (traj, segment=...)
            self._ros.moveit_execute_trajectory(traj, segment=key.seg)

        return StepSpec(
            label=str(label),
            send=_send,
            expect_result_key=key,
            expect_status="EXECUTED:OK",
            expect_traj_kind="executed",
            expect_traj_key=key,
        )

    def make_optimize_trajectory_step_bound(self, *, run: str, req_id: int, seg: str, traj: Any, label: str) -> StepSpec:
        # NOTE: RosBridge.moveit_optimize_trajectory(...) may NOT accept (run, req_id).
        # The keyed contract for optimize_trajectory is carried in traj.joint_trajectory.header.frame_id.
        key = _parse_key_from_traj_header(traj)

        def _send() -> None:
            # Most bridges accept: (traj, segment=...)
            self._ros.moveit_optimize_trajectory(traj, segment=key.seg)

        return StepSpec(
            label=str(label),
            send=_send,
            expect_result_key=key,
            expect_status="OPTIMIZED:OK",
            expect_traj_kind="optimized",
            expect_traj_key=key,
        )


    # ----- execution -----

    def run(self, steps: list[StepSpec]) -> None:
        self._stop = False
        self._steps = list(steps or [])
        self._idx = 0

        if not self._steps:
            QtCore.QTimer.singleShot(0, self.finished.emit)
            return

        self._start_current()

    def _start_current(self) -> None:
        if self._stop:
            return
        if self._idx >= len(self._steps):
            QtCore.QTimer.singleShot(0, self.finished.emit)
            return

        spec = self._steps[self._idx]
        _LOG.info("SegmentRunner: step %d/%d: %s", self._idx + 1, len(self._steps), spec.label)
        self._emit_log(f"step {self._idx+1}/{len(self._steps)}: {spec.label}")
        self._step_runner.start(spec)

    def _on_step_finished(self) -> None:
        if self._stop:
            return
        self._idx += 1
        self._start_current()

    def _on_step_error(self, msg: str) -> None:
        if self._stop:
            return
        self._emit_log(f"ERROR: {msg}")
        self.error.emit(str(msg or "Step ERROR"))
