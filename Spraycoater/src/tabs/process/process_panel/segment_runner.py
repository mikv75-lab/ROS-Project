# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/segment_runner.py
from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional, Callable, List, Dict

from PyQt6 import QtCore

from .process_contract import (
    ReqKey,
    parse_motion_result,
    parse_key_from_traj_header,
)
from .process_serialization import is_empty_robot_trajectory

_LOG = logging.getLogger("tabs.process.segment_runner")


# =============================================================================
# Step specification
# =============================================================================

@dataclass(frozen=True)
class StepSpec:
    """
    One atomic MoveIt action that must complete before the next step starts.
    """
    label: str
    send: Callable[[], None]

    expect_result_key: ReqKey
    expect_status: str  # "PLANNED:OK" / "EXECUTED:OK" / "OPTIMIZED:OK"

    expect_traj_kind: Optional[str] = None
    expect_traj_key: Optional[ReqKey] = None


# =============================================================================
# Step runner (wait for result + traj) - with Stability Check
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
        ready_poll_delay_ms: int = 50,
    ) -> None:
        super().__init__(parent)
        self._ros = ros

        # Retry Configuration
        self._retry_config = {
            "GOAL_REJECTED": {
                "max": int(max_goal_rejected_retries),
                "delay": int(goal_rejected_retry_delay_ms),
                "count": 0,
            },
            # EXEC_ERROR = generic error retry
            "EXEC_ERROR": {
                "max": int(max_exec_error_retries),
                "delay": int(exec_error_retry_delay_ms),
                "count": 0,
            },
        }

        self._ready_poll_delay_ms = int(ready_poll_delay_ms)
        self._gate_last_state = ""

        # Stability (Hysteresis)
        self._stability_counter = 0
        self._required_stability_cycles = 4  # 4 * 50ms = 200ms stable "ok"

        self._active: bool = False
        self._stop: bool = False
        self._spec: Optional[StepSpec] = None
        self._got_result: bool = False
        self._got_traj: bool = False

        self._connect_signals()

    # ---------------------------------------------------------------------
    # Wiring
    # ---------------------------------------------------------------------

    def _connect_signals(self) -> None:
        """
        Dynamic connection of MoveItPy signals.
        Expected: ros.moveitpy.signals.{motionResultChanged, plannedTrajectoryChanged, ...}
        """
        mp = getattr(self._ros, "moveitpy", None)
        signals = getattr(mp, "signals", None) if mp else None
        if not signals:
            return

        mapping = [
            ("motionResultChanged", self._on_motion_result),
            ("plannedTrajectoryChanged", self._on_planned_traj),
            ("executedTrajectoryChanged", self._on_executed_traj),
            ("optimizedTrajectoryChanged", self._on_optimized_traj),
        ]

        for sig_name, slot in mapping:
            try:
                sig = getattr(signals, sig_name, None)
                if sig is not None:
                    sig.connect(slot)
            except Exception:
                pass

    # ---------------------------------------------------------------------
    # UI logging helper (forward to SegmentRunner parent)
    # ---------------------------------------------------------------------

    def _ui_log(self, msg: str) -> None:
        text = str(msg or "").strip()
        if not text:
            return
        try:
            p = self.parent()
            if p and hasattr(p, "_emit_log"):
                p._emit_log(text)  # type: ignore[attr-defined]
            elif p and hasattr(p, "logLine"):
                getattr(p, "logLine").emit(text)
        except Exception:
            pass

    # ---------------------------------------------------------------------
    # Gating logic with stability check
    # ---------------------------------------------------------------------

    def _try_send_current(self) -> None:
        if not self._active or self._stop or self._spec is None:
            return

        # 1) Instant check
        is_busy = False
        is_ready = True
        try:
            if callable(getattr(self._ros, "moveit_is_busy", None)):
                is_busy = bool(self._ros.moveit_is_busy())
            if callable(getattr(self._ros, "moveit_tray_exec_ready", None)):
                is_ready = bool(self._ros.moveit_tray_exec_ready())
        except Exception:
            pass

        state = "busy" if is_busy else ("not_ready" if not is_ready else "ok")

        # 2) Status evaluation
        if state != "ok":
            self._stability_counter = 0

            if state != self._gate_last_state:
                self._ui_log(f"{self._spec.label}: wait ({state})")

            self._gate_last_state = state
            QtCore.QTimer.singleShot(self._ready_poll_delay_ms, self._try_send_current)
            return

        # 3) Status ok -> require stability
        self._stability_counter += 1
        if self._stability_counter < self._required_stability_cycles:
            if self._gate_last_state != "stabilizing":
                self._ui_log(f"{self._spec.label}: wait (stabilizing)")
            self._gate_last_state = "stabilizing"
            QtCore.QTimer.singleShot(self._ready_poll_delay_ms, self._try_send_current)
            return

        # 4) Stable ok -> send
        self._gate_last_state = "ok"
        try:
            self._ui_log(f"{self._spec.label}: send")
            self._spec.send()
        except Exception as e:
            self._fail(f"send failed: {e!r}")
            return

        self._maybe_finish()

    # ---------------------------------------------------------------------
    # Public API
    # ---------------------------------------------------------------------

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

        self._retry_config["GOAL_REJECTED"]["count"] = 0
        self._retry_config["EXEC_ERROR"]["count"] = 0

        self._gate_last_state = ""
        self._stability_counter = 0

        QtCore.QTimer.singleShot(self._ready_poll_delay_ms, self._try_send_current)

    # ---------------------------------------------------------------------
    # Retry logic
    # ---------------------------------------------------------------------

    def _attempt_retry(self, retry_key: str, error_msg: str) -> bool:
        """
        Generic retry handler.
        Returns True if retry was initiated.
        """
        cfg = self._retry_config.get(retry_key)
        if not cfg:
            return False

        if cfg["count"] >= cfg["max"]:
            return False

        cfg["count"] += 1

        if cfg["count"] % 5 == 0 or cfg["count"] == 1:
            _LOG.warning("StepRunner: %s retry %d/%d", retry_key, cfg["count"], cfg["max"])
            if self._spec is not None:
                self._ui_log(
                    f"{self._spec.label}: {retry_key} retry {cfg['count']}/{cfg['max']} ({error_msg})"
                )

        self._got_result = False
        self._got_traj = (self._spec.expect_traj_kind is None) if self._spec else False
        self._gate_last_state = ""
        self._stability_counter = 0

        delay = max(0, int(cfg["delay"]))
        if retry_key == "EXEC_ERROR":
            delay = max(delay, 500)

        QtCore.QTimer.singleShot(delay, self._try_send_current)
        return True

    # ---------------------------------------------------------------------
    # Callbacks
    # ---------------------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, text: str) -> None:
        if not self._active or self._stop or self._spec is None:
            return

        try:
            key, status, d = parse_motion_result(str(text or ""))
        except Exception:
            return

        if key.as_tuple() != self._spec.expect_result_key.as_tuple():
            return

        up = str(status or "").strip().upper()
        msg = str(d.get("msg") or d.get("error") or up).upper()

        # 1) Fatal invalid request
        if "INVALID" in up or "INVALID" in msg:
            self._fail(f"FATAL: Invalid Request: {msg}")
            return

        # 2) Retryable errors
        if up.startswith("ERROR") or "ERROR" in up:
            if up.startswith("ERROR:GOAL_REJECTED") or "GOAL_REJECTED" in msg:
                if self._attempt_retry("GOAL_REJECTED", msg):
                    return
            if self._attempt_retry("EXEC_ERROR", msg):
                return
            self._fail(f"ERROR final: {msg}")
            return

        # 3) Success only if matches expected status
        if up != str(self._spec.expect_status).upper():
            return

        self._ui_log(f"{self._spec.label}: result OK ({self._spec.expect_status})")
        self._got_result = True
        self._maybe_finish()

    def _consume_traj(self, *, kind: str, obj: Any) -> None:
        if not self._active or self._stop or self._spec is None:
            return

        spec = self._spec
        if spec.expect_traj_kind != kind:
            return
        if not spec.expect_traj_key:
            return
        if obj is None or is_empty_robot_trajectory(obj):
            return

        try:
            key = parse_key_from_traj_header(obj)
        except Exception:
            return

        if key.as_tuple() != spec.expect_traj_key.as_tuple():
            return

        self._ui_log(f"{spec.label}: traj OK ({kind})")
        self._got_traj = True
        self._maybe_finish()

    @QtCore.pyqtSlot(object)
    def _on_planned_traj(self, o: object) -> None:
        self._consume_traj(kind="planned", obj=o)

    @QtCore.pyqtSlot(object)
    def _on_executed_traj(self, o: object) -> None:
        self._consume_traj(kind="executed", obj=o)

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj(self, o: object) -> None:
        self._consume_traj(kind="optimized", obj=o)

    # ---------------------------------------------------------------------
    # Finish / fail
    # ---------------------------------------------------------------------

    def _fail(self, msg: str) -> None:
        label = self._spec.label if self._spec else "StepRunner"
        self._active = False
        self.error.emit(f"{label}: {msg}")

    def _maybe_finish(self) -> None:
        if not self._active or self._stop or self._spec is None:
            return
        if self._got_result and self._got_traj:
            self._ui_log(f"{self._spec.label}: DONE")
            self._active = False
            QtCore.QTimer.singleShot(50, self.finished.emit)


# =============================================================================
# SegmentRunner (Wrapper)
# =============================================================================

class SegmentRunner(QtCore.QObject):
    finished = QtCore.pyqtSignal()
    error = QtCore.pyqtSignal(str)
    logLine = QtCore.pyqtSignal(str)

    def __init__(
        self,
        *,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_goal_rejected_retries: int = 100,
        goal_rejected_retry_delay_ms: int = 0,
        ready_poll_delay_ms: int = 50,
    ) -> None:
        super().__init__(parent)
        self._ros = ros
        self._steps: List[StepSpec] = []
        self._idx: int = 0
        self._stop: bool = False

        self._step_runner = StepRunner(
            ros=self._ros,
            parent=self,
            max_goal_rejected_retries=int(max_goal_rejected_retries),
            goal_rejected_retry_delay_ms=int(goal_rejected_retry_delay_ms),
            ready_poll_delay_ms=int(ready_poll_delay_ms),
        )
        self._step_runner.finished.connect(self._on_step_finished)
        self._step_runner.error.connect(self._on_step_error)

    # -------------------------------------------------------------------------
    # Logging forward
    # -------------------------------------------------------------------------

    def _emit_log(self, text: str) -> None:
        msg = str(text or "")
        try:
            self.logLine.emit(msg)
        except Exception:
            pass

        p = self.parent()
        if p and hasattr(p, "logMessage"):
            try:
                p.logMessage.emit(msg)  # type: ignore[attr-defined]
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

    # -------------------------------------------------------------------------
    # STRICT plan_request sender (Variant-A) - RosBridge compatible
    # -------------------------------------------------------------------------

    def _emit_plan_request(self, *, key: ReqKey, payload: Optional[Dict[str, Any]]) -> None:
        """
        STRICT: uses RosBridge.moveit_publish_plan_request(*, key=dict, payload=dict)

        This avoids sending JSON strings from the UI and matches your RosBridge API.
        """
        ros = self._ros
        fn = getattr(ros, "moveit_publish_plan_request", None)
        if fn is None or not callable(fn):
            raise AttributeError("RosBridge missing moveit_publish_plan_request(*, key: dict, payload: dict)")

        key_dict = {"run": str(key.run), "id": int(key.id), "seg": str(key.seg), "op": str(key.op)}
        payload_dict = dict(payload or {})

        fn(key=key_dict, payload=payload_dict)  # keyword-only call (matches ros_bridge.py)

    # -------------------------------------------------------------------------
    # Bound Step Builders
    # -------------------------------------------------------------------------

    def make_plan_pilz_sequence_step_bound(
        self,
        *,
        run: str,
        req_id: int,
        seg: str,
        payload: Dict[str, Any],
        label: str,
    ) -> StepSpec:
        """
        Pilz sequence planning through the single plan_request channel (op=plan_pilz_sequence).
        """
        key = ReqKey(run=str(run), id=int(req_id), seg=str(seg), op="plan_pilz_sequence")

        def _send() -> None:
            self._emit_plan_request(key=key, payload=dict(payload or {}))

        return StepSpec(
            label=str(label),
            send=_send,
            expect_result_key=key,
            expect_status="PLANNED:OK",
            expect_traj_kind="planned",
            expect_traj_key=key,
        )

    def make_plan_pose_step_bound(self, *, run: str, req_id: int, seg: str, pose: Any, label: str) -> StepSpec:
        """
        plan_pose via RosBridge facade (still plan_request under the hood).
        """
        key = ReqKey(run=str(run), id=int(req_id), seg=str(seg), op="plan_pose")
        return StepSpec(
            label=str(label),
            send=lambda: self._ros.moveit_plan_pose(pose, run=key.run, req_id=key.id, segment=key.seg),
            expect_result_key=key,
            expect_status="PLANNED:OK",
            expect_traj_kind="planned",
            expect_traj_key=key,
        )

    def make_plan_cartesian_step_bound(
        self,
        *,
        run: str,
        req_id: int,
        seg: str,
        poses: List[Any],
        label: str,
    ) -> StepSpec:
        """
        CARTESIAN planning via RosBridge facade (if present).
        NOTE: requires ros.moveit_plan_cartesian_path(...) to exist.
        """
        key = ReqKey(run=str(run), id=int(req_id), seg=str(seg), op="plan_cartesian")
        return StepSpec(
            label=str(label),
            send=lambda: self._ros.moveit_plan_cartesian_path(poses, run=key.run, req_id=key.id, segment=key.seg),
            expect_result_key=key,
            expect_status="PLANNED:OK",
            expect_traj_kind="planned",
            expect_traj_key=key,
        )

    def make_execute_last_planned_step_bound(self, *, run: str, req_id: int, seg: str, label: str) -> StepSpec:
        """
        Execute must go through plan_request op='execute'.
        Node routes it to ExecuteRequestHandler and uses its last_planned.
        """
        key = ReqKey(run=str(run), id=int(req_id), seg=str(seg), op="execute")
        return StepSpec(
            label=str(label),
            send=lambda: self._emit_plan_request(key=key, payload={}),
            expect_result_key=key,
            expect_status="EXECUTED:OK",
            expect_traj_kind="executed",
            expect_traj_key=key,
        )

    def make_execute_trajectory_step_bound(self, *, traj: Any, label: str) -> StepSpec:
        """
        Execute explicit RobotTrajectory via execute_trajectory topic (external replay).
        Result key MUST match the embedded key from traj header JSON.
        """
        key = parse_key_from_traj_header(traj)
        return StepSpec(
            label=str(label),
            send=lambda: self._ros.moveit_execute_trajectory(traj),
            expect_result_key=key,
            expect_status="EXECUTED:OK",
            expect_traj_kind="executed",
            expect_traj_key=key,
        )

    def make_optimize_trajectory_step_bound(self, *, traj: Any, label: str) -> StepSpec:
        """
        Optimize explicit RobotTrajectory via optimize_trajectory topic.
        Result key MUST match the embedded key from traj header JSON.
        """
        key = parse_key_from_traj_header(traj)
        return StepSpec(
            label=str(label),
            send=lambda: self._ros.moveit_optimize_trajectory(traj),
            expect_result_key=key,
            expect_status="OPTIMIZED:OK",
            expect_traj_kind="optimized",
            expect_traj_key=key,
        )

    # -------------------------------------------------------------------------
    # Execution
    # -------------------------------------------------------------------------

    def run(self, steps: List[StepSpec]) -> None:
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
