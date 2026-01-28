# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import copy
import logging
from typing import Any, Dict, List, Optional

from PyQt6 import QtCore

from plc.plc_client import PlcClientBase
from model.recipe.recipe_run_result import RunResult

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

from builtin_interfaces.msg import Duration as RosDuration  # type: ignore
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore

_LOG = logging.getLogger("tabs.process.execute_sm")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute run (STRICT, collect-only).

    Contract (updated for NEW MoveItPy node behavior):
      - Input: recipe.planned_traj (JTBySegment YAML v1) from Validate.
      - Action: execute per segment via ros.moveit_execute_trajectory(rt, segment=STATE_*).
      - Pairing: Base expects motion_result for the *current* segment.
      - IMPORTANT:
          * For execute_trajectory path, MoveItPy node typically DOES NOT emit "PLANNED:OK".
          * Therefore we MUST seed pending planned right before execute.
          * Node emits traj_cache_clear BEFORE each external move; Base clears only
            cached last_planned/last_executed, but our seeded pending planned stays.
      - No evaluation here. Only collect planned/executed trajectories into RunResult payload.
    """

    ROLE = "execute"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        plc: PlcClientBase | None,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            run_result=run_result,
            parent=parent,
            max_retries=max_retries,
        )
        self._plc = plc
        self._side: str = str(side or "top")

        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}
        self._planned_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        # inflight bookkeeping (for strict retry)
        self._inflight_seg: str = ""
        self._inflight_rt: Optional[RobotTrajectoryMsg] = None
        self._inflight_planned_jt: Optional[Dict[str, Any]] = None  # normalized JT dict for seeding planned

    # ------------------------------------------------------------------
    # STOP
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._inflight_seg = ""
        self._inflight_rt = None
        self._inflight_planned_jt = None
        super().request_stop()

    # ------------------------------------------------------------------
    # Retry hook (STRICT)
    # ------------------------------------------------------------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        if self._stop_requested:
            return False
        if self._machine is None:
            return False
        if seg_name != self._current_state:
            return False
        if self._inflight_seg != seg_name:
            return False
        if self._inflight_rt is None:
            return False

        _LOG.warning(
            "Execute: retry attempt %d for seg=%s reason=%s",
            int(attempt),
            str(seg_name),
            str(last_error),
        )

        # IMPORTANT:
        # - Base will clear pairing slots before retry.
        # - For execute_trajectory, PLANNED:OK may not arrive.
        # - Therefore re-seed planned pending AGAIN for this retry.
        if isinstance(self._inflight_planned_jt, dict):
            self._pending_planned_step[seg_name] = copy.deepcopy(self._inflight_planned_jt)
            self._pending_executed_step[seg_name] = None

        try:
            self._ros.moveit_execute_trajectory(self._inflight_rt, segment=str(seg_name))
            return True
        except Exception as e:
            _LOG.error("Execute: retry execute failed for seg=%s: %r", str(seg_name), e)
            return False

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", self._side) or self._side)
        except Exception:
            pass

        self._inflight_seg = ""
        self._inflight_rt = None
        self._inflight_planned_jt = None

        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self._error_msg = "Execute: recipe.planned_traj fehlt (bitte erst Validate ausführen)."
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self._error_msg = "Execute: planned_traj API fehlt (to_yaml_dict)."
            return False

        d = fn()
        if not isinstance(d, dict):
            self._error_msg = "Execute: planned_traj daten ungültig (non-dict)."
            return False

        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self._error_msg = f"Execute: planned_traj YAML invalid: {e}"
            return False

        self._planned_loaded_yaml = d

        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        try:
            self._planned_jt_by_segment = {}
            for seg_name, present in self._yaml_segments_present.items():
                if not present:
                    continue
                self._planned_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment_normalized(seg_name)
        except Exception as e:
            self._error_msg = f"Execute: build planned JT cache failed: {e}"
            return False

        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self._error_msg = "Execute: ros.moveit_execute_trajectory fehlt."
            return False

        return True

    # ------------------------------------------------------------------
    # YAML validation / normalization
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_jtbysegment_yaml_v1(d: Dict[str, Any]) -> None:
        if not isinstance(d, dict):
            raise ValueError("root is not dict")
        if int(d.get("version", 0)) != 1:
            raise ValueError("version must be 1")
        segs = d.get("segments")
        if not isinstance(segs, dict) or not segs:
            raise ValueError("segments missing/empty")

    def _segment_has_points(self, seg_name: str) -> bool:
        segs = self._planned_loaded_yaml.get("segments", {})
        seg = segs.get(seg_name) if isinstance(segs, dict) else None
        if not isinstance(seg, dict):
            return False
        pts = seg.get("points")
        return isinstance(pts, list) and len(pts) >= 2

    def _jt_dict_from_yaml_segment_normalized(self, seg_name: str) -> Dict[str, Any]:
        segs = self._planned_loaded_yaml.get("segments", {})
        seg = segs.get(seg_name) if isinstance(segs, dict) else None
        if not isinstance(seg, dict):
            raise ValueError(f"Execute: segment '{seg_name}' missing in planned_traj")

        jn = seg.get("joint_names")
        pts = seg.get("points")
        if not (isinstance(jn, list) and jn and isinstance(pts, list) and len(pts) >= 2):
            raise ValueError(f"Execute: segment '{seg_name}' invalid (joint_names/points)")

        out_pts: List[Dict[str, Any]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            tfs = p.get("time_from_start")
            if not isinstance(pos, list):
                continue

            t_sec = 0
            t_nsec = 0
            if isinstance(tfs, dict):
                t_sec = int(tfs.get("sec", 0))
                t_nsec = int(tfs.get("nanosec", 0))
            elif isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
                t_sec = int(tfs[0])
                t_nsec = int(tfs[1])

            q: Dict[str, Any] = {
                "positions": [float(x) for x in pos],
                "time_from_start": {"sec": t_sec, "nanosec": t_nsec},
            }

            if "velocities" in p and isinstance(p.get("velocities"), list):
                q["velocities"] = [float(x) for x in p.get("velocities") or []]
            if "accelerations" in p and isinstance(p.get("accelerations"), list):
                q["accelerations"] = [float(x) for x in p.get("accelerations") or []]
            if "effort" in p and isinstance(p.get("effort"), list):
                q["effort"] = [float(x) for x in p.get("effort") or []]

            out_pts.append(q)

        if len(out_pts) < 2:
            raise ValueError(f"Execute: segment '{seg_name}' has <2 valid points after normalization")

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    # ------------------------------------------------------------------
    # Segment gating
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        return bool(self._yaml_segments_present.get(seg_name, False))

    # ------------------------------------------------------------------
    # Segment enter
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        present = bool(self._yaml_segments_present.get(seg_name, False))
        if not present:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        try:
            planned_jt = self._planned_jt_by_segment.get(seg_name)
            if not isinstance(planned_jt, dict):
                raise ValueError("missing planned jt dict")
            rt = self._robot_trajectory_from_jt_dict(planned_jt)
        except Exception as e:
            self._signal_error(f"Execute: JT->RobotTrajectory failed for {seg_name}: {e}")
            return

        # STRICT: for execute_trajectory, PLANNED:OK is typically absent -> seed planned pending now.
        self._pending_planned_step[seg_name] = copy.deepcopy(planned_jt)
        self._pending_executed_step[seg_name] = None

        self._inflight_seg = str(seg_name)
        self._inflight_rt = rt
        self._inflight_planned_jt = copy.deepcopy(planned_jt)

        try:
            # Node will:
            #   - publish traj_cache_clear
            #   - publish planned_trajectory_rt (best effort)
            #   - execute via FollowJT
            #   - publish executed_trajectory_rt
            #   - then motion_result "EXECUTED:OK seg=..."
            self._ros.moveit_execute_trajectory(rt, segment=str(seg_name))
        except Exception as e:
            self._signal_error(f"Execute: Execute failed: {e}")

    # ------------------------------------------------------------------
    # Segment completion hook
    # ------------------------------------------------------------------

    def _on_segment_ok(self, seg_name: str) -> None:
        super()._on_segment_ok(seg_name)

        # clear inflight marker when the segment completed
        if self._inflight_seg == seg_name:
            self._inflight_seg = ""
            self._inflight_rt = None
            self._inflight_planned_jt = None

    # ------------------------------------------------------------------
    # Final payload
    # ------------------------------------------------------------------

    def _on_finished(self) -> None:
        planned = copy.deepcopy(self._planned_loaded_yaml)
        executed = self._jt_by_segment_yaml(which="executed")

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # Helper: normalized JT dict -> RobotTrajectoryMsg
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_time_dict(tfs: Any) -> RosDuration:
        d = RosDuration()
        if isinstance(tfs, dict):
            d.sec = int(tfs.get("sec", 0))
            d.nanosec = int(tfs.get("nanosec", 0))
            return d
        if isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
            d.sec = int(tfs[0])
            d.nanosec = int(tfs[1])
            return d
        d.sec = 0
        d.nanosec = 0
        return d

    def _robot_trajectory_from_jt_dict(self, jt: Dict[str, Any]) -> RobotTrajectoryMsg:
        jn = jt.get("joint_names") or []
        pts = jt.get("points") or []

        msg = RobotTrajectoryMsg()
        msg.joint_trajectory = JointTrajectory()
        msg.joint_trajectory.joint_names = [str(x) for x in list(jn)]

        for p in pts:
            if not isinstance(p, dict) or "positions" not in p:
                continue
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in list(p.get("positions") or [])]
            pt.time_from_start = self._duration_from_time_dict(p.get("time_from_start"))

            if "velocities" in p and isinstance(p.get("velocities"), list):
                pt.velocities = [float(x) for x in p.get("velocities") or []]
            if "accelerations" in p and isinstance(p.get("accelerations"), list):
                pt.accelerations = [float(x) for x in p.get("accelerations") or []]
            if "effort" in p and isinstance(p.get("effort"), list):
                pt.effort = [float(x) for x in p.get("effort") or []]

            msg.joint_trajectory.points.append(pt)

        return msg
