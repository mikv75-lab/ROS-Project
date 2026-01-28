# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import copy
import logging
from typing import Any, Dict, List, Optional, Tuple

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

# JTBySegment -> RobotTrajectoryMsg
from builtin_interfaces.msg import Duration as RosDuration  # type: ignore
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore

_LOG = logging.getLogger("tabs.process.execute_sm")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute run (STRICT, collect-only):

    GOAL:
      - Execute drives the whole path "in one piece" (single MoveIt execute call).
      - State machine must NOT step through MOVE_RECIPE/MOVE_RETREAT/etc. waiting for additional
        motionResult signals (those will not come in single-piece mode).

    Contract:
      - Planned baseline is recipe.planned_traj (JTBySegment YAML v1).
      - Executed capture may arrive as one blob -> we split it deterministically by planned point counts.
      - No evaluation here. Only trajectory collection into RunResult.
    """

    ROLE = "execute"

    # "execute in one piece" mode (STRICT: always True for Execute)
    EXECUTE_AS_SINGLE_TRAJECTORY: bool = True
    ALL_SEGMENT_ID: str = "__EXECUTE_ALL__"

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

        # planned baseline (loaded from recipe.planned_traj)
        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}

        # cached planned JT dict per segment (normalized time_from_start dict)
        self._planned_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        # last "motion command" for retry (execute only)
        self._inflight_seg: str = ""
        self._inflight_traj: Optional[RobotTrajectoryMsg] = None

        # single-trajectory execution cache
        self._exec_seg_ids: List[str] = []  # ALL segments present in YAML, for concat/splitting (independent of SM gating)
        self._exec_traj_msg: Optional[RobotTrajectoryMsg] = None
        self._planned_point_counts: Dict[str, int] = {}

        # Segment name used for state-machine pairing / capture bucketing
        # In single-piece mode we commit exactly ONE pair under this segment key.
        self._commit_seg_name: str = STATE_MOVE_PREDISPENSE

    # ------------------------------------------------------------------
    # STOP OVERRIDE
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._inflight_seg = ""
        self._inflight_traj = None
        self._exec_traj_msg = None
        super().request_stop()

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
        self._inflight_traj = None
        self._exec_traj_msg = None
        self._exec_seg_ids = []
        self._planned_point_counts = {}
        self._commit_seg_name = STATE_MOVE_PREDISPENSE

        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self._error_msg = "Execute: recipe.planned_traj fehlt (planned_traj.yaml nicht geladen?)."
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self._error_msg = "Execute: recipe.planned_traj hat kein to_yaml_dict() (JTBySegment API fehlt)."
            return False

        d = fn()
        if not isinstance(d, dict):
            self._error_msg = "Execute: planned_traj.to_yaml_dict() returned non-dict."
            return False

        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self._error_msg = f"Execute: planned_traj YAML invalid: {e}"
            return False

        self._planned_loaded_yaml = d

        # per-segment availability (>=2 points)
        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        # Build strict planned JT dicts (normalize time_from_start to dict)
        try:
            self._planned_jt_by_segment = {}
            for seg_name, present in self._yaml_segments_present.items():
                if not present:
                    continue
                self._planned_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment(seg_name)
        except Exception as e:
            self._error_msg = f"Execute: build planned JT cache failed: {e}"
            return False

        # Determine ALL execute segments in order (for concat + splitting) - DO NOT use _segment_exists here
        self._exec_seg_ids = [
            s
            for s in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME)
            if bool(self._yaml_segments_present.get(s, False))
        ]
        if not self._exec_seg_ids:
            self._error_msg = "Execute: no segments present in planned_traj (nothing to execute)."
            return False

        # Commit bucket: we always commit the single executed blob under MOVE_PREDISPENSE
        # (state machine will only run this state in single-piece mode)
        if not bool(self._yaml_segments_present.get(STATE_MOVE_PREDISPENSE, False)):
            # If predispense is missing but recipe exists, we still need a deterministic first state.
            # STRICT: require predispense segment presence in planned_traj for Execute.
            self._error_msg = f"Execute: planned_traj missing required segment {STATE_MOVE_PREDISPENSE!r}."
            return False
        self._commit_seg_name = STATE_MOVE_PREDISPENSE

        # Cache planned point counts for deterministic executed splitting
        try:
            self._planned_point_counts = {}
            for seg_name in self._exec_seg_ids:
                seg = self._planned_loaded_yaml.get("segments", {}).get(seg_name)
                pts = seg.get("points") if isinstance(seg, dict) else None
                self._planned_point_counts[seg_name] = int(len(pts)) if isinstance(pts, list) else 0
                if self._planned_point_counts[seg_name] < 2:
                    raise ValueError(f"segment {seg_name!r} has <2 planned points")
        except Exception as e:
            self._error_msg = f"Execute: planned point count cache failed: {e}"
            return False

        # REQUIRE ROS API
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self._error_msg = "Execute: ros.moveit_execute_trajectory(traj, segment=...) fehlt (strict)."
            return False

        # Build concatenated RobotTrajectoryMsg (single-piece execution)
        if self.EXECUTE_AS_SINGLE_TRAJECTORY:
            try:
                self._exec_traj_msg = self._robot_trajectory_concat_from_yaml(self._exec_seg_ids)
            except Exception as e:
                self._error_msg = f"Execute: build concatenated trajectory failed: {e}"
                return False

        return True

    # ------------------------------------------------------------------
    # Strict YAML validation
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_jtbysegment_yaml_v1(d: Dict[str, Any]) -> None:
        if not isinstance(d, dict):
            raise ValueError("root is not dict")

        try:
            ver = int(d.get("version", 0))
        except Exception:
            raise ValueError("missing/invalid version")

        if ver != 1:
            raise ValueError(f"version must be 1, got {ver!r}")

        segs = d.get("segments", None)
        if not isinstance(segs, dict) or not segs:
            raise ValueError("missing/empty segments")

        for seg_name, seg in segs.items():
            if not isinstance(seg, dict):
                raise ValueError(f"segment {seg_name!r} is not dict")

            jn = seg.get("joint_names", None)
            if not (isinstance(jn, list) and jn and all(isinstance(x, str) and x.strip() for x in jn)):
                raise ValueError(f"segment {seg_name!r}: invalid joint_names")

            pts = seg.get("points", None)
            if not isinstance(pts, list) or len(pts) < 2:
                raise ValueError(f"segment {seg_name!r}: needs >=2 points")

            last_t = None
            for i, p in enumerate(pts):
                if not isinstance(p, dict):
                    raise ValueError(f"segment {seg_name!r}: point[{i}] not dict")

                pos = p.get("positions", None)
                if not isinstance(pos, list) or len(pos) != len(jn):
                    raise ValueError(
                        f"segment {seg_name!r}: point[{i}] positions length "
                        f"{0 if not isinstance(pos, list) else len(pos)} != {len(jn)}"
                    )
                _ = [float(x) for x in pos]

                tfs = p.get("time_from_start", None)
                if not (isinstance(tfs, (list, tuple)) and len(tfs) >= 2):
                    raise ValueError(f"segment {seg_name!r}: point[{i}] time_from_start must be [sec,nsec]")

                sec = int(tfs[0])
                nsec = int(tfs[1])
                if sec < 0 or nsec < 0:
                    raise ValueError(f"segment {seg_name!r}: point[{i}] negative time_from_start")

                t_key = (sec, nsec)
                if last_t is not None and t_key < last_t:
                    raise ValueError(f"segment {seg_name!r}: time_from_start not monotonic at point[{i}]")
                last_t = t_key

    def _segment_has_points(self, seg_name: str) -> bool:
        segs = self._planned_loaded_yaml.get("segments", {})
        seg = segs.get(seg_name, None) if isinstance(segs, dict) else None
        if not isinstance(seg, dict):
            return False
        pts = seg.get("points", None)
        return isinstance(pts, list) and len(pts) >= 2

    # ------------------------------------------------------------------
    # Segment gating (STATE MACHINE)
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        """
        IMPORTANT:
          - In single-piece mode we MUST only run ONE state to avoid waiting for
            non-existent per-segment EXECUTED:OK signals.
          - We commit the executed blob under MOVE_PREDISPENSE.
        """
        if self.EXECUTE_AS_SINGLE_TRAJECTORY:
            return seg_name == self._commit_seg_name
        return bool(self._yaml_segments_present.get(seg_name, False))

    # ------------------------------------------------------------------
    # Planned JT dict from YAML (for pairing) -> normalized time_from_start dict
    # ------------------------------------------------------------------

    def _jt_dict_from_yaml_segment(self, seg_name: str) -> Dict[str, Any]:
        seg = self._planned_loaded_yaml["segments"].get(seg_name, None)
        if not isinstance(seg, dict):
            raise ValueError(f"Execute: Segment '{seg_name}' fehlt in planned_traj.")
        jn = seg.get("joint_names")
        pts = seg.get("points")
        if not (isinstance(jn, list) and isinstance(pts, list) and len(pts) >= 2):
            raise ValueError(f"Execute: Segment '{seg_name}' invalid in planned_traj (jn/pts).")

        out_pts: List[Dict[str, Any]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            tfs = p.get("time_from_start")
            if not (isinstance(pos, list) and isinstance(tfs, (list, tuple)) and len(tfs) >= 2):
                continue
            out_pts.append(
                {
                    "positions": [float(x) for x in pos],
                    "time_from_start": {"sec": int(tfs[0]), "nanosec": int(tfs[1])},
                }
            )

        if len(out_pts) < 2:
            raise ValueError(f"Execute: Segment '{seg_name}' has <2 valid points after normalization.")

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    # ------------------------------------------------------------------
    # Retry hook (Execute): re-send last execute command
    # ------------------------------------------------------------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        if self._stop_requested:
            return False
        if not self._machine or not self._machine.isRunning():
            return False
        if seg_name != self._current_state:
            return False
        if not self._inflight_traj:
            return False

        _LOG.warning(
            "Execute: retry last motion attempt %d for state=%s reason=%s",
            int(attempt),
            str(seg_name),
            str(last_error),
        )

        try:
            self._ros.moveit_execute_trajectory(self._inflight_traj, segment=str(self._inflight_seg or seg_name))
            return True
        except Exception as e:
            _LOG.error("Execute: retry failed for state=%s: %r", str(seg_name), e)
            return False

    # ------------------------------------------------------------------
    # YAML -> RobotTrajectoryMsg
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_yaml(tfs: Any) -> RosDuration:
        """
        Accept both JTBySegment YAML v1 style [sec,nsec] and normalized dict {"sec","nanosec"}.
        """
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

    def _robot_trajectory_from_yaml(self, seg_name: str) -> RobotTrajectoryMsg:
        seg = self._planned_loaded_yaml["segments"].get(seg_name, None)
        if not isinstance(seg, dict):
            raise ValueError(f"Execute: Segment '{seg_name}' fehlt in planned_traj.")

        joint_names = seg.get("joint_names", [])
        points = seg.get("points", [])

        jt = JointTrajectory()
        jt.joint_names = [str(x) for x in joint_names]

        for p in points:
            if not isinstance(p, dict) or "positions" not in p:
                continue

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in p.get("positions") or []]
            pt.time_from_start = self._duration_from_yaml(p.get("time_from_start"))

            if "velocities" in p:
                pt.velocities = [float(x) for x in p.get("velocities") or []]
            if "accelerations" in p:
                pt.accelerations = [float(x) for x in p.get("accelerations") or []]
            if "effort" in p:
                pt.effort = [float(x) for x in p.get("effort") or []]

            jt.points.append(pt)

        if len(jt.points) < 2:
            raise ValueError(f"Execute: Segment '{seg_name}' hat <2 Punkte.")

        msg = RobotTrajectoryMsg()
        msg.joint_trajectory = jt
        return msg

    # ------------------------------------------------------------------
    # Single-piece concatenation helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _tfs_to_ns(tfs: Any) -> int:
        if isinstance(tfs, dict):
            sec = int(tfs.get("sec", 0))
            nsec = int(tfs.get("nanosec", 0))
            return sec * 1_000_000_000 + nsec
        if isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
            return int(tfs[0]) * 1_000_000_000 + int(tfs[1])
        return 0

    @staticmethod
    def _ns_to_tfs_list(ns: int) -> List[int]:
        if ns < 0:
            ns = 0
        sec = ns // 1_000_000_000
        nsec = ns - (sec * 1_000_000_000)
        return [int(sec), int(nsec)]

    def _robot_trajectory_concat_from_yaml(self, seg_ids: List[str]) -> RobotTrajectoryMsg:
        """
        Build ONE RobotTrajectoryMsg containing all points from the listed segments.
        time_from_start is made strictly increasing by offsetting each segment by cumulative duration.
        """
        if not seg_ids:
            raise ValueError("concat: seg_ids empty")

        # Validate joint_names consistency across segments
        base_jn: Optional[List[str]] = None
        for seg_name in seg_ids:
            seg = self._planned_loaded_yaml["segments"].get(seg_name)
            if not isinstance(seg, dict):
                raise ValueError(f"concat: segment {seg_name!r} missing")
            jn = seg.get("joint_names")
            if not (isinstance(jn, list) and jn and all(isinstance(x, str) for x in jn)):
                raise ValueError(f"concat: segment {seg_name!r} invalid joint_names")
            if base_jn is None:
                base_jn = [str(x) for x in jn]
            else:
                if [str(x) for x in jn] != base_jn:
                    raise ValueError(f"concat: joint_names mismatch in segment {seg_name!r}")

        assert base_jn is not None

        jt = JointTrajectory()
        jt.joint_names = base_jn

        offset_ns = 0
        first = True

        for seg_name in seg_ids:
            seg = self._planned_loaded_yaml["segments"][seg_name]
            pts = seg.get("points")
            if not isinstance(pts, list) or len(pts) < 2:
                raise ValueError(f"concat: segment {seg_name!r} has <2 points")

            seg_start_ns = self._tfs_to_ns(pts[0].get("time_from_start")) if isinstance(pts[0], dict) else 0

            for i, p in enumerate(pts):
                if not isinstance(p, dict) or "positions" not in p:
                    continue

                # Drop the first point of subsequent segments to avoid duplicate boundary points
                if not first and i == 0:
                    continue

                pt = JointTrajectoryPoint()
                pt.positions = [float(x) for x in p.get("positions") or []]

                local_ns = self._tfs_to_ns(p.get("time_from_start")) - seg_start_ns
                if local_ns < 0:
                    local_ns = 0
                t_ns = offset_ns + local_ns

                d = RosDuration()
                d.sec = int(t_ns // 1_000_000_000)
                d.nanosec = int(t_ns - d.sec * 1_000_000_000)
                pt.time_from_start = d

                if "velocities" in p:
                    pt.velocities = [float(x) for x in p.get("velocities") or []]
                if "accelerations" in p:
                    pt.accelerations = [float(x) for x in p.get("accelerations") or []]
                if "effort" in p:
                    pt.effort = [float(x) for x in p.get("effort") or []]

                jt.points.append(pt)

            last_ns = self._tfs_to_ns(pts[-1].get("time_from_start")) if isinstance(pts[-1], dict) else 0
            seg_dur_ns = max(0, last_ns - seg_start_ns)
            offset_ns += seg_dur_ns
            first = False

        if len(jt.points) < 2:
            raise ValueError("concat: resulting trajectory has <2 points")

        msg = RobotTrajectoryMsg()
        msg.joint_trajectory = jt
        return msg

    # ------------------------------------------------------------------
    # Execution entry (single-piece)
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        # In single-piece mode, the state machine runs exactly one state (MOVE_PREDISPENSE).
        if self.EXECUTE_AS_SINGLE_TRAJECTORY:
            if seg_name != self._commit_seg_name:
                self._signal_error(f"Execute: expected only state {self._commit_seg_name}, got {seg_name}")
                return

            if self._inflight_traj is not None:
                # Already executing; ignore re-entry.
                return

            if not self._exec_traj_msg:
                self._signal_error("Execute: missing concatenated trajectory (prepare failed?)")
                return

            # IMPORTANT: pre-seed planned pending for commit bucket so Base can commit on EXECUTED:OK only.
            planned_jt = self._planned_jt_by_segment.get(self._commit_seg_name)
            if not isinstance(planned_jt, dict):
                self._signal_error(f"Execute: missing planned JT baseline for seg={self._commit_seg_name}")
                return
            self._pending_planned_step[self._commit_seg_name] = copy.deepcopy(planned_jt)
            self._pending_executed_step[self._commit_seg_name] = None

            # Use stable segment id for correlation on the ROS side (even if Base buckets into MOVE_PREDISPENSE)
            self._inflight_seg = str(self.ALL_SEGMENT_ID)
            self._inflight_traj = self._exec_traj_msg

            try:
                self._ros.moveit_execute_trajectory(self._exec_traj_msg, segment=str(self._inflight_seg))
            except Exception as e:
                self._signal_error(f"Execute: moveit_execute_trajectory failed: {e}")
            return

        # Fallback (should not be used for Execute, but kept strict)
        if not bool(self._yaml_segments_present.get(seg_name, False)):
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        try:
            traj_msg = self._robot_trajectory_from_yaml(seg_name)
        except Exception as e:
            self._signal_error(f"Execute: YAML->RobotTrajectory failed for {seg_name}: {e}")
            return

        self._inflight_seg = str(seg_name)
        self._inflight_traj = traj_msg

        try:
            self._ros.moveit_execute_trajectory(traj_msg, segment=str(seg_name))
        except Exception as e:
            self._signal_error(f"Execute: moveit_execute_trajectory failed for {seg_name}: {e}")

    # ------------------------------------------------------------------
    # Planned/Executed payload
    # ------------------------------------------------------------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        planned = copy.deepcopy(self._planned_loaded_yaml)
        executed = self._jt_by_segment_yaml(which="executed")

        # If single-piece capture produced one segment, split it back into real segments.
        if self.EXECUTE_AS_SINGLE_TRAJECTORY:
            try:
                executed = self._split_executed_if_single_blob(executed)
            except Exception as e:
                _LOG.error("Execute: split executed blob failed: %s", e)

        return planned, executed

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()

        e_segs = executed.get("segments") if isinstance(executed, dict) else None
        if not (isinstance(e_segs, dict) and e_segs):
            self.notifyError.emit(
                "Execute finished, but executed trajectory capture is empty. "
                "Check MoveItPyNode executed_trajectory_rt publish + BaseProcessStatemachine capture."
            )
            self._cleanup()
            return

        self._rr.set_planned(traj=planned)
        self._rr.set_executed(traj=executed)
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # Executed splitting (single-piece capture -> JTBySegment)
    # ------------------------------------------------------------------

    def _split_executed_if_single_blob(self, executed: Dict[str, Any]) -> Dict[str, Any]:
        """
        If executed capture yields only one segment (e.g. MOVE_PREDISPENSE bucket or "__EXECUTE_ALL__"),
        split points back into segments using planned point counts.
        """
        if not isinstance(executed, dict):
            return executed
        if int(executed.get("version", 0) or 0) != 1:
            return executed

        segs = executed.get("segments")
        if not isinstance(segs, dict) or not segs:
            return executed

        if len(segs.keys()) != 1:
            return executed

        only_name = list(segs.keys())[0]
        only_seg = segs.get(only_name)
        if not isinstance(only_seg, dict):
            return executed

        pts = only_seg.get("points")
        jn = only_seg.get("joint_names")
        if not (isinstance(pts, list) and isinstance(jn, list) and len(pts) >= 2):
            return executed

        if not self._exec_seg_ids or not self._planned_point_counts:
            return executed

        total_needed = 0
        for s in self._exec_seg_ids:
            total_needed += int(self._planned_point_counts.get(s, 0))

        expected_concat = total_needed - max(0, (len(self._exec_seg_ids) - 1))
        if len(pts) < expected_concat:
            _LOG.warning(
                "Execute: executed blob has fewer points than expected for split: got=%d expected>=%d",
                int(len(pts)),
                int(expected_concat),
            )
            return executed

        out_segments: Dict[str, Any] = {}
        idx = 0
        first = True

        for seg_name in self._exec_seg_ids:
            n_planned = int(self._planned_point_counts.get(seg_name, 0))
            if n_planned < 2:
                continue

            take = n_planned if first else (n_planned - 1)
            if take < 2:
                return executed

            seg_pts = pts[idx : idx + take]
            idx += take
            if not isinstance(seg_pts, list) or len(seg_pts) < 2:
                return executed

            seg_pts_norm = self._normalize_points_time_from_start(seg_pts)

            out_segments[seg_name] = {
                "joint_names": [str(x) for x in jn],
                "points": seg_pts_norm,
            }
            first = False

        if not out_segments:
            return executed

        return {"version": 1, "segments": out_segments}

    def _normalize_points_time_from_start(self, pts: List[Any]) -> List[Dict[str, Any]]:
        """
        pts: list of point dicts with time_from_start as [sec,nsec] or {"sec","nanosec"}.
        Returns new list with time_from_start as [sec,nsec] starting at 0 for first point.
        """
        if not pts or not isinstance(pts[0], dict):
            return []

        t0_ns = self._tfs_to_ns(pts[0].get("time_from_start"))
        out: List[Dict[str, Any]] = []

        last_ns = -1
        for p in pts:
            if not isinstance(p, dict):
                continue
            q = dict(p)

            t_ns = self._tfs_to_ns(q.get("time_from_start")) - t0_ns
            if t_ns < 0:
                t_ns = 0
            if last_ns >= 0 and t_ns < last_ns:
                t_ns = last_ns
            last_ns = t_ns

            q["time_from_start"] = self._ns_to_tfs_list(int(t_ns))
            out.append(q)

        return out
