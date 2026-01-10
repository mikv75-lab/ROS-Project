# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import logging
import copy
from typing import Any, Optional, Dict, List, Tuple

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
from builtin_interfaces.msg import Duration as RosDuration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg

_LOG = logging.getLogger("tabs.process.execute_sm")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute run (STRICT, NO FALLBACKS):

      Input:
        - recipe.planned_traj MUST be present and must provide a JTBySegment-like API:
            - to_yaml_dict() -> dict (JTBySegment YAML v1)
        - ros MUST expose:
            - moveit_execute_trajectory(traj: RobotTrajectoryMsg, segment: str = "") -> None

      Behavior:
        - planned baseline is exactly recipe.planned_traj (as YAML dict)
        - for each segment in Base SEG_ORDER:
            - build RobotTrajectoryMsg from YAML segment data
            - execute via ros.moveit_execute_trajectory(..., segment=SEG)
        - executed truth is captured by BaseProcessStatemachine (MoveItPy executed callbacks)

    NOTE (API-Compat):
      - 'skip_home' accepted and forwarded to Base.
    """

    ROLE = "execute"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        plc: PlcClientBase,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            parent=parent,
            max_retries=max_retries,
            skip_home=bool(skip_home),
        )
        self._plc = plc
        self._side: str = str(side or "top")

        # planned baseline (loaded from recipe.planned_traj)
        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # side kept for compat (does not affect joint-trajectory execution)
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", self._side) or self._side)
        except Exception:
            pass

        # REQUIRE recipe.planned_traj (no filesystem, no fallbacks)
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

        # strict validate schema (JTBySegment YAML v1)
        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self._error_msg = f"Execute: planned_traj YAML invalid: {e}"
            return False

        self._planned_loaded_yaml = d

        # precompute segment availability (>=2 points)
        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        # clear base capture buffers (strict)
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()

        # REQUIRE ROS API
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self._error_msg = "Execute: ros.moveit_execute_trajectory(traj, segment=...) fehlt (strict)."
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
                # float-castable
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
        seg = self._planned_loaded_yaml["segments"].get(seg_name, None)
        if not isinstance(seg, dict):
            return False
        pts = seg.get("points", None)
        return isinstance(pts, list) and len(pts) >= 2

    # ------------------------------------------------------------------
    # Segment gating
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and bool(self._skip_home):
            return False
        return bool(self._yaml_segments_present.get(seg_name, False))

    # ------------------------------------------------------------------
    # YAML -> RobotTrajectoryMsg
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_yaml(tfs: Any) -> RosDuration:
        d = RosDuration()
        d.sec = int(tfs[0])
        d.nanosec = int(tfs[1])
        return d

    def _robot_trajectory_from_yaml(self, seg_name: str) -> RobotTrajectoryMsg:
        seg = self._planned_loaded_yaml["segments"].get(seg_name, None)
        if not isinstance(seg, dict):
            raise ValueError(f"Execute: Segment '{seg_name}' fehlt in planned_traj.")

        joint_names = seg["joint_names"]
        points = seg["points"]

        jt = JointTrajectory()
        jt.joint_names = [str(x) for x in joint_names]

        for p in points:
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in p["positions"]]
            pt.time_from_start = self._duration_from_yaml(p["time_from_start"])

            # optional fields only if present
            if "velocities" in p:
                pt.velocities = [float(x) for x in p["velocities"]]
            if "accelerations" in p:
                pt.accelerations = [float(x) for x in p["accelerations"]]
            if "effort" in p:
                pt.effort = [float(x) for x in p["effort"]]

            jt.points.append(pt)

        if len(jt.points) < 2:
            raise ValueError(f"Execute: Segment '{seg_name}' hat <2 Punkte.")

        msg = RobotTrajectoryMsg()
        msg.joint_trajectory = jt
        return msg

    # ------------------------------------------------------------------
    # Execution per segment
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        if seg_name == STATE_MOVE_HOME and bool(self._skip_home):
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        try:
            traj_msg = self._robot_trajectory_from_yaml(seg_name)
        except Exception as e:
            self._signal_error(f"Execute: YAML->RobotTrajectory failed for {seg_name}: {e}")
            return

        # strict (validated in _prepare_run)
        self._ros.moveit_execute_trajectory(traj_msg, segment=str(seg_name))

    # ------------------------------------------------------------------
    # Planned/Executed payload override
    # ------------------------------------------------------------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        planned = copy.deepcopy(self._planned_loaded_yaml)
        executed = self._jt_by_segment_yaml(which="executed")
        return planned, executed

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()

        # executed must exist (point of Execute)
        e_segs = executed.get("segments") if isinstance(executed, dict) else None
        if not (isinstance(e_segs, dict) and e_segs):
            self.notifyError.emit(
                "Execute finished, but executed trajectory capture is empty. "
                "Check MoveItPyNode executed_trajectory_rt publish + BaseProcessStatemachine capture."
            )
            self._cleanup()
            return

        rr = RunResult()
        rr.set_planned(traj=planned)
        rr.set_executed(traj=executed)

        self.notifyFinished.emit(rr.to_process_payload())
        self._cleanup()
