# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import yaml
from typing import Any, Dict, List, Optional

from PyQt6 import QtCore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .base_statemachine import (
    BaseProcessStatemachine,
    SEG_ORDER,
    STATE_MOVE_HOME,
)


class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    """
    Optimize:
      - nutzt validate_traj aus Recipe
      - retimed JE Segment
      - fährt Segmente aus
    """

    def __init__(self, *, recipe: Any, ros: Any, parent=None) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, skip_home=True)

        p = recipe.optimize_params
        self._time_scale = float(p.get("time_scale", 1.0))
        self._min_dt = float(p.get("min_dt", 0.01))

        self._segments: Dict[str, JointTrajectory] = {}

    def _segment_exists(self, seg_name: str) -> bool:
        return seg_name in self._segments and len(self._segments[seg_name].points) >= 2

    def _prepare_run(self) -> bool:
        data = self._recipe.get_validate_traj()
        if not data:
            self._error_msg = "Kein validate_traj im Recipe"
            return False

        self._segments.clear()
        for seg_name, seg in data["segments"].items():
            jt = self._retime(self._points_to_traj(seg))
            if jt:
                self._segments[seg_name] = jt
        return bool(self._segments)

    def _points_to_traj(self, seg: dict) -> Optional[JointTrajectory]:
        jt = JointTrajectory()
        jt.joint_names = seg["joints"]

        last_t = -1.0
        for p in seg["points"]:
            t = max(float(p["t"]), last_t + self._min_dt)
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = p["positions"]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            jt.points.append(pt)

        return jt if len(jt.points) >= 2 else None

    def _retime(self, jt: JointTrajectory) -> JointTrajectory:
        if abs(self._time_scale - 1.0) < 1e-6:
            return jt

        out = JointTrajectory()
        out.joint_names = list(jt.joint_names)
        for p in jt.points:
            t = (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) * self._time_scale
            pt = JointTrajectoryPoint()
            pt.positions = list(p.positions)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            out.points.append(pt)
        return out

    def _on_enter_segment(self, seg_name: str) -> None:
        jt = self._segments.get(seg_name)
        if not jt:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        self._ros.moveit_execute_joint_trajectory(jt)
