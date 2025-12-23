# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

from typing import Any, Dict

from PyQt6 import QtCore
from trajectory_msgs.msg import JointTrajectory

from .base_statemachine import BaseProcessStatemachine, SEG_ORDER


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute:
      - nutzt optimize_traj (oder validate_traj)
      - fährt Segmente exakt so wie sie sind
    """

    def __init__(self, *, recipe: Any, ros: Any, parent=None) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent)
        self._segments: Dict[str, JointTrajectory] = {}

    def _segment_exists(self, seg_name: str) -> bool:
        return seg_name in self._segments and len(self._segments[seg_name].points) >= 2

    def _prepare_run(self) -> bool:
        data = self._recipe.get_execute_traj()
        if not data:
            self._error_msg = "Kein execute_traj im Recipe"
            return False

        self._segments = data["segments"]
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        jt = self._segments.get(seg_name)
        if not jt:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        self._ros.moveit_execute_joint_trajectory(jt)
