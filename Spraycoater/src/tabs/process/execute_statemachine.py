# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

from typing import Any, Dict

from PyQt6 import QtCore
from trajectory_msgs.msg import JointTrajectory

from .base_statemachine import BaseProcessStatemachine


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute:
      - nutzt vorbereitete Segment-Trajs (z.B. aus Recipe/run yaml)
      - fährt Segmente exakt so wie sie sind
    """

    def __init__(self, *, recipe: Any, ros: Any, parent=None) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent)
        self._segments: Dict[str, JointTrajectory] = {}

    def _segment_exists(self, seg_name: str) -> bool:
        jt = self._segments.get(seg_name)
        return bool(jt and len(jt.points) >= 2)

    def _prepare_run(self) -> bool:
        # Erwartung: du lieferst hier die Segment-Trajs (wie du’s schon hast)
        data = self._recipe.get_execute_traj() if hasattr(self._recipe, "get_execute_traj") else None
        if not data:
            self._error_msg = "Kein execute_traj im Recipe"
            return False

        self._segments = data.get("segments") or {}
        if not isinstance(self._segments, dict) or not self._segments:
            self._error_msg = "execute_traj.segments ist leer/ungültig"
            return False

        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        jt = self._segments.get(seg_name)
        if not jt:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        self._ros.moveit_execute_joint_trajectory(jt)

    def _build_result(self):
        out = super()._build_result()
        out["mode"] = "execute"
        return out
