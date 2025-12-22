# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, Dict, List

import yaml
from PyQt6 import QtCore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .base_statemachine import BaseProcessStatemachine, SEG_ORDER, STATE_MOVE_HOME

_LOG = logging.getLogger("app.tabs.process.execute_statemachine")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute:
      - lädt run yaml (aus recipe)
      - baut segment JointTrajectory
      - exec segmentweise
    """

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        skip_home: bool = False,
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=2, skip_home=skip_home)
        self._joints: List[str] = []
        self._segments_traj: Dict[str, JointTrajectory] = {}

    def _segment_exists(self, seg_name: str) -> bool:
        if self._skip_home and seg_name == STATE_MOVE_HOME:
            return False
        return seg_name in self._segments_traj and len(self._segments_traj[seg_name].points) >= 2

    def _prepare_run(self) -> bool:
        run_yaml_path = getattr(self._recipe, "run_yaml_path", None)
        if not run_yaml_path and hasattr(self._recipe, "get_run_yaml_path"):
            run_yaml_path = self._recipe.get_run_yaml_path(mode="execute")
        if not run_yaml_path:
            self._error_msg = "Execute: recipe.run_yaml_path fehlt."
            return False

        try:
            with open(run_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self._error_msg = f"Execute: Run-YAML konnte nicht geladen werden: {e}"
            return False

        joints = data.get("joints") or []
        segs = data.get("segments") or []
        if not isinstance(joints, list) or not joints or not isinstance(segs, list) or not segs:
            self._error_msg = "Execute: Ungültiges Run-YAML (joints/segments)."
            return False

        self._joints = list(joints)
        self._segments_traj = {}

        # map segments by name
        for seg in segs:
            name = (seg.get("name") or "").strip().upper()
            pts = seg.get("points") or []
            if name in SEG_ORDER and isinstance(pts, list) and pts:
                jt = self._points_to_joint_traj(pts)
                if jt and len(jt.points) >= 2:
                    self._segments_traj[name] = jt

        if not self._segments_traj:
            self._error_msg = "Execute: Keine gültigen Segmente."
            return False

        return True

    def _points_to_joint_traj(self, pts: List[dict]) -> Optional[JointTrajectory]:
        n_j = len(self._joints)
        if n_j <= 0:
            return None

        try:
            t0 = float(pts[0].get("t", 0.0))
        except Exception:
            t0 = 0.0

        jt = JointTrajectory()
        jt.joint_names = list(self._joints)

        last_t = -1.0
        for p in pts:
            pos = p.get("positions") or []
            if not isinstance(pos, list) or len(pos) != n_j:
                continue
            try:
                t = float(p.get("t", 0.0)) - t0
            except Exception:
                continue
            if t <= last_t:
                t = last_t + 0.001
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            jt.points.append(pt)

        return jt if len(jt.points) >= 2 else None

    def _on_enter_segment(self, seg_name: str) -> None:
        jt = self._segments_traj.get(seg_name)
        if jt is None:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        for fn in ("moveit_execute_joint_trajectory", "moveit_execute_trajectory", "moveit_execute"):
            if hasattr(self._ros, fn):
                getattr(self._ros, fn)(jt)
                return

        self._signal_error("Execute: RosBridge hat keine moveit_execute_* Methode.")

    def _build_result(self):
        out = super()._build_result()
        out["mode"] = "execute"
        return out
