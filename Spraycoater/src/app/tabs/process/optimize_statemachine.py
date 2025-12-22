# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, Dict, List

import yaml
from PyQt6 import QtCore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .base_statemachine import BaseProcessStatemachine, SEG_ORDER, STATE_MOVE_HOME

_LOG = logging.getLogger("app.tabs.process.optimize_statemachine")


class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    """
    Optimize:
      - lädt run yaml (aus recipe)
      - retime (nur zeit/vel, keine geometrie)
      - exec segmentweise
    """

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        time_scale: float = 1.0,
        min_dt: float = 0.01,
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=2, skip_home=True)
        self._time_scale = float(time_scale)
        self._min_dt = float(min_dt)

        self._joints: List[str] = []
        self._segments_traj: Dict[str, JointTrajectory] = {}

    def _segment_exists(self, seg_name: str) -> bool:
        if self._skip_home and seg_name == STATE_MOVE_HOME:
            return False
        return seg_name in self._segments_traj and len(self._segments_traj[seg_name].points) >= 2

    def _prepare_run(self) -> bool:
        # run yaml path aus recipe
        run_yaml_path = getattr(self._recipe, "run_yaml_path", None)
        if not run_yaml_path and hasattr(self._recipe, "get_run_yaml_path"):
            run_yaml_path = self._recipe.get_run_yaml_path(mode="optimize")
        if not run_yaml_path:
            self._error_msg = "Optimize: recipe.run_yaml_path fehlt."
            return False

        try:
            with open(run_yaml_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
        except Exception as e:
            self._error_msg = f"Optimize: Run-YAML konnte nicht geladen werden: {e}"
            return False

        joints = data.get("joints") or []
        segs = data.get("segments") or []
        if not isinstance(joints, list) or not joints or not isinstance(segs, list) or not segs:
            self._error_msg = "Optimize: Ungültiges Run-YAML (joints/segments)."
            return False

        self._joints = list(joints)
        self._segments_traj = {}
        for seg in segs:
            name = (seg.get("name") or "").strip().upper()
            pts = seg.get("points") or []
            if name in SEG_ORDER and isinstance(pts, list) and pts:
                jt = self._points_to_joint_traj(pts)
                if jt and len(jt.points) >= 2:
                    self._segments_traj[name] = self._retime(jt)

        if not self._segments_traj:
            self._error_msg = "Optimize: Keine Segmente im YAML."
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
                t = last_t + max(self._min_dt, 0.001)
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            jt.points.append(pt)

        return jt if len(jt.points) >= 2 else None

    def _retime(self, jt: JointTrajectory) -> JointTrajectory:
        s = max(0.05, float(self._time_scale))
        if abs(s - 1.0) < 1e-6:
            return jt

        out = JointTrajectory()
        out.joint_names = list(jt.joint_names)
        for p in jt.points:
            t = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            t2 = max(self._min_dt, t * s)
            q = JointTrajectoryPoint()
            q.positions = list(p.positions)
            q.time_from_start.sec = int(t2)
            q.time_from_start.nanosec = int((t2 - int(t2)) * 1e9)
            out.points.append(q)
        return out

    def _on_enter_segment(self, seg_name: str) -> None:
        jt = self._segments_traj.get(seg_name)
        if jt is None:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # execute via ros
        for fn in ("moveit_execute_joint_trajectory", "moveit_execute_trajectory", "moveit_execute"):
            if hasattr(self._ros, fn):
                getattr(self._ros, fn)(jt)
                return

        self._signal_error("Optimize: RosBridge hat keine moveit_execute_* Methode.")

    def _build_result(self):
        out = super()._build_result()
        out["mode"] = "optimize"
        return out
