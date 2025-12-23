# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Dict, Optional, Tuple

from PyQt6 import QtCore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from model.recipe.recipe_run_result import RunResult
from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
)

_LOG = logging.getLogger("tabs.process.optimize_sm")


class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    """
    Optimize:
      - arbeitet auf recipe.trajectories['traj'] (planned)
      - retimed die Zeiten (time_scale, min_dt)
      - KEINE Roboterbewegung (nur "compute"/rewrite)
      - liefert planned_traj im Result zurück (für UI+Persistenz)
    """

    ROLE = "optimize"

    def __init__(self, *, recipe: Any, ros: Any, parent: Optional[QtCore.QObject] = None) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=0, skip_home=True)

        p = getattr(recipe, "optimize_params", {}) or {}
        self._time_scale = float(p.get("time_scale", 1.0))
        self._min_dt = float(p.get("min_dt", 0.01))

        self._segments_in: Dict[str, dict] = {}
        self._segments_out: Dict[str, JointTrajectory] = {}
        self._optimized_payload: Dict[str, Any] = {}

    def _segment_exists(self, seg_name: str) -> bool:
        jt = self._segments_out.get(seg_name)
        return bool(jt and len(jt.points) >= 2)

    def _prepare_run(self) -> bool:
        traj = None
        try:
            traj = (getattr(self._recipe, "trajectories", {}) or {}).get(getattr(self._recipe, "TRAJ_TRAJ", "traj"))
        except Exception:
            traj = None

        if not isinstance(traj, dict) or not traj:
            self._error_msg = "Optimize: Kein planned traj (recipe.trajectories['traj']) vorhanden."
            return False

        segs = traj.get("segments") or {}
        if not isinstance(segs, dict) or not segs:
            self._error_msg = "Optimize: traj.segments ist leer/ungültig."
            return False

        self._segments_in = dict(segs)
        self._segments_out.clear()

        # wir optimieren NUR die 3 Segmente, die die Base-Machine abläuft
        wanted = [STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT]
        for name in wanted:
            src = self._segments_in.get(name)
            if not isinstance(src, dict):
                continue
            jt0 = self._points_to_traj(src)
            if jt0 is None:
                continue
            self._segments_out[name] = self._retime(jt0)

        if not self._segments_out:
            self._error_msg = "Optimize: Keine retime-baren Segmente gefunden (predispense/recipe/retreat)."
            return False

        # payload bauen, den ProcessTab/Repo speichern kann
        self._optimized_payload = {
            "segments": {k: self._traj_to_points(v) for k, v in self._segments_out.items()},
            "meta": {
                "optimized": True,
                "time_scale": float(self._time_scale),
                "min_dt": float(self._min_dt),
            },
        }
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        # Kein Motion-Request; Segment ist “fertig”, sobald es existiert.
        if not self._segment_exists(seg_name):
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # snapshot meta pro segment (damit rr.meta planned_by_segment gefüllt ist)
        try:
            self._planned_by_segment[seg_name] = self._traj_to_points(self._segments_out[seg_name])
        except Exception:
            pass

        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ---------------- helpers ----------------

    def _points_to_traj(self, seg: dict) -> Optional[JointTrajectory]:
        joints = seg.get("joints")
        points = seg.get("points")
        if not isinstance(joints, list) or not isinstance(points, list) or len(joints) == 0:
            return None

        jt = JointTrajectory()
        jt.joint_names = list(joints)

        last_t = -1.0
        for p in points:
            try:
                t_raw = float(p.get("t", 0.0))
                pos = list(p["positions"])
            except Exception:
                continue

            t = max(t_raw, last_t + self._min_dt)
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = pos
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
            jt.points.append(pt)

        return jt if len(jt.points) >= 2 else None

    def _retime(self, jt: JointTrajectory) -> JointTrajectory:
        if abs(self._time_scale - 1.0) < 1e-9:
            return jt

        out = JointTrajectory()
        out.joint_names = list(jt.joint_names)

        last_t = -1.0
        for p in jt.points:
            t = (float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9) * self._time_scale
            t = max(t, last_t + self._min_dt)
            last_t = t

            pt = JointTrajectoryPoint()
            pt.positions = list(p.positions)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1.0) * 1e9)
            out.points.append(pt)

        return out

    def _traj_to_points(self, jt: JointTrajectory) -> dict:
        pts = []
        for p in jt.points:
            t = float(p.time_from_start.sec) + float(p.time_from_start.nanosec) * 1e-9
            pts.append({"t": float(t), "positions": list(p.positions)})
        return {"joints": list(jt.joint_names), "points": pts}

    # ---------------- result override ----------------

    def _build_result(self) -> RunResult:
        rr = RunResult(
            role=self._role,
            ok=not self._stopped and not bool(self._error_msg),
            message="stopped" if self._stopped else "finished",
        )
        rr.meta.update(
            {
                "status": "stopped" if self._stopped else "finished",
                "optimized": True,
                "planned_by_segment": dict(self._planned_by_segment),
                "executed_by_segment": dict(self._executed_by_segment),
            }
        )
        rr.planned_traj = dict(self._optimized_payload or {})
        rr.executed_traj = {}  # optimize erzeugt nichts executed

        rid = getattr(self._recipe, "id", None)
        if rid is not None:
            rr.meta["recipe_id"] = rid
        return rr
