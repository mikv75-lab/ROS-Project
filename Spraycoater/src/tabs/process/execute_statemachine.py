# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import logging
import copy
from typing import Any, Optional, Dict, List, Tuple, Iterable

from PyQt6 import QtCore
from geometry_msgs.msg import PoseArray, Pose

from plc.plc_client import PlcClientBase

from model.recipe.recipe_run_result import RunResult

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
    SEG_ORDER,
)

_LOG = logging.getLogger("tabs.process.execute_sm")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute run:
      - executes the already-loaded compiled path
      - returns executed trajectories captured during execution
      - planned trajectory is taken from the loaded recipe (planned_traj.yaml), not from capture

    Rationale:
      - execute should be "truth = executed"
      - planned is a reference baseline (loaded artifact)
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
        side: str = "top",
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=max_retries, skip_home=False)
        self._plc = plc

        self._side: str = str(side or "top")
        self._frame: str = "scene"

        # robust list of points in mm: [(x,y,z), ...]
        self._pts_mm: List[Tuple[float, float, float]] = []

        # planned traj loaded from recipe (JTBySegment yaml dict)
        self._planned_loaded_yaml: Dict[str, Any] = {}

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # side / frame from params if present
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", self._side) or self._side)
        except Exception:
            pass

        try:
            pc = getattr(self._recipe, "paths_compiled", None)
            if isinstance(pc, dict) and pc.get("frame"):
                self._frame = str(pc.get("frame") or self._frame)
        except Exception:
            pass

        # compiled points (mm) robust
        self._pts_mm = self._get_compiled_points_mm(self._side)
        if len(self._pts_mm) < 2:
            self._error_msg = f"Execute: Zu wenige compiled points für side='{self._side}' (n={len(self._pts_mm)})."
            return False

        # planned trajectory baseline (loaded artifact)
        self._planned_loaded_yaml = {}
        try:
            planned_traj = getattr(self._recipe, "planned_traj", None)
            if planned_traj is not None and hasattr(planned_traj, "to_yaml_dict"):
                d = planned_traj.to_yaml_dict()
                self._planned_loaded_yaml = d if isinstance(d, dict) else {}
        except Exception:
            self._planned_loaded_yaml = {}

        # clear base capture buffers (strict)
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()

        return True

    # ------------------------------------------------------------------
    # Segment existence
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        # We always run HOME unless explicitly skipped (Execute uses skip_home=False in ctor)
        if seg_name == STATE_MOVE_HOME:
            return True

        n = len(self._pts_mm)
        if seg_name == STATE_MOVE_PREDISPENSE:
            return n >= 2
        if seg_name == STATE_MOVE_RECIPE:
            return n >= 3  # needs at least one middle point
        if seg_name == STATE_MOVE_RETREAT:
            return n >= 2
        return True

    # ------------------------------------------------------------------
    # Execution calls
    # ------------------------------------------------------------------

    def _call_first(self, names: List[str], *args, **kwargs) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                fn(*args, **kwargs)
                return True
        return False

    def _pose_array_slice(self, a: int, b: int) -> Optional[PoseArray]:
        n = len(self._pts_mm)
        if n <= 0:
            return None

        a = max(0, min(int(a), n - 1))
        b = max(0, min(int(b), n - 1))
        if b < a:
            return None

        pa = PoseArray()
        pa.header.frame_id = str(self._frame or "scene")

        for i in range(a, b + 1):
            x, y, z = self._pts_mm[i]
            p = Pose()
            p.position.x = float(x) * 1e-3
            p.position.y = float(y) * 1e-3
            p.position.z = float(z) * 1e-3
            p.orientation.w = 1.0
            pa.poses.append(p)

        if len(pa.poses) < 2:
            return None
        return pa

    def _on_enter_segment(self, seg_name: str) -> None:
        n = len(self._pts_mm)

        if seg_name == STATE_MOVE_HOME:
            ok = self._call_first(["moveit_home", "moveit_go_home"])
            if not ok:
                QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        if seg_name == STATE_MOVE_PREDISPENSE:
            pa = self._pose_array_slice(0, 1)
        elif seg_name == STATE_MOVE_RECIPE:
            if n < 3:
                QtCore.QTimer.singleShot(0, self._sig_done.emit)
                return
            pa = self._pose_array_slice(1, n - 2)
        elif seg_name == STATE_MOVE_RETREAT:
            pa = self._pose_array_slice(max(n - 2, 0), n - 1)
        else:
            pa = None

        if pa is None:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # execute: plan+execute oder execute-only, je nach ROS API
        ok = self._call_first(
            [
                "moveit_plan_execute_pose_array",
                "moveit_plan_execute_poses",
                "moveit_execute_pose_array",
                "moveit_execute_poses",
            ],
            pa,
        )
        if not ok:
            self._signal_error(f"Execute: ROS API fehlt für Segment {seg_name}.")

    # ------------------------------------------------------------------
    # Planned/Executed payload override
    # ------------------------------------------------------------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        planned:
          - primarily from loaded recipe.planned_traj (planned_traj.yaml)
          - fallback: base captured planned (if any)
        executed:
          - base captured executed
        """
        executed = self._jt_by_segment_yaml(which="executed")

        planned = {}
        if isinstance(self._planned_loaded_yaml, dict) and self._planned_loaded_yaml:
            planned = copy.deepcopy(self._planned_loaded_yaml)
        else:
            planned = self._jt_by_segment_yaml(which="planned")

        return planned, executed

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()

        # Guard: executed must exist (this is the whole point of Execute)
        e_segs = executed.get("segments") if isinstance(executed, dict) else None
        if not (isinstance(e_segs, dict) and e_segs):
            self.notifyError.emit(
                "Execute finished, but executed trajectory capture is empty (segments={}). "
                "Check MoveItPyBridge executedTrajectoryChanged / ros.moveit_executed_trajectory() accessors."
            )
            self._cleanup()
            return

        rr = RunResult()
        rr.set_planned(traj=planned)     # baseline from loaded artifact (best effort)
        rr.set_executed(traj=executed)  # captured truth

        self.notifyFinished.emit(rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # Compiled points extraction (robust for ndarray OR list)
    # ------------------------------------------------------------------

    def _get_compiled_points_mm(self, side: str) -> List[Tuple[float, float, float]]:
        """
        Accepts:
          - numpy ndarray (has .tolist())
          - list/tuple of rows (already list)
          - any iterable of rows

        Expects rows shaped like [x,y,z] in mm (as produced by Recipe.draft_poses_quat()).
        """
        out: List[Tuple[float, float, float]] = []
        try:
            pts = self._recipe.compiled_points_mm_for_side(str(side))
        except Exception:
            pts = None

        if pts is None:
            return out

        rows: Iterable[Any]
        if hasattr(pts, "tolist"):
            try:
                rows = pts.tolist()
            except Exception:
                rows = pts
        else:
            rows = pts

        try:
            for row in rows:
                if row is None:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    z = float(row[2])
                except Exception:
                    continue
                out.append((x, y, z))
        except Exception:
            return out

        return out
