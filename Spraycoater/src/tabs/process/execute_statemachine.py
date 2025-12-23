# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional

from PyQt6 import QtCore
from geometry_msgs.msg import PoseArray, Pose

from plc.plc_client import PlcClientBase
from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process.execute_sm")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    ROLE = "execute"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        plc: PlcClientBase,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=max_retries, skip_home=False)
        self._plc = plc

        self._side: str = "top"
        self._frame: str = "scene"
        self._pts_mm = None

    def _prepare_run(self) -> bool:
        # Optional: PLC checks hier
        try:
            try:
                self._side = str(getattr(self._recipe, "parameters", {}).get("active_side", "top"))
            except Exception:
                self._side = "top"

            try:
                self._frame = str((getattr(self._recipe, "paths_compiled", {}) or {}).get("frame") or "scene")
            except Exception:
                self._frame = "scene"

            if not hasattr(self._recipe, "compiled_points_mm_for_side"):
                self._error_msg = "Execute: Recipe.compiled_points_mm_for_side fehlt."
                return False

            self._pts_mm = self._recipe.compiled_points_mm_for_side(self._side)
            if self._pts_mm is None or getattr(self._pts_mm, "size", 0) == 0:
                self._error_msg = f"Execute: Keine compiled points für side='{self._side}'."
                return False

            if int(self._pts_mm.shape[0]) < 2:
                self._error_msg = "Execute: Zu wenige Punkte."
                return False

            return True
        except Exception as e:
            self._error_msg = f"Execute: Prepare failed: {e}"
            return False

    def _call_first(self, names: list[str], *args, **kwargs) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                fn(*args, **kwargs)
                return True
        return False

    def _pose_array_slice(self, a: int, b: int) -> Optional[PoseArray]:
        n = int(self._pts_mm.shape[0])
        a = max(0, min(a, n - 1))
        b = max(0, min(b, n - 1))
        if b < a:
            return None

        pa = PoseArray()
        pa.header.frame_id = self._frame

        for i in range(a, b + 1):
            x, y, z = float(self._pts_mm[i, 0]), float(self._pts_mm[i, 1]), float(self._pts_mm[i, 2])
            p = Pose()
            p.position.x = x * 1e-3
            p.position.y = y * 1e-3
            p.position.z = z * 1e-3
            p.orientation.w = 1.0
            pa.poses.append(p)

        if len(pa.poses) < 2:
            return None
        return pa

    def _on_enter_segment(self, seg_name: str) -> None:
        n = int(self._pts_mm.shape[0])

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

        # execute: plan+execute oder execute-only, je nach deiner ROS API
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
