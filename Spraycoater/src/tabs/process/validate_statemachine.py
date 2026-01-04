# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, List, Tuple

from PyQt6 import QtCore

from geometry_msgs.msg import PoseStamped

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process.validate_sm")


class ProcessValidateStatemachine(BaseProcessStatemachine):
    """
    Validate:
      - nimmt compiled points (mm) aus Recipe
      - fährt Segmente (predispense/recipe/retreat/home)
      - nutzt die "neuen" RosBridge / MoveItPySignals Calls:
          - moveit_move_home()
          - moveitpy.signals.moveToPoseRequested(PoseStamped)
      - Segment-Queue: Pose für Pose, Transition erst nach letzter Pose im Segment.
    """

    ROLE = "validate"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 3,
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=max_retries, skip_home=False)

        self._side: str = "top"
        self._frame: str = "scene"
        self._pts_mm = None  # numpy array

        # --- per segment pose queue ---
        self._pending_poses: List[PoseStamped] = []
        self._pending_seg: str = ""
        self._pending_idx: int = 0

    def _prepare_run(self) -> bool:
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
                self._error_msg = "Validate: Recipe.compiled_points_mm_for_side fehlt."
                return False

            self._pts_mm = self._recipe.compiled_points_mm_for_side(self._side)
            if self._pts_mm is None or getattr(self._pts_mm, "size", 0) == 0:
                self._error_msg = f"Validate: Keine compiled points für side='{self._side}'."
                return False

            if int(self._pts_mm.shape[0]) < 2:
                self._error_msg = "Validate: Zu wenige Punkte."
                return False

            # reset queue
            self._pending_poses = []
            self._pending_seg = ""
            self._pending_idx = 0

            return True
        except Exception as e:
            self._error_msg = f"Validate: Prepare failed: {e}"
            return False

    def _call_first(self, names: list[str], *args, **kwargs) -> bool:
        for n in names:
            fn = getattr(self._ros, n, None)
            if callable(fn):
                fn(*args, **kwargs)
                return True
        return False

    def _build_pose(self, xyz_mm: Tuple[float, float, float]) -> PoseStamped:
        """compiled_points_mm_for_side liefert mm; MoveItPy erwartet Meter."""
        x_mm, y_mm, z_mm = xyz_mm
        ps = PoseStamped()
        ps.header.frame_id = self._frame

        ps.pose.position.x = float(x_mm) * 1e-3
        ps.pose.position.y = float(y_mm) * 1e-3
        ps.pose.position.z = float(z_mm) * 1e-3

        # TODO: Orientation aus normals/tangents übernehmen, sobald verfügbar.
        ps.pose.orientation.w = 1.0
        return ps

    def _segment_indices(self, seg_name: str, n: int) -> Optional[Tuple[int, int]]:
        """
        Liefert [a,b] inklusive, im Sinne deines bisherigen Slicings:
          - predispense: 0..1
          - recipe:      1..n-2
          - retreat:     n-2..n-1
        """
        if n < 2:
            return None

        if seg_name == STATE_MOVE_PREDISPENSE:
            return (0, 1)

        if seg_name == STATE_MOVE_RECIPE:
            if n < 3:
                return None
            return (1, n - 2)

        if seg_name == STATE_MOVE_RETREAT:
            return (max(n - 2, 0), n - 1)

        return None

    def _start_pose_queue_for_segment(self, seg_name: str, a: int, b: int) -> None:
        """Initialisiert Queue und sendet die erste Pose."""
        self._pending_seg = seg_name
        self._pending_idx = 0
        self._pending_poses = []

        for i in range(a, b + 1):
            xyz = (float(self._pts_mm[i, 0]), float(self._pts_mm[i, 1]), float(self._pts_mm[i, 2]))
            self._pending_poses.append(self._build_pose(xyz))

        if not self._pending_poses:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        self._emit_pending_pose()

    def _emit_pending_pose(self) -> None:
        """Sendet die aktuelle Pose aus der Queue via MoveItPySignals."""
        if not self._pending_poses or self._pending_idx < 0 or self._pending_idx >= len(self._pending_poses):
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        ps = self._pending_poses[self._pending_idx]

        sig = self._moveitpy_signals
        if sig is None or not hasattr(sig, "moveToPoseRequested"):
            self._signal_error(f"Validate: MoveItPySignals.moveToPoseRequested fehlt (seg={self._pending_seg}).")
            return

        # Emit queued (thread-safe enough; signal crosses threads)
        try:
            sig.moveToPoseRequested.emit(ps)
        except Exception as e:
            self._signal_error(f"Validate: moveToPoseRequested emit failed: {e}")

    def _on_enter_segment(self, seg_name: str) -> None:
        n = int(self._pts_mm.shape[0])

        # HOME: neuer RosBridge Call
        if seg_name == STATE_MOVE_HOME:
            ok = self._call_first(["moveit_move_home", "moveit_home", "moveit_go_home"])
            if not ok:
                # Wenn Home nicht verfügbar ist, segment als "done" behandeln
                QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        idx = self._segment_indices(seg_name, n)
        if idx is None:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        a, b = idx
        if b < a:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # Segment queue (Pose für Pose)
        self._start_pose_queue_for_segment(seg_name, a, b)

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        """
        Wird bei 'EXECUTED:OK' aufgerufen.
        Wenn wir noch Posen im aktuellen Segment haben -> nächste Pose senden und NICHT transitionen.
        """
        if seg_name != self._pending_seg or not self._pending_poses:
            return True

        # current pose finished -> next?
        if self._pending_idx < len(self._pending_poses) - 1:
            self._pending_idx += 1
            QtCore.QTimer.singleShot(0, self._emit_pending_pose)
            return False

        # last pose done -> clear queue and allow transition
        self._pending_poses = []
        self._pending_seg = ""
        self._pending_idx = 0
        return True
