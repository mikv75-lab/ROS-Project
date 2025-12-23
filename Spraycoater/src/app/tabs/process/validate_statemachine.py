# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, List

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped, PoseArray

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("app.tabs.process.validate_statemachine")


class ProcessValidateStatemachine(BaseProcessStatemachine):
    """
    Validate:
      - nutzt aktuelle Rezeptstrecke (PoseArray) aus SprayPath-State
      - fährt pose-by-pose via MoveItPy
      - MOVE_RECIPE konsumiert viele EXECUTED:OK bis alle Zwischenpunkte abgefahren sind
    """

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 3,
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=max_retries, skip_home=False)
        self._poses: List[PoseStamped] = []
        self._idx: int = 0  # aktueller Pose-Index innerhalb RECIPE (1..n-2)

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        pa: Optional[PoseArray] = None
        try:
            pa = self._ros.spraypath_state.poses() if hasattr(self._ros, "spraypath_state") else None
        except Exception:
            pa = None

        if pa is None:
            try:
                pa = self._ros.spraypath.poses()
            except Exception:
                pa = None

        if pa is None or not getattr(pa, "poses", None):
            self._error_msg = "Validate: Keine Rezept-Posen vorhanden."
            return False

        frame = pa.header.frame_id or "scene"
        self._poses = []
        for p in pa.poses:
            ps = PoseStamped()
            ps.header.frame_id = frame
            ps.pose = p
            self._poses.append(ps)

        if len(self._poses) < 2:
            self._error_msg = "Validate: Zu wenige Posen."
            return False

        self._idx = 0
        return True

    # ------------------------------------------------------------------
    # Segment start
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        if seg_name == STATE_MOVE_PREDISPENSE:
            self._idx = 0
            self._ros.moveit_move_to_pose(self._poses[0])
            return

        if seg_name == STATE_MOVE_RECIPE:
            # Zwischenpunkte 1..n-2 (ohne predispense=0 und retreat=n-1)
            self._idx = 1
            self._move_next_recipe_pose()
            return

        if seg_name == STATE_MOVE_RETREAT:
            self._ros.moveit_move_to_pose(self._poses[-1])
            return

        if seg_name == STATE_MOVE_HOME:
            self._ros.moveit_move_home()
            return

        self._signal_error(f"Validate: Unbekanntes Segment: {seg_name}")

    def _move_next_recipe_pose(self) -> None:
        if self._stop_requested:
            self._signal_error("Validate: gestoppt.")
            return
        # nur Zwischenpunkte fahren: idx in [1 .. len-2]
        if self._idx < 1 or self._idx >= (len(self._poses) - 1):
            return
        self._ros.moveit_move_to_pose(self._poses[self._idx])

    # ------------------------------------------------------------------
    # Motion result handling (override!)
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot(str)
    def _on_motion_result(self, result: str) -> None:
        """
        Überschreibt Base, weil Base nach jedem OK automatisch _sig_done emittiert.
        Für MOVE_RECIPE dürfen wir aber erst "done" senden, wenn alle Zwischenpunkte abgefahren sind.
        """
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested or not self._current_state:
            return

        _LOG.info("Validate: motion_result=%s (state=%s, idx=%d)", result, self._current_state, self._idx)

        # ---------- ERROR / retry ----------
        if result.startswith("ERROR"):
            if self._retry_count < self._max_retries:
                self._retry_count += 1
                _LOG.warning(
                    "Validate: Retry %d/%d für %s (idx=%d)",
                    self._retry_count, self._max_retries, self._current_state, self._idx
                )
                # retry current segment (bei RECIPE bleibt idx unverändert -> gleicher Pose erneut)
                self._on_enter_segment(self._current_state)
            else:
                self._signal_error(result)
            return

        # ---------- ignore non-final statuses ----------
        if not result.startswith("EXECUTED:OK"):
            return

        # ---------- SPECIAL: MOVE_RECIPE consumes many OK ----------
        if self._current_state == STATE_MOVE_RECIPE:
            # OK für aktuellen Zwischenpunkt -> nächsten starten oder fertig
            self._idx += 1

            # Ende erreicht? (wir fahren 1..n-2) => nach OK für (n-2) ist idx == (n-1)
            if self._idx >= (len(self._poses) - 1):
                # Segment fertig -> erst jetzt Transition erlauben
                QtCore.QTimer.singleShot(0, self._sig_done.emit)
                return

            # Noch Zwischenpunkte offen -> nächsten Pose starten und NICHT _sig_done emittieren
            self._retry_count = 0
            self._move_next_recipe_pose()
            return

        # ---------- Default: wie Base ----------
        self._on_segment_ok(self._current_state)
        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    # ------------------------------------------------------------------
    # Result
    # ------------------------------------------------------------------

    def _build_result(self):
        out = super()._build_result()
        out["mode"] = "validate"
        return out
