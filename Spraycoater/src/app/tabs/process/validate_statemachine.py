# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Optional, List

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped, PoseArray

from .base_statemachine import BaseProcessStatemachine, STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME

_LOG = logging.getLogger("app.tabs.process.validate_statemachine")


class ProcessValidateStatemachine(BaseProcessStatemachine):
    """
    Validate:
      - nutzt Rezeptstrecke (PoseArray) und fährt Pose-by-Pose via MoveItPy
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
        self._idx: int = 0

    def _prepare_run(self) -> bool:
        # Stelle sicher: spraypath zeigt Rezept (kann ProcessThread schon vorher pushen)
        pa: Optional[PoseArray] = None
        try:
            pa = self._ros.spraypath_state.poses() if hasattr(self._ros, "spraypath_state") else None
        except Exception:
            pa = None

        # Fallback: direkt über Bridge-State (bei dir: ros.spraypath.poses())
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
            ps.pose = p  # Pose kopieren
            self._poses.append(ps)

        if len(self._poses) < 2:
            self._error_msg = "Validate: Zu wenige Posen."
            return False

        self._idx = 0
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        if seg_name == STATE_MOVE_PREDISPENSE:
            self._idx = 0
            self._ros.moveit_move_to_pose(self._poses[0])
            return

        if seg_name == STATE_MOVE_RECIPE:
            # fahre alle Zwischenpunkte (1..n-2)
            self._idx = 1
            self._move_next_recipe_pose()
            return

        if seg_name == STATE_MOVE_RETREAT:
            self._ros.moveit_move_to_pose(self._poses[-1])
            return

        if seg_name == STATE_MOVE_HOME:
            self._ros.moveit_move_home()
            return

    def _on_segment_ok(self, seg_name: str) -> None:
        # Für RECIPE schalten wir nicht per motion_result automatisch weiter,
        # sondern “konsumieren” viele OKs bis alle Zwischenpunkte gefahren sind.
        if seg_name != STATE_MOVE_RECIPE:
            return

        self._idx += 1
        # Ende erreicht? (wir fahren 1..n-2)
        if self._idx >= len(self._poses) - 1:
            # Segment fertig → Base darf in next state
            return

        # Noch Punkte offen → direkt nächsten fahren, Base soll NICHT weitertransitionen
        # Trick: wir starten sofort den nächsten Pose und verhindern "done" durch einen kleinen Hack:
        # Base ruft _sig_done nach EXECUTED:OK immer; wir “re-arm” hier indem wir den nächsten schon starten.
        self._move_next_recipe_pose()

    def _move_next_recipe_pose(self) -> None:
        if self._stop_requested:
            self._signal_error("Validate: gestoppt.")
            return
        if self._idx >= len(self._poses) - 1:
            return
        self._ros.moveit_move_to_pose(self._poses[self._idx])

    def _build_result(self):
        out = super()._build_result()
        out["mode"] = "validate"
        return out
