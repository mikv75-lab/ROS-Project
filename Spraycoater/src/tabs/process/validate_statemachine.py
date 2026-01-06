# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from PyQt6 import QtCore

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process.validate_statemachine")


class ProcessValidateStatemachine(BaseProcessStatemachine):
    """
    Validate:
      - nutzt compiled path (mm) aus Recipe.paths_compiled
      - segmentiert in: predispense (erster Punkt), recipe (mittlere Punkte, gestreamt), retreat (letzter Punkt), home
      - Transition-Logik läuft über BaseProcessStatemachine (motionResultChanged: "EXECUTED:OK")
    """

    ROLE = "validate"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
        side: str = "top",
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=max_retries, skip_home=skip_home)

        self._side: str = str(side or "top")

        # compiled path points (mm)
        self._compiled_pts_mm: List[Tuple[float, float, float]] = []

        # segment points (mm)
        self._pts_by_seg: Dict[str, List[Tuple[float, float, float]]] = {}

        # streaming queue for MOVE_RECIPE
        self._pending_recipe_mm: List[Tuple[float, float, float]] = []

        # move meta (optional)
        self._frame_id: str = ""
        self._tool_frame_id: str = ""
        self._vel_scale: float = 1.0

    # ---------------- Hooks (Base) ----------------

    def _prepare_run(self) -> bool:
        # optional meta (falls vorhanden)
        self._frame_id = getattr(self._recipe, "frame_id", "") or getattr(self._recipe, "frame", "") or ""
        self._tool_frame_id = getattr(self._recipe, "tool_frame_id", "") or getattr(self._recipe, "tool_frame", "") or ""
        self._vel_scale = float(getattr(self._recipe, "vel_scale", 1.0) or 1.0)

        self._compiled_pts_mm = self._get_compiled_points_mm(self._side)
        if not self._compiled_pts_mm:
            self._error_msg = f"Validate: compiled path ist leer (side='{self._side}')."
            return False

        pts = self._compiled_pts_mm
        n = len(pts)

        pre = [pts[0]] if n >= 1 else []
        ret = [pts[-1]] if n >= 2 else []
        mid = pts[1:-1] if n >= 3 else []

        self._pts_by_seg = {
            STATE_MOVE_PREDISPENSE: pre,
            STATE_MOVE_RECIPE: list(mid),
            STATE_MOVE_RETREAT: ret,
            STATE_MOVE_HOME: [],
        }

        self._pending_recipe_mm = list(self._pts_by_seg[STATE_MOVE_RECIPE])
        return True

    def _segment_exists(self, seg_name: str) -> bool:
        # Base kümmert sich um skip_home; wir erweitern nur um "leere Segmente skippen"
        if self._skip_home and seg_name == STATE_MOVE_HOME:
            return False

        if seg_name == STATE_MOVE_RECIPE:
            # Segment existiert nur, wenn es wirklich Punkte zu streamen gibt
            return bool(self._pending_recipe_mm)

        return bool(self._pts_by_seg.get(seg_name))

    def _on_enter_segment(self, seg_name: str) -> None:
        if seg_name == STATE_MOVE_PREDISPENSE:
            self._run_single_point(seg_name)
            return

        if seg_name == STATE_MOVE_RECIPE:
            # Wenn keine Punkte vorhanden: Segment sofort als done markieren (Base -> next state)
            if not self._pending_recipe_mm:
                QtCore.QTimer.singleShot(0, self._sig_done.emit)
                return
            self._send_next_recipe_point()
            return

        if seg_name == STATE_MOVE_RETREAT:
            self._run_single_point(seg_name)
            return

        if seg_name == STATE_MOVE_HOME:
            # Home wird typischerweise als "named target" o.Ä. in ros implementiert.
            # Falls dein ros-Wrapper einen anderen Namen hat, hier anpassen.
            try:
                self._ros.moveit_home()
            except Exception:
                # wenn nicht vorhanden, segment sofort durchlaufen
                QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        self._signal_error(f"Validate: Unknown segment '{seg_name}'")

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        # MOVE_RECIPE: im selben Segment bleiben, bis alle Punkte gesendet wurden
        if seg_name == STATE_MOVE_RECIPE:
            if self._pending_recipe_mm:
                self._send_next_recipe_point()
                return False
            return True
        return True

    # ---------------- Internals ----------------

    def _run_single_point(self, seg_name: str) -> None:
        pts = self._pts_by_seg.get(seg_name) or []
        if not pts:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        x, y, z = pts[0]
        self._move_pose_mm(x, y, z)

    def _send_next_recipe_point(self) -> None:
        if not self._pending_recipe_mm:
            return
        x, y, z = self._pending_recipe_mm.pop(0)
        self._move_pose_mm(x, y, z)

    def _move_pose_mm(self, x: float, y: float, z: float) -> None:
        # Du hast in deinem Log: STOP:REQ / ModeManager / EXECUTED:OK, d.h. ros.move_pose_mm triggert MoveItPy.
        # Base hört auf motionResultChanged und macht die Transitions.
        self._ros.move_pose_mm(
            x=float(x),
            y=float(y),
            z=float(z),
            side=self._side,
            frame_id=self._frame_id,
            tool_frame_id=self._tool_frame_id,
            vel_scale=float(self._vel_scale),
        )

    def _get_compiled_points_mm(self, side: str) -> List[Tuple[float, float, float]]:
        # Recipe.compiled_points_mm_for_side(side) -> np.ndarray (N,3) float32
        pts = self._recipe.compiled_points_mm_for_side(side)
        pts = np.asarray(pts, dtype=float).reshape(-1, 3)
        return [tuple(row) for row in pts]
