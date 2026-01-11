# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, List, Tuple, Iterable

from PyQt6 import QtCore

from model.recipe.recipe_run_result import RunResult  # NEW: seeded RunResult comes from ProcessThread

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
    Validate run: streams recipe points as consecutive MoveIt pose commands.

    IMPORTANT:
      - Trajectory capture is handled by BaseProcessStatemachine on each "EXECUTED:OK".
      - Base notifyFinished emits STRICT RunResult payload (NOT traj-only):
          {
            "planned_run":  {"traj": <JTBySegment v1 yaml dict>, "tcp": <Draft v1 yaml dict or {}>},
            "executed_run": {"traj": <JTBySegment v1 yaml dict>, "tcp": <Draft v1 yaml dict or {}>},
            "fk_meta": {...},
            "eval": {...},
            "valid": bool,
            "invalid_reason": str,
            "urdf_xml": str,
            "srdf_xml": str,
          }

        For Validate, "tcp" is intentionally left {} here; it is computed later
        (e.g. in ProcessTab via TrajFkBuilder) to ensure reproducibility/compare.
    """

    ROLE = "validate"

    _NEXT_POSE_QT_DELAY_MS = 0
    _JOINT_UPDATE_FALLBACK_MS = 200

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,  # NEW
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            run_result=run_result,  # NEW
            parent=parent,
            max_retries=max_retries,
            skip_home=skip_home,
        )

        self._side = str(side or "top")

        self._compiled_pts_mm: List[Tuple[float, float, float]] = []
        self._cmd_pts_by_seg: Dict[str, List[Tuple[float, float, float]]] = {}
        self._pending_recipe_mm: List[Tuple[float, float, float]] = []

        self._frame: str = "world"
        self._speed_mm_s: float = 200.0

        self._await_joint_update_for_next_pose: bool = False
        self._joint_sig_connected: bool = False

        # timers
        self._joint_fallback_timer = QtCore.QTimer(self)
        self._joint_fallback_timer.setSingleShot(True)
        self._joint_fallback_timer.timeout.connect(self._on_joint_fallback_timeout)

        # robot joints gating (recipe streaming)
        try:
            self._ros.robot.signals.jointsChanged.connect(self._on_joints_changed)
            self._joint_sig_connected = True
        except Exception:
            self._joint_sig_connected = False

    # ------------------------------------------------------------------
    # Hooks
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # frame
        self._frame = "scene"
        try:
            pc = getattr(self._recipe, "paths_compiled", None)
            if isinstance(pc, dict) and pc.get("frame"):
                self._frame = str(pc.get("frame"))
        except Exception:
            pass

        # speed
        self._speed_mm_s = 200.0
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            pass

        # compiled points (mm)
        self._compiled_pts_mm = self._get_compiled_points_mm(self._side)

        # reset segment plans
        self._cmd_pts_by_seg.clear()
        self._pending_recipe_mm.clear()
        self._await_joint_update_for_next_pose = False
        self._joint_fallback_timer.stop()

        # ensure Base buffers are clean (Base.start() already clears, but keep strictness here)
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()

        if not self._compiled_pts_mm:
            self._signal_error(f"Validate: compiled path ist leer (side='{self._side}').")
            return False

        pts = self._compiled_pts_mm
        n = len(pts)

        pre = [pts[0]] if n >= 1 else []
        ret = [pts[-1]] if n >= 2 else []
        mid = pts[1:-1] if n >= 3 else []

        self._cmd_pts_by_seg[STATE_MOVE_PREDISPENSE] = pre
        self._cmd_pts_by_seg[STATE_MOVE_RECIPE] = list(mid)
        self._cmd_pts_by_seg[STATE_MOVE_RETREAT] = ret
        self._cmd_pts_by_seg[STATE_MOVE_HOME] = []

        self._pending_recipe_mm = list(self._cmd_pts_by_seg[STATE_MOVE_RECIPE])
        return True

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and self._skip_home:
            return False
        if seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT):
            return bool(self._cmd_pts_by_seg.get(seg_name))
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        self._await_joint_update_for_next_pose = False
        self._joint_fallback_timer.stop()

        if seg_name == STATE_MOVE_PREDISPENSE:
            self._run_single_pose_segment(STATE_MOVE_PREDISPENSE)
            return
        if seg_name == STATE_MOVE_RECIPE:
            self._run_recipe_stream_segment()
            return
        if seg_name == STATE_MOVE_RETREAT:
            self._run_single_pose_segment(STATE_MOVE_RETREAT)
            return
        if seg_name == STATE_MOVE_HOME:
            self._run_home_segment()
            return

        self._signal_error(f"Validate: Unknown segment '{seg_name}'")

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        # RECIPE: stay in the same segment until we've issued all poses
        if seg_name == STATE_MOVE_RECIPE:
            if self._pending_recipe_mm:
                self._await_joint_update_for_next_pose = True
                self._arm_joint_fallback_timer()
                return False
            return True
        return True

    # ------------------------------------------------------------------
    # Segment runners
    # ------------------------------------------------------------------

    def _run_single_pose_segment(self, seg_name: str) -> None:
        pts = self._cmd_pts_by_seg.get(seg_name) or []
        if not pts:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        x, y, z = pts[0]
        self._ros_move_pose_mm(x, y, z)

    def _run_recipe_stream_segment(self) -> None:
        if not self._pending_recipe_mm:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        self._send_next_recipe_pose()

    def _send_next_recipe_pose(self) -> None:
        self._joint_fallback_timer.stop()
        self._await_joint_update_for_next_pose = False

        if not self._pending_recipe_mm:
            return
        x, y, z = self._pending_recipe_mm.pop(0)
        self._ros_move_pose_mm(x, y, z)

    def _run_home_segment(self) -> None:
        if self._skip_home:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        try:
            self._ros.moveitpy.signals.moveToHomeRequestedWithSpeed.emit(float(self._speed_mm_s))
        except Exception as ex:
            self._signal_error(f"Validate: move_home failed: {ex}")

    # ------------------------------------------------------------------
    # Robot state gating
    # ------------------------------------------------------------------

    def _arm_joint_fallback_timer(self) -> None:
        self._joint_fallback_timer.stop()
        ms = int(self._JOINT_UPDATE_FALLBACK_MS)
        if ms <= 0:
            ms = 1
        self._joint_fallback_timer.start(ms)

    @QtCore.pyqtSlot()
    def _on_joint_fallback_timeout(self) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if self._current_state != STATE_MOVE_RECIPE:
            return
        if not self._await_joint_update_for_next_pose:
            return
        if not self._pending_recipe_mm:
            return

        _LOG.warning(
            "Validate: jointsChanged fallback triggered (no joint update within %d ms). Proceeding with next recipe pose.",
            int(self._JOINT_UPDATE_FALLBACK_MS),
        )

        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    @QtCore.pyqtSlot(object)
    def _on_joints_changed(self, _msg: object) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if self._current_state != STATE_MOVE_RECIPE:
            return
        if not self._await_joint_update_for_next_pose:
            return
        if not self._pending_recipe_mm:
            return

        self._joint_fallback_timer.stop()
        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    # ------------------------------------------------------------------
    # ROS facade helpers
    # ------------------------------------------------------------------

    def _ros_move_pose_mm(self, x: float, y: float, z: float) -> None:
        from geometry_msgs.msg import PoseStamped  # type: ignore

        try:
            # optional: speed signal (if supported by MoveItPyBridge)
            try:
                self._ros.moveitpy.signals.motionSpeedChanged.emit(float(self._speed_mm_s))
            except Exception:
                pass

            ps = PoseStamped()
            ps.header.frame_id = str(self._frame or "world")

            try:
                node = (
                    getattr(self._ros, "node", None)
                    or getattr(self._ros, "_node", None)
                    or getattr(self._ros, "rclpy_node", None)
                )
                if node is not None:
                    ps.header.stamp = node.get_clock().now().to_msg()
            except Exception:
                pass

            # mm -> m
            ps.pose.position.x = float(x) / 1000.0
            ps.pose.position.y = float(y) / 1000.0
            ps.pose.position.z = float(z) / 1000.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            self._ros.moveitpy.signals.moveToPoseRequested.emit(ps)
        except Exception as ex:
            self._signal_error(f"Validate: move_pose failed ({x:.1f},{y:.1f},{z:.1f}): {ex}")

    # ------------------------------------------------------------------
    # Compiled points extraction (robust for ndarray OR list)
    # ------------------------------------------------------------------

    def _get_compiled_points_mm(self, side: str) -> List[Tuple[float, float, float]]:
        """
        Accepts:
          - numpy ndarray (has .tolist())
          - list/tuple of rows (already list)
          - any iterable of rows

        Expects rows shaped like [x,y,z] in mm.
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
                rows = pts.tolist()  # numpy-like
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

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        if self._joint_sig_connected:
            try:
                self._ros.robot.signals.jointsChanged.disconnect(self._on_joints_changed)
            except Exception:
                pass
            self._joint_sig_connected = False

        super()._cleanup()
