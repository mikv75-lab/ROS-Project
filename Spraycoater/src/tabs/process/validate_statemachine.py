# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, List, Tuple

from PyQt6 import QtCore

from model.recipe.recipe_run_result import RunResult

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
    Validate run: streams recipe points (Position + Orientation) as consecutive MoveIt pose commands.

    STRICT (frame):
      - HARD-CODE frame to 'substrate' for now.
      - No 'scene' frame usage.

    Retry (STRICT):
      - Base retries ONLY the last motion command on soft errors (ERROR:EXEC...).
      - This class implements _on_retry_last_motion() to re-send the in-flight pose.
      - No segment replay. No rebuilding pending queues on retry.
    """

    ROLE = "validate"

    _NEXT_POSE_QT_DELAY_MS = 0
    _JOINT_UPDATE_FALLBACK_MS = 200

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 3,
        skip_home: bool = False,
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            run_result=run_result,
            parent=parent,
            max_retries=max_retries,
            skip_home=skip_home,
        )

        self._side = str(side or "top")

        # Compiled poses: tuples of 7 floats: (x, y, z, qx, qy, qz, qw) in mm + quat
        self._compiled_poses: List[Tuple[float, float, float, float, float, float, float]] = []
        self._cmd_poses_by_seg: Dict[str, List[Tuple[float, ...]]] = {}
        self._pending_recipe_poses: List[Tuple[float, ...]] = []

        # In-flight pose (last sent; waiting for EXECUTED:OK)
        self._inflight_pose: Optional[Tuple[float, ...]] = None
        self._inflight_seg: str = ""

        # HARD-CODED (requested): always publish PoseStamped in 'substrate'
        self._frame: str = "substrate"
        self._speed_mm_s: float = 200.0

        # Planner config
        self._planner_cfg: Dict[str, Any] = {}

        # gating for recipe streaming
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
    # STOP OVERRIDE
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._await_joint_update_for_next_pose = False
        self._pending_recipe_poses.clear()
        self._inflight_pose = None
        self._inflight_seg = ""

        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        super().request_stop()

    # ------------------------------------------------------------------
    # Retry hook (Base calls this on soft errors)
    # ------------------------------------------------------------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        """
        Re-send ONLY the last in-flight pose for the current segment.
        IMPORTANT: Do not touch _pending_recipe_poses (no segment replay).
        """
        if self._stop_requested:
            return False
        if not self._machine or not self._machine.isRunning():
            return False
        if seg_name != self._current_state:
            return False

        pose = self._inflight_pose
        if pose is None:
            return False

        # On retry, do not auto-advance; we are waiting for the retried motion result.
        self._await_joint_update_for_next_pose = False
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        _LOG.warning(
            "Validate: retry last motion attempt %d for seg=%s (pose still in-flight). reason=%s",
            int(attempt),
            str(seg_name),
            str(last_error),
        )

        self._ros_move_pose_tuple(pose)
        return True

    # ------------------------------------------------------------------
    # Hooks
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # 1) Frame: HARD-CODED
        self._frame = "substrate"

        # 2) Speed
        self._speed_mm_s = 200.0
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            pass

        # 3) PLANNER CONFIG
        if not self._apply_planner_config():
            return False

        # 4) Compiled poses (Position + Orientation)
        self._compiled_poses = self._get_compiled_poses(self._side)

        # reset segment plans
        self._cmd_poses_by_seg.clear()
        self._pending_recipe_poses.clear()
        self._await_joint_update_for_next_pose = False
        self._inflight_pose = None
        self._inflight_seg = ""
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        # ensure Base buffers are clean
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()

        if not self._compiled_poses:
            self._signal_error(f"Validate: compiled path ist leer (side='{self._side}').")
            return False

        pts = self._compiled_poses
        n = len(pts)

        pre = [pts[0]] if n >= 1 else []
        ret = [pts[-1]] if n >= 2 else []
        mid = pts[1:-1] if n >= 3 else []

        self._cmd_poses_by_seg[STATE_MOVE_PREDISPENSE] = pre
        self._cmd_poses_by_seg[STATE_MOVE_RECIPE] = list(mid)
        self._cmd_poses_by_seg[STATE_MOVE_RETREAT] = ret
        self._cmd_poses_by_seg[STATE_MOVE_HOME] = []

        # recipe queue starts fresh on segment entry; we DO NOT rebuild it on retry
        self._pending_recipe_poses = list(self._cmd_poses_by_seg[STATE_MOVE_RECIPE])
        return True

    def _apply_planner_config(self) -> bool:
        try:
            planner_sect = getattr(self._recipe, "planner", {}) or {}
        except Exception:
            planner_sect = {}

        cfg = planner_sect.get("validate")
        if not isinstance(cfg, dict):
            cfg = planner_sect.get("move")

        if not isinstance(cfg, dict):
            _LOG.warning("Validate: Keine Planner-Config gefunden. Nutze Default.")
            return True

        pipeline = str(cfg.get("pipeline") or "").strip()
        planner_id = str(cfg.get("planner_id") or "").strip()
        params = cfg.get("params") or {}

        if not planner_id:
            _LOG.warning("Validate: Planner config gefunden, aber planner_id leer.")
            return True

        self._planner_cfg = {
            "role": "validate",
            "pipeline": pipeline,
            "planner_id": planner_id,
            "params": params,
        }

        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp is not None else None
            if sig is not None and hasattr(sig, "plannerCfgChanged"):
                _LOG.info("Validate: Setze Planner '%s/%s'", pipeline, planner_id)
                sig.plannerCfgChanged.emit(self._planner_cfg)
            else:
                _LOG.warning("Validate: MoveItPyBridge hat kein plannerCfgChanged Signal.")
        except Exception as e:
            _LOG.error("Validate: Fehler beim Senden der Planner-Config: %s", e)
            return False

        return True

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and self._skip_home:
            return False
        if seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT):
            return bool(self._cmd_poses_by_seg.get(seg_name))
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        self._await_joint_update_for_next_pose = False
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        # entering a new segment resets in-flight (segment-local)
        self._inflight_pose = None
        self._inflight_seg = ""

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
        """
        Called by Base after EXECUTED:OK for this segment.
        For MOVE_RECIPE we advance the queue by exactly ONE pose (the in-flight pose),
        then either wait for jointsChanged and send the next, or transition to next segment.
        """
        # Clear in-flight on OK (for any segment)
        if self._inflight_seg == seg_name:
            if seg_name == STATE_MOVE_RECIPE:
                if self._inflight_pose is not None and self._pending_recipe_poses:
                    # Queue head must match in-flight pose (strict)
                    if self._pending_recipe_poses[0] == self._inflight_pose:
                        self._pending_recipe_poses.pop(0)
                    else:
                        self._signal_error(
                            "Validate: queue/inflight mismatch in MOVE_RECIPE (refusing to continue)."
                        )
                        return False
            self._inflight_pose = None
            self._inflight_seg = ""

        if seg_name == STATE_MOVE_RECIPE:
            if self._pending_recipe_poses:
                # Wait for joints update (or fallback timer) before sending next pose
                self._await_joint_update_for_next_pose = True
                self._arm_joint_fallback_timer()
                return False
            return True

        return True

    # ------------------------------------------------------------------
    # Segment runners
    # ------------------------------------------------------------------

    def _run_single_pose_segment(self, seg_name: str) -> None:
        poses = self._cmd_poses_by_seg.get(seg_name) or []
        if not poses:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        pose = poses[0]
        self._inflight_pose = pose
        self._inflight_seg = seg_name
        self._ros_move_pose_tuple(pose)

    def _run_recipe_stream_segment(self) -> None:
        if not self._pending_recipe_poses:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        self._send_next_recipe_pose()

    def _send_next_recipe_pose(self) -> None:
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass
        self._await_joint_update_for_next_pose = False

        if not self._pending_recipe_poses:
            return

        # STRICT: do not pop here; pop only after EXECUTED:OK
        pose_tuple = self._pending_recipe_poses[0]
        self._inflight_pose = pose_tuple
        self._inflight_seg = STATE_MOVE_RECIPE
        self._ros_move_pose_tuple(pose_tuple)

    def _run_home_segment(self) -> None:
        if self._skip_home:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        try:
            self._ros.moveitpy.signals.moveToHomeRequestedWithSpeed.emit(float(self._speed_mm_s))
            self._inflight_pose = None
            self._inflight_seg = STATE_MOVE_HOME
        except Exception as ex:
            self._signal_error(f"Validate: move_home failed: {ex}")

    # ------------------------------------------------------------------
    # Robot state gating
    # ------------------------------------------------------------------

    def _arm_joint_fallback_timer(self) -> None:
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass
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
        if not self._pending_recipe_poses:
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
        if not self._pending_recipe_poses:
            return

        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass
        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    # ------------------------------------------------------------------
    # ROS facade helpers
    # ------------------------------------------------------------------

    def _ros_move_pose_tuple(self, p: Tuple[float, ...]) -> None:
        """
        Sends move command.
        Expects tuple (x, y, z, qx, qy, qz, qw)
        """
        if len(p) < 7:
            self._signal_error(f"Validate: Invalid pose tuple len={len(p)}")
            return

        x, y, z = p[0], p[1], p[2]
        qx, qy, qz, qw = p[3], p[4], p[5], p[6]

        from geometry_msgs.msg import PoseStamped  # type: ignore

        try:
            try:
                self._ros.moveitpy.signals.motionSpeedChanged.emit(float(self._speed_mm_s))
            except Exception:
                pass

            ps = PoseStamped()
            ps.header.frame_id = str(self._frame or "substrate")

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
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)

            _LOG.info(
                "Validate: Moving to %s: Pos=%.1f/%.1f/%.1f Orient=%.3f/%.3f/%.3f/%.3f",
                ps.header.frame_id,
                float(x),
                float(y),
                float(z),
                float(qx),
                float(qy),
                float(qz),
                float(qw),
            )

            self._ros.moveitpy.signals.moveToPoseRequested.emit(ps)
        except Exception as ex:
            self._signal_error(f"Validate: move_pose failed ({x:.1f},{y:.1f},{z:.1f}): {ex}")

    # ------------------------------------------------------------------
    # Compiled poses extraction (FULL POSE: Pos + Rot)
    # ------------------------------------------------------------------

    def _get_compiled_poses(self, side: str) -> List[Tuple[float, float, float, float, float, float, float]]:
        """
        Returns list of (x, y, z, qx, qy, qz, qw).
        Coordinates x,y,z in mm.
        """
        out: List[Tuple[float, float, float, float, float, float, float]] = []
        try:
            poses = self._recipe.draft_poses_quat(str(side))
        except Exception:
            return []

        if not poses:
            return []

        for p in poses:
            try:
                if hasattr(p, "x"):
                    val = (
                        float(p.x),
                        float(p.y),
                        float(p.z),
                        float(p.qx),
                        float(p.qy),
                        float(p.qz),
                        float(p.qw),
                    )
                elif isinstance(p, dict):
                    val = (
                        float(p["x"]),
                        float(p["y"]),
                        float(p["z"]),
                        float(p["qx"]),
                        float(p["qy"]),
                        float(p["qz"]),
                        float(p["qw"]),
                    )
                else:
                    continue
                out.append(val)
            except Exception:
                continue

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
