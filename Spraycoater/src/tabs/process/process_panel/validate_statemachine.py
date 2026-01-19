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

    IMPORTANT FIX (missing first leg / "wird gefahren aber nicht gespeichert"):
      - MOVE_RECIPE must NOT start with pts[0] (NO-OP), because predispense already moved to pts[0].
        NO-OP => often no trajectory => nothing to capture.
      - First MOVE_RECIPE command must be pts[1] so the first leg (pts[0]->pts[1]) is recorded.

      Therefore (compiled points list pts[]):
        MOVE_PREDISPENSE: [pts[0]]
        MOVE_RECIPE:      pts[1:-1]
        MOVE_RETREAT:     [pts[-1]]

    Trajectory capture is gated by BaseProcessStatemachine WAIT_RESULTS.
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
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            run_result=run_result,
            parent=parent,
            max_retries=max_retries,
        )

        self._side = str(side or "top")

        self._compiled_poses: List[Tuple[float, float, float, float, float, float, float]] = []
        self._cmd_poses_by_seg: Dict[str, List[Tuple[float, ...]]] = {}
        self._pending_recipe_poses: List[Tuple[float, ...]] = []

        self._inflight_pose: Optional[Tuple[float, ...]] = None
        self._inflight_seg: str = ""

        self._frame: str = "substrate"
        self._speed_mm_s: float = 200.0

        self._planner_cfg: Dict[str, Any] = {}

        self._await_joint_update_for_next_pose: bool = False
        self._joint_sig_connected: bool = False

        self._joint_fallback_timer = QtCore.QTimer(self)
        self._joint_fallback_timer.setSingleShot(True)
        self._joint_fallback_timer.timeout.connect(self._on_joint_fallback_timeout)

        try:
            self._ros.robot.signals.jointsChanged.connect(self._on_joints_changed)
            self._joint_sig_connected = True
        except Exception:
            self._joint_sig_connected = False

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

    # --------- Retry Hook (called by Base) ---------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        if self._stop_requested:
            return False
        if self._machine is None:
            return False
        if seg_name != self._current_state:
            return False

        pose = self._inflight_pose
        if pose is None:
            return False

        self._await_joint_update_for_next_pose = False
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        _LOG.warning(
            "Validate: retry last motion attempt %d for seg=%s reason=%s",
            int(attempt),
            str(seg_name),
            str(last_error),
        )

        self._ros_move_pose_tuple(pose)
        return True

    # --------- Prepare / Segment wiring ---------

    def _prepare_run(self) -> bool:
        self._frame = "substrate"

        self._speed_mm_s = 200.0
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            pass

        if not self._apply_planner_config():
            return False

        self._compiled_poses = self._get_compiled_poses(self._side)

        self._cmd_poses_by_seg.clear()
        self._pending_recipe_poses.clear()
        self._await_joint_update_for_next_pose = False
        self._inflight_pose = None
        self._inflight_seg = ""
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        if not self._compiled_poses:
            self._signal_error(f"Validate: compiled path ist leer (side='{self._side}').")
            return False

        pts = self._compiled_poses
        n = len(pts)

        pre: List[Tuple[float, ...]] = [pts[0]] if n >= 1 else []

        if n >= 2:
            recipe: List[Tuple[float, ...]] = list(pts[1:-1])
            ret: List[Tuple[float, ...]] = [pts[-1]]
        else:
            recipe = []
            ret = []

        self._cmd_poses_by_seg[STATE_MOVE_PREDISPENSE] = pre
        self._cmd_poses_by_seg[STATE_MOVE_RECIPE] = recipe
        self._cmd_poses_by_seg[STATE_MOVE_RETREAT] = ret
        self._cmd_poses_by_seg[STATE_MOVE_HOME] = []

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
        if seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT):
            return bool(self._cmd_poses_by_seg.get(seg_name))
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        self._await_joint_update_for_next_pose = False
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

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
        if self._inflight_seg == seg_name:
            if seg_name == STATE_MOVE_RECIPE:
                if self._inflight_pose is not None and self._pending_recipe_poses:
                    if self._pending_recipe_poses[0] == self._inflight_pose:
                        self._pending_recipe_poses.pop(0)
                    else:
                        self._signal_error("Validate: queue/inflight mismatch in MOVE_RECIPE.")
                        return False
            self._inflight_pose = None
            self._inflight_seg = ""

        if seg_name == STATE_MOVE_RECIPE:
            if self._pending_recipe_poses:
                self._await_joint_update_for_next_pose = True
                self._arm_joint_fallback_timer()
                return False
            return True

        return True

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

        pose_tuple = self._pending_recipe_poses[0]
        self._inflight_pose = pose_tuple
        self._inflight_seg = STATE_MOVE_RECIPE
        self._ros_move_pose_tuple(pose_tuple)

    def _run_home_segment(self) -> None:
        try:
            self._ros.moveitpy.signals.moveToHomeRequestedWithSpeed.emit(float(self._speed_mm_s))
            self._inflight_pose = None
            self._inflight_seg = STATE_MOVE_HOME
        except Exception as ex:
            self._signal_error(f"Validate: move_home failed: {ex}")

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
        if self._machine is None:
            return
        if self._stop_requested:
            return
        if self._current_state != STATE_MOVE_RECIPE:
            return
        if not self._await_joint_update_for_next_pose:
            return
        if not self._pending_recipe_poses:
            return

        _LOG.warning("Validate: jointsChanged fallback triggered. Proceeding with next recipe pose.")
        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    @QtCore.pyqtSlot(object)
    def _on_joints_changed(self, _msg: object) -> None:
        if self._machine is None:
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

    def _ros_move_pose_tuple(self, p: Tuple[float, ...]) -> None:
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

            ps.pose.position.x = float(x) / 1000.0
            ps.pose.position.y = float(y) / 1000.0
            ps.pose.position.z = float(z) / 1000.0
            ps.pose.orientation.x = float(qx)
            ps.pose.orientation.y = float(qy)
            ps.pose.orientation.z = float(qz)
            ps.pose.orientation.w = float(qw)

            self._ros.moveitpy.signals.moveToPoseRequested.emit(ps)
        except Exception as ex:
            self._signal_error(f"Validate: move_pose failed ({x:.1f},{y:.1f},{z:.1f}): {ex}")

    def _get_compiled_poses(self, side: str) -> List[Tuple[float, float, float, float, float, float, float]]:
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
