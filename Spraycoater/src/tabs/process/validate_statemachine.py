# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, List, Tuple

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
    ROLE = "validate"

    # Optional: kleine Entkopplung im Qt-Eventloop (ms)
    _NEXT_POSE_QT_DELAY_MS = 0

    # Fallback: wenn kein jointsChanged kommt, trotzdem weiter (ms)
    _JOINT_UPDATE_FALLBACK_MS = 200

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

        self._side = str(side or "top")

        # cached compiled points for this run
        self._compiled_pts_mm: List[Tuple[float, float, float]] = []

        # per-segment commanded points (for payload "tcp" -> should path)
        self._cmd_pts_by_seg: Dict[str, List[Tuple[float, float, float]]] = {}

        # RECIPE streaming queue
        self._pending_recipe_mm: List[Tuple[float, float, float]] = []

        # frames + speed
        self._frame: str = "world"
        self._speed_mm_s: float = 200.0

        # streaming gate: after EXECUTED:OK wait for next jointsChanged (or fallback)
        self._await_joint_update_for_next_pose: bool = False
        self._joint_sig_connected: bool = False

        # Qt fallback timer (single-shot)
        self._joint_fallback_timer = QtCore.QTimer(self)
        self._joint_fallback_timer.setSingleShot(True)
        self._joint_fallback_timer.timeout.connect(self._on_joint_fallback_timeout)

        # Connect RobotBridge jointsChanged (used as "state updated" indicator)
        try:
            self._ros.robot.signals.jointsChanged.connect(self._on_joints_changed)
            self._joint_sig_connected = True
        except Exception:
            self._joint_sig_connected = False

    # ---------------- Stop handling ----------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        Deterministic stop for Validate worker.

        Validate can be in a gated RECIPE streaming state (waiting for jointsChanged / fallback timer).
        Only setting a stop flag may leave the state machine without emitting notifyError/notifyFinished.
        Therefore we actively terminate the run immediately.
        """
        # Let base set internal stop flags (best effort)
        try:
            super().request_stop()
        except Exception:
            # fallback if base doesn't implement request_stop
            try:
                self._stop_requested = True
            except Exception:
                pass

        # Cancel any pending gating/timers immediately
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        self._await_joint_update_for_next_pose = False
        try:
            self._pending_recipe_mm.clear()
        except Exception:
            pass

        # Terminate run now (queued to keep thread-safety with Qt)
        QtCore.QTimer.singleShot(0, lambda: self._signal_error("STOP requested"))

    # ---------------- Hooks ----------------

    def _prepare_run(self) -> bool:
        # frame
        self._frame = "world"
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

        # compiled points (single source of truth)
        self._compiled_pts_mm = self._get_compiled_points_mm(self._side)

        self._cmd_pts_by_seg.clear()
        self._pending_recipe_mm.clear()
        self._await_joint_update_for_next_pose = False
        self._joint_fallback_timer.stop()

        if not self._compiled_pts_mm:
            self._signal_error(f"Validate: compiled path ist leer (side='{self._side}').")
            return False

        # segment slicing (match your intended semantics)
        # - predispense: first point
        # - recipe:      middle points
        # - retreat:     last point
        pts = self._compiled_pts_mm
        n = len(pts)

        pre = [pts[0]] if n >= 1 else []
        ret = [pts[-1]] if n >= 2 else []  # if only 1 point, retreat would equal pre; keep empty then
        mid = pts[1:-1] if n >= 3 else []

        self._cmd_pts_by_seg[STATE_MOVE_PREDISPENSE] = pre
        self._cmd_pts_by_seg[STATE_MOVE_RECIPE] = list(mid)
        self._cmd_pts_by_seg[STATE_MOVE_RETREAT] = ret
        self._cmd_pts_by_seg[STATE_MOVE_HOME] = []  # home has no commanded TCP list

        # queue for recipe streaming
        self._pending_recipe_mm = list(self._cmd_pts_by_seg[STATE_MOVE_RECIPE])

        return True

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and self._skip_home:
            return False
        # execute segments only if they have commanded points (except home)
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
        # Stream recipe points in the same state:
        # After EXECUTED:OK, wait until we observe a fresh JointState update.
        # If no update arrives quickly (signal loss / scheduling / QoS), use fallback timer.
        if seg_name == STATE_MOVE_RECIPE:
            if self._pending_recipe_mm:
                self._await_joint_update_for_next_pose = True
                self._arm_joint_fallback_timer()
                return False
            return True
        return True

    # ---------------- Segment runners ----------------

    def _run_single_pose_segment(self, seg_name: str) -> None:
        pts = self._cmd_pts_by_seg.get(seg_name) or []
        if not pts:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        x, y, z = pts[0]
        self._ros_move_pose_mm(x, y, z)

    def _run_recipe_stream_segment(self) -> None:
        # nothing -> ok
        if not self._pending_recipe_mm:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # Start first recipe pose immediately (no gating needed yet)
        self._send_next_recipe_pose()

    def _send_next_recipe_pose(self) -> None:
        # Stop-safe: never send further poses after stop
        if self._stop_requested:
            return

        # When we actively send, stop any pending fallback so we don't double-send.
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
            # canonical interface (see moveitpy_bridge.py)
            self._ros.moveitpy.signals.moveToHomeRequestedWithSpeed.emit(float(self._speed_mm_s))
        except Exception as ex:
            self._signal_error(f"Validate: move_home failed: {ex}")

    # ---------------- Robot state gating ----------------

    def _arm_joint_fallback_timer(self) -> None:
        """Arm fallback timer that unblocks recipe streaming if jointsChanged does not arrive."""
        self._joint_fallback_timer.stop()

        # If no joint signal is connected, fallback is mandatory.
        # Even if connected, we still use fallback for robustness.
        ms = int(self._JOINT_UPDATE_FALLBACK_MS)
        if ms <= 0:
            ms = 1
        self._joint_fallback_timer.start(ms)

    @QtCore.pyqtSlot()
    def _on_joint_fallback_timeout(self) -> None:
        """
        Fallback when jointsChanged doesn't arrive.
        We only proceed if we're still in the gated situation.
        """
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
        """
        Gate for MoveIt start-state validity:
        After EXECUTED:OK in RECIPE we wait until we see at least one jointsChanged.
        Then we emit the next pose request.
        """
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

        # Cancel fallback: we received the joint update we wanted.
        self._joint_fallback_timer.stop()

        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    # ---------------- ROS facade helpers ----------------

    def _ros_move_pose_mm(self, x: float, y: float, z: float) -> None:
        """
        Use RosBridge facade, which forwards to MoveItPyBridge.moveToPoseRequested
        and auto-executes when PLANNED:OK pose arrives.
        """
        from geometry_msgs.msg import PoseStamped

        try:
            # canonical interface (see moveitpy_bridge.py)
            self._ros.moveitpy.signals.motionSpeedChanged.emit(float(self._speed_mm_s))

            ps = PoseStamped()
            ps.header.frame_id = str(self._frame or "world")

            # Stamp if possible (reduces MoveIt/TF edge cases)
            try:
                # common patterns: ros.node, ros._node, ros.rclpy_node
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
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            self._ros.moveitpy.signals.moveToPoseRequested.emit(ps)
        except Exception as ex:
            self._signal_error(f"Validate: move_pose failed ({x:.1f},{y:.1f},{z:.1f}): {ex}")

    # ---------------- payload ----------------

    def _build_result(self) -> Dict[str, Any]:
        frame = self._frame or "world"
        segs: Dict[str, Dict[str, Any]] = {}

        def _tcp_segment(pts_mm: List[Tuple[float, float, float]]) -> Dict[str, Any]:
            poses_quat = [
                {"x": float(x), "y": float(y), "z": float(z), "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
                for (x, y, z) in pts_mm
            ]
            return {
                "frame": frame,
                "tool_frame": (getattr(self._recipe, "planner", {}) or {}).get("tool_frame", "tool_mount"),
                "sides": {str(self._side): {"poses_quat": poses_quat}},
            }

        for s in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            pts = self._cmd_pts_by_seg.get(s) or []
            traj = self._executed_by_segment.get(s)
            planned = self._planned_by_segment.get(s)

            # Only persist non-empty segments, except HOME if it actually executed and has a traj
            if not pts and not traj and s != STATE_MOVE_HOME:
                continue
            if s == STATE_MOVE_HOME and (traj is None and planned is None):
                continue

            segs[s] = {
                "tcp": _tcp_segment(pts) if pts else {"frame": frame, "tool_frame": "tool_mount", "sides": {}},
                "traj": traj,
                "planned": planned,
                "meta": {
                    "role": self.ROLE,
                    "state": s,
                    "side": str(self._side),
                    "frame": frame,
                    "n_points": int(len(pts)),
                    "source": "compiled_commanded_mm",
                },
            }

        return {
            "version": 1,
            "meta": {
                "role": self.ROLE,
                "side": str(self._side),
                "frame": frame,
                "source": "validate_statemachine",
            },
            "segments": segs,
        }

    # ---------------- compiled points extraction ----------------

    def _get_compiled_points_mm(self, side: str) -> List[Tuple[float, float, float]]:
        """Canonical Recipe API: compiled_points_mm_for_side(side) -> Optional[np.ndarray] Nx3 (mm)."""
        out: List[Tuple[float, float, float]] = []
        pts = self._recipe.compiled_points_mm_for_side(str(side))
        if pts is None:
            return out
        for row in pts.tolist():
            out.append((float(row[0]), float(row[1]), float(row[2])))
        return out

    # ---------------- Cleanup ----------------

    def _cleanup(self) -> None:
        # stop timers first to avoid late callbacks during teardown
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        # disconnect robot joints signal (we connected it in __init__)
        if self._joint_sig_connected:
            try:
                self._ros.robot.signals.jointsChanged.disconnect(self._on_joints_changed)
            except Exception:
                pass
            self._joint_sig_connected = False

        super()._cleanup()
