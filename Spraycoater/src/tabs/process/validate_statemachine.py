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

    _NEXT_POSE_QT_DELAY_MS = 0
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

        # trajectory capture wiring flag
        self._traj_sig_connected: bool = False

        # robot joints gating (recipe streaming)
        try:
            self._ros.robot.signals.jointsChanged.connect(self._on_joints_changed)
            self._joint_sig_connected = True
        except Exception:
            self._joint_sig_connected = False

        # IMPORTANT: capture MoveIt trajectories (planned/executed) per segment
        self._connect_moveit_traj_signals()

    # ------------------------------------------------------------------
    # Wiring: MoveIt planned/executed (RobotTrajectory messages)
    # ------------------------------------------------------------------

    def _connect_moveit_traj_signals(self) -> None:
        """
        Ensure we collect trajectories into BaseProcessStatemachine buffers.
        Without this, payload segments stay empty => save_run fails.
        """
        try:
            sig = self._ros.moveitpy.signals
        except Exception:
            _LOG.exception("Validate: cannot access ros.moveitpy.signals")
            self._traj_sig_connected = False
            return

        try:
            # MoveItPyBridge signals:
            # - plannedTrajectoryChanged(RobotTrajectoryMsg)
            # - executedTrajectoryChanged(RobotTrajectoryMsg)
            sig.plannedTrajectoryChanged.connect(self._on_planned_traj_changed)
            sig.executedTrajectoryChanged.connect(self._on_executed_traj_changed)
            self._traj_sig_connected = True
            _LOG.info("Validate: connected MoveItPy trajectory signals (planned/executed).")
        except Exception:
            _LOG.exception("Validate: connecting MoveItPy trajectory signals failed")
            self._traj_sig_connected = False

    def _ensure_traj_maps(self) -> None:
        """
        BaseProcessStatemachine should already provide these.
        If not, create them so Validate can still run.
        """
        if not hasattr(self, "_planned_steps_by_segment") or getattr(self, "_planned_steps_by_segment") is None:
            self._planned_steps_by_segment = {}
        if not hasattr(self, "_executed_steps_by_segment") or getattr(self, "_executed_steps_by_segment") is None:
            self._executed_steps_by_segment = {}
        if not hasattr(self, "_planned_by_segment") or getattr(self, "_planned_by_segment") is None:
            self._planned_by_segment = {}
        if not hasattr(self, "_executed_by_segment") or getattr(self, "_executed_by_segment") is None:
            self._executed_by_segment = {}

    @staticmethod
    def _jt_msg_to_event_dict(jt_msg) -> Dict[str, Any]:
        """
        Convert trajectory_msgs/JointTrajectory -> JSON-serializable "event" dict.
        (This dict is an EVENT entry inside segments[SEG] list.)
        """
        out: Dict[str, Any] = {
            "joint_names": list(getattr(jt_msg, "joint_names", []) or []),
            "points": [],
        }

        pts = getattr(jt_msg, "points", None) or []
        for p in pts:
            tfs = getattr(p, "time_from_start", None)
            sec = int(getattr(tfs, "sec", 0) or 0) if tfs is not None else 0
            nsec = int(getattr(tfs, "nanosec", 0) or 0) if tfs is not None else 0

            out["points"].append(
                {
                    "positions": list(getattr(p, "positions", []) or []),
                    "velocities": list(getattr(p, "velocities", []) or []),
                    "accelerations": list(getattr(p, "accelerations", []) or []),
                    "effort": list(getattr(p, "effort", []) or []),
                    "time_from_start": {"sec": sec, "nanosec": nsec},
                }
            )

        return out

    def _append_step_and_merge(self, *, which: str, seg: str, jt_event: Dict[str, Any]) -> None:
        """
        Store per-step JT event dict and keep merged JT in *_by_segment updated.
        """
        self._ensure_traj_maps()

        if which == "planned":
            steps_map = self._planned_steps_by_segment
            merged_map = self._planned_by_segment
        else:
            steps_map = self._executed_steps_by_segment
            merged_map = self._executed_by_segment

        steps = steps_map.get(seg)
        if steps is None:
            steps = []
            steps_map[seg] = steps

        steps.append(jt_event)

        # update merged (internal only; not necessarily written into payload)
        try:
            merged_map[seg] = self._concat_joint_traj_dicts(list(steps))
        except Exception:
            merged_map[seg] = jt_event

    @QtCore.pyqtSlot(object)
    def _on_planned_traj_changed(self, rt_msg: object) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested:
            return

        seg = getattr(self, "_current_state", "") or ""
        if seg not in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            return

        try:
            jt_msg = getattr(rt_msg, "joint_trajectory", None)
            if jt_msg is None:
                return
            ev = self._jt_msg_to_event_dict(jt_msg)
            if not (ev.get("points") or []):
                return
            self._append_step_and_merge(which="planned", seg=seg, jt_event=ev)
        except Exception:
            _LOG.exception("Validate: planned trajectory capture failed")

    @QtCore.pyqtSlot(object)
    def _on_executed_traj_changed(self, rt_msg: object) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested:
            return

        seg = getattr(self, "_current_state", "") or ""
        if seg not in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            return

        try:
            jt_msg = getattr(rt_msg, "joint_trajectory", None)
            if jt_msg is None:
                return
            ev = self._jt_msg_to_event_dict(jt_msg)
            if not (ev.get("points") or []):
                return
            self._append_step_and_merge(which="executed", seg=seg, jt_event=ev)
        except Exception:
            _LOG.exception("Validate: executed trajectory capture failed")

    # ------------------------------------------------------------------
    # Hooks
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        self._frame = "world"
        try:
            pc = getattr(self._recipe, "paths_compiled", None)
            if isinstance(pc, dict) and pc.get("frame"):
                self._frame = str(pc.get("frame"))
        except Exception:
            pass

        self._speed_mm_s = 200.0
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            pass

        self._compiled_pts_mm = self._get_compiled_points_mm(self._side)

        self._cmd_pts_by_seg.clear()
        self._pending_recipe_mm.clear()
        self._await_joint_update_for_next_pose = False
        self._joint_fallback_timer.stop()

        # IMPORTANT: clear collected trajectories per run
        self._ensure_traj_maps()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()

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
        from geometry_msgs.msg import PoseStamped

        try:
            self._ros.moveitpy.signals.motionSpeedChanged.emit(float(self._speed_mm_s))

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
    # Payloads
    # ------------------------------------------------------------------

    def _build_run_payload(self, *, which: str) -> Dict[str, Any]:
        """
        Schema FIX for save_run validator:

        segments[SEG] MUST be a LIST (events), not a dict.

        We therefore export:
          segments:
            MOVE_PREDISPENSE: [ <jt_event_dict>, ... ]
            MOVE_RECIPE:      [ <jt_event_dict>, ... ]
            ...

        Merged trajectories remain available internally (self._planned_by_segment etc.)
        but are NOT emitted in the payload, because your validator rejects dict keys.
        """
        if which not in ("planned", "executed"):
            raise ValueError(f"_build_run_payload: invalid which={which}")

        frame = self._frame or "world"
        steps_map = self._planned_steps_by_segment if which == "planned" else self._executed_steps_by_segment

        segs: Dict[str, List[Dict[str, Any]]] = {}
        dbg: List[str] = []

        for s in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            events = list((steps_map or {}).get(s) or [])
            if not events:
                dbg.append(f"{which}:{s}: no events")
                continue

            # IMPORTANT: segment value is a LIST of event dicts
            segs[s] = events

            # debug counts
            try:
                n_pts = int(len((events[-1].get("points") or []))) if events else 0
            except Exception:
                n_pts = 0
            dbg.append(f"{which}:{s}: ok (events={len(events)}, last_points={n_pts})")

        if not segs:
            msg = "Validate: keine JointTrajectory erfasst (payload leer). " + "; ".join(dbg[:50])
            _LOG.error(msg)

        return {
            "version": 1,
            "meta": {
                "role": self.ROLE,
                "side": str(self._side),
                "frame": frame,
                "source": "validate_statemachine",
                "format": "joint_events_by_segment",
                "kind": which,
            },
            "segments": segs,
            "debug": dbg[:200],
        }

    def _build_result(self) -> Dict[str, Any]:
        planned = self._build_run_payload(which="planned")
        executed = self._build_run_payload(which="executed")
        return {"planned_run": planned, "executed_run": executed}

    # ------------------------------------------------------------------
    # Compiled points extraction
    # ------------------------------------------------------------------

    def _get_compiled_points_mm(self, side: str) -> List[Tuple[float, float, float]]:
        out: List[Tuple[float, float, float]] = []
        pts = self._recipe.compiled_points_mm_for_side(str(side))
        if pts is None:
            return out
        for row in pts.tolist():
            out.append((float(row[0]), float(row[1]), float(row[2])))
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

        if self._traj_sig_connected:
            try:
                sig = self._ros.moveitpy.signals
                sig.plannedTrajectoryChanged.disconnect(self._on_planned_traj_changed)
                sig.executedTrajectoryChanged.disconnect(self._on_executed_traj_changed)
            except Exception:
                pass
            self._traj_sig_connected = False

        super()._cleanup()
