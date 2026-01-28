# -*- coding: utf-8 -*-
# File: tabs/process/validate_statemachine.py
from __future__ import annotations

import copy
import logging
from dataclasses import dataclass
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


@dataclass
class _JTBySegmentYamlV1:
    """
    Minimal wrapper so downstream (Optimize/Execute) can rely on the API:
      recipe.planned_traj.to_yaml_dict() -> Dict[str, Any]
    """
    yaml: Dict[str, Any]

    def to_yaml_dict(self) -> Dict[str, Any]:
        return copy.deepcopy(self.yaml)


class ProcessValidateStatemachine(BaseProcessStatemachine):
    """
    Validate run: streams poses as consecutive MoveIt pose commands.

    COLLECT-ONLY (STRICT):
      - This statemachine ONLY drives motions and lets BaseProcessStatemachine collect raw trajectory snapshots.
      - It does NOT run FK, does NOT compute TCP docs, and does NOT evaluate anything.
      - The returned RunResult payload contains planned/executed trajectories by segment only.
        Postprocess (FK/TCP) + evaluation happens later in RecipePanel/RecipeEvaluator.

    Segment/pose contract (STRICT, NEW draft layout):
      - MOVE_PREDISPENSE: exactly draft.sides[side].predispense[0]
      - MOVE_RECIPE:      exactly draft.sides[side].poses_quat[*]   (recipe-only)
      - MOVE_RETREAT:     exactly draft.sides[side].retreat[0]
      - MOVE_HOME:        (no poses; dedicated command)

    IMPORTANT:
      - No slicing like recipe[0] / recipe[-1] anymore.
      - We rely on Base helper `_draft_segment_pose_tuples()` so the contract is enforced in one place.
      - Validate MUST write recipe.planned_traj (JTBySegment YAML v1) so Optimize/Execute can consume it.
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

    # ------------------------------------------------------------------
    # STOP
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
    # Retry Hook (called by Base)
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Prepare / Segment wiring
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # STRICT: frame for pose commands in this architecture
        self._frame = "substrate"

        self._speed_mm_s = 200.0
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            pass

        if not self._apply_planner_config():
            return False

        self._cmd_poses_by_seg.clear()
        self._pending_recipe_poses.clear()
        self._await_joint_update_for_next_pose = False
        self._inflight_pose = None
        self._inflight_seg = ""
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        # STRICT: segment-specific extraction (no slicing)
        try:
            pre = self._draft_segment_pose_tuples(side=self._side, seg=STATE_MOVE_PREDISPENSE)
            recipe = self._draft_segment_pose_tuples(side=self._side, seg=STATE_MOVE_RECIPE)
            ret = self._draft_segment_pose_tuples(side=self._side, seg=STATE_MOVE_RETREAT)
        except Exception as e:
            self._signal_error(f"Validate: Draft-Segmentierung fehlgeschlagen (side='{self._side}'): {e}")
            return False

        if not pre:
            self._signal_error(f"Validate: predispense fehlt (side='{self._side}').")
            return False
        if not recipe:
            self._signal_error(f"Validate: poses_quat (recipe) ist leer (side='{self._side}').")
            return False
        if not ret:
            self._signal_error(f"Validate: retreat fehlt (side='{self._side}').")
            return False

        self._cmd_poses_by_seg[STATE_MOVE_PREDISPENSE] = list(pre[:1])   # exactly one
        self._cmd_poses_by_seg[STATE_MOVE_RECIPE] = list(recipe)         # full recipe list
        self._cmd_poses_by_seg[STATE_MOVE_RETREAT] = list(ret[:1])       # exactly one
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

    # ------------------------------------------------------------------
    # Segment gating / enter
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT):
            return bool(self._cmd_poses_by_seg.get(seg_name))
        # HOME always exists
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
        """
        Base calls this AFTER it has a committed planned+executed pair and wants to know
        whether it can transition to the next segment.

        For MOVE_RECIPE we stream multiple poses. Therefore:
          - If there are still pending poses, do NOT transition yet.
          - Instead wait for jointsChanged (or fallback timer) then send next pose.
        """
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

    # ------------------------------------------------------------------
    # Joint update gating for streaming
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

    # ------------------------------------------------------------------
    # ROS command: pose tuple -> MoveIt request
    # ------------------------------------------------------------------

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

            # Input tuples are in mm; ROS uses meters
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

    # ------------------------------------------------------------------
    # FINISH: build JTBySegment YAML v1 and write recipe.planned_traj
    # ------------------------------------------------------------------

    def _on_finished(self) -> None:
        # Base provides raw snapshots; we persist both planned/executed by segment.
        planned = self._jt_by_segment_yaml(which="planned")
        executed = self._jt_by_segment_yaml(which="executed")

        # STRICT: planned must not be empty, because Optimize/Execute depend on it
        p_segs = planned.get("segments") if isinstance(planned, dict) else None
        if not (isinstance(p_segs, dict) and p_segs):
            self.notifyError.emit(
                "Validate finished, but planned trajectory capture is empty. "
                "Check MoveItPyNode planned_trajectory publish + Base capture timeout."
            )
            self._cleanup()
            return

        # Persist planned_traj into recipe for downstream stages (Optimize/Execute)
        try:
            setattr(self._recipe, "planned_traj", _JTBySegmentYamlV1(planned))
        except Exception as e:
            self.notifyError.emit(f"Validate: failed to write recipe.planned_traj: {e}")
            self._cleanup()
            return

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

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
