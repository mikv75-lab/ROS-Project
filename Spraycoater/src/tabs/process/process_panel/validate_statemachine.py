# -*- coding: utf-8 -*-
"""
File: src/tabs/process/process_panel/validate_statemachine.py

Validate statemachine (STRICT, keyed) — now supports strategy selection:
- MOVE_RECIPE can run either:
    * PtpStrategy  (plan_pose + execute per pose)
    * PilzSequenceStrategy (single blended pilz sequence + execute)
  based purely on recipe.planner config (pipeline + planner_id).

IMPORTANT:
- We keep sending planner cfg to MoveIt (ros.moveit_set_planner_cfg) exactly as before.
- For PilzSequenceStrategy we ALSO pass the same segment planner_cfg into the strategy,
  so it can derive pilz payload fields from planner_cfg["params"] (strict contract).

Also: _on_finished now merges planned trajectories for additional op:
  - plan_pose_array_pilz
So planned_traj YAML contains MOVE_RECIPE even when planned by Pilz sequence.
"""

from __future__ import annotations

import time
import copy
import logging
from typing import Any, Dict, List, Optional

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped

from model.recipe.recipe_run_result import RunResult
from model.spray_paths.draft import Draft, PoseQuat
from model.spray_paths.trajectory import JTBySegment

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)
from .segment_runner import SegmentRunner, StepSpec
from .movement_strategies import MoveStrategy, PtpStrategy, PilzSequenceStrategy

_LOG = logging.getLogger("tabs.process.validate_statemachine")


class ProcessValidateStatemachine(BaseProcessStatemachine):
    ROLE = "validate"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 0,
        side: str = "top",
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, run_result=run_result, parent=parent, max_retries=max_retries)

        self._input_side = str(side or "top")
        self._frame: str = "substrate"
        self._speed_mm_s: float = 200.0
        self._poses_by_seg: Dict[str, List[PoseQuat]] = {}

        # Cache for active segment planner selection
        self._active_planner_pipeline: str = "ompl"
        self._active_planner_id: str = "PTP"
        self._active_planner_cfg: Dict[str, Any] = {}  # NEW: pass-through for strategies

        self._runner = SegmentRunner(ros=self._ros, parent=self, max_goal_rejected_retries=int(max_retries))
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)

        self._active_seg: str = ""

    # ------------------------------------------------------------
    # stop
    # ------------------------------------------------------------
    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        try:
            self._runner.request_stop()
        except Exception:
            pass
        super().request_stop()

    # ------------------------------------------------------------
    # prepare
    # ------------------------------------------------------------
    def _prepare_run(self) -> bool:
        self._speed_mm_s = 200.0
        try:
            params = self._recipe.parameters if isinstance(self._recipe.parameters, dict) else {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            self._speed_mm_s = 200.0

        d: Optional[Draft]
        try:
            d = self._recipe.draft
        except Exception:
            d = None
        if d is None:
            self.segment_error("Validate: recipe.draft ist None.")
            return False

        try:
            pre = d.poses_quat_segment(self._side, STATE_MOVE_PREDISPENSE)
            recipe = d.poses_quat_segment(self._side, STATE_MOVE_RECIPE)
            ret = d.poses_quat_segment(self._side, STATE_MOVE_RETREAT)
        except Exception as e:
            self.segment_error(f"Validate: Draft-Segmentierung failed: {e}")
            return False

        if not pre or not recipe or not ret:
            self.segment_error(f"Validate: Leere Segmente im Draft (side='{self._side}').")
            return False

        # NOTE: for validate we keep moves as single-point segments (as before)
        self._poses_by_seg = {
            STATE_MOVE_PREDISPENSE: list(pre[:1]),
            STATE_MOVE_RECIPE: list(recipe),
            STATE_MOVE_RETREAT: list(ret[:1]),
            STATE_MOVE_HOME: [],
        }
        return True

    # ------------------------------------------------------------
    # planner config per segment
    # ------------------------------------------------------------
    def _load_and_apply_config_for_segment(self, seg_name: str) -> bool:
        """
        Loads planner config for the segment and forwards it to ROS (MoveIt node).

        Selection rules (STRICT, config-driven):
          - For MOVE_RECIPE: use recipe.planner["path"] and side override.
          - For MOVE_* moves: use recipe.planner["move"].
          - Legacy fallback still allowed if config is missing.
        """
        planner_sect: Dict[str, Any] = {}
        try:
            if isinstance(self._recipe.planner, dict):
                planner_sect = self._recipe.planner
        except Exception:
            planner_sect = {}

        conf_data: Any = {}

        if seg_name == STATE_MOVE_RECIPE:
            # 'path' section, possibly side-specific
            path_cfg = planner_sect.get("path")
            if isinstance(path_cfg, dict):
                if self._input_side in path_cfg and isinstance(path_cfg.get(self._input_side), dict):
                    conf_data = path_cfg[self._input_side]
                elif "pipeline" in path_cfg:
                    conf_data = path_cfg

            # legacy fallback if needed
            if not isinstance(conf_data, dict) or not conf_data:
                conf_data = planner_sect.get("recipe") or planner_sect.get("validate")
        else:
            # moves
            conf_data = planner_sect.get("move")

        # defaults if missing
        if not isinstance(conf_data, dict):
            conf_data = {
                "pipeline": "ompl",
                "planner_id": "RRTConnectkConfigDefault",
                "params": {},
            }

        pipeline = str(conf_data.get("pipeline") or "ompl").strip()
        planner_id = str(conf_data.get("planner_id") or "").strip()
        params = conf_data.get("params", {})
        if not isinstance(params, dict):
            params = {}

        cfg_msg: Dict[str, Any] = {
            "pipeline": pipeline,
            "planner_id": planner_id,
            "params": params,
        }

        # send to ROS
        try:
            self._ros.moveit_set_planner_cfg(cfg_msg)
        except Exception as e:
            self.segment_error(f"Validate: Config set failed for {seg_name}: {e}")
            return False

        # cache active selection + full cfg for strategy payload mapping
        self._active_planner_pipeline = pipeline.lower()
        self._active_planner_id = planner_id.upper()
        self._active_planner_cfg = copy.deepcopy(cfg_msg)

        _LOG.info(f"Validate Config Applied for '{seg_name}' -> Pipeline: {pipeline}, ID: {planner_id}")
        return True

    # ------------------------------------------------------------
    # strategy selection
    # ------------------------------------------------------------
    def _get_strategy_for_segment(self) -> MoveStrategy:
        """
        Strategy choice based on the segment's loaded config.
        - PilzSequenceStrategy only when pipeline contains "pilz" and planner_id is LIN/CIRC.
        - Everything else: PtpStrategy.
        """
        is_pilz = "pilz" in (self._active_planner_pipeline or "")
        is_linear_seq = (self._active_planner_id or "") in ("LIN", "CIRC")

        if is_pilz and is_linear_seq:
            return PilzSequenceStrategy(self._runner, self._run_id, self._active_planner_cfg)

        return PtpStrategy(self._runner, self._run_id, self._active_planner_cfg)

    # ------------------------------------------------------------
    # segment transitions
    # ------------------------------------------------------------
    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME:
            return True
        return bool(self._poses_by_seg.get(seg_name))

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)

        # 1) load planner config first (segment-specific)
        if not self._load_and_apply_config_for_segment(seg_name):
            return

        # 2) apply speed
            self._ros.moveit_set_speed_mm_s(float(self._speed_mm_s))
      

        # 3) build & run steps
        steps = self._build_steps(seg_name)
        if not steps:
            self.segment_done()
            return
        self._runner.run(steps)

    # ------------------------------------------------------------
    # step construction
    # ------------------------------------------------------------
    def _build_steps(self, seg: str) -> List[StepSpec]:
        if seg == STATE_MOVE_HOME:
            return self._build_home_steps()

        poses_quat = self._poses_by_seg.get(seg) or []
        if not poses_quat:
            return []

        ps_list = [self._pq_to_ps(pq) for pq in poses_quat]

        strategy = self._get_strategy_for_segment()

        # STRICT: no synthetic anchor here; strategy may add anchor if it needs (pilz sequence)
        return strategy.create_steps(seg, ps_list)

    def _build_home_steps(self) -> List[StepSpec]:
        try:
            ps = self._ros.poses_state.st.home
        except Exception:
            ps = None
        if not ps:
            raise RuntimeError("HOME Pose missing")

        rid = int(time.time() * 1000)
        return [
            self._runner.make_plan_pose_step_bound(
                run=self._run_id,
                req_id=rid,
                seg=STATE_MOVE_HOME,
                pose=ps,
                label="HOME:plan",
            ),
            self._runner.make_execute_last_planned_step_bound(
                run=self._run_id,
                req_id=rid,
                seg=STATE_MOVE_HOME,
                label="HOME:exec",
            ),
        ]

    # ------------------------------------------------------------
    # conversions
    # ------------------------------------------------------------
    def _pq_to_ps(self, pq: PoseQuat) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = str(self._frame)

        # PoseQuat uses mm -> convert to meters
        ps.pose.position.x = float(pq.x) / 1000.0
        ps.pose.position.y = float(pq.y) / 1000.0
        ps.pose.position.z = float(pq.z) / 1000.0

        ps.pose.orientation.x = float(pq.qx)
        ps.pose.orientation.y = float(pq.qy)
        ps.pose.orientation.z = float(pq.qz)
        ps.pose.orientation.w = float(pq.qw)
        return ps

    # ------------------------------------------------------------
    # runner callbacks
    # ------------------------------------------------------------
    def _on_runner_finished(self) -> None:
        if self._stop_requested or not self._machine or self._error_msg:
            return
        if self.current_segment() != self._active_seg:
            return
        self.segment_done()

    def _on_runner_error(self, msg: str) -> None:
        if self._stop_requested or not self._machine or self._error_msg:
            return
        if self.current_segment() != self._active_seg:
            return
        self.segment_error(msg)

    # ------------------------------------------------------------
    # finish: merge planned + executed YAML (include Pilz op)
    # ------------------------------------------------------------
    def _on_finished(self) -> None:
        self._deduplicate_storage()

        # Planned: merge all planning ops that may exist
        p_ptp = self._build_trajectory_yaml(self._traj_planned, which_op="plan_pose")
        p_cart = self._build_trajectory_yaml(self._traj_planned, which_op="plan_cartesian")
        p_seq = self._build_trajectory_yaml(self._traj_planned, which_op="plan_pose_array")
        p_pilz_seq = self._build_trajectory_yaml(self._traj_planned, which_op="plan_pilz_sequence")
        
        _merge(planned, p_pilz_seq)
        planned = copy.deepcopy(p_ptp)

        def _merge(target: Dict[str, Any], source: Dict[str, Any]) -> None:
            if source.get("segments"):
                if "segments" not in target:
                    target["segments"] = {}
                target["segments"].update(source["segments"])

        _merge(planned, p_cart)
        _merge(planned, p_seq)
        _merge(planned, p_pilz_seq)  # NEW

        executed = self._build_trajectory_yaml(self._traj_executed, which_op="execute")

        if not planned.get("segments"):
            self.notifyError.emit("Validate: planned trajectory empty.")
            self._cleanup()
            return

        try:
            self._recipe.planned_traj = JTBySegment.from_yaml_dict(planned)
        except Exception as e:
            self.notifyError.emit(f"Validate: failed write recipe.planned_traj: {e}")
            self._cleanup()
            return

        try:
            self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception:
            self._recipe.executed_traj = None

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))

        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()
