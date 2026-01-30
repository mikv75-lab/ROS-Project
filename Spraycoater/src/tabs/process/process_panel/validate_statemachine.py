# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/validate_statemachine.py
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
from .movement_strategies import MoveStrategy, PtpStrategy, CartesianStrategy, PilzSequenceStrategy

_LOG = logging.getLogger("tabs.process.validate_statemachine")

class ProcessValidateStatemachine(BaseProcessStatemachine):
    ROLE = "validate"

    def __init__(self, *, recipe: Any, ros: Any, run_result: RunResult, parent: Optional[QtCore.QObject] = None, max_retries: int = 0, side: str = "top") -> None:
        super().__init__(recipe=recipe, ros=ros, run_result=run_result, parent=parent, max_retries=max_retries)
        self._input_side = str(side or "top")
        self._frame: str = "substrate"
        self._speed_mm_s: float = 200.0
        self._poses_by_seg: Dict[str, List[PoseQuat]] = {}
        
        # Cache for the currently active planner configuration
        self._active_planner_pipeline: str = "ompl" 
        self._active_planner_id: str = "PTP"

        self._runner = SegmentRunner(ros=self._ros, parent=self, max_goal_rejected_retries=int(max_retries))
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)
        self._active_seg: str = ""

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        try: self._runner.request_stop()
        except Exception: pass
        super().request_stop()

    def _prepare_run(self) -> bool:
        self._speed_mm_s = 200.0
        try:
            params = self._recipe.parameters if isinstance(self._recipe.parameters, dict) else {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            self._speed_mm_s = 200.0

        d: Optional[Draft]
        try: d = self._recipe.draft
        except Exception: d = None
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

        self._poses_by_seg = {
            STATE_MOVE_PREDISPENSE: list(pre[:1]),
            STATE_MOVE_RECIPE: list(recipe),
            STATE_MOVE_RETREAT: list(ret[:1]),
            STATE_MOVE_HOME: []
        }
        return True

    def _load_and_apply_config_for_segment(self, seg_name: str) -> bool:
        """
        Loads the Planner config for the current segment.
        Explicitly supports the structure: planner -> path -> <side>
        """
        planner_sect = {}
        try:
            if isinstance(self._recipe.planner, dict):
                planner_sect = self._recipe.planner
        except Exception:
            pass

        conf_data = {}

        if seg_name == STATE_MOVE_RECIPE:
            # 1. Prio: Specific path for active side (e.g. planner.path.top)
            path_cfg = planner_sect.get("path")
            if isinstance(path_cfg, dict):
                if self._input_side in path_cfg:
                    conf_data = path_cfg[self._input_side]
                elif "pipeline" in path_cfg:
                    conf_data = path_cfg
            
            # 2. Prio: General Keywords
            if not conf_data:
                conf_data = planner_sect.get("recipe") or planner_sect.get("validate") or planner_sect.get("move")
        else:
            # For Moves (Approach/Retreat/Home) use 'move'
            conf_data = planner_sect.get("move")

        # Fallback Defaults
        if not isinstance(conf_data, dict):
            conf_data = {
                "pipeline": "ompl",
                "planner_id": "RRTConnectkConfigDefault",
                "params": {}
            }

        # Extract values
        pipeline = str(conf_data.get("pipeline") or "ompl").strip()
        planner_id = str(conf_data.get("planner_id") or "").strip()
        params = conf_data.get("params", {})

        # Send to ROS
        cfg_msg = {
            "pipeline": pipeline,
            "planner_id": planner_id,
            "params": params
        }
        
        try:
            self._ros.moveit_set_planner_cfg(cfg_msg)
        except Exception as e:
            self.segment_error(f"Validate: Config set failed for {seg_name}: {e}")
            return False

        # Store status for Strategy selection
        self._active_planner_pipeline = pipeline.lower()
        self._active_planner_id = planner_id.upper()
        
        _LOG.info(f"Validate Config Applied for '{seg_name}' -> Pipeline: {pipeline}, ID: {planner_id}")
        return True

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME: return True
        return bool(self._poses_by_seg.get(seg_name))

    def _get_strategy_for_segment(self) -> MoveStrategy:
        """
        Selects the strategy based on the *currently loaded* Config.
        """
        is_pilz = "pilz" in self._active_planner_pipeline
        is_linear_seq = self._active_planner_id in ("LIN", "CIRC")

        # Only if Pilz AND (LIN or CIRC) are selected -> Sequence Strategy
        if is_pilz and is_linear_seq:
            return PilzSequenceStrategy(self._runner, self._run_id)
        
        # Otherwise PTP
        return PtpStrategy(self._runner, self._run_id)

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)
        
        # Load config (this sets _active_planner_pipeline & _active_planner_id)
        if not self._load_and_apply_config_for_segment(seg_name):
            return

        try: self._ros.moveit_set_speed_mm_s(float(self._speed_mm_s))
        except Exception: pass
        
        # Build steps (Strategy selection happens now with correct Config)
        steps = self._build_steps(seg_name)
        if not steps:
            self.segment_done()
            return
        self._runner.run(steps)

    def _build_steps(self, seg: str) -> List[StepSpec]:
        if seg == STATE_MOVE_HOME:
            return self._build_home_steps()

        poses_quat = self._poses_by_seg.get(seg) or []
        if not poses_quat: return []

        ps_list = [self._pq_to_ps(pq) for pq in poses_quat]

        strategy = self._get_strategy_for_segment()
        steps = strategy.create_steps(seg, ps_list)

        # ---------------------------------------------------------------------
        # TRICK 17: ANCHOR MOVE (Gap Closing)
        # ---------------------------------------------------------------------
        # If we want to drive Pilz LIN, we MUST be exactly at the start.
        # Since PREDISPENSE ends at 49.0 and RECIPE starts at 48.869,
        # we insert a PTP step here that closes this gap.
        # FIX: Only apply Anchor if it is a TRUE sequence (>1 point)!
        if isinstance(strategy, PilzSequenceStrategy) and len(ps_list) > 1:
            start_pose = ps_list[0]
            rid = int(time.time()*1000)
            
            # We manually build a PTP step to the very first point of the sequence
            anchor_plan = self._runner.make_plan_pose_step_bound(
                run=self._run_id, req_id=rid, seg=seg, pose=start_pose, label=f"{seg}:anchor:plan"
            )
            anchor_exec = self._runner.make_execute_last_planned_step_bound(
                run=self._run_id, req_id=rid, seg=seg, label=f"{seg}:anchor:exec"
            )
            
            # Push Anchor BEFORE the actual sequence
            steps = [anchor_plan, anchor_exec] + steps
        # ---------------------------------------------------------------------

        return steps

    def _build_home_steps(self) -> List[StepSpec]:
        try: ps = self._ros.poses_state.st.home
        except Exception: ps = None
        if not ps: raise RuntimeError("HOME Pose missing")
        
        rid = int(time.time()*1000)
        # Home is always PTP (implicitly via _load_and_apply_config_for_segment with 'move' config usually defaulting or set to PTP)
        return [
            self._runner.make_plan_pose_step_bound(run=self._run_id, req_id=rid, seg=STATE_MOVE_HOME, pose=ps, label="HOME:plan"),
            self._runner.make_execute_last_planned_step_bound(run=self._run_id, req_id=rid, seg=STATE_MOVE_HOME, label="HOME:exec")
        ]

    def _pq_to_ps(self, pq: PoseQuat) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = str(self._frame)
        ps.pose.position.x = float(pq.x)/1000.0
        ps.pose.position.y = float(pq.y)/1000.0
        ps.pose.position.z = float(pq.z)/1000.0
        ps.pose.orientation.x = float(pq.qx)
        ps.pose.orientation.y = float(pq.qy)
        ps.pose.orientation.z = float(pq.qz)
        ps.pose.orientation.w = float(pq.qw)
        return ps

    def _on_runner_finished(self) -> None:
        if self._stop_requested or not self._machine or self._error_msg: return
        if self.current_segment() != self._active_seg: return
        self.segment_done()

    def _on_runner_error(self, msg: str) -> None:
        if self._stop_requested or not self._machine or self._error_msg: return
        if self.current_segment() != self._active_seg: return
        self.segment_error(msg)

    def _on_finished(self) -> None:
        self._deduplicate_storage()
        
        p_ptp = self._build_trajectory_yaml(self._traj_planned, which_op="plan_pose")
        p_cart = self._build_trajectory_yaml(self._traj_planned, which_op="plan_cartesian")
        p_seq = self._build_trajectory_yaml(self._traj_planned, which_op="plan_pose_array")
        
        planned = copy.deepcopy(p_ptp)
        
        def _merge(target, source):
            if source.get("segments"):
                if "segments" not in target: target["segments"] = {}
                target["segments"].update(source["segments"])

        _merge(planned, p_cart)
        _merge(planned, p_seq)

        executed = self._build_trajectory_yaml(self._traj_executed, which_op="execute")

        if not planned.get("segments"):
            self.notifyError.emit("Validate: planned trajectory empty.")
            self._cleanup()
            return

        try: self._recipe.planned_traj = JTBySegment.from_yaml_dict(planned)
        except Exception as e:
             self.notifyError.emit(f"Validate: failed write recipe.planned_traj: {e}")
             self._cleanup()
             return

        try: self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception: self._recipe.executed_traj = None

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))

        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()