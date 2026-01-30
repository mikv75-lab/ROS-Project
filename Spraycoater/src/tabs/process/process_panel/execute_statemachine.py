# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import copy
import time
import logging
from typing import Any, Dict, List, Optional

from PyQt6 import QtCore

from plc.plc_client import PlcClientBase
from model.recipe.recipe_run_result import RunResult
from model.spray_paths.trajectory import JTBySegment

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)
from .segment_runner import SegmentRunner, StepSpec
from .process_contract import create_header_frame_id
from .process_serialization import jt_dict_to_robottrajectory_msg

_LOG = logging.getLogger("tabs.process.execute_sm")

class ProcessExecuteStatemachine(BaseProcessStatemachine):
    ROLE = "execute"

    def __init__(self, *, recipe: Any, ros: Any, plc: PlcClientBase | None, run_result: RunResult, parent: Optional[QtCore.QObject] = None, max_retries: int = 0, side: str = "top") -> None:
        super().__init__(recipe=recipe, ros=ros, run_result=run_result, parent=parent, max_retries=max_retries)
        self._plc = plc
        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._planned_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        self._runner = SegmentRunner(ros=self._ros, parent=self)
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)
        self._active_seg: str = ""

    def _prepare_run(self) -> bool:
        pt = getattr(self._recipe, "planned_traj", None)
        if pt is None:
            self.segment_error("Execute: recipe.planned_traj fehlt.")
            return False
        
        try: d = pt.to_yaml_dict()
        except Exception: d = {}
        if not isinstance(d, dict):
            self.segment_error("Execute: planned_traj invalid.")
            return False

        self._planned_loaded_yaml = d
        self._planned_jt_by_segment = {}
        
        segs = d.get("segments", {})
        for seg in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            s_data = segs.get(seg)
            if s_data and isinstance(s_data.get("points"), list) and len(s_data["points"]) >= 2:
                self._planned_jt_by_segment[seg] = s_data

        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self.segment_error("Execute: ros.moveit_execute_trajectory fehlt.")
            return False
        return True

    def _segment_exists(self, seg_name: str) -> bool:
        return seg_name in self._planned_jt_by_segment

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)
        if seg_name not in self._planned_jt_by_segment:
            self.segment_done()
            return

        planned_jt = self._planned_jt_by_segment[seg_name]
        req_id = int(time.time() * 1000)

        header_json = create_header_frame_id(self._run_id, req_id, seg_name, "execute_trajectory")
        try:
            rt = jt_dict_to_robottrajectory_msg(planned_jt, header_json)
        except Exception as e:
            self.segment_error(f"Execute: Build RobotTrajectory failed: {e}")
            return

        steps = [
            self._runner.make_execute_trajectory_step_bound(
                run=self._run_id, req_id=req_id, seg=seg_name, traj=rt, label=f"{seg_name}:exec"
            )
        ]
        self._runner.run(steps)

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
        
        planned = copy.deepcopy(self._planned_loaded_yaml)
        # Using Base helper to collect execution data
        executed = self._build_trajectory_yaml(self._traj_executed, which_op="execute_trajectory")

        try: self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception: self._recipe.executed_traj = None

        self._rr.set_planned(traj=planned)
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()