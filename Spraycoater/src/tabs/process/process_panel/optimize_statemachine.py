# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import copy
import time
import logging
from typing import Any, Dict, Tuple, Optional

from PyQt6 import QtCore
from model.recipe.recipe_run_result import RunResult
from model.spray_paths.trajectory import JTBySegment

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE, 
    STATE_MOVE_RECIPE, 
    STATE_MOVE_RETREAT, 
    STATE_MOVE_HOME
)
from .segment_runner import SegmentRunner, StepSpec
from .process_contract import ReqKey, create_header_frame_id, parse_key_from_traj_header
from .process_serialization import jt_dict_to_robottrajectory_msg, jt_msg_to_dict_normalized

_LOG = logging.getLogger("tabs.process.optimize_sm")

class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    ROLE = "optimize"

    def __init__(self, *, recipe: Any, ros: Any, run_result: RunResult, parent: Optional[QtCore.QObject] = None, max_retries: int = 0, side: str = "top") -> None:
        super().__init__(recipe=recipe, ros=ros, run_result=run_result, parent=parent, max_retries=max_retries)
        self._baseline_jt_by_segment: Dict[str, Dict[str, Any]] = {}
        self._planned_loaded_yaml: Dict[str, Any] = {}
        
        self._opt_msg_by_key: Dict[Tuple[str, int, str, str], Any] = {}
        self._opt_sig_connected = False

        self._runner = SegmentRunner(ros=self._ros, parent=self)
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)
        
        self._active_seg: str = ""
        self._connect_opt_sig()

    def _connect_opt_sig(self) -> None:
        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp else None
            if sig and hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self._on_opt_capture)
                self._opt_sig_connected = True
        except Exception: pass

    @QtCore.pyqtSlot(object)
    def _on_opt_capture(self, obj: object) -> None:
        if not obj: return
        try: key = parse_key_from_traj_header(obj)
        except Exception: return
        if str(key.run) == self._run_id and str(key.op) == "optimize_trajectory":
            self._opt_msg_by_key[key.as_tuple()] = obj

    def _cleanup(self) -> None:
        if self._opt_sig_connected:
            try: self._ros.moveitpy.signals.optimizedTrajectoryChanged.disconnect(self._on_opt_capture)
            except Exception: pass
        super()._cleanup()

    def _prepare_run(self) -> bool:
        self._opt_msg_by_key.clear()
        
        pt = getattr(self._recipe, "planned_traj", None)
        if not pt: return False
        try: d = pt.to_yaml_dict()
        except Exception: return False
        
        self._planned_loaded_yaml = d
        self._baseline_jt_by_segment = {}
        segs = d.get("segments", {})
        for s in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            if segs.get(s): self._baseline_jt_by_segment[s] = segs[s]

        if not callable(getattr(self._ros, "moveit_optimize_trajectory", None)): return False
        return True

    def _segment_exists(self, seg_name: str) -> bool:
        return seg_name in self._baseline_jt_by_segment

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)
        if seg_name not in self._baseline_jt_by_segment:
            self.segment_done()
            return
        
        baseline = self._baseline_jt_by_segment[seg_name]
        req_id = int(time.time()*1000)
        
        # Step 1: Optimize
        hdr_opt = create_header_frame_id(self._run_id, req_id, seg_name, "optimize_trajectory")
        try: rt_base = jt_dict_to_robottrajectory_msg(baseline, hdr_opt)
        except Exception as e:
            self.segment_error(f"Opt build failed: {e}")
            return
            
        step_opt = self._runner.make_optimize_trajectory_step_bound(
            run=self._run_id, req_id=req_id, seg=seg_name, traj=rt_base, label=f"{seg_name}:opt"
        )
        
        # Step 2: Execute
        exec_key = ReqKey(self._run_id, req_id, seg_name, "execute_trajectory")
        opt_key = ReqKey(self._run_id, req_id, seg_name, "optimize_trajectory")
        
        def _send_exec() -> None:
            opt_msg = self._opt_msg_by_key.get(opt_key.as_tuple())
            if not opt_msg: raise RuntimeError("Missing optimizedTrajectory result")
            
            jt_dict = jt_msg_to_dict_normalized(opt_msg)
            if not jt_dict: raise RuntimeError("Optimized Trajectory empty")
            
            hdr_exec = create_header_frame_id(self._run_id, req_id, seg_name, "execute_trajectory")
            rt_exec = jt_dict_to_robottrajectory_msg(jt_dict, hdr_exec)
            self._ros.moveit_execute_trajectory(rt_exec)

        step_exec = StepSpec(
            label=f"{seg_name}:exec",
            send=_send_exec,
            expect_result_key=exec_key,
            expect_status="EXECUTED:OK",
            expect_traj_kind="executed",
            expect_traj_key=exec_key
        )
        self._runner.run([step_opt, step_exec])

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
        
        planned_opt = self._build_trajectory_yaml(self._traj_optimized, which_op="optimize_trajectory")
        
        final_planned = copy.deepcopy(self._planned_loaded_yaml)
        if planned_opt.get("segments"):
            final_planned["segments"].update(planned_opt["segments"])

        executed = self._build_trajectory_yaml(self._traj_executed, which_op="execute_trajectory")

        try: self._recipe.planned_traj = JTBySegment.from_yaml_dict(final_planned)
        except Exception: pass
        try: self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception: self._recipe.executed_traj = None

        self._rr.set_planned(traj=final_planned)
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()