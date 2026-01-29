# -*- coding: utf-8 -*-
# File: src/tabs/process/process_panel/validate_statemachine.py
from __future__ import annotations

import copy
import logging
import time
from typing import Any, Dict, List, Optional, Tuple

from PyQt6 import QtCore
from geometry_msgs.msg import PoseStamped  # type: ignore

from model.recipe.recipe_run_result import RunResult
from model.spray_paths.draft import Draft, PoseQuat
from model.spray_paths.trajectory import JTBySegment

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
    SEG_ORDER,
)

from .segment_runner import SegmentRunner, StepSpec

_LOG = logging.getLogger("tabs.process.validate_statemachine")


class ProcessValidateStatemachine(BaseProcessStatemachine):
    """
    Validate (STRICT, sequential) using SegmentRunner.

    NEW (collect-only Base):
      - BaseProcessStatemachine is ONLY transport wiring + stashes.
      - Per segment we start ONE SegmentRunner that runs steps sequentially:
            [plan_pose(p1), exec(p1), plan_pose(p2), exec(p2), ...]
      - Segment completion is controlled ONLY by:
            SegmentRunner.finished -> self.segment_done()
            SegmentRunner.error    -> self.segment_error(msg)

    Contract assumptions (Variant-A):
      - plan_pose => motion_result status "PLANNED:OK" and plannedTrajectory with key(op="plan_pose")
      - execute_last_planned => motion_result status "EXECUTED:OK" and executedTrajectory with key(op="execute")
    """

    ROLE = "validate"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 0,  # API compat; unused
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

        self._frame: str = "substrate"
        self._speed_mm_s: float = 200.0
        self._planner_cfg: Dict[str, Any] = {}

        self._poses_by_seg: Dict[str, List[PoseQuat]] = {}
        self._run_id: str = ""

        self._runner = SegmentRunner(ros=self._ros, parent=self, max_goal_rejected_retries=int(max_retries))
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)

        # guard against late callbacks
        self._active_seg: str = ""

        _ = max_retries  # API compat

    # ------------------------------------------------------------------
    # STOP
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        try:
            self._runner.request_stop()
        except Exception:
            pass
        super().request_stop()

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # run id: stable for this run
        try:
            self._run_id = str(self._rr.run_id or "").strip()
        except Exception:
            self._run_id = ""
        if not self._run_id:
            self._run_id = f"run_{int(time.time() * 1000)}"

        # speed (best-effort)
        self._speed_mm_s = 200.0
        try:
            params = self._recipe.parameters if isinstance(self._recipe.parameters, dict) else {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            self._speed_mm_s = 200.0

        # planner cfg (optional)
        if not self._apply_planner_config():
            return False

        # Draft must exist (STRICT)
        d: Optional[Draft]
        try:
            d = self._recipe.draft
        except Exception:
            d = None
        if d is None:
            self.segment_error("Validate: recipe.draft ist None (kein Draft geladen/kompiliert).")
            return False

        # segment-specific extraction via Draft API (STRICT)
        try:
            pre = d.poses_quat_segment(self._side, STATE_MOVE_PREDISPENSE)
            recipe = d.poses_quat_segment(self._side, STATE_MOVE_RECIPE)
            ret = d.poses_quat_segment(self._side, STATE_MOVE_RETREAT)
        except Exception as e:
            self.segment_error(f"Validate: Draft-Segmentierung fehlgeschlagen (side='{self._side}'): {e}")
            return False

        if not pre:
            self.segment_error(f"Validate: predispense fehlt (side='{self._side}').")
            return False
        if not recipe:
            self.segment_error(f"Validate: poses_quat (recipe) ist leer (side='{self._side}').")
            return False
        if not ret:
            self.segment_error(f"Validate: retreat fehlt (side='{self._side}').")
            return False

        self._poses_by_seg.clear()

        # convention: pre/ret are single move poses, recipe can be long
        self._poses_by_seg[STATE_MOVE_PREDISPENSE] = list(pre[:1])
        self._poses_by_seg[STATE_MOVE_RECIPE] = list(recipe)
        self._poses_by_seg[STATE_MOVE_RETREAT] = list(ret[:1])
        self._poses_by_seg[STATE_MOVE_HOME] = []

        return True

    def _apply_planner_config(self) -> bool:
        cfg: Dict[str, Any] = {}
        try:
            planner_sect = self._recipe.planner if isinstance(self._recipe.planner, dict) else {}
            v = planner_sect.get("validate")
            if not isinstance(v, dict):
                v = planner_sect.get("move")
            if isinstance(v, dict):
                pipeline = str(v.get("pipeline") or "").strip()
                planner_id = str(v.get("planner_id") or "").strip()
                params = v.get("params") or {}
                if planner_id:
                    cfg = {
                        "role": "validate",
                        "pipeline": pipeline,
                        "planner_id": planner_id,
                        "params": params if isinstance(params, dict) else {},
                    }
        except Exception:
            cfg = {}

        self._planner_cfg = cfg

        if not cfg:
            _LOG.info("Validate: Keine Planner-Config gefunden. Nutze Default.")
            return True

        try:
            self._ros.moveit_set_planner_cfg(cfg)
            _LOG.info("Validate: PlannerCfg gesetzt: %s/%s", cfg.get("pipeline"), cfg.get("planner_id"))
        except Exception as e:
            self.segment_error(f"Validate: Fehler beim Senden der Planner-Config: {e}")
            return False

        return True

    # ------------------------------------------------------------------
    # Segment existence
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT):
            return bool(self._poses_by_seg.get(seg_name))
        return True  # HOME always exists

    # ------------------------------------------------------------------
    # Segment entry: build step list and run sequentially
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)

        # push speed (best-effort)
        try:
            self._ros.moveit_set_speed_mm_s(float(self._speed_mm_s))
        except Exception:
            pass

        # stop any previous runner
        try:
            self._runner.request_stop()
        except Exception:
            pass

        steps = self._build_steps_for_segment(seg_name)

        # Empty segment => immediately done (Base does not auto-advance)
        if not steps:
            self.segment_done()
            return

        self._runner.run(steps)

    def _build_steps_for_segment(self, seg: str) -> List[StepSpec]:
        seg = str(seg)
        steps: List[StepSpec] = []

        def new_id(i: int) -> int:
            # monotonic-ish within this call; must be int
            return int(time.time() * 1000) + int(i)

        if seg in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RETREAT, STATE_MOVE_RECIPE):
            poses = list(self._poses_by_seg.get(seg) or [])
            if not poses:
                return []

            for i, pq in enumerate(poses):
                rid = new_id(i)
                ps = self._posequat_to_posestamped(pq)

                # plan + execute share same id, different op
                steps.append(
                    self._runner.make_plan_pose_step_bound(
                        run=self._run_id,
                        req_id=rid,
                        seg=seg,
                        pose=ps,
                        label=f"{seg}:plan[{i}]",
                    )
                )
                steps.append(
                    self._runner.make_execute_last_planned_step_bound(
                        run=self._run_id,
                        req_id=rid,
                        seg=seg,
                        label=f"{seg}:exec[{i}]",
                    )
                )
            return steps

        if seg == STATE_MOVE_HOME:
            ps_home = self._require_home_pose()
            rid = new_id(0)

            steps.append(
                self._runner.make_plan_pose_step_bound(
                    run=self._run_id,
                    req_id=rid,
                    seg=seg,
                    pose=ps_home,
                    label="HOME:plan",
                )
            )
            steps.append(
                self._runner.make_execute_last_planned_step_bound(
                    run=self._run_id,
                    req_id=rid,
                    seg=seg,
                    label="HOME:exec",
                )
            )
            return steps

        raise RuntimeError(f"Validate: Unknown segment {seg!r}")

    def _posequat_to_posestamped(self, pq: PoseQuat) -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = str(self._frame or "substrate")
        ps.pose.position.x = float(pq.x) / 1000.0
        ps.pose.position.y = float(pq.y) / 1000.0
        ps.pose.position.z = float(pq.z) / 1000.0
        ps.pose.orientation.x = float(pq.qx)
        ps.pose.orientation.y = float(pq.qy)
        ps.pose.orientation.z = float(pq.qz)
        ps.pose.orientation.w = float(pq.qw)
        return ps

    def _require_home_pose(self) -> PoseStamped:
        ps: Optional[PoseStamped]
        try:
            ps = self._ros.poses_state.st.home  # adapter exposes .st
        except Exception:
            ps = None
        if ps is None:
            raise RuntimeError("Validate: HOME Pose fehlt (ros.poses_state.st.home ist None).")
        if not str(ps.header.frame_id or "").strip():
            raise RuntimeError("Validate: HOME PoseStamped.header.frame_id ist leer.")
        return ps

    # ------------------------------------------------------------------
    # Runner callbacks -> Base transition API
    # ------------------------------------------------------------------

    def _on_runner_finished(self) -> None:
        if self._stop_requested or self._machine is None or self._error_msg:
            return
        # guard: ignore late finished from old segment
        if self.current_segment() != self._active_seg:
            return
        self.segment_done()

    def _on_runner_error(self, msg: str) -> None:
        # guard: ignore late error from old segment
        if self._stop_requested or self._machine is None or self._error_msg:
            return
        if self.current_segment() != self._active_seg:
            return
        self.segment_error(str(msg or "Runner ERROR"))

    # ------------------------------------------------------------------
    # Finish: pack JTBySegment YAML v1 from Base stashes (Variant-A keys)
    # ------------------------------------------------------------------

    def _on_finished(self) -> None:
        planned = self._jt_by_segment_yaml(which="planned")
        executed = self._jt_by_segment_yaml(which="executed")

        p_segs = planned.get("segments") if isinstance(planned, dict) else None
        if not (isinstance(p_segs, dict) and p_segs):
            self.notifyError.emit(
                "Validate finished, but planned trajectory capture is empty. "
                "Check planned_trajectory publish + JSON key tags in header.frame_id."
            )
            self._cleanup()
            return

        try:
            self._recipe.planned_traj = JTBySegment.from_yaml_dict(planned)
        except Exception as e:
            self.notifyError.emit(f"Validate: failed to write recipe.planned_traj: {e}")
            self._cleanup()
            return

        try:
            self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception:
            self._recipe.executed_traj = None

        # store into RunResult (meta-only, strict)
        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))

        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # JTBySegment helpers (Variant-A: sort by id, concat time_from_start)
    # ------------------------------------------------------------------

    @staticmethod
    def _id_sort_key(rid: int) -> int:
        try:
            return int(rid)
        except Exception:
            return 2**63 - 1

    def _jt_msg_to_dict(self, traj_msg: Any) -> Optional[Dict[str, Any]]:
        if traj_msg is None:
            return None
        try:
            jt = traj_msg.joint_trajectory
            jn = [str(x) for x in list(jt.joint_names or [])]
            pts_msg = list(jt.points or [])
            if not jn or not pts_msg:
                return None
            pts: List[Dict[str, Any]] = []
            for p in pts_msg:
                tfs = getattr(p, "time_from_start", None)
                sec = int(getattr(tfs, "sec", 0)) if tfs is not None else 0
                nsec = int(getattr(tfs, "nanosec", 0)) if tfs is not None else 0
                pts.append(
                    {
                        "positions": [float(x) for x in list(p.positions or [])],
                        "time_from_start": [sec, nsec],
                    }
                )
            return {"joint_names": jn, "points": pts}
        except Exception:
            return None

    def _collect_steps_for_seg(self, *, which: str, seg: str) -> List[Tuple[int, Dict[str, Any]]]:
        out: List[Tuple[int, Dict[str, Any]]] = []

        if which == "planned":
            store = self._traj_planned or {}
            want_op = "plan_pose"
        else:
            store = self._traj_executed or {}
            want_op = "execute"

        for (run, rid, s, op), msg in list(store.items()):
            if str(run) != str(self._run_id):
                continue
            if str(s) != str(seg):
                continue
            if str(op) != str(want_op):
                continue
            d = self._jt_msg_to_dict(msg)
            if d:
                out.append((int(rid), d))

        out.sort(key=lambda x: self._id_sort_key(x[0]))
        return out

    def _concat_jt_dicts(self, dicts: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not dicts:
            return None
        base_names = list(dicts[0].get("joint_names") or [])
        merged_pts: List[Dict[str, Any]] = []
        t_offset_ns = 0
        last_global_ns = -1

        for d in dicts:
            pts = d.get("points") or []
            for p in pts:
                tfs = p.get("time_from_start") or [0, 0]
                t_local_ns = int(tfs[0]) * 1_000_000_000 + int(tfs[1])
                t_global_ns = t_offset_ns + t_local_ns
                if t_global_ns <= last_global_ns:
                    t_global_ns = last_global_ns + 1_000_000  # +1ms safety
                q = copy.deepcopy(p)
                q["time_from_start"] = [t_global_ns // 1_000_000_000, t_global_ns % 1_000_000_000]
                merged_pts.append(q)
                last_global_ns = t_global_ns
            t_offset_ns = last_global_ns

        return {"joint_names": base_names, "points": merged_pts}

    def _jt_by_segment_yaml(self, *, which: str) -> Dict[str, Any]:
        segments: Dict[str, Any] = {}
        for seg in SEG_ORDER:
            steps = self._collect_steps_for_seg(which=which, seg=seg)
            jts = [d for _, d in steps]
            if not jts:
                continue
            merged = self._concat_jt_dicts(jts)
            if merged:
                segments[seg] = merged
        return {"version": 1, "segments": segments}