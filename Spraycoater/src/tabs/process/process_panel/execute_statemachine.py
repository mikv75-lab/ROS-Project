# -*- coding: utf-8 -*-
# File: tabs/process/execute_statemachine.py
from __future__ import annotations

import copy
import json
import logging
import time
from typing import Any, Dict, List, Optional, Tuple

from PyQt6 import QtCore

from plc.plc_client import PlcClientBase
from model.recipe.recipe_run_result import RunResult
from model.spray_paths.trajectory import JTBySegment

from builtin_interfaces.msg import Duration as RosDuration  # type: ignore
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
    SEG_ORDER,
)

from .segment_runner import SegmentRunner, StepSpec

_LOG = logging.getLogger("tabs.process.execute_sm")


class ProcessExecuteStatemachine(BaseProcessStatemachine):
    """
    Execute run (STRICT, SegmentRunner) — Variant-A external replay.

    NEW (collect-only Base):
      - BaseProcessStatemachine is ONLY transport wiring + stashes.
      - Per segment we start ONE SegmentRunner with ONE step:
            [execute_trajectory(rt_segment)]
      - Segment completion is controlled ONLY by:
            SegmentRunner.finished -> self.segment_done()
            SegmentRunner.error    -> self.segment_error(msg)

    Contract assumptions (Variant-A):
      - execute_trajectory => motion_result status "EXECUTED:OK"
      - executedTrajectoryChanged(rt) with the SAME key encoded in header.frame_id JSON

    Notes:
      - Input is recipe.planned_traj (JTBySegment YAML v1) from Validate.
      - We do NOT plan anything here.
      - We publish ros.moveit_execute_trajectory(rt_msg) once per segment.
    """

    ROLE = "execute"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        plc: PlcClientBase | None,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 0,  # API compat; unused in new model
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            run_result=run_result,
            parent=parent,
            max_retries=max_retries,
        )
        self._plc = plc
        self._side: str = str(side or "top")

        self._run_id: str = ""

        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}
        self._planned_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        self._runner = SegmentRunner(ros=self._ros, parent=self)
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)

        self._active_seg: str = ""

        _ = max_retries  # API compat

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # run id
        self._run_id = str(getattr(self._rr, "run_id", "") or "").strip()
        if not self._run_id:
            self._run_id = f"run_{int(time.time() * 1000)}"

        # best-effort: take side from recipe params
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", self._side) or self._side)
        except Exception:
            pass

        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self.segment_error("Execute: recipe.planned_traj fehlt (bitte erst Validate ausführen).")
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self.segment_error("Execute: planned_traj API fehlt (to_yaml_dict).")
            return False

        d = fn()
        if not isinstance(d, dict):
            self.segment_error("Execute: planned_traj daten ungültig (non-dict).")
            return False

        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self.segment_error(f"Execute: planned_traj YAML invalid: {e}")
            return False

        self._planned_loaded_yaml = d

        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        try:
            self._planned_jt_by_segment = {}
            for seg_name, present in self._yaml_segments_present.items():
                if not present:
                    continue
                self._planned_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment_normalized(seg_name)
        except Exception as e:
            self.segment_error(f"Execute: build planned JT cache failed: {e}")
            return False

        # STRICT: require external replay API
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self.segment_error("Execute: ros.moveit_execute_trajectory(traj_msg) fehlt (Variant-A external replay).")
            return False

        return True

    # ------------------------------------------------------------------
    # YAML validation / normalization
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_jtbysegment_yaml_v1(d: Dict[str, Any]) -> None:
        if not isinstance(d, dict):
            raise ValueError("root is not dict")
        if int(d.get("version", 0)) != 1:
            raise ValueError("version must be 1")
        segs = d.get("segments")
        if not isinstance(segs, dict) or not segs:
            raise ValueError("segments missing/empty")

    def _segment_has_points(self, seg_name: str) -> bool:
        segs = self._planned_loaded_yaml.get("segments", {})
        seg = segs.get(seg_name) if isinstance(segs, dict) else None
        if not isinstance(seg, dict):
            return False
        pts = seg.get("points")
        return isinstance(pts, list) and len(pts) >= 2

    def _jt_dict_from_yaml_segment_normalized(self, seg_name: str) -> Dict[str, Any]:
        segs = self._planned_loaded_yaml.get("segments", {})
        seg = segs.get(seg_name) if isinstance(segs, dict) else None
        if not isinstance(seg, dict):
            raise ValueError(f"segment {seg_name} missing")
        jn = seg.get("joint_names") or []
        pts = seg.get("points") or []
        if not isinstance(jn, list) or not jn:
            raise ValueError(f"segment {seg_name}: joint_names missing/empty")
        if not isinstance(pts, list) or len(pts) < 2:
            raise ValueError(f"segment {seg_name}: points missing/<2")

        out_pts: List[Dict[str, Any]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            tfs = p.get("time_from_start")
            if not isinstance(pos, list) or len(pos) != len(jn):
                continue
            if not isinstance(tfs, (list, tuple)) or len(tfs) < 2:
                continue
            out_pts.append(
                {
                    "positions": [float(x) for x in list(pos)],
                    "time_from_start": {"sec": int(tfs[0]), "nanosec": int(tfs[1])},
                }
            )

        if len(out_pts) < 2:
            raise ValueError(f"Execute: segment '{seg_name}' has <2 valid points after normalization")

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    # ------------------------------------------------------------------
    # Segment gating
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        return bool(self._yaml_segments_present.get(seg_name, False))

    # ------------------------------------------------------------------
    # Segment enter (core): ONE runner per segment, ONE step per segment
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)

        present = bool(self._yaml_segments_present.get(seg_name, False))
        if not present:
            self.segment_done()
            return

        planned_jt = self._planned_jt_by_segment.get(seg_name)
        if not isinstance(planned_jt, dict):
            self.segment_error(f"Execute: missing planned JT for seg={seg_name}")
            return

        # Build RobotTrajectoryMsg with embedded correlation JSON in header.frame_id.
        # IMPORTANT: SegmentRunner expects ReqKey(run,id,seg,op). We generate a stable-ish int id.
        req_id = int(time.time() * 1000)

        try:
            rt = self._jt_dict_to_robottrajectory_msg(
                planned_jt,
                run=self._run_id,
                req_id=req_id,
                seg=str(seg_name),
                op="execute_trajectory",
            )
        except Exception as e:
            self.segment_error(f"Execute: build RobotTrajectory failed for seg={seg_name}: {e}")
            return

        steps: List[StepSpec] = [
            self._runner.make_execute_trajectory_step_bound(
                run=self._run_id,
                req_id=req_id,
                seg=str(seg_name),
                traj=rt,
                label=f"{seg_name}:execute_trajectory",
            )
        ]
        self._runner.run(steps)

    def _jt_dict_to_robottrajectory_msg(
        self,
        jt_dict: Dict[str, Any],
        *,
        run: str,
        req_id: int,
        seg: str,
        op: str,
    ) -> RobotTrajectoryMsg:
        jn = jt_dict.get("joint_names") or []
        pts = jt_dict.get("points") or []
        if not isinstance(jn, list) or not jn:
            raise ValueError("jt_dict.joint_names missing/empty")
        if not isinstance(pts, list) or len(pts) < 2:
            raise ValueError("jt_dict.points missing/<2")

        # STRICT Variant-A key tag in header.frame_id JSON
        # Acceptable shapes:
        #   {"key": {"run":..., "id":..., "seg":..., "op":...}}
        # We send the nested form to be unambiguous.
        frame_id = json.dumps(
            {"key": {"run": str(run), "id": int(req_id), "seg": str(seg), "op": str(op)}},
            ensure_ascii=False,
        )

        jt = JointTrajectory()
        jt.joint_names = [str(x) for x in jn]
        jt.header.frame_id = frame_id

        out_pts: List[JointTrajectoryPoint] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            tfs = p.get("time_from_start")
            if not isinstance(pos, list) or len(pos) != len(jn):
                continue
            if not isinstance(tfs, dict):
                continue

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]

            d = RosDuration()
            d.sec = int(tfs.get("sec", 0))
            d.nanosec = int(tfs.get("nanosec", 0))
            pt.time_from_start = d

            out_pts.append(pt)

        if len(out_pts) < 2:
            raise ValueError("RobotTrajectory would have <2 valid points")

        jt.points = out_pts

        rt = RobotTrajectoryMsg()
        rt.joint_trajectory = jt
        return rt

    # ------------------------------------------------------------------
    # Runner callbacks -> Base transition API
    # ------------------------------------------------------------------

    def _on_runner_finished(self) -> None:
        if self._stop_requested or self._machine is None or self._error_msg:
            return
        if self.current_segment() != self._active_seg:
            return
        self.segment_done()

    def _on_runner_error(self, msg: str) -> None:
        if self._stop_requested or self._machine is None or self._error_msg:
            return
        if self.current_segment() != self._active_seg:
            return
        self.segment_error(str(msg or "Runner ERROR"))

    # ------------------------------------------------------------------
    # Final payload
    # ------------------------------------------------------------------

    def _on_finished(self) -> None:
        planned = copy.deepcopy(self._planned_loaded_yaml)
        executed = self._jt_by_segment_yaml(which="executed")

        # Keep recipe fields aligned with the rest of your pipeline
        try:
            self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception:
            self._recipe.executed_traj = None

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # JTBySegment packing (executed from Base stash; Variant-A keys)
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
            jt = getattr(traj_msg, "joint_trajectory", None)
            if jt is None:
                return None
            jn = [str(x) for x in list(getattr(jt, "joint_names") or [])]
            pts_msg = list(getattr(jt, "points") or [])
            if not jn or not pts_msg:
                return None
            pts: List[Dict[str, Any]] = []
            for p in pts_msg:
                tfs = getattr(p, "time_from_start", None)
                sec = int(getattr(tfs, "sec", 0)) if tfs is not None else 0
                nsec = int(getattr(tfs, "nanosec", 0)) if tfs is not None else 0
                pts.append(
                    {
                        "positions": [float(x) for x in list(getattr(p, "positions", []) or [])],
                        "time_from_start": [sec, nsec],
                    }
                )
            return {"joint_names": jn, "points": pts}
        except Exception:
            return None

    def _collect_steps_for_seg(self, *, seg: str) -> List[Tuple[int, Dict[str, Any]]]:
        out: List[Tuple[int, Dict[str, Any]]] = []
        store = getattr(self, "_traj_executed", {}) or {}

        # SegmentRunner uses op="execute_trajectory" for replay.
        want_op = "execute_trajectory"

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
        _ = which  # only "executed" used here; kept for API parity
        segments: Dict[str, Any] = {}
        for seg in SEG_ORDER:
            steps = self._collect_steps_for_seg(seg=seg)
            jts = [d for _, d in steps]
            if not jts:
                continue
            merged = self._concat_jt_dicts(jts)
            if merged:
                segments[seg] = merged
        return {"version": 1, "segments": segments}
