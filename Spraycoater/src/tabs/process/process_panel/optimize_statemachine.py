# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import copy
import json
import logging
import time
from typing import Any, Dict, List, Optional, Tuple

from PyQt6 import QtCore

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
from .base_statemachine import ReqKey, _parse_key_from_traj_header  # reuse strict parser

_LOG = logging.getLogger("tabs.process.optimize_sm")


class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    """
    Optimize run (STRICT, SegmentRunner) — Variant-A (external optimize + external replay).

    NEW (collect-only Base):
      - BaseProcessStatemachine is ONLY transport wiring + stashes.
      - Per segment we run ONE SegmentRunner with TWO steps:
            1) optimize_trajectory(baseline_rt)   op="optimize_trajectory" -> OPTIMIZED:OK + optimizedTrajectory
            2) execute_trajectory(optimized_rt)   op="execute_trajectory"  -> EXECUTED:OK  + executedTrajectory
      - Segment completion is controlled ONLY by:
            SegmentRunner.finished -> self.segment_done()
            SegmentRunner.error    -> self.segment_error(msg)

    Input:
      - recipe.planned_traj (JTBySegment YAML v1) from Validate (baseline).

    Output:
      - planned := optimized JTBySegment YAML v1 (per segment optimized if available, else baseline)
      - executed := captured executed trajectories (from Base stash, keyed Variant-A)
    """

    ROLE = "optimize"

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
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

        self._side: str = str(side or "top")
        self._run_id: str = ""

        # baseline input (raw YAML v1) + normalized JT dicts
        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}
        self._baseline_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        # optimized per segment (JT dict + optional raw msg)
        self._optimized_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        # per segment active key (Variant-A: run,id,seg,op)
        self._seg_req_id: Dict[str, int] = {}
        self._active_seg: str = ""

        # optimized message stash keyed by (run,id,seg,op="optimize_trajectory")
        self._opt_msg_by_key: Dict[Tuple[str, int, str, str], Any] = {}

        # runner
        self._runner = SegmentRunner(ros=self._ros, parent=self)
        self._runner.finished.connect(self._on_runner_finished)
        self._runner.error.connect(self._on_runner_error)

        # additionally listen to optimizedTrajectoryChanged to capture the optimized RT payload
        self._opt_sig_connected: bool = False
        self._connect_optimized_signal()

        _ = max_retries  # API compat

    # ------------------------------------------------------------------
    # Wiring: capture optimized trajectories
    # ------------------------------------------------------------------

    def _connect_optimized_signal(self) -> None:
        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp is not None else None
            if sig is None:
                return
            if hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj_capture)
                self._opt_sig_connected = True
        except Exception:
            self._opt_sig_connected = False

    def _disconnect_optimized_signal(self) -> None:
        if not self._opt_sig_connected:
            return
        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp is not None else None
            if sig is None:
                return
            if hasattr(sig, "optimizedTrajectoryChanged"):
                try:
                    sig.optimizedTrajectoryChanged.disconnect(self._on_optimized_traj_capture)
                except Exception:
                    pass
        finally:
            self._opt_sig_connected = False

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj_capture(self, obj: object) -> None:
        """
        Capture optimized trajectory messages so the execute-step can replay them.

        SegmentRunner's StepRunner *waits* for optimizedTrajectoryChanged, but does not expose
        the message. We stash the message keyed by the Variant-A ReqKey extracted from header.frame_id.
        """
        if obj is None:
            return
        try:
            key = _parse_key_from_traj_header(obj)  # ReqKey(run,id,seg,op)
        except Exception:
            return
        if str(key.run) != str(self._run_id):
            return
        if str(key.op) != "optimize_trajectory":
            return
        self._opt_msg_by_key[key.as_tuple()] = obj

        # Also precompute the JT dict for planned output (cheap + deterministic)
        seg = str(key.seg)
        jt = self._jt_msg_to_jt_dict(obj)
        if isinstance(jt, dict):
            self._optimized_jt_by_segment[seg] = copy.deepcopy(jt)

    # ------------------------------------------------------------------
    # STOP / Cleanup
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        try:
            self._runner.request_stop()
        except Exception:
            pass
        super().request_stop()

    def _cleanup(self) -> None:
        self._disconnect_optimized_signal()
        super()._cleanup()

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

        self._optimized_jt_by_segment.clear()
        self._seg_req_id.clear()
        self._opt_msg_by_key.clear()

        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self.segment_error("Optimize: recipe.planned_traj fehlt (bitte erst Validate ausführen).")
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self.segment_error("Optimize: planned_traj API fehlt (to_yaml_dict).")
            return False

        d = fn()
        if not isinstance(d, dict):
            self.segment_error("Optimize: planned_traj daten ungültig (non-dict).")
            return False

        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self.segment_error(f"Optimize: planned_traj YAML invalid: {e}")
            return False

        self._planned_loaded_yaml = d

        # segment availability
        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        # normalize baseline JT dicts
        try:
            self._baseline_jt_by_segment = {}
            for seg_name, present in self._yaml_segments_present.items():
                if not present:
                    continue
                self._baseline_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment_normalized(seg_name)
        except Exception as e:
            self.segment_error(f"Optimize: build baseline JT cache failed: {e}")
            return False

        # STRICT: require external APIs
        if not callable(getattr(self._ros, "moveit_optimize_trajectory", None)):
            self.segment_error("Optimize: ros.moveit_optimize_trajectory(traj_msg) fehlt.")
            return False
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self.segment_error("Optimize: ros.moveit_execute_trajectory(traj_msg) fehlt.")
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
            raise ValueError(f"Optimize: segment '{seg_name}' has <2 valid points after normalization")

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    # ------------------------------------------------------------------
    # Segment gating
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        return bool(self._yaml_segments_present.get(seg_name, False))

    # ------------------------------------------------------------------
    # Segment enter: ONE runner per segment, TWO steps per segment
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        self._active_seg = str(seg_name)

        present = bool(self._yaml_segments_present.get(seg_name, False))
        if not present:
            self.segment_done()
            return

        baseline_jt = self._baseline_jt_by_segment.get(seg_name)
        if not isinstance(baseline_jt, dict):
            self.segment_error(f"Optimize: missing baseline JT for seg={seg_name}")
            return

        # one req_id shared across optimize + execute steps, ops differ
        req_id = int(time.time() * 1000)
        self._seg_req_id[str(seg_name)] = int(req_id)

        # Build baseline RobotTrajectory (tagged as optimize_trajectory)
        try:
            rt_baseline = self._jt_dict_to_robottrajectory_msg(
                baseline_jt,
                run=self._run_id,
                req_id=req_id,
                seg=str(seg_name),
                op="optimize_trajectory",
            )
        except Exception as e:
            self.segment_error(f"Optimize: build baseline RobotTrajectory failed seg={seg_name}: {e}")
            return

        # Step 1: optimize (expects OPTIMIZED:OK + optimizedTrajectory with same ReqKey(op=optimize_trajectory))
        step_opt = self._runner.make_optimize_trajectory_step_bound(
            run=self._run_id,
            req_id=req_id,
            seg=str(seg_name),
            traj=rt_baseline,
            label=f"{seg_name}:optimize_trajectory",
        )

        # Step 2: execute optimized (expects EXECUTED:OK + executedTrajectory with ReqKey(op=execute_trajectory))
        # We cannot know optimized RT at build-time, so send() loads it from our stash.
        exec_key = ReqKey(run=str(self._run_id), id=int(req_id), seg=str(seg_name), op="execute_trajectory")
        opt_key = ReqKey(run=str(self._run_id), id=int(req_id), seg=str(seg_name), op="optimize_trajectory")

        def _send_exec() -> None:
            # Find optimized RT message captured from optimizedTrajectoryChanged
            opt_msg = self._opt_msg_by_key.get(opt_key.as_tuple())
            if opt_msg is None:
                raise RuntimeError(
                    f"Optimize: missing optimizedTrajectory for seg={seg_name} "
                    f"(key={opt_key.token_base()})"
                )
            jt_opt = self._jt_msg_to_jt_dict(opt_msg)
            if not isinstance(jt_opt, dict):
                raise RuntimeError(f"Optimize: optimizedTrajectory invalid/empty for seg={seg_name}")

            # Keep planned output cache (idempotent)
            self._optimized_jt_by_segment[str(seg_name)] = copy.deepcopy(jt_opt)

            # Build execute RT with execute_trajectory key
            rt_exec = self._jt_dict_to_robottrajectory_msg(
                jt_opt,
                run=self._run_id,
                req_id=req_id,
                seg=str(seg_name),
                op="execute_trajectory",
            )
            self._ros.moveit_execute_trajectory(rt_exec)

        step_exec = StepSpec(
            label=f"{seg_name}:execute_trajectory",
            send=_send_exec,
            expect_result_key=exec_key,
            expect_status="EXECUTED:OK",
            expect_traj_kind="executed",
            expect_traj_key=exec_key,
        )

        self._runner.run([step_opt, step_exec])

    # ------------------------------------------------------------------
    # JT <-> RobotTrajectory helpers (Variant-A JSON in header.frame_id)
    # ------------------------------------------------------------------

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

    def _jt_msg_to_jt_dict(self, traj_msg: Any) -> Optional[Dict[str, Any]]:
        """
        Convert RobotTrajectoryMsg -> JT dict:
          {"joint_names":[...], "points":[{"positions":[...], "time_from_start":{"sec":..,"nanosec":..}}, ...]}
        """
        if traj_msg is None:
            return None
        try:
            jt = getattr(traj_msg, "joint_trajectory", None)
            if jt is None:
                return None
            jn = [str(x) for x in list(getattr(jt, "joint_names") or [])]
            pts_msg = list(getattr(jt, "points") or [])
            if not jn or len(pts_msg) < 2:
                return None

            pts: List[Dict[str, Any]] = []
            for p in pts_msg:
                tfs = getattr(p, "time_from_start", None)
                sec = int(getattr(tfs, "sec", 0)) if tfs is not None else 0
                nsec = int(getattr(tfs, "nanosec", 0)) if tfs is not None else 0
                pts.append(
                    {
                        "positions": [float(x) for x in list(getattr(p, "positions", []) or [])],
                        "time_from_start": {"sec": sec, "nanosec": nsec},
                    }
                )
            return {"joint_names": jn, "points": pts} if len(pts) >= 2 else None
        except Exception:
            return None

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

    def _build_planned_payload_yaml_v1(self) -> Dict[str, Any]:
        """
        planned_traj YAML v1 output:
          - if segment optimized -> optimized JT (converted to [sec,nsec] pairs)
          - else baseline
        """
        segs_out: Dict[str, Any] = {}
        for seg in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            if not self._yaml_segments_present.get(seg, False):
                continue
            jt = self._optimized_jt_by_segment.get(seg) or self._baseline_jt_by_segment.get(seg)
            if not isinstance(jt, dict):
                continue

            pts_out = []
            for p in list(jt.get("points") or []):
                if not isinstance(p, dict):
                    continue
                tfs = p.get("time_from_start")
                if isinstance(tfs, dict):
                    t_pair = [int(tfs.get("sec", 0)), int(tfs.get("nanosec", 0))]
                else:
                    t_pair = [0, 0]
                pts_out.append({"positions": list(p.get("positions") or []), "time_from_start": t_pair})

            if len(pts_out) >= 2:
                segs_out[seg] = {"joint_names": list(jt.get("joint_names") or []), "points": pts_out}

        return {"version": 1, "segments": segs_out}

    # executed YAML: pack from Base stash (Variant-A keys, op="execute_trajectory")
    @staticmethod
    def _id_sort_key(rid: int) -> int:
        try:
            return int(rid)
        except Exception:
            return 2**63 - 1

    def _jt_msg_to_dict_execpack(self, traj_msg: Any) -> Optional[Dict[str, Any]]:
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
            pts = []
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
        want_op = "execute_trajectory"

        for (run, rid, s, op), msg in list(store.items()):
            if str(run) != str(self._run_id):
                continue
            if str(s) != str(seg):
                continue
            if str(op) != str(want_op):
                continue
            d = self._jt_msg_to_dict_execpack(msg)
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

    def _executed_by_segment_yaml(self) -> Dict[str, Any]:
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

    def _on_finished(self) -> None:
        planned = self._build_planned_payload_yaml_v1()
        executed = self._executed_by_segment_yaml()

        # keep recipe fields aligned
        try:
            self._recipe.planned_traj = JTBySegment.from_yaml_dict(planned)
        except Exception:
            pass
        try:
            self._recipe.executed_traj = JTBySegment.from_yaml_dict(executed) if executed.get("segments") else None
        except Exception:
            self._recipe.executed_traj = None

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()
