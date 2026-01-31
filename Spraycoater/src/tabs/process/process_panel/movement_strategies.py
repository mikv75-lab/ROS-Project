# -*- coding: utf-8 -*-
"""
File: src/tabs/process/process_panel/movement_strategies.py

Movement strategies for Validate/Execute pipelines.

STRICT goals:
- Strategy decides which SegmentRunner steps to emit.
- Strategy is config-driven (planner_cfg), but does NOT send planner_cfg to MoveIt;
  it derives pilz-sequence payload fields from planner_cfg["params"].

Notes:
- PtpStrategy -> plan_pose + execute (per pose).
- PilzSequenceStrategy -> plan_pilz_sequence + execute (single blended sequence),
  with strict payload dict (frame, sequence[], blend/vel/acc/time).

Assumptions / Contracts:
- SegmentRunner provides:
    - make_plan_pose_step_bound(...)
    - make_execute_last_planned_step_bound(...)
    - make_plan_pilz_sequence_step_bound(run, req_id, seg, payload, label)
- The ROS bridge publishes a keyed plan_request with op="plan_pilz_sequence"
  and payload being a dict with:
    {
      "frame": "<frame>",
      "sequence": [{"cmd":"LIN|PTP","pose":{position:{...},orientation:{...}}}, ...],
      "blend_radius_m": float,
      "planning_time_s": float,
      "vel_scale": float,
      "acc_scale": float,
      "start": "current"|"first_pose"
    }
"""

from __future__ import annotations

import time
import copy
import logging
from abc import ABC, abstractmethod
from typing import List, Any, Optional, Dict

from geometry_msgs.msg import PoseStamped

from .segment_runner import SegmentRunner, StepSpec

_LOG = logging.getLogger("tabs.process.movement_strategies")


class MoveStrategy(ABC):
    """
    Base movement strategy.

    planner_cfg (segment-specific):
      {
        "pipeline": "...",
        "planner_id": "...",
        "params": {...}
      }
    """

    def __init__(self, runner: SegmentRunner, run_id: str, planner_cfg: Optional[Dict[str, Any]] = None):
        self.runner = runner
        self.run_id = str(run_id or "")
        self.planner_cfg: Dict[str, Any] = dict(planner_cfg or {})

    @abstractmethod
    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        raise NotImplementedError

    def _new_id(self, i: int) -> int:
        return int(time.time() * 1000) + int(i)

    def _transform_stamped(self, ps: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        Uses ros.tf_buffer (if available) to transform PoseStamped into target_frame.
        Best-effort; returns None on failure.
        """
        try:
            ros = self.runner._ros  # existing pattern in your codebase
            tfb = getattr(ros, "tf_buffer", None)
            if tfb is None:
                return None

            source_frame = str(getattr(getattr(ps, "header", None), "frame_id", "") or "").strip()
            target_frame = str(target_frame or "").strip()
            if not source_frame or not target_frame:
                return None
            if source_frame == target_frame:
                return ps

            from rclpy.time import Time
            from rclpy.duration import Duration

            try:
                out = tfb.transform(ps, target_frame, timeout=Duration(seconds=0.25))
                out.header.frame_id = target_frame
                return out
            except Exception:
                pass

            from tf2_geometry_msgs import do_transform_pose  # type: ignore
            if not tfb.can_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=0.25)):
                return None

            tf = tfb.lookup_transform(target_frame, source_frame, Time())

            try:
                out = do_transform_pose(ps, tf)  # type: ignore[arg-type]
                out.header.frame_id = target_frame
                return out
            except Exception:
                out = PoseStamped()
                out.header.frame_id = target_frame
                out.header.stamp = ps.header.stamp
                out.pose = do_transform_pose(ps.pose, tf)  # type: ignore[arg-type]
                return out

        except Exception as e:
            _LOG.warning("Strategy Transform failed: %r", e)
        return None


class PtpStrategy(MoveStrategy):
    """Plans+executes each pose individually via plan_pose (PTP or OMPL)."""

    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        steps: List[StepSpec] = []
        for i, pose in enumerate(list(poses or [])):
            rid = self._new_id(i)
            steps.append(
                self.runner.make_plan_pose_step_bound(
                    run=self.run_id,
                    req_id=rid,
                    seg=seg_name,
                    pose=pose,
                    label=f"{seg_name}:ptp:plan[{i}]",
                )
            )
            steps.append(
                self.runner.make_execute_last_planned_step_bound(
                    run=self.run_id,
                    req_id=rid,
                    seg=seg_name,
                    label=f"{seg_name}:ptp:exec[{i}]",
                )
            )
        return steps


class PilzSequenceStrategy(MoveStrategy):
    """
    Builds a single blended sequence request (Pilz LIN/PTP) instead of per-point planning.

    planner_cfg["params"] mapping (all optional):
      - blend_radius_m (float, default 0.0)
      - planning_time_s (float, default 5.0)
      - vel_scale (float, default 0.10)   <-- IMPORTANT default
      - acc_scale (float, default 0.10)   <-- IMPORTANT default
      - cmd (str: "LIN"|"PTP", default "LIN")
      - cmds (list[str], optional per-point command list, len==N)
      - start (str: "current"|"first_pose", default "current")
    """

    def _pilz_payload_fields(self) -> Dict[str, Any]:
        raw = self.planner_cfg.get("params")
        params = raw if isinstance(raw, dict) else {}

        def _f(name: str, default: float, *, lo: Optional[float] = None, hi: Optional[float] = None) -> float:
            try:
                v = float(params.get(name, default))
            except Exception:
                v = float(default)
            if lo is not None:
                v = max(lo, v)
            if hi is not None:
                v = min(hi, v)
            return float(v)

        blend = _f("blend_radius_m", 0.0, lo=0.0)
        planning_time = _f("planning_time_s", 5.0, lo=0.01)

        # SAFE defaults (avoid Pilz joint-limit violations when params missing)
        vel = _f("vel_scale", 0.10, lo=0.01, hi=1.0)
        acc = _f("acc_scale", 0.10, lo=0.01, hi=1.0)

        cmd = str(params.get("cmd", "LIN") or "LIN").strip().upper()
        if cmd not in ("LIN", "PTP"):
            cmd = "LIN"

        start = str(params.get("start", "current") or "current").strip().lower()
        if start not in ("current", "first_pose"):
            start = "current"

        cmds_raw = params.get("cmds", None)
        cmds: Optional[List[str]] = None
        if isinstance(cmds_raw, list) and cmds_raw:
            tmp: List[str] = []
            ok = True
            for x in cmds_raw:
                s = str(x or "").strip().upper()
                if s not in ("LIN", "PTP"):
                    ok = False
                    break
                tmp.append(s)
            if ok:
                cmds = tmp

        return {
            "blend_radius_m": blend,
            "planning_time_s": planning_time,
            "vel_scale": vel,
            "acc_scale": acc,
            "cmd": cmd,
            "cmds": cmds,
            "start": start,
        }

    @staticmethod
    def _pose_to_dict(ps: PoseStamped) -> Dict[str, Any]:
        p = ps.pose
        return {
            "position": {"x": float(p.position.x), "y": float(p.position.y), "z": float(p.position.z)},
            "orientation": {
                "x": float(p.orientation.x),
                "y": float(p.orientation.y),
                "z": float(p.orientation.z),
                "w": float(p.orientation.w),
            },
        }

    def _build_pilz_sequence_payload(self, *, frame: str, poses: List[PoseStamped]) -> Dict[str, Any]:
        fields = self._pilz_payload_fields()

        base_cmd = str(fields["cmd"]).strip().upper()
        cmds = fields.get("cmds", None)

        if isinstance(cmds, list) and len(cmds) == len(poses):
            cmds_list = [str(x).strip().upper() for x in cmds]
        else:
            cmds_list = [base_cmd for _ in poses]

        seq: List[Dict[str, Any]] = []
        for ps, cmd in zip(poses, cmds_list):
            if cmd not in ("LIN", "PTP"):
                cmd = base_cmd
            seq.append({"cmd": cmd, "pose": self._pose_to_dict(ps)})

        return {
            "frame": str(frame),
            "sequence": seq,
            "blend_radius_m": float(fields["blend_radius_m"]),
            "planning_time_s": float(fields["planning_time_s"]),
            "vel_scale": float(fields["vel_scale"]),
            "acc_scale": float(fields["acc_scale"]),
            "start": str(fields["start"]),
        }

    def create_steps(self, seg_name: str, poses: List[Any]) -> List[StepSpec]:
        if not poses:
            return []

        rid = self._new_id(0)
        final_poses = list(poses)

        target_frame = "substrate"
        try:
            if isinstance(final_poses[0], PoseStamped):
                target_frame = str(final_poses[0].header.frame_id or "").strip() or "substrate"
        except Exception:
            target_frame = "substrate"

        if len(final_poses) < 2:
            return self._fallback_single_ptp(seg_name, final_poses[-1], rid)

        norm: List[PoseStamped] = []
        for i, ps in enumerate(final_poses):
            if not isinstance(ps, PoseStamped):
                _LOG.warning("Strategy: poses[%d] not PoseStamped -> fallback PTP.", i)
                return self._fallback_single_ptp(seg_name, final_poses[-1], rid)

            src = str(ps.header.frame_id or "").strip()
            if src and src != target_frame:
                tps = self._transform_stamped(ps, target_frame)
                if tps is None:
                    _LOG.warning("Strategy: TF failed for pose[%d] %s->%s, fallback PTP.", i, src, target_frame)
                    return self._fallback_single_ptp(seg_name, final_poses[-1], rid)
                norm.append(tps)
            else:
                if not src:
                    cps = copy.deepcopy(ps)
                    cps.header.frame_id = target_frame
                    norm.append(cps)
                else:
                    norm.append(ps)

        payload = self._build_pilz_sequence_payload(frame=target_frame, poses=norm)

        return [
            self.runner.make_plan_pilz_sequence_step_bound(
                run=self.run_id,
                req_id=rid,
                seg=seg_name,
                payload=payload,
                label=f"{seg_name}:seq:plan(pilz)",
            ),
            self.runner.make_execute_last_planned_step_bound(
                run=self.run_id,
                req_id=rid,
                seg=seg_name,
                label=f"{seg_name}:seq:exec",
            ),
        ]

    def _fallback_single_ptp(self, seg_name: str, pose: Any, rid: int) -> List[StepSpec]:
        return [
            self.runner.make_plan_pose_step_bound(
                run=self.run_id,
                req_id=rid,
                seg=seg_name,
                pose=pose,
                label=f"{seg_name}:seq:single",
            ),
            self.runner.make_execute_last_planned_step_bound(
                run=self.run_id,
                req_id=rid,
                seg=seg_name,
                label=f"{seg_name}:seq:exec",
            ),
        ]
