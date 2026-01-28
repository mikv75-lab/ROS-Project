# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import copy
import json
import logging
from typing import Any, Dict, Optional, List

from PyQt6 import QtCore

from builtin_interfaces.msg import Duration as RosDuration  # type: ignore
from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg  # type: ignore
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # type: ignore

from model.recipe.recipe_run_result import RunResult

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process.optimize_sm")


class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    """
    Optimize run (STRICT, collect-only) – UPDATED for new MoveItPy node behavior.

    Flow:
      1) request optimization via ros.moveit_optimize_trajectory(in_rt, segment=SEG)
      2) wait for optimizedTrajectoryChanged (or fallback getter on timeout)
      3) execute optimized via ros.moveit_execute_trajectory(opt_rt, segment=SEG)

    Pairing contract (important with new node):
      - For execute_trajectory, PLANNED:OK may be absent -> we seed Base pending planned
        right before execute (and again on retry).
      - Node may emit traj_cache_clear before each move; Base clears only cached
        last_planned/last_executed, but our seeded pending planned must survive.
    """

    ROLE = "optimize"

    _OPTIMIZE_TIMEOUT_MS = 6000  # optimization can be slow

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
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

        # planned baseline input (raw YAML v1) + normalized (internal)
        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}
        self._planned_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        # optimized RobotTrajectoryMsg cache per segment (for execute + retry)
        self._optimized_rt_by_segment: Dict[str, RobotTrajectoryMsg] = {}

        # optimize config (flat cfg to node)
        self._opt_cfg: Dict[str, Any] = {}
        self._opt_planner_cfg_json: str = ""

        # optimize waiting state
        self._opt_waiting: bool = False
        self._opt_wait_seg: str = ""
        self._opt_prev_msg: Optional[RobotTrajectoryMsg] = None

        self._opt_timeout_timer = QtCore.QTimer(self)
        self._opt_timeout_timer.setSingleShot(True)
        self._opt_timeout_timer.timeout.connect(self._on_optimize_timeout)

        # inflight (for retries)
        self._inflight_seg: str = ""
        self._inflight_stage: str = ""  # "", "optimize", "execute"
        self._inflight_opt_in: Optional[RobotTrajectoryMsg] = None
        self._inflight_exec_rt: Optional[RobotTrajectoryMsg] = None
        self._inflight_exec_planned_jt: Optional[Dict[str, Any]] = None  # seed planned on execute retry

        # connect optimized trajectory signal (best-effort)
        self._opt_sig_connected: bool = False
        try:
            sig = getattr(getattr(self._ros, "moveitpy", None), "signals", None)
            if sig is not None and hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj_changed)
                self._opt_sig_connected = True
        except Exception:
            self._opt_sig_connected = False

    # ------------------------------------------------------------------
    # STOP
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._opt_waiting = False
        self._opt_wait_seg = ""

        self._inflight_seg = ""
        self._inflight_stage = ""
        self._inflight_opt_in = None
        self._inflight_exec_rt = None
        self._inflight_exec_planned_jt = None

        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        super().request_stop()

    # ------------------------------------------------------------------
    # Retry hook (STRICT)
    # ------------------------------------------------------------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        if self._stop_requested:
            return False
        if self._machine is None:
            return False
        if seg_name != self._current_state:
            return False
        if self._inflight_seg != seg_name:
            return False

        stage = str(self._inflight_stage or "")

        if stage == "optimize" and self._inflight_opt_in is not None:
            _LOG.warning(
                "Optimize: retry OPTIMIZE request attempt %d for seg=%s reason=%s",
                int(attempt),
                str(seg_name),
                str(last_error),
            )
            return self._retry_optimize_request(seg_name, self._inflight_opt_in)

        if stage == "execute" and self._inflight_exec_rt is not None:
            _LOG.warning(
                "Optimize: retry EXECUTE attempt %d for seg=%s reason=%s",
                int(attempt),
                str(seg_name),
                str(last_error),
            )

            # CRITICAL: re-seed planned pending for execute retry (PLANNED:OK may not arrive)
            if isinstance(self._inflight_exec_planned_jt, dict):
                self._pending_planned_step[seg_name] = copy.deepcopy(self._inflight_exec_planned_jt)
                self._pending_executed_step[seg_name] = None

            try:
                self._ros.moveit_execute_trajectory(self._inflight_exec_rt, segment=str(seg_name))
                return True
            except Exception as e:
                _LOG.error("Optimize: retry execute failed for seg=%s: %r", str(seg_name), e)
                return False

        return False

    def _retry_optimize_request(self, seg_name: str, in_rt: RobotTrajectoryMsg) -> bool:
        # re-arm wait + timer and resend request
        if not callable(getattr(self._ros, "moveit_optimize_trajectory", None)):
            return False

        self._publish_planner_cfg_to_node()

        self._opt_waiting = True
        self._opt_wait_seg = str(seg_name)

        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        self._opt_timeout_timer.start(int(self._OPTIMIZE_TIMEOUT_MS))

        self._opt_prev_msg = self._safe_get_optimized_msg()

        # inflight state
        self._inflight_seg = str(seg_name)
        self._inflight_stage = "optimize"
        self._inflight_opt_in = in_rt
        self._inflight_exec_rt = None
        self._inflight_exec_planned_jt = None

        try:
            self._ros.moveit_optimize_trajectory(in_rt, segment=str(seg_name))
            return True
        except Exception as e:
            _LOG.error("Optimize: retry optimize request failed seg=%s: %r", str(seg_name), e)
            return False

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", self._side) or self._side)
        except Exception:
            pass

        self._inflight_seg = ""
        self._inflight_stage = ""
        self._inflight_opt_in = None
        self._inflight_exec_rt = None
        self._inflight_exec_planned_jt = None

        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self._error_msg = "Optimize: recipe.planned_traj fehlt (bitte erst Validate ausführen)."
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self._error_msg = "Optimize: planned_traj API fehlt (to_yaml_dict)."
            return False

        d = fn()
        if not isinstance(d, dict):
            self._error_msg = "Optimize: planned_traj daten ungültig (non-dict)."
            return False

        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self._error_msg = f"Optimize: planned_traj YAML invalid: {e}"
            return False

        self._planned_loaded_yaml = d

        # segment availability
        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        # normalize planned JT dicts (time_from_start => {"sec","nanosec"})
        try:
            self._planned_jt_by_segment = {}
            for seg_name, present in self._yaml_segments_present.items():
                if not present:
                    continue
                self._planned_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment_normalized(seg_name)
        except Exception as e:
            self._error_msg = f"Optimize: build planned JT cache failed: {e}"
            return False

        self._optimized_rt_by_segment.clear()

        if not self._read_optimize_cfg_from_recipe():
            return False

        if not callable(getattr(self._ros, "moveit_optimize_trajectory", None)):
            self._error_msg = "Optimize: ros.moveit_optimize_trajectory fehlt."
            return False
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self._error_msg = "Optimize: ros.moveit_execute_trajectory fehlt."
            return False

        self._publish_planner_cfg_to_node()

        self._opt_waiting = False
        self._opt_wait_seg = ""
        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        self._opt_prev_msg = self._safe_get_optimized_msg()

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
            raise ValueError(f"Optimize: segment '{seg_name}' missing in planned_traj")

        jn = seg.get("joint_names")
        pts = seg.get("points")
        if not (isinstance(jn, list) and jn and isinstance(pts, list) and len(pts) >= 2):
            raise ValueError(f"Optimize: segment '{seg_name}' invalid (joint_names/points)")

        out_pts: List[Dict[str, Any]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            tfs = p.get("time_from_start")
            if not isinstance(pos, list):
                continue

            t_sec = 0
            t_nsec = 0
            if isinstance(tfs, dict):
                t_sec = int(tfs.get("sec", 0))
                t_nsec = int(tfs.get("nanosec", 0))
            elif isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
                t_sec = int(tfs[0])
                t_nsec = int(tfs[1])

            q: Dict[str, Any] = {
                "positions": [float(x) for x in pos],
                "time_from_start": {"sec": t_sec, "nanosec": t_nsec},
            }

            if "velocities" in p and isinstance(p.get("velocities"), list):
                q["velocities"] = [float(x) for x in p.get("velocities") or []]
            if "accelerations" in p and isinstance(p.get("accelerations"), list):
                q["accelerations"] = [float(x) for x in p.get("accelerations") or []]
            if "effort" in p and isinstance(p.get("effort"), list):
                q["effort"] = [float(x) for x in p.get("effort") or []]

            out_pts.append(q)

        if len(out_pts) < 2:
            raise ValueError(f"Optimize: segment '{seg_name}' has <2 valid points after normalization")

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    # ------------------------------------------------------------------
    # Config Loading (flatten)
    # ------------------------------------------------------------------

    def _read_optimize_cfg_from_recipe(self) -> bool:
        try:
            planner = getattr(self._recipe, "planner", {}) or {}
        except Exception:
            planner = {}

        cfg = planner.get("optimize") if isinstance(planner, dict) else None
        if not isinstance(cfg, dict):
            self._error_msg = "Optimize: Config in recipe fehlt (planner.optimize)."
            return False

        pipeline = str(cfg.get("pipeline") or "").strip()
        planner_id = str(cfg.get("planner_id") or "").strip()
        raw_params = cfg.get("params") or {}

        if not planner_id:
            self._error_msg = "Optimize: planner_id fehlt."
            return False

        self._opt_cfg = {"role": "optimize", "pipeline": pipeline, "planner_id": planner_id}

        if isinstance(raw_params, dict):
            if "velocity_scaling" in raw_params:
                self._opt_cfg["max_velocity_scaling_factor"] = float(raw_params["velocity_scaling"])
            elif "vel_scale" in raw_params:
                self._opt_cfg["max_velocity_scaling_factor"] = float(raw_params["vel_scale"])

            if "acceleration_scaling" in raw_params:
                self._opt_cfg["max_acceleration_scaling_factor"] = float(raw_params["acceleration_scaling"])
            elif "acc_scale" in raw_params:
                self._opt_cfg["max_acceleration_scaling_factor"] = float(raw_params["acc_scale"])

            for k, v in raw_params.items():
                if k not in ("velocity_scaling", "vel_scale", "acceleration_scaling", "acc_scale"):
                    self._opt_cfg[k] = v

        try:
            self._opt_planner_cfg_json = json.dumps(self._opt_cfg, ensure_ascii=False)
        except Exception:
            self._opt_planner_cfg_json = ""

        return True

    def _publish_planner_cfg_to_node(self) -> None:
        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp is not None else None
            if sig is not None and hasattr(sig, "plannerCfgChanged"):
                _LOG.info("Optimize: Sende Config (flat): %s", self._opt_cfg)
                sig.plannerCfgChanged.emit(self._opt_cfg)
        except Exception as ex:
            _LOG.warning("Optimize: Config send failed: %s", ex)

    # ------------------------------------------------------------------
    # Segment gating
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        return bool(self._yaml_segments_present.get(seg_name, False))

    # ------------------------------------------------------------------
    # Segment enter
    # ------------------------------------------------------------------

    def _on_enter_segment(self, seg_name: str) -> None:
        present = bool(self._yaml_segments_present.get(seg_name, False))
        if not present:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # build input RobotTrajectory from normalized planned jt
        try:
            baseline_jt = self._planned_jt_by_segment.get(seg_name)
            if not isinstance(baseline_jt, dict):
                raise ValueError("missing planned baseline jt dict")
            in_rt = self._robot_trajectory_from_jt_dict(baseline_jt)
        except Exception as e:
            self._signal_error(f"Optimize: JT->RobotTrajectory failed for {seg_name}: {e}")
            return

        self._start_optimize_for_segment(seg_name, in_rt)

    # ------------------------------------------------------------------
    # Optimize request / waiting
    # ------------------------------------------------------------------

    def _safe_get_optimized_msg(self) -> Optional[RobotTrajectoryMsg]:
        try:
            fn = getattr(self._ros, "moveit_optimized_trajectory", None)
            v = fn() if callable(fn) else None
            return v if isinstance(v, RobotTrajectoryMsg) else None
        except Exception:
            return None

    def _start_optimize_for_segment(self, seg_name: str, in_rt: RobotTrajectoryMsg) -> None:
        if self._stop_requested or self._opt_waiting:
            return

        self._publish_planner_cfg_to_node()

        self._opt_waiting = True
        self._opt_wait_seg = str(seg_name)

        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        self._opt_timeout_timer.start(int(self._OPTIMIZE_TIMEOUT_MS))

        self._opt_prev_msg = self._safe_get_optimized_msg()

        # inflight for retry
        self._inflight_seg = str(seg_name)
        self._inflight_stage = "optimize"
        self._inflight_opt_in = in_rt
        self._inflight_exec_rt = None
        self._inflight_exec_planned_jt = None

        try:
            self._ros.moveit_optimize_trajectory(in_rt, segment=str(seg_name))
        except Exception as ex:
            try:
                self._opt_timeout_timer.stop()
            except Exception:
                pass
            self._opt_waiting = False
            self._signal_error(f"Optimize request failed: {ex}")

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj_changed(self, msg: object) -> None:
        if self._stop_requested:
            return
        if not self._opt_waiting:
            return
        if not isinstance(msg, RobotTrajectoryMsg):
            return
        if self._opt_prev_msg is not None and msg is self._opt_prev_msg:
            return

        seg = str(self._opt_wait_seg)

        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        self._opt_waiting = False
        self._opt_wait_seg = ""

        self._optimized_rt_by_segment[seg] = msg

        # Normalize optimized into JT dict (for planned payload + seeding)
        opt_jt = self._jt_from_any(msg)
        if isinstance(opt_jt, dict):
            self._planned_jt_by_segment[seg] = opt_jt

        self._execute_optimized_segment(seg)

    @QtCore.pyqtSlot()
    def _on_optimize_timeout(self) -> None:
        if self._stop_requested:
            return
        if not self._opt_waiting:
            return

        seg = str(self._opt_wait_seg)
        self._opt_waiting = False

        last = self._safe_get_optimized_msg()
        if last is not None and (self._opt_prev_msg is None or last is not self._opt_prev_msg):
            _LOG.info("Optimize: Late arrival via getter (seg=%s).", seg)
            self._optimized_rt_by_segment[seg] = last
            opt_jt = self._jt_from_any(last)
            if isinstance(opt_jt, dict):
                self._planned_jt_by_segment[seg] = opt_jt
            self._execute_optimized_segment(seg)
            return

        self._signal_error(f"Optimize: Timeout waiting for result (seg={seg})")

    def _execute_optimized_segment(self, seg_name: str) -> None:
        if self._stop_requested:
            return
        rt = self._optimized_rt_by_segment.get(seg_name)
        if rt is None:
            self._signal_error("Optimize: Internal error (no optimized traj)")
            return

        # CRITICAL: seed planned pending right before execute (PLANNED:OK may be absent)
        planned_jt = self._planned_jt_by_segment.get(seg_name)
        if isinstance(planned_jt, dict):
            self._pending_planned_step[seg_name] = copy.deepcopy(planned_jt)
            self._pending_executed_step[seg_name] = None

        # inflight for retry
        self._inflight_seg = str(seg_name)
        self._inflight_stage = "execute"
        self._inflight_exec_rt = rt
        self._inflight_exec_planned_jt = copy.deepcopy(planned_jt) if isinstance(planned_jt, dict) else None

        try:
            self._ros.moveit_execute_trajectory(rt, segment=str(seg_name))
        except Exception as e:
            self._signal_error(f"Optimize: Execute failed: {e}")

    # ------------------------------------------------------------------
    # Final payload (planned=optimized JTBySegment, executed=captured)
    # ------------------------------------------------------------------

    def _jt_dict_to_yaml_segment(self, jt: Optional[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
        if not isinstance(jt, dict):
            return None
        jn = jt.get("joint_names")
        pts = jt.get("points")
        if not (isinstance(jn, list) and jn and isinstance(pts, list) and len(pts) >= 2):
            return None

        out_pts: List[Dict[str, Any]] = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions")
            tfs = p.get("time_from_start")
            if not isinstance(pos, list):
                continue

            sec = 0
            nsec = 0
            if isinstance(tfs, dict):
                sec = int(tfs.get("sec", 0))
                nsec = int(tfs.get("nanosec", 0))
            elif isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
                sec = int(tfs[0])
                nsec = int(tfs[1])

            q: Dict[str, Any] = {"positions": [float(x) for x in pos], "time_from_start": [sec, nsec]}
            if "velocities" in p and isinstance(p.get("velocities"), list):
                q["velocities"] = [float(x) for x in p.get("velocities") or []]
            if "accelerations" in p and isinstance(p.get("accelerations"), list):
                q["accelerations"] = [float(x) for x in p.get("accelerations") or []]
            if "effort" in p and isinstance(p.get("effort"), list):
                q["effort"] = [float(x) for x in p.get("effort") or []]
            out_pts.append(q)

        if len(out_pts) < 2:
            return None

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    def _build_traj_payload(self) -> Dict[str, Any]:
        segments: Dict[str, Any] = {}
        for seg in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            jt = self._planned_jt_by_segment.get(seg)
            seg_yaml = self._jt_dict_to_yaml_segment(jt)
            if seg_yaml:
                segments[seg] = seg_yaml
        return {"version": 1, "segments": segments}

    def _on_finished(self) -> None:
        planned = self._build_traj_payload()
        executed = self._jt_by_segment_yaml(which="executed")

        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # Helper: normalized JT dict -> RobotTrajectoryMsg
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_time_dict(tfs: Any) -> RosDuration:
        d = RosDuration()
        if isinstance(tfs, dict):
            d.sec = int(tfs.get("sec", 0))
            d.nanosec = int(tfs.get("nanosec", 0))
            return d
        if isinstance(tfs, (list, tuple)) and len(tfs) >= 2:
            d.sec = int(tfs[0])
            d.nanosec = int(tfs[1])
            return d
        d.sec = 0
        d.nanosec = 0
        return d

    def _robot_trajectory_from_jt_dict(self, jt: Dict[str, Any]) -> RobotTrajectoryMsg:
        jn = jt.get("joint_names") or []
        pts = jt.get("points") or []

        msg = RobotTrajectoryMsg()
        msg.joint_trajectory = JointTrajectory()
        msg.joint_trajectory.joint_names = [str(x) for x in list(jn)]

        for p in pts:
            if not isinstance(p, dict) or "positions" not in p:
                continue
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in list(p.get("positions") or [])]
            pt.time_from_start = self._duration_from_time_dict(p.get("time_from_start"))

            if "velocities" in p and isinstance(p.get("velocities"), list):
                pt.velocities = [float(x) for x in p.get("velocities") or []]
            if "accelerations" in p and isinstance(p.get("accelerations"), list):
                pt.accelerations = [float(x) for x in p.get("accelerations") or []]
            if "effort" in p and isinstance(p.get("effort"), list):
                pt.effort = [float(x) for x in p.get("effort") or []]

            msg.joint_trajectory.points.append(pt)

        return msg

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass

        try:
            if self._opt_sig_connected:
                sig = getattr(getattr(self._ros, "moveitpy", None), "signals", None)
                if sig is not None and hasattr(sig, "optimizedTrajectoryChanged"):
                    try:
                        sig.optimizedTrajectoryChanged.disconnect(self._on_optimized_traj_changed)
                    except Exception:
                        pass
        except Exception:
            pass
        self._opt_sig_connected = False

        super()._cleanup()
