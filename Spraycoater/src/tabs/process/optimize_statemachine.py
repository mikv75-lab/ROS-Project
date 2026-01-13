# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import copy
import json
import logging
from typing import Any, Dict, Optional, Tuple

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
    Optimize run (STRICT, Base-compatible; Execute-like):

      - Input trajectory is recipe.planned_traj (JTBySegment YAML v1) (NO filesystem, NO fallbacks)
      - For each segment:
          1) optimize: ros.moveit_optimize_trajectory(RobotTrajectoryMsg, segment=...)
             planner cfg comes from recipe.planner['optimize'] (role/pipeline/planner_id/params)
          2) execute:  ros.moveit_execute_trajectory(optimized RobotTrajectoryMsg, segment=...)
      - Pairing/capture:
          planned  = optimized baseline JT dict for current segment (like Execute baseline pairing)
          executed = ros.moveit_executed_trajectory() (preferred) or last executed cache fallback

    Notes:
      - Execute-like: Base ordering guard is satisfied deterministically (_seen_planned_ok[seg]=True),
        because optimize/execute may not generate PLANNED:OK events.
      - We ignore latched optimized outputs by tracking the previous optimized message instance.
    """

    ROLE = "optimize"

    # timeout waiting for optimized output (ms)
    _OPTIMIZE_TIMEOUT_MS = 4000

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        run_result: RunResult,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
        side: str = "top",
    ) -> None:
        super().__init__(
            recipe=recipe,
            ros=ros,
            run_result=run_result,
            parent=parent,
            max_retries=max_retries,
            skip_home=bool(skip_home),
        )

        self._side: str = str(side or "top")

        # Planned baseline from recipe.planned_traj (YAML v1)
        self._planned_loaded_yaml: Dict[str, Any] = {}
        self._yaml_segments_present: Dict[str, bool] = {}

        # planned JT dict per segment (initially from YAML; later overwritten by optimized JT dicts)
        self._planned_jt_by_segment: Dict[str, Dict[str, Any]] = {}

        # optimized RobotTrajectoryMsg per segment (for execution)
        self._optimized_rt_by_segment: Dict[str, RobotTrajectoryMsg] = {}

        # optimizer cfg (from recipe.planner.optimize)
        self._opt_cfg: Dict[str, Any] = {}
        self._opt_planner_cfg_json: str = ""

        # optimize waiting state (per segment)
        self._opt_waiting: bool = False
        self._opt_wait_seg: str = ""
        self._opt_prev_msg: Optional[RobotTrajectoryMsg] = None

        self._opt_timeout_timer = QtCore.QTimer(self)
        self._opt_timeout_timer.setSingleShot(True)
        self._opt_timeout_timer.timeout.connect(self._on_optimize_timeout)

        # connect optimized trajectory signal (optional; topics.yaml must provide optimized_trajectory_rt)
        try:
            sig = getattr(getattr(self._ros, "moveitpy", None), "signals", None)
            if sig is not None and hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj_changed)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # STOP OVERRIDE (Fix for "Optimize not stopping cleanly")
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        """
        Override: Ensure internal optimization wait logic is killed immediately.
        Otherwise, a late arriving 'optimizedTrajectory' signal triggers execution
        EVEN AFTER the user pressed stop.
        """
        # 1. Kill internal wait state
        self._opt_waiting = False
        self._opt_wait_seg = ""
        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass

        # 2. Delegate to Base to stop ROS motion and clear segment context
        super().request_stop()

    # ------------------------------------------------------------------
    # Prepare
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # side kept for compatibility (does not affect JT execution)
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._side = str(params.get("active_side", self._side) or self._side)
        except Exception:
            pass

        # REQUIRE recipe.planned_traj (no filesystem, no fallbacks)
        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self._error_msg = "Optimize: recipe.planned_traj fehlt (planned_traj.yaml nicht geladen?)."
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self._error_msg = "Optimize: recipe.planned_traj hat kein to_yaml_dict() (JTBySegment API fehlt)."
            return False

        d = fn()
        if not isinstance(d, dict):
            self._error_msg = "Optimize: planned_traj.to_yaml_dict() returned non-dict."
            return False

        # strict validate schema (JTBySegment YAML v1)
        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self._error_msg = f"Optimize: planned_traj YAML invalid: {e}"
            return False

        self._planned_loaded_yaml = d

        # precompute segment availability (>=2 points)
        self._yaml_segments_present = {}
        for seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            self._yaml_segments_present[seg_name] = self._segment_has_points(seg_name)

        # build strict planned JT dicts for pairing (time_from_start -> {sec,nanosec})
        try:
            self._planned_jt_by_segment = {}
            for seg_name, present in self._yaml_segments_present.items():
                if not present:
                    continue
                self._planned_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment(seg_name)
        except Exception as e:
            self._error_msg = f"Optimize: build planned JT cache failed: {e}"
            return False

        # clear base capture buffers (strict)
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()
        self._seen_planned_ok.clear()

        self._optimized_rt_by_segment.clear()

        # optimizer config from recipe (SSoT)
        if not self._read_optimize_cfg_from_recipe():
            return False

        # REQUIRE ROS API
        if not callable(getattr(self._ros, "moveit_optimize_trajectory", None)):
            self._error_msg = "Optimize: ros.moveit_optimize_trajectory(traj, segment=...) fehlt (strict)."
            return False
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self._error_msg = "Optimize: ros.moveit_execute_trajectory(traj, segment=...) fehlt (strict)."
            return False

        # push planner cfg to node now (best-effort)
        self._publish_planner_cfg_to_node()

        # reset optimize waiting state
        self._opt_waiting = False
        self._opt_wait_seg = ""
        self._opt_timeout_timer.stop()
        self._opt_prev_msg = self._safe_get_optimized_msg()

        return True

    # ------------------------------------------------------------------
    # Strict YAML validation (copied pattern from Execute)
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_jtbysegment_yaml_v1(d: Dict[str, Any]) -> None:
        if not isinstance(d, dict):
            raise ValueError("root is not dict")

        try:
            ver = int(d.get("version", 0))
        except Exception:
            raise ValueError("missing/invalid version")

        if ver != 1:
            raise ValueError(f"version must be 1, got {ver!r}")

        segs = d.get("segments", None)
        if not isinstance(segs, dict) or not segs:
            raise ValueError("missing/empty segments")

        for seg_name, seg in segs.items():
            if not isinstance(seg, dict):
                raise ValueError(f"segment {seg_name!r} is not dict")

            jn = seg.get("joint_names", None)
            if not (isinstance(jn, list) and jn and all(isinstance(x, str) and x.strip() for x in jn)):
                raise ValueError(f"segment {seg_name!r}: invalid joint_names")

            pts = seg.get("points", None)
            if not isinstance(pts, list) or len(pts) < 2:
                raise ValueError(f"segment {seg_name!r}: needs >=2 points")

            last_t = None
            for i, p in enumerate(pts):
                if not isinstance(p, dict):
                    raise ValueError(f"segment {seg_name!r}: point[{i}] not dict")

                pos = p.get("positions", None)
                if not isinstance(pos, list) or len(pos) != len(jn):
                    raise ValueError(
                        f"segment {seg_name!r}: point[{i}] positions length "
                        f"{0 if not isinstance(pos, list) else len(pos)} != {len(jn)}"
                    )
                _ = [float(x) for x in pos]  # float-castable

                tfs = p.get("time_from_start", None)
                if not (isinstance(tfs, (list, tuple)) and len(tfs) >= 2):
                    raise ValueError(f"segment {seg_name!r}: point[{i}] time_from_start must be [sec,nsec]")

                sec = int(tfs[0])
                nsec = int(tfs[1])
                if sec < 0 or nsec < 0:
                    raise ValueError(f"segment {seg_name!r}: point[{i}] negative time_from_start")

                t_key = (sec, nsec)
                if last_t is not None and t_key < last_t:
                    raise ValueError(f"segment {seg_name!r}: time_from_start not monotonic at point[{i}]")
                last_t = t_key

    def _segment_has_points(self, seg_name: str) -> bool:
        segs = self._planned_loaded_yaml.get("segments", None)
        if not isinstance(segs, dict):
            return False
        seg = segs.get(seg_name, None)
        if not isinstance(seg, dict):
            return False
        pts = seg.get("points", None)
        return isinstance(pts, list) and len(pts) >= 2

    def _jt_dict_from_yaml_segment(self, seg_name: str) -> Dict[str, Any]:
        seg = self._planned_loaded_yaml["segments"].get(seg_name, None)
        if not isinstance(seg, dict):
            raise ValueError(f"Optimize: Segment '{seg_name}' fehlt in planned_traj.")

        jn = seg.get("joint_names", None)
        pts = seg.get("points", None)
        if not (isinstance(jn, list) and jn and isinstance(pts, list) and len(pts) >= 2):
            raise ValueError(f"Optimize: Segment '{seg_name}' invalid (joint_names/points).")

        out_pts = []
        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions", None)
            tfs = p.get("time_from_start", None)
            if not (isinstance(pos, list) and len(pos) == len(jn)):
                continue
            if not (isinstance(tfs, (list, tuple)) and len(tfs) >= 2):
                continue

            out_pts.append(
                {
                    "positions": [float(x) for x in pos],
                    "time_from_start": {"sec": int(tfs[0]), "nanosec": int(tfs[1])},
                }
            )

        if len(out_pts) < 2:
            raise ValueError(f"Optimize: Segment '{seg_name}' has <2 valid points after normalization.")

        return {"joint_names": [str(x) for x in jn], "points": out_pts}

    # ------------------------------------------------------------------
    # Override pairing source for Optimize (Execute-like)
    # ------------------------------------------------------------------

    def _get_step_sources(self) -> Tuple[Any, Any]:
        """
        Optimize MUST pair:
          planned  = optimized baseline JT dict for current segment (fallback: original YAML baseline)
          executed = ros cache/getter
        """
        seg = str(self._current_state or "")
        planned = self._planned_jt_by_segment.get(seg)

        re = getattr(self._ros, "moveit_executed_trajectory", None)
        if callable(re):
            try:
                return planned, re()
            except Exception as e:
                self._signal_error(f"RosBridge moveit_executed_trajectory() failed: {e!r}")
                return planned, None

        return planned, self._last_executed_any

    # ------------------------------------------------------------------
    # Cache-clear boundary: keep ordering guard open (Execute-like)
    # ------------------------------------------------------------------

    @QtCore.pyqtSlot()
    def _on_traj_cache_clear(self) -> None:
        try:
            super()._on_traj_cache_clear()
        except Exception:
            pass

        # Optimize/Execute may not produce PLANNED:OK events -> keep guard satisfied.
        if self._current_state:
            self._seen_planned_ok[self._current_state] = True

    # ------------------------------------------------------------------
    # YAML -> RobotTrajectoryMsg
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_dict(tfs: Any) -> RosDuration:
        d = RosDuration()
        # tfs comes from _jt_dict_from_yaml_segment: {"sec":..,"nanosec":..}
        d.sec = int(tfs.get("sec", 0))
        d.nanosec = int(tfs.get("nanosec", 0))
        return d

    def _robot_trajectory_from_jt_dict(self, jt: Dict[str, Any], *, seg_name: str) -> RobotTrajectoryMsg:
        jn = jt.get("joint_names")
        pts = jt.get("points")
        if not (isinstance(jn, list) and jn and isinstance(pts, list) and len(pts) >= 2):
            raise ValueError(f"Optimize: jt dict invalid for segment '{seg_name}'.")

        jt_msg = JointTrajectory()
        jt_msg.joint_names = [str(x) for x in jn]

        for p in pts:
            if not isinstance(p, dict):
                continue
            pos = p.get("positions", None)
            tfs = p.get("time_from_start", None)
            if not (isinstance(pos, list) and len(pos) == len(jt_msg.joint_names)):
                continue
            if not isinstance(tfs, dict):
                continue

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in pos]
            pt.time_from_start = self._duration_from_dict(tfs)
            jt_msg.points.append(pt)

        if len(jt_msg.points) < 2:
            raise ValueError(f"Optimize: Segment '{seg_name}' has <2 valid points (after build).")

        msg = RobotTrajectoryMsg()
        msg.joint_trajectory = jt_msg
        return msg

    # ------------------------------------------------------------------
    # Segment control
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and bool(self._skip_home):
            return False
        return bool(self._yaml_segments_present.get(seg_name, False))

    def _on_enter_segment(self, seg_name: str) -> None:
        # skip-home
        if seg_name == STATE_MOVE_HOME and bool(self._skip_home):
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        present = bool(self._yaml_segments_present.get(seg_name, False))
        if not present:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # Satisfy base ordering guard deterministically (Execute-like).
        self._seen_planned_ok[seg_name] = True

        # Start optimize -> on optimized callback we will execute.
        try:
            baseline_jt = self._planned_jt_by_segment.get(seg_name)
            if not isinstance(baseline_jt, dict):
                raise ValueError("missing planned baseline jt dict")
            in_rt = self._robot_trajectory_from_jt_dict(baseline_jt, seg_name=seg_name)
        except Exception as e:
            self._signal_error(f"Optimize: YAML/JT->RobotTrajectory failed for {seg_name}: {e}")
            return

        self._start_optimize_for_segment(seg_name, in_rt)

    # ------------------------------------------------------------------
    # Optimize flow
    # ------------------------------------------------------------------

    def _read_optimize_cfg_from_recipe(self) -> bool:
        """
        Strict: read recipe.planner['optimize'] dict.
        Build a JSON payload we will send via set_planner_cfg (best-effort).
        """
        try:
            planner = getattr(self._recipe, "planner", {}) or {}
        except Exception:
            planner = {}

        cfg = planner.get("optimize") if isinstance(planner, dict) else None
        if not isinstance(cfg, dict):
            self._signal_error("Optimize: recipe.planner['optimize'] fehlt oder ist kein dict.")
            return False

        role = str(cfg.get("role") or "").strip()
        if role and role != "optimize":
            _LOG.warning("Optimize: planner.optimize.role ist '%s' (erwartet 'optimize') – weiter.", role)

        pipeline = str(cfg.get("pipeline") or "").strip()
        planner_id = str(cfg.get("planner_id") or "").strip()
        params = cfg.get("params") or {}
        params = dict(params) if isinstance(params, dict) else {}

        if not planner_id:
            self._signal_error("Optimize: recipe.planner['optimize'].planner_id fehlt/leer.")
            return False

        self._opt_cfg = {
            "role": "optimize",
            "pipeline": pipeline,
            "planner_id": planner_id,
            "params": params,
        }

        try:
            self._opt_planner_cfg_json = json.dumps(self._opt_cfg, ensure_ascii=False)
        except Exception:
            self._signal_error("Optimize: planner cfg konnte nicht zu JSON serialisiert werden.")
            return False

        return True

    def _publish_planner_cfg_to_node(self) -> None:
        """
        Best-effort: send planner cfg dict to node (MoveItPyBridge.plannerCfgChanged).
        """
        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp is not None else None
            if sig is None or not hasattr(sig, "plannerCfgChanged"):
                _LOG.warning("Optimize: MoveItPyBridge hat kein plannerCfgChanged – kann optimize cfg nicht setzen.")
                return
            sig.plannerCfgChanged.emit(self._opt_cfg)
        except Exception as ex:
            _LOG.warning("Optimize: publish planner cfg failed: %s", ex)

    def _safe_get_optimized_msg(self) -> Optional[RobotTrajectoryMsg]:
        """
        Best-effort pull of last optimized trajectory from ros facade if available.
        Used only to ignore a latched old message after triggering optimize.
        """
        try:
            fn = getattr(self._ros, "moveit_optimized_trajectory", None)
            v = fn() if callable(fn) else None
            return v if isinstance(v, RobotTrajectoryMsg) else None
        except Exception:
            return None

    def _start_optimize_for_segment(self, seg_name: str, in_rt: RobotTrajectoryMsg) -> None:
        if self._stop_requested:
            return

        if self._opt_waiting:
            self._signal_error(f"Optimize: internal error (already waiting) while entering {seg_name}.")
            return

        # ensure node has correct cfg (in case someone changed it mid-run)
        self._publish_planner_cfg_to_node()

        self._opt_waiting = True
        self._opt_wait_seg = str(seg_name)
        self._opt_timeout_timer.stop()
        self._opt_timeout_timer.start(int(self._OPTIMIZE_TIMEOUT_MS))

        # remember previous optimized message to avoid accepting latched old one
        self._opt_prev_msg = self._safe_get_optimized_msg()

        try:
            self._ros.moveit_optimize_trajectory(in_rt, segment=str(seg_name))
        except Exception as ex:
            self._opt_timeout_timer.stop()
            self._opt_waiting = False
            self._opt_wait_seg = ""
            self._signal_error(f"Optimize: optimize_trajectory publish failed for {seg_name}: {ex}")

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj_changed(self, msg: object) -> None:
        if not isinstance(msg, RobotTrajectoryMsg):
            return
        # GUARD: If stopped, ignore arrival
        if self._stop_requested or not self._opt_waiting:
            return

        # ignore latched old message instance
        if self._opt_prev_msg is not None and msg is self._opt_prev_msg:
            return

        seg = str(self._opt_wait_seg or "")
        if not seg:
            return

        self._opt_timeout_timer.stop()
        self._opt_waiting = False
        self._opt_wait_seg = ""

        # store optimized for execution
        self._optimized_rt_by_segment[seg] = msg

        # overwrite planned baseline JT dict for pairing + persistence
        opt_jt = self._jt_from_any(msg)
        if opt_jt is not None:
            self._planned_jt_by_segment[seg] = opt_jt

        # execute optimized now
        self._execute_optimized_segment(seg)

    @QtCore.pyqtSlot()
    def _on_optimize_timeout(self) -> None:
        # GUARD: If stopped, ignore timeout
        if self._stop_requested or not self._opt_waiting:
            return

        seg = str(self._opt_wait_seg or "")
        self._opt_waiting = False
        self._opt_wait_seg = ""

        # last-chance: pull via getter and accept if it's not the previous instance
        last = self._safe_get_optimized_msg()
        if last is not None and (self._opt_prev_msg is None or last is not self._opt_prev_msg) and seg:
            self._optimized_rt_by_segment[seg] = last
            opt_jt = self._jt_from_any(last)
            if opt_jt is not None:
                self._planned_jt_by_segment[seg] = opt_jt
            self._execute_optimized_segment(seg)
            return

        self._signal_error(
            f"Optimize: Timeout waiting for optimized_trajectory_rt after {int(self._OPTIMIZE_TIMEOUT_MS)} ms (seg={seg}). "
            "Prüfe: topics.yaml publish id=optimized_trajectory_rt + Node publish."
        )

    # ------------------------------------------------------------------
    # Execute optimized segment
    # ------------------------------------------------------------------

    def _execute_optimized_segment(self, seg_name: str) -> None:
        if self._stop_requested:
            return

        rt = self._optimized_rt_by_segment.get(seg_name)
        if rt is None:
            self._signal_error(f"Optimize: internal error, optimized trajectory missing for segment {seg_name}.")
            return

        try:
            self._ros.moveit_execute_trajectory(rt, segment=str(seg_name))
        except Exception as e:
            self._signal_error(f"Optimize: execute optimized trajectory failed for {seg_name}: {e}")

    # ------------------------------------------------------------------
    # Planned/Executed payload override
    # ------------------------------------------------------------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        """
        planned  = optimized baseline (JTBySegment YAML v1) built from _planned_jt_by_segment
        executed = captured via Base (JTBySegment YAML v1)
        """
        segments: Dict[str, Any] = {}
        for seg in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            if seg == STATE_MOVE_HOME and bool(self._skip_home):
                continue
            jt = self._planned_jt_by_segment.get(seg)
            seg_yaml = self._jt_dict_to_segment_yaml(jt)
            if seg_yaml is not None:
                segments[seg] = seg_yaml

        planned = {"version": 1, "segments": segments}
        executed = self._jt_by_segment_yaml(which="executed")
        return planned, executed

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()

        # executed must exist (point of Optimize is: execute optimized)
        e_segs = executed.get("segments") if isinstance(executed, dict) else None
        if not (isinstance(e_segs, dict) and e_segs):
            self.notifyError.emit(
                "Optimize finished, but executed trajectory capture is empty. "
                "Check MoveItPyNode executed_trajectory_rt publish + BaseProcessStatemachine capture."
            )
            self._cleanup()
            return

        # IMPORTANT: use injected RunResult (seeded by ProcessThread)
        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))

        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        try:
            sig = getattr(getattr(self._ros, "moveitpy", None), "signals", None)
            if sig is not None and hasattr(sig, "optimizedTrajectoryChanged"):
                try:
                    sig.optimizedTrajectoryChanged.disconnect(self._on_optimized_traj_changed)
                except Exception:
                    pass
        except Exception:
            pass

        super()._cleanup()