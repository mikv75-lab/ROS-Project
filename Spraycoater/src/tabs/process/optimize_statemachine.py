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
    Optimize run:
      - Input: recipe.planned_traj (The dense trajectory from Validate)
      - Action: Sends trajectory to 'moveit_optimize_trajectory' (Retiming/TOTG)
      - FIX: Flattens parameter config so Velocity Scaling 1.0 reaches the Node correctly.
      - FIX: Implements _on_enter_segment to prevent NotImplementedError.

    Retry (STRICT):
      - Base retries ONLY last motion on ERROR:EXEC..., but for Optimize we never want to "retry execute"
        as a motion-level retry because the segment workflow is optimize->execute and we need to keep
        it deterministic.
      - Therefore: soft-error retry hook returns False (abort with RETRY_EXHAUSTED).
        If you later want retry, implement it here with explicit state.
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

        # connect optimized trajectory signal
        self._opt_sig_connected: bool = False
        try:
            sig = getattr(getattr(self._ros, "moveitpy", None), "signals", None)
            if sig is not None and hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj_changed)
                self._opt_sig_connected = True
        except Exception:
            self._opt_sig_connected = False

    @QtCore.pyqtSlot()
    def request_stop(self) -> None:
        self._opt_waiting = False
        self._opt_wait_seg = ""
        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        super().request_stop()

    # ------------------------------------------------------------------
    # Retry hook (STRICT)
    # ------------------------------------------------------------------

    def _on_retry_last_motion(self, seg_name: str, attempt: int, last_error: str) -> bool:
        # Optimize is a two-stage workflow (optimize -> execute). We do not retry "last motion"
        # implicitly here, because we would need to know whether we are retrying optimize request
        # or execute request. Abort deterministically.
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

        # REQUIRE recipe.planned_traj
        planned_traj = getattr(self._recipe, "planned_traj", None)
        if planned_traj is None:
            self._error_msg = "Optimize: recipe.planned_traj fehlt (bitte erst Validate ausführen)."
            return False

        fn = getattr(planned_traj, "to_yaml_dict", None)
        if not callable(fn):
            self._error_msg = "Optimize: planned_traj API fehlt."
            return False

        d = fn()
        if not isinstance(d, dict):
            self._error_msg = "Optimize: planned_traj daten ungültig."
            return False

        try:
            self._validate_jtbysegment_yaml_v1(d)
        except Exception as e:
            self._error_msg = f"Optimize: planned_traj YAML invalid: {e}"
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
                self._planned_jt_by_segment[seg_name] = self._jt_dict_from_yaml_segment(seg_name)
        except Exception as e:
            self._error_msg = f"Optimize: build planned JT cache failed: {e}"
            return False

        # Base capture buffers (executed comes from ros executed)
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()
        self._seen_planned_ok.clear()
        self._optimized_rt_by_segment.clear()

        # optimizer config from recipe (SSoT)
        if not self._read_optimize_cfg_from_recipe():
            return False

        if not callable(getattr(self._ros, "moveit_optimize_trajectory", None)):
            self._error_msg = "Optimize: ros.moveit_optimize_trajectory fehlt."
            return False
        if not callable(getattr(self._ros, "moveit_execute_trajectory", None)):
            self._error_msg = "Optimize: ros.moveit_execute_trajectory fehlt."
            return False

        # Push cfg now (Flattened!)
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
    # YAML handling
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_jtbysegment_yaml_v1(d: Dict[str, Any]) -> None:
        if not isinstance(d, dict) or int(d.get("version", 0)) != 1:
            raise ValueError("Invalid version/root")
        segs = d.get("segments")
        if not isinstance(segs, dict):
            raise ValueError("No segments")

    def _segment_has_points(self, seg_name: str) -> bool:
        seg = self._planned_loaded_yaml.get("segments", {}).get(seg_name)
        return bool(seg and isinstance(seg.get("points"), list) and len(seg["points"]) >= 2)

    def _jt_dict_from_yaml_segment(self, seg_name: str) -> Dict[str, Any]:
        return self._planned_loaded_yaml["segments"][seg_name]

    # ------------------------------------------------------------------
    # Config Loading (FLATTENING FIX)
    # ------------------------------------------------------------------

    def _read_optimize_cfg_from_recipe(self) -> bool:
        """
        Reads recipe.planner['optimize'] and flattens parameters so MoveItPyNode
        receives 'max_velocity_scaling_factor' at top level.
        """
        try:
            planner = getattr(self._recipe, "planner", {}) or {}
        except Exception:
            planner = {}

        cfg = planner.get("optimize") if isinstance(planner, dict) else None
        if not isinstance(cfg, dict):
            self._error_msg = "Optimize: Config in recipe fehlt."
            return False

        pipeline = str(cfg.get("pipeline") or "").strip()
        planner_id = str(cfg.get("planner_id") or "").strip()
        raw_params = cfg.get("params") or {}

        if not planner_id:
            self._error_msg = "Optimize: planner_id fehlt."
            return False

        # --- FLATTENING LOGIC ---
        self._opt_cfg = {
            "role": "optimize",
            "pipeline": pipeline,
            "planner_id": planner_id,
        }

        if isinstance(raw_params, dict):
            # Map generic names to MoveIt names
            if "velocity_scaling" in raw_params:
                self._opt_cfg["max_velocity_scaling_factor"] = float(raw_params["velocity_scaling"])
            elif "vel_scale" in raw_params:
                self._opt_cfg["max_velocity_scaling_factor"] = float(raw_params["vel_scale"])

            if "acceleration_scaling" in raw_params:
                self._opt_cfg["max_acceleration_scaling_factor"] = float(raw_params["acceleration_scaling"])
            elif "acc_scale" in raw_params:
                self._opt_cfg["max_acceleration_scaling_factor"] = float(raw_params["acc_scale"])

            # Copy everything else flat (except the aliases above)
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
    # Segment Control
    # ------------------------------------------------------------------

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and bool(self._skip_home):
            return False
        return bool(self._yaml_segments_present.get(seg_name, False))

    def _on_enter_segment(self, seg_name: str) -> None:
        # 1) Skip if needed
        if seg_name == STATE_MOVE_HOME and bool(self._skip_home):
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        present = bool(self._yaml_segments_present.get(seg_name, False))
        if not present:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return

        # 2) Satisfy base ordering guard deterministically.
        #    Optimize does not emit PLANNED:OK, so allow EXECUTED:OK.
        self._seen_planned_ok[seg_name] = True

        # 3) Convert YAML JT dict -> RobotTrajectory
        try:
            baseline_jt = self._planned_jt_by_segment.get(seg_name)
            if not isinstance(baseline_jt, dict):
                raise ValueError("missing planned baseline jt dict")
            in_rt = self._robot_trajectory_from_jt_dict(baseline_jt)
        except Exception as e:
            self._signal_error(f"Optimize: YAML/JT->RobotTrajectory failed for {seg_name}: {e}")
            return

        # 4) Trigger optimization request
        self._start_optimize_for_segment(seg_name, in_rt)

    # ------------------------------------------------------------------
    # Optimize execution
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

        # Ensure config is current
        self._publish_planner_cfg_to_node()

        self._opt_waiting = True
        self._opt_wait_seg = str(seg_name)
        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass
        self._opt_timeout_timer.start(int(self._OPTIMIZE_TIMEOUT_MS))
        self._opt_prev_msg = self._safe_get_optimized_msg()

        try:
            self._ros.moveit_optimize_trajectory(in_rt, segment=str(seg_name))
        except Exception as ex:
            try:
                self._opt_timeout_timer.stop()
            except Exception:
                pass
            self._opt_waiting = False
            self._signal_error(f"Optimize failed request: {ex}")

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj_changed(self, msg: object) -> None:
        if not isinstance(msg, RobotTrajectoryMsg) or not self._opt_waiting or self._stop_requested:
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

        # Store optimized RT (for execution)
        self._optimized_rt_by_segment[seg] = msg

        # Planned (for evaluation + pairing) = optimized version
        opt_jt = self._jt_from_any(msg)
        if opt_jt is not None:
            self._planned_jt_by_segment[seg] = opt_jt

        self._execute_optimized_segment(seg)

    @QtCore.pyqtSlot()
    def _on_optimize_timeout(self) -> None:
        if not self._opt_waiting or self._stop_requested:
            return

        seg = str(self._opt_wait_seg)
        self._opt_waiting = False

        last = self._safe_get_optimized_msg()
        if last is not None and (self._opt_prev_msg is None or last is not self._opt_prev_msg):
            _LOG.info("Optimize: Late arrival via getter logic (seg=%s).", seg)
            self._optimized_rt_by_segment[seg] = last
            opt_jt = self._jt_from_any(last)
            if opt_jt is not None:
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
        try:
            self._ros.moveit_execute_trajectory(rt, segment=str(seg_name))
        except Exception as e:
            self._signal_error(f"Optimize: Exec failed: {e}")

    # ------------------------------------------------------------------
    # Override pairing source for Optimize
    # ------------------------------------------------------------------

    def _get_step_sources(self) -> Tuple[Any, Any]:
        seg = str(self._current_state or "")
        planned = self._planned_jt_by_segment.get(seg)

        re_ = getattr(self._ros, "moveit_executed_trajectory", None)
        executed = re_() if callable(re_) else self._last_executed_any
        return planned, executed

    # ------------------------------------------------------------------
    # Final Payload
    # ------------------------------------------------------------------

    def _build_traj_payload(self) -> Tuple[Dict[str, Any], Dict[str, Any]]:
        segments: Dict[str, Any] = {}
        for seg in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT, STATE_MOVE_HOME):
            if seg == STATE_MOVE_HOME and bool(self._skip_home):
                continue
            jt = self._planned_jt_by_segment.get(seg)
            seg_yaml = self._jt_dict_to_segment_yaml(jt)
            if seg_yaml:
                segments[seg] = seg_yaml

        planned = {"version": 1, "segments": segments}
        executed = self._jt_by_segment_yaml(which="executed")
        return planned, executed

    def _on_finished(self) -> None:
        planned, executed = self._build_traj_payload()
        self._rr.set_planned(traj=copy.deepcopy(planned))
        self._rr.set_executed(traj=copy.deepcopy(executed))
        self.notifyFinished.emit(self._rr.to_process_payload())
        self._cleanup()

    # ------------------------------------------------------------------
    # Helper: YAML->JT Conversion (needed for input)
    # ------------------------------------------------------------------

    @staticmethod
    def _duration_from_yaml_time(tfs: Any) -> RosDuration:
        """
        Accepts either:
          - {"sec": int, "nanosec": int}  (internal dict form)
          - [sec, nanosec]               (JTBySegment YAML v1 form)
        """
        d = RosDuration()
        if isinstance(tfs, dict):
            d.sec = int(tfs.get("sec", 0))
            d.nanosec = int(tfs.get("nanosec", 0))
            return d
        if isinstance(tfs, (list, tuple)) and len(tfs) == 2:
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
            if not isinstance(p, dict):
                continue
            if "positions" not in p:
                continue
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in list(p.get("positions") or [])]
            pt.time_from_start = self._duration_from_yaml_time(p.get("time_from_start"))
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
