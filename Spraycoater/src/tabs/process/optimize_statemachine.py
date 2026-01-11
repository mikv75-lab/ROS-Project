# -*- coding: utf-8 -*-
# File: tabs/process/optimize_statemachine.py
from __future__ import annotations

import json
import logging
from typing import Optional, Any, Dict, List, Tuple, Iterable

from PyQt6 import QtCore

from moveit_msgs.msg import RobotTrajectory as RobotTrajectoryMsg  # type: ignore

from .base_statemachine import (
    BaseProcessStatemachine,
    STATE_MOVE_PREDISPENSE,
    STATE_MOVE_RECIPE,
    STATE_MOVE_RETREAT,
    STATE_MOVE_HOME,
)

_LOG = logging.getLogger("tabs.process.optimize_statemachine")


class ProcessOptimizeStatemachine(BaseProcessStatemachine):
    """
    Optimize run:
      - bewegt wie Validate (Segmentstruktur + Rezeptpunkte streamen)
      - danach (am Ende von MOVE_RECIPE) wird die Trajectory im MoveItPy-Node optimiert/retimed
        über moveit_py/optimize_trajectory (RobotTrajectoryMsg).

    SINGLE SOURCE OF TRUTH:
      - Pfade/Frame/Speed kommen aus dem Recipe (wie Validate)
      - Optimizer-Config kommt aus recipe.planner["optimize"] (role/pipeline/planner_id/params)
      - Übergabe an Node erfolgt über moveit_py/set_planner_cfg (String JSON) + optimize_trajectory RobotTrajectoryMsg

    WICHTIG:
      - RosBridge.moveit_optimize_trajectory(traj, segment=...) akzeptiert KEINE kwargs wie trajectory/pipeline/planner_id/params.
        Diese Config wird vorab als planner_cfg an den Node geschickt.
      - optimize_trajectory Topic erwartet RobotTrajectoryMsg (nicht JT-dict).
    """

    ROLE = "optimize"

    _NEXT_POSE_QT_DELAY_MS = 0
    _JOINT_UPDATE_FALLBACK_MS = 200

    # timeout waiting for optimized output (ms)
    _OPTIMIZE_TIMEOUT_MS = 4000

    def __init__(
        self,
        *,
        recipe: Any,
        ros: Any,
        parent: Optional[QtCore.QObject] = None,
        max_retries: int = 2,
        skip_home: bool = False,
        side: str = "top",
    ) -> None:
        super().__init__(recipe=recipe, ros=ros, parent=parent, max_retries=max_retries, skip_home=skip_home)

        self._side = str(side or "top")

        # recipe-driven execution inputs (same as Validate)
        self._compiled_pts_mm: List[Tuple[float, float, float]] = []
        self._cmd_pts_by_seg: Dict[str, List[Tuple[float, float, float]]] = {}
        self._pending_recipe_mm: List[Tuple[float, float, float]] = []

        self._frame: str = "world"
        self._speed_mm_s: float = 200.0

        # gating (same pattern as Validate)
        self._await_joint_update_for_next_pose: bool = False
        self._joint_sig_connected: bool = False
        self._joint_fallback_timer = QtCore.QTimer(self)
        self._joint_fallback_timer.setSingleShot(True)
        self._joint_fallback_timer.timeout.connect(self._on_joint_fallback_timeout)

        # optimizer config (from recipe.planner.optimize)
        self._opt_cfg: Dict[str, Any] = {}
        self._opt_planner_cfg_json: str = ""

        # optimize flow control
        self._opt_waiting: bool = False
        self._opt_done: bool = False
        self._opt_prev_msg: Optional[RobotTrajectoryMsg] = None
        self._opt_last_msg: Optional[RobotTrajectoryMsg] = None
        self._opt_timeout_timer = QtCore.QTimer(self)
        self._opt_timeout_timer.setSingleShot(True)
        self._opt_timeout_timer.timeout.connect(self._on_optimize_timeout)

        # connect robot joints gating (recipe streaming)
        try:
            self._ros.robot.signals.jointsChanged.connect(self._on_joints_changed)
            self._joint_sig_connected = True
        except Exception:
            self._joint_sig_connected = False

        # connect optimized trajectory signal (optional; topics.yaml must provide optimized_trajectory_rt)
        try:
            sig = getattr(getattr(self._ros, "moveitpy", None), "signals", None)
            if sig is not None and hasattr(sig, "optimizedTrajectoryChanged"):
                sig.optimizedTrajectoryChanged.connect(self._on_optimized_traj_changed)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Hooks
    # ------------------------------------------------------------------

    def _prepare_run(self) -> bool:
        # --- frame (same as Validate) ---
        self._frame = "scene"
        try:
            pc = getattr(self._recipe, "paths_compiled", None)
            if isinstance(pc, dict) and pc.get("frame"):
                self._frame = str(pc.get("frame"))
        except Exception:
            pass

        # --- speed (same as Validate) ---
        self._speed_mm_s = 200.0
        try:
            params = getattr(self._recipe, "parameters", {}) or {}
            self._speed_mm_s = float(params.get("speed_mm_s", 200.0))
        except Exception:
            pass

        # --- compiled points (mm) ---
        self._compiled_pts_mm = self._get_compiled_points_mm(self._side)

        # segment split like Validate
        self._cmd_pts_by_seg.clear()
        self._pending_recipe_mm.clear()

        self._await_joint_update_for_next_pose = False
        self._joint_fallback_timer.stop()

        self._opt_waiting = False
        self._opt_done = False
        self._opt_timeout_timer.stop()
        self._opt_prev_msg = self._safe_get_optimized_msg()
        self._opt_last_msg = None

        # optimizer config from recipe (SSoT)
        if not self._read_optimize_cfg_from_recipe():
            return False

        # Base buffers are cleared by Base.start(), but keep strictness here too
        self._planned_by_segment.clear()
        self._executed_by_segment.clear()
        self._planned_steps_by_segment.clear()
        self._executed_steps_by_segment.clear()

        if not self._compiled_pts_mm:
            self._signal_error(f"Optimize: compiled path ist leer (side='{self._side}').")
            return False

        pts = self._compiled_pts_mm
        n = len(pts)

        pre = [pts[0]] if n >= 1 else []
        ret = [pts[-1]] if n >= 2 else []
        mid = pts[1:-1] if n >= 3 else []

        self._cmd_pts_by_seg[STATE_MOVE_PREDISPENSE] = pre
        self._cmd_pts_by_seg[STATE_MOVE_RECIPE] = list(mid)
        self._cmd_pts_by_seg[STATE_MOVE_RETREAT] = ret
        self._cmd_pts_by_seg[STATE_MOVE_HOME] = []

        self._pending_recipe_mm = list(self._cmd_pts_by_seg[STATE_MOVE_RECIPE])

        # push planner cfg to node now (best-effort), so subsequent optimize uses correct settings
        self._publish_planner_cfg_to_node()

        return True

    def _segment_exists(self, seg_name: str) -> bool:
        if seg_name == STATE_MOVE_HOME and self._skip_home:
            return False
        if seg_name in (STATE_MOVE_PREDISPENSE, STATE_MOVE_RECIPE, STATE_MOVE_RETREAT):
            return bool(self._cmd_pts_by_seg.get(seg_name))
        return True

    def _on_enter_segment(self, seg_name: str) -> None:
        self._await_joint_update_for_next_pose = False
        self._joint_fallback_timer.stop()

        if seg_name == STATE_MOVE_PREDISPENSE:
            self._run_single_pose_segment(STATE_MOVE_PREDISPENSE)
            return
        if seg_name == STATE_MOVE_RECIPE:
            # entering recipe segment resets optimize state
            self._opt_waiting = False
            self._opt_done = False
            self._opt_timeout_timer.stop()
            self._opt_prev_msg = self._safe_get_optimized_msg()
            self._opt_last_msg = None
            self._run_recipe_stream_segment()
            return
        if seg_name == STATE_MOVE_RETREAT:
            self._run_single_pose_segment(STATE_MOVE_RETREAT)
            return
        if seg_name == STATE_MOVE_HOME:
            self._run_home_segment()
            return

        self._signal_error(f"Optimize: Unknown segment '{seg_name}'")

    def _should_transition_on_ok(self, seg_name: str, result: str) -> bool:
        # RECIPE streaming: stay until all points have been issued
        if seg_name == STATE_MOVE_RECIPE:
            if self._pending_recipe_mm:
                self._await_joint_update_for_next_pose = True
                self._arm_joint_fallback_timer()
                return False

            # all poses issued. Now run optimization exactly once.
            if not self._opt_done:
                if not self._opt_waiting:
                    self._start_optimize_current_segment()
                return False

            # optimization completed: allow transition now
            return True

        return True

    def _on_segment_ok(self, seg_name: str) -> None:
        """
        We snapshot planned/executed as usual.
        For MOVE_RECIPE, if optimization produced an optimized RobotTrajectoryMsg,
        we overwrite planned (and optionally executed) dicts for this segment from that optimized result.
        """
        super()._on_segment_ok(seg_name)

        if seg_name != STATE_MOVE_RECIPE:
            return

        opt_msg = self._opt_last_msg
        if opt_msg is None:
            return

        # overwrite planned segment with optimized JT (dict form for persistence)
        opt_jt = self._jt_from_any(opt_msg)
        if opt_jt is not None:
            self._planned_by_segment[STATE_MOVE_RECIPE] = opt_jt
        # executed optimization is node-defined; if your node outputs optimized as executed artifact instead,
        # you can also map it here. For now we keep executed as captured.

    # ------------------------------------------------------------------
    # Segment runners (movement) - same as Validate
    # ------------------------------------------------------------------

    def _run_single_pose_segment(self, seg_name: str) -> None:
        pts = self._cmd_pts_by_seg.get(seg_name) or []
        if not pts:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        x, y, z = pts[0]
        self._ros_move_pose_mm(x, y, z)

    def _run_recipe_stream_segment(self) -> None:
        if not self._pending_recipe_mm:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        self._send_next_recipe_pose()

    def _send_next_recipe_pose(self) -> None:
        self._joint_fallback_timer.stop()
        self._await_joint_update_for_next_pose = False

        if not self._pending_recipe_mm:
            return
        x, y, z = self._pending_recipe_mm.pop(0)
        self._ros_move_pose_mm(x, y, z)

    def _run_home_segment(self) -> None:
        if self._skip_home:
            QtCore.QTimer.singleShot(0, self._sig_done.emit)
            return
        try:
            self._ros.moveitpy.signals.moveToHomeRequestedWithSpeed.emit(float(self._speed_mm_s))
        except Exception as ex:
            self._signal_error(f"Optimize: move_home failed: {ex}")

    # ------------------------------------------------------------------
    # Robot state gating (recipe streaming) - same as Validate
    # ------------------------------------------------------------------

    def _arm_joint_fallback_timer(self) -> None:
        self._joint_fallback_timer.stop()
        ms = int(self._JOINT_UPDATE_FALLBACK_MS)
        if ms <= 0:
            ms = 1
        self._joint_fallback_timer.start(ms)

    @QtCore.pyqtSlot()
    def _on_joint_fallback_timeout(self) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if self._current_state != STATE_MOVE_RECIPE:
            return
        if not self._await_joint_update_for_next_pose:
            return
        if not self._pending_recipe_mm:
            return

        _LOG.warning(
            "Optimize: jointsChanged fallback triggered (no joint update within %d ms). Proceeding with next recipe pose.",
            int(self._JOINT_UPDATE_FALLBACK_MS),
        )

        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    @QtCore.pyqtSlot(object)
    def _on_joints_changed(self, _msg: object) -> None:
        if not self._machine or not self._machine.isRunning():
            return
        if self._stop_requested:
            return
        if self._current_state != STATE_MOVE_RECIPE:
            return
        if not self._await_joint_update_for_next_pose:
            return
        if not self._pending_recipe_mm:
            return

        self._joint_fallback_timer.stop()
        self._await_joint_update_for_next_pose = False
        QtCore.QTimer.singleShot(self._NEXT_POSE_QT_DELAY_MS, self._send_next_recipe_pose)

    # ------------------------------------------------------------------
    # ROS facade helpers
    # ------------------------------------------------------------------

    def _ros_move_pose_mm(self, x: float, y: float, z: float) -> None:
        from geometry_msgs.msg import PoseStamped  # type: ignore

        try:
            # speed signal (best-effort)
            try:
                self._ros.moveitpy.signals.motionSpeedChanged.emit(float(self._speed_mm_s))
            except Exception:
                pass

            ps = PoseStamped()
            ps.header.frame_id = str(self._frame or "world")

            try:
                node = (
                    getattr(self._ros, "node", None)
                    or getattr(self._ros, "_node", None)
                    or getattr(self._ros, "rclpy_node", None)
                )
                if node is not None:
                    ps.header.stamp = node.get_clock().now().to_msg()
            except Exception:
                pass

            # mm -> m
            ps.pose.position.x = float(x) / 1000.0
            ps.pose.position.y = float(y) / 1000.0
            ps.pose.position.z = float(z) / 1000.0
            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            self._ros.moveitpy.signals.moveToPoseRequested.emit(ps)
        except Exception as ex:
            self._signal_error(f"Optimize: move_pose failed ({x:.1f},{y:.1f},{z:.1f}): {ex}")

    # ------------------------------------------------------------------
    # Optimize execution (Node post-processing)
    # ------------------------------------------------------------------

    def _read_optimize_cfg_from_recipe(self) -> bool:
        """
        Strict: read recipe.planner['optimize'] dict.
        Build a JSON payload we will send via set_planner_cfg.
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

        # Store config dict; the node decides how to apply these for optimize.
        self._opt_cfg = {
            "role": "optimize",
            "pipeline": pipeline,
            "planner_id": planner_id,
            "params": params,
        }

        try:
            self._opt_planner_cfg_json = json.dumps(self._opt_cfg, ensure_ascii=False)
        except Exception:
            # should not happen; but keep strict
            self._signal_error("Optimize: planner cfg konnte nicht zu JSON serialisiert werden.")
            return False

        return True

    def _publish_planner_cfg_to_node(self) -> None:
        """
        Best-effort: send planner cfg JSON to node.
        This is the mechanism to ensure "values are taken from recipe" for optimize.
        """
        try:
            mp = getattr(self._ros, "moveitpy", None)
            sig = getattr(mp, "signals", None) if mp is not None else None
            if sig is None or not hasattr(sig, "plannerCfgChanged"):
                _LOG.warning("Optimize: MoveItPyBridge hat kein plannerCfgChanged – kann optimize cfg nicht setzen.")
                return
            # bridge accepts object (string or dict), it will json.dumps if not string
            sig.plannerCfgChanged.emit(self._opt_cfg)
        except Exception as ex:
            _LOG.warning("Optimize: publish planner cfg failed: %s", ex)

    def _start_optimize_current_segment(self) -> None:
        """
        Trigger optimization in node using RobotTrajectoryMsg.
        Waits for optimized_trajectory_rt (latched) -> optimizedTrajectoryChanged.
        """
        if self._opt_waiting:
            return

        # ensure node has correct cfg (in case someone changed it mid-run)
        self._publish_planner_cfg_to_node()

        planned_any, _executed_any = self._get_step_sources()

        planned_rt = self._robot_traj_from_any(planned_any)
        if planned_rt is None:
            self._signal_error("Optimize: keine planned RobotTrajectory verfügbar (für optimize_trajectory).")
            return

        self._opt_waiting = True
        self._opt_done = False
        self._opt_timeout_timer.stop()
        self._opt_timeout_timer.start(int(self._OPTIMIZE_TIMEOUT_MS))

        # remember previous optimized message to avoid immediately accepting a latched old one
        self._opt_prev_msg = self._safe_get_optimized_msg()
        self._opt_last_msg = None

        try:
            # tag segment (best-effort) + publish optimize request
            self._ros.moveit_optimize_trajectory(planned_rt, segment=STATE_MOVE_RECIPE)
        except Exception as ex:
            self._opt_timeout_timer.stop()
            self._opt_waiting = False
            self._signal_error(f"Optimize: optimize_trajectory publish failed: {ex}")

    def _safe_get_optimized_msg(self) -> Optional[RobotTrajectoryMsg]:
        try:
            v = self._ros.moveit_optimized_trajectory()
            return v if isinstance(v, RobotTrajectoryMsg) else None
        except Exception:
            return None

    @QtCore.pyqtSlot(object)
    def _on_optimized_traj_changed(self, msg: object) -> None:
        if not isinstance(msg, RobotTrajectoryMsg):
            return

        # ignore if not currently waiting
        if not self._opt_waiting:
            return

        # ignore an old latched message equal to what we saw before we triggered optimize
        if self._opt_prev_msg is not None and msg is self._opt_prev_msg:
            return

        self._opt_timeout_timer.stop()
        self._opt_last_msg = msg
        self._opt_waiting = False
        self._opt_done = True

        # Now we must snapshot segment and transition manually.
        # We are still in MOVE_RECIPE state; BaseProcessStatemachine won't transition until we emit done.
        self._on_segment_ok(STATE_MOVE_RECIPE)
        QtCore.QTimer.singleShot(0, self._sig_done.emit)

    @QtCore.pyqtSlot()
    def _on_optimize_timeout(self) -> None:
        if not self._opt_waiting:
            return
        self._opt_waiting = False
        self._signal_error(
            f"Optimize: Timeout waiting for optimized_trajectory_rt after {int(self._OPTIMIZE_TIMEOUT_MS)} ms. "
            "Prüfe: topics.yaml publish id=optimized_trajectory_rt + Node publish."
        )

    @staticmethod
    def _robot_traj_from_any(obj: Any, *, _depth: int = 0, _max_depth: int = 6) -> Optional[RobotTrajectoryMsg]:
        """
        Extract RobotTrajectoryMsg from common wrappers/containers.
        Does NOT convert dict->msg.
        """
        if obj is None:
            return None
        if _depth > _max_depth:
            return None

        if isinstance(obj, RobotTrajectoryMsg):
            return obj

        # common attribute: robot_trajectory or trajectory
        for attr in ("robot_trajectory", "robotTrajectory", "trajectory", "result", "data", "response"):
            try:
                v = getattr(obj, attr, None)
            except Exception:
                v = None
            if v is not None:
                rt = ProcessOptimizeStatemachine._robot_traj_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if rt is not None:
                    return rt

        # dict traversal (only to find embedded msg objects)
        if isinstance(obj, dict) and obj:
            for k in ("robot_trajectory", "robotTrajectory", "trajectory", "planned", "executed", "result", "data"):
                try:
                    v = obj.get(k)
                except Exception:
                    v = None
                rt = ProcessOptimizeStatemachine._robot_traj_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if rt is not None:
                    return rt
            for v in obj.values():
                rt = ProcessOptimizeStatemachine._robot_traj_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if rt is not None:
                    return rt

        # list/tuple
        if isinstance(obj, (list, tuple)) and obj:
            for v in obj:
                rt = ProcessOptimizeStatemachine._robot_traj_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                if rt is not None:
                    return rt

        # __dict__ scan
        try:
            d = getattr(obj, "__dict__", None)
            if isinstance(d, dict) and d:
                for v in d.values():
                    rt = ProcessOptimizeStatemachine._robot_traj_from_any(v, _depth=_depth + 1, _max_depth=_max_depth)
                    if rt is not None:
                        return rt
        except Exception:
            pass

        return None

    # ------------------------------------------------------------------
    # Compiled points extraction (robust for ndarray OR list) - same as Validate
    # ------------------------------------------------------------------

    def _get_compiled_points_mm(self, side: str) -> List[Tuple[float, float, float]]:
        out: List[Tuple[float, float, float]] = []
        try:
            pts = self._recipe.compiled_points_mm_for_side(str(side))
        except Exception:
            pts = None

        if pts is None:
            return out

        rows: Iterable[Any]
        if hasattr(pts, "tolist"):
            try:
                rows = pts.tolist()
            except Exception:
                rows = pts
        else:
            rows = pts

        try:
            for row in rows:
                if row is None:
                    continue
                try:
                    x = float(row[0])
                    y = float(row[1])
                    z = float(row[2])
                except Exception:
                    continue
                out.append((x, y, z))
        except Exception:
            return out

        return out

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def _cleanup(self) -> None:
        try:
            self._joint_fallback_timer.stop()
        except Exception:
            pass

        try:
            self._opt_timeout_timer.stop()
        except Exception:
            pass

        if self._joint_sig_connected:
            try:
                self._ros.robot.signals.jointsChanged.disconnect(self._on_joints_changed)
            except Exception:
                pass
            self._joint_sig_connected = False

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
