# ros/clients/motion_client.py
from __future__ import annotations

import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Callable

import rclpy
from rclpy.duration import Duration as RclDuration
from rclpy.node import Node

from builtin_interfaces.msg import Duration as RosDuration
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from moveit_msgs.msg import (
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
    RobotTrajectory,
)
from moveit_msgs.srv import (
    GetPositionIK,
    GetMotionPlan,
    ExecuteKnownTrajectory,
)

from ..common.topics import Topics
from ..common.qos import qos_default, qos_latched, qos_sensor_data


def _err_text(code: int) -> str:
    m = {
        MoveItErrorCodes.SUCCESS: "SUCCESS",
        MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
        MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
        MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
        MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
        MoveItErrorCodes.START_STATE_IN_COLLISION: "START_IN_COLLISION",
        MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
        MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
    }
    return m.get(int(code), f"ERROR_{int(code)}")


class MotionClient:
    """
    Motion-Client für zwei Pfade:
      1) IK → JointTrajectory direkt an den Controller
      2) (Optional) Planner: IK-Ziel → JointConstraints → GetMotionPlan → Execute

    Dependencies (Topics/Services):
      - pub:   joint_trajectory(controller)                 (trajectory_msgs/JointTrajectory)
      - sub:   joint_states                                  (sensor_msgs/JointState)
      - srv:   compute_ik                                    (moveit_msgs/srv/GetPositionIK)
      - srv:   plan_kinematic_path                           (moveit_msgs/srv/GetMotionPlan)
      - srv:   execute_kinematic_path                        (moveit_msgs/srv/ExecuteKnownTrajectory)

    Hinweis:
      * Für named poses kannst du entweder einen PosesClient verwenden,
        oder die Convenience-Methode move_to_named_pose(...), die das latched Topic einmalig liest.
    """

    def __init__(
        self,
        node: Node,
        topics: Optional[Topics] = None,
        *,
        group_name: str = "meca_arm_group",
        controller_name: Optional[str] = None,   # wenn None -> Topics.controller wird verwendet
        ee_link_candidates: Sequence[Optional[str]] = ("tcp", "meca_axis_6_link", "tool0", "flange", None),
        default_time_to_target: float = 2.0,
        # optionaler Pose-Resolver: name -> PoseStamped (z. B. aus PosesClient)
        pose_resolver: Optional[Callable[[str], Optional[PoseStamped]]] = None,
    ) -> None:
        self._node = node
        self._log = node.get_logger()
        self._topics = topics or Topics()
        if controller_name:
            # lokale Topics-Instanz mit überschriebenem Controller
            self._topics = Topics(base_ns=self._topics.base_ns, controller=controller_name, servo_ns=self._topics.servo_ns)

        self._group = group_name
        self._ee_links = tuple(ee_link_candidates)
        self._default_T = float(default_time_to_target)
        self._pose_resolver = pose_resolver

        # State
        self._last_js: Optional[JointState] = None

        # IO
        self._pub_traj = node.create_publisher(JointTrajectory, self._topics.joint_trajectory(), qos_default())
        self._sub_js = node.create_subscription(JointState, self._topics.joint_states, self._on_joint_states, qos_sensor_data())

        # Service Clients
        self._client_ik = node.create_client(GetPositionIK, self._topics.compute_ik)
        self._client_plan = node.create_client(GetMotionPlan, self._topics.plan_kinematic_path)
        self._client_exec = node.create_client(ExecuteKnownTrajectory, self._topics.execute_kinematic_path)

        self._log.info("MotionClient bereit (IK + optional Planner).")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def move_to_named_pose(
        self,
        name: str,
        *,
        time_to_target: Optional[float] = None,
        use_planner: bool = False,
        plan_only: bool = False,
        avoid_collisions: bool = False,
        timeout_sec: float = 2.5,
    ) -> bool:
        """
        Named Pose anfahren (holt PoseStamped aus PosesClient oder einmalig latched vom Topic).
        """
        ps = self._resolve_pose(name)
        if ps is None:
            self._log.error(f"MotionClient: Keine Pose '{name}' verfügbar.")
            return False
        return self.move_to_pose_stamped(
            ps,
            time_to_target=time_to_target,
            use_planner=use_planner,
            plan_only=plan_only,
            avoid_collisions=avoid_collisions,
            timeout_sec=timeout_sec,
        )

    def move_to_pose_stamped(
        self,
        target: PoseStamped,
        *,
        time_to_target: Optional[float] = None,
        use_planner: bool = False,
        plan_only: bool = False,
        avoid_collisions: bool = False,
        timeout_sec: float = 2.5,
    ) -> bool:
        """
        PoseStamped anfahren:
          - IK direkt → JointTrajectory publish
          - oder Planner (IK → JointConstraints → GetMotionPlan → Execute/plan_only)
        """
        if use_planner:
            plan = self._plan_via_ik_and_constraints(target, avoid_collisions=avoid_collisions, timeout_sec=timeout_sec)
            if plan is None:
                return False
            if plan_only:
                self._log.info("MotionClient: Plan erstellt (plan_only=True).")
                return True
            return self._execute_plan(plan, wait=True, timeout_sec=timeout_sec)

        # Direkt-IK
        js_goal = self._compute_ik(target, avoid_collisions=avoid_collisions, timeout_sec=timeout_sec)
        if js_goal is None:
            return False
        return self._send_trajectory_to_controller(js_goal, time_to_target or self._default_T)

    def move_joints(
        self,
        joint_positions: Dict[str, float],
        *,
        time_to_target: Optional[float] = None,
    ) -> bool:
        """Direktes Joint-Ziel (rad) an den Controller schicken."""
        if not joint_positions:
            self._log.error("MotionClient: move_joints() ohne Ziele.")
            return False
        return self._send_trajectory_to_controller_map(joint_positions, time_to_target or self._default_T)

    # ------------------------------------------------------------------
    # Internals: JointState / Helpers
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        self._last_js = msg

    def _current_arm_joint_order(self) -> Optional[List[str]]:
        """
        Ermittelt eine sinnvolle Reihenfolge der 6 Arm-Gelenke basierend auf dem letzten JointState.
        Erwartet Namen wie 'meca_axis_1_joint' ... 'meca_axis_6_joint'.
        """
        if self._last_js is None:
            return None
        idx = {n: i for i, n in enumerate(self._last_js.name)}
        wanted = [f"meca_axis_{i}_joint" for i in range(1, 7)]
        order = [n for n in wanted if n in idx]
        return order if order else None

    # ------------------------------------------------------------------
    # Internals: Pose resolving
    # ------------------------------------------------------------------

    def _resolve_pose(self, name: str) -> Optional[PoseStamped]:
        """
        Versucht Pose zu besorgen:
          1) via injected pose_resolver(name)
          2) via einmaligem Subscribe auf latched Topic /meca/poses/<name>
        """
        name = name.strip()
        if self._pose_resolver:
            try:
                ps = self._pose_resolver(name)
                if ps:
                    return ps
            except Exception as e:
                self._log.warn(f"MotionClient: pose_resolver Fehler: {e}")

        # Fallback: latched Topic einmalig
        topic = self._topics.pose(name)
        box: List[PoseStamped] = []

        def _cb(msg: PoseStamped):
            box.append(msg)

        sub = self._node.create_subscription(PoseStamped, topic, _cb, qos_latched())
        try:
            ok = self._wait_until(lambda: len(box) > 0, timeout=1.2)
            if not ok:
                return None
            return box[0]
        finally:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Internals: IK
    # ------------------------------------------------------------------

    def _compute_ik(
        self,
        target_ps: PoseStamped,
        *,
        avoid_collisions: bool = False,
        timeout_sec: float = 2.5,
    ) -> Optional[JointState]:
        """ruft /compute_ik, probiert nacheinander die ee_link_candidates."""
        if self._last_js is None:
            self._log.warning("MotionClient: Keine JointStates – IK nicht möglich.")
            return None

        if not self._client_ik.wait_for_service(timeout_sec=timeout_sec):
            self._log.error("MotionClient: /compute_ik nicht verfügbar.")
            return None

        last_code: Optional[int] = None
        for ik_link in self._ee_links:
            req = GetPositionIK.Request()
            req.ik_request.group_name = self._group
            if ik_link:
                req.ik_request.ik_link_name = ik_link
            req.ik_request.pose_stamped = target_ps
            req.ik_request.avoid_collisions = bool(avoid_collisions)
            req.ik_request.attempts = 10
            req.ik_request.timeout = RclDuration(seconds=0.5).to_msg()
            req.ik_request.robot_state.joint_state = self._last_js

            fut = self._client_ik.call_async(req)
            rclpy.spin_until_future_complete(self._node, fut, timeout_sec=timeout_sec)
            if not fut.done() or fut.result() is None:
                continue

            res = fut.result()
            last_code = int(res.error_code.val)
            if last_code == MoveItErrorCodes.SUCCESS:
                return res.solution.joint_state

        self._log.error(f"MotionClient: IK fehlgeschlagen ({_err_text(last_code or -1)}).")
        return None

    # ------------------------------------------------------------------
    # Internals: Planner (IK-Ziel → JointConstraints → GetMotionPlan)
    # ------------------------------------------------------------------

    def _plan_via_ik_and_constraints(
        self,
        target_ps: PoseStamped,
        *,
        avoid_collisions: bool = False,
        timeout_sec: float = 3.0,
        joint_tolerance: float = 1e-3,
    ) -> Optional[RobotTrajectory]:
        """
        Planner-Flow:
          1) IK → Ziel-JointState
          2) JointConstraints daraus bauen
          3) GetMotionPlan aufrufen
        """
        if self._last_js is None:
            self._log.warning("MotionClient: Keine JointStates – Planen nicht möglich.")
            return None

        js_goal = self._compute_ik(target_ps, avoid_collisions=avoid_collisions, timeout_sec=timeout_sec)
        if js_goal is None:
            return None

        if not self._client_plan.wait_for_service(timeout_sec=timeout_sec):
            self._log.error("MotionClient: /plan_kinematic_path nicht verfügbar.")
            return None

        # JointConstraints aus IK-Ziel
        jmap = {n: p for n, p in zip(js_goal.name, js_goal.position)}
        constraints = Constraints()
        for jn, pos in jmap.items():
            jc = JointConstraint()
            jc.joint_name = jn
            jc.position = float(pos)
            jc.tolerance_above = joint_tolerance
            jc.tolerance_below = joint_tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req = GetMotionPlan.Request()
        # start state: aktueller last_js
        req.motion_plan_request.start_state.joint_state = self._last_js
        req.motion_plan_request.group_name = self._group
        req.motion_plan_request.goal_constraints = [constraints]
        # Optional: Planner-IDs/Parameter kannst du später via params setzen
        # req.motion_plan_request.pipeline_id = "ompl"
        # req.motion_plan_request.planner_id = "RRTConnectkConfigDefault"
        req.motion_plan_request.allowed_planning_time = 2.0

        fut = self._client_plan.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=timeout_sec)
        if not fut.done() or fut.result() is None:
            self._log.error("MotionClient: Planner-Response leer.")
            return None

        res = fut.result()
        code = int(res.motion_plan_response.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            self._log.error(f"MotionClient: Planner-Fehler: {_err_text(code)}")
            return None

        traj: RobotTrajectory = res.motion_plan_response.trajectory
        if len(traj.joint_trajectory.points) == 0:
            self._log.error("MotionClient: Planner lieferte leere Trajectory.")
            return None

        self._log.info(f"MotionClient: Plan ok ({len(traj.joint_trajectory.points)} Punkte).")
        return traj

    def _execute_plan(self, traj: RobotTrajectory, *, wait: bool = True, timeout_sec: float = 5.0) -> bool:
        """ruft ExecuteKnownTrajectory auf."""
        if not self._client_exec.wait_for_service(timeout_sec=timeout_sec):
            self._log.error("MotionClient: /execute_kinematic_path nicht verfügbar.")
            return False

        req = ExecuteKnownTrajectory.Request()
        req.trajectory = traj
        req.wait_for_execution = bool(wait)

        fut = self._client_exec.call_async(req)
        rclpy.spin_until_future_complete(self._node, fut, timeout_sec=timeout_sec)
        if not fut.done() or fut.result() is None:
            self._log.error("MotionClient: Execute-Response leer/Timeout.")
            return False

        res = fut.result()
        code = int(res.error_code.val)
        if code != MoveItErrorCodes.SUCCESS:
            self._log.error(f"MotionClient: Execute-Fehler: {_err_text(code)}")
            return False

        self._log.info("MotionClient: Ausführung OK.")
        return True

    # ------------------------------------------------------------------
    # Internals: Controller Publish
    # ------------------------------------------------------------------

    def _send_trajectory_to_controller(self, js_goal: JointState, time_to_target: float) -> bool:
        """Ordnet Zielwerte gemäß aktueller Arm-Jointreihenfolge und publish't JointTrajectory."""
        order = self._current_arm_joint_order()
        if order is None:
            self._log.error("MotionClient: Unbekannte Arm-Joint-Namen im JointState.")
            return False

        target_map = {n: p for n, p in zip(js_goal.name, js_goal.position)}
        try:
            positions = [float(target_map[n]) for n in order]
        except KeyError as e:
            self._log.error(f"MotionClient: IK-Ziel enthält Joint '{e.args[0]}' nicht.")
            return False

        traj = JointTrajectory()
        traj.joint_names = order
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = RosDuration(
            sec=int(time_to_target),
            nanosec=int((time_to_target - int(time_to_target)) * 1e9),
        )
        traj.points.append(pt)

        self._pub_traj.publish(traj)
        self._log.info(f"MotionClient: JointTrajectory → Controller (T={time_to_target:.2f}s).")
        return True

    def _send_trajectory_to_controller_map(self, target_map: Dict[str, float], time_to_target: float) -> bool:
        order = self._current_arm_joint_order()
        if order is None:
            self._log.error("MotionClient: Unbekannte Arm-Joint-Namen im JointState.")
            return False

        try:
            positions = [float(target_map[n]) for n in order]
        except KeyError as e:
            self._log.error(f"MotionClient: Ziel enthält Joint '{e.args[0]}' nicht.")
            return False

        traj = JointTrajectory()
        traj.joint_names = order
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start = RosDuration(
            sec=int(time_to_target),
            nanosec=int((time_to_target - int(time_to_target)) * 1e9),
        )
        traj.points.append(pt)
        self._pub_traj.publish(traj)
        self._log.info(f"MotionClient: JointTrajectory (map) → Controller (T={time_to_target:.2f}s).")
        return True

    # ------------------------------------------------------------------
    # Small wait helper
    # ------------------------------------------------------------------

    def _wait_until(self, predicate, *, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        while time.monotonic() < deadline:
            try:
                rclpy.spin_once(self._node, timeout_sec=0.05)
            except Exception:
                time.sleep(0.05)
            if predicate():
                return True
        return predicate()
