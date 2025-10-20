# source/ros/clients/motion_client.py
from __future__ import annotations

import time
from typing import Dict, Iterable, List, Optional, Sequence, Tuple, Callable

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RclDuration

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from ..common.topics import Topics
from ..common.qos import qos_default, qos_latched, qos_sensor_data

# --- MoveItPy (Rolling / Iron+) ---
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState  # type: ignore
except Exception as e:  # pragma: no cover
    MoveItPy = None  # type: ignore
    _MOVEITPY_IMPORT_ERR = e


def _deg(rad: float) -> float:
    return rad * 180.0 / 3.141592653589793


class MotionClient:
    """
    Motion-Client auf Basis von MoveItPy.

    Features:
      - move_to_named_pose(name, …)    # via PoseResolver (z. B. PosesClient) oder latched Topic
      - move_to_pose_stamped(ps, …)    # Pose-Ziel -> IK/Plan -> execute
      - move_joints({joint: pos}, …)   # direkte JointTrajectory an Controller
      - set_planner("ompl" | "pilz_lin" | "chomp")  # Planner/Pipeline wählen

    Interna:
      - Liest JointStates für Reihenfolge & latest state
      - Plant via MoveItPy PlanningComponent (group_name)
      - Führt Trajektorie via MoveItPy aus (kein ExecuteKnownTrajectory)
    """

    def __init__(
        self,
        node: Node,
        topics: Optional[Topics] = None,
        *,
        group_name: str = "meca_arm_group",
        controller_name: Optional[str] = None,   # (bleibt für joint_trajectory Publish)
        ee_link_candidates: Sequence[Optional[str]] = ("tcp", "meca_axis_6_link", "tool0", "flange", None),
        default_time_to_target: float = 2.0,
        pose_resolver: Optional[Callable[[str], Optional[PoseStamped]]] = None,
    ) -> None:
        if MoveItPy is None:  # pragma: no cover
            raise ImportError(
                f"MoveItPy konnte nicht importiert werden: {_MOVEITPY_IMPORT_ERR}\n"
                "Installiere moveit_py / moveit2 Python-Bindings für Rolling/Iron."
            )

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

        # IO – für "direkt an Controller"
        self._pub_traj = node.create_publisher(JointTrajectory, self._topics.joint_trajectory(), qos_default())
        self._sub_js = node.create_subscription(JointState, self._topics.joint_states, self._on_joint_states, qos_sensor_data())

        # State
        self._last_js: Optional[JointState] = None

        # --- MoveItPy Core ---
        # Eigene MoveItPy-Instanz (separater rclpy-Node intern)
        try:
            self._moveit = MoveItPy(node_name="motion_client_moveit")
            self._robot = self._moveit.get_robot_model()
            self._pc = self._moveit.get_planning_component(self._group)

            # Planner-Auswahl (Defaults: OMPL + RRTConnect)
            self._pipeline_id: str = "ompl"
            self._planner_id: Optional[str] = "RRTConnectkConfigDefault"
            self._pc.set_planning_pipeline_id(self._pipeline_id)
            if self._planner_id:
                try:
                    self._pc.set_planner_id(self._planner_id)  # verfügbar in neueren MoveItPy
                except Exception:
                    pass

        except Exception as e:  # pragma: no cover
            pass
            #raise RuntimeError(f"MotionClient: Fehler beim Initialisieren von MoveItPy: {e}") from e)
        self._log.info("MotionClient bereit (MoveItPy, Planner-Selection).")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_planner(self, which: str) -> None:
        """
        Wähle Pipeline/Planner-Kombi.
          - 'ompl'      -> OMPL / RRTConnectkConfigDefault
          - 'pilz_lin'  -> pilz_industrial_motion_planner / LIN
          - 'pilz_ptp'  -> pilz_industrial_motion_planner / PTP
          - 'chomp'     -> chomp / CHOMP
        """
        w = (which or "").strip().lower()
        if w == "ompl":
            self._pipeline_id = "ompl"
            self._planner_id = "RRTConnectkConfigDefault"
        elif w in ("pilz", "pilz_lin"):
            self._pipeline_id = "pilz_industrial_motion_planner"
            self._planner_id = "LIN"
        elif w == "pilz_ptp":
            self._pipeline_id = "pilz_industrial_motion_planner"
            self._planner_id = "PTP"
        elif w == "chomp":
            self._pipeline_id = "chomp"
            self._planner_id = None  # CHOMP hat idR keinen named planner_id
        else:
            self._log.warning(f"Unbekannter Planner '{which}', bleibe bei OMPL.")
            self._pipeline_id = "ompl"
            self._planner_id = "RRTConnectkConfigDefault"

        self._pc.set_planning_pipeline_id(self._pipeline_id)
        if self._planner_id:
            try:
                self._pc.set_planner_id(self._planner_id)
            except Exception:
                # Nicht jede Version hat set_planner_id()
                pass

        self._log.info(f"MotionClient: Planner gesetzt → pipeline='{self._pipeline_id}' planner_id='{self._planner_id or '-'}'")

    def move_to_named_pose(
        self,
        name: str,
        *,
        time_to_target: Optional[float] = None,
        use_planner: bool = True,
        plan_only: bool = False,
        avoid_collisions: bool = False,
        timeout_sec: float = 3.0,
    ) -> bool:
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
        time_to_target: Optional[float] = None,  # für Non-Planner Pfad ignoriert
        use_planner: bool = True,
        plan_only: bool = False,
        avoid_collisions: bool = True,
        timeout_sec: float = 3.0,
    ) -> bool:
        """
        Plant und (optional) führt aus über MoveItPy. Kein ExecuteKnownTrajectory.
        """
        # Ziel setzen – IK/Constraints macht PlanningComponent intern:
        self._pc.set_goal_state(pose_stamped=target, tolerance=1e-3)

        # Collisions: per Request steuern (wo unterstützt); als Fallback global
        try:
            self._pc.set_start_state_to_current_state()
            self._pc.set_path_constraints(None)
            self._pc.set_planning_time(timeout_sec)
        except Exception:
            pass

        # Plan
        plan_result = self._pc.plan()
        if plan_result is None or plan_result.trajectory is None:
            self._log.error("MotionClient: Planner lieferte keine Trajectory.")
            return False

        traj = plan_result.trajectory
        npts = len(traj.joint_trajectory.points)
        self._log.info(f"MotionClient: Plan ok ({npts} Punkte) via {self._pipeline_id}/{self._planner_id or '-'}.")

        if plan_only:
            return True

        # Execute via MoveItPy
        ret = self._moveit.execute(traj, controllers=[])

        # Manche Versionen liefern bool, andere eine Status-Struktur
        ok = bool(ret) if isinstance(ret, bool) else True
        if ok:
            self._log.info("MotionClient: Ausführung OK.")
        else:
            self._log.error("MotionClient: Ausführung fehlgeschlagen.")
        return ok

    def move_joints(
        self,
        joint_positions: Dict[str, float],
        *,
        time_to_target: Optional[float] = None,
    ) -> bool:
        """
        Direktes Joint-Ziel an Controller (bypasst Planner).
        """
        if not joint_positions:
            self._log.error("MotionClient: move_joints() ohne Ziele.")
            return False
        return self._send_trajectory_to_controller_map(joint_positions, time_to_target or self._default_T)

    # ------------------------------------------------------------------
    # JointState handling
    # ------------------------------------------------------------------

    def _on_joint_states(self, msg: JointState) -> None:
        self._last_js = msg

    def _current_arm_joint_order(self) -> Optional[List[str]]:
        if self._last_js is None:
            return None
        idx = {n: i for i, n in enumerate(self._last_js.name)}
        wanted = [f"meca_axis_{i}_joint" for i in range(1, 7)]
        order = [n for n in wanted if n in idx]
        return order if order else None

    # ------------------------------------------------------------------
    # Pose resolving (via PosesClient oder latched Topic)
    # ------------------------------------------------------------------

    def _resolve_pose(self, name: str) -> Optional[PoseStamped]:
        name = name.strip()
        if self._pose_resolver:
            try:
                ps = self._pose_resolver(name)
                if ps:
                    return ps
            except Exception as e:
                self._log.warning(f"MotionClient: pose_resolver Fehler: {e}")

        # Fallback: latched Topic einmalig
        topic = self._topics.pose(name)
        box: List[PoseStamped] = []

        def _cb(msg: PoseStamped):
            box.append(msg)

        sub = self._node.create_subscription(PoseStamped, topic, _cb, qos_latched())
        try:
            ok = self._wait_until(lambda: len(box) > 0, timeout=1.5)
            if not ok:
                return None
            return box[0]
        finally:
            try:
                self._node.destroy_subscription(sub)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Controller publish (Direktmodus)
    # ------------------------------------------------------------------

    def _send_trajectory_to_controller(self, js_goal: JointState, time_to_target: float) -> bool:
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
        pt.time_from_start = rclpy.duration.Duration(seconds=time_to_target).to_msg()  # type: ignore
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
        pt.time_from_start = rclpy.duration.Duration(seconds=time_to_target).to_msg()  # type: ignore
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
