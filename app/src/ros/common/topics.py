# ros/common/topics.py
"""
Zentrale Topic-/Service-/Action-Namen (RELATIV, ohne führenden '/').
Best Practices:
- Namen relativ halten, damit Namespaces/Remaps greifen.
- Für den endgültigen Namen bei Bedarf node.resolve_topic_name(...) / resolve_service_name(...) nutzen.
- Controller- und Servo-Namespaces sind parametrierbar.

Typ-Hinweise (nur Orientierung):
- Topics: std_msgs/String, std_msgs/Empty, sensor_msgs/JointState,
          geometry_msgs/PoseStamped, geometry_msgs/PoseArray, geometry_msgs/TwistStamped,
          trajectory_msgs/JointTrajectory, control_msgs/JointJog,
          moveit_msgs/ServoStatus, visualization_msgs/MarkerArray
- Services: moveit_msgs/srv/GetPositionIK, GetMotionPlan, ExecuteKnownTrajectory,
            ApplyPlanningScene, GetPlanningScene
- Actions:  control_msgs/action/FollowJointTrajectory (Basisname, ohne /goal etc.)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

# ---------- helpers ----------

def _join(*parts: Optional[str]) -> str:
    """Robustes Joinen zu einem RELATIVEN ROS-Namen (kein führender '/')."""
    cleaned = []
    for p in parts:
        if not p:
            continue
        s = str(p).strip().strip("/")
        if s:
            cleaned.append(s)
    return "/".join(cleaned)

def resolve_topic(node, name: str) -> str:
    """Gibt den vollständig aufgelösten Topic-Namen (inkl. Namespace/Remaps) zurück."""
    return node.resolve_topic_name(name)

def resolve_service(node, name: str) -> str:
    """Gibt den vollständig aufgelösten Service-Namen (inkl. Namespace/Remaps) zurück."""
    return node.resolve_service_name(name)

# ---------- model ----------

@dataclass(frozen=True)
class Topics:
    """
    Erzeugt alle Namen relativ zu:
      - dem Node-Namespace (durch Launch/Remap vorgegeben) UND
      - optional einem zusätzlichen 'base_ns' (Subnamespace relativ zum Node),
      - plus konfigurierbarem Controller & Servo-Namespace.

    Hinweis:
    - 'controller' variiert je nach Controller-Setup (Default unten).
    - 'servo_ns' ist der Namespace des Servo-Servers (z. B. 'moveit_servo').
    """
    base_ns: Optional[str] = None
    controller: str = "meca_arm_group_controller"
    servo_ns: str = "moveit_servo"

    # -------- Tool --------
    @property
    def tool_set(self) -> str:                     # std_msgs/String (App -> ToolManager)
        return _join(self.base_ns, "meca", "tool", "set")

    @property
    def tool_current(self) -> str:                 # std_msgs/String (ToolManager -> App, latched)
        return _join(self.base_ns, "meca", "tool", "current")

    @property
    def tool_republish(self) -> str:               # std_msgs/Empty (App -> ToolManager)
        return _join(self.base_ns, "meca", "tool", "republish")

    # -------- Posen --------
    @property
    def poses_republish(self) -> str:              # std_msgs/Empty (App -> PosesManager)
        return _join(self.base_ns, "meca", "poses", "republish")

    @property
    def poses_set_from_tcp(self) -> str:           # std_msgs/String (App -> PosesManager)
        return _join(self.base_ns, "meca", "poses", "set_from_tcp")

    def pose(self, name: str) -> str:              # geometry_msgs/PoseStamped (latched)
        return _join(self.base_ns, "meca", "poses", name)

    def pose_set(self, name: str) -> str:          # optional: geometry_msgs/PoseStamped
        return _join(self.base_ns, "meca", "poses", "set", name)

    # -------- Substrate / Mount / Environment --------
    # Substrate
    @property
    def substrate_load(self) -> str:               # std_msgs/String (Pfad zur STL)
        return _join(self.base_ns, "meca", "substrate", "load")

    @property
    def substrate_remove(self) -> str:             # std_msgs/Empty
        return _join(self.base_ns, "meca", "substrate", "remove")

    @property
    def substrate_current(self) -> str:            # std_msgs/String (latched; aktuelles Substrat)
        return _join(self.base_ns, "meca", "substrate", "current")

    # Mount
    @property
    def mount_load(self) -> str:                   # std_msgs/String (Pfad zur STL)
        return _join(self.base_ns, "meca", "mount", "load")

    @property
    def mount_current(self) -> str:                # std_msgs/String (latched; aktueller Mount)
        return _join(self.base_ns, "meca", "mount", "current")

    # Environment
    @property
    def environment_load(self) -> str:             # std_msgs/String (Pfad zur STL)
        return _join(self.base_ns, "meca", "environment", "load")

    @property
    def environment_current(self) -> str:          # std_msgs/String (latched; aktuelles Env)
        return _join(self.base_ns, "meca", "environment", "current")

    # --- Alias für Rückwärtskompatibilität (ehemals "workspace_*") ---
    @property
    def workspace_load(self) -> str:               # std_msgs/String -> Alias auf substrate_load
        return self.substrate_load

    @property
    def workspace_remove(self) -> str:             # std_msgs/Empty  -> Alias auf substrate_remove
        return self.substrate_remove

    @property
    def workspace_current(self) -> str:            # std_msgs/String -> Alias auf substrate_current
        return self.substrate_current

    # -------- Spray-Pfad (Datenquelle, NICHT Visualisierung) --------
    @property
    def spray_path_set(self) -> str:               # geometry_msgs/PoseArray
        return _join(self.base_ns, "meca", "spray_path", "set")

    @property
    def spray_path_clear(self) -> str:             # std_msgs/Empty
        return _join(self.base_ns, "meca", "spray_path", "clear")

    @property
    def spray_path_current(self) -> str:           # optional: geometry_msgs/PoseArray (latched)
        return _join(self.base_ns, "meca", "spray_path", "current")

    # -------- Visualisierung (Scene/Recipe Preview) --------
    @property
    def recipe_markers(self) -> str:               # visualization_msgs/MarkerArray (latched)
        return _join(self.base_ns, "meca", "recipe", "markers")

    # -------- Motion / Controller --------
    def joint_trajectory(self, controller: Optional[str] = None) -> str:
        """trajectory_msgs/JointTrajectory (App -> Controller)"""
        ctrl = controller or self.controller
        return _join(self.base_ns, ctrl, "joint_trajectory")

    def fjt_action(self, controller: Optional[str] = None) -> str:
        """control_msgs/action/FollowJointTrajectory (Basisname, ohne /goal etc.)"""
        ctrl = controller or self.controller
        return _join(self.base_ns, ctrl, "follow_joint_trajectory")

    @property
    def joint_states(self) -> str:                 # sensor_msgs/JointState
        return _join(self.base_ns, "joint_states")

    # -------- IK / Planung / Scene (MoveIt) --------
    @property
    def compute_ik(self) -> str:                   # moveit_msgs/srv/GetPositionIK
        return _join(self.base_ns, "compute_ik")

    @property
    def plan_kinematic_path(self) -> str:          # moveit_msgs/srv/GetMotionPlan
        return _join(self.base_ns, "plan_kinematic_path")

    @property
    def execute_kinematic_path(self) -> str:       # moveit_msgs/srv/ExecuteKnownTrajectory
        return _join(self.base_ns, "execute_kinematic_path")

    @property
    def apply_planning_scene(self) -> str:         # moveit_msgs/srv/ApplyPlanningScene
        return _join(self.base_ns, "apply_planning_scene")

    @property
    def get_planning_scene(self) -> str:           # moveit_msgs/srv/GetPlanningScene
        return _join(self.base_ns, "get_planning_scene")

    # -------- MoveIt Servo (Jogging) --------
    @property
    def servo_twist(self) -> str:                  # geometry_msgs/TwistStamped
        return _join(self.base_ns, self.servo_ns, "delta_twist_cmds")

    @property
    def servo_joint(self) -> str:                  # control_msgs/JointJog
        return _join(self.base_ns, self.servo_ns, "delta_joint_cmds")

    @property
    def servo_status(self) -> str:                 # moveit_msgs/ServoStatus
        return _join(self.base_ns, self.servo_ns, "status")

    # -------- Legacy / Debug (nur lesen, falls benötigt) --------
    @property
    def rviz_marker(self) -> str:                  # visualization_msgs/Marker (legacy)
        return _join(self.base_ns, "visualization_marker")

    @property
    def collision_object(self) -> str:             # moveit_msgs/CollisionObject (legacy direct publish)
        return _join(self.base_ns, "collision_object")

    # ---------- factories ----------

    @classmethod
    def for_node(
        cls,
        node,
        *,
        base_ns: Optional[str] = None,
        controller: Optional[str] = None,
        servo_ns: Optional[str] = None,
    ) -> "Topics":
        """
        Factory für eine Topics-Instanz im Kontext eines Nodes.
        - 'base_ns' bleibt RELATIV (Sub-Namespace).
        - 'controller' und 'servo_ns' können hier bequem überschrieben werden.
        """
        return cls(
            base_ns=base_ns,
            controller=controller or cls.controller,   # type: ignore[attr-defined]
            servo_ns=servo_ns or cls.servo_ns,         # type: ignore[attr-defined]
        )

# Bequemer Default ohne zusätzliches base_ns
TOPICS = Topics()

__all__ = ["Topics", "TOPICS", "resolve_topic", "resolve_service"]
