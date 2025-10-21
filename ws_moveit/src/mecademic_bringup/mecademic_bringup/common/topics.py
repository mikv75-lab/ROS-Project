from __future__ import annotations
from dataclasses import dataclass
from typing import Optional

# ------------------------------------------------------------
# Helpers
# ------------------------------------------------------------
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
    return node.resolve_topic_name(name)

def resolve_service(node, name: str) -> str:
    return node.resolve_service_name(name)

# ------------------------------------------------------------
# Topics model
# ------------------------------------------------------------
@dataclass(frozen=True)
class Topics:
    base_ns: Optional[str] = None
    controller: str = "meca_arm_group_controller"
    servo_ns: str = "moveit_servo"

    # -------- Tool --------
    @property
    def tool_set(self) -> str: return _join(self.base_ns, "meca", "tool", "set")
    @property
    def tool_current(self) -> str: return _join(self.base_ns, "meca", "tool", "current")
    @property
    def tool_republish(self) -> str: return _join(self.base_ns, "meca", "tool", "republish")

    # -------- Posen --------
    @property
    def poses_republish(self) -> str: return _join(self.base_ns, "meca", "poses", "republish")
    @property
    def poses_set_from_tcp(self) -> str: return _join(self.base_ns, "meca", "poses", "set_from_tcp")

    def pose(self, name: str) -> str: return _join(self.base_ns, "meca", "poses", name)
    def pose_set(self, name: str) -> str: return _join(self.base_ns, "meca", "poses", "set", name)

    # -------- Substrate / Mount / Env --------
    @property
    def substrate_load(self) -> str: return _join(self.base_ns, "meca", "substrate", "load")
    @property
    def substrate_remove(self) -> str: return _join(self.base_ns, "meca", "substrate", "remove")
    @property
    def substrate_current(self) -> str: return _join(self.base_ns, "meca", "substrate", "current")

    @property
    def mount_load(self) -> str: return _join(self.base_ns, "meca", "mount", "load")
    @property
    def mount_current(self) -> str: return _join(self.base_ns, "meca", "mount", "current")

    @property
    def environment_load(self) -> str: return _join(self.base_ns, "meca", "environment", "load")
    @property
    def environment_current(self) -> str: return _join(self.base_ns, "meca", "environment", "current")

    # --- Spray Path ---
    @property
    def spray_path_set(self) -> str: return _join(self.base_ns, "meca", "spray_path", "set")
    @property
    def spray_path_clear(self) -> str: return _join(self.base_ns, "meca", "spray_path", "clear")
    @property
    def spray_path_current(self) -> str: return _join(self.base_ns, "meca", "spray_path", "current")

    # --- Visualization ---
    @property
    def recipe_markers(self) -> str: return _join(self.base_ns, "meca", "recipe", "markers")

    # --- Controller & States ---
    def joint_trajectory(self, controller: Optional[str] = None) -> str:
        ctrl = controller or self.controller
        return _join(self.base_ns, ctrl, "joint_trajectory")

    def fjt_action(self, controller: Optional[str] = None) -> str:
        ctrl = controller or self.controller
        return _join(self.base_ns, ctrl, "follow_joint_trajectory")

    @property
    def joint_states(self) -> str: return _join(self.base_ns, "joint_states")

    # --- MoveIt Services ---
    @property
    def compute_ik(self) -> str: return _join(self.base_ns, "compute_ik")
    @property
    def plan_kinematic_path(self) -> str: return _join(self.base_ns, "plan_kinematic_path")
    @property
    def execute_kinematic_path(self) -> str: return _join(self.base_ns, "execute_kinematic_path")
    @property
    def apply_planning_scene(self) -> str: return _join(self.base_ns, "apply_planning_scene")
    @property
    def get_planning_scene(self) -> str: return _join(self.base_ns, "get_planning_scene")

    # --- MoveIt Servo ---
    @property
    def servo_twist(self) -> str: return _join(self.base_ns, self.servo_ns, "delta_twist_cmds")
    @property
    def servo_joint(self) -> str: return _join(self.base_ns, self.servo_ns, "delta_joint_cmds")
    @property
    def servo_status(self) -> str: return _join(self.base_ns, self.servo_ns, "status")

    # --- Collision / Planning Scene ---
    @property
    def collision_object(self) -> str:
        return _join(self.base_ns, "attached_collision_object")

    @property
    def spray_path_markers(self) -> str:
        return _join(self.base_ns, "meca", "spray_path", "markers")

    # --- Factories ---
    @classmethod
    def for_node(cls, node, *, base_ns: Optional[str] = None,
                 controller: Optional[str] = None, servo_ns: Optional[str] = None) -> Topics:
        return cls(
            base_ns=base_ns,
            controller=controller or cls.controller,  # type: ignore[attr-defined]
            servo_ns=servo_ns or cls.servo_ns,        # type: ignore[attr-defined]
        )

# Default ohne Namespace
TOPICS = Topics()

__all__ = ["Topics", "TOPICS", "resolve_topic", "resolve_service"]
