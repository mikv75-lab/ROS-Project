from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, Sequence

@dataclass
class AppParams:
    # MoveIt / Robot
    group_name: str = "meca_arm_group"
    controller_name: str = "meca_arm_group_controller"
    ee_link_candidates: Sequence[Optional[str]] = field(
        default_factory=lambda: ("tcp", "meca_axis_6_link", "tool0", "flange", None)
    )

    # Planning
    planning_pipeline: str = "ompl"                     # "ompl" | "pilz" | "chomp"
    planner_id: str = "RRTConnectkConfigDefault"
    vel_scaling: float = 0.4
    acc_scaling: float = 0.4

    # Servo / Jogging
    servo_ns: str = "moveit_servo"
    jog_linear_scale: float = 0.5
    jog_angular_scale: float = 0.5

    # Frames
    pose_parent_frame: str = "meca_base"


def load_params(node) -> AppParams:
    """
    Lädt Parameter (mit Defaults), falls im Node gesetzt/geremapped.
    Nutze in __init__: self.params = load_params(self)
    """
    p = AppParams()

    def _declare(name: str, value):
        try:
            node.declare_parameter(name, value)
        except Exception:
            pass

    _declare("group_name", p.group_name)
    _declare("controller_name", p.controller_name)
    _declare("ee_link_candidates", list(p.ee_link_candidates))
    _declare("planning_pipeline", p.planning_pipeline)
    _declare("planner_id", p.planner_id)
    _declare("vel_scaling", p.vel_scaling)
    _declare("acc_scaling", p.acc_scaling)
    _declare("servo_ns", p.servo_ns)
    _declare("jog_linear_scale", p.jog_linear_scale)
    _declare("jog_angular_scale", p.jog_angular_scale)
    _declare("pose_parent_frame", p.pose_parent_frame)

    # Werte übernehmen
    p.group_name = node.get_parameter("group_name").get_parameter_value().string_value or p.group_name
    p.controller_name = node.get_parameter("controller_name").get_parameter_value().string_value or p.controller_name

    ee = node.get_parameter("ee_link_candidates").get_parameter_value().string_array_value
    if ee:
        p.ee_link_candidates = tuple(s if s else None for s in ee)

    p.planning_pipeline = node.get_parameter("planning_pipeline").get_parameter_value().string_value or p.planning_pipeline
    p.planner_id = node.get_parameter("planner_id").get_parameter_value().string_value or p.planner_id

    vs = node.get_parameter("vel_scaling").get_parameter_value().double_value
    if vs > 0: p.vel_scaling = vs
    ac = node.get_parameter("acc_scaling").get_parameter_value().double_value
    if ac > 0: p.acc_scaling = ac

    p.servo_ns = node.get_parameter("servo_ns").get_parameter_value().string_value or p.servo_ns

    jl = node.get_parameter("jog_linear_scale").get_parameter_value().double_value
    if jl > 0: p.jog_linear_scale = jl
    ja = node.get_parameter("jog_angular_scale").get_parameter_value().double_value
    if ja > 0: p.jog_angular_scale = ja

    p.pose_parent_frame = node.get_parameter("pose_parent_frame").get_parameter_value().string_value or p.pose_parent_frame

    return p

# ------------------------------------------------------------
# YAML Parameter Keys (used in Launch & Nodes)
# ------------------------------------------------------------
PARAM_SCENE_CONFIG = "scene_yaml"
PARAM_POSES_CONFIG = "poses_yaml"
PARAM_TOOL_CONFIG = "tools_yaml"
PARAM_SPRAY_PATH_CONFIG = "spray_paths_yaml"

__all__ = ["AppParams", "load_params"]
