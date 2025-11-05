#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MoveItPy motion planning tutorial ported to Omron Viper setup (YAML-free).

- Startet MoveItPy ohne launch_params_filepaths (nur node_name)
- Verwendet Planungsgruppe 'omron_arm_group'
- Zeigt Zieldefinition via RobotState, PoseStamped und JointConstraints
- Versucht Multi-Pipeline-Planung; bei fehlenden Parametern sauberer Fallback
"""

import sys
import time
import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.core.robot_state import RobotState
from moveit.core.kinematic_constraints import construct_joint_constraint
from moveit.planning import MoveItPy, MultiPipelinePlanRequestParameters

GROUP_NAME = "omron_arm_group"  # deine Gruppe (muss zur SRDF passen)
WORLD_FRAME = "world"
EE_LINK = "tool_mount"          # TCP/Endeffektor-Link (muss zur URDF/SRDF passen)


def _strip_launch_yaml_args():
    """Entfernt ggf. von launch injizierte YAML-Args, um rcl-Parsingfehler zu vermeiden."""
    sys.argv[:] = [
        a for a in sys.argv
        if not (a.endswith(".yaml") or a.startswith("/tmp/launch_params_") or a == "--params-file")
    ]


def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time: float = 0.0,
):
    """Helper: planen und (bei Erfolg) ausführen."""
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
    elif single_plan_parameters is not None:
        result = planning_component.plan(single_plan_parameters=single_plan_parameters)
    else:
        result = planning_component.plan()

    if result and getattr(result, "trajectory", None):
        logger.info("Executing plan")
        robot.execute(result.trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)


def main():
    _strip_launch_yaml_args()
    rclpy.init()
    logger = get_logger("moveit_py.omron_tutorial_yaml_free")

    # --- MoveItPy OHNE YAML ---
    try:
        robot = MoveItPy(node_name="moveit_py")
    except Exception as e:
        logger.error(f"Failed to init MoveItPy: {e}")
        return

    # kurze Wartezeit, bis move_group/PlanningScene komplett steht
    time.sleep(1.0)

    arm = robot.get_planning_component(GROUP_NAME)
    logger.info("✅ MoveItPy instance created (YAML-free)")

    # ---------------- Plan 1: RobotState (mittlere Pose) ----------
    robot_model = robot.get_robot_model()
    rs = RobotState(robot_model)
    rs.joint_positions = {
        "joint_1": 0.0,
        "joint_2": -1.0,
        "joint_3": 1.5,
        "joint_4": 0.0,
        "joint_5": 0.0,
        "joint_6": 0.0,
    }
    arm.set_start_state_to_current_state()
    logger.info("Plan 1: goal via RobotState (mid joints)")
    arm.set_goal_state(robot_state=rs)
    plan_and_execute(robot, arm, logger, sleep_time=1.5)

    # ---------------- Plan 2: Random RobotState -------------------
    rs_random = RobotState(robot_model)
    rs_random.set_to_random_positions()
    arm.set_start_state_to_current_state()
    logger.info("Plan 2: goal via randomized RobotState")
    arm.set_goal_state(robot_state=rs_random)
    plan_and_execute(robot, arm, logger, sleep_time=1.5)

    # ---------------- Plan 3: PoseStamped -------------------------
    arm.set_start_state_to_current_state()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = WORLD_FRAME
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = 0.30
    goal_pose.pose.position.y = 0.00
    goal_pose.pose.position.z = 0.20
    arm.set_goal_state(pose_stamped_msg=goal_pose, pose_link=EE_LINK)
    logger.info("Plan 3: goal via PoseStamped (world->tool_mount)")
    plan_and_execute(robot, arm, logger, sleep_time=1.5)

    # ---------------- Plan 4: Constraints (JointConstraint) -------
    arm.set_start_state_to_current_state()
    rs_constraints = RobotState(robot_model)
    rs_constraints.joint_positions = {
        "joint_1": -0.5,
        "joint_2": -1.2,
        "joint_3": 1.2,
        "joint_4": 0.5,
        "joint_5": 0.0,
        "joint_6": 0.0,
    }
    jc = construct_joint_constraint(
        robot_state=rs_constraints,
        joint_model_group=robot_model.get_joint_model_group(GROUP_NAME),
    )
    logger.info("Plan 4: goal via JointConstraint")
    arm.set_goal_state(motion_plan_constraints=[jc])
    plan_and_execute(robot, arm, logger, sleep_time=1.5)

    # ---------------- Plan 5: Multi-Pipeline (robust) -------------
    arm.set_start_state_to_current_state()
    multi_goal = PoseStamped()
    multi_goal.header.frame_id = WORLD_FRAME
    multi_goal.pose.orientation.w = 1.0
    multi_goal.pose.position.x = 0.25
    multi_goal.pose.position.y = 0.10
    multi_goal.pose.position.z = 0.18
    arm.set_goal_state(pose_stamped_msg=multi_goal, pose_link=EE_LINK)

    try:
        # Diese Namespaces müssen im Param-Server existieren (z. B. von move_group bereitgestellt).
        multi_params = MultiPipelinePlanRequestParameters(
            robot, ["ompl_rrtc", "pilz_lin", "chomp_planner"]
        )
        logger.info("Plan 5: multi-pipeline planning (ompl_rrtc, pilz_lin, chomp_planner)")
        plan_and_execute(robot, arm, logger, multi_plan_parameters=multi_params, sleep_time=1.5)
    except Exception as e:
        logger.warning(f"Multi-pipeline params unavailable ({e}) – fallback to default planning.")
        plan_and_execute(robot, arm, logger, sleep_time=1.5)

    time.sleep(0.3)


if __name__ == "__main__":
    main()
