#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
    PARAM_SPRAY_PATH_CONFIG,
)

def generate_launch_description():
    def launch_setup(context):
        # --- Package Paths ---
        bringup_pkg = FindPackageShare("mecademic_bringup").perform(context)
        moveit_pkg = FindPackageShare("mecademic_moveit_config").perform(context)

        # --- YAML Configs ---
        tool_yaml = os.path.join(bringup_pkg, "config", "tools.yaml")
        scene_yaml = os.path.join(bringup_pkg, "config", "scene.yaml")
        poses_yaml = os.path.join(bringup_pkg, "config", "poses.yaml")
        spray_path_yaml = os.path.join(bringup_pkg, "config", "spray_paths.yaml")

        # --- Robot (MoveIt + ros2_control) ---
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_pkg, "launch", "robot.launch.py")
            )
        )

        # --- Core Managers ---
        tool_manager = Node(
            package="mecademic_bringup",
            executable="tool_manager",
            name="tool_manager",
            parameters=[{PARAM_TOOL_CONFIG: tool_yaml}],
            output="screen",
        )

        scene_manager = Node(
            package="mecademic_bringup",
            executable="scene_manager",
            name="scene_manager",
            parameters=[{PARAM_SCENE_CONFIG: scene_yaml}],
            output="screen",
        )

        poses_manager = Node(
            package="mecademic_bringup",
            executable="poses_manager",
            name="poses_manager",
            parameters=[{PARAM_POSES_CONFIG: poses_yaml}],
            output="screen",
        )

        spray_path_manager = Node(
            package="mecademic_bringup",
            executable="spray_path_manager",
            name="spray_path_manager",
            parameters=[{PARAM_SPRAY_PATH_CONFIG: spray_path_yaml}],
            output="screen",
        )

        # --- Motion + Visualization ---
        motion_manager = Node(
            package="mecademic_bringup",
            executable="motion_manager.py",
            name="motion_manager",
            output="screen",
        )

        trajectory_marker = Node(
            package="mecademic_bringup",
            executable="trajectory_marker_node.py",
            name="trajectory_marker_node",
            output="screen",
        )

        # --- Grouped Startup Sequence ---
        main_group = GroupAction([
            robot_launch,
            tool_manager,
            scene_manager,
            poses_manager,
            spray_path_manager,

            # Motion components
            TimerAction(period=3.0, actions=[motion_manager]),
            TimerAction(period=4.0, actions=[trajectory_marker]),
        ])

        return [main_group]

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
