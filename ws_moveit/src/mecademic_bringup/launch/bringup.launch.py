#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
)

def generate_launch_description():
    use_fake_hw_arg = DeclareLaunchArgument(
        "use_fake_hw", default_value="true", description="Use fake hardware"
    )

    def launch_setup(context):
        bringup_pkg = FindPackageShare("mecademic_bringup").perform(context)
        moveit_pkg = FindPackageShare("mecademic_moveit_config").perform(context)

        # --- Manager-Konfigurationen ---
        tool_yaml  = os.path.join(bringup_pkg, "config", "tools.yaml")
        scene_yaml = os.path.join(bringup_pkg, "config", "scene.yaml")
        poses_yaml = os.path.join(bringup_pkg, "config", "poses.yaml")

        # --- Haupt-Roboter (MoveIt, ros2_control, RViz, TF, etc.) ---
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit_pkg, "launch", "robot.launch.py")
            ),
            launch_arguments={
                "use_fake_hw": "true",
                "rviz_config": "moveit.rviz",
            }.items(),
        )

        # --- Zusatz-Manager ---
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
            output="screen",
        )

        return [GroupAction([
            robot_launch,
            tool_manager,
            scene_manager,
            poses_manager,
            spray_path_manager,
        ])]

    return LaunchDescription([
        use_fake_hw_arg,
        OpaqueFunction(function=launch_setup),
    ])
