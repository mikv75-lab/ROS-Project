#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
import os
from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
    PARAM_SPRAY_PATH_CONFIG,
)

from launch.actions.set_environment_variable import SetEnvironmentVariable

SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp"),

def generate_launch_description():

    def launch_setup(context):
        bringup_pkg = FindPackageShare("mecademic_bringup").perform(context)

        # --- YAML Pfade ---
        tool_yaml = PathJoinSubstitution([bringup_pkg, "config", "tools.yaml"])
        scene_yaml = PathJoinSubstitution([bringup_pkg, "config", "scene.yaml"])
        poses_yaml = PathJoinSubstitution([bringup_pkg, "config", "poses.yaml"])
        spray_path_yaml = PathJoinSubstitution([bringup_pkg, "config", "spray_paths.yaml"])

        # --- MoveIt + ros2_control starten ---
        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("mecademic_moveit_config"),
                    "launch",
                    "robot.launch.py",
                ])
            )
        )

        # --- Manager Nodes ---
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

        # --- MoveIt Grundkonfiguration ---
        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .to_moveit_configs()
        )
        cfg = moveit_config.to_dict()

        # --- Motion Manager Node ---
        motion_manager = Node(
            package="mecademic_bringup",
            executable="motion_manager",
            name="motion_manager",
            output="screen",
            parameters=[cfg],
        )

        # --- Reihenfolge + Delays ---
        scene_after_tool = RegisterEventHandler(
            OnProcessStart(
                target_action=tool_manager,
                on_start=[TimerAction(period=3.0, actions=[scene_manager])],
            )
        )

        poses_after_scene = RegisterEventHandler(
            OnProcessStart(
                target_action=scene_manager,
                on_start=[TimerAction(period=3.0, actions=[poses_manager])],
            )
        )

        spray_after_poses = RegisterEventHandler(
            OnProcessStart(
                target_action=poses_manager,
                on_start=[TimerAction(period=5.0, actions=[spray_path_manager])],
            )
        )

        # --- Topic-Watcher: wartet auf Scene ---
        topic_watcher = ExecuteProcess(
            cmd=["ros2", "topic", "echo", "/meca/scene/current", "--once"],
            output="screen",
            shell=False,
            additional_env={"RCUTILS_LOGGING_BUFFERED_STREAM": "1"},
            on_exit=[TimerAction(period=3.0, actions=[motion_manager])],
        )

        return [
            robot_launch,
            tool_manager,
            scene_after_tool,
            poses_after_scene,
            spray_after_poses,
            topic_watcher,
        ]

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
