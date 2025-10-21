#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from pprint import pprint
import yaml
import os

from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
    PARAM_SPRAY_PATH_CONFIG,
)


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

        # --- MoveIt Grundkonfiguration (URDF, SRDF, Kinematics etc.) ---
        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .to_moveit_configs()
        )

        # Dict aus Builder erzeugen
        cfg = moveit_config.to_dict()

        # --- Planungspipelines manuell hinzufügen (Workaround für Builder-Bug) ---
        cfg["planning_pipelines"] = {
            "pipeline_names": ["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
            "default_planning_pipeline": "ompl",
            "ompl": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "ompl_planning",
            },
            "pilz_industrial_motion_planner": {
                "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "pilz_industrial_motion_planner_planning",
            },
            "chomp": {
                "planning_plugin": "chomp_interface/CHOMPPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "chomp_planning",
            },
            "stomp": {
                "planning_plugin": "stomp_moveit/StompPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "stomp_planning",
            },
        }

        # --- Zusätzliche Struktur für MoveItCPP (Sicherheitsnetz) ---
        cfg["moveit_cpp"] = {
            "planning_pipelines": {
                "pipeline_names": ["ompl"],
                "default_planning_pipeline": "ompl",
            }
        }

        # --- Debug-Ausgabe ---
        print("\n=== MoveIt Config Dump ===")
        pprint(cfg, width=120)
        debug_path = "/tmp/moveit_config_effective.yaml"
        with open(debug_path, "w") as f:
            yaml.dump(cfg, f, sort_keys=False, width=120)
        print(f"📄 Effektive MoveIt-Parameter gespeichert unter: {debug_path}\n")

        # --- Motion Manager Node ---
        motion_manager = Node(
            package="mecademic_bringup",
            executable="motion_manager",
            name="motion_manager",
            output="screen",
            parameters=[cfg],
        )

        # --- Hauptgruppe (ohne MotionManager) ---
        main_group = GroupAction([
            robot_launch,
            tool_manager,
            scene_manager,
            poses_manager,
            spray_path_manager,
        ])

        # --- MotionManager mit Verzögerung ---
        delayed_motion_manager = TimerAction(
            period=10.0,  # Sek. Verzögerung nach Start aller anderen Nodes
            actions=[motion_manager],
        )

        return [main_group, delayed_motion_manager]

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
