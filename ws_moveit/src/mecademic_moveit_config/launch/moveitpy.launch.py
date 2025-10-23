#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
from pprint import pprint
import os


def generate_launch_description():
    def launch_setup(context):
        # Pfad zum Config-Package
        pkg_share = FindPackageShare("mecademic_moveit_config").find("mecademic_moveit_config")

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

        return [motion_manager]

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
