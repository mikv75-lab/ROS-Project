#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_common.py

Zentraler Helper zum Laden der MoveIt-Konfiguration für den Omron Viper S650.

Geladen wird:

- robot_description              → config/omron_viper_s650.urdf.xacro
- robot_description_semantic     → config/omron_viper_s650.srdf
- robot_description_kinematics   → config/kinematics.yaml
- joint_limits                   → config/joint_limits.yaml
- trajectory_execution           → config/moveit_controllers.yaml
- planning_pipelines             → OMPL (default), STOMP, CHOMP, Pilz
- planning_scene_monitor         → einfache Flags

NICHT geladen werden:

- sensors_3d.yaml
- moveit_cpp.yaml
"""

from moveit_configs_utils import MoveItConfigsBuilder


def create_omron_moveit_config():
    """
    Erzeugt eine MoveIt-Konfiguration für den Omron Viper S650.

    Pipelines:
        - ompl                         (Default)
        - stomp
        - chomp
        - pilz_industrial_motion_planner
    """
    builder = (
        MoveItConfigsBuilder(
            robot_name="omron_viper_s650",
            package_name="omron_moveit_config",
        )
        # explizit deine Dateien nehmen (relativ zum Paket-Pfad)
        .robot_description("config/omron_viper_s650.urdf.xacro")
        .robot_description_semantic("config/omron_viper_s650.srdf")
        .robot_description_kinematics("config/kinematics.yaml")
        .joint_limits("config/joint_limits.yaml")
        # MoveIt-Controller-Konfiguration
        .trajectory_execution("config/moveit_controllers.yaml")
        # PlanningSceneMonitor – nur Flags setzen, keine extra Datei
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=[
                "ompl",
                "stomp",
                "chomp",
                "pilz_industrial_motion_planner",
            ],
            # load_all ist egal, wenn pipelines explizit gesetzt sind,
            # aber wir halten es explizit bei False:
            load_all=False,
        )
    )

    return builder.to_moveit_configs()
