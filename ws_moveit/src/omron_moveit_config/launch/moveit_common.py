#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_common.py

Zentraler Helper zum Laden der MoveIt-Konfiguration für den Omron Viper S650.

create_omron_moveit_config()  → Basis-MoveIt-Config (für move_group/RViz/MoveItPy).

Die Pipelines kommen aus:
  - config/*_planning.yaml         (Plugins, Request-Adapter, usw.)
  - config/moveit_cpp.yaml         (MoveItCpp-spezifische Optionen)

Es gibt EINE zentrale Liste PIPELINES, die überall verwendet wird.
"""

from __future__ import annotations

from moveit_configs_utils import MoveItConfigsBuilder

# Eine Quelle der Wahrheit für die Pipelines
PIPELINES = ["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"]


def create_omron_moveit_config():
    """
    Erzeugt eine MoveIt-Konfiguration für den Omron Viper S650.

    Pipelines:
        - ompl                         (Default)
        - pilz_industrial_motion_planner
        - chomp
        - stomp
    """
    builder = (
        MoveItConfigsBuilder(
            robot_name="omron_viper_s650",
            package_name="omron_moveit_config",
        )
        # explizit deine Dateien (relativ zum Paket-Pfad)
        .robot_description("config/omron_viper_s650.urdf.xacro")
        .robot_description_semantic("config/omron_viper_s650.srdf")
        .robot_description_kinematics("config/kinematics.yaml")
        .joint_limits("config/joint_limits.yaml")
        .trajectory_execution("config/moveit_controllers.yaml")
        # PlanningSceneMonitor-Optionen (für move_group / RViz)
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=False,
            publish_robot_description_semantic=False,
        )
        # Globale Planning-Pipelines (Plugins + Default)
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=PIPELINES,
            load_all=False,
        )
        # NEU: MoveItCpp-spezifische Konfiguration aus config/moveit_cpp.yaml
        .moveit_cpp("config/moveit_cpp.yaml")
    )

    return builder.to_moveit_configs()
