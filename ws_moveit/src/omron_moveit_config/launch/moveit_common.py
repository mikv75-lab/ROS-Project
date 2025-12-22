#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
moveit_common.py

Zentraler Helper zum Laden der MoveIt-Konfiguration für den Omron Viper S650.

create_omron_moveit_config()  → Basis-MoveIt-Config (für move_group/RViz/MoveItPy).

Die Pipelines kommen aus:
  - config/*_planning.yaml                       (Plugins, Request/Response-Adapter, Planner-Configs)
  - config/moveit_cpp.yaml                       (MoveItCpp-spezifische Optionen)
  - config/pilz_cartesian_limits.yaml            (Pilz Cartesian Limits) ✅
"""

from __future__ import annotations

from moveit_configs_utils import MoveItConfigsBuilder

# Single source of truth
PIPELINES = ["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"]


def create_omron_moveit_config():
    """
    Erzeugt eine MoveIt-Konfiguration für den Omron Viper S650.
    """
    return (
        MoveItConfigsBuilder(
            robot_name="omron_viper_s650",
            package_name="omron_moveit_config",
        )
        # URDF/SRDF/etc
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
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        # Pipelines + Default
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=PIPELINES,
            load_all=False,
        )
        # ✅ Pilz Limits explizit laden (safer/lesbarer)
        .pilz_cartesian_limits("config/pilz_cartesian_limits.yaml")
        # MoveItCpp-spezifische Konfig
        .moveit_cpp("config/moveit_cpp.yaml")
        .to_moveit_configs()
    )
