#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    ld = LaunchDescription()

    # ---------------- Args ----------------
    ld.add_action(DeclareLaunchArgument("sim", default_value="true"))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))

    # WICHTIG: SRDF-Virtual-Joint-TFs standardmäßig AUS (sonst doppelte TFs)
    ld.add_action(DeclareBooleanLaunchArg("use_srdf_virtual_joint_tfs", default_value=False))

    # Mount-Pose (kommt bei dir aus bringup.robot.yaml)
    for name, default in [
        ("mount_parent", "world"),
        ("mount_child", "robot_mount"),
        ("mount_x", "0.0"),
        ("mount_y", "0.0"),
        ("mount_z", "0.0"),
        ("mount_qx", "0.0"),
        ("mount_qy", "0.0"),
        ("mount_qz", "0.0"),
        ("mount_qw", "1.0"),
    ]:
        ld.add_action(DeclareLaunchArgument(name, default_value=default))

    # ---------------- MoveIt Config ----------------
    # Falls dein Paketname anders ist, hier anpassen:
    pkg_name = "omron_moveit_config"
    moveit_config = (
        MoveItConfigsBuilder(robot_name="omron_viper_s650", package_name=pkg_name)
        .to_moveit_configs()
    )

    # ---------------- Static TF: world -> robot_mount ----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_robot_mount",
        output="screen",
        arguments=[
            "--x",  LaunchConfiguration("mount_x"),
            "--y",  LaunchConfiguration("mount_y"),
            "--z",  LaunchConfiguration("mount_z"),
            "--qx", LaunchConfiguration("mount_qx"),
            "--qy", LaunchConfiguration("mount_qy"),
            "--qz", LaunchConfiguration("mount_qz"),
            "--qw", LaunchConfiguration("mount_qw"),
            "--frame-id",       LaunchConfiguration("mount_parent"),  # z.B. "world"
            "--child-frame-id", LaunchConfiguration("mount_child"),   # z.B. "robot_mount"
        ],
    )
    ld.add_action(static_tf)

    # ---------------- RSP (robot_state_publisher) ----------------
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"publish_frequency": 30.0},
        ],
        additional_env={"RMW_FASTRTPS_USE_SHM": "0"},
    )
    ld.add_action(rsp_node)

    # ---------------- Optional: SRDF-Virtual-Joint-TFs ----------------
    static_vj_launch = Path(moveit_config.package_path) / "launch" / "static_virtual_joint_tfs.launch.py"
    if static_vj_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(static_vj_launch)),
                condition=IfCondition(LaunchConfiguration("use_srdf_virtual_joint_tfs")),
            )
        )

    # ---------------- Move Group ----------------
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    move_group_params = [moveit_config.to_dict(), move_group_configuration]

    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(Path(moveit_config.package_path) / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        additional_env={
            "DISPLAY": os.environ.get("DISPLAY", ""),
            "RMW_FASTRTPS_USE_SHM": "0",
        },
    )

    # ---------------- RViz (optional) ----------------
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(moveit_config.package_path) / "launch" / "moveit_rviz.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # ---------------- Warehouse DB (optional) ----------------
    warehouse_launch = Path(moveit_config.package_path) / "launch" / "warehouse_db.launch.py"
    if warehouse_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(warehouse_launch)),
                condition=IfCondition(LaunchConfiguration("db")),
            )
        )

    # ---------------- ros2_control + Spawner ----------------
    # Node zuerst...
    ros2_control = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[ str(Path(moveit_config.package_path) / "config" / "ros2_controllers.yaml") ],
    remappings=[("/controller_manager/robot_description", "/robot_description")],
    output="screen",
    additional_env={"RMW_FASTRTPS_USE_SHM": "0"},
    )
    ld.add_action(ros2_control)

    # ...dann Spawner mit Delay (damit Services sicher da sind)
    spawners = GroupAction(
        actions=[
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_state_broadcaster"],
                        output="screen",
                        additional_env={"RMW_FASTRTPS_USE_SHM": "0"},
                    )
                ],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["omron_arm_group_controller"],
                        output="screen",
                        additional_env={"RMW_FASTRTPS_USE_SHM": "0"},
                    )
                ],
            ),
        ]
    )
    ld.add_action(spawners)

    return ld
