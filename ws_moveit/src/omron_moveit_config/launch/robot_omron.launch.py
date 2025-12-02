#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# omron_moveit_config/launch/robot_omron.launch.py

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# --- moveit_common importierbar machen ---
_current_dir = Path(__file__).resolve().parent
if str(_current_dir) not in sys.path:
    sys.path.insert(0, str(_current_dir))

from moveit_common import create_omron_moveit_config  # noqa: E402


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")

    rviz_cfg = os.path.join(cfg_pkg, "config", "moveit.rviz")

    # ----------------- Launch-Argumente -----------------
    mount_parent_arg = DeclareLaunchArgument(
        "mount_parent",
        default_value="world",
        description="Parent Frame für robot_mount",
    )
    mount_child_arg = DeclareLaunchArgument(
        "mount_child",
        default_value="robot_mount",
        description="Child Frame für robot_mount",
    )

    mount_x_arg = DeclareLaunchArgument("mount_x", default_value="0.0")
    mount_y_arg = DeclareLaunchArgument("mount_y", default_value="0.0")
    mount_z_arg = DeclareLaunchArgument("mount_z", default_value="0.0")
    mount_qx_arg = DeclareLaunchArgument("mount_qx", default_value="0.0")
    mount_qy_arg = DeclareLaunchArgument("mount_qy", default_value="0.0")
    mount_qz_arg = DeclareLaunchArgument("mount_qz", default_value="0.0")
    mount_qw_arg = DeclareLaunchArgument("mount_qw", default_value="1.0")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Sim-Time verwenden (für RViz etc.)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="RViz mit starten?",
    )

    # ----------------- LaunchConfigurations -----------------
    mount_parent = LaunchConfiguration("mount_parent")
    mount_child = LaunchConfiguration("mount_child")

    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_qx = LaunchConfiguration("mount_qx")
    mount_qy = LaunchConfiguration("mount_qy")
    mount_qz = LaunchConfiguration("mount_qz")
    mount_qw = LaunchConfiguration("mount_qw")

    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")

    # ----------------- MoveIt-Konfiguration -----------------
    moveit_config = create_omron_moveit_config()

    # ----------------- static TF: world → robot_mount -----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_omron_mount",
        arguments=[
            "--x",
            mount_x,
            "--y",
            mount_y,
            "--z",
            mount_z,
            "--qx",
            mount_qx,
            "--qy",
            mount_qy,
            "--qz",
            mount_qz,
            "--qw",
            mount_qw,
            "--frame-id",
            mount_parent,
            "--child-frame-id",
            mount_child,
        ],
        output="screen",
    )

    # Nur robot_state_publisher (Echt-Roboter kommt von außen)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription(
        [
            mount_parent_arg,
            mount_child_arg,
            mount_x_arg,
            mount_y_arg,
            mount_z_arg,
            mount_qx_arg,
            mount_qy_arg,
            mount_qz_arg,
            mount_qw_arg,
            use_sim_time_arg,
            rviz_arg,
            static_tf,
            robot_state_pub,
            rviz_node,
        ]
    )
