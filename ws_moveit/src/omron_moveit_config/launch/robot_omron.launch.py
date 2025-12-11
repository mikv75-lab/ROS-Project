#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# omron_moveit_config/launch/robot_omron.launch.py
#
# Real-Omron-Setup (Backend: echter Roboter über TCP/ACE)
#
# Startet:
#   - static TF world -> robot_mount
#   - robot_state_publisher (URDF; /joint_states muss von deinem Robot-Node kommen)
#   - OmronTcpBridge (TCP/IP zu ACE / Emulator / realer Omron)
#   - move_group (MoveIt Planung)
#   - RViz (MoveIt Motion Planning Plugin)
#
# API:
#   * mount_parent, mount_child
#   * mount_{x,y,z}, mount_{qx,qy,qz,qw}
#   * use_sim_time
#   * rviz
#   * omron_host, omron_port, omron_timeout
#   * namespace

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# 🔹 Launch-Ordner auf sys.path setzen, damit moveit_common importierbar ist
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

    # Pose des robot_mount relativ zu mount_parent
    mount_x_arg = DeclareLaunchArgument(
        "mount_x", default_value="0.0", description="X-Offset von mount_parent"
    )
    mount_y_arg = DeclareLaunchArgument(
        "mount_y", default_value="0.0", description="Y-Offset von mount_parent"
    )
    mount_z_arg = DeclareLaunchArgument(
        "mount_z", default_value="0.0", description="Z-Offset von mount_parent"
    )
    mount_qx_arg = DeclareLaunchArgument(
        "mount_qx", default_value="0.0", description="Quat-X von mount_parent"
    )
    mount_qy_arg = DeclareLaunchArgument(
        "mount_qy", default_value="0.0", description="Quat-Y von mount_parent"
    )
    mount_qz_arg = DeclareLaunchArgument(
        "mount_qz", default_value="0.0", description="Quat-Z von mount_parent"
    )
    mount_qw_arg = DeclareLaunchArgument(
        "mount_qw", default_value="1.0", description="Quat-W von mount_parent"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Sim-Time verwenden (normalerweise false beim echten Roboter)",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="RViz mit starten?",
    )

    # 🔌 TCP-Parameter für OmronTcpBridge
    omron_host_arg = DeclareLaunchArgument(
        "omron_host",
        default_value="host.docker.internal",
        description="Hostname/IP des ACE/Omron-TCP-Servers (Emulator/real)",
    )
    omron_port_arg = DeclareLaunchArgument(
        "omron_port",
        default_value="5000",
        description="TCP-Port des ACE/Omron-TCP-Servers",
    )
    omron_timeout_arg = DeclareLaunchArgument(
        "omron_timeout",
        default_value="3.0",
        description="TCP-Timeout in Sekunden",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="live",
        description="ROS-Namespace für diese Instanz (z.B. 'live')",
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

    omron_host = LaunchConfiguration("omron_host")
    omron_port = LaunchConfiguration("omron_port")
    omron_timeout = LaunchConfiguration("omron_timeout")

    namespace = LaunchConfiguration("namespace")

    # ----------------- MoveIt-Konfiguration -----------------
    moveit_config = create_omron_moveit_config()

    # ----------------- static TF: world → robot_mount -----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_omron_mount",
        namespace=namespace,
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

    # 1) robot_state_publisher
    # /joint_states muss von deinem eigenen Omron-Node kommen, der den echten Zustand via TCP/ACE holt.
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # 2) OmronTcpBridge (TCP/IP zu ACE / Emulator / Real)
    omron_tcp_bridge = Node(
        package="spraycoater_nodes_py",
        executable="omron_tcp_bridge",
        name="omron_tcp_bridge",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "host": omron_host,
                "port": omron_port,
                "timeout": omron_timeout,
            }
        ],
    )

    # 3) MoveIt: move_group (Planning + PlanningSceneService)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # 4) RViz (MoveIt Motion Planning Plugin)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
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
            omron_host_arg,
            omron_port_arg,
            omron_timeout_arg,
            namespace_arg,
            static_tf,
            robot_state_pub,
            omron_tcp_bridge,
            move_group_node,
            rviz_node,
        ]
    )
