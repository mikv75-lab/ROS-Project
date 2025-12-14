#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# omron_moveit_config/launch/robot.launch.py
#
# Gemeinsames Robot-Launchfile für Shadow (sim) und Live (real).
#
# role := shadow → ros2_control + FakeHardware
# role := live   → OmronTcpBridge (real robot)
#
# MoveGroup, TF, robot_state_publisher laufen in beiden Fällen identisch.

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# sys.path erweitern
_current_dir = Path(__file__).resolve().parent
if str(_current_dir) not in sys.path:
    sys.path.insert(0, str(_current_dir))

from moveit_common import create_omron_moveit_config


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")

    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")
    rviz_cfg = os.path.join(cfg_pkg, "config", "moveit.rviz")

    # ---------------------------------------------------------------------
    # Launch Arguments
    # ---------------------------------------------------------------------
    role_arg = DeclareLaunchArgument(
        "role",
        default_value="shadow",
        description="shadow = sim, live = real"
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="shadow",
        description="ROS Namespace"
    )

    # TF Pose
    pose_args = [
        DeclareLaunchArgument("mount_parent", default_value="world"),
        DeclareLaunchArgument("mount_child", default_value="robot_mount"),
        DeclareLaunchArgument("mount_x", default_value="0.0"),
        DeclareLaunchArgument("mount_y", default_value="0.0"),
        DeclareLaunchArgument("mount_z", default_value="0.0"),
        DeclareLaunchArgument("mount_qx", default_value="0.0"),
        DeclareLaunchArgument("mount_qy", default_value="0.0"),
        DeclareLaunchArgument("mount_qz", default_value="0.0"),
        DeclareLaunchArgument("mount_qw", default_value="1.0"),
    ]

    # 👉 Default jetzt true, damit Shadow ohne Args sim time nutzt
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Sim-Time verwenden (shadow=typisch true, live=false)"
    )

    rviz_arg = DeclareLaunchArgument("rviz", default_value="true")

    # Real TCP Bridge
    tcp_args = [
        DeclareLaunchArgument("omron_host", default_value="host.docker.internal"),
        DeclareLaunchArgument("omron_port", default_value="5000"),
        DeclareLaunchArgument("omron_timeout", default_value="3.0"),
    ]

    # ---------------------------------------------------------------------
    # Config Objekte
    # ---------------------------------------------------------------------
    role = LaunchConfiguration("role")
    namespace = LaunchConfiguration("namespace")
    rviz = LaunchConfiguration("rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    mount_parent = LaunchConfiguration("mount_parent")
    mount_child = LaunchConfiguration("mount_child")
    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_qx = LaunchConfiguration("mount_qx")
    mount_qy = LaunchConfiguration("mount_qy")
    mount_qz = LaunchConfiguration("mount_qz")
    mount_qw = LaunchConfiguration("mount_qw")

    omron_host = LaunchConfiguration("omron_host")
    omron_port = LaunchConfiguration("omron_port")
    omron_timeout = LaunchConfiguration("omron_timeout")

    moveit_config = create_omron_moveit_config()

    # ---------------------------------------------------------------------
    # TF
    # ---------------------------------------------------------------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace=namespace,
        name="tf_world_to_mount",
        arguments=[
            "--x", mount_x, "--y", mount_y, "--z", mount_z,
            "--qx", mount_qx, "--qy", mount_qy, "--qz", mount_qz, "--qw", mount_qw,
            "--frame-id", mount_parent,
            "--child-frame-id", mount_child,
        ],
    )

    # ---------------------------------------------------------------------
    # robot_state_publisher (immer)
    # ---------------------------------------------------------------------
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        parameters=[moveit_config.robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ---------------------------------------------------------------------
    # Shadow → ros2_control
    # ---------------------------------------------------------------------
    sim_backend = GroupAction(
        condition=IfCondition(PythonExpression(["'", role, "' == 'shadow'"])),
        actions=[
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                namespace=namespace,
                parameters=[
                    moveit_config.robot_description,
                    ros2_controllers_path,
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager", "controller_manager",
                    "--param-file", ros2_controllers_path,
                ],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace=namespace,
                arguments=[
                    "omron_arm_controller",
                    "--controller-manager", "controller_manager",
                    "--param-file", ros2_controllers_path,
                ],
            ),
        ],
    )

    # ---------------------------------------------------------------------
    # Live → TCP Bridge only
    # ---------------------------------------------------------------------
    real_backend = GroupAction(
        condition=IfCondition(PythonExpression(["'", role, "' == 'live'"])),
        actions=[
            Node(
                package="spraycoater_nodes_py",
                executable="omron_tcp_bridge",
                namespace=namespace,
                name="omron_tcp_bridge",
                parameters=[{
                    "host": omron_host,
                    "port": omron_port,
                    "timeout": omron_timeout
                }],
                output="screen",
            ),
        ],
    )

    # ---------------------------------------------------------------------
    # MoveGroup (immer)
    # ---------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=namespace,
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ---------------------------------------------------------------------
    # Optional RViz
    # ---------------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        arguments=["-d", rviz_cfg],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    # ---------------------------------------------------------------------
    return LaunchDescription(
        [role_arg, namespace_arg] +
        pose_args +
        [use_sim_time_arg, rviz_arg] +
        tcp_args + [
            static_tf,
            robot_state_pub,
            sim_backend,
            real_backend,
            move_group_node,
            rviz_node,
        ]
    )
