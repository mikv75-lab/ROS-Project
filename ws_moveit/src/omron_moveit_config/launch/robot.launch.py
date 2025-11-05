#!/usr/bin/env python3
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # Mount-/TF-Argumente: Längen in METER, Orientierung als QUATERNION
    args = [
        DeclareLaunchArgument("mount_parent",  default_value="world"),
        DeclareLaunchArgument("mount_child",   default_value="robot_mount"),
        DeclareLaunchArgument("mount_x",       default_value="0.0"),
        DeclareLaunchArgument("mount_y",       default_value="0.0"),
        DeclareLaunchArgument("mount_z",       default_value="0.0"),
        DeclareLaunchArgument("mount_qx",      default_value="0.0"),
        DeclareLaunchArgument("mount_qy",      default_value="0.0"),
        DeclareLaunchArgument("mount_qz",      default_value="0.0"),
        DeclareLaunchArgument("mount_qw",      default_value="1.0"),
        DeclareLaunchArgument("sim",           default_value="true"),
    ]

    # Static TF world -> robot_mount (Quaternion-Flags, robust)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_robot_mount",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("mount_x"),
            "--y", LaunchConfiguration("mount_y"),
            "--z", LaunchConfiguration("mount_z"),
            "--qx", LaunchConfiguration("mount_qx"),
            "--qy", LaunchConfiguration("mount_qy"),
            "--qz", LaunchConfiguration("mount_qz"),
            "--qw", LaunchConfiguration("mount_qw"),
            "--frame-id", LaunchConfiguration("mount_parent"),
            "--child-frame-id", LaunchConfiguration("mount_child"),
        ],
    )

    # MoveIt Demo-Launch
    moveit_config = (
        MoveItConfigsBuilder("omron_viper_s650", package_name="omron_moveit_config")
        .to_moveit_configs()
    )
    demo_ld = generate_demo_launch(moveit_config)

    for a in args:
        demo_ld.add_action(a)
    demo_ld.add_action(static_tf)

    return demo_ld
