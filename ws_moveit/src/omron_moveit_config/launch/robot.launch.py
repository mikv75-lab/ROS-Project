# omron_moveit_config/launch/robot.launch.py
#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    # --- Mount-/TF-Argumente (alle Winkel in RAD!) ---
    args = [
        DeclareLaunchArgument("mount_parent",    default_value="world"),
        DeclareLaunchArgument("mount_child",     default_value="robot_mount"),
        DeclareLaunchArgument("mount_x",         default_value="0.0"),
        DeclareLaunchArgument("mount_y",         default_value="0.0"),
        DeclareLaunchArgument("mount_z",         default_value="0.0"),
        DeclareLaunchArgument("mount_roll_rad",  default_value="0.0"),
        DeclareLaunchArgument("mount_pitch_rad", default_value="0.0"),
        DeclareLaunchArgument("mount_yaw_rad",   default_value="0.0"),
    ]

    # --- Static TF world -> robot_mount ---
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_robot_mount",
        # x y z roll pitch yaw frame_id child_frame_id   (roll/pitch/yaw in RAD!)
        arguments=[
            LaunchConfiguration("mount_x"),
            LaunchConfiguration("mount_y"),
            LaunchConfiguration("mount_z"),
            LaunchConfiguration("mount_roll_rad"),
            LaunchConfiguration("mount_pitch_rad"),
            LaunchConfiguration("mount_yaw_rad"),
            LaunchConfiguration("mount_parent"),
            LaunchConfiguration("mount_child"),
        ],
        output="screen",
    )

    # --- MoveIt Demo-Launch (wie bisher) ---
    moveit_config = (
        MoveItConfigsBuilder("omron_viper_s650", package_name="omron_moveit_config")
        .to_moveit_configs()
    )
    demo_ld = generate_demo_launch(moveit_config)  # LaunchDescription

    # RICHTIG: jedes Action-Objekt einzeln hinzufügen
    for a in args:
        demo_ld.add_action(a)
    demo_ld.add_action(static_tf)

    return demo_ld
