#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

# Launch-Ordner auf sys.path setzen, damit moveit_common importierbar ist
_current_dir = Path(__file__).resolve().parent
if str(_current_dir) not in sys.path:
    sys.path.insert(0, str(_current_dir))

from moveit_common import create_omron_moveit_config  # noqa: E402


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")
    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")

    # ✅ nur noch EIN RViz-File
    rviz_cfg = os.path.join(cfg_pkg, "config", "shadow_moveit.rviz")

    # ----------------- Launch-Argumente -----------------
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="shadow",
        description="ROS namespace",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz",
    )

    # Mount TF args
    mount_parent_arg = DeclareLaunchArgument("mount_parent", default_value="world")
    mount_child_arg = DeclareLaunchArgument("mount_child", default_value="robot_mount")

    mount_x_arg = DeclareLaunchArgument("mount_x", default_value="0.0")
    mount_y_arg = DeclareLaunchArgument("mount_y", default_value="0.0")
    mount_z_arg = DeclareLaunchArgument("mount_z", default_value="0.0")
    mount_qx_arg = DeclareLaunchArgument("mount_qx", default_value="0.0")
    mount_qy_arg = DeclareLaunchArgument("mount_qy", default_value="0.0")
    mount_qz_arg = DeclareLaunchArgument("mount_qz", default_value="0.0")
    mount_qw_arg = DeclareLaunchArgument("mount_qw", default_value="1.0")

    # ----------------- Launch Config -----------------
    namespace = LaunchConfiguration("namespace")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz = LaunchConfiguration("rviz")

    moveit_config = create_omron_moveit_config()

    # ----------------- Static TF -----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_mount",
        namespace=namespace,
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

    # ----------------- robot_state_publisher -----------------
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ----------------- ros2_control -----------------
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=namespace,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},
        ],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_jsb",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", ["/", namespace, "/controller_manager"],
        ],
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm",
        namespace=namespace,
        arguments=[
            "omron_arm_controller",
            "--controller-manager", ["/", namespace, "/controller_manager"],
        ],
    )

    # ✅ move_group IMMER starten (sonst keine PlanningScene/CollisionObjects)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace=namespace,
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    # ✅ RViz optional per launch arg
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=namespace,
        arguments=["-d", rviz_cfg],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        rviz_arg,

        mount_parent_arg,
        mount_child_arg,
        mount_x_arg,
        mount_y_arg,
        mount_z_arg,
        mount_qx_arg,
        mount_qy_arg,
        mount_qz_arg,
        mount_qw_arg,

        static_tf,
        robot_state_pub,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,

        move_group_node,
        rviz_node,
    ])
