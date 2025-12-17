#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, GroupAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

# moveit_common.py liegt in diesem launch/ Ordner
_current_dir = Path(__file__).resolve().parent
if str(_current_dir) not in sys.path:
    sys.path.insert(0, str(_current_dir))

from moveit_common import create_omron_moveit_config  # noqa: E402


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")
    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")

    # ✅ Default: alte shadow_moveit.rviz
    default_rviz_cfg = os.path.join(cfg_pkg, "config", "shadow_moveit.rviz")

    # ----------------- Launch-Argumente -----------------
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="shadow",
        description="Instanz-Namespace (z.B. shadow | live | inst1)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_cfg,
        description="RViz config file (default: shadow_moveit.rviz)",
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
    ns = LaunchConfiguration("namespace")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)
    rviz_cfg = LaunchConfiguration("rviz_config")

    moveit_config = create_omron_moveit_config()

    # ✅ Spawner MUSS den vollqualifizierten controller_manager bekommen
    cm_fqn = ["/", ns, "/controller_manager"]

    # ----------------- Nodes (ohne namespace=... !) -----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_mount",
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
        output="screen",
        emulate_tty=True,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 100.0},
        ],
        output="screen",
        emulate_tty=True,
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        emulate_tty=True,
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_jsb",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", cm_fqn,
        ],
        output="screen",
        emulate_tty=True,
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm",
        arguments=[
            "omron_arm_controller",
            "--controller-manager", cm_fqn,
        ],
        output="screen",
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_cfg],  # ✅ shadow_moveit.rviz (default)
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )

    delayed_stack = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=2.0, actions=[jsb_spawner]),
                TimerAction(period=3.0, actions=[arm_spawner]),
                TimerAction(period=6.0, actions=[rviz_node]),
            ],
        )
    )

    # ----------------- Group: Namespace + TF Remap (relativ -> absolut) -----------------
    group = GroupAction(actions=[
        PushRosNamespace(ns),
        SetRemap(src="tf", dst="/tf"),
        SetRemap(src="tf_static", dst="/tf_static"),
        static_tf,
        robot_state_pub,
        ros2_control_node,
        delayed_stack,
    ])

    return LaunchDescription([
        namespace_arg,
        use_sim_time_arg,
        rviz_config_arg,

        mount_parent_arg,
        mount_child_arg,
        mount_x_arg,
        mount_y_arg,
        mount_z_arg,
        mount_qx_arg,
        mount_qy_arg,
        mount_qz_arg,
        mount_qw_arg,

        group,
    ])
