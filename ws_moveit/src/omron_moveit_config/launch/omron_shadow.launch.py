#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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

    # Controller manager fully-qualified name (string!)
    cm_name = ["/", namespace, "/controller_manager"]

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
        output="screen",
        emulate_tty=True,
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
            {"publish_frequency": 100.0},  # Sync mit controller_manager update_rate
        ],
        # ✅ Nur Errors loggen - filtert "Moved backwards in time" Warning
        arguments=['--ros-args', '--log-level', 'ERROR'],
        output="screen",
        emulate_tty=True,
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
        output="screen",
        emulate_tty=True,
    )

    # ----------------- Spawner (mit Timing-Fix) -----------------
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_jsb",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", cm_name,
        ],
        output="screen",
        emulate_tty=True,
    )

    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_arm",
        namespace=namespace,
        arguments=[
            "omron_arm_controller",
            "--controller-manager", cm_name,
        ],
        output="screen",
        emulate_tty=True,
    )

    # OPTIONAL: MoveGroup (wenn du den Service /get_planning_scene willst)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace=namespace,
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
        emulate_tty=True,
    )

    # RViz im Namespace starten, damit relative Topics funktionieren
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=namespace,
        arguments=["-d", rviz_cfg],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        condition=IfCondition(rviz),
        output="screen",
        emulate_tty=True,
    )

    # ✅ Timing-Strategie:
    # - Warte bis ros2_control_node gestartet ist
    # - dann: JSB nach 2s
    # - dann: Arm controller nach 3s
    # - optional: move_group nach 5s
    # - optional: rviz nach 6s
    delayed_stack = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=2.0, actions=[jsb_spawner]),
                TimerAction(period=3.0, actions=[arm_spawner]),
                # Wenn du MoveIt Scene + /get_planning_scene willst:
                # TimerAction(period=5.0, actions=[move_group_node]),
                TimerAction(period=6.0, actions=[rviz_node]),
            ],
        )
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

        # ✅ spawner + rviz kommen NACH ros2_control_node start
        delayed_stack,
    ])