#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# omron_moveit_config/launch/robot_sim.launch.py

import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory

# Launch-Ordner auf sys.path setzen
_current_dir = Path(__file__).resolve().parent
if str(_current_dir) not in sys.path:
    sys.path.insert(0, str(_current_dir))

from moveit_common import create_omron_moveit_config  # noqa: E402


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")

    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")
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

    mount_x_arg = DeclareLaunchArgument("mount_x", default_value="0.0", description="X-Offset")
    mount_y_arg = DeclareLaunchArgument("mount_y", default_value="0.0", description="Y-Offset")
    mount_z_arg = DeclareLaunchArgument("mount_z", default_value="0.0", description="Z-Offset")
    mount_qx_arg = DeclareLaunchArgument("mount_qx", default_value="0.0", description="Quat-X")
    mount_qy_arg = DeclareLaunchArgument("mount_qy", default_value="0.0", description="Quat-Y")
    mount_qz_arg = DeclareLaunchArgument("mount_qz", default_value="0.0", description="Quat-Z")
    mount_qw_arg = DeclareLaunchArgument("mount_qw", default_value="1.0", description="Quat-W")

    # Hinweis:
    # - true nur sinnvoll, wenn /clock von Gazebo/Bag/etc. kommt.
    # - ohne /clock besser false.
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Nur true, wenn /clock existiert (Gazebo/rosbag --clock).",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="RViz mit starten?",
    )

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="shadow",
        description="ROS-Namespace für diese Instanz",
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
    use_sim_time_bool = ParameterValue(use_sim_time, value_type=bool)

    rviz = LaunchConfiguration("rviz")
    namespace = LaunchConfiguration("namespace")

    # ----------------- MoveIt-Konfiguration -----------------
    moveit_config = create_omron_moveit_config()

    # ----------------- static TF: world → robot_mount -----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=[namespace, "_tf_world_to_mount"],
        namespace=namespace,
        arguments=[
            "--x", mount_x,
            "--y", mount_y,
            "--z", mount_z,
            "--qx", mount_qx,
            "--qy", mount_qy,
            "--qz", mount_qz,
            "--qw", mount_qw,
            "--frame-id", mount_parent,
            "--child-frame-id", mount_child,
        ],
        output="screen",
    )

    # 1) robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=[namespace, "_robot_state_publisher"],
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time_bool},
        ],
    )

    # 2) ros2_control_node (Controller Manager + Hardware)
    # WICHTIG: Name "controller_manager", damit Services /<ns>/controller_manager/... heißen
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=namespace,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": use_sim_time_bool},
        ],
        output="screen",
    )

    # 3) JointStateBroadcaster spawner
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", ["/", namespace, "/controller_manager"],
            "--param-file", ros2_controllers_path,
        ],
        output="screen",
    )

    # 4) Arm-Controller spawner
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_omron_arm_controller",
        namespace=namespace,
        arguments=[
            "omron_arm_controller",
            "--controller-manager", ["/", namespace, "/controller_manager"],
            "--param-file", ros2_controllers_path,
        ],
        output="screen",
    )

    # 5) MoveIt: move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name=[namespace, "_move_group"],
        namespace=namespace,
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time_bool},
        ],
    )

    # 6) RViz (optional) -> global (kein namespace)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_cfg],
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time_bool},
        ],
        output="screen",
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
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
        namespace_arg,
        static_tf,
        robot_state_pub,
        ros2_control_node,
        jsb_spawner,
        arm_spawner,
        move_group_node,
        rviz_node,
    ])
