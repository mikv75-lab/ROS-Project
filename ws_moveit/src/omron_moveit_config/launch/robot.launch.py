#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# omron_moveit_config/launch/robot.launch.py

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")

    # Wrapper-URDF mit ros2_control
    urdf_xacro = os.path.join(cfg_pkg, "config", "omron_viper_s650.urdf.xacro")
    rviz_cfg = os.path.join(cfg_pkg, "config", "moveit.rviz")
    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")

    # ---------- Launch-Argumente (werden vom bringup gesetzt) ----------
    mount_parent_arg = DeclareLaunchArgument("mount_parent", default_value="world")
    mount_child_arg = DeclareLaunchArgument("mount_child", default_value="robot_mount")

    mount_x_arg = DeclareLaunchArgument("mount_x", default_value="0.0")
    mount_y_arg = DeclareLaunchArgument("mount_y", default_value="0.0")
    mount_z_arg = DeclareLaunchArgument("mount_z", default_value="0.0")
    mount_qx_arg = DeclareLaunchArgument("mount_qx", default_value="0.0")
    mount_qy_arg = DeclareLaunchArgument("mount_qy", default_value="0.0")
    mount_qz_arg = DeclareLaunchArgument("mount_qz", default_value="0.0")
    mount_qw_arg = DeclareLaunchArgument("mount_qw", default_value="1.0")

    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="Simulationsmodus (true|false) – aktuell nur für später vorgesehen",
    )

    mount_parent = LaunchConfiguration("mount_parent")
    mount_child = LaunchConfiguration("mount_child")
    mount_x = LaunchConfiguration("mount_x")
    mount_y = LaunchConfiguration("mount_y")
    mount_z = LaunchConfiguration("mount_z")
    mount_qx = LaunchConfiguration("mount_qx")
    mount_qy = LaunchConfiguration("mount_qy")
    mount_qz = LaunchConfiguration("mount_qz")
    mount_qw = LaunchConfiguration("mount_qw")
    sim = LaunchConfiguration("sim")

    # ---------- MoveIt-Konfiguration ----------
    moveit_config = (
        MoveItConfigsBuilder("omron_viper_s650", package_name="omron_moveit_config")
        .robot_description(
            file_path=urdf_xacro,
            mappings={"hardware_type": "FakeSystem"}  # später: FakeSystem/RealSystem umschalten
        )
        .robot_description_semantic(file_path="config/omron_viper_s650.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Logging leicht puffern
    rt_env = SetEnvironmentVariable(
        name="RCUTILS_LOGGING_BUFFERED_STREAM",
        value="1",
    )

    # ---------- Static TF: (mount_parent) -> (mount_child) ----------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_omron_mount",
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

    # --- robot_state_publisher: URDF + TF ---
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # --- ros2_control_node mit FakeSystem (aus URDF) ---
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",          # 🔑 explizit
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="screen",
    )

    # JointState-Broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # JointTrajectoryController für MoveIt & Servo
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omron_arm_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )

    # 🔁 Spawner erst starten, wenn controller_manager läuft
    load_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    load_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[arm_controller_spawner],
        )
    )

    # --- MoveIt Servo Node ---
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="omron_servo_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "moveit_servo": {
                    "command_in_type": "speed_units",
                    "move_group_name": "omron_arm_group",
                    "planning_frame": "world",

                    "cartesian_command_in_topic": "/servo/cartesian_mm",
                    "joint_command_in_topic": "/servo/joint_jog",
                    "pose_command_in_topic": "/servo/delta_pose_cmds",

                    "command_out_topic": "/omron_arm_controller/joint_trajectory",
                    "command_out_type": "trajectory_msgs/JointTrajectory",

                    "publish_period": 0.01,
                    "incoming_command_timeout": 0.2,

                    # Services anlassen – du nutzt /switch_command_type
                    "use_servo_services": True,
                    "check_collisions": False,

                    "publish_joint_positions": True,
                    "publish_joint_velocities": False,
                    "publish_joint_accelerations": False,

                    "use_singularity_avoidance": True,
                    "lower_singularity_threshold": 15.0,
                    "hard_stop_singularity_threshold": 30.0,
                }
            },
        ],
        arguments=["--ros-args", "--disable-external-lib-logs"],
    )

    # --- MoveIt move_group (Motion Planning) ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # --- RViz2 mit MoveIt-Config ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription(
        [
            rt_env,
            sim_arg,
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
            robot_state_publisher,
            ros2_control_node,
            load_joint_state_broadcaster,
            load_arm_controller,
            move_group_node,
            servo_node,
            rviz_node,
        ]
    )
