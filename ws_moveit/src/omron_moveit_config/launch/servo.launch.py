#!/usr/bin/env python3
# omron_moveit_config/launch/servo.launch.py

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    cfg_pkg = get_package_share_directory("omron_moveit_config")

    # Wrapper-URDF mit ros2_control
    urdf_xacro = os.path.join(cfg_pkg, "config", "omron_viper_s650.urdf.xacro")
    rviz_cfg = os.path.join(cfg_pkg, "config", "moveit.rviz")
    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")

    # MoveIt-Konfiguration
    moveit_config = (
        MoveItConfigsBuilder("omron_viper_s650", package_name="omron_moveit_config")
        .robot_description(file_path=urdf_xacro)
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
    
    # Static TF: world -> robot_mount (Omron-Mount liegt aktuell im Weltursprung)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_omron_mount",
        arguments=[
            "--x",  "0.0",
            "--y",  "0.0",
            "--z",  "0.0",
            "--qx", "0.0",
            "--qy", "0.0",
            "--qz", "0.0",
            "--qw", "1.0",
            "--frame-id",      "world",
            "--child-frame-id","robot_mount",
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
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # JointState-Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # JointTrajectoryController für MoveIt & Servo
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omron_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
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
            {"use_sim_time": False},
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
                    "use_servo_services": True,
                    "check_collisions": False,

                    "publish_joint_positions": True,
                    "publish_joint_velocities": False,
                    "publish_joint_accelerations": False,

                    # <<< HIER WICHTIG >>>
                    "use_singularity_avoidance": True,
                    "lower_singularity_threshold": 15.0,
                    "hard_stop_singularity_threshold": 30.0,
                }
            },
        ],
        arguments=["--ros-args", "--disable-external-lib-logs"],
    )

    # --- RViz2 mit MoveIt-Config ---
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
        ],
        arguments=["-d", rviz_cfg],
    )

    return LaunchDescription(
        [   
            static_tf,
            rt_env,
            robot_state_publisher,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            servo_node,
            rviz_node,
        ]
    )
