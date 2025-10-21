#!/usr/bin/env python3
import os
import yaml
from math import radians
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    TimerAction,
    GroupAction,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from tf_transformations import quaternion_from_euler


def generate_launch_description():
    def launch_setup(context):
        # --- Package Path ---
        cfg_pkg = FindPackageShare("mecademic_moveit_config").perform(context)

        # --- Static TF (world → meca_mount) aus YAML ---
        mount_yaml = os.path.join(cfg_pkg, "config", "meca_mount.yaml")
        with open(mount_yaml, "r") as f:
            mount = yaml.safe_load(f)["meca_mount"]

        xyz = mount.get("xyz", [0.0, 0.0, 0.0])
        rpy_deg = mount.get("rpy_deg", [0.0, 0.0, 0.0])
        world_frame = mount.get("parent", "world")

        qx, qy, qz, qw = quaternion_from_euler(
            radians(rpy_deg[0]), radians(rpy_deg[1]), radians(rpy_deg[2])
        )

        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_world_to_meca_mount",
            arguments=[
                "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                "--qx", str(qx), "--qy", str(qy), "--qz", str(qz), "--qw", str(qw),
                "--frame-id", world_frame,
                "--child-frame-id", "meca_mount",
            ],
            output="screen",
        )

        # --- MoveIt Config ---
        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .to_moveit_configs()
        )

        # --- ros2_control Node ---
        ros2_control = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                os.path.join(cfg_pkg, "config", "ros2_controllers.yaml"),
            ],
            output="screen",
        )

        # --- Robot State Publisher ---
        robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        )

        # --- Controller Spawner ---
        joint_state_broadcaster = TimerAction(
            period=2.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["joint_state_broadcaster"],
                    output="screen",
                )
            ],
        )

        arm_controller = TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=["meca_arm_group_controller"],
                    output="screen",
                )
            ],
        )

        # --- MoveGroup Node ---
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            arguments=["--ros-args", "--log-level", "info"],
        )

        # --- Servo Node ---
        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            name="moveit_servo",
            output="screen",
            parameters=[
                {
                    "moveit_servo.move_group_name": "meca_arm_group",
                    "moveit_servo.planning_frame": "world",
                    "moveit_servo.command_type": "velocity",
                    "moveit_servo.publish_period": 0.01,
                    "moveit_servo.cartesian_command_in_topic": "/moveit_servo/delta_twist_cmds",
                    "moveit_servo.command_out_topic": "/meca_arm_group_controller/joint_trajectory",
                    "moveit_servo.command_out_type": "trajectory_msgs/JointTrajectory",
                    "moveit_servo.scale.linear": 0.3,
                    "moveit_servo.scale.rotational": 0.8,
                    "moveit_servo.scale.joint": 0.6,
                    "moveit_servo.allow_missing_joints": True,
                    "moveit_servo.use_smoothing": False,
                    "moveit_servo.check_collisions": False,
                },
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )

        # --- RViz Node (immer moveit.rviz aus config/) ---
        rviz_config_path = PathJoinSubstitution(
            [FindPackageShare("mecademic_moveit_config"), "config", "moveit.rviz"]
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )

        return [
            GroupAction([
                static_tf,
                robot_state_pub,
                ros2_control,
                joint_state_broadcaster,
                arm_controller,
                move_group_node,
                servo_node,
                rviz_node,
            ])
        ]

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
