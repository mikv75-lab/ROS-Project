#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
omron_moveit_config/launch/omron_shadow.launch.py

Servo als Node starten.
=> Node:     /<ns>/servo_node
=> Service:  /<ns>/servo_node/switch_command_type

Namespace-Regeln:
- Stack-Nodes laufen unter namespace=<ns> (default: shadow)
- /tf und /tf_static bleiben global
- Servo Topics/Services RELATIV (ohne führenden "/"), damit /<ns>/...
- FIX: ServoNode defaultet oft auf /joint_states (global). Wir remappen das sauber
       auf 'joint_states' => /<ns>/joint_states
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

_current_dir = Path(__file__).resolve().parent
if str(_current_dir) not in sys.path:
    sys.path.insert(0, str(_current_dir))

from moveit_common import create_omron_moveit_config  # noqa: E402


def generate_launch_description() -> LaunchDescription:
    cfg_pkg = get_package_share_directory("omron_moveit_config")
    ros2_controllers_path = os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")
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
        description="RViz config file",
    )

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
    rviz_cfg = LaunchConfiguration("rviz_config")
    use_sim_time = ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)

    moveit_config = create_omron_moveit_config()

    # controller_manager fully qualified: /<ns>/controller_manager
    cm_fqn = [TextSubstitution(text="/"), ns, TextSubstitution(text="/controller_manager")]

    # Global TF bleibt global
    tf_remaps_global = [("tf", "/tf"), ("tf_static", "/tf_static")]

    # WICHTIG: ServoNode (und teils MoveIt) nimmt gerne "/joint_states" absolut.
    # Wir remappen absolute -> relative, damit es im Namespace landet.
    joint_states_remap = [("/joint_states", "joint_states")]

    # ----------------- Nodes -----------------
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="tf_world_to_mount",
        namespace=ns,
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
        remappings=tf_remaps_global,
        output="screen",
        emulate_tty=True,
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=ns,
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 100.0},
        ],
        remappings=tf_remaps_global + joint_states_remap,
        output="screen",
        emulate_tty=True,
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        namespace=ns,
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
            {"use_sim_time": use_sim_time},
        ],
        remappings=tf_remaps_global,
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
            "--controller-manager-timeout", "300",
            # optional: wenn du auch hier sicher global/relativ trennen willst:
            # "--controller-ros-args", "-r", "/joint_states:=joint_states",
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
            "--controller-manager-timeout", "300",
        ],
        output="screen",
        emulate_tty=True,
    )

    # ----------------- Servo Parameters (inline) -----------------
    servo_params_inline = {
        "use_sim_time": use_sim_time,

        # Wichtig für online_signal_smoothing::AccelerationLimitedPlugin
        "update_period": 0.01,

        "moveit_servo": {
            "command_in_type": "speed_units",
            "move_group_name": "omron_arm_group",
            "planning_frame": "robot_mount",
            "ee_frame_name": "tcp",

            # FIX: ServoNode soll im Namespace auf joint_states lauschen
            "joint_topic": "joint_states",

            # Input topics (relativ => /<ns>/servo/...)
            "cartesian_command_in_topic": "servo/delta_twist_cmds",
            "joint_command_in_topic": "servo/delta_joint_cmds",
            "pose_command_in_topic": "servo/delta_pose_cmds",

            "use_servo_services": True,
            "check_collisions": False,

            "publish_period": 0.01,
            "incoming_command_timeout": 0.2,
        },
    }

    servo_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        servo_params_inline,
    ]

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        namespace=ns,  # -> /shadow/servo_node
        parameters=servo_parameters,
        remappings=tf_remaps_global + joint_states_remap,
        output="screen",
        emulate_tty=True,
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        namespace=ns,
        arguments=["-d", rviz_cfg],
        parameters=[moveit_config.to_dict(), {"use_sim_time": use_sim_time}],
        remappings=tf_remaps_global + joint_states_remap,
        output="screen",
        emulate_tty=True,
    )

    # ----------------- Startup-Order -----------------
    delayed_stack = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(period=2.0, actions=[jsb_spawner]),
                TimerAction(period=3.0, actions=[arm_spawner]),
                TimerAction(period=4.0, actions=[servo_node]),
                TimerAction(period=6.0, actions=[rviz_node]),
            ],
        )
    )

    return LaunchDescription(
        [
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
            static_tf,
            robot_state_pub,
            ros2_control_node,
            delayed_stack,
        ]
    )
