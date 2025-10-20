#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import yaml

from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
    PARAM_USE_FAKE_HW,
)

def generate_launch_description():
    use_fake_hw_arg = DeclareLaunchArgument("use_fake_hw", default_value="true")

    def launch_setup(context):
        use_fake_hw = LaunchConfiguration("use_fake_hw").perform(context)

        desc_pkg = FindPackageShare("mecademic_description").perform(context)
        cfg_pkg = FindPackageShare("mecademic_moveit_config").perform(context)
        bringup_pkg = FindPackageShare("mecademic_bringup").perform(context)

        
        
        tool_yaml  = os.path.join(bringup_pkg, "config", "tools.yaml")
        scene_yaml = os.path.join(bringup_pkg, "config", "scene.yaml")
        poses_yaml = os.path.join(bringup_pkg, "config", "poses.yaml")
        # ✅ Static TF aus YAML (alte Lösung)
        mount_yaml = os.path.join(bringup_pkg, "config", "meca_mount.yaml")
        with open(mount_yaml, "r") as f:
            mount = yaml.safe_load(f)["meca_mount"]

        world_frame = mount["parent"]
        xyz = mount["xyz"]
        rpy_deg = mount["rpy_deg"]

        def deg2rad(deg):
            from math import radians
            return [radians(x) for x in deg]

        from tf_transformations import quaternion_from_euler
        qx, qy, qz, qw = quaternion_from_euler(*deg2rad(rpy_deg))

        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_world_to_meca_mount",
            arguments=[
                "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                "--qx", str(qx), "--qy", str(qy), "--qz", str(qz), "--qw", str(qw),
                "--frame-id", world_frame,
                "--child-frame-id", "meca_mount"
            ],
            output="screen"
        )

        # --- MoveIt Konfig
        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .robot_description(
                file_path=os.path.join(desc_pkg, "urdf", "meca_500_r3.urdf.xacro"),
                mappings={"use_fake_hw": use_fake_hw},
            )
            .robot_description_semantic(
                file_path=os.path.join(cfg_pkg, "config", "meca_500_r3.srdf")
            )
            .robot_description_kinematics(
                file_path=os.path.join(cfg_pkg, "config", "kinematics.yaml")
            )
            .joint_limits(file_path=os.path.join(cfg_pkg, "config", "joint_limits.yaml"))
            .trajectory_execution(
                file_path=os.path.join(cfg_pkg, "config", "moveit_controllers.yaml")
            )
            .planning_pipelines(
                default_planning_pipeline="ompl",
                pipelines=["ompl", "pilz_industrial_motion_planner"],
            )
            .pilz_cartesian_limits(
                file_path=os.path.join(cfg_pkg, "config", "pilz_cartesian_limits.yaml")
            )
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
                publish_geometry_updates=False,
                publish_state_updates=False,
                publish_transforms_updates=False,
            )
            .to_moveit_configs()
        )

        # --- Basis Nodes
        robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        )

        ros2_control = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                os.path.join(cfg_pkg, "config", "ros2_controllers.yaml"),
            ],
            output="screen",
        )

        spawner_js = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen",
        )

        spawner_arm = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["meca_arm_group_controller"],
            output="screen",
        )

        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"moveit_controller_manager": "moveit_ros_control_interface/Ros2ControlManager"},
            ],
        )

        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            name="moveit_servo",
            output="screen",
            parameters=[
                {
                    "moveit_servo.move_group_name": "meca_arm_group",
                    "moveit_servo.planning_frame": "world",
                },
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="moveit_rviz",
            output="screen",
            arguments=["-d", os.path.join(bringup_pkg, "config", "bringup.rviz")],
        )

        tool_manager = Node(
            package="mecademic_bringup",
            executable="tool_manager",
            name="tool_manager",
            parameters=[{PARAM_TOOL_CONFIG: tool_yaml}],
            output="screen"
        )

        scene_manager = Node(
            package="mecademic_bringup",
            executable="scene_manager",
            name="scene_manager",
            parameters=[{PARAM_SCENE_CONFIG: scene_yaml}],
            output="screen"
        )

        poses_manager = Node(
            package="mecademic_bringup",
            executable="poses_manager",
            name="poses_manager",
            parameters=[{PARAM_POSES_CONFIG: poses_yaml}],
            output="screen"
        )
        spray_path_manager = Node(package="mecademic_bringup", executable="spray_path_manager")

        actions = [
            static_tf,  # ✅ Wichtig: TF zuerst!
            robot_state_pub,
            ros2_control,
            spawner_js,
            RegisterEventHandler(OnProcessExit(target_action=spawner_js, on_exit=[spawner_arm])),
            RegisterEventHandler(OnProcessExit(target_action=spawner_arm, on_exit=[move_group, servo_node, rviz_node])),
            RegisterEventHandler(
                OnProcessStart(
                    target_action=move_group,
                    on_start=[tool_manager, scene_manager, poses_manager, spray_path_manager],
                )
            )        
        ]

        return [GroupAction(actions)]

    return LaunchDescription([use_fake_hw_arg, OpaqueFunction(function=launch_setup)])
