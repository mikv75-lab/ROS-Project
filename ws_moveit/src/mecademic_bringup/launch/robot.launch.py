#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    use_fake_hw_arg = DeclareLaunchArgument("use_fake_hw", default_value="true")

    def launch_setup(context):
        use_fake_hw = LaunchConfiguration("use_fake_hw").perform(context)

        desc_pkg = FindPackageShare("mecademic_description").perform(context)
        cfg_pkg = FindPackageShare("mecademic_moveit_config").perform(context)
        bringup_pkg = FindPackageShare("mecademic_bringup").perform(context)

        # ✅ Lockfile Cleanup FIX
        cleanup_lock = ExecuteProcess(
            cmd=["bash", "-c", "rm -f /tmp/ros2-control-controller-spawner.lock"],
            shell=True
        )

        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .robot_description(
                file_path=os.path.join(desc_pkg, "urdf", "meca_500_r3.urdf.xacro"),
                mappings={"use_fake_hw": use_fake_hw}
            )
            .robot_description_semantic(file_path=os.path.join(cfg_pkg, "config", "meca_500_r3.srdf"))
            .robot_description_kinematics(file_path=os.path.join(cfg_pkg, "config", "kinematics.yaml"))
            .joint_limits(file_path=os.path.join(cfg_pkg, "config", "joint_limits.yaml"))
            .trajectory_execution(file_path=os.path.join(cfg_pkg, "config", "moveit_controllers.yaml"))
            .planning_pipelines(
                default_planning_pipeline="ompl",
                pipelines=["ompl", "pilz_industrial_motion_planner"]
            )
            .pilz_cartesian_limits(file_path=os.path.join(cfg_pkg, "config", "pilz_cartesian_limits.yaml"))
            .planning_scene_monitor(
                publish_robot_description=True,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
                publish_geometry_updates=True,
                publish_state_updates=True,
                publish_transforms_updates=True
            )
            .to_moveit_configs()
        )

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
                os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")
            ],
            output="screen",
        )

        # ✅ FIX: Controller sequentiell starten
        joint_state_broadcaster = TimerAction(
            period=2.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster"],
                output="screen",
            )]
        )

        arm_controller = TimerAction(
            period=4.0,
            actions=[Node(
                package="controller_manager",
                executable="spawner",
                arguments=["meca_arm_group_controller"],
                output="screen",
            )]
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
                    "moveit_servo.command_type": "velocity",
                    "moveit_servo.publish_period": 0.01,
                    "moveit_servo.joint_command_in_topic": "/moveit_servo/delta_joint_cmds",
                    "moveit_servo.cartesian_command_in_topic": "/moveit_servo/delta_twist_cmds",
                    "moveit_servo.command_out_topic": "/meca_arm_group_controller/joint_trajectory",
                    "moveit_servo.command_out_type": "trajectory_msgs/JointTrajectory",
                    "moveit_servo.scale.linear": 0.3,
                    "moveit_servo.scale.rotational": 0.8,
                    "moveit_servo.scale.joint": 0.6,
                    "moveit_servo.lower_singularity_threshold": 0.001,
                    "moveit_servo.hard_stop_singularity_threshold": 0.01,
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

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="moveit_rviz",
            output="screen",
            arguments=["-d", os.path.join(bringup_pkg, "config", "bringup.rviz")],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        )

        return [GroupAction([
            cleanup_lock,  # ✅ Important
            robot_state_pub,
            ros2_control,
            joint_state_broadcaster,
            arm_controller,
            move_group,
            servo_node,
            rviz_node,
        ])]

    return LaunchDescription([
        use_fake_hw_arg,
        OpaqueFunction(function=launch_setup),
    ])
