#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    tool_arg = DeclareLaunchArgument("tool", default_value="none")
    use_fake_hw_arg = DeclareLaunchArgument("use_fake_hw", default_value="true")

    def launch_setup(context):
        tool = LaunchConfiguration("tool").perform(context)
        use_fake_hw = LaunchConfiguration("use_fake_hw").perform(context)

        print(f"[robot_with_tool] Starting with tool = {tool}, fake_hw = {use_fake_hw}")

        use_tool1 = "true" if tool == "spray_nozzle_01" else "false"
        use_tool2 = "true" if tool == "spray_nozzle_02" else "false"

        # ----- Static TF world -> meca_mount -----
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_world_to_meca_mount",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "1.5708",
                "--frame-id", "world",
                "--child-frame-id", "meca_mount"
            ]
        )

        # ----- MoveIt Config -----
        desc_pkg = FindPackageShare("mecademic_description").perform(context)
        cfg_pkg = FindPackageShare("mecademic_moveit_config").perform(context)

        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .robot_description(
                file_path=os.path.join(desc_pkg, "urdf", "meca_500_r3_with_tool.xacro"),
                mappings={
                    "use_tool1": use_tool1,
                    "use_tool2": use_tool2,
                    "use_fake_hw": use_fake_hw
                }
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


        # ----- Nodes -----
        robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description]
        )

        ros2_control = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description,
                        os.path.join(cfg_pkg, "config", "ros2_controllers.yaml")],
            output="screen"
        )

        joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        )

        arm_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["meca_arm_group_controller"],
            output="screen"
        )

        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            name="moveit_servo",
            output="screen",
            parameters=[
                # --- Servo Konfiguration ---
                {
                    "moveit_servo.move_group_name": "meca_arm_group",
                    "moveit_servo.planning_frame": "world",
                    "moveit_servo.command_type": "velocity",
                    "moveit_servo.publish_period": 0.01,

                    # Topics
                    "moveit_servo.joint_command_in_topic": "/moveit_servo/delta_joint_cmds",
                    "moveit_servo.cartesian_command_in_topic": "/moveit_servo/delta_twist_cmds",

                    # Scaling
                    "moveit_servo.scale.linear": 0.3,
                    "moveit_servo.scale.rotational": 0.8,
                    "moveit_servo.scale.joint": 0.6,

                    # Safety
                    "moveit_servo.lower_singularity_threshold": 0.001,
                    "moveit_servo.hard_stop_singularity_threshold": 0.01,

                    # Output to controller
                    "moveit_servo.command_out_topic": "/meca_arm_group_controller/joint_trajectory",
                    "moveit_servo.command_out_type": "trajectory_msgs/JointTrajectory",
                    "moveit_servo.allow_missing_joints": True,
                    "moveit_servo.use_smoothing": False,
                    "moveit_servo.check_collisions": False,
                },

                # ---- MoveIt Required Parameters ---
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )

        
        move_group = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {"moveit_controller_manager": "moveit_ros_control_interface/Ros2ControlManager"}
            ]
        )

        # ----- RViz -----
        rviz_config = PathJoinSubstitution([FindPackageShare("mecademic_bringup"), "config", "bringup.rviz"])
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics
            ]
        )

        return [
            static_tf,
            robot_state_pub,
            ros2_control,
            joint_state_broadcaster,
            arm_controller,
            move_group,
            servo_node,
            rviz,
        ]

    return LaunchDescription([tool_arg, use_fake_hw_arg, OpaqueFunction(function=launch_setup)])
