#!/usr/bin/env python3
import os, yaml
from math import radians
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from moveit_configs_utils import MoveItConfigsBuilder
from tf_transformations import quaternion_from_euler


def generate_launch_description():
    SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'UDPv4'),
    def launch_setup(context):
        cfg_pkg = FindPackageShare("mecademic_moveit_config").perform(context)

        # --- Static TF world → meca_mount ---
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

        # --- MoveIt Config inkl. Pipelines laden ---
        moveit_config = (
            MoveItConfigsBuilder("meca_500_r3", package_name="mecademic_moveit_config")
            .to_moveit_configs()
        )

        # --- ROS2 Control + Publisher + Controllers ---
        ros2_control = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                os.path.join(cfg_pkg, "config", "ros2_controllers.yaml"),
            ],
            output="screen",
        )

        robot_state_pub = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[moveit_config.robot_description],
            output="screen",
        )

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

        # --- MoveGroup + Servo + RViz ---
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            arguments=["--ros-args", "--log-level", "info"],
        )

        disable_internal_servo = SetEnvironmentVariable(
            name="MOVEIT_SERVO_AUTO_START", value="false"
        )

        servo_node = Node(
            package="moveit_servo",
            executable="servo_node",
            name="servo",
            output="screen",
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                {"use_sim_time": False},  # echte Hardware -> false; mit /clock -> true
                {
                    "moveit_servo": {
                        "command_in_type": "speed_units",              # wir schicken m/s & rad/s rein
                        "move_group_name": "meca_arm_group",
                        "planning_frame": "world",
                        "cartesian_command_in_topic": "/servo/cartesian_mm",
                        "joint_command_in_topic":     "/servo/joint_jog",
                        "pose_command_in_topic":      "/servo/delta_pose_cmds",

                        # >>> NEU: Ausgabe an den Positions-Controller
                        "command_out_topic": "/meca_arm_position_controller/commands",
                        "command_out_type":  "std_msgs/msg/Float64MultiArray",

                        "publish_period": 0.01,                         # 100 Hz
                        "incoming_command_timeout": 0.2,
                        "use_servo_services": True,
                        "check_collisions": False,

                        # Sinnvoll, wenn Float64MultiArray verwendet wird:
                        "publish_joint_positions": True,
                        "publish_joint_velocities": False,
                        "publish_joint_accelerations": False,
                    }
                },
            ],
            env={
                "FASTDDS_SHM_DEFAULT": "0",
                "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4",
            }
        )



        rviz_config_path = PathJoinSubstitution(
            [FindPackageShare("mecademic_moveit_config"), "config", "moveit.rviz"]
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
                moveit_config.pilz_cartesian_limits,
                {"use_sim_time": False},
            ],
        )

        return [
            static_tf,
            robot_state_pub,
            ros2_control,
            joint_state_broadcaster,
            arm_controller,
            move_group_node,
            disable_internal_servo,
            servo_node,
            rviz_node,
        ]

    return LaunchDescription([OpaqueFunction(function=launch_setup)])
