#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    GroupAction,
    OpaqueFunction,
    TimerAction,
    IncludeLaunchDescription,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import yaml

from mecademic_bringup.utils import rpy_deg_to_quat
from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
    PARAM_USE_FAKE_HW,
)

def generate_launch_description():
    use_fake_hw_arg = DeclareLaunchArgument("use_fake_hw", default_value="true")

    def launch_setup(context):
        pkg_share = FindPackageShare("mecademic_bringup").perform(context)

        # Load mount YAML
        mount_yaml = os.path.join(pkg_share, "config", "meca_mount.yaml")
        with open(mount_yaml, "r") as f:
            mount = yaml.safe_load(f)["meca_mount"]

        world_frame = mount["parent"]
        xyz = mount["xyz"]
        rpy_deg = mount["rpy_deg"]
        qx, qy, qz, qw = rpy_deg_to_quat(*rpy_deg)

        poses_yaml = os.path.join(pkg_share, "config", "poses.yaml")
        scene_yaml = os.path.join(pkg_share, "config", "scene.yaml")
        tool_yaml = os.path.join(pkg_share, "config", "tools.yaml")

        env = [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        ]

        # ---- Static TF world → meca_mount ----
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_world_to_meca_mount',
            arguments=[
                "--x", str(xyz[0]), "--y", str(xyz[1]), "--z", str(xyz[2]),
                "--qx", str(qx), "--qy", str(qy), "--qz", str(qz), "--qw", str(qw),
                "--frame-id", world_frame,
                "--child-frame-id", "meca_mount"
            ],
            output="screen"
        )

        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, "launch", "robot.launch.py")
            )
        )

        # Managers
        scene_manager = Node(
            package="mecademic_bringup",
            executable="scene_manager",
            name="scene_manager",
            parameters=[{PARAM_SCENE_CONFIG: scene_yaml}],
            output="screen",
        )

        tool_manager = Node(
            package="mecademic_bringup",
            executable="tool_manager",
            name="tool_manager",
            parameters=[{
                PARAM_TOOL_CONFIG: tool_yaml,
                PARAM_USE_FAKE_HW: True,
            }],
            output="screen",
        )

        poses_manager = Node(
            package="mecademic_bringup",
            executable="poses_manager",
            name="poses_manager",
            parameters=[{PARAM_POSES_CONFIG: poses_yaml}],
            output="screen",
        )

        spray_path_manager = Node(
            package="mecademic_bringup",
            executable="spray_path_manager",
            name="spray_path_manager",
            output="screen",
        )

        delayed_start = TimerAction(
            period=10.0,
            actions=[
                scene_manager,
                tool_manager,
                poses_manager,
                spray_path_manager,
            ]
        )

        return [
            GroupAction([
                *env,
                static_tf,
                robot_launch,
                delayed_start
            ])
        ]

    return LaunchDescription([
        use_fake_hw_arg,
        OpaqueFunction(function=launch_setup)
    ])
