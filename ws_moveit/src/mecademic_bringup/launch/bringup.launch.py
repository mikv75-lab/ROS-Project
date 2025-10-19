#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    GroupAction,
    TimerAction,
    OpaqueFunction,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

import os
import yaml
from mecademic_bringup.scene.utils import rpy_deg_to_quat
from mecademic_bringup.common.params import (
    PARAM_SCENE_CONFIG,
    PARAM_POSES_CONFIG,
    PARAM_TOOL_CONFIG,
    PARAM_USE_FAKE_HW,
)

def generate_launch_description():
    # Optional: Forward-Arg, falls du später Fake-HW von oben schalten willst.
    # (Aktuell wird es nicht weiterverwendet, da robot.launch.py own default hat.)
    use_fake_hw_arg = DeclareLaunchArgument("use_fake_hw", default_value="true")

    def launch_setup(context):
        # --- Basis Frames ---
        parent_frame = "meca_mount"
        world_xyz = (0.0, 0.0, 0.0)
        world_rpy_deg = (0.0, 0.0, 90.0)  # yaw 90°
        qx, qy, qz, qw = rpy_deg_to_quat(*world_rpy_deg)

        pkg_share = FindPackageShare("mecademic_bringup").perform(context)

        poses_yaml = os.path.join(pkg_share, "config", "poses.yaml")
        scene_yaml = os.path.join(pkg_share, "config", "scene.yaml")
        tool_yaml  = os.path.join(pkg_share, "config", "tools.yaml")

        # Aktives Tool aus tools.yaml lesen (nur Info/Zweck: ToolManager lädt es)
        try:
            with open(tool_yaml, "r") as f:
                tools_data = yaml.safe_load(f) or {}
            active_tool = tools_data.get("active_tool", "none")
            if active_tool not in (tools_data.get("tools") or {}):
                active_tool = "none"
        except Exception:
            active_tool = "none"

        # --- Umwelt / RMW ---
        env = [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        ]

        # --- Static TF world -> meca_mount ---
        tf_world_to_mount = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_world_to_meca_mount",
            arguments=[
                "--x",   str(world_xyz[0]),
                "--y",   str(world_xyz[1]),
                "--z",   str(world_xyz[2]),
                "--qx",  str(qx),
                "--qy",  str(qy),
                "--qz",  str(qz),
                "--qw",  str(qw),
                "--frame-id", "world",
                "--child-frame-id", parent_frame,
            ],
            output="screen",
        )

        # --- Robot (URDF, ros2_control, MoveIt, RViz) ---
        robot_launch = PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "robot.launch.py")
        )
        robot_include = OpaqueFunction(function=lambda ctx: [  # include wie Group
            # Wichtig: keine "tool"-Argumente übergeben – ToolManager übernimmt das.
            # Optional könntest du hier use_fake_hw durchreichen: {"use_fake_hw": LaunchConfiguration("use_fake_hw").perform(ctx)}
            IncludeLaunchDescription(robot_launch)
        ])

        # --- Scene/Tool/Pose/Spray mit stabiler Reihenfolge via Timer ---
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
                PARAM_USE_FAKE_HW: True,  # wie bisher
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

        # Start-Reihenfolge:
        # 1) static TF
        # 2) robot (URDF+MoveIt+RViz)
        # 3) scene_manager  (erst Szene/TFs)
        # 4) tool_manager   (dann Tool attach)
        # 5) poses_manager
        # 6) spray_path_manager
        ordered = [
            *env,
            tf_world_to_mount,
            # Robot sofort starten (MoveIt & RViz booten)
            robot_include,
            # Danach in staffelnden Delays:
            TimerAction(period=3.0, actions=[scene_manager]),
            TimerAction(period=4.0, actions=[tool_manager]),
            TimerAction(period=4.5, actions=[poses_manager]),
            TimerAction(period=5.0, actions=[spray_path_manager]),
        ]

        return [GroupAction(ordered)]

    return LaunchDescription([
        use_fake_hw_arg,
        OpaqueFunction(function=launch_setup),
    ])
