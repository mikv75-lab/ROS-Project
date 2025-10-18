#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import math, os, yaml

def _rpy_deg_to_quat(r, p, y):
    rx, ry, rz = map(math.radians, (r, p, y))
    cr, sr = math.cos(rx/2), math.sin(rx/2)
    cp, sp = math.cos(ry/2), math.sin(ry/2)
    cy, sy = math.cos(rz/2), math.sin(rz/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - cr*sp*cy
    return qx, qy, qz, qw

def generate_launch_description():
    def launch_setup(context):
        # Feste Frames / Default-Lage
        parent_frame = "meca_mount"
        world_xyz = (0.0, 0.0, 0.0)
        world_rpy_deg = (0.0, 0.0, 90.0)  # yaw 90°
        qx, qy, qz, qw = _rpy_deg_to_quat(*world_rpy_deg)

        pkg_share = FindPackageShare("mecademic_bringup").perform(context)

        poses_yaml = f"{pkg_share}/config/poses.yaml"
        scene_yaml = f"{pkg_share}/config/scene.yaml"
        tool_yaml  = f"{pkg_share}/config/tools.yaml"
        rviz_config = f"{pkg_share}/config/bringup.rviz"

        # Aktives Tool aus tools.yaml lesen
        try:
            with open(tool_yaml, "r") as f:
                tools_data = yaml.safe_load(f) or {}
            active_tool = tools_data.get("active_tool", "none")
            if active_tool not in (tools_data.get("tools") or {}):
                active_tool = "none"
        except Exception:
            active_tool = "none"

        nodes = [
            # Env
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            SetEnvironmentVariable("XDG_RUNTIME_DIR", "/tmp/runtime-root"),

            # 1) Static TF world -> meca_mount (mit benannten Flags!)
            Node(
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
            ),

            # 2) ToolManager (verwaltet nur Tool-State)
            Node(
                package="mecademic_bringup",
                executable="tool_manager",
                name="tool_manager",
                parameters=[{
                    "tool_config": tool_yaml,
                    "use_fake_hw": True,   # Schalter bleibt wie gehabt in robot_with_tool.launch.py
                }],
                output="screen",
            ),

            # 3) Poses Manager (persistente Posen)
            Node(
                package="mecademic_bringup",
                executable="poses_manager",
                name="poses_manager",
                parameters=[{
                    "poses_yaml": poses_yaml,
                }],
                output="screen",
            ),

            # 4) Scene Manager (lädt scene.yaml)
            Node(
                package="mecademic_bringup",
                executable="scene_manager",
                name="scene_manager",
                parameters=[{
                    "scene_config": scene_yaml,
                }],
                output="screen",
            ),

            # 5) Robot & MoveIt (immer fest hier starten) – Tool aus tools.yaml
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, "launch", "robot_with_tool.launch.py")
                ),
                launch_arguments={
                    "tool": active_tool,
                    # use_fake_hw-Schalter bleibt wie in robot_with_tool.launch.py definiert (default true)
                }.items(),
            ),

            # 6) RViz (immer an)
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]

        return nodes

    return LaunchDescription([
        OpaqueFunction(function=launch_setup),
    ])
