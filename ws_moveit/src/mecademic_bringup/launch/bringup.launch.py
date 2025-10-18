#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import math

def _parse_vec3(s: str):
    parts = [p for p in s.replace(",", " ").split() if p]
    if len(parts) != 3:
        raise RuntimeError(f"expected 3 floats, got: {s!r}")
    return [float(parts[0]), float(parts[1]), float(parts[2])]

def _rpy_deg_to_quat(r, p, y):
    rx, ry, rz = map(math.radians, (r, p, y))
    cr, sr = math.cos(rx/2), math.sin(rx/2)
    cp, sp = math.cos(ry/2), math.sin(ry/2)
    cy, sy = math.cos(rz/2), math.sin(rz/2)
    qw = cr*cp*cy + sr*sp*sy
    qx = sr*cp*cy - cr*sp*sy
    qy = cr*sp*cy + sr*cp*sy
    qz = cr*cp*sy - sr*sp*cy
    return qx, qy, qz, qw

def generate_launch_description():
    # --- Args ---
    world_xyz_arg = DeclareLaunchArgument("world_to_meca_xyz", default_value="0 0 0")
    world_rpy_arg = DeclareLaunchArgument("world_to_meca_rpy_deg", default_value="0 0 90")
    scene_file_arg = DeclareLaunchArgument(
        "scene_file",
        default_value=PathJoinSubstitution([FindPackageShare("mecademic_bringup"), "config", "scene.yaml"]),
        description="Scene YAML to load on startup",
    )
    start_toolmgr_arg = DeclareLaunchArgument(
        "start_tool_manager", default_value="false",
        description="Start tool_manager (robot bringup handled elsewhere)"
    )

    def launch_setup(context):
        parent_frame = "meca_mount"

        world_xyz = _parse_vec3(LaunchConfiguration("world_to_meca_xyz").perform(context))
        world_rpy = _parse_vec3(LaunchConfiguration("world_to_meca_rpy_deg").perform(context))
        qx, qy, qz, qw = _rpy_deg_to_quat(*world_rpy)

        scene_file = LaunchConfiguration("scene_file").perform(context)
        start_toolmgr = LaunchConfiguration("start_tool_manager").perform(context).lower() == "true"

        nodes = [
            # Env fixes
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

            # 2) Poses Manager
            Node(
                package="mecademic_bringup",
                executable="poses_manager",
                name="poses_manager",
                parameters=[{
                    "positions_yaml": PathJoinSubstitution([
                        FindPackageShare("mecademic_bringup"), "config", "fixed_positions.yaml"
                    ]),
                    "parent_frame": parent_frame,
                }],
                output="screen",
            ),

            # 3) Scene Manager (lädt scene.yaml automatisch)
            Node(
                package="mecademic_bringup",
                executable="scene_manager",
                name="scene_manager",
                parameters=[{"scene_config": scene_file}],
                output="screen",
            ),

            # 4) RViz immer starten
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", PathJoinSubstitution([
                    FindPackageShare("mecademic_bringup"), "config", "bringup.rviz"
                ])],
                output="screen",
            ),
        ]

        # Optional: tool_manager nur wenn explizit gewünscht
        if start_toolmgr:
            nodes.append(Node(
                package="mecademic_bringup",
                executable="tool_manager",
                name="tool_manager",
                # Achtung: Ob der ToolManager wirklich keinen Roboter startet,
                # hängt von seiner internen Logik/YAML ab. Hier setzen wir KEINE Auto-Starts.
                parameters=[{"start_robot": False}],
                output="screen",
            ))

        return nodes

    return LaunchDescription([
        world_xyz_arg,
        world_rpy_arg,
        scene_file_arg,
        start_toolmgr_arg,
        OpaqueFunction(function=launch_setup),
    ])
