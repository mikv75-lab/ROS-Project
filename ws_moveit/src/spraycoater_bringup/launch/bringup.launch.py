#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import yaml
from math import radians, sin, cos

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _rpy_rad_to_quat(roll, pitch, yaw):
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,  # x
        cr * sp * cy + sr * sp * sy,  # y
        cr * cp * sy - sr * sp * cy,  # z
        cr * cp * cy + sr * sp * sy,  # w
    )


def _get_world_to_mount(cfg: dict) -> dict:
    """
    Unterstützt beide Layouts:
      cfg['tf']['world_to_robot_mount']
      cfg['world_to_robot_mount']
    """
    tf_root = cfg.get("tf")
    if isinstance(tf_root, dict):
        w2m = tf_root.get("world_to_robot_mount")
        if isinstance(w2m, dict):
            return w2m

    w2m = cfg.get("world_to_robot_mount")
    if isinstance(w2m, dict):
        return w2m

    raise KeyError(
        "[bringup] world_to_robot_mount fehlt. Erwartet entweder "
        "cfg['tf']['world_to_robot_mount'] oder cfg['world_to_robot_mount']"
    )


def generate_launch_description():
    # ---------------------------------------------------------
    # Launch Args
    # ---------------------------------------------------------
    role_arg = DeclareLaunchArgument(
        "role",
        default_value="shadow",
        description="Instanz-Name/Namespace (z.B. shadow oder live)",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time",
    )

    def _setup(context):
        role = LaunchConfiguration("role").perform(context).strip()
        use_sim_time = LaunchConfiguration("use_sim_time").perform(context).strip().lower() == "true"

        # ---------------------------------------------------------
        # robot.yaml laden
        # ---------------------------------------------------------
        bringup_share = FindPackageShare("spraycoater_bringup").perform(context)
        robot_yaml = os.path.join(bringup_share, "config", "robot.yaml")
        if not os.path.exists(robot_yaml):
            raise FileNotFoundError(f"[bringup] robot.yaml nicht gefunden: {robot_yaml}")

        with open(robot_yaml, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}

        sel = cfg.get("selected")
        robots = cfg.get("robots") or {}
        if not isinstance(sel, str) or sel not in robots:
            raise KeyError("[bringup] robot.yaml: 'selected' fehlt/ungültig oder nicht in 'robots'")

        robot = robots[sel]
        moveit_pkg = robot.get("moveit_pkg")
        if not isinstance(moveit_pkg, str) or not moveit_pkg:
            raise KeyError("[bringup] robot.yaml: robots[selected].moveit_pkg fehlt")

        # ---------------------------------------------------------
        # TF world->robot_mount aus cfg
        # ---------------------------------------------------------
        tf_cfg = _get_world_to_mount(cfg)
        for k in ("parent", "child", "xyz", "rpy_deg"):
            if k not in tf_cfg:
                raise KeyError(f"[bringup] world_to_robot_mount.{k} fehlt")

        x, y, z = map(float, tf_cfg["xyz"])
        roll, pitch, yaw = map(lambda d: radians(float(d)), tf_cfg["rpy_deg"])
        qx, qy, qz, qw = _rpy_rad_to_quat(roll, pitch, yaw)

        # ---------------------------------------------------------
        # MoveIt package share
        # ---------------------------------------------------------
        moveit_share = FindPackageShare(moveit_pkg).perform(context)

        # moveit_common.py aus moveit_pkg/launch laden
        moveit_launch_dir = os.path.join(moveit_share, "launch")
        if moveit_launch_dir not in sys.path:
            sys.path.insert(0, moveit_launch_dir)

        from moveit_common import create_omron_moveit_config  # noqa: E402
        moveit_config = create_omron_moveit_config()

        # ---------------------------------------------------------
        # shadow/live Auswahl
        # ---------------------------------------------------------
        if role == "live":
            robot_launch = os.path.join(moveit_share, "launch", "omron_live.launch.py")
            backend = "omron_real"
            robot_exec = "robot_omron"
        else:
            robot_launch = os.path.join(moveit_share, "launch", "omron_shadow.launch.py")
            backend = "omron_sim"
            robot_exec = "robot_sim"

        if not os.path.exists(robot_launch):
            raise FileNotFoundError(f"[bringup] robot launch fehlt: {robot_launch}")

        include_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "namespace": role,
                "use_sim_time": "true" if use_sim_time else "false",
                "mount_parent": str(tf_cfg["parent"]),
                "mount_child": str(tf_cfg["child"]),
                "mount_x": str(x),
                "mount_y": str(y),
                "mount_z": str(z),
                "mount_qx": str(qx),
                "mount_qy": str(qy),
                "mount_qz": str(qz),
                "mount_qw": str(qw),
            }.items(),
        )

        # ---------------------------------------------------------
        # Unsere Nodes
        # ---------------------------------------------------------
        common_params = {"backend": backend, "use_sim_time": use_sim_time}

        # TF GLOBAL lassen
        tf_remaps_global = [("tf", "/tf"), ("tf_static", "/tf_static")]

        robot_node = Node(
            package="spraycoater_nodes_py",
            executable=robot_exec,
            name="robot",
            namespace=role,
            parameters=[common_params],
            remappings=tf_remaps_global,
            output="screen",
            emulate_tty=True,
        )

        scene = Node(
            package="spraycoater_nodes_py",
            executable="scene",
            name="scene",
            namespace=role,
            parameters=[common_params],
            remappings=tf_remaps_global,
            output="screen",
            emulate_tty=True,
        )

        poses = Node(
            package="spraycoater_nodes_py",
            executable="poses",
            name="poses",
            namespace=role,
            parameters=[common_params],
            remappings=tf_remaps_global,
            output="screen",
            emulate_tty=True,
        )

        spray = Node(
            package="spraycoater_nodes_py",
            executable="spray_path",
            name="spray_path",
            namespace=role,
            parameters=[common_params],
            remappings=tf_remaps_global,
            output="screen",
            emulate_tty=True,
        )

        servo_bridge = Node(
            package="spraycoater_nodes_py",
            executable="servo",
            name="servo",
            namespace=role,
            parameters=[common_params],
            remappings=tf_remaps_global,
            output="screen",
            emulate_tty=True,
        )

        moveitpy_node = Node(
            package="spraycoater_nodes_py",
            executable="moveit_py",
            name="moveit_py",
            namespace=role,
            parameters=[
                moveit_config.to_dict(),
                common_params,
            ],
            remappings=tf_remaps_global,
            output="screen",
            emulate_tty=True,
        )

        return [
            include_robot,
            TimerAction(period=6.0, actions=[robot_node]),
            TimerAction(period=7.0, actions=[scene]),
            TimerAction(period=8.0, actions=[poses]),
            TimerAction(period=9.0, actions=[spray]),
            TimerAction(period=10.0, actions=[servo_bridge]),
            TimerAction(period=12.0, actions=[moveitpy_node]),
        ]

    return LaunchDescription([
        role_arg,
        use_sim_time_arg,
        OpaqueFunction(function=_setup),
    ])
