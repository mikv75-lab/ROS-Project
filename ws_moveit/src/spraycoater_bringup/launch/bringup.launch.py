#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import yaml
from math import radians, sin, cos

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# -----------------------------------------------------------
# Helpers
# -----------------------------------------------------------

def _rpy_rad_to_quat(roll, pitch, yaw):
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * sp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _get_world_to_mount(cfg: dict) -> dict:
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


# -----------------------------------------------------------
# Launch
# -----------------------------------------------------------

def generate_launch_description():
    quiet_env = SetEnvironmentVariable(
        name="RCUTILS_LOGGING_SEVERITY",
        value="ERROR",
    )

    role_arg = DeclareLaunchArgument(
        "role",
        default_value="shadow",
        description="shadow | live",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulated time",
    )

    def _setup(context):
        role = LaunchConfiguration("role").perform(context).strip()
        use_sim_time = (
            LaunchConfiguration("use_sim_time")
            .perform(context)
            .strip()
            .lower() == "true"
        )

        # ---------------------------------------------------
        # Load robot.yaml
        # ---------------------------------------------------
        bringup_share = FindPackageShare("spraycoater_bringup").perform(context)
        with open(
            os.path.join(bringup_share, "config", "robot.yaml"),
            "r",
            encoding="utf-8",
        ) as f:
            cfg = yaml.safe_load(f) or {}

        sel = cfg.get("selected")
        robots = cfg.get("robots") or {}
        if not isinstance(sel, str) or sel not in robots:
            raise KeyError("[bringup] robot.yaml: 'selected' fehlt/ungültig")

        robot = robots[sel]

        moveit_pkg = robot.get("moveit_pkg")
        if not isinstance(moveit_pkg, str) or not moveit_pkg:
            raise KeyError("[bringup] robot.yaml: robots[selected].moveit_pkg fehlt")

        # ---------------------------------------------------
        # World → robot_mount TF
        # ---------------------------------------------------
        tf_cfg = _get_world_to_mount(cfg)
        for k in ("parent", "child", "xyz", "rpy_deg"):
            if k not in tf_cfg:
                raise KeyError(f"[bringup] world_to_robot_mount.{k} fehlt")

        x, y, z = map(float, tf_cfg["xyz"])
        roll, pitch, yaw = map(lambda d: radians(float(d)), tf_cfg["rpy_deg"])
        qx, qy, qz, qw = _rpy_rad_to_quat(roll, pitch, yaw)

        moveit_share = FindPackageShare(moveit_pkg).perform(context)

        # ---------------------------------------------------
        # Role → Robot Launch + Backend
        # ---------------------------------------------------
        if role == "live":
            robot_launch = os.path.join(moveit_share, "launch", "omron_live.launch.py")
            backend = "omron_real"
            robot_exec = "robot_omron"   # ✅ entry_point aus setup.py
        else:
            robot_launch = os.path.join(moveit_share, "launch", "omron_shadow.launch.py")
            backend = "omron_sim"
            robot_exec = "robot_sim"    # ✅ entry_point aus setup.py

        if not os.path.exists(robot_launch):
            raise FileNotFoundError(f"[bringup] robot launch fehlt: {robot_launch}")

        # ---------------------------------------------------
        # Include robot launch (move_group + RViz laufen dort immer)
        # ---------------------------------------------------
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

        # ---------------------------------------------------
        # Spraycoater Nodes
        # ---------------------------------------------------
        common_params = {
            "backend": backend,
            "use_sim_time": use_sim_time,
        }

        # ✅ RobotNode (publisht tcp_pose + joints für GUI)
        robot_node = Node(
            package="spraycoater_nodes_py",
            executable=robot_exec,     # ✅ robot_sim / robot_omron
            name="robot",             # ✅ Node-Name bleibt stabil "robot"
            namespace=role,
            parameters=[common_params],
            output="screen",
            emulate_tty=True,
        )

        scene = Node(
            package="spraycoater_nodes_py",
            executable="scene",
            namespace=role,
            parameters=[common_params],
        )

        poses = Node(
            package="spraycoater_nodes_py",
            executable="poses",
            namespace=role,
            parameters=[common_params],
        )

        spray = Node(
            package="spraycoater_nodes_py",
            executable="spray_path",
            namespace=role,
            parameters=[common_params],
        )

        servo_bridge = Node(
            package="spraycoater_nodes_py",
            executable="servo",
            namespace=role,
            parameters=[common_params],
        )

        motion = Node(
            package="spraycoater_nodes_py",
            executable="motion",
            namespace=role,
            parameters=[common_params],
        )

        return [
            include_robot,
            TimerAction(period=6.0, actions=[robot_node]),
            TimerAction(period=7.0, actions=[scene]),
            TimerAction(period=8.0, actions=[poses]),
            TimerAction(period=9.0, actions=[spray]),
            TimerAction(period=10.0, actions=[servo_bridge]),
            TimerAction(period=12.0, actions=[motion]),
        ]

    return LaunchDescription(
        [
            quiet_env,
            role_arg,
            use_sim_time_arg,
            OpaqueFunction(function=_setup),
        ]
    )
