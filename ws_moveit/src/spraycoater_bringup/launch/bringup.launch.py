#!/usr/bin/env python3
import os
import yaml
from math import radians, sin, cos

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _rpy_rad_to_quat(roll, pitch, yaw):
    cr = cos(roll * 0.5); sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5); sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5);   sy = sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def generate_launch_description():
    sim_arg = DeclareLaunchArgument("sim", default_value="true", description="Simulationsmodus (true|false)")

    def _setup(context):
        bringup_share = FindPackageShare("spraycoater_bringup").perform(context)
        robot_yaml = os.path.join(bringup_share, "config", "robot.yaml")

        # --- robot.yaml laden
        with open(robot_yaml, "r") as f:
            cfg = yaml.safe_load(f) or {}
        if not isinstance(cfg, dict):
            raise ValueError("[bringup] robot.yaml: Mapping erwartet")

        sel    = cfg.get("selected")
        robots = cfg.get("robots") or {}
        if not isinstance(sel, str) or not sel:
            raise ValueError("[bringup] 'selected' muss String sein (z. B. meca|omron)")
        if sel not in robots or not isinstance(robots[sel], dict):
            raise ValueError(f"[bringup] robots['{sel}'] fehlt/ungültig")
        sel_entry = robots[sel]

        moveit_pkg = str(sel_entry.get("moveit_pkg", "")).strip()
        if not moveit_pkg:
            raise ValueError(f"[bringup] robots.{sel}.moveit_pkg fehlt/leer")

        # --- world_to_robot_mount (top-level oder unter 'tf')
        tf_root = cfg.get("tf", {})
        tf_cfg  = tf_root.get("world_to_robot_mount") if isinstance(tf_root, dict) else None
        if tf_cfg is None:
            tf_cfg = cfg.get("world_to_robot_mount")
        if not isinstance(tf_cfg, dict):
            raise ValueError("[bringup] world_to_robot_mount fehlt (top-level oder unter 'tf')")

        parent = tf_cfg.get("parent")
        child  = tf_cfg.get("child")
        xyz    = tf_cfg.get("xyz")
        rpydeg = tf_cfg.get("rpy_deg")
        if not (isinstance(parent, str) and parent):
            raise ValueError("[bringup] tf.parent muss String sein")
        if not (isinstance(child, str) and child):
            raise ValueError("[bringup] tf.child muss String sein")
        if not (isinstance(xyz, list) and len(xyz) == 3):
            raise ValueError("[bringup] tf.xyz muss Liste[3] sein")
        if not (isinstance(rpydeg, list) and len(rpydeg) == 3):
            raise ValueError("[bringup] tf.rpy_deg muss Liste[3] sein")

        x, y, z = map(float, xyz)
        roll, pitch, yaw = map(lambda d: radians(float(d)), rpydeg)
        qx, qy, qz, qw = _rpy_rad_to_quat(roll, pitch, yaw)

        # --- MoveIt robot.launch.py einbinden
        moveit_share = FindPackageShare(moveit_pkg).perform(context)
        robot_launch = os.path.join(moveit_share, "launch", "robot.launch.py")
        if not os.path.exists(robot_launch):
            raise FileNotFoundError(f"[bringup] {moveit_pkg}/launch/robot.launch.py fehlt: {robot_launch}")

        include_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "mount_parent":  parent,
                "mount_child":   child,
                "mount_x":       str(x),
                "mount_y":       str(y),
                "mount_z":       str(z),
                "mount_qx":      str(qx),
                "mount_qy":      str(qy),
                "mount_qz":      str(qz),
                "mount_qw":      str(qw),
                "sim":           LaunchConfiguration("sim"),
            }.items()
        )

        # --- frames.yaml (flach, ohne Gruppen)
        frames_yaml = os.path.join(bringup_share, "config", "frames.yaml")
        if not os.path.exists(frames_yaml):
            raise FileNotFoundError(f"[bringup] frames.yaml fehlt: {frames_yaml}")

        # --- SceneManager
        scene_yaml  = os.path.join(bringup_share, "config", "scene.yaml")
        topics_yaml = os.path.join(bringup_share, "config", "topics.yaml")
        qos_yaml    = os.path.join(bringup_share, "config", "qos.yaml")
        if not os.path.exists(scene_yaml):
            raise FileNotFoundError(f"[bringup] scene.yaml fehlt: {scene_yaml}")
        for p in (topics_yaml, qos_yaml):
            if not os.path.exists(p):
                raise FileNotFoundError(f"[bringup] fehlt: {p}")

        cage_dirs      = sel_entry.get("cage_dirs",      [os.path.join(bringup_share, "resource", "cage")])
        mount_dirs     = sel_entry.get("mount_dirs",     [os.path.join(bringup_share, "resource", "substrate_mounts")])
        substrate_dirs = sel_entry.get("substrate_dirs", [os.path.join(bringup_share, "resource", "substrates")])

        scene_manager = Node(
            package="spraycoater_nodes_py",
            executable="scene",
            name="scene",
            output="screen",
            parameters=[
                {"scene_config": scene_yaml},
                {"frames_yaml":  frames_yaml},
                {"topics_yaml":  topics_yaml},
                {"qos_yaml":     qos_yaml},
                {"cage_dirs":      cage_dirs},
                {"mount_dirs":     mount_dirs},
                {"substrate_dirs": substrate_dirs},
            ],
        )
        scene_after_robot = TimerAction(period=5.0, actions=[scene_manager])

        # --- PosesManager (ebenfalls ohne frames_group)
        poses_yaml = os.path.join(bringup_share, "config", "poses.yaml")
        if not os.path.exists(poses_yaml):
            raise FileNotFoundError(f"[bringup] fehlt: {poses_yaml}")

        poses_manager = Node(
            package="spraycoater_nodes_py",
            executable="poses",
            name="poses",
            output="screen",
            parameters=[
                {"poses_config": poses_yaml},
                {"topics_yaml":  topics_yaml},
                {"qos_yaml":     qos_yaml},
                {"frames_yaml":  frames_yaml},
            ],
        )
        poses_after_robot = TimerAction(period=5.0, actions=[poses_manager])

        return [include_robot, scene_after_robot, poses_after_robot]

    return LaunchDescription([sim_arg, OpaqueFunction(function=_setup)])
