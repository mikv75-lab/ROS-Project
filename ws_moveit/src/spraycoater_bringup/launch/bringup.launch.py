# spraycoater_bringup/launch/bringup.launch.py
#!/usr/bin/env python3
import os
import yaml
from math import radians

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # einziges zusätzliches Argument
    sim_arg = DeclareLaunchArgument(
        "sim",
        default_value="true",
        description="Simulationsmodus (true|false)"
    )

    def _setup(context):
        # --- robot.yaml strikt laden ---
        bringup_share = FindPackageShare("spraycoater_bringup").perform(context)
        robot_yaml = os.path.join(bringup_share, "config", "robot.yaml")
        if not os.path.exists(robot_yaml):
            raise FileNotFoundError(f"[bringup] Konfigurationsdatei fehlt: {robot_yaml}")

        with open(robot_yaml, "r") as f:
            cfg = yaml.safe_load(f)
        if not isinstance(cfg, dict):
            raise ValueError("[bringup] robot.yaml: Mapping erwartet")

        # Pflichtfelder prüfen
        sel = cfg.get("selected")
        robots = cfg.get("robots")
        tf_root = cfg.get("tf")
        if not isinstance(sel, str) or not sel:
            raise ValueError("[bringup] 'selected' muss String sein (meca|omron)")
        if not isinstance(robots, dict) or sel not in robots:
            raise ValueError(f"[bringup] robots['{sel}'] fehlt")
        if not isinstance(tf_root, dict) or "world_to_robot_mount" not in tf_root:
            raise ValueError("[bringup] tf.world_to_robot_mount fehlt")

        # MoveIt-Paket ermitteln
        sel_entry = robots[sel]
        if not isinstance(sel_entry, dict) or "moveit_pkg" not in sel_entry:
            raise ValueError(f"[bringup] robots.{sel}.moveit_pkg fehlt")
        moveit_pkg = str(sel_entry["moveit_pkg"]).strip()
        if not moveit_pkg:
            raise ValueError(f"[bringup] robots.{sel}.moveit_pkg ist leer")

        # TF-Parameter lesen
        tf_cfg = tf_root["world_to_robot_mount"]
        if not isinstance(tf_cfg, dict):
            raise ValueError("[bringup] tf.world_to_robot_mount: Mapping erwartet")

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

        # --- Static TF: world -> robot_mount (RPY in RAD) ---
        static_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_world_to_robot_mount",
            # x y z roll pitch yaw parent child
            arguments=[str(x), str(y), str(z),
                       str(roll), str(pitch), str(yaw),
                       parent, child],
            output="screen",
        )

        # --- Robot-Launch strikt einbinden ---
        moveit_share = FindPackageShare(moveit_pkg).perform(context)
        robot_launch = os.path.join(moveit_share, "launch", "robot.launch.py")
        if not os.path.exists(robot_launch):
            raise FileNotFoundError(f"[bringup] {moveit_pkg}/launch/robot.launch.py fehlt: {robot_launch}")

        include_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "mount_parent":    parent,
                "mount_child":     child,
                "mount_x":         str(x),
                "mount_y":         str(y),
                "mount_z":         str(z),
                "mount_roll_rad":  str(roll),
                "mount_pitch_rad": str(pitch),
                "mount_yaw_rad":   str(yaw),
                "sim":             LaunchConfiguration("sim"),  # nur sim zusätzlich
            }.items()
        )

        return [static_tf, include_robot]

    return LaunchDescription([
        sim_arg,
        OpaqueFunction(function=_setup),
    ])
