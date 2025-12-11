#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
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


def _rpy_rad_to_quat(roll, pitch, yaw):
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * sp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def generate_launch_description():
    # global: logs leiser
    quiet_env = SetEnvironmentVariable(name="RCUTILS_LOGGING_SEVERITY", value="ERROR")

    # Rolle/Namespace: shadow | live
    role_arg = DeclareLaunchArgument(
        "role",
        default_value="shadow",
        description="Rolle/Namespace für diese Instanz (z.B. 'shadow' oder 'live')",
    )

    # Sim-Flag (für Auswahl robot_sim / robot_omron)
    sim_arg = DeclareLaunchArgument(
        "sim", default_value="true", description="Simulationsmodus (true|false)"
    )

    # Backend (z.B. default/omron)
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="default",
        description="Backend für Spraycoater-Nodes (z.B. 'default' oder 'omron')",
    )

    # use_sim_time
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Simulated Time nutzen (true|false)",
    )

    def _setup(context):
        bringup_share = FindPackageShare("spraycoater_bringup").perform(context)
        robot_yaml = os.path.join(bringup_share, "config", "robot.yaml")

        # LaunchConfigs abrufen
        role = LaunchConfiguration("role")           # Namespace für alle Nodes
        backend = LaunchConfiguration("backend")
        use_sim_time = LaunchConfiguration("use_sim_time")
        sim = LaunchConfiguration("sim").perform(context)

        # robot.yaml laden
        with open(robot_yaml, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        if not isinstance(cfg, dict):
            raise ValueError("[bringup] robot.yaml: Mapping erwartet")

        sel = cfg.get("selected")
        robots = cfg.get("robots") or {}
        if not isinstance(sel, str) or not sel:
            raise ValueError("[bringup] 'selected' muss String sein (z. B. meca|omron)")
        if sel not in robots or not isinstance(robots[sel], dict):
            raise ValueError(f"[bringup] robots['{sel}'] fehlt/ungültig")
        sel_entry = robots[sel]

        moveit_pkg = str(sel_entry.get("moveit_pkg", "")).strip()
        if not moveit_pkg:
            raise ValueError(f"[bringup] robots.{sel}.moveit_pkg fehlt/leer")

        # Robot-Name aus robot.yaml oder aus moveit_pkg ableiten (optional)
        robot_name = sel_entry.get("robot_name")
        if not robot_name:
            robot_name = moveit_pkg
            if robot_name.endswith("_moveit_config"):
                robot_name = robot_name[: -len("_moveit_config")]

        # world_to_robot_mount (top-level oder unter 'tf')
        tf_root = cfg.get("tf", {})
        tf_cfg = tf_root.get("world_to_robot_mount") if isinstance(tf_root, dict) else None
        if tf_cfg is None:
            tf_cfg = cfg.get("world_to_robot_mount")
        if not isinstance(tf_cfg, dict):
            raise ValueError("[bringup] world_to_robot_mount fehlt (top-level oder unter 'tf')")

        parent = tf_cfg.get("parent")
        child = tf_cfg.get("child")
        xyz = tf_cfg.get("xyz")
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

        # --------- MoveIt robot launch file einbinden ---------
        moveit_share = FindPackageShare(moveit_pkg).perform(context)

        # Je nach sim-Flag: robot_sim.launch.py oder robot_omron.launch.py
        robot_launch_file = "robot_sim.launch.py" if sim.lower() == "true" else "robot_omron.launch.py"
        robot_launch = os.path.join(moveit_share, "launch", robot_launch_file)

        if not os.path.exists(robot_launch):
            raise FileNotFoundError(f"[bringup] MoveIt launch fehlt: {robot_launch}")

        include_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                # Pose / Mount
                "mount_parent": parent,
                "mount_child": child,
                "mount_x": str(x),
                "mount_y": str(y),
                "mount_z": str(z),
                "mount_qx": str(qx),
                "mount_qy": str(qy),
                "mount_qz": str(qz),
                "mount_qw": str(qw),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                # 🔁 Namespace -> kommt in robot_sim/robot_omron als LaunchArgument("namespace")
                "namespace": role,
            }.items(),
        )

        # --------- MoveIt-Konfig für Motion-Node (MoveItPy) ----------
        launch_dir = os.path.join(moveit_share, "launch")
        if launch_dir not in sys.path:
            sys.path.insert(0, launch_dir)

        from moveit_common import create_omron_moveit_config  # type: ignore

        moveit_config = create_omron_moveit_config()
        moveit_cfg_dict = moveit_config.to_dict()

        # ✨ KRITISCHER FIX: YAML-Aliase auflösen
        moveit_cfg_yaml_str = yaml.dump(moveit_cfg_dict, default_flow_style=False)
        moveit_cfg_dict_clean = yaml.safe_load(moveit_cfg_yaml_str)

        # Optional: Existenzprüfungen der zentralen Configs
        must_exist = [
            os.path.join(bringup_share, "config", "frames.yaml"),
            os.path.join(bringup_share, "config", "scene.yaml"),
            os.path.join(bringup_share, "config", "topics.yaml"),
            os.path.join(bringup_share, "config", "qos.yaml"),
            os.path.join(bringup_share, "config", "poses.yaml"),
        ]
        for p in must_exist:
            if not os.path.exists(p):
                raise FileNotFoundError(f"[bringup] fehlt: {p}")

        # --------- Spraycoater-Nodes mit backend + use_sim_time + Namespace ---------

        # Scene-Node
        scene_node = Node(
            package="spraycoater_nodes_py",
            executable="scene",
            name="scene",
            namespace=role,
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "error"],
            parameters=[{"backend": backend, "use_sim_time": use_sim_time}],
        )
        scene_after_robot = TimerAction(period=10.0, actions=[scene_node])

        # Poses-Node
        poses_node = Node(
            package="spraycoater_nodes_py",
            executable="poses",
            name="poses",
            namespace=role,
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "error"],
            parameters=[{"backend": backend, "use_sim_time": use_sim_time}],
        )
        poses_after_robot = TimerAction(period=10.0, actions=[poses_node])

        # Spray-Path-Node
        spray_node = Node(
            package="spraycoater_nodes_py",
            executable="spray_path",
            name="spray_path",
            namespace=role,
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "error"],
            parameters=[{"backend": backend, "use_sim_time": use_sim_time}],
        )
        spray_after_robot = TimerAction(period=10.0, actions=[spray_node])

        # Servo-Node
        servo_node = Node(
            package="spraycoater_nodes_py",
            executable="servo",
            name="servo",
            namespace=role,
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[{"backend": backend, "use_sim_time": use_sim_time}],
        )
        servo_after_robot = TimerAction(period=10.0, actions=[servo_node])

        # Motion-Node mit bereinigter MoveIt-Config
        motion_node = Node(
            package="spraycoater_nodes_py",
            executable="motion",
            name="motion",
            namespace=role,
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[
                moveit_cfg_dict_clean,  # ← Bereinigte Version ohne YAML-Aliase
                {"backend": backend, "use_sim_time": use_sim_time},
            ],
        )
        motion_after_robot = TimerAction(period=10.0, actions=[motion_node])

        # Rückgabe der Sequenz
        return [
            include_robot,
            scene_after_robot,
            poses_after_robot,
            spray_after_robot,
            servo_after_robot,
            motion_after_robot,
        ]

    return LaunchDescription(
        [
            quiet_env,
            role_arg,
            sim_arg,
            backend_arg,
            use_sim_time_arg,
            OpaqueFunction(function=_setup),
        ]
    )
