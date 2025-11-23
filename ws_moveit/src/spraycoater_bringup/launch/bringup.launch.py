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
from moveit_configs_utils import MoveItConfigsBuilder


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

    sim_arg = DeclareLaunchArgument(
        "sim", default_value="true", description="Simulationsmodus (true|false)"
    )

    def _setup(context):
        bringup_share = FindPackageShare("spraycoater_bringup").perform(context)
        robot_yaml = os.path.join(bringup_share, "config", "robot.yaml")

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

        # MoveIt robot.launch.py einbinden (ROS2-Control, Servo, move_group, RViz)
        moveit_share = FindPackageShare(moveit_pkg).perform(context)
        robot_launch = os.path.join(moveit_share, "launch", "robot.launch.py")
        if not os.path.exists(robot_launch):
            raise FileNotFoundError(f"[bringup] MoveIt robot.launch.py fehlt: {robot_launch}")

        include_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_launch),
            launch_arguments={
                "mount_parent": parent,
                "mount_child": child,
                "mount_x": str(x),
                "mount_y": str(y),
                "mount_z": str(z),
                "mount_qx": str(qx),
                "mount_qy": str(qy),
                "mount_qz": str(qz),
                "mount_qw": str(qw),
                "sim": LaunchConfiguration("sim"),
            }.items(),
        )

        # MoveIt-Konfig für Motion-Node (MoveItPy)
        # aktuell hart auf omron_viper_s650, passt aber zu deinem Setup
        moveit_config = (
            MoveItConfigsBuilder("omron_viper_s650", package_name=moveit_pkg)
            .to_moveit_configs()
        )
        moveit_cfg_dict = moveit_config.to_dict()

        # --- Planungspipelines: exakt wie im funktionierenden Beispiel ---
        moveit_cfg_dict["planning_pipelines"] = {
            "pipeline_names": ["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
            "default_planning_pipeline": "ompl",
            "ompl": {
                "planning_plugin": "ompl_interface/OMPLPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "ompl_planning",
            },
            "pilz_industrial_motion_planner": {
                "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "pilz_industrial_motion_planner_planning",
            },
            "chomp": {
                "planning_plugin": "chomp_interface/CHOMPPlanner",
                "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "chomp_planning",
            },
            "stomp": {
                "planning_plugin": "stomp_moveit/StompPlanner",
                "request_adapters": "default_planning_request_adapters/AddTimeOptimalParameterization",
                "start_state_max_bounds_error": 0.1,
                "parameter_namespace": "stomp_planning",
            },
        }

        moveit_cfg_dict["moveit_cpp"] = {
            "default_planning_pipeline": "ompl",
            "planning_pipelines": {
                "pipeline_names": ["ompl", "pilz_industrial_motion_planner", "chomp", "stomp"],
                "default_planning_pipeline": "ompl",
            },
        }

        # 🔧 Plan-Request-Parameter, die dein MotionNode abfragt
        moveit_cfg_dict["plan_request_params"] = {
            "planning_pipeline": "ompl",
            "planner_id": "",
            "planning_time": 5.0,
            "planning_attempts": 1,
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0,
        }
        # Optional: Existenzprüfungen der zentralen Configs (werden von config_hub genutzt)
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

        # Scene-Node
        scene_node = Node(
            package="spraycoater_nodes_py",
            executable="scene",
            name="scene",
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "error"],
        )
        scene_after_robot = TimerAction(period=10.0, actions=[scene_node])

        # Poses-Node
        poses_node = Node(
            package="spraycoater_nodes_py",
            executable="poses",
            name="poses",
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "error"],
        )
        poses_after_robot = TimerAction(period=10.0, actions=[poses_node])

        # Spray-Path-Node
        spray_node = Node(
            package="spraycoater_nodes_py",
            executable="spray_path",
            name="spray_path",
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "error"],
        )
        spray_after_robot = TimerAction(period=10.0, actions=[spray_node])

        # Servo-Node (Bridge UI ↔ moveit_servo)
        servo_node = Node(
            package="spraycoater_nodes_py",
            executable="servo",
            name="servo",
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "info"],
        )
        servo_after_robot = TimerAction(period=10.0, actions=[servo_node])

        # Motion-Node (MoveItPy: plan_named home/service/recipe)
        motion_node = Node(
            package="spraycoater_nodes_py",
            executable="motion",
            name="motion",
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[moveit_cfg_dict],
        )
        motion_after_robot = TimerAction(period=10.0, actions=[motion_node])

        # Robot-Node (Status/Kommandos ↔ realer Adapter / Sim)
        robot_node = Node(
            package="spraycoater_nodes_py",
            executable="robot",
            name="robot",
            output="log",
            emulate_tty=False,
            arguments=["--ros-args", "--log-level", "info"],
        )
        robot_after_robot = TimerAction(period=10.0, actions=[robot_node])

        # Rückgabe der Sequenz
        return [
            include_robot,
            scene_after_robot,
            poses_after_robot,
            spray_after_robot,
            servo_after_robot,
            motion_after_robot,
            robot_after_robot,
        ]

    return LaunchDescription([quiet_env, sim_arg, OpaqueFunction(function=_setup)])
