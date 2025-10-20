#!/usr/bin/env python3
# app/kill_all_ros.py
from __future__ import annotations
import os
import time
import subprocess
from typing import Iterable, List


def _run(cmd: List[str]) -> None:
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass


def _pkill(sig: str, patterns: Iterable[str]) -> None:
    for pat in patterns:
        _run(["pkill", f"-{sig}", "-f", pat])


def kill_all_ros(extra_patterns: Iterable[str] = ()) -> None:
    """
    Beendet gängige ROS/MoveIt/GUI-Prozesse.
    Erst sanft (TERM/INT), dann hart (KILL).
    """
    patterns = [
        # ROS2 Core & Launch
        "ros2 launch", "ros2 daemon", "ros2cli", "ros2 run",
        "ros2_control_node", "controller_manager",
        "spawner", "spawner_joint_state_broadcaster", "spawner_meca_arm_group_controller",

        # MoveIt
        "move_group", "moveit_servo", "servo_node",
        "moveit_ros", "trajectory_execution",

        # TF & Robot Description
        "static_transform_publisher", "robot_state_publisher",

        # Visualization
        "rviz2",

        # Nodes aus deinem Projekt
        "mecademic_bringup", "scene_manager", "poses_manager",
        "tool_manager", "spray_path_manager",

        # Joint Controllers
        "joint_state_broadcaster", "joint_trajectory_controller",
        "meca_arm_group_controller",

        # Gazebo/Ignition
        "gzserver", "gzclient", "ign gazebo", "ignition.gz",

        # Warehouse / MongoDB (MoveIt Memory)
        "warehouse_ros", "mongodb_server",

        # Python ROS Nodes
        "python3 .*ros", "python3 .*node",

        # DDS Kommunikationsschichten
        "rmw_fastrtps", "rmw_cyclonedds", "fastdds", "microxrcedds",

        # ROS Utility Prozesse
        "rosout", "parameter_server", "topic_monitor",

        # Debugging / Rosbridge
        "rosbridge", "websocket", "foxglove",

        # Dein Framework (Sicherheit)
        "bringup", "launch.py", "meca_", "mecademic", "workspace"
    ]
    patterns.extend(list(extra_patterns))


    print("[kill] sending TERM/INT …", flush=True)
    _pkill("TERM", patterns)
    _pkill("INT", patterns)
    time.sleep(0.4)

    print("[kill] sending KILL …", flush=True)
    _pkill("KILL", patterns)
    time.sleep(0.2)

    # ros2 daemon sicher stoppen (ignoriere Fehler)
    try:
        subprocess.run(["/bin/bash", "-lc", "ros2 daemon stop || true"], check=False)
    except Exception:
        pass

    print("[kill] done.", flush=True)


def main():
    # optional: zusätzliche Patterns aus ENV (kommagetrennt)
    extra_env = os.environ.get("KILL_EXTRA", "")
    extra = [p.strip() for p in extra_env.split(",") if p.strip()] if extra_env else []
    kill_all_ros(extra_patterns=extra)


if __name__ == "__main__":
    main()
