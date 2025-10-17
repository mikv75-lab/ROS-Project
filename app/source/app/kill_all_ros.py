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
        "ros2 launch",
        "ros2 daemon",
        "move_group",
        "ros2_control_node",
        "joint_state_broadcaster",
        "joint_trajectory_controller",
        "robot_state_publisher",
        "static_transform_publisher",
        "rviz2",
        "servo_node",
        "warehouse_ros", "mongodb_server",
        "gzserver", "gzclient", "ign gazebo",
        "spawn_platform", "spawn_mesh",
        "tool_manager", "poses_manager", "workpiece_manager",
        "mecademic_bringup",
        "rosout",
        # häufige Python-Nodes
        "python3 .* ros/bridge/ui_bridge.py",
        "python3 .* ros_service.py",
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
