#!/usr/bin/env python3
# app/kill_all_ros.py
from __future__ import annotations
import os
import sys
import time
import shutil
import subprocess
from typing import Iterable, List

# --- Terminal sofort leeren ---
def term_clear() -> None:
    try:
        if sys.stdout.isatty():
            # bevorzuge echtes 'clear', sonst ANSI-Fallback
            if shutil.which("clear"):
                subprocess.run(["clear"], check=False)
            else:
                # ANSI: Screen löschen + Cursor Home
                print("\033[2J\033[H", end="", flush=True)
    except Exception:
        # notfalls ignorieren
        pass

# Hilfsfunktionen
def _run(cmd: List[str]) -> None:
    try:
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass

def _sh(cmd: str) -> None:
    try:
        subprocess.run(["/bin/bash", "-lc", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
    except Exception:
        pass

def _pkill(sig: str, patterns: Iterable[str]) -> None:
    for pat in patterns:
        if not pat:
            continue
        _run(["pkill", f"-{sig}", "-f", pat])

def _sleep(sec: float) -> None:
    try:
        time.sleep(sec)
    except Exception:
        pass

def kill_all_ros(extra_patterns: Iterable[str] = ()) -> None:
    """
    Beendet gängige ROS/MoveIt/GUI/CLI-Prozesse.
    Erst sanft (TERM/INT), dann hart (KILL).
    Stoppt & leert außerdem den ros2-daemon-Cache.
    """
    # 1) harte Übeltäter: CLI-Spammer & Launch
    cli_patterns = [
        r"ros2 topic echo", r"ros2 topic pub", r"ros2 bag.*record",
        r"python3 .*ros2cli", r"python3 -m ros2cli",
        r"python3 .*launch",  r"python3 -m launch",
        r"launch_ros", r"ros2 launch",
    ]

    # 2) typische Nodes (ergänzt)
    node_patterns = [
        # dein Projekt
        r"\bmecademic_bringup\b", r"\bservo\b", r"\bservo_node\b", r"moveit_servo",
        r"\bscene_manager\b", r"\bposes_manager\b", r"\btool_manager\b", r"\bspray_path_manager\b",
        r"\bmeca_arm_group_controller\b", r"spawner_meca_arm_group_controller",
        r"spawner", r"spawner_joint_state_broadcaster",

        # ROS2 core & control
        r"ros2_control_node", r"\bcontroller_manager\b", r"ros2 daemon", r"ros2cli",
        r"\brobot_state_publisher\b", r"\bstatic_transform_publisher\b",
        r"\bjoint_state_broadcaster\b", r"\bjoint_trajectory_controller\b",

        # MoveIt
        r"\bmove_group\b", r"trajectory_execution", r"moveit_ros",

        # Visualisierung
        r"\brviz2\b", r"\brqt\b",

        # DDS libs (falls Prozesse separat auftauchen)
        r"rmw_fastrtps", r"rmw_cyclonedds", r"fastdds", r"microxrcedds",

        # Gazebo/Ignition
        r"\bgzserver\b", r"\bgzclient\b", r"ign gazebo", r"ignition.gz",
    ]

    node_patterns.extend(list(extra_patterns))

    print("[kill] killing ROS2 CLI spam (echo/pub/launch) …", flush=True)
    _pkill("TERM", cli_patterns)
    _pkill("INT",  cli_patterns)
    _sleep(0.3)
    _pkill("KILL", cli_patterns)

    print("[kill] killing nodes …", flush=True)
    _pkill("TERM", node_patterns)
    _pkill("INT",  node_patterns)
    _sleep(0.6)
    _pkill("KILL", node_patterns)
    _sleep(0.2)

    # manchmal hängen die Binaries noch unter ihrem Namen:
    for binname in ["rviz2", "servo", "servo_node", "move_group",
                    "ros2_control_node", "controller_manager"]:
        _run(["killall", "-q", "-9", binname])

    # 3) ros2 daemon stoppen & Cache leeren (verhindert Ghost-Topics)
    print("[kill] stopping ros2 daemon + clearing cache …", flush=True)
    _sh("ros2 daemon stop || true")
    _sleep(0.2)
    # CLI-Cache (versch. Homes berücksichtigen)
    for path in ["~/.ros/ros2cli", "/root/.ros/ros2cli"]:
        _sh(f"rm -rf {path}")

    # 4) optional: Logs aufräumen (rein kosmetisch)
    for path in ["~/.ros/log", "/root/.ros/log"]:
        _sh(f"find {path} -maxdepth 1 -type d -mmin +5 -print0 2>/dev/null | xargs -0r rm -rf")

    print("[kill] done.", flush=True)

def show_live_graph() -> None:
    print("\n[verify] LIVE Sicht (ohne Daemon-Cache):", flush=True)
    try:
        subprocess.run(["ros2", "node", "list", "--no-daemon"], check=False)
        subprocess.run(["ros2", "topic", "list", "--no-daemon"], check=False)
    except Exception:
        pass

def main():
    term_clear()  # <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

    # optionale Extra-Patterns aus ENV (Komma-getrennt)
    extra_env = os.environ.get("KILL_EXTRA", "")
    extra = [p.strip() for p in extra_env.split(",") if p.strip()] if extra_env else []

    kill_all_ros(extra_patterns=extra)
    _sleep(0.3)
    show_live_graph()

if __name__ == "__main__":
    main()
