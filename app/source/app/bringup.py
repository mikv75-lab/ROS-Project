#!/usr/bin/env python3
# app/bringup.py
from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time
from typing import List, Optional

# interner Status
_BRINGUP_PROC: Optional[subprocess.Popen] = None
_LOG = "/tmp/bringup.log"


# ---------------------------- utils ----------------------------

def _log(msg: str) -> None:
    print(f"[bringup] {msg}", flush=True)


def _run(cmd: List[str], check: bool = False, timeout: Optional[float] = None):
    try:
        return subprocess.run(cmd, check=check, timeout=timeout)
    except Exception as e:
        _log(f"run error: {cmd} -> {e}")
        return None


def _bash(cmd: str, check: bool = False, timeout: Optional[float] = None):
    """Convenience-Wrapper: bash -lc '<cmd>' (für Sourcing etc.)."""
    return _run(["/bin/bash", "-lc", cmd], check=check, timeout=timeout)


# ---------------------------- public status api ----------------------------

def BRINGUP_RUNNING() -> bool:
    return _BRINGUP_PROC is not None and _BRINGUP_PROC.poll() is None


def bringup_pid() -> Optional[int]:
    return _BRINGUP_PROC.pid if BRINGUP_RUNNING() else None


def bringup_log_path() -> str:
    return _LOG


def tail_log(lines: int = 80) -> None:
    try:
        with open(_LOG, "r", encoding="utf-8", errors="ignore") as f:
            content = f.read().splitlines()
    except Exception as e:
        _log(f"cannot read log {_LOG}: {e}")
        return
    tail = content[-lines:] if len(content) > lines else content
    print("\n".join(f"[bringup.log] {l}" for l in tail), flush=True)


# ---------------------------- kill helpers ----------------------------

def kill_ros_graph() -> None:
    """Alle typischen ROS-Prozesse hart beenden (clean graph)."""
    _log("killing existing ROS graph…")

    # ros2 daemon soft stop
    _bash("ros2 daemon stop || true")

    # bekannte Prozesse (je nach Setup ggf. ergänzen)
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
    ]

    # graceful versuch
    for p in patterns:
        _bash(f"pkill -TERM -f '{p}' || true")
    time.sleep(0.4)

    # hart
    for p in patterns:
        _bash(f"pkill -9 -f '{p}' || true")
    time.sleep(0.2)


# ---------------------------- launch helpers ----------------------------

def _compose_launch_cmd(
    cmd: List[str],
    extra_launch_args: Optional[List[str]],
    ros_setup: str,
    ws_setup: str,
) -> List[str]:
    """
    Startet 'ros2 launch …' in gesourcter Bash-Session.
    Beispiel cmd: ["ros2","launch","mecademic_bringup","bringup.launch.py"]
    """
    full_cmd = cmd + (extra_launch_args or [])
    launch = shlex.join(full_cmd)  # sauberes Quoting

    parts = []
    if ros_setup:
        parts.append(f'[ -f "{ros_setup}" ] && source "{ros_setup}" || true')
    if ws_setup:
        parts.append(f'[ -f "{ws_setup}" ] && source "{ws_setup}" || true')
    parts.append(f"exec {launch}")
    full = " && ".join(parts)
    return ["/bin/bash", "-lc", full]


def ensure_clean_graph_then_launch(
    cmd: List[str],
    extra_launch_args: Optional[List[str]] = None,
    log_path: Optional[str] = None,
    *,
    ros_setup: Optional[str] = None,
    ws_setup: Optional[str] = None,
) -> None:
    """
    1) existierende ROS-Prozesse killen
    2) ros2 launch starten (eigene Prozessgruppe)
    3) Logs nach {log_path} schreiben (Default: /tmp/bringup.log)
    """
    global _BRINGUP_PROC, _LOG

    if BRINGUP_RUNNING():
        _log("bringup already running -> shutting it down first…")
        shutdown_bringup()

    kill_ros_graph()

    if log_path:
        _LOG = log_path

    # Setups aus ENV übersteuerbar, sonst Defaults
    ros_setup = ros_setup or os.environ.get("ROS_SETUP", "/opt/ros/rolling/setup.bash")
    ws_setup = ws_setup or os.environ.get("WS_SETUP", "/root/ws_moveit/install/setup.bash")

    launch_cmd = _compose_launch_cmd(cmd, extra_launch_args, ros_setup, ws_setup)
    _log(
        f"starting bringup: {' '.join(cmd)}"
        + (f" args={extra_launch_args}" if extra_launch_args else "")
    )

    try:
        # Logfile neu anlegen
        with open(_LOG, "w", encoding="utf-8") as f:
            f.write("=== bringup start ===\n")

        # Prozessgruppe starten, damit wir später PGID killen können
        _BRINGUP_PROC = subprocess.Popen(
            launch_cmd,
            stdout=open(_LOG, "a", buffering=1),
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,  # eigene Prozessgruppe
        )
    except FileNotFoundError as e:
        _log(f"FAILED to start bringup (FileNotFound): {e}")
        tail_log()
        _BRINGUP_PROC = None
        return
    except Exception as e:
        _log(f"FAILED to start bringup: {e}")
        tail_log()
        _BRINGUP_PROC = None
        return

    # kurz warten und Status prüfen
    time.sleep(1.0)
    if not BRINGUP_RUNNING():
        _log("bringup exited immediately – log tail follows:")
        tail_log()
        _BRINGUP_PROC = None
    else:
        _log(f"bringup pid={_BRINGUP_PROC.pid} running. (log: {_LOG})")


def restart_bringup(
    cmd: List[str],
    extra_launch_args: Optional[List[str]] = None,
    log_path: Optional[str] = None,
    *,
    ros_setup: Optional[str] = None,
    ws_setup: Optional[str] = None,
) -> None:
    shutdown_bringup()
    ensure_clean_graph_then_launch(
        cmd,
        extra_launch_args=extra_launch_args,
        log_path=log_path,
        ros_setup=ros_setup,
        ws_setup=ws_setup,
    )


def shutdown_bringup(timeout: float = 5.0) -> None:
    """Bringup beenden (ganze Prozessgruppe)."""
    global _BRINGUP_PROC
    if not BRINGUP_RUNNING():
        _log("bringup not running.")
        _BRINGUP_PROC = None
        return

    _log("stopping bringup…")
    try:
        pgid = os.getpgid(_BRINGUP_PROC.pid)
        os.killpg(pgid, signal.SIGTERM)
    except Exception as e:
        _log(f"SIGTERM failed: {e}")

    # warten
    t0 = time.time()
    while time.time() - t0 < timeout:
        if _BRINGUP_PROC.poll() is not None:
            break
        time.sleep(0.2)

    if BRINGUP_RUNNING():
        _log("bringup still alive -> SIGKILL")
        try:
            pgid = os.getpgid(_BRINGUP_PROC.pid)
            os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            _log(f"SIGKILL failed: {e}")

    _BRINGUP_PROC = None
    _log("bringup stopped.")
