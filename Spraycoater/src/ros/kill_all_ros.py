# Spraycoater/src/ros/kill_all_ros.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import signal
import time
import logging
from typing import List

_LOG = logging.getLogger("ros.kill_all_ros")

try:
    import psutil  # type: ignore
except Exception:
    psutil = None


# ROS / Middleware / Simulation Prozesse (als cmdline-match)
ROS_NAME_MATCHES = {
    "rviz2",
    "move_group",
    "gzserver",
    "gzclient",
    "controller_manager",
    "ros2_control_node",
    "robot_state_publisher",
    "static_transform_publisher",
    "spawner",
}

# Deine Nodes (Python) – cmdline-fragments
PYTHON_NODE_MATCHES = [
    "spraycoater_nodes_py",
    "scene --ros-args",
    "poses --ros-args",
    "spray_path --ros-args",
    "robot --ros-args",
    "servo --ros-args",
    "motion --ros-args",
    "omron_tcp_bridge",
]

# ROS-CLI Muster
CLI_MATCHES = [
    "ros2 launch",
    "ros2 run",
    "ros2 node",
    "ros2 topic",
    "ros2 service",
    "ros2 bag",
]

# NIEMALS killen
EXCLUDE_SUBSTRINGS = [
    "main_gui.py",
    "Spraycoater/src/app/main_gui.py",
    "debugpy",
]


def _should_exclude(pid: int, cmd: List[str]) -> bool:
    if pid == os.getpid():
        return True
    s = " ".join(cmd)
    return any(ex in s for ex in EXCLUDE_SUBSTRINGS)


def _matches_ros(cmd: List[str], name: str) -> bool:
    """
    WICHTIG:
      - p.info['name'] ist oft nur 'python3'
      - deshalb MUSS auf cmdline gematcht werden.
    """
    s = " ".join(cmd).lower()
    n = (name or "").lower()

    # ros2 daemon (python process)
    if "ros2cli.daemon.daemonize" in s or "--name ros2-daemon" in s:
        return True

    # ros2 tools / launch
    if any(sub.lower() in s for sub in CLI_MATCHES):
        return True

    # bekannte binaries / libs in cmdline
    if any(x in s for x in (
        "/rviz2",
        " rviz2 ",
        "moveit_ros_move_group",
        " move_group",
        "ros2_control_node",
        "controller_manager",
        "spawner",
        "robot_state_publisher",
        "static_transform_publisher",
        "joint_state_broadcaster",
        "gzserver",
        "gzclient",
        "ign gazebo",
        "gz sim",
    )):
        return True

    # deine python nodes
    if any(sub.lower() in s for sub in PYTHON_NODE_MATCHES):
        return True

    # fallback: process-name match (selten hilfreich, aber ok)
    if n in (x.lower() for x in ROS_NAME_MATCHES):
        return True

    return False


def _kill_pid(pid: int, sig=signal.SIGTERM) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass
    except Exception as e:
        _LOG.debug("kill(%s, %s) failed: %s", pid, sig, e)


def kill_all_ros(timeout: float = 2.0, *, restart_daemon: bool = True) -> None:
    """
    Killt *ALLE* ROS-relevanten Prozesse außer deiner UI.
    Zusätzlich:
      - stoppt ros2 daemon vor dem Kill
      - startet ros2 daemon optional wieder (restart_daemon=True)
    """
    import subprocess

    # ros2 daemon stoppen (nimmt oft "hängende" CLI-States weg)
    try:
        subprocess.run(
            ["ros2", "daemon", "stop"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    if psutil is None:
        _kill_fallback(timeout, restart_daemon=restart_daemon)
        return

    victims = []

    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            cmd = p.info.get("cmdline") or []
            name = (p.info.get("name") or "")
            if not cmd:
                continue

            # UI nicht killen
            if _should_exclude(p.pid, cmd):
                continue

            # ROS-Prozess?
            if _matches_ros(cmd, name):
                victims.append(p)

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    if not victims:
        _LOG.info("[kill] nothing to stop.")
        # daemon optional wieder starten
        if restart_daemon:
            try:
                subprocess.run(
                    ["ros2", "daemon", "start"],
                    check=False,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                pass
        return

    _LOG.info("[kill] stopping %d ROS-related processes…", len(victims))

    # TERM
    for p in victims:
        try:
            cmdline = " ".join(p.info.get("cmdline") or [])
        except Exception:
            cmdline = ""
        _LOG.debug(" -> TERM pid=%s: %s", p.pid, cmdline)
        _kill_pid(p.pid, signal.SIGTERM)

    # warten bis weg
    t0 = time.time()
    while time.time() - t0 < timeout:
        alive = []
        for p in victims:
            try:
                if p.is_running():
                    alive.append(p)
            except Exception:
                pass
        if not alive:
            break
        time.sleep(0.1)

    # KILL falls nötig
    for p in victims:
        try:
            if p.is_running():
                _LOG.debug(" -> KILL pid=%s", p.pid)
                _kill_pid(p.pid, signal.SIGKILL)
        except Exception:
            pass

    # daemon optional wieder starten
    if restart_daemon:
        try:
            subprocess.run(
                ["ros2", "daemon", "start"],
                check=False,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except Exception:
            pass


def _kill_fallback(timeout: float, *, restart_daemon: bool) -> None:
    """
    Ohne psutil – vorsichtige pkill-Muster.
    """
    import subprocess

    # ros2 daemon stoppen
    try:
        subprocess.run(["ros2", "daemon", "stop"], check=False)
    except Exception:
        pass

    patterns = [
        "ros2cli.daemon.daemonize",
        "--name ros2-daemon",
        "rviz2",
        "move_group",
        "ros2_control_node",
        "controller_manager",
        "spawner",
        "robot_state_publisher",
        "static_transform_publisher",
        "ros2 launch",
        "ros2 run",
        "spraycoater_nodes_py",
        "scene --ros-args",
        "poses --ros-args",
        "spray_path --ros-args",
        "robot --ros-args",
        "servo --ros-args",
        "motion --ros-args",
    ]

    for pat in patterns:
        try:
            subprocess.run(["pkill", "-f", pat], check=False)
        except Exception:
            pass

    time.sleep(timeout)

    if restart_daemon:
        try:
            subprocess.run(["ros2", "daemon", "start"], check=False)
        except Exception:
            pass
