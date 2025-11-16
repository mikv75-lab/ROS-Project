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


# Alles was ROS ist
ROS_NAME_MATCHES = {
    "rviz2",
    "move_group",
    "ros2daemon",
    "gzserver",
    "gzclient",
    "controller_manager",
}

# Deine Nodes (Python)
PYTHON_NODE_MATCHES = [
    "spraycoater_nodes_py",
    "scene --ros-args",
    "poses --ros-args",
    "spray_path --ros-args",
    "robot --ros-args",
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
    "debugpy",
    "Spraycoater/src/app/main_gui.py",
]


def _should_exclude(pid: int, cmd: List[str]) -> bool:
    if pid == os.getpid():
        return True
    s = " ".join(cmd)
    return any(ex in s for ex in EXCLUDE_SUBSTRINGS)


def _matches_ros(cmd: List[str], name: str) -> bool:
    s = " ".join(cmd)
    if name.lower() in ROS_NAME_MATCHES:
        return True
    if any(sub in s for sub in CLI_MATCHES):
        return True
    if any(sub in s for sub in PYTHON_NODE_MATCHES):
        return True
    return False


def _kill_pid(pid: int, sig=signal.SIGTERM) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass
    except Exception as e:
        _LOG.debug(f"kill({pid}, {sig}) failed: {e}")


def kill_all_ros(timeout: float = 2.0) -> None:
    """
    Killt *ALLE* ROS-relevanten Prozesse außer deiner UI.
    """
    if psutil is None:
        _kill_fallback(timeout)
        return

    victims = []

    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            cmd = p.info.get("cmdline") or []
            name = (p.info.get("name") or "").lower()
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
        return

    _LOG.info(f"[kill] stopping {len(victims)} ROS-related processes…")

    # TERM
    for p in victims:
        _LOG.debug(f" -> TERM pid={p.pid}: {' '.join(p.info.get('cmdline') or [])}")
        _kill_pid(p.pid, signal.SIGTERM)

    # Kurz warten
    t0 = time.time()
    while time.time() - t0 < timeout:
        alive = [p for p in victims if p.is_running()]
        if not alive:
            break
        time.sleep(0.1)

    # KILL falls nötig
    for p in victims:
        if p.is_running():
            _LOG.debug(f" -> KILL pid={p.pid}")
            _kill_pid(p.pid, signal.SIGKILL)


def _kill_fallback(timeout: float) -> None:
    """
    Ohne psutil – vorsichtige pkill-Muster.
    """
    import subprocess

    patterns = [
        "rviz2",
        "move_group",
        "ros2 launch",
        "spraycoater_nodes_py",
        "scene --ros-args",
        "poses --ros-args",
        "spray_path --ros-args",
        "robot --ros-args",
        "controller_manager",
    ]

    for p in patterns:
        try:
            subprocess.run(["pkill", "-f", p], check=False)
        except Exception:
            pass

    time.sleep(timeout)
