# Spraycoater/src/ros/kill_all_ros.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import signal
import time
import logging
from typing import List
from pathlib import Path

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
    # MoveIt controller mgr node-names (können in python / moveitpy auftauchen)
    "moveit_simple_controller_manager",
}

# Deine Nodes (Python) – cmdline-fragments
# WICHTIG: deine install-binaries enthalten oft NICHT "scene --ros-args", sondern nur den install-path.
PYTHON_NODE_MATCHES = [
    # Paket / Install-Pfad-Matches (DAS ist bei dir entscheidend!)
    "install/spraycoater_nodes_py/lib/",
    "/root/ws_moveit/install/spraycoater_nodes_py/lib/",
    "/ws_moveit/install/spraycoater_nodes_py/lib/",
    "lib/spraycoater_nodes_py",

    # falls doch in cmdline enthalten:
    "spraycoater_nodes_py",
    "scene --ros-args",
    "poses --ros-args",
    "spray_path --ros-args",
    "robot --ros-args",
    "servo --ros-args",
    "motion --ros-args",

    # Real-Bridge
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


def _kill_pid(pid: int, sig=signal.SIGTERM) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        pass
    except Exception as e:
        _LOG.debug("kill(%s, %s) failed: %s", pid, sig, e)


def _cleanup_fastdds_shm() -> None:
    """
    Entspricht:
      rm -f /dev/shm/{fastdds*,FastDDS*,fastrtps*,FastRTPS*,sem.fastdds*,sem.fastrtps*} 2>/dev/null || true
    """
    shm = Path("/dev/shm")
    if not shm.exists():
        return

    patterns = [
        "fastdds*",
        "FastDDS*",
        "fastrtps*",
        "FastRTPS*",
        "sem.fastdds*",
        "sem.fastrtps*",
    ]

    removed = 0
    for pat in patterns:
        for p in shm.glob(pat):
            try:
                p.unlink(missing_ok=True)
                removed += 1
            except Exception:
                pass

    if removed:
        _LOG.info("[kill] cleaned %d FastDDS/RTPS SHM files in /dev/shm", removed)


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
        # MoveIt controller manager node can appear in args/params
        "moveit_simple_controller_manager",
        "simple_controller_manager",
        # MoveItPy internal nodes (falls sichtbar)
        "moveit.py.cpp_initializer",
        "moveit_py",
    )):
        return True

    # deine python nodes (inkl. install-path)
    if any(sub.lower() in s for sub in PYTHON_NODE_MATCHES):
        return True

    # fallback: process-name match
    if n in (x.lower() for x in ROS_NAME_MATCHES):
        return True

    return False


def kill_all_ros(timeout: float = 3.0, *, restart_daemon: bool = True) -> None:
    """
    Killt *ALLE* ROS-relevanten Prozesse außer deiner UI.
    Zusätzlich:
      - stoppt ros2 daemon VOR dem Kill (verhindert stale graph)
      - räumt FastDDS/RTPS SHM Reste in /dev/shm weg
      - startet ros2 daemon optional wieder
    """
    import subprocess

    # 1) ros2 daemon stoppen (WICHTIG: sonst siehst du stale nodes)
    try:
        subprocess.run(
            ["ros2", "daemon", "stop"],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except Exception:
        pass

    # 2) SHM cleanup (Locks/Files)
    _cleanup_fastdds_shm()

    if psutil is None:
        _kill_fallback(timeout, restart_daemon=restart_daemon)
        return

    victims: list[psutil.Process] = []

    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            cmd = p.info.get("cmdline") or []
            name = (p.info.get("name") or "")
            if not cmd:
                continue

            # UI nicht killen
            if _should_exclude(p.pid, cmd):
                continue

            if _matches_ros(cmd, name):
                victims.append(p)

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    if not victims:
        _LOG.info("[kill] nothing to stop.")
        # SHM cleanup nochmal (harmlos)
        _cleanup_fastdds_shm()

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

    # 3) TERM
    for p in victims:
        try:
            cmdline = " ".join(p.info.get("cmdline") or [])
        except Exception:
            cmdline = ""
        _LOG.debug(" -> TERM pid=%s: %s", p.pid, cmdline)
        _kill_pid(p.pid, signal.SIGTERM)

    # 4) warten bis weg
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

    # 5) KILL falls nötig
    for p in victims:
        try:
            if p.is_running():
                _LOG.debug(" -> KILL pid=%s", p.pid)
                _kill_pid(p.pid, signal.SIGKILL)
        except Exception:
            pass

    # 6) SHM cleanup nochmal (nach Kill!)
    _cleanup_fastdds_shm()

    # 7) daemon optional wieder starten
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

    # daemon stop + shm cleanup
    try:
        subprocess.run(["ros2", "daemon", "stop"], check=False)
    except Exception:
        pass
    _cleanup_fastdds_shm()

    patterns = [
        "ros2cli.daemon.daemonize",
        "--name ros2-daemon",
        "rviz2",
        "move_group",
        "moveit_simple_controller_manager",
        "ros2_control_node",
        "controller_manager",
        "spawner",
        "robot_state_publisher",
        "static_transform_publisher",
        "ros2 launch",
        "ros2 run",
        "spraycoater_nodes_py",
        "/root/ws_moveit/install/spraycoater_nodes_py/lib/",
        "/ws_moveit/install/spraycoater_nodes_py/lib/",
        "install/spraycoater_nodes_py/lib/",
        "omron_tcp_bridge",
    ]

    for pat in patterns:
        try:
            subprocess.run(["pkill", "-f", pat], check=False)
        except Exception:
            pass

    time.sleep(timeout)

    _cleanup_fastdds_shm()

    if restart_daemon:
        try:
            subprocess.run(["ros2", "daemon", "start"], check=False)
        except Exception:
            pass


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s:%(name)s:%(message)s")
    kill_all_ros()
