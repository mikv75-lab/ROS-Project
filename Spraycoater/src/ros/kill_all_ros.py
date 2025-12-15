# Spraycoater/src/ros/kill_all_ros.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import signal
import time
import logging
import subprocess
from typing import List, Optional, Set

_LOG = logging.getLogger("ros.kill_all_ros")

try:
    import psutil  # type: ignore
except Exception:
    psutil = None


# ---------------------------------------------------------------------------
# Match-Regeln
# ---------------------------------------------------------------------------

# Prozesse, die wir definitiv als ROS/Bringup ansehen (per cmdline substring)
KILL_CMD_SUBSTRINGS = [
    # ROS launch / run / cli (nur die, die dauerhaft laufen)
    "ros2 launch",
    "ros2 run",

    # ros2 daemon
    "ros2cli.daemon.daemonize",
    "--name ros2-daemon",

    # Core runtime / nodes
    "/lib/rviz2/rviz2",
    " rviz2 ",
    "/moveit_ros_move_group/move_group",
    " move_group ",
    "/controller_manager/ros2_control_node",
    "controller_manager",
    "ros2_control_node",

    # Gazebo (falls jemals)
    "gzserver",
    "gzclient",
]

# Deine Nodes (Python) – optional, aber sinnvoll
PYTHON_NODE_MATCHES = [
    "spraycoater_nodes_py",
    " scene --ros-args",
    " poses --ros-args",
    " spray_path --ros-args",
    " robot --ros-args",
    " servo --ros-args",
    " motion --ros-args",
]

# NIEMALS killen (UI / Debugger)
EXCLUDE_SUBSTRINGS = [
    "main_gui.py",
    "Spraycoater/src/app/main_gui.py",
    "debugpy",
]


def _cmdline_str(cmd: List[str]) -> str:
    return " ".join(cmd or []).strip()


def _should_exclude(pid: int, cmd: List[str]) -> bool:
    if pid == os.getpid():
        return True
    s = _cmdline_str(cmd)
    return any(ex in s for ex in EXCLUDE_SUBSTRINGS)


def _matches_any(s: str, patterns: List[str]) -> bool:
    return any(p in s for p in patterns)


def _is_ros_related(cmd: List[str]) -> bool:
    s = f" {_cmdline_str(cmd)} "  # spaces helfen für " word " matches
    if _matches_any(s, KILL_CMD_SUBSTRINGS):
        return True
    if _matches_any(s, PYTHON_NODE_MATCHES):
        return True
    return False


def _kill_pid(pid: int, sig: int) -> None:
    try:
        os.kill(pid, sig)
    except ProcessLookupError:
        return
    except Exception as e:
        _LOG.debug("kill(%s, %s) failed: %s", pid, sig, e)


def _collect_victims() -> List["psutil.Process"]:
    assert psutil is not None
    victims: List[psutil.Process] = []

    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            cmd = p.info.get("cmdline") or []
            if not cmd:
                continue

            if _should_exclude(p.pid, cmd):
                continue

            if _is_ros_related(cmd):
                victims.append(p)

        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    return victims


def _expand_with_children(victims: List["psutil.Process"]) -> List["psutil.Process"]:
    """Alle Kinder der Victims mit aufnehmen (und später zuerst killen)."""
    assert psutil is not None
    seen: Set[int] = set()
    expanded: List[psutil.Process] = []

    def add_proc(proc: psutil.Process) -> None:
        if proc.pid in seen:
            return
        seen.add(proc.pid)
        expanded.append(proc)

    # erst victims aufnehmen
    for v in victims:
        add_proc(v)
        try:
            for c in v.children(recursive=True):
                add_proc(c)
        except Exception:
            pass

    # Kill-Reihenfolge: Kinder zuerst → sort by depth (heuristisch: längere cmdline / später hinzugefügt)
    # Einfach: zuerst alle Prozesse, die nicht parent von anderen sind.
    # Robust genug: reverse by parent chain length
    def depth(proc: psutil.Process) -> int:
        d = 0
        try:
            cur = proc
            while True:
                cur = cur.parent()
                if cur is None:
                    break
                d += 1
        except Exception:
            pass
        return d

    expanded.sort(key=depth, reverse=True)
    return expanded


def _wait_gone(procs: List["psutil.Process"], timeout: float) -> None:
    t0 = time.time()
    while time.time() - t0 < timeout:
        alive = []
        for p in procs:
            try:
                if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                    alive.append(p)
            except Exception:
                continue
        if not alive:
            return
        time.sleep(0.1)


def _restart_ros2_daemon() -> None:
    # ros2 daemon stop/start ist am saubersten
    try:
        subprocess.run(["ros2", "daemon", "stop"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.2)
        subprocess.run(["ros2", "daemon", "start"], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        _LOG.debug("ros2 daemon restart failed: %s", e)


def kill_all_ros(timeout_term: float = 2.0, timeout_kill: float = 1.5, restart_daemon: bool = True) -> None:
    """
    Killt *ALLE* ROS-relevanten Prozesse außer deiner UI.

    - TERM → warten → KILL
    - Kinder zuerst (ros2 launch etc.)
    - optional: ros2 daemon restart
    """
    if psutil is None:
        _kill_fallback(timeout_term, restart_daemon=restart_daemon)
        return

    victims = _collect_victims()
    if not victims:
        _LOG.info("[kill] nothing to stop.")
        if restart_daemon:
            _restart_ros2_daemon()
        return

    victims = _expand_with_children(victims)

    _LOG.info("[kill] stopping %d ROS-related processes…", len(victims))
    for p in victims:
        try:
            _LOG.debug(" -> TERM pid=%s: %s", p.pid, _cmdline_str(p.info.get("cmdline") or []))
        except Exception:
            pass
        _kill_pid(p.pid, signal.SIGTERM)

    _wait_gone(victims, timeout_term)

    # KILL remaining
    remaining: List[psutil.Process] = []
    for p in victims:
        try:
            if p.is_running() and p.status() != psutil.STATUS_ZOMBIE:
                remaining.append(p)
        except Exception:
            pass

    if remaining:
        _LOG.info("[kill] forcing %d stubborn processes (SIGKILL)…", len(remaining))
        for p in remaining:
            try:
                _LOG.debug(" -> KILL pid=%s: %s", p.pid, _cmdline_str(p.info.get("cmdline") or []))
            except Exception:
                pass
            _kill_pid(p.pid, signal.SIGKILL)

        _wait_gone(remaining, timeout_kill)

    # ros2 daemon restart (wichtig, damit ros2 cli nicht “verwirrt” ist)
    if restart_daemon:
        _restart_ros2_daemon()


def _kill_fallback(timeout: float, restart_daemon: bool = True) -> None:
    """
    Ohne psutil – grobe pkill patterns.
    """
    patterns = [
        "ros2 launch",
        "ros2 run",
        "ros2cli.daemon.daemonize",
        "ros2-daemon",
        "rviz2",
        "move_group",
        "controller_manager",
        "ros2_control_node",
        "gzserver",
        "gzclient",
        "spraycoater_nodes_py",
        " scene --ros-args",
        " poses --ros-args",
        " spray_path --ros-args",
        " robot --ros-args",
        " servo --ros-args",
        " motion --ros-args",
    ]

    for pat in patterns:
        try:
            subprocess.run(["pkill", "-TERM", "-f", pat], check=False)
        except Exception:
            pass

    time.sleep(timeout)

    for pat in patterns:
        try:
            subprocess.run(["pkill", "-KILL", "-f", pat], check=False)
        except Exception:
            pass

    if restart_daemon:
        _restart_ros2_daemon()
