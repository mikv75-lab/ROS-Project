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

# Prozesse, die wir typischerweise beenden möchten
SAFE_NAME_MATCHES = {
    "rviz2",
    "gzserver",
    "gzclient",
    "ros2daemon",
}

# Kommandozeilen-Muster, die eindeutig ROS-CLIs/Launches kennzeichnen
CLI_SUBSTRINGS = [
    "ros2 launch",
    "ros2 topic echo",
    "ros2 topic pub",
    "ros2 bag play",
    "ros2 bag record",
    "-m launch_ros",
]

# Dinge, die wir NIEMALS killen wollen (UI/Debugger etc.)
EXCLUDE_SUBSTRINGS = [
    "main_gui.py",
    "debugpy",
    "Spraycoater/src/app/main_gui.py",
]

def _matches_cli(cmd: List[str]) -> bool:
    s = " ".join(cmd)
    return any(sub in s for sub in CLI_SUBSTRINGS)

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

def kill_all_ros(timeout: float = 2.0) -> None:
    """
    Beendet typische ROS-/RViz-/Gazebo-CLI-Prozesse, ohne die UI selbst zu treffen.
    Erst TERM, dann (falls nötig) KILL nach kurzer Wartezeit.
    """
    if psutil is None:
        _kill_fallback(timeout)
        return

    victims = []
    for p in psutil.process_iter(attrs=["pid", "name", "cmdline"]):
        try:
            name = (p.info.get("name") or "").lower()
            cmd  = p.info.get("cmdline") or []
            if not cmd:
                continue
            if _should_exclude(p.pid, cmd):
                continue

            if name in SAFE_NAME_MATCHES or _matches_cli(cmd):
                victims.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue

    if not victims:
        _LOG.info("[kill] nothing to stop.")
        return

    _LOG.info("[kill] stopping %d ROS-related processes…", len(victims))
    for p in victims:
        _LOG.debug(" -> TERM pid=%s name=%s cmd=%s", p.pid, p.info.get("name"), " ".join(p.info.get("cmdline") or []))
        _kill_pid(p.pid, signal.SIGTERM)

    t0 = time.time()
    while time.time() - t0 < timeout:
        all_gone = True
        for p in victims:
            try:
                if p.is_running():
                    all_gone = False
                    break
            except psutil.NoSuchProcess:
                continue
        if all_gone:
            break
        time.sleep(0.1)

    # Eskalation
    for p in victims:
        try:
            if p.is_running():
                _LOG.debug(" -> KILL pid=%s", p.pid)
                _kill_pid(p.pid, signal.SIGKILL)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass

def _kill_fallback(timeout: float) -> None:
    """
    Fallback ohne psutil: vorsichtige pkill-Aufrufe (exakte Namen + schmale -f Muster).
    """
    import subprocess
    _LOG.info("[kill] psutil nicht verfügbar – fallback pkill wird verwendet.")
    cmds = [
        ["pkill", "-x", "rviz2"],
        ["pkill", "-x", "gzserver"],
        ["pkill", "-x", "gzclient"],
        ["pkill", "-x", "ros2daemon"],
        # vorsichtige CLI-Kills (keine UI/Debug-Muster):
        ["pkill", "-f", "ros2 launch "],
        ["pkill", "-f", "ros2 topic echo"],
        ["pkill", "-f", "ros2 topic pub"],
        ["pkill", "-f", "-m launch_ros"],
    ]
    for c in cmds:
        try:
            subprocess.run(c, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception:
            pass
    time.sleep(timeout)
