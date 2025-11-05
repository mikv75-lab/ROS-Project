# Spraycoater/src/ros/ros_launcher.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time
import threading
from typing import List, Optional, Dict, Tuple

import logging
from ros.kill_all_ros import kill_all_ros  # gleicher Ordner

# -----------------------------------------------------------------------------
# Modul-Status
# -----------------------------------------------------------------------------
_LOG = "/tmp/bringup.log"
_BRINGUP_PROC: Optional[subprocess.Popen] = None
_BRINGUP_THREAD: Optional[threading.Thread] = None
_STOP_STREAM = threading.Event()

# Separater Logger NUR für den roslaunch-Stream
_LAUNCH_LOG = logging.getLogger("ros.launch")

# -----------------------------------------------------------------------------
# Public Status-API
# -----------------------------------------------------------------------------
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
        _LAUNCH_LOG.warning("cannot read bringup log '%s': %s", _LOG, e)
        return
    tail = content[-lines:] if len(content) > lines else content
    for l in tail:
        _LAUNCH_LOG.info("[bringup.log] %s", l)

# -----------------------------------------------------------------------------
# Command Helper
# -----------------------------------------------------------------------------
def make_bringup_cmd(
    sim: bool,
    *,
    package: str = "spraycoater_bringup",
    launch_file: str = "bringup.launch.py",
) -> Tuple[List[str], List[str]]:
    """
    Baue den 'ros2 launch' Aufruf + einziges Launch-Argument 'sim'.
    Rückgabe:
      (cmd, extra_args)
      cmd = ["ros2","launch", package, launch_file]
      extra_args = ["sim:=true" | "sim:=false"]
    """
    cmd = ["ros2", "launch", package, launch_file]
    extra_args: List[str] = [f"sim:={'true' if sim else 'false'}"]
    return cmd, extra_args

# -----------------------------------------------------------------------------
# Internals
# -----------------------------------------------------------------------------
def _compose_launch_cmd(
    cmd: List[str],
    extra_launch_args: Optional[List[str]],
    ros_setup: Optional[str],
    ws_setup: Optional[str],
) -> List[str]:
    """
    Startet 'ros2 launch …' in gesourcter Bash-Session.
    Es werden ausschließlich die explizit übergebenen setup-Pfade gesourct (keine Fallbacks).
    """
    full_cmd = cmd + (extra_launch_args or [])
    launch = shlex.join(full_cmd)

    parts = []
    if ros_setup:
        parts.append(f'[ -f "{ros_setup}" ] && source "{ros_setup}"')
    if ws_setup:
        parts.append(f'[ -f "{ws_setup}" ] && source "{ws_setup}"')
    parts.append(f"exec {launch}")

    shell_line = " && ".join(parts)
    return ["/bin/bash", "-lc", shell_line]

def _stream_proc_to_logger(proc: subprocess.Popen) -> None:
    """Liest roslaunch stdout (stderr via STDOUT) zeilenweise und loggt nach 'ros.launch'."""
    _LAUNCH_LOG.info("=== roslaunch stream START === pid=%s", proc.pid)
    try:
        for line in iter(proc.stdout.readline, ''):
            if _STOP_STREAM.is_set():
                break
            if not line:
                break
            _LAUNCH_LOG.info(line.rstrip())
    except Exception as e:
        _LAUNCH_LOG.exception("stream error: %s", e)
    finally:
        try:
            if proc.stdout:
                proc.stdout.close()
        except Exception:
            pass
        _LAUNCH_LOG.info("=== roslaunch stream END ===")

# -----------------------------------------------------------------------------
# Public Control-API
# -----------------------------------------------------------------------------
def ensure_clean_graph_then_launch(
    cmd: List[str],
    extra_launch_args: Optional[List[str]] = None,
    log_path: Optional[str] = None,
    *,
    ros_setup: Optional[str] = None,
    ws_setup: Optional[str] = None,
    env: Optional[Dict[str, str]] = None,
) -> None:
    """
    1) existierende ROS-Prozesse killen
    2) ros2 launch starten (eigene Prozessgruppe)
    3) stdout/stderr des Launchers live nach Logger 'ros.launch' streamen
    4) optional weiterhin nach bringup_log spiegeln
    """
    global _BRINGUP_PROC, _LOG, _BRINGUP_THREAD

    if BRINGUP_RUNNING():
        _LAUNCH_LOG.info("bringup already running -> shutting it down first…")
        shutdown_bringup()

    # Harte Bereinigung des Graphen (eigener Helper)
    kill_all_ros()

    if log_path:
        _LOG = log_path

    # ENV mergen + robuste Locale + unbuffered Python
    env_merged = dict(os.environ)
    if env:
        env_merged.update(env)
    env_merged.setdefault("PYTHONUNBUFFERED", "1")
    env_merged.setdefault("LC_ALL", "C.UTF-8")
    env_merged.setdefault("LANG", "C.UTF-8")

    launch_cmd = _compose_launch_cmd(cmd, extra_launch_args, ros_setup, ws_setup)
    _LAUNCH_LOG.info(
        "starting bringup: %s",
        " ".join(cmd) + (f" {shlex.join(extra_launch_args)}" if extra_launch_args else "")
    )

    try:
        _STOP_STREAM.clear()
        _BRINGUP_PROC = subprocess.Popen(
            launch_cmd,
            stdout=subprocess.PIPE,                 # wir lesen hier ab
            stderr=subprocess.STDOUT,               # stderr -> stdout
            text=True, encoding="utf-8", errors="replace",
            bufsize=1,                              # line buffered
            preexec_fn=os.setsid,                   # eigene Prozessgruppe
            env=env_merged,
        )
    except FileNotFoundError as e:
        _LAUNCH_LOG.exception("FAILED to start bringup (FileNotFound): %s", e)
        _BRINGUP_PROC = None
        return
    except Exception as e:
        _LAUNCH_LOG.exception("FAILED to start bringup: %s", e)
        _BRINGUP_PROC = None
        return

    # Stream-Thread starten
    _BRINGUP_THREAD = threading.Thread(
        target=_stream_proc_to_logger, args=(_BRINGUP_PROC,), daemon=True
    )
    _BRINGUP_THREAD.start()

    # optional weiterhin in bringup_log mitloggen (Marker)
    try:
        with open(_LOG, "a", encoding="utf-8") as f:
            f.write(f"=== bringup started pid={_BRINGUP_PROC.pid} ===\n")
    except Exception:
        pass

    time.sleep(0.8)
    if _BRINGUP_PROC.poll() is not None:
        _LAUNCH_LOG.error("bringup exited immediately (code=%s)", _BRINGUP_PROC.returncode)
        tail_log()

def restart_bringup(
    cmd: List[str],
    extra_launch_args: Optional[List[str]] = None,
    log_path: Optional[str] = None,
    *,
    ros_setup: Optional[str] = None,
    ws_setup: Optional[str] = None,
    env: Optional[Dict[str, str]] = None,
) -> None:
    shutdown_bringup()
    ensure_clean_graph_then_launch(
        cmd,
        extra_launch_args=extra_launch_args,
        log_path=log_path,
        ros_setup=ros_setup,
        ws_setup=ws_setup,
        env=env,
    )

def shutdown_bringup(timeout: float = 5.0) -> None:
    """Bringup beenden (ganze Prozessgruppe) + Stream-Thread sauber einsammeln."""
    global _BRINGUP_PROC, _BRINGUP_THREAD
    if not BRINGUP_RUNNING():
        _BRINGUP_PROC = None
        return

    _LAUNCH_LOG.info("stopping bringup…")
    try:
        pgid = os.getpgid(_BRINGUP_PROC.pid)
        os.killpg(pgid, signal.SIGTERM)
    except Exception as e:
        _LAUNCH_LOG.warning("SIGTERM failed: %s", e)

    t0 = time.time()
    while time.time() - t0 < timeout and BRINGUP_RUNNING():
        time.sleep(0.2)

    if BRINGUP_RUNNING():
        _LAUNCH_LOG.error("bringup still alive -> SIGKILL")
        try:
            pgid = os.getpgid(_BRINGUP_PROC.pid)
            os.killpg(pgid, signal.SIGKILL)
        except Exception as e:
            _LAUNCH_LOG.error("SIGKILL failed: %s", e)

    # Stream-Thread stoppen
    _STOP_STREAM.set()
    try:
        if _BRINGUP_THREAD and _BRINGUP_THREAD.is_alive():
            _BRINGUP_THREAD.join(timeout=1.0)
    except Exception:
        pass

    # Zombie vermeiden
    try:
        if _BRINGUP_PROC and _BRINGUP_PROC.poll() is None:
            _BRINGUP_PROC.wait(timeout=1.0)
    except Exception:
        pass

    _BRINGUP_PROC = None
    _BRINGUP_THREAD = None
    _LAUNCH_LOG.info("bringup stopped.")
