# Spraycoater/src/ros/rviz_manager.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time
from typing import Optional, Dict, List
import logging


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/ros
    return os.path.abspath(os.path.join(here, "..", ".."))

def resource_path(*parts: str) -> str:
    return os.path.join(_project_root(), "resource", *parts)


class _BaseRvizManager:
    """
    Startet/stoppt genau EINE rviz2-Instanz.
    - Kein Fallback: start() erfordert eine gültige .rviz-Config
    - Stdout/err -> python-Logger 'ros.rviz.<name>'
    """
    def __init__(self, name: str):
        self._name = name
        self._proc: Optional[subprocess.Popen] = None
        self._log = logging.getLogger(f"ros.rviz.{name}")

    # ---------------- status ----------------
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def pid(self) -> Optional[int]:
        return self._proc.pid if self.is_running() else None

    # ---------------- control ----------------
    def start(self, config_path: str, extra_args: Optional[List[str]] = None, env: Optional[Dict[str, str]] = None) -> None:
        if not config_path or not os.path.exists(config_path):
            raise FileNotFoundError(f"{self._name}: RViz-Config fehlt: {config_path}")

        if self.is_running():
            self._log.info("rviz already running (pid=%s) -> restarting …", self._proc.pid)
            self.stop()

        args = ["rviz2", "-d", config_path]
        if extra_args:
            args += list(extra_args)

        env_merged = dict(os.environ)
        if env:
            env_merged.update(env)
        env_merged.setdefault("LC_ALL", "C.UTF-8")
        env_merged.setdefault("LANG", "C.UTF-8")

        self._log.info("starting rviz: %s", " ".join(shlex.quote(a) for a in args))
        try:
            self._proc = subprocess.Popen(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True, encoding="utf-8", errors="replace",
                bufsize=1,
                env=env_merged,
            )
        except FileNotFoundError as e:
            self._log.exception("rviz2 not found: %s", e)
            self._proc = None
            return
        except Exception as e:
            self._log.exception("failed to start rviz2: %s", e)
            self._proc = None
            return

        # asynchron stdout „tailen“
        # (simpel: ohne Thread-Join; Prozessende wird über stop() gehandhabt)
        def _pump(p: subprocess.Popen):
            try:
                for line in iter(p.stdout.readline, ""):
                    if not line:
                        break
                    self._log.info(line.rstrip())
            except Exception as e:
                self._log.exception("rviz stream error: %s", e)

        import threading
        t = threading.Thread(target=_pump, args=(self._proc,), daemon=True)
        t.start()

        time.sleep(0.3)
        if self._proc.poll() is not None:
            self._log.error("rviz exited immediately (code=%s)", self._proc.returncode)

    def stop(self, timeout: float = 3.0) -> None:
        if not self.is_running():
            self._proc = None
            return
        self._log.info("stopping rviz (pid=%s)…", self._proc.pid)
        try:
            self._proc.terminate()
        except Exception:
            pass

        t0 = time.time()
        while time.time() - t0 < timeout and self.is_running():
            time.sleep(0.1)

        if self.is_running():
            self._log.warning("rviz still alive -> SIGKILL")
            try:
                self._proc.kill()
            except Exception:
                pass

        # cleanup
        try:
            if self._proc and self._proc.poll() is None:
                self._proc.wait(timeout=0.5)
        except Exception:
            pass
        self._proc = None
        self._log.info("rviz stopped.")

    # convenience
    def restart(self, config_path: str, extra_args: Optional[List[str]] = None, env: Optional[Dict[str, str]] = None) -> None:
        self.stop()
        self.start(config_path, extra_args=extra_args, env=env)


# ---------------- Singletons ----------------
class LiveRvizManager(_BaseRvizManager):
    _inst: Optional["LiveRvizManager"] = None

    def __init__(self):
        super().__init__("live")

    @classmethod
    def instance(cls) -> "LiveRvizManager":
        if cls._inst is None:
            cls._inst = LiveRvizManager()
        return cls._inst


class ShadowRvizManager(_BaseRvizManager):
    _inst: Optional["ShadowRvizManager"] = None

    def __init__(self):
        super().__init__("shadow")

    @classmethod
    def instance(cls) -> "ShadowRvizManager":
        if cls._inst is None:
            cls._inst = ShadowRvizManager()
        return cls._inst
