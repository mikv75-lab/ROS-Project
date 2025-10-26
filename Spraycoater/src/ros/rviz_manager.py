# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import signal
import subprocess
import logging
from typing import Optional, List, Dict
from PyQt5.QtWidgets import QWidget

_LOG = logging.getLogger("ros.rviz")

class _BaseRvizManager:
    """
    Startet genau EINE rviz2-Instanz, optional mit display-config.
    'attach(container)' ist aktuell ein No-Op (Platzhalter für echtes Embedding/Reparenting).
    """
    def __init__(self, name: str):
        self._name = name
        self._proc: Optional[subprocess.Popen] = None
        self._alive = False
        self._current_container: Optional[QWidget] = None
        self._config_path: Optional[str] = None

    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, *, config_path: Optional[str] = None, extra_args: Optional[List[str]] = None,
              env: Optional[Dict[str, str]] = None) -> None:
        if self.is_running():
            _LOG.info("[%s] rviz already running (pid=%s)", self._name, self._proc.pid if self._proc else "?")
            return

        self._config_path = config_path
        args = ["rviz2"]
        if config_path and os.path.exists(config_path):
            args += ["-d", config_path]
        if extra_args:
            args += list(extra_args)

        run_env = dict(os.environ)
        if env:
            run_env.update(env)
        run_env.setdefault("LC_ALL", "C.UTF-8")
        run_env.setdefault("LANG", "C.UTF-8")

        _LOG.info("[%s] starting rviz: %s", self._name, " ".join(args))
        try:
            self._proc = subprocess.Popen(
                args,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,  # eigene Prozessgruppe
                env=run_env,
            )
            self._alive = True
        except FileNotFoundError:
            _LOG.error("[%s] rviz2 binary not found in PATH", self._name)
            self._proc = None
            self._alive = False
        except Exception as e:
            _LOG.exception("[%s] failed to start rviz: %s", self._name, e)
            self._proc = None
            self._alive = False

    def attach(self, container: QWidget) -> None:
        """
        Platzhalter: hier später echtes Embedding (librviz/QWindow reparent).
        Aktuell merken wir uns nur, wohin „gehängt“ werden soll.
        """
        self._current_container = container
        obj = container.objectName() if container else "?"
        _LOG.debug("[%s] attach requested to container=%s", self._name, obj)

    def stop(self) -> None:
        if not self.is_running():
            self._proc = None
            self._alive = False
            return
        _LOG.info("[%s] stopping rviz …", self._name)
        try:
            pgid = os.getpgid(self._proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception as e:
            _LOG.warning("[%s] SIGTERM failed: %s", self._name, e)
        # kein hartes KILL hier; rviz kann ordentlich schließen
        self._proc = None
        self._alive = False


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
