# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import signal
import subprocess
import logging
import time
from typing import Optional, List, Dict, Tuple

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QWidget as QtWidget
from PyQt5.QtGui import QWindow
from PyQt5.QtCore import Qt

_LOG = logging.getLogger("ros.rviz")

def _ensure_env(base: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    env = dict(os.environ)
    if base:
        env.update(base)
    # robuste Defaults
    home = env.get("HOME", "/root")
    env.setdefault("HOME", home)
    env.setdefault("ROS_LOG_DIR", os.path.join(home, ".ros", "log"))
    env.setdefault("XDG_RUNTIME_DIR", "/tmp/runtime-root")
    env.setdefault("RCUTILS_LOGGING_USE_STDOUT", "1")
    env.setdefault("LC_ALL", "C.UTF-8")
    env.setdefault("LANG", "C.UTF-8")
    os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
    os.makedirs(env["XDG_RUNTIME_DIR"], exist_ok=True)
    return env

def _call(cmd: List[str]) -> Tuple[int, str]:
    try:
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True)
        return 0, out
    except subprocess.CalledProcessError as e:
        return e.returncode, e.output or ""
    except FileNotFoundError:
        return 127, ""

def _find_rviz_window_id(timeout_s: float = 5.0, poll_ms: int = 120) -> Optional[int]:
    """
    Sucht nach der Top-Level Window-ID von RViz2 via wmctrl.
    Rückgabe: X11 Window ID als int (base 16), oder None.
    """
    t0 = time.time()
    while (time.time() - t0) < timeout_s:
        code, out = _call(["wmctrl", "-lx"])
        if code == 0 and out:
            # typischer Klassenname ist "rviz2.rviz2"
            for line in out.splitlines():
                # Format: 0x04600007  0 host rviz2.rviz2  RViz2
                parts = line.split()
                if len(parts) >= 5 and ("rviz2" in parts[2] or "rviz2" in parts[3] or "rviz2" in line.lower()):
                    try:
                        wid_hex = parts[0]
                        return int(wid_hex, 16)
                    except Exception:
                        pass
        time.sleep(poll_ms / 1000.0)
    return None

def _hide_window(wid: int, hide: bool = True) -> None:
    action = "add,hidden" if hide else "remove,hidden"
    _call(["wmctrl", "-i", "-r", hex(wid), "-b", action])

class _BaseRvizManager:
    """
    Startet genau EINE rviz2-Instanz, optional mit display-config,
    kann das RViz-Fenster in ein Qt-Container-Widget einbetten (X11 Reparenting).
    """
    def __init__(self, name: str):
        self._name = name
        self._proc: Optional[subprocess.Popen] = None
        self._config_path: Optional[str] = None
        self._container: Optional[QWidget] = None
        self._embedded_widget: Optional[QtWidget] = None
        self._wid: Optional[int] = None

    # ---------- Status ----------
    def is_running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def pid(self) -> Optional[int]:
        return self._proc.pid if self._proc and self._proc.poll() is None else None

    # ---------- Public API ----------
    def start(self, *, config_path: Optional[str] = None, extra_args: Optional[List[str]] = None,
              env: Optional[Dict[str, str]] = None, silent: bool = True) -> None:
        if self.is_running():
            _LOG.info("[%s] rviz already running (pid=%s)", self._name, self._proc.pid)
            return

        self._config_path = config_path
        args = ["rviz2"]
        # Splash “vermeiden”: kein offizieller off-switch; Workaround ist,
        # erst zu embedden, wenn Main-Window da ist (hidden bis dahin).
        if config_path and os.path.exists(config_path):
            args += ["-d", config_path]
        if extra_args:
            args += list(extra_args)

        run_env = _ensure_env(env)

        _LOG.info("[%s] starting rviz: %s", self._name, " ".join(args))
        try:
            self._proc = subprocess.Popen(
                args,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,  # eigene Prozessgruppe
                env=run_env,
            )
        except FileNotFoundError:
            _LOG.error("[%s] rviz2 binary not found in PATH", self._name)
            self._proc = None
            return
        except Exception as e:
            _LOG.exception("[%s] failed to start rviz: %s", self._name, e)
            self._proc = None
            return

        # Fenster finden und ggf. verstecken, bis eingebettet wird
        wid = _find_rviz_window_id(timeout_s=6.0)
        if wid:
            self._wid = wid
            if silent:
                _hide_window(wid, True)
            # sofort einbetten, falls Container schon bekannt
            if self._container:
                self._embed_into(self._container)
                if silent:
                    _hide_window(wid, False)
        else:
            _LOG.warning("[%s] RViz window not found for embedding (will keep external).", self._name)

    def stop(self) -> None:
        if not self.is_running():
            self._cleanup_embed()
            self._proc = None
            return
        _LOG.info("[%s] stopping rviz …", self._name)
        try:
            pgid = os.getpgid(self._proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception as e:
            _LOG.warning("[%s] SIGTERM failed: %s", self._name, e)
        self._cleanup_embed()
        self._proc = None

    def restart(self, *, config_path: Optional[str] = None, extra_args: Optional[List[str]] = None,
                env: Optional[Dict[str, str]] = None, silent: bool = True) -> None:
        self.stop()
        self.start(config_path=config_path or self._config_path, extra_args=extra_args, env=env, silent=silent)

    def attach(self, container: QWidget) -> None:
        """
        Setzt/ändert das Ziel-Qt-Widget. Wenn RViz schon läuft, wird sofort eingebettet.
        """
        self._container = container
        if self._wid and self.is_running():
            self._embed_into(container)

    # ---------- Intern ----------
    def _embed_into(self, container: QWidget) -> None:
        try:
            if not self._wid:
                # Versuche, Window-ID nachträglich zu finden (z.B. wenn Start zuvor war)
                self._wid = _find_rviz_window_id(timeout_s=3.0)
                if not self._wid:
                    _LOG.warning("[%s] cannot embed: no RViz window id found.", self._name)
                    return

            # QWindow aus Window-ID erstellen und in QWidget verpacken
            qwin = QWindow.fromWinId(self._wid)
            qwin.setFlags(Qt.FramelessWindowHint | Qt.BypassWindowManagerHint)
            qwin.setOpacity(1.0)

            container.setContentsMargins(0, 0, 0, 0)
            if not isinstance(container.layout(), QVBoxLayout):
                lay = QVBoxLayout(container)
                lay.setContentsMargins(0, 0, 0, 0)
                lay.setSpacing(0)
                container.setLayout(lay)

            host = QtWidget.createWindowContainer(qwin, container)
            host.setFocusPolicy(Qt.StrongFocus)
            host.setMinimumSize(10, 10)

            # altes Embedded-Widget entfernen
            self._cleanup_embed()
            container.layout().addWidget(host)
            self._embedded_widget = host

            _LOG.info("[%s] RViz embedded into %s (wid=%s)", self._name, container.objectName(), hex(self._wid))
        except Exception as e:
            _LOG.exception("[%s] embedding failed: %s", self._name, e)

    def _cleanup_embed(self) -> None:
        if self._embedded_widget:
            try:
                self._embedded_widget.setParent(None)
                self._embedded_widget.deleteLater()
            except Exception:
                pass
        self._embedded_widget = None
        self._wid = None


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
