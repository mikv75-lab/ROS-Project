# -*- coding: utf-8 -*-
import os
import time
import signal
import subprocess
from typing import Optional, Sequence

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QWindow
from PyQt6.QtWidgets import QWidget, QVBoxLayout

import logging

# X11: externes Fenster via PID finden
try:
    from Xlib import display, X
except Exception as e:
    display = None
    _xlib_import_error = e
else:
    _xlib_import_error = None


class EmbeddedRviz:
    """
    Startet 'rviz2 -d <config>' als Subprozess und embeded das native Fenster
    in ein Qt-Container-Widget. Kann später per .reparent() in einen anderen
    Container umgehängt werden (selbe RViz-Instanz).
    """

    def __init__(self, container: QWidget, config_path: str,
                 extra_args: Optional[Sequence[str]] = None):
        self.log = logging.getLogger("app.rviz.embed")
        self.container = container
        self.config_path = os.path.abspath(config_path)
        self.extra_args = list(extra_args or [])
        self._proc: Optional[subprocess.Popen] = None
        self._qwindow: Optional[QWindow] = None
        self._container_child: Optional[QWidget] = None

        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"RViz-Config fehlt: {self.config_path}")
        if display is None:
            raise RuntimeError(f"python-xlib/X11 nicht verfügbar: {_xlib_import_error}")

        # Container braucht Layout
        if self.container.layout() is None:
            lay = QVBoxLayout(self.container)
            lay.setContentsMargins(0, 0, 0, 0)
            self.container.setLayout(lay)

    # ---------- Public API ----------
    def start(self, *, timeout_s: float = 12.0) -> None:
        if self._proc is not None:
            raise RuntimeError("RViz läuft bereits (start).")

        env = dict(os.environ)
        if env.get("QT_QPA_PLATFORM", "xcb") != "xcb":
            raise RuntimeError("QT_QPA_PLATFORM muss 'xcb' (X11) sein.")

        cmd = ["rviz2", "-d", self.config_path]
        if self.extra_args:
            cmd.extend(self.extra_args)

        self.log.info("Start RViz: %s", " ".join(cmd))
        self._proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True, encoding="utf-8", errors="replace",
            bufsize=1,
            preexec_fn=os.setsid,
            env=env,
        )

        wid = self._wait_for_window(self._proc.pid, timeout_s=timeout_s)
        self._qwindow = QWindow.fromWinId(wid)
        self._qwindow.setFlags(Qt.FramelessWindowHint | Qt.WindowTransparentForInput)
        self._mount_into(self.container)

    def reparent(self, new_container: QWidget) -> None:
        """
        Hängt die laufende RViz-Instanz in einen anderen Container um,
        ohne RViz neu zu starten.
        """
        if self._proc is None or self._qwindow is None:
            raise RuntimeError("RViz läuft nicht – reparent() nicht möglich.")

        if new_container is self.container:
            return

        # altes Container-Widget entfernen
        try:
            if self._container_child is not None:
                lay = self.container.layout()
                if lay is not None:
                    lay.removeWidget(self._container_child)
                self._container_child.setParent(None)
                self._container_child.deleteLater()
        except Exception:
            pass
        self._container_child = None

        # neuen Container setzen + mounten
        self.container = new_container
        if self.container.layout() is None:
            lay = QVBoxLayout(self.container)
            lay.setContentsMargins(0, 0, 0, 0)
            self.container.setLayout(lay)

        self._mount_into(self.container)
        self.log.info("RViz umgehängt in neuen Container.")

    def stop(self, *, timeout_s: float = 3.0) -> None:
        # Container loslösen
        try:
            if self._container_child is not None:
                lay = self.container.layout()
                if lay is not None:
                    lay.removeWidget(self._container_child)
                self._container_child.deleteLater()
        finally:
            self._container_child = None
            self._qwindow = None

        # Subprozess sauber beenden
        if self._proc is None:
            return
        try:
            pgid = os.getpgid(self._proc.pid)
            os.killpg(pgid, signal.SIGTERM)
        except Exception:
            pass

        t0 = time.time()
        while time.time() - t0 < timeout_s and self._proc.poll() is None:
            time.sleep(0.1)

        if self._proc.poll() is None:
            try:
                pgid = os.getpgid(self._proc.pid)
                os.killpg(pgid, signal.SIGKILL)
            except Exception:
                pass

        self._proc = None
        self.log.info("RViz gestoppt.")

    # ---------- Internals ----------
    def _wait_for_window(self, pid: int, *, timeout_s: float) -> int:
        d = display.Display()
        root = d.screen().root
        NET_CLIENT_LIST = d.intern_atom("_NET_CLIENT_LIST")
        NET_WM_PID = d.intern_atom("_NET_WM_PID")

        end = time.time() + timeout_s
        last_dump = 0.0

        while time.time() < end:
            try:
                wins = root.get_full_property(NET_CLIENT_LIST, X.AnyPropertyType)
                if wins and wins.value:
                    for w in wins.value:
                        win = d.create_resource_object("window", w)
                        pid_prop = win.get_full_property(NET_WM_PID, X.AnyPropertyType)
                        if pid_prop and pid_prop.value and int(pid_prop.value[0]) == pid:
                            return int(w)
            except Exception:
                pass

            # sporadisch stdout dumpen (hilft beim Debuggen)
            if self._proc and self._proc.stdout and (time.time() - last_dump) > 1.5:
                try:
                    line = self._proc.stdout.readline().strip()
                    if line:
                        self.log.info("[rviz2] %s", line)
                except Exception:
                    pass
                last_dump = time.time()

            time.sleep(0.05)

        raise RuntimeError("RViz-Fenster nicht gefunden (X11).")

    def _mount_into(self, parent: QWidget) -> None:
        container = QWidget.createWindowContainer(self._qwindow, parent)
        container.setFocusPolicy(Qt.StrongFocus)
        parent.layout().addWidget(container)
        self._container_child = container
