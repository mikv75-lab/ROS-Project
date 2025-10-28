# Spraycoater/src/app/tabs/process/process_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import QTimer

from ros.rviz_manager import LiveRvizManager

_LOG = logging.getLogger("app.tabs.process")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/process
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    # neue Struktur: resource/ui/tabs/process/process_tab.ui
    return os.path.join(_project_root(), "resource", "ui", "tabs", "process", filename)

def _rviz_cfg_path() -> str:
    # live.rviz liegt jetzt in resource/rviz/
    return os.path.join(_project_root(), "resource", "rviz", "live.rviz")

def _rviz_env() -> dict:
    home = os.environ.get("HOME", "/root")
    env = {
        "HOME": home,
        "ROS_LOG_DIR": os.path.join(home, ".ros", "log"),
        "XDG_RUNTIME_DIR": os.environ.get("XDG_RUNTIME_DIR", "/tmp/runtime-root"),
        "RCUTILS_LOGGING_USE_STDOUT": os.environ.get("RCUTILS_LOGGING_USE_STDOUT", "1"),
        "LC_ALL": "C.UTF-8",
        "LANG": "C.UTF-8",
    }
    try:
        os.makedirs(env["ROS_LOG_DIR"], exist_ok=True)
        os.makedirs(env["XDG_RUNTIME_DIR"], exist_ok=True)
    except Exception:
        pass
    return env

class ProcessTab(QWidget):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("process_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("ProcessTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)

        self._rviz = LiveRvizManager.instance()
        self._cfg = _rviz_cfg_path()

        if not os.path.exists(self._cfg):
            self.lblLiveCfg.setText(f"Fehlt: {self._cfg}")
            self.btnLiveStart.setEnabled(False)
            self.btnLiveRestart.setEnabled(False)
        else:
            self.lblLiveCfg.setText(self._cfg)

        # Zielcontainer für späteres Einbetten setzen
        if hasattr(self, "rvizContainer"):
            self._rviz.attach(self.rvizContainer)

        # Buttons
        self.btnLiveStart.clicked.connect(self._on_start)
        self.btnLiveStop.clicked.connect(self._on_stop)
        self.btnLiveRestart.clicked.connect(self._on_restart)

        # --- Silent & Embedded Start direkt beim Tab-Init ---
        if os.path.exists(self._cfg):
            # minimal verzögert starten, damit das UI fertig konstruiert ist
            QTimer.singleShot(50, self._auto_start_silent)
        else:
            self._safe_refresh()

    # ---------- intern ----------
    def _auto_start_silent(self):
        # Start "silent": Fenster wird versteckt, eingebettet, dann sichtbar gemacht
        self._rviz.start(config_path=self._cfg, extra_args=[], env=_rviz_env(), silent=True)
        # kurze Delays, um das Window zu finden und einzubetten
        QTimer.singleShot(350, self._embed_and_refresh)

    def _embed_and_refresh(self):
        # Falls attach noch nicht eingebettet hat, versuche es explizit
        if hasattr(self, "rvizContainer"):
            self._rviz.attach(self.rvizContainer)
        QTimer.singleShot(100, self._safe_refresh)

    # ---------- Button-Handler ----------
    def _on_start(self):
        self._rviz.start(config_path=self._cfg, extra_args=[], env=_rviz_env(), silent=True)
        QTimer.singleShot(300, self._embed_and_refresh)

    def _on_stop(self):
        self._rviz.stop()
        self._safe_refresh()

    def _on_restart(self):
        self._rviz.restart(config_path=self._cfg, extra_args=[], env=_rviz_env(), silent=True)
        QTimer.singleShot(350, self._embed_and_refresh)

    # ---------- UI-safe ----------
    def _safe_refresh(self):
        try:
            self._refresh_status()
        except RuntimeError:
            pass

    def _refresh_status(self):
        if self._rviz.is_running():
            pid = self._rviz.pid() if hasattr(self._rviz, "pid") else "?"
            self.lblLiveStatus.setText(f"running (pid={pid})")
        else:
            self.lblLiveStatus.setText("stopped")
