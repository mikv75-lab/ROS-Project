# Spraycoater/src/app/tabs/process/process_tab.py
# -*- coding: utf-8 -*-
import os
import logging
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget

from ros.rviz_manager import LiveRvizManager


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/process
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", filename)

def _rviz_cfg_path() -> str:
    # feste Datei – kein Fallback
    return os.path.join(_project_root(), "resource", "config", "live.rviz")


class ProcessTab(QWidget):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        uic.loadUi(_ui_path("process_tab.ui"), self)

        self._log = logging.getLogger("app.tabs.process")
        self._rviz = LiveRvizManager.instance()
        self._cfg = _rviz_cfg_path()

        if not os.path.exists(self._cfg):
            # harter Fehler -> Button deaktivieren + Hinweis
            self.lblLiveCfg.setText(f"Fehlt: {self._cfg}")
            self.btnLiveStart.setEnabled(False)
            self.btnLiveRestart.setEnabled(False)
        else:
            self.lblLiveCfg.setText(self._cfg)

        # wiring
        self.btnLiveStart.clicked.connect(self._on_start)
        self.btnLiveStop.clicked.connect(self._on_stop)
        self.btnLiveRestart.clicked.connect(self._on_restart)

        self._refresh_status()

    # --- actions ---
    def _on_start(self):
        self._rviz.start(self._cfg)
        self._refresh_status()

    def _on_stop(self):
        self._rviz.stop()
        self._refresh_status()

    def _on_restart(self):
        self._rviz.restart(self._cfg)
        self._refresh_status()

    def _refresh_status(self):
        if self._rviz.is_running():
            self.lblLiveStatus.setText(f"running (pid={self._rviz.pid()})")
        else:
            self.lblLiveStatus.setText("stopped")
