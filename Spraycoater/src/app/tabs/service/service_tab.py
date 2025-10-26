# Spraycoater/src/app/tabs/service/service_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget

from ros.rviz_manager import LiveRvizManager

_LOG = logging.getLogger("app.tabs.service")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/service
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    # neue Struktur: resource/ui/tabs/service/service_tab.ui
    return os.path.join(_project_root(), "resource", "ui", "tabs", "service", filename)

def _rviz_cfg_path() -> str:
    # live.rviz liegt jetzt in resource/rviz/
    return os.path.join(_project_root(), "resource", "rviz", "live.rviz")

class ServiceTab(QWidget):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("service_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("ServiceTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)

        self._rviz = LiveRvizManager.instance()
        self._cfg = _rviz_cfg_path()

        if not os.path.exists(self._cfg):
            self.lblLiveCfg.setText(f"Fehlt: {self._cfg}")
            self.btnLiveStart.setEnabled(False)
            self.btnLiveRestart.setEnabled(False)
        else:
            self.lblLiveCfg.setText(self._cfg)

        self.btnLiveStart.clicked.connect(self._on_start)
        self.btnLiveStop.clicked.connect(self._on_stop)
        self.btnLiveRestart.clicked.connect(self._on_restart)

        self._refresh_status()

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
