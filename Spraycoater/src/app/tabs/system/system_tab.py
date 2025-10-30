# Spraycoater/src/app/tabs/system/system_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from PyQt6 import uic
from PyQt6.QtWidgets import QWidget

_LOG = logging.getLogger("app.tabs.system")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/system
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    # neue Struktur: resource/ui/tabs/system/system_tab.ui
    return os.path.join(_project_root(), "resource", "ui", "tabs", "system", filename)

class SystemTab(QWidget):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        ui_file = _ui_path("system_tab.ui")
        if not os.path.exists(ui_file):
            _LOG.error("SystemTab UI nicht gefunden: %s", ui_file)
        uic.loadUi(ui_file, self)
