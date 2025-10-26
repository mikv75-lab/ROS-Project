# -*- coding: utf-8 -*-
import os
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget

def _ui_path(filename: str) -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    return os.path.join(root, "resource", "ui", filename)

class SystemTab(QWidget):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        uic.loadUi(_ui_path("system_tab.ui"), self)
