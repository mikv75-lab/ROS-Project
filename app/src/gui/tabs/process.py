# source/gui/tabs/process.py
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
import os

class ProcessTab(QWidget):
    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        ui_file = os.path.join(os.path.dirname(__file__), "../ui/process_tab.ui")
        uic.loadUi(ui_file, self)
        self.bridge = bridge
