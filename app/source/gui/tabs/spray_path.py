# source/gui/tabs/spray_path.py
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget
import os

class SprayPathTab(QWidget):
    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        ui_file = os.path.join(os.path.dirname(__file__), "../ui/spray_path_tab.ui")
        uic.loadUi(ui_file, self)
        self.bridge = bridge
