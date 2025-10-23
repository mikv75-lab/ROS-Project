from .mesh_utils import load_mesh
from .path_visualizer import visualize_spray_path
from PyQt5.QtWidgets import QWidget
from PyQt5 import uic
import os

class SprayPathTab(QWidget):
    def __init__(self, bridge, parent=None):
        super().__init__(parent)
        ui_file = os.path.join(os.path.dirname(__file__), "../../resources/ui/spray_path_tab.ui")
        uic.loadUi(ui_file, self)
        self.bridge = bridge

        # Load mesh and visualize path
        mesh = load_mesh('mesh_file_1.stl')
        visualize_spray_path(100, 50)  # Spray path with 100mm radius and 50mm height
