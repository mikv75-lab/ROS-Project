# -*- coding: utf-8 -*-
import os
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget

def _ui_path(filename: str) -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    return os.path.join(root, "resource", "ui", filename)

class RecipeTab(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(_ui_path("recipe_tab.ui"), self)
