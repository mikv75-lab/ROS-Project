# -*- coding: utf-8 -*-
import os
from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QVBoxLayout

# Lokale Imports aus demselben Ordner
from .path_visualizer import RobotPathVisualizer


def _project_root_from_here() -> str:
    """
    recipe_tab.py liegt unter:
      Spraycoater/src/app/tabs/recipe/recipe_tab.py
    -> 5x hoch zum Projekt-Root.
    """
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    root = _project_root_from_here()
    return os.path.join(root, "resource", "ui", filename)


class RecipeTab(QWidget):
    """
    UI-Tab, der das RobotPathVisualizer-Widget (PyVista-QtInteractor) einbettet.
    Die Controls (Shape/Path/Normals + 'Pfad generieren') liegen im Visualizer selbst.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        uic.loadUi(_ui_path("recipe_tab.ui"), self)

        # In der UI muss ein QWidget namens 'previewContainer' existieren
        container = getattr(self, "previewContainer", None)
        if container is None:
            raise RuntimeError("recipe_tab.ui: QWidget 'previewContainer' nicht gefunden.")

        # Falls der Container noch kein Layout hat, eines setzen
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))

        # Visualizer erzeugen und in den Container hängen
        self.visualizer = RobotPathVisualizer(parent=container)
        container.layout().addWidget(self.visualizer)
