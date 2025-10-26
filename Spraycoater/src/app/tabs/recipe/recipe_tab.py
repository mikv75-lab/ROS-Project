# -*- coding: utf-8 -*-
# Spraycoater/src/app/tabs/recipe/recipe_tab.py
from __future__ import annotations
import os
import sys
import logging
from typing import Optional

from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QVBoxLayout

_LOG = logging.getLogger("app.tabs.recipe.tab")

# --- Robust machen: PROJECT_ROOT und SRC_ROOT ermitteln und in sys.path eintragen ---
HERE = os.path.abspath(os.path.dirname(__file__))
# …/src/app/tabs/recipe -> PROJECT_ROOT = …/
PROJECT_ROOT = os.path.abspath(os.path.join(HERE, "..", "..", "..", ".."))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

# Jetzt funktionieren absolute Paket-Imports immer – egal ob über main_gui.py oder direkt
from app.tabs.recipe.recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from app.tabs.recipe.coating_preview_panel.coating_preview_panel import CoatingPreviewPanel
from app.tabs.recipe.planning_panel.planning_panel import PlanningPanel

def _ui_path(filename: str) -> str:
    return os.path.join(PROJECT_ROOT, "resource", "ui", "tabs", "recipe", filename)


class RecipeTab(QWidget):
    """
    Orchestriert die 3 Panels:
      [links]  RecipeEditorPanel  – Load/Delete/Create, Formular/Params, Update Viewer
      [mitte]  CoatingPreviewPanel – PyVista Szene + Pfad/Traj Vorschau
      [rechts] PlanningPanel – Planner-Auswahl, Validate/Optimize/Save
    """
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        uic.loadUi(_ui_path("recipe_tab.ui"), self)

        self.left  = RecipeEditorPanel(ctx=self.ctx, parent=self)
        self.mid   = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        self.right = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)

        self._mount_into(self.leftContainer,   self.left)
        self._mount_into(self.middleContainer, self.mid)
        self._mount_into(self.rightContainer,  self.right)

        self.left.recipeChanged.connect(self._on_recipe_changed)
        self.mid.readyChanged.connect(self._on_preview_ready_changed)

        if hasattr(self.right, "set_model_provider"):
            self.right.set_model_provider(self.left.current_recipe_model)
        if hasattr(self.right, "set_traj_provider"):
            self.right.set_traj_provider(self.mid.last_trajectory_dict)
        if hasattr(self.right, "set_bridge"):
            self.right.set_bridge(self.bridge)

        try:
            self._on_recipe_changed(self.left.current_recipe_model(), self.left.current_sides())
        except Exception as e:
            _LOG.warning("initial render failed: %s", e)

    def _mount_into(self, container, widget: QWidget):
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))
        container.layout().addWidget(widget)

    def _on_recipe_changed(self, model, sides):
        ok = self.mid.render_from_model(model, sides)
        if hasattr(self.right, "set_model_provider"):
            self.right.set_model_provider(self.left.current_recipe_model)
        if hasattr(self.right, "set_traj_provider"):
            self.right.set_traj_provider(self.mid.last_trajectory_dict)
        if hasattr(self.right, "set_preview_ready"):
            self.right.set_preview_ready(ok, "preview updated" if ok else "preview failed")

    def _on_preview_ready_changed(self, ok: bool, msg: str):
        if hasattr(self.right, "set_preview_ready"):
            self.right.set_preview_ready(ok, msg)
