# -*- coding: utf-8 -*-
# Spraycoater/src/app/tabs/recipe/recipe_tab.py
from __future__ import annotations
import os
import logging
from typing import Optional

from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel
from .planning_panel.planning_panel import PlanningPanel

_LOG = logging.getLogger("app.tabs.recipe.tab")

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/recipe
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    # UI liegt unter resource/ui/tabs/recipe/
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

class RecipeTab(QWidget):
    """
    Orchestriert die 3 Panels:
      [links]  RecipeEditorPanel
      [mitte]  CoatingPreviewPanel
      [rechts] PlanningPanel
    """
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # --- UI laden (benötigt exakt: leftContainer, plotContainer, rightTop) ---
        uic.loadUi(_ui_path("recipe_tab.ui"), self)
        for name in ("leftContainer", "plotContainer", "rightTop"):
            if not hasattr(self, name):
                raise RuntimeError(f"recipe_tab.ui erwartet ein QWidget namens '{name}'")

        # --- Panels erzeugen ---
        self.left  = RecipeEditorPanel(ctx=self.ctx, parent=self)
        self.mid   = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        self.right = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)

        # --- Panels montieren ---
        self._mount_into(self.leftContainer, self.left)
        self._mount_into(self.plotContainer, self.mid)
        self._mount_into(self.rightTop, self.right)

        # --- Signale ---
        self.left.recipeChanged.connect(self._on_recipe_changed)
        self.mid.readyChanged.connect(self._on_preview_ready_changed)

        # --- Planning: Provider setzen (Editor/Preview liefern Callables) ---
        if hasattr(self.right, "set_model_provider"):
            self.right.set_model_provider(self.left.current_model)  # <— korrekt
        if hasattr(self.right, "set_traj_provider"):
            self.right.set_traj_provider(self.mid.last_trajectory_dict)
        if hasattr(self.right, "set_bridge"):
            self.right.set_bridge(self.bridge)

        # Initial einmal rendern (falls Editor Defaults hat)
        try:
            self._on_recipe_changed(self.left.current_model(), self.left.selected_sides())
        except Exception as e:
            _LOG.warning("initial render failed: %s", e)

    # ---------- Intern: Container-Helper ----------
    def _mount_into(self, container, widget: QWidget):
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))
        container.layout().addWidget(widget)

    # ---------- Slots ----------
    def _on_recipe_changed(self, model, sides):
        ok = self.mid.render_from_model(model, sides)

        # Provider nachziehen (damit Planning aktuelle Callables hat)
        if hasattr(self.right, "set_model_provider"):
            self.right.set_model_provider(self.left.current_model)
        if hasattr(self.right, "set_traj_provider"):
            self.right.set_traj_provider(self.mid.last_trajectory_dict)

        if hasattr(self.right, "set_preview_ready"):
            self.right.set_preview_ready(ok, "preview updated" if ok else "preview failed")

    def _on_preview_ready_changed(self, ok: bool, msg: str):
        if hasattr(self.right, "set_preview_ready"):
            self.right.set_preview_ready(ok, msg)
