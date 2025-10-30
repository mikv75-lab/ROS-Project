# Spraycoater/src/app/tabs/recipe/recipe_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional

from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6 import sip  # noqa: F401

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .planning_panel.planning_panel import PlanningPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

_LOG = logging.getLogger("app.tabs.recipe.tab")

class RecipeTab(QWidget):
    """
    Simple Layout (no containers):
      [RecipeEditorPanel] | [CoatingPreviewPanel] | [PlanningPanel]

    - Plotter und Render-API leben vollständig im CoatingPreviewPanel.
    - Dieser Tab verdrahtet nur Panels und triggert das Rendern.
    """
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # --- Root HBox (no containers) ---
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Left: Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Center: Preview (owns QtInteractor + camera/mesh API)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Right: Planner
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)
        hroot.addWidget(self.planningPanel, 0)

        # Editor → Preview render trigger
        self.recipePanel.updatePreviewRequested.connect(self.previewPanel.render_from_model)
        
        # Provide providers to PlanningPanel (if present)
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            self.planningPanel.set_traj_provider(
                getattr(self.previewPanel, "last_trajectory_dict", lambda: None)
            )
        if hasattr(self.planningPanel, "set_bridge"):
            self.planningPanel.set_bridge(self.bridge)
