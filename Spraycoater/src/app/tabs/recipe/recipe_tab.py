# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional

from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .planning_panel.planning_panel import PlanningPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

_LOG = logging.getLogger("app.tabs.recipe.tab")


class RecipeTab(QWidget):
    """
    Layout:
      [RecipeEditorPanel] | [CoatingPreviewPanel (UI/Signals)] | [PlanningPanel]

    Hinweis:
      - Alle PyVista-Aufrufe passieren im MainWindow (preview_api).
      - Dieses Tab verdrahtet nur Signale.
    """

    def __init__(self, *, ctx, bridge, preview_api, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self.preview_api = preview_api  # MainWindow-Instanz mit Plot-API

        # Root Layout
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Left: Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Center: PreviewPanel (ohne PyVista; sendet nur Signale)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Right: Planner
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)
        hroot.addWidget(self.planningPanel, 0)

        # Editor -> PreviewPanel (modellwechsel)
        self.recipePanel.updatePreviewRequested.connect(
            lambda model: self.previewPanel.render_from_model(model),
            Qt.ConnectionType.QueuedConnection
        )

        # PreviewPanel -> MainWindow (Rendern zentral)
        self.previewPanel.renderRequested.connect(
            lambda mount_key, substrate_key: self.preview_api.preview_render_model(mount_key, substrate_key),
            Qt.ConnectionType.QueuedConnection
        )

        # Kamera-Buttons -> MainWindow
        self.previewPanel.cameraIsoRequested.connect(self.preview_api.preview_view_iso)
        self.previewPanel.cameraTopRequested.connect(self.preview_api.preview_view_top)
        self.previewPanel.cameraFrontRequested.connect(self.preview_api.preview_view_front)
        self.previewPanel.cameraLeftRequested.connect(self.preview_api.preview_view_left)
        self.previewPanel.cameraRightRequested.connect(self.preview_api.preview_view_right)
        self.previewPanel.cameraBackRequested.connect(self.preview_api.preview_view_back)

        # Providers an PlanningPanel
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            self.planningPanel.set_traj_provider(getattr(self.previewPanel, "last_trajectory_dict", lambda: None))
        if hasattr(self.planningPanel, "set_bridge"):
            self.planningPanel.set_bridge(self.bridge)
