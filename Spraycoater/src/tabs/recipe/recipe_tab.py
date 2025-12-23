# app/tabs/recipe/recipe_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from typing import Optional, Callable

from PyQt6.QtWidgets import QWidget, QHBoxLayout

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore

_LOG = logging.getLogger("tabs.recipe")


class RecipeTab(QWidget):
    """
    Links:  RecipeEditorPanel
    Rechts: CoatingPreviewPanel (PyVista-Host wird extern attached)

    Wichtig:
      - Preview läuft wie gehabt, aber: KEIN ROS publish aus dem RecipeTab.
      - Validate/Optimize passieren nicht mehr hier (Buttons entfernt im Panel).
    """

    def __init__(
        self,
        *,
        ctx,
        store: RecipeStore | None,
        ros,
        attach_preview_widget: Callable[[QWidget], None] | None = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.ros = ros

        # ---------------- Store SSoT ----------------
        # Falls MainWindow store=None übergibt, holen wir ihn aus ctx.
        if store is None:
            store = getattr(ctx, "store", None) or getattr(ctx, "recipe_store", None)
        if store is None:
            raise RuntimeError(
                "RecipeTab: RecipeStore fehlt (store=None). "
                "MainWindow muss store übergeben oder ctx.store/ctx.recipe_store setzen."
            )
        self.store: RecipeStore = store

        self._initial_preview_done = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, store=self.store, parent=self)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, ros=self.ros, parent=self)

        layout.addWidget(self.recipePanel, 2)
        layout.addWidget(self.previewPanel, 3)

        if attach_preview_widget is not None:
            attach_preview_widget(self.previewPanel)

        self.recipePanel.updatePreviewRequested.connect(self._on_update_preview_requested)

    def _on_update_preview_requested(self, model: Recipe) -> None:
        try:
            self.previewPanel.update_preview(model)
        except Exception:
            _LOG.exception("RecipeTab: preview update failed")

    def on_after_show(self) -> None:
        if self._initial_preview_done:
            return

        try:
            model = self.recipePanel.current_model()
            if model is None:
                _LOG.info("RecipeTab: initial preview -> kein Model verfügbar")
                return

            self._initial_preview_done = True
            self._on_update_preview_requested(model)
            _LOG.info("RecipeTab: initial preview ausgeführt")
        except Exception:
            _LOG.exception("RecipeTab: initial preview emit failed")
