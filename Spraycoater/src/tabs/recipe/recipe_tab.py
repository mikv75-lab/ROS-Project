# -*- coding: utf-8 -*-
# File: src/tabs/recipe/recipe_tab.py
from __future__ import annotations

import logging
from typing import Optional, Callable, Any

from PyQt6.QtWidgets import QWidget, QHBoxLayout

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore

_LOG = logging.getLogger("tabs.recipe")


class RecipeTab(QWidget):
    """
    Recipe Tab.
    Left: RecipeEditorPanel (Editing)
    Right: CoatingPreviewPanel (Preview 3D/2D)

    Note: PyVista is injected via attach_preview_widget to the *inner* widget of CoatingPreviewPanel.
    """

    def __init__(
        self,
        *,
        ctx,
        store: RecipeStore,
        repo: Any,
        attach_preview_widget: Callable[[QWidget], None] | None = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.ctx = ctx

        if store is None:
            raise RuntimeError("RecipeTab: store ist None.")
        if not isinstance(store, RecipeStore):
            raise TypeError(f"RecipeTab: store ist kein RecipeStore (got: {type(store)}).")
        if repo is None:
            raise RuntimeError("RecipeTab: repo ist None.")

        self.store: RecipeStore = store
        self.repo = repo
        self._initial_preview_done = False

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)

        # Left Panel: Editor
        self.recipePanel = RecipeEditorPanel(
            ctx=self.ctx,
            store=self.store,
            repo=self.repo,
            parent=self,
        )

        # Right Panel: Preview (passed ctx for SceneManager)
        self.previewPanel = CoatingPreviewPanel(
            ctx=self.ctx,
            store=self.store,
            parent=self,
        )

        layout.addWidget(self.recipePanel, 2)
        layout.addWidget(self.previewPanel, 3)

        # Inject PyVista Interactor if callback provided
        if attach_preview_widget is not None:
            # We get the actual PV host widget from the preview panel (inside its 3D tab)
            pv_host = self.previewPanel.get_pv_host()
            attach_preview_widget(pv_host)

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