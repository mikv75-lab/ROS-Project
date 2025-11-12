# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable

from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

_LOG = logging.getLogger("app.tabs.recipe")


class RecipeTab(QWidget):
    def __init__(
        self,
        *,
        ctx,
        store,
        bridge,
        attach_preview_widget: Callable[[QWidget], None] | None = None,
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        # ---------- Layout ----------
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Links: Recipe-Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, store=self.store, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Rechts: Preview
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, store=self.store, parent=self)
        hroot.addWidget(self.previewPanel, 1)
        hroot.setStretch(0, 0)
        hroot.setStretch(1, 1)

        # Optional: externen Host (den echten PyVista-Interactor) anhängen
        # WICHTIG: wir geben NICHT das ganze Panel, sondern den dedizierten Host weiter
        if callable(self._attach_preview_widget):
            try:
                self._attach_preview_widget(self.previewPanel.get_pv_host())
            except Exception:
                _LOG.exception("attach_preview_widget failed")

        # ---------- Direkte Signal-Verdrahtung ----------
        # Editor -> Preview: immer das Recipe-Objekt senden
        self.recipePanel.updatePreviewRequested.connect(
            self.previewPanel.handle_update_preview,
            Qt.ConnectionType.QueuedConnection,
        )
