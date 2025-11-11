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

        # Optional: externen Host informieren (Backwards-Compat)
        if self._attach_preview_widget is not None:
            self._attach_preview_widget(self.previewPanel)

        # ---------- Direkte Signal-Verdrahtung (ohne Fallbacks) ----------
        # Editor -> Preview: immer das Recipe-Objekt senden
        # Erwartete Signatur:
        #   RecipeEditorPanel.updatePreviewRequested: pyqtSignal(object)  # model: Recipe
        #   CoatingPreviewPanel.handle_update_preview(model: object) -> None
        self.recipePanel.updatePreviewRequested.connect(
            self.previewPanel.handle_update_preview,
            Qt.ConnectionType.QueuedConnection,
        )

        # (Optional) Wenn du später RViz koppeln willst, nimm EINEN festen Hook,
        # z.B. Bridge.spraypath.publish_preview(marker_array) und hier verbinden:
        # self.previewPanel.sprayPathSetRequested.connect(
        #     self.bridge.spraypath.publish_preview,
        #     Qt.ConnectionType.QueuedConnection,
        # )
