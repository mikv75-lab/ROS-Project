# app/tabs/recipe/recipe_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from typing import Optional, Callable

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QWidget, QHBoxLayout

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from app.model.recipe.recipe import Recipe
from app.model.recipe import recipe_markers
from visualization_msgs.msg import MarkerArray

_LOG = logging.getLogger("app.tabs.recipe")


class RecipeTab(QWidget):
    """
    Links:  RecipeEditorPanel
    Rechts: CoatingPreviewPanel (PyVista-Host wird extern attached)

    Wichtig:
      - Preview und RViz/Marker werden immer aus derselben Recipe-Quelle erzeugt.
      - Das initiale Preview läuft genau 1x – erst wenn der 3D-Interactor bereit ist.
    """

    def __init__(
        self,
        *,
        ctx,
        store,
        bridge,
        attach_preview_widget: Callable[[QWidget], None] | None = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        self._initial_preview_done: bool = False

        # ---------------- Layout ----------------
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, store=self.store, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, store=self.store, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        hroot.setStretch(0, 0)
        hroot.setStretch(1, 1)

        # Externen PyVista-Interactor-Host an das Preview-Panel hängen (z. B. QtInteractor-Widget)
        if callable(self._attach_preview_widget):
            self._attach_preview_widget(self.previewPanel.get_pv_host())

        # ---------------- Signals ----------------
        # Update-Button (und Load) → Preview + Marker
        self.recipePanel.updatePreviewRequested.connect(
            self._on_update_preview_requested,
            Qt.ConnectionType.QueuedConnection,
        )

        # Initiales Preview erst, wenn der Interactor da ist (kein "Fallback"-Timer)
        self.previewPanel.interactorReady.connect(self._emit_initial_preview)

    # ==================== Preview + RViz aus derselben Quelle ====================

    def _on_update_preview_requested(self, model: Recipe) -> None:
        """
        Einziger Einstiegspunkt für Preview + Marker-Update:
          1) PyVista-Preview aktualisieren
          2) MarkerArray aus derselben Recipe-Quelle bauen und über die Bridge publizieren
        """
        # 1) Preview
        try:
            self.previewPanel.handle_update_preview(model)
        except Exception:
            _LOG.exception("RecipeTab: handle_update_preview failed")

        # 2) Marker / RViz
        try:
            checked_sides = None
            try:
                checked_sides = self.recipePanel.content.checked_sides()
            except Exception:
                checked_sides = None

            ma: MarkerArray = recipe_markers.build_marker_array_from_recipe(
                model,
                sides=checked_sides,
                frame_id="scene",
            )

            if isinstance(ma, MarkerArray) and getattr(ma, "markers", None):
                if self.bridge is not None:
                    self.bridge.set_spraypath(ma)
                _LOG.info(
                    "RecipeTab: SprayPath MarkerArray gesendet (%d Marker, frame=scene, sides=%s)",
                    len(ma.markers),
                    ", ".join(checked_sides or []),
                )
            else:
                _LOG.warning("RecipeTab: MarkerArray ist leer (nichts zu publizieren)")
        except Exception:
            _LOG.exception("RecipeTab: Fehler beim Erzeugen/Publizieren des MarkerArray")

    def _emit_initial_preview(self) -> None:
        """
        Initiales Preview nach Start – läuft genau einmal, sobald der Interactor bereit ist.
        """
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
