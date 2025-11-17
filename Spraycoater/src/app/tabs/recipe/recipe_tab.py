# app/tabs/recipe/recipe_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable

from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt, QTimer

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from app.model.recipe.recipe import Recipe
from app.model.recipe import recipe_markers
from visualization_msgs.msg import MarkerArray

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

        # Flag: initiales Preview nur genau 1x
        self._initial_preview_done: bool = False

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
        if callable(self._attach_preview_widget):
            try:
                self._attach_preview_widget(self.previewPanel.get_pv_host())
            except Exception:
                _LOG.exception("attach_preview_widget failed")

        # ---------- Direkte Signal-Verdrahtung ----------
        self.recipePanel.updatePreviewRequested.connect(
            self._on_update_preview_requested,
            Qt.ConnectionType.QueuedConnection,
        )

        # ---------- Initiales Preview NACH Interactor ----------
        try:
            self.previewPanel.interactorReady.connect(self._emit_initial_preview)
        except Exception:
            pass

        # Sanfter Fallback, falls interactorReady nicht feuert
        QTimer.singleShot(500, self._emit_initial_preview)

    # ==================== Preview + RViz: gemeinsame Quelle ====================

    def _on_update_preview_requested(self, model: Recipe) -> None:
        """
        Wird aufgerufen, wenn der Update-Button gedrückt wird ODER
        beim initialen Preview.

        1) PyVista-Preview aktualisieren
        2) MarkerArray für SprayPath/RViz aus *derselben* Recipe-Quelle erzeugen
           und via Bridge publizieren.
        """
        # 1) Preview aktualisieren
        try:
            self.previewPanel.handle_update_preview(model)
        except Exception:
            _LOG.exception("handle_update_preview failed")

        # 2) RViz / SprayPath: MarkerArray bauen und über Bridge senden
        try:
            # aktive Seiten aus der CheckableTabBar holen (z.B. top, sides, bottom ...)
            try:
                checked_sides = self.recipePanel.content.checked_sides()
            except Exception:
                checked_sides = None

            # MarkerArray aus Recipe – Frame ist 'scene'
            ma: MarkerArray = recipe_markers.build_marker_array_from_recipe(
                model,
                sides=checked_sides,
                frame_id="scene",
            )

            if ma and isinstance(ma, MarkerArray) and ma.markers:
                try:
                    if self.bridge is not None:
                        self.bridge.set_spraypath(ma)
                        _LOG.info(
                            "RecipeTab: MarkerArray mit %d Markern an SprayPath gesendet (frame=scene, sides=%s)",
                            len(ma.markers),
                            ", ".join(checked_sides or []),
                        )
                except Exception:
                    _LOG.exception("bridge.set_spraypath failed")
            else:
                _LOG.warning(
                    "RecipeTab: build_marker_array_from_recipe lieferte kein/nur leeres MarkerArray"
                )
        except Exception:
            _LOG.exception("Fehler beim Erzeugen/Publizieren des MarkerArray für SprayPath")

    def _emit_initial_preview(self) -> None:
        """
        Initiales Preview nach Start:
        nutzt denselben Pfad wie der Update-Button, damit Preview & RViz identisch sind.
        Läuft nur genau EINMAL.
        """
        if self._initial_preview_done:
            return

        try:
            model = self.recipePanel.current_model()
            if model is None:
                _LOG.info("initial preview: no model yet")
                return

            self._initial_preview_done = True
            self._on_update_preview_requested(model)
            _LOG.info("initial preview ausgeführt")
        except Exception:
            _LOG.exception("initial preview emit failed")
