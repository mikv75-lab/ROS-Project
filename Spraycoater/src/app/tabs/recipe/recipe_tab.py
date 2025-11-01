# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable

from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt, QTimer   # <- QTimer hinzugefügt

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .planning_panel.planning_panel import PlanningPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from .coating_preview_panel.mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)

_LOG = logging.getLogger("app.tabs.recipe.tab")


class RecipeTab(QWidget):
    """
    Layout: [RecipeEditorPanel] | [CoatingPreviewPanel(host)] | [PlanningPanel]

    - Der QtInteractor lebt im MainWindow.
    - attach_preview_widget(host) hängt ihn in den Host aus dem PreviewPanel.
    - Rendering (clear/add_mesh/camera/render) läuft vollständig über das Panel.
    """

    def __init__(
        self,
        *,
        ctx,
        bridge,
        attach_preview_widget: Callable[[QWidget], None],
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Left: Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Center: Preview (Host + Buttons; kein Interactor im Panel selbst)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Interactor in den Host hängen (wie bisher über MainWindow)
        self._attach_preview_widget(self.previewPanel.preview_host())

        # >>> NEU: Interactor sofort ins Panel adoptieren (damit _ia gesetzt ist)
        try:
            win = self.window() or self.parent()
            if win is not None and hasattr(win, "previewPlot") and win.previewPlot is not None:
                self.previewPanel.attach_interactor(win.previewPlot)
        except Exception:
            _LOG.exception("RecipeTab: adopt previewPlot into previewPanel failed")

        # Init-Szene im Panel (entspricht dem früheren Main-Grid/Bounds)
        if hasattr(self.previewPanel, "build_init_scene_mainstyle"):
            self.previewPanel.build_init_scene_mainstyle(grid_step=10.0)

        # Right: Planner
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)
        hroot.addWidget(self.planningPanel, 0)

        # Editor → Preview render trigger (Queued)
        self.recipePanel.updatePreviewRequested.connect(
            lambda model: self._render_preview(model),
            Qt.ConnectionType.QueuedConnection,
        )

        # Optional: Provider an PlanningPanel
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            self.planningPanel.set_traj_provider(lambda: None)
        if hasattr(self.planningPanel, "set_bridge"):
            self.planningPanel.set_bridge(self.bridge)

        # >>> NEU: initiales Preview NACH Layout/Adoption asynchron triggern
        QTimer.singleShot(0, self.trigger_initial_preview)

    # -------- Public: Initiales Preview nach App-Start ------------------------
    def trigger_initial_preview(self):
        """
        Wird nach dem ersten show() via QTimer(0) aufgerufen.
        Zeichnet das Grid und rendert – falls vorhanden – das aktuelle Modell einmalig.
        """
        _LOG.debug("RecipeTab.trigger_initial_preview()")
        try:
            if hasattr(self.previewPanel, "build_init_scene_mainstyle"):
                self.previewPanel.build_init_scene_mainstyle(grid_step=10.0)
        except Exception:
            _LOG.exception("RecipeTab.build_init_scene_mainstyle() failed")

        # Falls der Editor bereits ein Modell hat, direkt rendern
        try:
            provider = getattr(self.recipePanel, "current_model", None)
            model = provider() if callable(provider) else provider
            if model:
                self._render_preview(model)
        except Exception:
            _LOG.exception("RecipeTab.trigger_initial_preview() -> _render_preview failed")

    # -------- Render orchestration: Panel macht clear/add_mesh/render --------
    def _render_preview(self, model: object):
        try:
            # Pflichtfelder lesen
            mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt 'substrate_mount'.")
            substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt 'substrate'.")

            # Szene neu aufbauen (Panel cleart + Grid)
            try:
                self.previewPanel.clear()
            except Exception:
                _LOG.exception("previewPanel.clear() failed")

            # Mount
            try:
                mmesh = load_mount_mesh_from_key(self.ctx, mount_key)
                self.previewPanel.add_mesh(
                    mmesh, color="lightgray", opacity=0.3, lighting=False
                )
            except Exception as e:
                _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

            # Substrat
            try:
                smesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
                smesh = place_substrate_on_mount(self.ctx, smesh, mount_key=mount_key)
                self.previewPanel.add_mesh(
                    smesh, color="#3498db", opacity=0.95, lighting=False
                )
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

            # Kamera + Render über Panel
            try:
                self.previewPanel.view_isometric()
            except Exception:
                pass
            self.previewPanel.render(reset_camera=True)

        except Exception:
            _LOG.exception("Render orchestration failed")

    @staticmethod
    def _get_required_str(model: object, key: str, err: str) -> str:
        if hasattr(model, key):
            val = getattr(model, key)
        elif isinstance(model, dict):
            val = model.get(key)
        else:
            val = None
        if not isinstance(val, str) or not val.strip():
            raise ValueError(err)
        return val.strip()
