# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable, Any

from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt

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
    - preview_api liefert die Methoden: clear/add_mesh/view_*/render
    """

    def __init__(
        self,
        *,
        ctx,
        bridge,
        attach_preview_widget: Callable[[QWidget], None],
        preview_api: Any,
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget
        self._preview = preview_api

        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Left: Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Center: Preview (stellt Host + Buttons, kein Interactor!)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Interactor in den Host hängen
        self._attach_preview_widget(self.previewPanel.preview_host())

        # Kamera-Button-Hooks an MainWindow-API binden
        self.previewPanel.set_camera_hooks(
            on_iso=self._preview.preview_view_iso,
            on_top=self._preview.preview_view_top,
            on_front=self._preview.preview_view_front,
            on_left=self._preview.preview_view_left,
            on_right=self._preview.preview_view_right,
            on_back=self._preview.preview_view_back,
            on_after=lambda: self._preview.preview_render(reset_camera=True),
        )

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

    # -------- Render orchestration (MainWindow macht die Plot-Calls) --------
    def _render_preview(self, model: object):
        try:
            # Pflichtfelder lesen
            mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt 'substrate_mount'.")
            substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt 'substrate'.")

            # Szene neu aufbauen
            self._preview.preview_clear()

            # Mount
            try:
                mmesh = load_mount_mesh_from_key(self.ctx, mount_key)
                self._preview.preview_add_mesh(mmesh, color="lightgray", opacity=0.3, lighting=False)
            except Exception as e:
                _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

            # Substrat
            try:
                smesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
                smesh = place_substrate_on_mount(self.ctx, smesh, mount_key=mount_key)
                self._preview.preview_add_mesh(smesh, color="#3498db", opacity=0.95, lighting=False)
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

            # Kamera und Render
            self._preview.preview_view_iso()
            self._preview.preview_render(reset_camera=True)

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
