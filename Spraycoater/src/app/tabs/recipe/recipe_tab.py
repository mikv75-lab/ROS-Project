# -*- coding: utf-8 -*-
from __future__ import annotations
import os, logging
from typing import Optional, Tuple, Any

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout
import sip

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .planning_panel.planning_panel import PlanningPanel

_LOG = logging.getLogger("app.tabs.recipe.tab")
SAFE_NO_PREVIEW = os.environ.get("SAFE_NO_PREVIEW", "0") == "1"

def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

class RecipeTab(QWidget):
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        uic.loadUi(_ui_path("recipe_tab.ui"), self)
        for name in ("leftContainer", "plotContainer", "rightTop"):
            if not hasattr(self, name):
                raise RuntimeError(f"recipe_tab.ui erwartet ein QWidget namens '{name}'")

        self.recipePanel  = RecipeEditorPanel(ctx=self.ctx, parent=self)
        self.previewPanel = None
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)

        self._mount_into(self.leftContainer, self.recipePanel)
        self._mount_into(self.rightTop,     self.planningPanel)

        # Nur auf den expliziten Button reagieren:
        self.recipePanel.updatePreviewRequested.connect(self._on_update_preview_requested)

        # Lazy-Init des Preview, aber ohne Auto-Render
        self._connect_tab_activation()

    def _mount_into(self, container: QWidget, widget: QWidget):
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))
        container.layout().addWidget(widget)

    def _connect_tab_activation(self):
        if SAFE_NO_PREVIEW:
            return
        p = self.parent()
        while p is not None and p.parent() is not None:
            if p.metaObject().className() == "QTabWidget":
                break
            p = p.parent()
        if p is None or p.metaObject().className() != "QTabWidget":
            QTimer.singleShot(0, self._ensure_preview)
            return

        tab_widget = p
        try:
            idx = tab_widget.indexOf(self)
        except Exception:
            idx = -1

        def _on_current_changed(i: int):
            if i == idx:
                self._ensure_preview()

        tab_widget.currentChanged.connect(_on_current_changed)
        try:
            if tab_widget.currentIndex() == idx:
                self._ensure_preview()
        except Exception:
            pass

    def _ensure_preview(self):
        if SAFE_NO_PREVIEW:
            return
        if self.previewPanel is not None and not sip.isdeleted(self.previewPanel):
            return
        try:
            from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel
            self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
            self._mount_into(self.plotContainer, self.previewPanel)

            if hasattr(self.previewPanel, "readyChanged"):
                self.previewPanel.readyChanged.connect(self._on_preview_ready_changed)

            if hasattr(self.planningPanel, "set_model_provider"):
                self.planningPanel.set_model_provider(self.recipePanel.current_model)
            if hasattr(self.planningPanel, "set_traj_provider"):
                self.planningPanel.set_traj_provider(
                    getattr(self.previewPanel, "last_trajectory_dict", lambda: None)
                )
            if hasattr(self.planningPanel, "set_bridge"):
                self.planningPanel.set_bridge(self.bridge)

        except Exception as e:
            _LOG.critical("PreviewPanel init failed: %s", e, exc_info=True)

    # --- Slots ---
    def _on_update_preview_requested(self, recipe_model: object) -> None:
        """Kommt ausschließlich vom Button im Editor: komplettes Recipe-Modell rendern."""
        if SAFE_NO_PREVIEW or self.previewPanel is None or sip.isdeleted(self.previewPanel):
            return
        try:
            # Sides optional aus dem Modell bestimmen (falls vorhanden)
            pbs = getattr(recipe_model, "paths_by_side", None) or (
                recipe_model.get("paths_by_side") if isinstance(recipe_model, dict) else None
            )
            sides = list(pbs.keys()) if isinstance(pbs, dict) else []
            if hasattr(self.previewPanel, "render_from_model"):
                self.previewPanel.render_from_model(recipe_model, sides)
        except Exception as e:
            _LOG.error("update_preview render failed: %s", e, exc_info=True)
            if hasattr(self.planningPanel, "set_preview_ready"):
                self.planningPanel.set_preview_ready(False, str(e))

    def _on_preview_ready_changed(self, ok: bool, msg: str):
        if hasattr(self.planningPanel, "set_preview_ready"):
            self.planningPanel.set_preview_ready(ok, msg)
