# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional, Tuple, Any

from PyQt5 import uic
from PyQt5.QtWidgets import QWidget, QVBoxLayout

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel
from .planning_panel.planning_panel import PlanningPanel

_LOG = logging.getLogger("app.tabs.recipe.tab")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/recipe
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    # UI liegt unter resource/ui/tabs/recipe/
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class RecipeTab(QWidget):
    """
    Orchestriert die 3 Panels:
      [links]  RecipeEditorPanel   -> self.recipePanel
      [mitte]  CoatingPreviewPanel -> self.previewPanel
      [rechts] PlanningPanel       -> self.planningPanel
    """
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        # --- UI laden (benötigt exakt diese Container) ---
        uic.loadUi(_ui_path("recipe_tab.ui"), self)
        for name in ("leftContainer", "plotContainer", "rightTop"):
            if not hasattr(self, name):
                raise RuntimeError(f"recipe_tab.ui erwartet ein QWidget namens '{name}'")

        # --- Panels erzeugen ---
        self.recipePanel  = RecipeEditorPanel(ctx=self.ctx, parent=self)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)

        # --- Panels montieren ---
        self._mount_into(self.leftContainer, self.recipePanel)
        self._mount_into(self.plotContainer, self.previewPanel)
        self._mount_into(self.rightTop,     self.planningPanel)

        # --- Signale verbinden ---
        # Editor -> meldet Änderungen am Rezept
        if hasattr(self.recipePanel, "recipeChanged"):
            self.recipePanel.recipeChanged.connect(self._on_recipe_changed)

        # Preview -> meldet Ready/Fehler
        if hasattr(self.previewPanel, "readyChanged"):
            self.previewPanel.readyChanged.connect(self._on_preview_ready_changed)

        # --- Provider in Planning setzen (Option B: Editor hat current_model()) ---
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            # CoatingPreviewPanel soll eine Callable oder Methode liefern,
            # die das letzte Trajektorie-Dict zurückgibt.
            self.planningPanel.set_traj_provider(
                getattr(self.previewPanel, "last_trajectory_dict", lambda: None)
            )
        if hasattr(self.planningPanel, "set_bridge"):
            self.planningPanel.set_bridge(self.bridge)

        # --- Initial einmal Preview rendern (defaults) ---
        try:
            model = self.recipePanel.current_model()
            self._render_in_preview(model, sides=["top"])  # in diesem Tab: feste Default-Side
        except Exception as e:
            _LOG.warning("initial render failed: %s", e)

    # ---------- Intern: Mount Helper ----------
    def _mount_into(self, container: QWidget, widget: QWidget):
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))
        container.layout().addWidget(widget)

    # ---------- Slots ----------
    def _on_recipe_changed(self, *args: Any):
        """
        Editor kann (model) oder (model, sides) emittieren.
        Dieser Tab arbeitet ohne Seitentoggles -> sides=["top"] als Default,
        außer der Editor liefert sie explizit mit.
        """
        model, sides = self._unpack_model_sides(args)
        self._render_in_preview(model, sides=sides)

        # Provider aktualisieren (damit Planning sofort aktuelle Callables hat)
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            self.planningPanel.set_traj_provider(
                getattr(self.previewPanel, "last_trajectory_dict", lambda: None)
            )

        if hasattr(self.planningPanel, "set_preview_ready"):
            self.planningPanel.set_preview_ready(True, "preview updated")

    def _on_preview_ready_changed(self, ok: bool, msg: str):
        if hasattr(self.planningPanel, "set_preview_ready"):
            self.planningPanel.set_preview_ready(ok, msg)

    # ---------- Render Helper ----------
    def _render_in_preview(self, model, *, sides):
        if hasattr(self.previewPanel, "render_from_model"):
            self.previewPanel.render_from_model(model, sides)

    @staticmethod
    def _unpack_model_sides(args: Tuple[Any, ...]) -> Tuple[Any, list]:
        """
        Erwartete Varianten:
          - (model,)
          - (model, sides)
        Wir geben immer (model, sides_list) zurück; Default sides=["top"].
        """
        if not args:
            raise ValueError("recipeChanged ohne Argumente")
        model = args[0]
        if len(args) >= 2 and isinstance(args[1], (list, tuple)):
            sides = list(args[1]) or ["top"]
        else:
            sides = ["top"]
        return model, sides
