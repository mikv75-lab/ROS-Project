# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import logging
from typing import Optional, Tuple, Any

from PyQt5 import uic
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QWidget, QVBoxLayout
import sip

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .planning_panel.planning_panel import PlanningPanel

_LOG = logging.getLogger("app.tabs.recipe.tab")


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/recipe
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class RecipeTab(QWidget):
    """
    Orchestriert:
      [links]  RecipeEditorPanel
      [mitte]  CoatingPreviewPanel (LAZY, erst wenn Tab sichtbar)
      [rechts] PlanningPanel
    """
    def __init__(self, *, ctx, bridge, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge

        uic.loadUi(_ui_path("recipe_tab.ui"), self)
        for name in ("leftContainer", "plotContainer", "rightTop"):
            if not hasattr(self, name):
                raise RuntimeError(f"recipe_tab.ui erwartet ein QWidget namens '{name}'")

        self.recipePanel  = RecipeEditorPanel(ctx=self.ctx, parent=self)
        self.previewPanel = None  # lazy – siehe _ensure_preview()
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)

        self._mount_into(self.leftContainer, self.recipePanel)
        self._mount_into(self.rightTop,     self.planningPanel)

        # Editor-Signal verbinden (Preview kann noch None sein; wir rendern dann später)
        if hasattr(self.recipePanel, "recipeChanged"):
            self.recipePanel.recipeChanged.connect(self._on_recipe_changed)

        # Falls der Tab via QTabWidget verwaltet wird: Aktivierungs-Hook -> lazy init
        self._activation_timer = None
        self._connect_tab_activation()

    # ---------- Mount ----------
    def _mount_into(self, container: QWidget, widget: QWidget):
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))
        container.layout().addWidget(widget)

    # ---------- Tab-Aktivierung ----------
    def _connect_tab_activation(self):
        # gehe die Eltern hoch und suche QTabWidget
        p = self.parent()
        while p is not None and p.parent() is not None:
            if p.metaObject().className() == "QTabWidget":
                break
            p = p.parent()
        if p is None or p.metaObject().className() != "QTabWidget":
            # kein QTabWidget – Preview beim ersten showEvent initialisieren
            self._schedule_initial_render(delay_ms=150)
            return

        tab_widget = p
        try:
            idx = tab_widget.indexOf(self)
        except Exception:
            idx = -1

        def _on_current_changed(i: int):
            if i == idx:
                # Wir sind jetzt sichtbar → Preview sicherstellen + initial render leicht verzögert
                self._ensure_preview()
                self._schedule_initial_render(delay_ms=120)

        tab_widget.currentChanged.connect(_on_current_changed)

        # Falls der Tab initial bereits aktiv ist:
        try:
            if tab_widget.currentIndex() == idx:
                self._ensure_preview()
                self._schedule_initial_render(delay_ms=120)
        except Exception:
            pass

    # ---------- Preview lazy ----------
    def _ensure_preview(self):
        if self.previewPanel is not None and not sip.isdeleted(self.previewPanel):
            return
        try:
            from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel
            self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
            self._mount_into(self.plotContainer, self.previewPanel)
            if hasattr(self.previewPanel, "readyChanged"):
                self.previewPanel.readyChanged.connect(self._on_preview_ready_changed)

            # Provider in Planning setzen
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

    def _schedule_initial_render(self, *, delay_ms: int = 120):
        if self._activation_timer is not None:
            self._activation_timer.stop()
            self._activation_timer.deleteLater()
            self._activation_timer = None

        self._activation_timer = QTimer(self)
        self._activation_timer.setSingleShot(True)

        def _do():
            self._activation_timer = None
            try:
                self._initial_render()
            except Exception as e:
                _LOG.critical("Initiales Rendern fehlgeschlagen: %s", e, exc_info=True)
                try:
                    if self.previewPanel and hasattr(self.previewPanel, "readyChanged"):
                        self.previewPanel.readyChanged.emit(False, f"initial render failed: {e}")
                except Exception:
                    pass

        self._activation_timer.timeout.connect(_do)
        self._activation_timer.start(delay_ms)

    def _initial_render(self):
        if self.previewPanel is None or sip.isdeleted(self.previewPanel):
            self._ensure_preview()
            if self.previewPanel is None:
                return
        model = self.recipePanel.current_model()
        pbs = getattr(model, "paths_by_side", None) or (model.get("paths_by_side") if isinstance(model, dict) else None)
        if not isinstance(pbs, dict) or not pbs:
            raise RuntimeError("Initiales Rendern: paths_by_side fehlt oder ist leer.")
        sides = list(pbs.keys())
        self._render_in_preview(model, sides=sides)

    # ---------- Slots ----------
    def _on_recipe_changed(self, *args: Any):
        try:
            model, sides = self._unpack_model_sides(args)
            pbs = getattr(model, "paths_by_side", None) or (model.get("paths_by_side") if isinstance(model, dict) else None)
            if not isinstance(pbs, dict) or not pbs:
                raise RuntimeError("recipeChanged: paths_by_side fehlt oder ist leer.")
            if not sides:
                sides = list(pbs.keys())
                if not sides:
                    raise RuntimeError("recipeChanged: keine Side angegeben & keine Sides im Modell.")

            # Preview sicherstellen; wenn noch nicht da, wird Render beim Aktivieren nachgeholt
            self._ensure_preview()
            self._render_in_preview(model, sides=sides)

            if hasattr(self.planningPanel, "set_model_provider"):
                self.planningPanel.set_model_provider(self.recipePanel.current_model)
            if hasattr(self.planningPanel, "set_traj_provider") and self.previewPanel is not None:
                self.planningPanel.set_traj_provider(getattr(self.previewPanel, "last_trajectory_dict", lambda: None))
            if hasattr(self.planningPanel, "set_preview_ready"):
                self.planningPanel.set_preview_ready(True, "preview updated")

        except Exception as e:
            _LOG.error("on_recipe_changed failed: %s", e, exc_info=True)
            try:
                if hasattr(self.planningPanel, "set_preview_ready"):
                    self.planningPanel.set_preview_ready(False, str(e))
            except Exception:
                pass

    def _on_preview_ready_changed(self, ok: bool, msg: str):
        if hasattr(self.planningPanel, "set_preview_ready"):
            self.planningPanel.set_preview_ready(ok, msg)

    # ---------- Render Helper ----------
    def _render_in_preview(self, model, *, sides):
        if self.previewPanel is None or sip.isdeleted(self.previewPanel):
            return
        try:
            if hasattr(self.previewPanel, "render_from_model"):
                self.previewPanel.render_from_model(model, sides)
        except Exception as e:
            _LOG.error("render_in_preview failed: %s", e, exc_info=True)
            raise

    @staticmethod
    def _unpack_model_sides(args: Tuple[Any, ...]) -> Tuple[Any, list]:
        if not args:
            raise ValueError("recipeChanged ohne Argumente")
        model = args[0]
        if len(args) >= 2 and isinstance(args[1], (list, tuple)):
            sides = list(args[1])
        else:
            sides = []  # keine Default-Side; Validierung greift
        return model, sides
