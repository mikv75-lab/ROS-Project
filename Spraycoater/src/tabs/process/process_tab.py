# -*- coding: utf-8 -*-
# File: src/tabs/process/process_tab.py
from __future__ import annotations

import logging
from typing import Any, Optional

from PyQt6 import QtCore
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from .recipe_panel.recipe_panel import RecipePanel
from .process_panel.process_panel import ProcessPanel

_LOG = logging.getLogger("tabs.process")


class ProcessTab(QWidget):
    """
    STRICT V2 ProcessTab (Option 2)

    Responsibilities:
      - Layout panels
      - Wire signals between RecipePanel and ProcessPanel

    ProcessPanel:
      - Starts/stops ProcessThread
      - Emits sig_run_started/sig_run_finished/sig_run_error

    RecipePanel:
      - Selects/loads recipes via repo
      - Displays recipe + stored results
      - Handles on_run_started/on_run_finished/on_run_error
      - Persists via repo.save_run_result_if_valid(...)
    """

    def __init__(
        self,
        *,
        ctx: Any,
        repo: Any,
        ros: RosBridge,
        plc: PlcClientBase | None = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        if ctx is None:
            raise RuntimeError("ProcessTab(V2): ctx ist None (strict)")
        if repo is None:
            raise RuntimeError("ProcessTab(V2): repo ist None (strict)")
        if ros is None:
            raise RuntimeError("ProcessTab(V2): ros ist None (strict)")

        # Basic repo validations (strict-minimum)
        for name in ("list_recipes", "load_for_process"):
            fn = getattr(repo, name, None)
            if not callable(fn):
                raise RuntimeError(f"ProcessTab(V2): repo missing required API: {name}()")
        if not callable(getattr(repo, "save_run_result_if_valid", None)):
            _LOG.warning("ProcessTab(V2): repo.save_run_result_if_valid() fehlt – Persist wird im RecipePanel fehlschlagen.")

        self.ctx = ctx
        self.repo = repo
        self.ros: RosBridge = ros
        self.plc = plc

        # ---------------- UI ----------------
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        row = QHBoxLayout()
        row.setSpacing(8)

        self.recipePanel = RecipePanel(ctx=self.ctx, repo=self.repo, ros=self.ros, parent=self)
        self.processPanel = ProcessPanel(repo=self.repo, ctx=self.ctx, ros=self.ros, plc=self.plc, parent=self)
        row.addWidget(self.recipePanel, 3)
        row.addWidget(self.processPanel, 2)

        root.addLayout(row, 1)

        # ---------------- Wiring ----------------

        # Recipe selection -> ProcessPanel context
        self.recipePanel.sig_recipe_selected.connect(self._on_recipe_selected)
        self.recipePanel.sig_recipe_cleared.connect(self._on_recipe_cleared)

        # Process lifecycle -> RecipePanel
        self.processPanel.sig_run_started.connect(self.recipePanel.on_run_started)
        self.processPanel.sig_run_finished.connect(self.recipePanel.on_run_finished)
        self.processPanel.sig_run_error.connect(self.recipePanel.on_run_error)

    # ---------------- Slots ----------------

    @QtCore.pyqtSlot(str, object)
    def _on_recipe_selected(self, key: str, model: object) -> None:
        try:
            self.processPanel.set_recipe(key, model)  # RecipePanel emits (key, Recipe)
        except Exception as e:
            _LOG.exception("ProcessTab(V2): processPanel.set_recipe failed: %s", e)

    @QtCore.pyqtSlot()
    def _on_recipe_cleared(self) -> None:
        try:
            self.processPanel.clear_recipe()
        except Exception as e:
            _LOG.exception("ProcessTab(V2): processPanel.clear_recipe failed: %s", e)
