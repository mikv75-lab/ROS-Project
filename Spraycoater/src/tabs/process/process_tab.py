# -*- coding: utf-8 -*-
# File: src/tabs/process/process_tab.py
from __future__ import annotations

from typing import Optional, Any

from PyQt6 import QtCore
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QSizePolicy

from plc.plc_client import PlcClientBase
from ros.bridge.ros_bridge import RosBridge

from .recipe_panel.recipe_panel import RecipePanel
from .process_panel.process_panel import ProcessPanel


class ProcessTab(QWidget):
    """
    STRICT split:

      - ProcessTab hosts exactly two panels:
          * RecipePanel (load + UI + postprocess/eval/persist on finished)
          * ProcessPanel (threads + buttons + runresult creation only)

      - ProcessTab contains no process logic.
      - Panels communicate exclusively via Qt signals/slots.

    RecipePanel API (STRICT):
      - sig_recipe_selected(str key, object recipe_model)
      - sig_recipe_cleared()
      - slot: on_run_started(str mode, str key)
      - slot: on_run_finished(str key, object rr)
      - slot: on_run_error(str key, str message)

    ProcessPanel API (STRICT):
      - slot: set_recipe(str key, object recipe_model)
      - slot: clear_recipe()
      - signals: sig_run_started(str mode, str key)
                 sig_run_finished(str key, object rr)
                 sig_run_error(str key, str message)
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
            raise RuntimeError("ProcessTab: ctx is None (strict)")
        if repo is None:
            raise RuntimeError("ProcessTab: repo is None (strict)")
        if ros is None:
            raise RuntimeError("ProcessTab: ros is None (strict)")

        self.ctx = ctx
        self.repo = repo
        self.ros = ros
        self.plc = plc

        self.recipePanel = RecipePanel(ctx=self.ctx, repo=self.repo, ros=self.ros, parent=self)
        self.processPanel = ProcessPanel(ctx=self.ctx, ros=self.ros, plc=self.plc, parent=self)

        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(8)

        root.addWidget(self.recipePanel, 3)
        root.addWidget(self.processPanel, 2)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Expanding)
        self.setSizePolicy(sp)

        self._wire_panels_strict()

    def _wire_panels_strict(self) -> None:
        rp = self.recipePanel
        pp = self.processPanel

        # RecipePanel -> ProcessPanel
        sig_recipe_selected = getattr(rp, "sig_recipe_selected", None)
        if not isinstance(sig_recipe_selected, QtCore.pyqtBoundSignal):
            raise RuntimeError("ProcessTab: RecipePanel missing sig_recipe_selected(key, recipe_model) (strict)")
        sig_recipe_cleared = getattr(rp, "sig_recipe_cleared", None)
        if not isinstance(sig_recipe_cleared, QtCore.pyqtBoundSignal):
            raise RuntimeError("ProcessTab: RecipePanel missing sig_recipe_cleared() (strict)")

        if not callable(getattr(pp, "set_recipe", None)):
            raise RuntimeError("ProcessTab: ProcessPanel missing set_recipe(key, recipe_model) (strict)")
        if not callable(getattr(pp, "clear_recipe", None)):
            raise RuntimeError("ProcessTab: ProcessPanel missing clear_recipe() (strict)")

        sig_recipe_selected.connect(pp.set_recipe)
        sig_recipe_cleared.connect(pp.clear_recipe)

        # ProcessPanel -> RecipePanel
        sig_run_started = getattr(pp, "sig_run_started", None)
        if not isinstance(sig_run_started, QtCore.pyqtBoundSignal):
            raise RuntimeError("ProcessTab: ProcessPanel missing sig_run_started(mode, key) (strict)")
        sig_run_finished = getattr(pp, "sig_run_finished", None)
        if not isinstance(sig_run_finished, QtCore.pyqtBoundSignal):
            raise RuntimeError("ProcessTab: ProcessPanel missing sig_run_finished(key, rr) (strict)")
        sig_run_error = getattr(pp, "sig_run_error", None)
        if not isinstance(sig_run_error, QtCore.pyqtBoundSignal):
            raise RuntimeError("ProcessTab: ProcessPanel missing sig_run_error(key, message) (strict)")

        if not callable(getattr(rp, "on_run_started", None)):
            raise RuntimeError("ProcessTab: RecipePanel missing on_run_started(mode, key) (strict)")
        if not callable(getattr(rp, "on_run_finished", None)):
            raise RuntimeError("ProcessTab: RecipePanel missing on_run_finished(key, rr) (strict)")
        if not callable(getattr(rp, "on_run_error", None)):
            raise RuntimeError("ProcessTab: RecipePanel missing on_run_error(key, message) (strict)")

        sig_run_started.connect(rp.on_run_started)
        sig_run_finished.connect(rp.on_run_finished)
        sig_run_error.connect(rp.on_run_error)