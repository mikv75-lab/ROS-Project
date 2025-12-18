# -*- coding: utf-8 -*-
from __future__ import annotations

import os
import re
from typing import Optional, Dict, Any

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QPushButton,
    QFileDialog,
    QMessageBox,
    QSizePolicy,
    QFrame,
)

from app.model.recipe.recipe import Recipe
from app.model.recipe.recipe_store import RecipeStore
from .recipe_editor_content import RecipeEditorContent


def _slugify(s: str) -> str:
    s = (s or "").strip()
    s = re.sub(r"\s+", "_", s)
    s = re.sub(r"[^-a-zA-Z0-9_]", "", s)
    s = re.sub(r"_+", "_", s)
    return s


def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setFrameShadow(QFrame.Sunken)
    return f


def _set_policy(
    w: QWidget,
    *,
    h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
    v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


class RecipeEditorPanel(QWidget):
    """
    Commands:
      Row 1: New | Load | Save | Delete
      Row 2: Divider
      Row 3: Update Preview | Validate | Optimize

    Content:
      RecipeEditorContent (Meta/Setup | Globals/Move Planner | Sides)
    """

    updatePreviewRequested = pyqtSignal(object)  # model: Recipe
    validateRequested = pyqtSignal()
    optimizeRequested = pyqtSignal()

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        self._active_model: Optional[Recipe] = None
        self._active_rec_def: Optional[Dict[str, Any]] = None
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        vroot = QVBoxLayout(self)
        vroot.setContentsMargins(8, 8, 8, 8)
        vroot.setSpacing(8)

        # ---------------- Commands ----------------
        self.gbCommands = QGroupBox("Commands", self)
        _set_policy(self.gbCommands, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)

        cmd_layout = QVBoxLayout(self.gbCommands)
        cmd_layout.setContentsMargins(8, 6, 8, 6)
        cmd_layout.setSpacing(6)

        # Row 1
        h_buttons = QHBoxLayout()
        h_buttons.setContentsMargins(0, 0, 0, 0)
        h_buttons.setSpacing(6)

        self.btnNew = QPushButton("New", self.gbCommands)
        self.btnLoad = QPushButton("Load", self.gbCommands)
        self.btnSave = QPushButton("Save", self.gbCommands)
        self.btnDelete = QPushButton("Delete", self.gbCommands)

        for b in (self.btnNew, self.btnLoad, self.btnSave, self.btnDelete):
            b.setAutoDefault(False)
            _set_policy(b, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
            h_buttons.addWidget(b, 1)

        # Row 3
        h_footer = QHBoxLayout()
        h_footer.setContentsMargins(0, 0, 0, 0)
        h_footer.setSpacing(8)

        self.btnUpdatePreview = QPushButton("Update Preview", self.gbCommands)
        self.btnValidate = QPushButton("Validate", self.gbCommands)
        self.btnOptimize = QPushButton("Optimize", self.gbCommands)

        for b in (self.btnUpdatePreview, self.btnValidate, self.btnOptimize):
            b.setAutoDefault(False)
            _set_policy(b, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
            h_footer.addWidget(b, 1)

        cmd_layout.addLayout(h_buttons)
        cmd_layout.addWidget(_hline())
        cmd_layout.addLayout(h_footer)

        vroot.addWidget(self.gbCommands, 0)

        # ---------------- Content ----------------
        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        _set_policy(self.content, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)
        vroot.addWidget(self.content, 1)

        # ---------------- Wiring ----------------
        self._fill_recipe_select()
        if self.content.sel_recipe is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        self.btnNew.clicked.connect(self._on_new_clicked)
        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnSave.clicked.connect(self._on_save_clicked)
        self.btnDelete.clicked.connect(self._on_delete_clicked)

        self.btnValidate.clicked.connect(self.validateRequested.emit)
        self.btnOptimize.clicked.connect(self.optimizeRequested.emit)
        self.btnUpdatePreview.clicked.connect(self._relay_update_preview)

        # Init first recipe (if present)
        combo = getattr(self.content, "sel_recipe", None)
        if combo is not None and combo.count() > 0:
            if combo.currentIndex() < 0:
                combo.setCurrentIndex(0)
            self._on_recipe_select_changed(combo.currentIndex())

    # ============================ recipes ============================

    def _fill_recipe_select(self) -> None:
        combo = getattr(self.content, "sel_recipe", None)
        if combo is None:
            return

        self._recipes_by_id.clear()
        combo.blockSignals(True)
        combo.clear()

        for rec_def in (self.store.recipes or []):
            rid = str((rec_def or {}).get("id") or "").strip()
            if not rid:
                continue
            combo.addItem(rid)
            self._recipes_by_id[rid] = rec_def

        combo.blockSignals(False)

    def _current_recipe_def(self) -> Optional[Dict[str, Any]]:
        combo = getattr(self.content, "sel_recipe", None)
        if combo is None:
            return None
        rid = combo.currentText().strip()
        return self._recipes_by_id.get(rid)

    def _new_model_from_rec_def(self, rec_def: Dict[str, Any]) -> Recipe:
        params = self.store.collect_global_defaults()
        move_planner = self.store.collect_planner_defaults()
        pbs = self.store.build_default_paths_for_recipe(rec_def)

        subs = rec_def.get("substrates") or []
        sub = subs[0] if subs else None
        mounts = rec_def.get("substrate_mounts") or []
        mnt = mounts[0] if mounts else None
        tools = rec_def.get("tools") or []
        tool = tools[0] if tools else None

        return Recipe.from_dict({
            "id": "",
            "description": rec_def.get("description") or "",
            "tool": tool,
            "substrate": sub,
            "substrate_mount": mnt,
            "parameters": params,
            "planner": move_planner,     # aktuell: Move-Planner
            "paths_by_side": pbs,
        })

    def _apply_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        self._active_model = model
        self._active_rec_def = rec_def

        self.content.set_meta(name=model.id or "", desc=model.description or "")
        self.content.apply_recipe_model(model, rec_def)
        self.content.apply_planner_model(model.planner or {})

    def _on_recipe_select_changed(self, _index: int) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        self._apply_model(self._new_model_from_rec_def(rec_def), rec_def)

    # ============================ UI events ============================

    def _on_new_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            self.content.apply_defaults()
            self._active_model = None
            self._active_rec_def = None
            self.content.set_meta(name="", desc="")
            return

        self._apply_model(self._new_model_from_rec_def(rec_def), rec_def)

    def _on_load_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return

        # 1) Load full model
        model = Recipe.load_yaml(fname)
        if not model.id:
            raise ValueError("Geladenes Rezept hat kein 'id'-Feld.")

        # 2) Find matching definition strictly by model.id
        rec_def = self._recipes_by_id.get(model.id)
        if not rec_def:
            raise ValueError(
                f"Kein recipes[..].id == '{model.id}' gefunden. "
                f"Bitte ein Rezept mit gültiger ID laden."
            )

        # 3) Set recipe selection (without triggering auto-new)
        combo = getattr(self.content, "sel_recipe", None)
        if combo is not None:
            idx = combo.findText(str(model.id))
            if idx >= 0:
                combo.blockSignals(True)
                combo.setCurrentIndex(idx)
                combo.blockSignals(False)

        # 4) Apply model
        self._apply_model(model, rec_def)

        # 5) Update preview
        self.updatePreviewRequested.emit(model)

        QMessageBox.information(self, "Geladen", os.path.basename(fname))

    def _on_save_clicked(self) -> None:
        model = self.current_model()

        name_inp, _desc_inp = self.content.meta_values()
        name = _slugify((name_inp or "").strip())
        if not name:
            QMessageBox.warning(self, "Name fehlt", "Bitte zuerst einen Recipe-Namen eingeben.")
            return

        model.id = name

        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        suggested = f"{name}.yaml"
        fname, _ = QFileDialog.getSaveFileName(
            self,
            "Rezept speichern",
            os.path.join(start_dir, suggested),
            "YAML (*.yaml *.yml)",
        )
        if not fname:
            return

        model.save_yaml(fname)
        QMessageBox.information(self, "Gespeichert", os.path.basename(fname))

    def _on_delete_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            self.content.apply_defaults()
            self._active_model = None
            self._active_rec_def = None
            self.content.set_meta(name="", desc="")
            return

        self._apply_model(self._new_model_from_rec_def(rec_def), rec_def)

    # ============================ export / trigger ============================

    def current_model(self) -> Recipe:
        """
        Builds a Recipe from current UI state (globals, move-planner, paths_by_side).
        """
        params = self.content.collect_globals()
        paths_by_side = self.content.collect_paths_by_side()
        move_planner = self.content.collect_planner()
        tool, sub, mnt = self.content.active_selectors_values()
        name, desc = self.content.meta_values()

        model = self._active_model
        if model is None:
            rec_def = self._current_recipe_def() or {}
            model = self._new_model_from_rec_def(rec_def)

        model.id = (name or "").strip()
        model.description = (desc or "").strip()
        model.tool = tool
        model.substrate = sub
        model.substrate_mount = mnt
        model.parameters = params
        model.planner = move_planner
        model.paths_by_side = paths_by_side

        self._active_model = model
        return model

    def _relay_update_preview(self) -> None:
        self.updatePreviewRequested.emit(self.current_model())
