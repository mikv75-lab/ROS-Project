# -*- coding: utf-8 -*-
# File: tabs/recipe/recipe_editor_panel.py
from __future__ import annotations
import os
from typing import Optional, Dict, Any

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLineEdit, QLabel, QFileDialog, QMessageBox, QSizePolicy
)

from app.model.recipe.recipe import Recipe
from app.model.recipe.recipe_store import RecipeStore
from .recipe_editor_content import RecipeEditorContent


class RecipeEditorPanel(QWidget):
    """VBox: Recipe-GB (Buttons + Description) | Content (mit Selectors inkl. Recipe) | UpdatePreview"""
    updatePreviewRequested = pyqtSignal(object)   # emits Recipe

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = RecipeStore.from_ctx(ctx)

        # interne States
        self._active_model: Optional[Recipe] = None
        self._active_rec_def: Optional[Dict[str, Any]] = None
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        # ---------- Root-Layout ----------
        vroot = QVBoxLayout(self)
        vroot.setContentsMargins(8, 8, 8, 8)
        vroot.setSpacing(8)

        # ---------- [1] Recipe GroupBox (Buttons + Description) ----------
        self.gbRecipe = QGroupBox("Recipe", self)
        vroot.addWidget(self.gbRecipe)
        vRecipe = QVBoxLayout(self.gbRecipe)
        vRecipe.setContentsMargins(8, 6, 8, 6)
        vRecipe.setSpacing(6)

        # Buttonzeile
        hButtons = QHBoxLayout()
        self.btnNew    = QPushButton("New", self.gbRecipe)
        self.btnLoad   = QPushButton("Load", self.gbRecipe)
        self.btnSave   = QPushButton("Save", self.gbRecipe)
        self.btnDelete = QPushButton("Delete", self.gbRecipe)
        for b in (self.btnNew, self.btnLoad, self.btnSave, self.btnDelete):
            sp = b.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Maximum)
            sp.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
            b.setSizePolicy(sp)
            hButtons.addWidget(b)
        hButtons.addStretch(1)
        vRecipe.addLayout(hButtons)

        # Description-Feld (gehört jetzt nach oben in die Recipe-Box)
        hDesc = QHBoxLayout()
        self.lblDesc = QLabel("Description:", self.gbRecipe)
        self.e_desc = QLineEdit(self.gbRecipe)
        self.e_desc.setPlaceholderText("Short description of the recipe ...")
        hDesc.addWidget(self.lblDesc)
        hDesc.addWidget(self.e_desc)
        vRecipe.addLayout(hDesc)

        # ---------- [2] Content (enthält Selectors inkl. 'recipe' + Sides-Scroll) ----------
        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        vroot.addWidget(self.content)

        # Defaults ins Formular
        self.content.apply_defaults()

        # ---------- [3] Update Preview ----------
        self.btnUpdatePreview = QPushButton("Update Preview", self)
        sp_upd = self.btnUpdatePreview.sizePolicy()
        sp_upd.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_upd.setVerticalPolicy(QSizePolicy.Policy.Maximum)
        self.btnUpdatePreview.setSizePolicy(sp_upd)
        vroot.addWidget(self.btnUpdatePreview)

        # ---------- Wiring ----------
        self._fill_recipe_select()  # füllt self.content.sel_recipe
        if self.content.sel_recipe is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        self.btnNew.clicked.connect(self._on_new_clicked)
        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnSave.clicked.connect(self._on_save_clicked)
        self.btnDelete.clicked.connect(self._on_delete_clicked)

        self.btnUpdatePreview.clicked.connect(lambda: self.updatePreviewRequested.emit(self.current_model()))

        # Erstes Rezept initialisieren (kein Preview-Emit)
        if self.content.sel_recipe and self.content.sel_recipe.count() > 0:
            if self.content.sel_recipe.currentIndex() < 0:
                self.content.sel_recipe.setCurrentIndex(0)
            self._on_recipe_select_changed(self.content.sel_recipe.currentIndex())

    # ============================ Rezepte ===============================

    def _fill_recipe_select(self) -> None:
        """Füllt die Recipe-Combo in den Selectors (content.sel_recipe)."""
        combo = getattr(self.content, "sel_recipe", None)
        if combo is None:
            return
        self._recipes_by_id.clear()
        combo.blockSignals(True)
        combo.clear()
        for rec_def in self.store.recipes:
            rid = str(rec_def.get("id") or "").strip()
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
        pbs = self.store.build_default_paths_for_recipe(rec_def)

        subs = rec_def.get("substrates") or []
        sub = subs[0] if subs else None
        mounts = rec_def.get("substrate_mounts") or []
        mnt = mounts[0] if mounts else None
        tools = rec_def.get("tools") or []
        tool = tools[0] if tools else None

        model = Recipe.from_dict({
            "id": rec_def.get("id") or "recipe",
            "description": rec_def.get("description") or "",
            "tool": tool,
            "substrate": sub,
            "substrates": [sub] if sub else [],
            "substrate_mount": mnt,
            "parameters": params,
            "paths_by_side": pbs,
        })
        # Kataloge für UI-Coalesce
        model.tools = tools
        model.substrates = subs
        model.substrate_mounts = mounts
        return model

    def _apply_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        self._active_model = model
        self._active_rec_def = rec_def
        # description in Top-Box spiegeln
        self.e_desc.setText(getattr(model, "description", "") or "")
        self.content.apply_recipe_model(model, rec_def)

    def _on_recipe_select_changed(self, _index: int) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        model = self._new_model_from_rec_def(rec_def)
        self._apply_model(model, rec_def)

    # ============================ UI Events =============================

    def _on_new_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if rec_def:
            model = self._new_model_from_rec_def(rec_def)
            self._apply_model(model, rec_def)
        else:
            self.content.apply_defaults()
            self._active_model = None
            self._active_rec_def = None
            self.e_desc.setText("")

    def _on_load_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)
            rec_def = self._recipes_by_id.get(model.id) or self._current_recipe_def() or {}
            self._apply_model(model, rec_def)
            QMessageBox.information(self, "Geladen", os.path.basename(fname))
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_save_clicked(self) -> None:
        try:
            model = self.current_model()
            start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
            suggested = f"{(model.id or 'recipe').strip()}.yaml"
            fname, _ = QFileDialog.getSaveFileName(
                self, "Rezept speichern", os.path.join(start_dir, suggested), "YAML (*.yaml *.yml)"
            )
            if not fname:
                return
            model.save_yaml(fname)
            QMessageBox.information(self, "Gespeichert", os.path.basename(fname))
        except Exception as e:
            QMessageBox.critical(self, "Speicherfehler", str(e))

    def _on_delete_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if rec_def:
            model = self._new_model_from_rec_def(rec_def)
            self._apply_model(model, rec_def)
        else:
            self.content.apply_defaults()
            self._active_model = None
            self._active_rec_def = None
            self.e_desc.setText("")

    # ============================ Export/Trigger ========================

    def current_model(self) -> Recipe:
        params = self.content.collect_globals()
        paths_by_side = self.content.collect_paths_by_side()
        tool, sub, mnt = self.content.active_selectors_values()
        desc = self.e_desc.text().strip()  # <- Description kommt jetzt aus Top-Box

        model = self._active_model
        if model is None:
            rec_def = self._current_recipe_def() or {}
            model = self._new_model_from_rec_def(rec_def)

        model.description = desc
        model.tool = tool
        model.substrate = sub
        model.substrates = [sub] if sub else []
        model.substrate_mount = mnt
        model.parameters = params
        model.paths_by_side = paths_by_side

        self._active_model = model
        return model
