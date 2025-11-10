# -*- coding: utf-8 -*-
from __future__ import annotations
import os, re
from typing import Optional, Dict, Any, List

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox,
    QPushButton, QLineEdit, QLabel, QFileDialog, QMessageBox, QSizePolicy, QFormLayout
)

from app.model.recipe.recipe import Recipe
from app.model.recipe.recipe_store import RecipeStore
from .recipe_editor_content import RecipeEditorContent  # erwartet collect_* / apply_* Shims

def _slugify(s: str) -> str:
    s = s.strip()
    s = re.sub(r"\s+", "_", s)
    s = re.sub(r"[^-a-zA-Z0-9_]", "", s)
    s = re.sub(r"_+", "_", s)
    return s

class RecipeEditorPanel(QWidget):
    """Layout:
        [ Commands ]                 -> New | Load | Save | Delete
        [ Name & Description ]       -> Name(LineEdit) + Description(LineEdit)
        [ Content ]                  -> Globals / Selectors / Planner / SideTabs
        [ Update Preview ]
    """
    updatePreviewRequested = pyqtSignal(object)

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

        # ---------- [1] Commands ----------
        self.gbCommands = QGroupBox("Commands", self)
        vroot.addWidget(self.gbCommands)
        hButtons = QHBoxLayout(self.gbCommands)
        hButtons.setContentsMargins(8, 6, 8, 6)
        hButtons.setSpacing(6)

        self.btnNew    = QPushButton("New", self.gbCommands)
        self.btnLoad   = QPushButton("Load", self.gbCommands)
        self.btnSave   = QPushButton("Save", self.gbCommands)
        self.btnDelete = QPushButton("Delete", self.gbCommands)

        for b in (self.btnNew, self.btnLoad, self.btnSave, self.btnDelete):
            sp = b.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Maximum)
            sp.setHorizontalPolicy(QSizePolicy.Expanding)
            b.setSizePolicy(sp)
            hButtons.addWidget(b, 1)

        # ---------- [2] Name & Description ----------
        self.gbMeta = QGroupBox("Name & Description", self)
        vroot.addWidget(self.gbMeta)
        metaForm = QFormLayout(self.gbMeta)
        metaForm.setContentsMargins(8, 6, 8, 6)
        metaForm.setSpacing(6)

        self.e_name = QLineEdit(self.gbMeta)
        self.e_name.setPlaceholderText("recipe_name (Dateiname & YAML id – wird nur beim Save verwendet)")
        self.e_desc = QLineEdit(self.gbMeta)
        self.e_desc.setPlaceholderText("Short description of the recipe ...")

        metaForm.addRow(QLabel("Name:", self.gbMeta), self.e_name)
        metaForm.addRow(QLabel("Description:", self.gbMeta), self.e_desc)

        # ---------- [3] Content ----------
        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        vroot.addWidget(self.content)
        self.content.apply_defaults()

        # ---------- [4] Update Preview ----------
        self.btnUpdatePreview = QPushButton("Update Preview", self)
        sp_upd = self.btnUpdatePreview.sizePolicy()
        sp_upd.setHorizontalPolicy(QSizePolicy.Expanding)
        sp_upd.setVerticalPolicy(QSizePolicy.Maximum)
        self.btnUpdatePreview.setSizePolicy(sp_upd)
        vroot.addWidget(self.btnUpdatePreview)

        # ---------- Wiring ----------
        self._fill_recipe_select()
        if self.content.sel_recipe is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        self.btnNew.clicked.connect(self._on_new_clicked)
        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnSave.clicked.connect(self._on_save_clicked)
        self.btnDelete.clicked.connect(self._on_delete_clicked)

        def _emit_update():
            payload = {
                "model": self.current_model(),
                "sides": self._selected_existing_sides()
            }
            self.updatePreviewRequested.emit(payload)
        self.btnUpdatePreview.clicked.connect(_emit_update)

        # Erstes Rezept initialisieren
        if self.content.sel_recipe and self.content.sel_recipe.count() > 0:
            if self.content.sel_recipe.currentIndex() < 0:
                self.content.sel_recipe.setCurrentIndex(0)
            self._on_recipe_select_changed(self.content.sel_recipe.currentIndex())

    # ============================ Rezepte ===============================

    def _fill_recipe_select(self) -> None:
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
        params  = self.store.collect_global_defaults()
        planner = self.store.collect_planner_defaults()
        pbs     = self.store.build_default_paths_for_recipe(rec_def)

        subs  = rec_def.get("substrates") or []
        sub   = subs[0] if subs else None
        mounts= rec_def.get("substrate_mounts") or []
        mnt   = mounts[0] if mounts else None
        tools = rec_def.get("tools") or []
        tool  = tools[0] if tools else None

        model = Recipe.from_dict({
            "id": "",
            "description": rec_def.get("description") or "",
            "tool": tool,
            "substrate": sub,
            "substrates": [sub] if sub else [],
            "substrate_mount": mnt,
            "parameters": params,
            "planner": planner,
            "paths_by_side": pbs,
        })

        # Optional (nur für UI-Komfort):
        model.tools = tools                     # type: ignore[attr-defined]
        model.substrates = subs                 # type: ignore[assignment]
        model.substrate_mounts = mounts         # type: ignore[attr-defined]
        return model

    def _apply_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        self._active_model = model
        self._active_rec_def = rec_def
        self.e_name.setText(getattr(model, "id", "") or "")
        self.e_desc.setText(getattr(model, "description", "") or "")
        self.content.apply_recipe_model(model, rec_def)
        apply_planner = getattr(self.content, "apply_planner_model", None)
        if callable(apply_planner):
            apply_planner(model.planner or {})

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
            self.e_name.setText("")
            self.e_desc.setText("")

    def _on_load_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)
            self.e_name.setText(model.id or "")
            self.e_desc.setText(getattr(model, "description", "") or "")
            rec_def = self._recipes_by_id.get(model.id) or self._current_recipe_def() or {}
            self._apply_model(model, rec_def)
            QMessageBox.information(self, "Geladen", os.path.basename(fname))
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_save_clicked(self) -> None:
        try:
            model = self.current_model()
            name = _slugify((self.e_name.text() or "").strip())
            if not name:
                QMessageBox.warning(self, "Name fehlt", "Bitte zuerst einen Recipe-Namen eingeben.")
                return
            model.id = name

            start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
            suggested = f"{name}.yaml"
            fname, _ = QFileDialog.getSaveFileName(
                self, "Rezept speichern", os.path.join(start_dir, suggested), "YAML (*.yaml *.yml)")
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
            self.e_name.setText("")
            self.e_desc.setText("")

    # ============================ Export/Trigger ========================

    def current_model(self) -> Recipe:
        """
        Sammelt UI -> Model, fasst 'id' NICHT an (id wird nur beim Save gesetzt).
        """
        collect_planner = getattr(self.content, "collect_planner", lambda: {})
        params         = self.content.collect_globals()
        paths_by_side  = self.content.collect_paths_by_side()
        planner        = collect_planner()
        tool, sub, mnt = self.content.active_selectors_values()
        desc           = self.e_desc.text().strip()

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
        model.planner = planner
        model.paths_by_side = paths_by_side

        self._active_model = model
        return model

    def _selected_existing_sides(self) -> List[str]:
        rec_def = self._current_recipe_def() or {}
        defined = set(((rec_def.get("sides") or {}).keys()))
        try:
            checked = self.content.checked_sides()
        except Exception:
            checked = []
        return [s for s in checked if s in defined]
