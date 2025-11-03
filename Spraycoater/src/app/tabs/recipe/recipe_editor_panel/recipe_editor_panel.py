# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from typing import Optional, Dict, Any

from PyQt6 import uic
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QFileDialog, QMessageBox, QVBoxLayout

from app.model.recipe.recipe import Recipe
from app.model.recipe.recipe_store import RecipeStore
from .recipe_editor_content import RecipeEditorContent


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class RecipeEditorPanel(QWidget):
    """Topbar + RecipeEditorContent. Arbeitet mit genau EINEM aktiven Recipe-Objekt."""
    updatePreviewRequested = pyqtSignal(object)  # emits Recipe

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = RecipeStore.from_ctx(ctx)
        uic.loadUi(_ui_path("recipe_editor_panel.ui"), self)

        # Aktives Model/Def
        self._active_model: Optional[Recipe] = None
        self._active_rec_def: Optional[Dict[str, Any]] = None

        # Content-Widget montieren
        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        host_layout = self.contentHost.layout()
        if host_layout is None:
            host_layout = QVBoxLayout()
            host_layout.setContentsMargins(0, 0, 0, 0)
            self.contentHost.setLayout(host_layout)
        if host_layout.indexOf(self.content) == -1:
            if self.content.parent() is not self.contentHost:
                self.content.setParent(self.contentHost)
            host_layout.addWidget(self.content)

        # Rezepte füllen
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}
        if hasattr(self, "comboRecipeSelect"):
            self.comboRecipeSelect.setVisible(True)
            self._fill_recipe_select()
            self.comboRecipeSelect.currentIndexChanged.connect(self._on_recipe_select_changed)

        # Defaults ins Formular
        self.content.apply_defaults()

        # Buttons
        self.btnNew.clicked.connect(self._on_new_clicked)
        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnDelete.clicked.connect(self._on_delete_clicked)
        self.btnUpdatePreview.clicked.connect(self._on_update_preview_clicked)

        # Initial: erstes Rezept in Formular (kein Preview-Emit)
        if hasattr(self, "comboRecipeSelect") and self.comboRecipeSelect.count() > 0:
            if self.comboRecipeSelect.currentIndex() < 0:
                self.comboRecipeSelect.setCurrentIndex(0)
            self._on_recipe_select_changed(self.comboRecipeSelect.currentIndex())

    # --- Rezepte ---
    def _fill_recipe_select(self) -> None:
        self._recipes_by_id.clear()
        self.comboRecipeSelect.blockSignals(True)
        self.comboRecipeSelect.clear()
        for rec_def in self.store.recipes:
            rid = str(rec_def.get("id") or "").strip()
            if not rid:
                continue
            self.comboRecipeSelect.addItem(rid)
            self._recipes_by_id[rid] = rec_def
        self.comboRecipeSelect.blockSignals(False)

    def _current_recipe_def(self) -> Optional[Dict[str, Any]]:
        if not hasattr(self, "comboRecipeSelect"):
            return None
        rid = self.comboRecipeSelect.currentText().strip()
        return self._recipes_by_id.get(rid)

    def _new_model_from_rec_def(self, rec_def: Dict[str, Any]) -> Recipe:
        """
        Erstellt ein frisch vorinitialisiertes Recipe:
        - parameters: globale Defaults aus recipe_params.globals
        - paths_by_side: default_path je Side aus Definition
        - tool/substrate/mount: jeweils erstes gültiges Element (falls vorhanden)
        """
        # Basis aus Definition
        base = dict(rec_def)

        # Globale Defaults
        params = self.store.collect_global_defaults()

        # Default-Paths je Side
        pbs = self.store.build_default_paths_for_recipe(rec_def)

        # Erstes Element für Auswahlfelder
        tool = None
        subs = rec_def.get("substrates") or []
        sub = subs[0] if subs else None
        mounts = rec_def.get("substrate_mounts") or []
        mnt = mounts[0] if mounts else None
        tools = rec_def.get("tools") or []
        tool = tools[0] if tools else None

        # Zusammenbauen
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

        # Lege Katalogfelder am Model ab (damit _coalesce_options sie direkt findet)
        model.tools = tools
        model.substrates = subs
        model.substrate_mounts = mounts
        return model

    def _apply_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        """Zentrale Stelle, um das aktive Model/Def zu setzen und in die UI zu bringen."""
        self._active_model = model
        self._active_rec_def = rec_def
        self.content.apply_recipe_model(model, rec_def)

    def _on_recipe_select_changed(self, _index: int) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        model = self._new_model_from_rec_def(rec_def)
        self._apply_model(model, rec_def)  # kein emit

    # --- UI Events ---
    def _on_new_clicked(self) -> None:
        """Neues (frisches) Model aus der aktuell gewählten Store-Definition laden."""
        rec_def = self._current_recipe_def()
        if rec_def:
            model = self._new_model_from_rec_def(rec_def)
            self._apply_model(model, rec_def)  # kein emit
        else:
            # Fallback: nur Defaults im Formular zeigen
            self.content.apply_defaults()
            self._active_model = None
            self._active_rec_def = None

    def _on_load_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)
            # passende Definition (falls vorhanden) zur UI-Belegung finden
            rec_def = self._recipes_by_id.get(model.id) or self._current_recipe_def() or {}
            self._apply_model(model, rec_def)  # Formular komplett aus dem geladenen Model
            QMessageBox.information(self, "Geladen", os.path.basename(fname))
            # kein emit
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_delete_clicked(self) -> None:
        """Zurück auf die Store-Defaults des aktuell ausgewählten Rezepts."""
        rec_def = self._current_recipe_def()
        if rec_def:
            model = self._new_model_from_rec_def(rec_def)
            self._apply_model(model, rec_def)  # kein emit
        else:
            self.content.apply_defaults()
            self._active_model = None
            self._active_rec_def = None

    # --- Export / Trigger ---
    def current_model(self) -> Recipe:
        """
        Liest die aktuellen UI-Werte und erzeugt ein Recipe-Objekt.
        Wenn bereits ein aktives Model existiert, werden dessen Felder durch UI-States überschrieben.
        """
        params = self.content.collect_globals()
        paths_by_side = self.content.collect_paths_by_side()
        tool, sub, mnt = self.content.active_selectors_values()
        desc = self.content.get_description()

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

    def _on_update_preview_clicked(self) -> None:
        self.updatePreviewRequested.emit(self.current_model())
