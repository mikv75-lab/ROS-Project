# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from typing import Any, Dict, Optional, List, Tuple

from PyQt5 import uic
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QFileDialog, QMessageBox, QVBoxLayout

from .recipe_model import Recipe                     # src/app/tabs/recipe/recipe_editor_panel/recipe_model.py
from .recipe_editor_content import RecipeEditorContent  # src/app/tabs/recipe/recipe_editor_panel/recipe_editor_content.py


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/recipe/recipe_editor_panel
    return os.path.abspath(os.path.join(here, "..", "..", "..", "..", ".."))


def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)


class RecipeEditorPanel(QWidget):
    """Linkes Panel: Topbar + dynamischer Inhalt (RecipeEditorContent)."""
    recipeChanged = pyqtSignal(object)  # emits Recipe (model)

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        uic.loadUi(_ui_path("recipe_editor_panel.ui"), self)

        # --- Content-Widget bauen & einhängen ---
        self.content = RecipeEditorContent(ctx=self.ctx, parent=self)
        host_layout = self.contentHost.layout()
        if host_layout is None:
            host_layout = QVBoxLayout(self.contentHost)
        host_layout.addWidget(self.content)

        # Defaults setzen
        self.content.apply_defaults()

        # --- Events ---
        if self.content.type_combo is not None:
            self.content.type_combo.currentIndexChanged.connect(self._on_type_changed)
        self.btnNew.clicked.connect(self._on_new_clicked)
        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnDelete.clicked.connect(self._on_delete_clicked)
        self.btnUpdatePreview.clicked.connect(self._emit_recipe_changed)

        # --- Rezepte in die Auswahl füllen ---
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}
        self._fill_recipe_select()
        self.comboRecipeSelect.currentIndexChanged.connect(self._on_recipe_select_changed)

        # initial
        if self.comboRecipeSelect.count() > 0:
            self._on_recipe_select_changed(self.comboRecipeSelect.currentIndex())

    # ---------------------------------------------------------------------
    # Helpers: Rezepte / YAML
    # ---------------------------------------------------------------------
    def _fill_recipe_select(self) -> None:
        self.comboRecipeSelect.clear()
        recipes = list(getattr(self.ctx, "recipes_yaml", {}).get("recipes", []) or [])
        self._recipes_by_id = {}
        for rec in recipes:
            rid = str(rec.get("id") or "").strip()
            if not rid:
                continue
            self.comboRecipeSelect.addItem(rid)
            self._recipes_by_id[rid] = rec

    def _find_recipe_by_id(self, rid: str) -> Optional[Dict[str, Any]]:
        return self._recipes_by_id.get(rid)

    @staticmethod
    def _split_type_mode(t: str, fallback_mode: str = "plane") -> Tuple[str, str]:
        """
        Unterstützt:
          - 'meander_plane' / 'spiral_cylinder'
          - oder alt: {'type':'meander','mode':'plane'}
        """
        if not t:
            return "meander", fallback_mode
        if "_" in t:
            a, b = t.split("_", 1)
            return a, b
        return t, fallback_mode

    # ---------------------------------------------------------------------
    # UI Events
    # ---------------------------------------------------------------------
    def _on_recipe_select_changed(self, _i: int) -> None:
        rid = self.comboRecipeSelect.currentText().strip()
        rec = self._find_recipe_by_id(rid)
        if not rec:
            return
        # type/mode aus YAML
        p = rec.get("path", {}) or {}
        rtype, rmode = self._split_type_mode(str(p.get("type", "meander_plane")))
        self.content.set_active_by_type(rtype=rtype, rmode=rmode)
        self.content.apply_recipe_to_forms(rec)
        self.content.fill_selectors_for_recipe(rec)

    def _on_type_changed(self, _idx: int) -> None:
        # Beim manuellen Umschalten Selektoren mit aktuellem Rezept nachziehen
        rid = self.comboRecipeSelect.currentText().strip()
        rec = self._find_recipe_by_id(rid) if rid else None
        if rec:
            self.content.fill_selectors_for_recipe(rec)

    def _on_new_clicked(self) -> None:
        self.content.apply_defaults()
        self.content.clear_active_selectors()
        self._emit_recipe_changed()

    def _on_load_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)
            rec = model.to_dict()
            # type/mode bestimmen und UI aktualisieren
            p = rec.get("path", {}) or {}
            t = p.get("type", "meander_plane")
            if isinstance(t, str):
                rtype, rmode = self._split_type_mode(t)
            else:
                rtype = str(t or "meander")
                rmode = str(p.get("mode", "plane"))
            self.content.set_active_by_type(rtype=rtype, rmode=rmode)
            self.content.apply_recipe_to_forms(rec)
            self.content.fill_selectors_for_recipe(rec)
            QMessageBox.information(self, "Geladen", f"Rezept geladen:\n{os.path.basename(fname)}")
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_delete_clicked(self) -> None:
        # Reset auf Defaults und leere Selektoren
        self.content.apply_defaults()
        self.content.clear_active_selectors()
        # Auswahl zurück auf erstes Rezept (falls vorhanden)
        if self.comboRecipeSelect.count() > 0:
            self.comboRecipeSelect.setCurrentIndex(0)
            self._on_recipe_select_changed(0)
        self._emit_recipe_changed()

    def _emit_recipe_changed(self) -> None:
        self.recipeChanged.emit(self.current_model())

    # ---------------------------------------------------------------------
    # Model export
    # ---------------------------------------------------------------------
    def current_model(self) -> Recipe:
        params = self.content.collect_globals()
        path = self.content.collect_path_current()
        tool, sub, mnt = self.content.active_selectors_values()
        rid = self.comboRecipeSelect.currentText().strip() or "recipe"
        return Recipe.from_dict({
            "id": rid,
            "tool": tool,
            "substrate": sub,
            "substrates": [sub] if sub else [],
            "substrate_mount": mnt,
            "mount": mnt,
            "parameters": params,
            "path": path,
        })
