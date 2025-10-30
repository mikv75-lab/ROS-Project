# -*- coding: utf-8 -*-
from __future__ import annotations
import os
from typing import Optional, Dict, Any

from PyQt6 import uic
from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import QWidget, QFileDialog, QMessageBox, QVBoxLayout, QGroupBox

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
    updatePreviewRequested = pyqtSignal(object) # NEU: emits Recipe

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = RecipeStore.from_ctx(ctx)
        uic.loadUi(_ui_path("recipe_editor_panel.ui"), self)

        # Content-Widget montieren
        self.content = RecipeEditorContent(ctx=self.ctx, parent=self)
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
            # Nur Formular aktualisieren, KEIN Preview-Update
            self.comboRecipeSelect.currentIndexChanged.connect(self._on_recipe_select_changed)

        # Defaults ins Formular
        self.content.apply_defaults()

        # Buttons
        self.btnNew.clicked.connect(self._on_new_clicked)
        self.btnLoad.clicked.connect(self._on_load_clicked)
        self.btnDelete.clicked.connect(self._on_delete_clicked)
        self.btnUpdatePreview.clicked.connect(self._on_update_preview_clicked)

        # Initial Formular mit erstem Rezept, aber KEIN Preview-Update
        if hasattr(self, "comboRecipeSelect") and self.comboRecipeSelect.count() > 0:
            if self.comboRecipeSelect.currentIndex() < 0:
                self.comboRecipeSelect.setCurrentIndex(0)
            self._on_recipe_select_changed(self.comboRecipeSelect.currentIndex())

    # --- Rezepte ---
    def _fill_recipe_select(self) -> None:
        self._recipes_by_id.clear()
        self.comboRecipeSelect.blockSignals(True)
        self.comboRecipeSelect.clear()
        for rec in self.store.recipes:
            rid = str(rec.get("id") or "").strip()
            if not rid:
                continue
            self.comboRecipeSelect.addItem(rid)
            self._recipes_by_id[rid] = rec
        self.comboRecipeSelect.blockSignals(False)

    def _current_recipe_def(self) -> Optional[Dict[str, Any]]:
        if not hasattr(self, "comboRecipeSelect"):
            return None
        rid = self.comboRecipeSelect.currentText().strip()
        return self._recipes_by_id.get(rid)

    def _on_recipe_select_changed(self, _index: int) -> None:
        rec = self._current_recipe_def()
        if not rec:
            return
        self.content.apply_recipe_to_forms(rec)
        # kein emit

    # --- UI Events ---
    def _on_new_clicked(self) -> None:
        self.content.apply_defaults()
        rec = self._current_recipe_def()
        if rec:
            self.content.apply_recipe_to_forms(rec)
        # kein emit

    def _on_load_clicked(self) -> None:
        start_dir = getattr(getattr(self.ctx, "paths", None), "recipe_dir", os.getcwd())
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)

            # Formular clearen und befüllen …
            self.content.apply_defaults()
            if getattr(model, "description", None):
                self.content.e_desc.setText(model.description)

            self.content.sel_substrate.clear()
            if model.substrates:
                self.content.sel_substrate.addItems([str(s) for s in model.substrates])
                self.content.sel_substrate.setCurrentIndex(0)
            elif model.substrate:
                self.content.sel_substrate.addItem(str(model.substrate))
                self.content.sel_substrate.setCurrentIndex(0)

            self.content.sel_mount.clear()
            if model.mount or model.substrate_mount:
                self.content.sel_mount.addItem(str(model.mount or model.substrate_mount))
                self.content.sel_mount.setCurrentIndex(0)

            if hasattr(self.content, "sel_tool") and self.content.sel_tool is not None:
                if model.tool:
                    idx = self.content.sel_tool.findText(str(model.tool))
                    if idx >= 0:
                        self.content.sel_tool.setCurrentIndex(idx)
                    else:
                        self.content.sel_tool.addItem(str(model.tool))
                        self.content.sel_tool.setCurrentIndex(self.content.sel_tool.count() - 1)

            if model.parameters:
                p = model.parameters
                def _opt(name, fn):
                    if name in p: fn(p[name])
                _opt("speed_mm_s",     lambda v: self.content.g_speed.setValue(float(v)))
                _opt("stand_off_mm",   lambda v: self.content.g_standoff.setValue(float(v)))
                _opt("spray_angle_deg",lambda v: self.content.g_angle.setValue(float(v)))
                _opt("pre_dispense_s", lambda v: self.content.g_pre.setValue(float(v)))
                _opt("post_dispense_s",lambda v: self.content.g_post.setValue(float(v)))
                _opt("flow_ml_min",    lambda v: self.content.g_flow.setValue(float(v)))
                _opt("overlap_pct",    lambda v: self.content.g_overlap.setValue(int(v)))
                _opt("enable_purge",   lambda v: self.content.g_purge.setChecked(bool(v)))
                _opt("sample_step_mm", lambda v: self.content.g_sample.setValue(float(v)))
                _opt("max_points",     lambda v: self.content.g_maxpts.setValue(int(v)))
                _opt("max_angle_deg",  lambda v: self.content.g_maxang.setValue(float(v)))

            if model.paths_by_side:
                self.content._clear_side_editors()
                from .side_path_editor import SidePathEditor
                for side_name, path in model.paths_by_side.items():
                    gb = QGroupBox(f"Side: {side_name}")
                    v = QVBoxLayout(gb); v.setContentsMargins(8,8,8,8); v.setSpacing(6)
                    ed = SidePathEditor(side_name); v.addWidget(ed)
                    ed.set_allowed_types(["meander_plane","spiral_plane","spiral_cylinder"])
                    ed.apply_default_path(dict(path or {}))
                    idx = max(0, self.content._sides_container.count() - 1)
                    self.content._sides_container.insertWidget(idx, gb)
                    self.content._side_editors[side_name] = ed

            QMessageBox.information(self, "Geladen", os.path.basename(fname))
            # kein emit
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_delete_clicked(self) -> None:
        self.content.apply_defaults()
        rec = self._current_recipe_def()
        if rec:
            self.content.apply_recipe_to_forms(rec)
        # kein emit

    # --- Export / Trigger ---
    def current_model(self) -> Recipe:
        params = self.content.collect_globals()
        paths_by_side = self.content.collect_paths_by_side()
        tool, sub, mnt = self.content.active_selectors_values()
        desc = self.content.get_description()
        return Recipe.from_dict({
            "id": "recipe",
            "description": desc,
            "tool": tool,
            "substrate": sub,
            "substrates": [sub] if sub else [],
            "substrate_mount": mnt,
            "parameters": params,
            "paths_by_side": paths_by_side,
        })

    def _on_update_preview_clicked(self) -> None:
        # EINZIGER Weg, das Preview zu füttern:
        self.updatePreviewRequested.emit(self.current_model())
