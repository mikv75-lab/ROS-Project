# app/tabs/recipe/recipe_editor_panel/recipe_editor_panel.py
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
    QSizePolicy,
    QMessageBox,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_repo import RecipeRepo
from model.recipe.recipe_bundle import RecipeBundle
from model.recipe.recipe_store import RecipeStore

from .recipe_editor_content import RecipeEditorContent


# ----------------------------- UI helpers -----------------------------

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
      - New    -> create_new(bundle) (draft only, clears compiled + runs)
      - Load   -> load_for_editor(bundle draft)
      - Save   -> save_editor(bundle draft, clears runs; compiled may be dropped on hash change)
      - Delete -> delete(bundle folder)
    """

    updatePreviewRequested = pyqtSignal(object)  # Recipe

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        # Recipe persistence (new bundle layout)
        recipes_root = getattr(getattr(self.ctx, "paths", None), "recipe_dir", None)
        if not recipes_root:
            raise RuntimeError("ctx.paths.recipe_dir fehlt (RecipeBundle root).")
        self.repo = RecipeRepo(bundle=RecipeBundle(recipes_root_dir=recipes_root))

        self._active_model: Optional[Recipe] = None
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        # ---------------- Layout ----------------
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)

        # --- buttons ---
        gb_cmd = QGroupBox("Recipe", self)
        gb_cmd_l = QHBoxLayout(gb_cmd)
        gb_cmd_l.setContentsMargins(8, 8, 8, 8)
        gb_cmd_l.setSpacing(8)

        self.btn_new = QPushButton("New", gb_cmd)
        self.btn_load = QPushButton("Load", gb_cmd)
        self.btn_save = QPushButton("Save", gb_cmd)
        self.btn_delete = QPushButton("Delete", gb_cmd)

        gb_cmd_l.addWidget(self.btn_new)
        gb_cmd_l.addWidget(self.btn_load)
        gb_cmd_l.addWidget(self.btn_save)
        gb_cmd_l.addStretch(1)
        gb_cmd_l.addWidget(self.btn_delete)

        root.addWidget(gb_cmd)
        root.addWidget(self.content, 1)

        _set_policy(self.content)

        # ---------------- Signals ----------------
        self.btn_new.clicked.connect(self._on_new_clicked)
        self.btn_load.clicked.connect(self._on_load_clicked)
        self.btn_save.clicked.connect(self._on_save_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)

        if getattr(self.content, "sel_recipe", None) is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        self._rebuild_recipe_defs()
        self._load_default_or_first()

    # ---------------- recipes list ----------------

    def _rebuild_recipe_defs(self) -> None:
        recipes = self.store.list_recipe_defs()
        self._recipes_by_id = {}
        for rd in recipes:
            rid = str(rd.get("id") or "").strip()
            if rid:
                self._recipes_by_id[rid] = rd

        combo = getattr(self.content, "sel_recipe", None)
        if combo is not None:
            combo.blockSignals(True)
            combo.clear()
            for rid in sorted(self._recipes_by_id.keys()):
                combo.addItem(rid)
            combo.blockSignals(False)

    def _current_recipe_def(self) -> Optional[Dict[str, Any]]:
        combo = getattr(self.content, "sel_recipe", None)
        if combo is None or combo.currentIndex() < 0:
            return None
        rid = combo.currentText().strip()
        return self._recipes_by_id.get(rid)

    def _load_default_or_first(self) -> None:
        combo = getattr(self.content, "sel_recipe", None)
        if combo is None:
            return

        if combo.count() <= 0:
            # nothing configured
            return

        # keep current selection if valid; else first
        rid = combo.currentText().strip()
        if not rid:
            combo.setCurrentIndex(0)
            rid = combo.currentText().strip()

        rec_def = self._recipes_by_id.get(rid)
        if not rec_def:
            combo.setCurrentIndex(0)
            rid = combo.currentText().strip()
            rec_def = self._recipes_by_id.get(rid)

        if not rec_def:
            return

        # Try load existing draft; otherwise show defaults
        try:
            model = self.repo.load_for_editor(rid)
        except Exception:
            model = self._new_model_from_rec_def(rec_def)

        self._apply_model(model, rec_def)
        self.updatePreviewRequested.emit(model)

    # ---------------- model ----------------

    def _new_model_from_rec_def(self, rec_def: Dict[str, Any]) -> Recipe:
        rid = str(rec_def.get("id") or "").strip()
        params = self.store.collect_global_defaults()
        move_planner = self.store.collect_planner_defaults()
        pbs = self.store.build_default_paths_for_recipe(rec_def)

        subs = rec_def.get("substrates") or []
        sub = subs[0] if subs else None
        mounts = rec_def.get("substrate_mounts") or []
        mnt = mounts[0] if mounts else None
        tools = rec_def.get("tools") or []
        tool = tools[0] if tools else None

        return Recipe.from_dict(
            {
                "id": rid,
                "description": rec_def.get("description") or "",
                "tool": tool,
                "substrate": sub,
                "substrate_mount": mnt,
                "parameters": params,
                "planner": move_planner,
                "paths_by_side": pbs,
            }
        )

    def _apply_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        rid = str(rec_def.get("id") or "").strip()
        desc = (rec_def.get("description") or "").strip()

        model.id = rid  # SSoT
        self._active_model = model

        self.content.set_meta(name=rid, desc=desc)
        self.content.apply_model_to_ui(model, rec_def)

    def current_model(self) -> Optional[Recipe]:
        """Return a Recipe instance that reflects the current UI state.

        Policy:
          - Bundle-ID is SSoT: always equals recipes[..].id (selected in combo).
          - Editor works on draft only (no traj/executed persisted here).
        """
        rec_def = self._current_recipe_def()
        if not rec_def:
            return None

        rid = str(rec_def.get("id") or "").strip()
        desc = (rec_def.get("description") or "").strip()

        model = self._active_model
        if model is None:
            model = self._new_model_from_rec_def(rec_def)

        # enforce bundle id == recipes[..].id
        model.id = rid

        # apply UI -> model
        self.content.set_meta(name=rid, desc=desc)
        self.content.apply_ui_to_model(model)

        # editor draft never carries persisted trajectories
        model.trajectories = {}
        return model

    # ---------------- UI handlers ----------------

    def _on_recipe_select_changed(self, *_args) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            return

        # load draft if exists; else show defaults
        try:
            model = self.repo.load_for_editor(rid)
        except FileNotFoundError:
            model = self._new_model_from_rec_def(rec_def)

        self._apply_model(model, rec_def)
        self.updatePreviewRequested.emit(model)

    def _on_new_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise ValueError("recipes[..].id ist leer.")

        base = self._new_model_from_rec_def(rec_def)

        # create/overwrite draft; clears compiled + runs per bundle policy
        self.repo.create_new(rid, base=base, overwrite=True)

        model = self.repo.load_for_editor(rid)
        self._apply_model(model, rec_def)
        self.updatePreviewRequested.emit(model)

    def _on_load_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise ValueError("recipes[..].id ist leer.")

        try:
            model = self.repo.load_for_editor(rid)
        except FileNotFoundError:
            # first time: create defaults
            base = self._new_model_from_rec_def(rec_def)
            self.repo.create_new(rid, base=base, overwrite=True)
            model = self.repo.load_for_editor(rid)

        self._apply_model(model, rec_def)
        self.updatePreviewRequested.emit(model)

    def _on_save_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise ValueError("recipes[..].id ist leer.")

        model = self.current_model()
        if model is None:
            return
        model.id = rid

        # Editor-save: draft only; bundle enforces "no runs in draft" and can drop compiled on hash changes
        self.repo.save_editor(rid, draft=model, compiled=None, delete_compiled_on_hash_change=True)

        QMessageBox.information(self, "Gespeichert", f"{rid} (draft)")

    def _on_delete_clicked(self) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise ValueError("recipes[..].id ist leer.")

        reply = QMessageBox.question(
            self,
            "Löschen",
            f"Rezept '{rid}' wirklich löschen? (draft + compiled + runs)",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        self.repo.delete(rid)

        # reset UI to defaults (unsaved draft)
        base = self._new_model_from_rec_def(rec_def)
        self._apply_model(base, rec_def)
        self.updatePreviewRequested.emit(base)
