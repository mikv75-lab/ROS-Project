# app/tabs/recipe/recipe_editor_panel/recipe_editor_panel.py
# -*- coding: utf-8 -*-
from __future__ import annotations

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
from model.recipe.recipe_store import RecipeStore
from .recipe_editor_content import RecipeEditorContent


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
    updatePreviewRequested = pyqtSignal(object)  # Recipe

    def __init__(
        self,
        *,
        ctx,
        store: RecipeStore,
        repo: Any,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.ctx = ctx

        if store is None:
            raise RuntimeError("RecipeEditorPanel: store ist None.")
        if not isinstance(store, RecipeStore):
            raise TypeError(f"RecipeEditorPanel: store ist kein RecipeStore (got: {type(store)}).")
        if repo is None:
            raise RuntimeError("RecipeEditorPanel: repo ist None.")

        self.store: RecipeStore = store
        self.repo = repo

        self._active_model: Optional[Recipe] = None
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)

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

        self.btn_new.clicked.connect(self._on_new_clicked)
        self.btn_load.clicked.connect(self._on_load_clicked)
        self.btn_save.clicked.connect(self._on_save_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)

        if getattr(self.content, "sel_recipe", None) is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        self._rebuild_recipe_defs()
        self._load_default_or_first()

    def _rebuild_recipe_defs(self) -> None:
        self._recipes_by_id = {}

        ids = [str(x).strip() for x in (self.store.recipe_ids() or []) if str(x).strip()]
        if not ids:
            raise KeyError("RecipeStore.recipe_ids() leer.")

        for rid in ids:
            rd = self.store.get_recipe_def(rid)
            if not isinstance(rd, dict):
                raise TypeError(f"get_recipe_def('{rid}') ist kein dict (got {type(rd)}).")
            d = dict(rd)
            d.setdefault("id", rid)
            self._recipes_by_id[rid] = d

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
        if combo is None or combo.count() <= 0:
            raise RuntimeError("Recipe selection combo leer/nicht vorhanden.")

        rid = combo.currentText().strip()
        if not rid:
            combo.setCurrentIndex(0)
            rid = combo.currentText().strip()

        rec_def = self._recipes_by_id.get(rid)
        if not rec_def:
            raise KeyError(f"RecipeDef für '{rid}' nicht gefunden.")

        try:
            model = self.repo.load_for_editor(rid)
        except FileNotFoundError:
            base = self._new_model_from_rec_def(rec_def)
            self.repo.create_new(rid, base=base, overwrite=True)
            model = self.repo.load_for_editor(rid)

        self._apply_model(model, rec_def)
        self.updatePreviewRequested.emit(model)

    def _new_model_from_rec_def(self, rec_def: Dict[str, Any]) -> Recipe:
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise KeyError("rec_def.id fehlt/leer.")

        # globals defaults (flat dict)
        params = self.store.collect_global_defaults()
        if not isinstance(params, dict):
            raise TypeError(f"collect_global_defaults() ist kein dict (got {type(params)}).")

        # planner defaults
        move_planner = self.store.planner_defaults()

        # paths_by_side strict aus sides + default_path
        sides_cfg = self.store.sides_for_recipe(rec_def)
        if not isinstance(sides_cfg, dict) or not sides_cfg:
            raise KeyError(f"Recipe '{rid}': sides fehlt/leer.")

        pbs: Dict[str, Any] = {}
        for side in sides_cfg.keys():
            scfg = self.store.allowed_and_default_for(rec_def, side)
            if not isinstance(scfg, dict):
                raise TypeError(f"sides['{side}'] ist kein dict.")

            allowed = scfg.get("allowed_path_types")
            if not isinstance(allowed, list) or not [x for x in allowed if str(x).strip()]:
                raise KeyError(f"Recipe '{rid}' side '{side}': allowed_path_types fehlt/leer.")

            dp = scfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe '{rid}' side '{side}': default_path fehlt/kein dict.")
            if "type" not in dp or not str(dp.get("type") or "").strip():
                raise KeyError(f"Recipe '{rid}' side '{side}': default_path.type fehlt/leer.")

            pbs[str(side)] = dict(dp)

        # tool/substrate/mount strict (erste Einträge)
        tools = rec_def.get("tools")
        subs = rec_def.get("substrates")
        mounts = rec_def.get("substrate_mounts")

        if not isinstance(tools, list) or not tools:
            raise KeyError(f"Recipe '{rid}': tools fehlt/leer.")
        if not isinstance(subs, list) or not subs:
            raise KeyError(f"Recipe '{rid}': substrates fehlt/leer.")
        if not isinstance(mounts, list) or not mounts:
            raise KeyError(f"Recipe '{rid}': substrate_mounts fehlt/leer.")

        return Recipe.from_dict(
            {
                "id": rid,
                "description": rec_def.get("description") or "",
                "tool": str(tools[0]),
                "substrate": str(subs[0]),
                "substrate_mount": str(mounts[0]),
                "parameters": params,
                "planner": move_planner,
                "paths_by_side": pbs,
                "trajectories": {},
            }
        )

    def _apply_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        rid = str(rec_def.get("id") or "").strip()
        desc = (rec_def.get("description") or "").strip()

        model.id = rid
        self._active_model = model

        self.content.set_meta(name=rid, desc=desc)
        self.content.apply_model_to_ui(model, rec_def)

    def current_model(self) -> Optional[Recipe]:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return None

        rid = str(rec_def.get("id") or "").strip()
        desc = (rec_def.get("description") or "").strip()

        model = self._active_model
        if model is None:
            model = self._new_model_from_rec_def(rec_def)

        model.id = rid
        self.content.set_meta(name=rid, desc=desc)
        self.content.apply_ui_to_model(model)

        model.trajectories = {}
        return model

    def _on_recipe_select_changed(self, *_args) -> None:
        rec_def = self._current_recipe_def()
        if not rec_def:
            return

        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise ValueError("recipes[..].id ist leer.")

        try:
            model = self.repo.load_for_editor(rid)
        except FileNotFoundError:
            base = self._new_model_from_rec_def(rec_def)
            self.repo.create_new(rid, base=base, overwrite=True)
            model = self.repo.load_for_editor(rid)

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

        base = self._new_model_from_rec_def(rec_def)
        self._apply_model(base, rec_def)
        self.updatePreviewRequested.emit(base)
