# -*- coding: utf-8 -*-
# File: src/tabs/recipe/recipe_editor_panel/recipe_editor_panel.py
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
    QInputDialog,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from model.recipe.recipe_repo import RecipeRepo


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


def _model_valid_save(model: Any) -> bool:
    """
    STRICT Save-Gate (SSoT):
      - ONLY model.info['validSave'] is allowed.
    """
    info = getattr(model, "info", None)
    if isinstance(info, dict) and "validSave" in info:
        return bool(info.get("validSave"))
    return False


def _ensure_validsave_fields(model: Any) -> None:
    if not isinstance(getattr(model, "info", None), dict):
        model.info = {}
    if "validSave" not in model.info:
        model.info["validSave"] = False
    if "validSaveReason" not in model.info:
        model.info["validSaveReason"] = "no_preview"


class RecipeEditorPanel(QWidget):
    """
    Save writes:
      - params.yaml
      - draft.yaml   (must include predispense+retreat; produced by Update Preview)
    """

    updatePreviewRequested = pyqtSignal(object)  # Recipe

    def __init__(
        self,
        *,
        ctx,
        store: RecipeStore,
        repo: RecipeRepo,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)

        self.ctx = ctx
        if store is None or not isinstance(store, RecipeStore):
            raise TypeError(f"RecipeEditorPanel: store invalid: {type(store)}")
        if repo is None or not isinstance(repo, RecipeRepo):
            raise TypeError(f"RecipeEditorPanel: repo invalid: {type(repo)}")

        self.store: RecipeStore = store
        self.repo: RecipeRepo = repo

        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        self._active_model: Optional[Recipe] = None
        self._active_template_id: str = ""
        self._active_name: str = ""

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        gb_cmd = QGroupBox("Recipe", self)
        gb_cmd_l = QHBoxLayout(gb_cmd)
        gb_cmd_l.setContentsMargins(8, 8, 8, 8)
        gb_cmd_l.setSpacing(8)

        self.btn_new = QPushButton("New", gb_cmd)
        self.btn_load = QPushButton("Load", gb_cmd)
        self.btn_save = QPushButton("Save", gb_cmd)
        self.btn_delete = QPushButton("Delete", gb_cmd)
        self.btn_update_preview = QPushButton("Update Preview", gb_cmd)

        gb_cmd_l.addWidget(self.btn_new)
        gb_cmd_l.addWidget(self.btn_load)
        gb_cmd_l.addWidget(self.btn_save)
        gb_cmd_l.addWidget(self.btn_update_preview)
        gb_cmd_l.addStretch(1)
        gb_cmd_l.addWidget(self.btn_delete)
        root.addWidget(gb_cmd)

        from .recipe_editor_content import RecipeEditorContent
        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        root.addWidget(self.content, 1)
        _set_policy(self.content)

        self.btn_new.clicked.connect(self._on_new_clicked)
        self.btn_load.clicked.connect(self._on_load_clicked)
        self.btn_save.clicked.connect(self._on_save_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        self.btn_update_preview.clicked.connect(self._on_update_preview_clicked)

        if getattr(self.content, "sel_recipe", None) is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_template_select_changed)

        self._rebuild_recipe_defs()
        self._populate_template_combo()
        self._load_default_into_editor(trigger_preview=True)

    # ---------------------------------------------------------
    # Catalog / Selection
    # ---------------------------------------------------------

    def _rebuild_recipe_defs(self) -> None:
        self._recipes_by_id = {}
        for r in (self.store.recipes or []):
            if not isinstance(r, dict):
                continue
            rid = str(r.get("id") or "").strip()
            if rid:
                self._recipes_by_id[rid] = dict(r)

    def _populate_template_combo(self) -> None:
        cb = getattr(self.content, "sel_recipe", None)
        if cb is None:
            return
        rids = sorted(self._recipes_by_id.keys())
        cb.blockSignals(True)
        cb.clear()
        for rid in rids:
            cb.addItem(rid)
        cb.blockSignals(False)
        if cb.count() == 0:
            raise RuntimeError("Selection->Recipe ist leer: store.recipes enthält keine IDs.")
        if cb.currentIndex() < 0:
            cb.setCurrentIndex(0)

    def _current_template_id(self) -> str:
        cb = getattr(self.content, "sel_recipe", None)
        if cb is None or cb.currentIndex() < 0:
            return ""
        return str(cb.currentText() or "").strip()

    def _template_def_for_id(self, template_id: str) -> Dict[str, Any]:
        template_id = str(template_id or "").strip()
        if not template_id:
            raise KeyError("template_id leer")
        rec_def = self._recipes_by_id.get(template_id)
        if not rec_def:
            raise KeyError(f"RecipeDef für '{template_id}' nicht gefunden")
        return rec_def

    def _first_template_id(self) -> str:
        rids = sorted(self._recipes_by_id.keys())
        return rids[0] if rids else ""

    # ---------------------------------------------------------
    # Helpers
    # ---------------------------------------------------------

    def _current_recipe_name(self) -> str:
        try:
            return str(self.content.get_recipe_name() or "").strip()
        except Exception:
            return ""

    def _require_recipe_name(self) -> str:
        name = self._current_recipe_name()
        if not name:
            raise ValueError("Recipe name ist leer. Bitte oben im Meta-Block setzen (Ordnername unter wafer/...).")
        return name

    def _default_recipe_name_for_template(self, template_id: str) -> str:
        tid = str(template_id or "recipe").strip() or "recipe"
        return f"{tid}_01"

    # ---------------------------------------------------------
    # Model creation
    # ---------------------------------------------------------

    def _new_model_from_template(self, rec_def: Dict[str, Any]) -> Recipe:
        template_id = str(rec_def.get("id") or "").strip()
        if not template_id:
            raise KeyError("rec_def.id fehlt/leer")

        params = self.store.collect_global_defaults()
        if not isinstance(params, dict):
            raise TypeError(f"collect_global_defaults() ist kein dict (got {type(params)}).")

        move_planner = self.store.planner_defaults()

        sides_cfg = self.store.sides_for_recipe(rec_def)
        if not isinstance(sides_cfg, dict) or not sides_cfg:
            raise KeyError(f"Recipe '{template_id}': sides fehlt/leer")

        pbs: Dict[str, Any] = {}
        for side in sides_cfg.keys():
            scfg = self.store.allowed_and_default_for(rec_def, side)
            if not isinstance(scfg, dict):
                raise TypeError(f"sides['{side}'] ist kein dict")

            allowed = scfg.get("allowed_path_types")
            if not isinstance(allowed, list) or not [x for x in allowed if str(x).strip()]:
                raise KeyError(f"Recipe '{template_id}' side '{side}': allowed_path_types fehlt/leer")

            dp = scfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe '{template_id}' side '{side}': default_path fehlt/kein dict")
            if "type" not in dp or not str(dp.get("type") or "").strip():
                raise KeyError(f"Recipe '{template_id}' side '{side}': default_path.type fehlt/leer")

            pbs[str(side)] = dict(dp)

        tools = rec_def.get("tools")
        subs = rec_def.get("substrates")
        mounts = rec_def.get("substrate_mounts")

        if not isinstance(tools, list) or not tools:
            raise KeyError(f"Recipe '{template_id}': tools fehlt/leer")
        if not isinstance(subs, list) or not subs:
            raise KeyError(f"Recipe '{template_id}': substrates fehlt/leer")
        if not isinstance(mounts, list) or not mounts:
            raise KeyError(f"Recipe '{template_id}': substrate_mounts fehlt/leer")

        placeholder_id = self._current_recipe_name() or self._default_recipe_name_for_template(template_id)

        model = Recipe.from_params_dict(
            {
                "id": placeholder_id,
                "description": rec_def.get("description") or "",
                "tool": str(tools[0]),
                "substrate": str(subs[0]),
                "substrate_mount": str(mounts[0]),
                "parameters": params,
                "planner": move_planner,
                "paths_by_side": pbs,
                "meta": {"template_id": template_id},
                "info": {},
            }
        )

        _ensure_validsave_fields(model)
        return model

    # ---------------------------------------------------------
    # UI apply
    # ---------------------------------------------------------

    def _load_default_into_editor(self, *, trigger_preview: bool) -> None:
        template_id = self._current_template_id() or self._first_template_id()
        if not template_id:
            return

        rec_def = self._template_def_for_id(template_id)
        model = self._new_model_from_template(rec_def)

        self._active_template_id = template_id
        self._active_model = model

        if not self._current_recipe_name():
            suggested = self._default_recipe_name_for_template(template_id)
            self.content.set_meta(name=suggested, desc=(rec_def.get("description") or "").strip())
        else:
            self.content.set_meta(desc=(rec_def.get("description") or "").strip())

        self.content.apply_model_to_ui(model, rec_def)

        if trigger_preview:
            m = self.current_model()
            if m is not None:
                self.updatePreviewRequested.emit(m)

    def current_model(self) -> Optional[Recipe]:
        template_id = self._current_template_id() or self._first_template_id()
        if not template_id:
            return None

        rec_def = self._template_def_for_id(template_id)
        model = self._active_model or self._new_model_from_template(rec_def)

        # Preserve preview-produced draft across UI->model apply
        prev_draft = getattr(model, "draft", None)

        # UI -> model
        self.content.apply_ui_to_model(model)

        # If UI code cleared draft inadvertently, restore it (preview is authoritative)
        if getattr(model, "draft", None) is None and prev_draft is not None:
            model.draft = prev_draft

        name_ui = self._current_recipe_name()
        if name_ui:
            model.id = name_ui
        else:
            model.id = str(model.id or "").strip() or self._default_recipe_name_for_template(template_id)

        self._active_name = model.id
        self._active_template_id = template_id

        _ensure_validsave_fields(model)

        self._active_model = model
        return model

    def _apply_loaded(self, *, recipe_name: str, model: Recipe) -> None:
        recipe_name = str(recipe_name or "").strip()
        if not recipe_name:
            return

        template_id = ""
        try:
            meta = getattr(model, "meta", {}) or {}
            if isinstance(meta, dict):
                template_id = str(meta.get("template_id") or "").strip()
        except Exception:
            template_id = ""

        if template_id and template_id in self._recipes_by_id:
            rec_def = self._template_def_for_id(template_id)
            if getattr(self.content, "sel_recipe", None) is not None:
                try:
                    self.content.sel_recipe.blockSignals(True)
                    self.content.sel_recipe.setCurrentText(template_id)
                finally:
                    self.content.sel_recipe.blockSignals(False)
        else:
            template_id = self._current_template_id() or self._first_template_id()
            rec_def = self._template_def_for_id(template_id) if template_id else {}

        model.id = recipe_name
        _ensure_validsave_fields(model)

        self._active_name = recipe_name
        self._active_template_id = template_id
        self._active_model = model

        self.content.set_meta(name=recipe_name, desc=str(getattr(model, "description", "") or ""))

        if rec_def:
            self.content.apply_model_to_ui(model, rec_def)

        self.updatePreviewRequested.emit(model)

    # ---------------------------------------------------------
    # Slots
    # ---------------------------------------------------------

    def _on_update_preview_clicked(self) -> None:
        model = self.current_model()
        if model is None:
            return
        if not str(model.id or "").strip():
            QMessageBox.warning(self, "Preview", "Recipe name ist leer (Meta).")
            return

        # Update Preview builds a NEW draft in preview pipeline (SceneManager),
        # uses it for 2D/3D display, and writes it on Save.
        self.updatePreviewRequested.emit(model)

    def _on_new_clicked(self) -> None:
        self._active_model = None
        self._active_name = ""
        self._load_default_into_editor(trigger_preview=True)

    def _on_template_select_changed(self, *_args) -> None:
        keep_name = self._current_recipe_name()
        self._load_default_into_editor(trigger_preview=True)
        if keep_name:
            self.content.set_meta(name=keep_name)

    def _on_load_clicked(self) -> None:
        names = self.repo.list_recipes() or []
        names = [str(x) for x in names if str(x).strip()]
        if not names:
            QMessageBox.information(self, "Load", "Keine gespeicherten Rezepte gefunden (params.yaml fehlt).")
            return

        suggest = self._current_recipe_name() or self._active_name or (names[0] if names else "")
        idx = names.index(suggest) if suggest in names else 0

        choice, ok = QInputDialog.getItem(
            self,
            "Recipe laden",
            "Rezept auswählen (Ordnername):",
            names,
            idx,
            editable=False,
        )
        if not ok:
            return

        name = str(choice).strip()
        if not name:
            return

        try:
            model = self.repo.load_for_editor(name)
        except Exception as e:
            QMessageBox.warning(self, "Load", f"Laden fehlgeschlagen:\n{e}")
            return

        self._apply_loaded(recipe_name=name, model=model)

    def _on_save_clicked(self) -> None:
        model = self.current_model()
        if model is None:
            return

        try:
            name = self._require_recipe_name()
        except Exception as e:
            QMessageBox.warning(self, "Save", str(e))
            return

        if not _model_valid_save(model):
            QMessageBox.warning(self, "Save", "Can not be saved")
            return

        # STRICT: draft must come from Update Preview (SceneManager) so it contains pre/ret
        if getattr(model, "draft", None) is None:
            QMessageBox.warning(self, "Save", "Draft fehlt. Bitte zuerst 'Update Preview' ausführen.")
            return

        try:
            # RecipeBundle.save_from_editor writes params.yaml and (if present) draft.yaml
            self.repo.save_from_editor(name, draft=model)
        except Exception as e:
            QMessageBox.warning(self, "Save", f"Speichern fehlgeschlagen:\n{e}")
            return

        self._active_name = name
        QMessageBox.information(self, "Gespeichert", f"{name} (params.yaml + draft.yaml)")

    def _on_delete_clicked(self) -> None:
        name = self._current_recipe_name() or self._active_name
        name = str(name or "").strip()
        if not name:
            QMessageBox.information(self, "Delete", "Kein recipe name ausgewählt (Meta).")
            return

        reply = QMessageBox.question(
            self,
            "Löschen",
            f"Rezept '{name}' wirklich löschen?\n(params + draft + planned/executed traj + planned/executed tcp)",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        try:
            self.repo.delete(name)
        except Exception as e:
            QMessageBox.warning(self, "Delete", f"Löschen fehlgeschlagen:\n{e}")
            return

        self._active_model = None
        self._active_name = ""
        self._load_default_into_editor(trigger_preview=True)
