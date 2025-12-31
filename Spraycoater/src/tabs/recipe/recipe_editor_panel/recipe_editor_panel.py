# app/tabs/recipe/recipe_editor_panel/recipe_editor_panel.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, List, Tuple

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
from model.recipe.path_builder import PathBuilder

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
    """
    Storage key Format (SSoT):
      key = "<recipe_id>/<recipe_name>"

    UI:
      - Selection->Recipe = recipe_id (Catalog / constraints)
      - Meta->Recipe Name = recipe_name (folder/key suffix)

    Verhalten:
      - Start: Default state (name="default") + Preview (keine Files)
      - New: reset default + Preview (keine Files)
      - Load: lädt key (rid/rname) + Preview
      - Save: schreibt (rid/rname) - EINZIGE Stelle die Files erstellt/überschreibt
      - Delete: löscht key (rid/rname) und reset auf Default + Preview
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
        self._active_compiled: Optional[dict] = None

        # active selection and name
        self._active_rid: str = ""          # recipe_id (Selection->Recipe)
        self._active_name: str = "default"  # recipe_name (Meta field)

        # ---------------- UI ----------------
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

        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        root.addWidget(self.content, 1)
        _set_policy(self.content)

        # ---------------- Signals ----------------
        self.btn_new.clicked.connect(self._on_new_clicked)
        self.btn_load.clicked.connect(self._on_load_clicked)
        self.btn_save.clicked.connect(self._on_save_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        self.btn_update_preview.clicked.connect(self._on_update_preview_clicked)

        if getattr(self.content, "sel_recipe", None) is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        # init
        self._rebuild_recipe_defs()
        self._populate_recipe_combo()
        self._load_default_into_editor(trigger_preview=True)

    # ---------------------------------------------------------
    # Key helpers
    # ---------------------------------------------------------

    def _split_key(self, key: str) -> Tuple[str, str]:
        # use bundle implementation (SSoT)
        return self.repo.bundle.split_key(key)

    def _make_key(self, rid: str, name: str) -> str:
        return self.repo.make_key(rid, name)

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

    def _populate_recipe_combo(self) -> None:
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

    def _current_rid(self) -> str:
        cb = getattr(self.content, "sel_recipe", None)
        if cb is None or cb.currentIndex() < 0:
            return ""
        return str(cb.currentText() or "").strip()

    def _select_rid_in_combo_blocked(self, rid: str) -> None:
        cb = getattr(self.content, "sel_recipe", None)
        if cb is None:
            return
        rid = str(rid or "").strip()
        if not rid:
            return
        idx = cb.findText(rid)
        if idx < 0:
            return
        cb.blockSignals(True)
        cb.setCurrentIndex(idx)
        cb.blockSignals(False)

    def _recipe_def_for_rid(self, rid: str) -> Dict[str, Any]:
        rid = str(rid or "").strip()
        if not rid:
            raise KeyError("rid leer")
        rec_def = self._recipes_by_id.get(rid)
        if not rec_def:
            raise KeyError(f"RecipeDef für '{rid}' nicht gefunden")
        return rec_def

    def _first_rid(self) -> str:
        rids = sorted(self._recipes_by_id.keys())
        return rids[0] if rids else ""

    # ---------------------------------------------------------
    # Meta name helpers
    # ---------------------------------------------------------

    def _recipe_name_from_ui(self, fallback: str = "default") -> str:
        fn = getattr(self.content, "get_recipe_name", None)
        if callable(fn):
            try:
                s = str(fn() or "").strip()
                if s:
                    return s
            except Exception:
                pass
        return (fallback or "default").strip() or "default"

    def _set_recipe_name_in_ui(self, name: str) -> None:
        fn = getattr(self.content, "set_recipe_name", None)
        if callable(fn):
            try:
                fn(name)
            except Exception:
                pass

    # ---------------------------------------------------------
    # Model creation / compile
    # ---------------------------------------------------------

    def _new_model_from_rec_def(self, rec_def: Dict[str, Any]) -> Recipe:
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise KeyError("rec_def.id fehlt/leer")

        params = self.store.collect_global_defaults()
        if not isinstance(params, dict):
            raise TypeError(f"collect_global_defaults() ist kein dict (got {type(params)}).")

        move_planner = self.store.planner_defaults()

        sides_cfg = self.store.sides_for_recipe(rec_def)
        if not isinstance(sides_cfg, dict) or not sides_cfg:
            raise KeyError(f"Recipe '{rid}': sides fehlt/leer")

        pbs: Dict[str, Any] = {}
        for side in sides_cfg.keys():
            scfg = self.store.allowed_and_default_for(rec_def, side)
            if not isinstance(scfg, dict):
                raise TypeError(f"sides['{side}'] ist kein dict")

            allowed = scfg.get("allowed_path_types")
            if not isinstance(allowed, list) or not [x for x in allowed if str(x).strip()]:
                raise KeyError(f"Recipe '{rid}' side '{side}': allowed_path_types fehlt/leer")

            dp = scfg.get("default_path")
            if not isinstance(dp, dict):
                raise KeyError(f"Recipe '{rid}' side '{side}': default_path fehlt/kein dict")
            if "type" not in dp or not str(dp.get("type") or "").strip():
                raise KeyError(f"Recipe '{rid}' side '{side}': default_path.type fehlt/leer")

            pbs[str(side)] = dict(dp)

        tools = rec_def.get("tools")
        subs = rec_def.get("substrates")
        mounts = rec_def.get("substrate_mounts")

        if not isinstance(tools, list) or not tools:
            raise KeyError(f"Recipe '{rid}': tools fehlt/leer")
        if not isinstance(subs, list) or not subs:
            raise KeyError(f"Recipe '{rid}': substrates fehlt/leer")
        if not isinstance(mounts, list) or not mounts:
            raise KeyError(f"Recipe '{rid}': substrate_mounts fehlt/leer")

        return Recipe.from_dict(
            {
                "id": rid,  # catalog recipe_id
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

    def _compile_from_model(self, model: Recipe) -> dict:
        sides: List[str] = list((model.paths_by_side or {}).keys())
        if not sides:
            return {}

        g = dict(model.parameters or {})
        sample_step_mm = float(g.get("sample_step_mm", 1.0))
        max_points = int(g.get("max_points", 500))

        out: dict = {
            "frame": "scene",
            "tool_frame": (model.planner or {}).get("tool_frame", "tool_mount"),
            "sides": {},
            "meta": {"sample_step_mm": sample_step_mm, "max_points": max_points},
        }

        pb = PathBuilder(sample_step_mm=sample_step_mm, max_points=max_points)
        for side in sides:
            path = (model.paths_by_side or {}).get(side) or {}
            pts = pb.build_points_mm(path)
            out["sides"][side] = {"points_mm": pts.tolist() if hasattr(pts, "tolist") else pts}
        return out

    # ---------------------------------------------------------
    # UI apply
    # ---------------------------------------------------------

    def _load_default_into_editor(self, *, trigger_preview: bool) -> None:
        rid = self._current_rid() or self._first_rid()
        if not rid:
            return

        self._select_rid_in_combo_blocked(rid)
        rec_def = self._recipe_def_for_rid(rid)
        model = self._new_model_from_rec_def(rec_def)

        self._active_rid = rid
        self._active_model = model
        self._active_compiled = None
        self._active_name = "default"

        desc = (rec_def.get("description") or "").strip()
        self._set_recipe_name_in_ui("default")
        self.content.set_meta(name="default", desc=desc)
        self.content.apply_model_to_ui(model, rec_def)

        if trigger_preview:
            self.updatePreviewRequested.emit(model)

    def current_model(self) -> Optional[Recipe]:
        rid = self._current_rid() or self._first_rid()
        if not rid:
            return None

        rec_def = self._recipe_def_for_rid(rid)
        model = self._active_model or self._new_model_from_rec_def(rec_def)

        # ALWAYS keep id == catalog rid
        model.id = rid
        self._active_rid = rid

        name = self._recipe_name_from_ui(fallback=self._active_name or "default")
        self._active_name = name

        desc = (rec_def.get("description") or "").strip()
        self.content.set_meta(name=name, desc=desc)

        self.content.apply_ui_to_model(model)
        model.trajectories = {}

        self._active_model = model
        return model

    def _apply_loaded(self, *, key: str, model: Recipe) -> None:
        # key is SSoT: rid/rname
        rid, rname = self._split_key(key)

        # selection from key
        self._select_rid_in_combo_blocked(rid)
        rec_def = self._recipe_def_for_rid(rid)

        # bundle already normalizes model.id=rid, but keep it explicit
        model.id = rid

        self._active_rid = rid
        self._active_model = model
        self._active_name = rname

        desc = (rec_def.get("description") or "").strip()
        self._set_recipe_name_in_ui(rname)
        self.content.set_meta(name=rname, desc=desc)
        self.content.apply_model_to_ui(model, rec_def)

        # optional compiled cache
        try:
            full = self.repo.load_for_process(key)
            self._active_compiled = (full.paths_compiled or None)
        except Exception:
            self._active_compiled = None

        # ✅ Load triggers preview
        self.updatePreviewRequested.emit(model)

    # ---------------------------------------------------------
    # Slots
    # ---------------------------------------------------------

    def _on_update_preview_clicked(self) -> None:
        model = self.current_model()
        if model is None:
            return
        try:
            self._active_compiled = self._compile_from_model(model)
        except Exception:
            self._active_compiled = None
        self.updatePreviewRequested.emit(model)

    def _on_new_clicked(self) -> None:
        self._load_default_into_editor(trigger_preview=True)

    def _on_recipe_select_changed(self, *_args) -> None:
        # switching catalog recipe resets to default name
        self._load_default_into_editor(trigger_preview=True)

    def _on_load_clicked(self) -> None:
        keys = self.repo.list_recipes() or []
        if not keys:
            QMessageBox.information(self, "Load", "Keine gespeicherten Rezepte gefunden.")
            return

        # suggest current key if it exists
        rid = self._current_rid() or self._active_rid or self._first_rid()
        rname = self._active_name or "default"
        suggest_key = self._make_key(rid, rname)
        idx = keys.index(suggest_key) if suggest_key in keys else 0

        choice, ok = QInputDialog.getItem(
            self,
            "Recipe laden",
            "Rezept auswählen (recipe_id/recipe_name):",
            keys,
            idx,
            editable=False,
        )
        if not ok:
            return

        key = str(choice).strip()
        if not key:
            return

        model = self.repo.load_for_editor(key)
        self._apply_loaded(key=key, model=model)

    def _on_save_clicked(self) -> None:
        model = self.current_model()
        if model is None:
            return

        rid = self._current_rid() or self._active_rid or self._first_rid()
        if not rid:
            QMessageBox.warning(self, "Save", "Kein Recipe (Catalog-ID) ausgewählt.")
            return

        name = self._recipe_name_from_ui(fallback=self._active_name or "default").strip()
        if not name:
            QMessageBox.warning(self, "Save", "Recipe Name ist leer.")
            return

        # ✅ key = rid/name  (SSoT)
        key = self._make_key(rid, name)

        if self._active_compiled is None:
            try:
                self._active_compiled = self._compile_from_model(model)
            except Exception:
                self._active_compiled = None

        # save (only place writing files)
        self.repo.save_from_editor(
            key,
            draft=model,
            compiled=self._active_compiled,
            delete_compiled_on_hash_change=True,
        )

        self._active_rid = rid
        self._active_name = name

        QMessageBox.information(self, "Gespeichert", f"{key} (recipe.yaml + compiled.yaml)")

    def _on_delete_clicked(self) -> None:
        rid = self._current_rid() or self._active_rid or self._first_rid()
        name = (self._active_name or "").strip()

        if not rid or not name or name == "default":
            QMessageBox.information(self, "Delete", "Kein gespeichertes Rezept ausgewählt (name != default).")
            return

        key = self._make_key(rid, name)
        reply = QMessageBox.question(
            self,
            "Löschen",
            f"Rezept '{key}' wirklich löschen? (draft + compiled + runs)",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        self.repo.delete(key)

        self._active_model = None
        self._active_compiled = None
        self._active_name = "default"
        self._load_default_into_editor(trigger_preview=True)
