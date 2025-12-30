# app/tabs/recipe/recipe_editor_panel/recipe_editor_panel.py
# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, List

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
    Storage-SSoT (dein Bundle/Repo):
      data/recipes/<RECIPE_NAME>/
        recipe.yaml
        compiled.yaml

    UI:
      - Recipe Name (editierbar) bestimmt <RECIPE_NAME>
      - Selection->Recipe Combo bestimmt model.id (Catalog-ID), nur für UI/Constraints

    Verhalten:
      - Load lädt draft und updated UI komplett (inkl. Selection->Recipe anhand model.id)
      - Update Preview baut Draft aus UI, compiled-cache und emittet 1x
      - Save speichert recipe.yaml + compiled.yaml unter <RECIPE_NAME>
      - New resettet UI auf Defaults (für aktuell selektiertes Catalog-Recipe), setzt Recipe Name auf "<rid>"
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

        if store is None:
            raise RuntimeError("RecipeEditorPanel: store ist None.")
        if not isinstance(store, RecipeStore):
            raise TypeError(f"RecipeEditorPanel: store ist kein RecipeStore (got: {type(store)}).")
        if repo is None:
            raise RuntimeError("RecipeEditorPanel: repo ist None.")
        if not isinstance(repo, RecipeRepo):
            raise TypeError(f"RecipeEditorPanel: repo ist kein RecipeRepo (got: {type(repo)}).")

        self.store: RecipeStore = store
        self.repo: RecipeRepo = repo

        self._active_model: Optional[Recipe] = None
        self._active_compiled: Optional[dict] = None
        self._active_name: Optional[str] = None  # <RECIPE_NAME> folder

        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # -------- Recipe GroupBox (Buttons) --------
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

        # -------- Content --------
        self.content = RecipeEditorContent(ctx=self.ctx, store=self.store, parent=self)
        root.addWidget(self.content, 1)
        _set_policy(self.content)

        # -------- Signals --------
        self.btn_new.clicked.connect(self._on_new_clicked)
        self.btn_load.clicked.connect(self._on_load_clicked)
        self.btn_save.clicked.connect(self._on_save_clicked)
        self.btn_delete.clicked.connect(self._on_delete_clicked)
        self.btn_update_preview.clicked.connect(self._on_update_preview_clicked)

        if getattr(self.content, "sel_recipe", None) is not None:
            self.content.sel_recipe.currentIndexChanged.connect(self._on_recipe_select_changed)

        self._rebuild_recipe_defs()
        self._load_default_or_first()

    # ---------------------------------------------------------
    # Helpers: rid / recipe_name / UI selection
    # ---------------------------------------------------------

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

    def _recipe_name_from_ui(self, fallback: str) -> str:
        fn = getattr(self.content, "get_recipe_name", None)
        if callable(fn):
            try:
                s = str(fn() or "").strip()
                if s:
                    return s
            except Exception:
                pass
        return str(fallback or "").strip() or "recipe"

    def _set_recipe_name_in_ui(self, name: str) -> None:
        fn = getattr(self.content, "set_recipe_name", None)
        if callable(fn):
            try:
                fn(name)
            except Exception:
                pass

    # ---------------------------------------------------------
    # Compile: paths_by_side -> compiled dict
    # ---------------------------------------------------------

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

        for side in sides:
            pd = PathBuilder.from_side(
                model,
                side=str(side),
                globals_params=g,
                sample_step_mm=sample_step_mm,
                max_points=max_points,
            )
            P = pd.points_mm  # Nx3 mm
            poses = [
                {"x": float(x), "y": float(y), "z": float(z), "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0}
                for x, y, z in P
            ]
            out["sides"][str(side)] = {"poses_quat": poses, "meta": dict(pd.meta or {})}

        return out

    # ---------------------------------------------------------
    # Recipe defs (Catalog)
    # ---------------------------------------------------------

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

    def _recipe_def_for_rid(self, rid: str) -> Dict[str, Any]:
        rid = str(rid or "").strip()
        if not rid:
            raise KeyError("rid leer.")
        rec_def = self._recipes_by_id.get(rid)
        if not rec_def:
            raise KeyError(f"RecipeDef für '{rid}' nicht gefunden.")
        return rec_def

    # ---------------------------------------------------------
    # Default/new model from rec_def
    # ---------------------------------------------------------

    def _new_model_from_rec_def(self, rec_def: Dict[str, Any]) -> Recipe:
        rid = str(rec_def.get("id") or "").strip()
        if not rid:
            raise KeyError("rec_def.id fehlt/leer.")

        params = self.store.collect_global_defaults()
        if not isinstance(params, dict):
            raise TypeError(f"collect_global_defaults() ist kein dict (got {type(params)}).")

        move_planner = self.store.planner_defaults()

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
                "id": rid,  # Catalog recipe-id
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

    # ---------------------------------------------------------
    # Apply loaded model into UI (and emit preview once)
    # ---------------------------------------------------------

    def _apply_loaded(self, *, name: str, model: Recipe) -> None:
        # model.id bestimmt, welches Catalog-Recipe aktiv ist
        rid = str(getattr(model, "id", "") or "").strip()
        if not rid:
            raise KeyError(f"Geladenes Rezept '{name}': model.id fehlt/leer (kann Catalog nicht auswählen).")

        rec_def = self._recipe_def_for_rid(rid)

        # 1) Selection->Recipe (Catalog) setzen
        self._select_rid_in_combo_blocked(rid)

        # 2) Active state
        self._active_model = model
        self._active_name = name

        # 3) Meta-UI setzen
        desc = (rec_def.get("description") or "").strip()
        self._set_recipe_name_in_ui(name)
        self.content.set_meta(name=name, desc=desc)

        # 4) Restliche UI aus Modell füllen
        self.content.apply_model_to_ui(model, rec_def)

        # 5) compiled cache (optional)
        try:
            full = self.repo.load_for_process(name)
            self._active_compiled = (full.paths_compiled or None)
        except Exception:
            self._active_compiled = None

        # 6) Preview: genau einmal mit geladenem Draft
        self.updatePreviewRequested.emit(model)

    # ---------------------------------------------------------
    # Build current model from UI
    # ---------------------------------------------------------

    def current_model(self) -> Optional[Recipe]:
        rid = self._current_rid()
        if not rid:
            return None

        rec_def = self._recipe_def_for_rid(rid)
        desc = (rec_def.get("description") or "").strip()

        model = self._active_model
        if model is None:
            model = self._new_model_from_rec_def(rec_def)

        # immer Catalog-ID setzen (UI constraints)
        model.id = rid

        # Meta
        name = self._recipe_name_from_ui(fallback=rid)
        self.content.set_meta(name=name, desc=desc)

        # UI -> Model
        self.content.apply_ui_to_model(model)
        model.trajectories = {}

        self._active_model = model
        self._active_name = name
        return model

    # ---------------------------------------------------------
    # Startup default
    # ---------------------------------------------------------

    def _load_default_or_first(self) -> None:
        names = self.repo.list_recipes() or []
        if names:
            name = names[0]
            model = self.repo.load_for_editor(name)
            self._apply_loaded(name=name, model=model)
            return

        # nichts gespeichert -> defaults für aktuell selektiertes Catalog-Recipe
        rid = self._current_rid()
        if not rid:
            raise RuntimeError("Kein Catalog-Recipe auswählbar (sel_recipe leer).")

        rec_def = self._recipe_def_for_rid(rid)
        base = self._new_model_from_rec_def(rec_def)

        name = rid  # default recipe-name
        self.repo.create_new(name, base=base, overwrite=True)
        model = self.repo.load_for_editor(name)
        self._apply_loaded(name=name, model=model)

    # ---------------------------------------------------------
    # UI handlers
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

    def _on_recipe_select_changed(self, *_args) -> None:
        # User wechselt Catalog recipe-id -> fresh defaults anzeigen
        rid = self._current_rid()
        if not rid:
            return

        rec_def = self._recipe_def_for_rid(rid)
        base = self._new_model_from_rec_def(rec_def)

        # defaults
        self._active_model = base
        self._active_compiled = None
        self._active_name = rid
        desc = (rec_def.get("description") or "").strip()
        self._set_recipe_name_in_ui(rid)
        self.content.set_meta(name=rid, desc=desc)
        self.content.apply_model_to_ui(base, rec_def)
        self.updatePreviewRequested.emit(base)

    def _on_new_clicked(self) -> None:
        # reset auf defaults für aktuelles Catalog-Recipe
        rid = self._current_rid()
        if not rid:
            return
        rec_def = self._recipe_def_for_rid(rid)
        base = self._new_model_from_rec_def(rec_def)

        self._active_model = base
        self._active_compiled = None
        self._active_name = rid

        desc = (rec_def.get("description") or "").strip()
        self._set_recipe_name_in_ui(rid)
        self.content.set_meta(name=rid, desc=desc)
        self.content.apply_model_to_ui(base, rec_def)
        self.updatePreviewRequested.emit(base)

    def _on_load_clicked(self) -> None:
        names = self.repo.list_recipes() or []
        if not names:
            QMessageBox.information(self, "Load", "Keine gespeicherten Rezepte gefunden.")
            return

        current = self._active_name if (self._active_name in names) else names[0]
        idx = names.index(current) if current in names else 0

        choice, ok = QInputDialog.getItem(
            self,
            "Recipe laden",
            "Rezept auswählen:",
            names,
            idx,
            editable=False,
        )
        if not ok:
            return

        name = str(choice).strip()
        model = self.repo.load_for_editor(name)
        self._apply_loaded(name=name, model=model)

    def _on_save_clicked(self) -> None:
        model = self.current_model()
        if model is None:
            return

        name = self._active_name or self._recipe_name_from_ui(fallback=self._current_rid() or "recipe")
        name = str(name).strip()
        if not name:
            QMessageBox.warning(self, "Save", "Recipe Name ist leer.")
            return

        # compile wenn nicht vorhanden
        if self._active_compiled is None:
            try:
                self._active_compiled = self._compile_from_model(model)
            except Exception:
                self._active_compiled = None

        self.repo.save_from_editor(
            name,
            draft=model,
            compiled=self._active_compiled,
            delete_compiled_on_hash_change=True,
        )
        self._active_name = name
        QMessageBox.information(self, "Gespeichert", f"{name} (recipe.yaml + compiled.yaml)")

    def _on_delete_clicked(self) -> None:
        if not self._active_name:
            QMessageBox.information(self, "Delete", "Kein Rezept geladen.")
            return

        name = self._active_name

        reply = QMessageBox.question(
            self,
            "Löschen",
            f"Rezept '{name}' wirklich löschen? (draft + compiled + runs)",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if reply != QMessageBox.StandardButton.Yes:
            return

        self.repo.delete(name)
        self._active_name = None
        self._active_model = None
        self._active_compiled = None

        self._load_default_or_first()
