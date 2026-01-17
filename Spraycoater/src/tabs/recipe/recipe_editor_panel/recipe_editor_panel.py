# -*- coding: utf-8 -*-
# File: app/tabs/recipe/recipe_editor_panel/recipe_editor_panel.py
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

from model.recipe.recipe import Recipe, Draft, PathSide, PoseQuat
from model.recipe.path_builder import PathBuilder
from model.recipe.recipe_store import RecipeStore
from model.recipe.recipe_repo import RecipeRepo

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


def _model_valid_save(model: Any) -> bool:
    """
    Save-Gate (SSoT):
      - Preferred: model.info['validSave'] (set by CoatingPreviewPanel after raycast)
      - Fallback: model.valid_save / model.validSave attributes (if present)
    """
    try:
        info = getattr(model, "info", None)
        if isinstance(info, dict) and "validSave" in info:
            return bool(info.get("validSave"))
    except Exception:
        pass

    try:
        if hasattr(model, "valid_save"):
            return bool(getattr(model, "valid_save"))
        if hasattr(model, "validSave"):
            return bool(getattr(model, "validSave"))
    except Exception:
        pass

    return False


def _ensure_validsave_fields(model: Any) -> None:
    """
    Ensure the fields exist (important for new + load),
    so the UI always has deterministic defaults.
    """
    try:
        if not isinstance(getattr(model, "info", None), dict):
            model.info = {}
        if "validSave" not in model.info:
            model.info["validSave"] = False
        if "validSaveReason" not in model.info:
            model.info["validSaveReason"] = "no_preview"
    except Exception:
        pass


class RecipeEditorPanel(QWidget):
    """
    Editor (Folder-name based).

    Persistence:
      <recipes_root_dir>/<recipe_name>/{params.yaml,draft.yaml,planned_traj.yaml,executed_traj.yaml}

    UI:
      - Meta->Recipe name = folder name (SSoT key)
      - Selection->Recipe = template/type from store.recipes (drives defaults)

    Save policy (STRICT):
      - Saving is only allowed if model.info['validSave'] == True.
      - If invalid -> MessageBox "Can not be saved" (per requirement).
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

        # Catalog templates by template-id (e.g. "cube")
        self._recipes_by_id: Dict[str, Dict[str, Any]] = {}

        self._active_model: Optional[Recipe] = None
        self._active_template_id: str = ""  # template/type from store
        self._active_name: str = ""         # folder name (persistence key)

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
            self.content.sel_recipe.currentIndexChanged.connect(self._on_template_select_changed)

        # init
        self._rebuild_recipe_defs()
        self._populate_template_combo()
        self._load_default_into_editor(trigger_preview=True)

    # ---------------------------------------------------------
    # Catalog / Selection (templates)
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
    # Helpers (name / id)
    # ---------------------------------------------------------

    def _current_recipe_name(self) -> str:
        # Meta->Recipe name (folder key)
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
        # deterministic placeholder to satisfy strict Recipe.from_dict()
        tid = str(template_id or "recipe").strip() or "recipe"
        return f"{tid}_01"

    # ---------------------------------------------------------
    # Model creation (from template)
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

        model = Recipe.from_dict(
            {
                "id": placeholder_id,
                "description": rec_def.get("description") or "",
                "tool": str(tools[0]),
                "substrate": str(subs[0]),
                "substrate_mount": str(mounts[0]),
                "parameters": params,
                "planner": move_planner,
                "paths_by_side": pbs,
                "trajectories": {},
                "meta": {"template_id": template_id},
            }
        )

        # IMPORTANT: new model starts as not-saveable until preview ran
        _ensure_validsave_fields(model)
        return model

    # ---------------------------------------------------------
    # Draft builder (UI-only)  -> draft.yaml schema (Path v1)
    # ---------------------------------------------------------

    def _ensure_draft(self, model: Recipe) -> None:
        """Create/refresh model.draft from current paths_by_side + globals."""
        pbs = getattr(model, "paths_by_side", None) or {}
        if not isinstance(pbs, dict) or not pbs:
            model.draft = None
            return

        params = getattr(model, "parameters", None) or {}
        if not isinstance(params, dict):
            params = {}

        sample_step_mm = float(params.get("sample_step_mm", 1.0))
        max_points = int(params.get("max_points", 1000))
        globals_params = dict(params)

        sides = [str(s) for s in pbs.keys() if str(s).strip()]
        if not sides:
            model.draft = None
            return

        out_sides: Dict[str, PathSide] = {}

        for side in sides:
            try:
                pd = PathBuilder.from_side(
                    model,
                    side=side,
                    globals_params=globals_params,
                    sample_step_mm=sample_step_mm,
                    max_points=max_points,
                )
            except Exception:
                continue

            P = pd.points_mm
            if P is None:
                continue

            # Plane Z offset: z = stand_off_mm + z_mm (if present)
            try:
                path_dict = dict((pbs.get(side) or {}) if isinstance(pbs.get(side), dict) else {})
                ptype = str(path_dict.get("type") or "").strip().lower()
                stand_off = float(globals_params.get("stand_off_mm", 0.0) or 0.0)
                z_mm = float(path_dict.get("z_mm", 0.0) or 0.0)

                if ptype in (
                    "path.meander.plane",
                    "path.spiral.plane",
                    "path.perimeter_follow.plane",
                ):
                    P = P.copy()
                    if P.shape[0] > 0:
                        P[:, 2] = stand_off + z_mm
            except Exception:
                pass

            P = self._apply_predispense_retreat_points(P, pbs.get(side) or {})
            if P.shape[0] < 1:
                continue

            poses = [
                PoseQuat(
                    x=float(x),
                    y=float(y),
                    z=float(z),
                    qx=0.0,
                    qy=0.0,
                    qz=0.0,
                    qw=1.0,
                )
                for x, y, z in P.tolist()
            ]

            if poses:
                out_sides[side] = PathSide(poses_quat=poses)

        model.draft = Draft(version=1, sides=out_sides) if out_sides else None

    @staticmethod
    def _apply_predispense_retreat_points(P: Any, path_dict: Any) -> Any:
        """Add optional predispense/retreat points along the tangent."""
        import numpy as np

        pts = np.asarray(P, dtype=float).reshape(-1, 3)
        if pts.shape[0] < 2:
            return pts

        pd = dict(path_dict) if isinstance(path_dict, dict) else {}
        pre_mm = 0.0
        ret_mm = 0.0

        try:
            pre = pd.get("predispense")
            if isinstance(pre, dict):
                pre_mm = float(pre.get("extend_mm", 0.0) or 0.0)
        except Exception:
            pre_mm = 0.0

        try:
            ret = pd.get("retreat")
            if isinstance(ret, dict):
                ret_mm = float(ret.get("extend_mm", 0.0) or 0.0)
        except Exception:
            ret_mm = 0.0

        def _unit(v: np.ndarray) -> np.ndarray:
            n = float(np.linalg.norm(v))
            return v / n if n > 1e-12 else np.zeros_like(v)

        out = pts

        if pre_mm > 0.0:
            t0 = _unit(out[1] - out[0])
            if float(np.linalg.norm(t0)) > 0.0:
                pre_pt = out[0] - pre_mm * t0
                out = np.vstack([pre_pt.reshape(1, 3), out])

        if ret_mm > 0.0:
            t1 = _unit(out[-1] - out[-2])
            if float(np.linalg.norm(t1)) > 0.0:
                ret_pt = out[-1] + ret_mm * t1
                out = np.vstack([out, ret_pt.reshape(1, 3)])

        return out

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

        # UI -> model
        self.content.apply_ui_to_model(model)

        name_ui = self._current_recipe_name()
        if name_ui:
            model.id = name_ui
        else:
            model.id = str(model.id or "").strip() or self._default_recipe_name_for_template(template_id)

        self._active_name = model.id
        self._active_template_id = template_id

        # Keep save gate fields stable
        _ensure_validsave_fields(model)

        # Clear trajectories in editor context
        model.trajectories = {}

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

        # Ensure id is the loaded folder name
        model.id = recipe_name

        # IMPORTANT: loaded model must have deterministic save-gate fields.
        # If params.yaml didn't have them yet -> defaults to False.
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
        self._ensure_draft(model)
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

        # STRICT: only allow saving if preview validated (validSave True)
        if not _model_valid_save(model):
            # Requirement: simple messagebox "Can not be saved" (no extra info needed)
            QMessageBox.warning(self, "Save", "Can not be saved")
            return

        # Save by folder key
        try:
            self._ensure_draft(model)
            self.repo.save_from_editor(name, draft=model, compiled=getattr(model, "paths_compiled", None))
        except TypeError:
            self.repo.save_from_editor(name, draft=model)  # type: ignore[arg-type]
        except Exception as e:
            QMessageBox.warning(self, "Save", f"Speichern fehlgeschlagen:\n{e}")
            return

        self._active_name = name
        QMessageBox.information(self, "Gespeichert", f"{name} (params.yaml; draft.yaml falls vorhanden)")

    def _on_delete_clicked(self) -> None:
        name = self._current_recipe_name() or self._active_name
        name = str(name or "").strip()
        if not name:
            QMessageBox.information(self, "Delete", "Kein recipe name ausgewählt (Meta).")
            return

        reply = QMessageBox.question(
            self,
            "Löschen",
            f"Rezept '{name}' wirklich löschen?\n(params + draft + trajectories)",
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
