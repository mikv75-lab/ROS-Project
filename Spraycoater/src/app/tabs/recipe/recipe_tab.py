# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import copy
import yaml
import logging
from typing import Any, Dict, List

from PyQt5 import uic, QtCore, QtGui
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QListWidgetItem, QMessageBox, QShortcut, QFileDialog
)
from pyvistaqt import QtInteractor

# Formular-Helfer (unverändert)
from .form_builder import (
    clear_form,
    build_form_section,
    apply_values,
    update_visibility_all,
)
from .preview import PreviewEngine

_LOG = logging.getLogger("app.tabs.recipe")


# ---------------- Pfad-Helfer ----------------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", filename)


class RecipeTab(QWidget):
    """
    Rezept-Editor (modernisiert):
      - Shapes: nur Hemisphäre
      - Pfade: Meander, Spiral (als zusammenhängende Polylines)
      - Rezept-abhängige Controls gesperrt bis Auswahl
      - Validate / Optimize über Bridge
      - Speichern über Dateidialog (Startordner: ctx.paths.recipe_dir)
      - Kamera-Views: ISO / Top / Front / Right
    """

    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        if self.ctx is None:
            raise RuntimeError("RecipeTab benötigt ctx=AppContext")

        uic.loadUi(_ui_path("recipe_tab.ui"), self)

        # Plotter / Preview
        if self.plotContainer.layout() is None:
            self.plotContainer.setLayout(QVBoxLayout(self.plotContainer))
        self.plotter = QtInteractor(self.plotContainer)
        self.plotContainer.layout().addWidget(self.plotter)
        self.preview = PreviewEngine(self.plotter)

        # Daten/Specs
        self._recipes: List[Dict[str, Any]] = list(self.ctx.recipes)
        self._specs: Dict[str, Any] = dict(self.ctx.recipe_params)
        self._param_widgets: Dict[str, Any] = {}
        self._visibility_rules: Dict[str, Any] = {}

        # Rezept-abhängige Controls sperren
        self._set_recipe_dependent_controls_enabled(False)

        # Liste
        self._fill_recipe_list()
        self.listRecipes.currentRowChanged.connect(self._on_recipe_selected)

        # Preview controls
        if hasattr(self, "btnViewIso"):   self.btnViewIso.clicked.connect(self.preview.view_iso)
        if hasattr(self, "btnViewTop"):   self.btnViewTop.clicked.connect(self.preview.view_top)
        if hasattr(self, "btnViewFront"): self.btnViewFront.clicked.connect(self.preview.view_front)
        if hasattr(self, "btnViewRight"): self.btnViewRight.clicked.connect(self.preview.view_right)

        # Umschalter
        if hasattr(self, "comboShape"):  self.comboShape.currentIndexChanged.connect(self._auto_preview)
        if hasattr(self, "comboPath"):   self.comboPath.currentIndexChanged.connect(self._auto_preview)
        if hasattr(self, "checkNormals"): self.checkNormals.toggled.connect(self._auto_preview)
        if hasattr(self, "checkRaycasts"): self.checkRaycasts.toggled.connect(self._auto_preview)

        # Typwechsel
        self.comboRecipeType.currentIndexChanged.connect(self._on_type_changed)

        # Aktionen
        if hasattr(self, "btnSaveRecipe"):   self.btnSaveRecipe.clicked.connect(self._on_save_clicked)
        if hasattr(self, "btnValidate"):     self.btnValidate.clicked.connect(self._on_validate_clicked)
        if hasattr(self, "btnOptimize"):     self.btnOptimize.clicked.connect(self._on_optimize_clicked)

        # Shortcuts
        QShortcut(QtGui.QKeySequence("Ctrl+S"), self, activated=self._on_save_clicked)

        # Erstes Rezept
        if self.listRecipes.count() > 0:
            self.listRecipes.setCurrentRow(0)
        else:
            self._build_preview_from_ui()

    # ---------------- Controls sperren ----------------
    def _set_recipe_dependent_controls_enabled(self, enabled: bool) -> None:
        candidates = [
            # Reihenfolge: globale Parameter, Rezepttyp, Preview -> okay
            "comboRecipeType",
            "formGlobals",
            "stackTypes",
            "comboPath",
            "checkNormals",
            "checkRaycasts",
            "btnViewIso", "btnViewTop", "btnViewFront", "btnViewRight",
            "btnValidate", "btnOptimize",
            "btnSaveRecipe",
        ]
        for name in candidates:
            w = getattr(self, name, None)
            if w is not None:
                w.setEnabled(enabled)

    # ---------------- Liste / Auswahl ----------------
    def _fill_recipe_list(self):
        self.listRecipes.clear()
        for rec in self._recipes:
            item = QListWidgetItem(f"{rec.get('id','?')} — {rec.get('description','')}")
            item.setData(QtCore.Qt.UserRole, rec)
            self.listRecipes.addItem(item)

    def _current_recipe_from_list(self) -> Dict[str, Any] | None:
        row = self.listRecipes.currentRow()
        if row < 0:
            return None
        it = self.listRecipes.item(row)
        return copy.deepcopy(it.data(QtCore.Qt.UserRole))

    def _page_index_for(self, rtype: str, rmode: str) -> int:
        if rtype == "meander" and rmode == "plane":     return 0
        if rtype == "spiral"  and rmode == "plane":     return 1
        if rtype == "spiral"  and rmode == "cylinder":  return 2
        if rtype == "explicit":                         return 3
        return 0

    def _type_label_for(self, rtype: str, rmode: str) -> str:
        if rtype == "meander" and rmode == "plane":     return "Meander (plane)"
        if rtype == "spiral"  and rmode == "plane":     return "Spiral (plane)"
        if rtype == "spiral"  and rmode == "cylinder":  return "Spiral (cylinder)"
        if rtype == "explicit":                         return "Explicit"
        return "Meander (plane)"

    def _spec_key_for_page_index(self, idx: int) -> str:
        return [
            "path.meander.plane",
            "path.spiral.plane",
            "path.spiral.cylinder",
            "path.explicit",
        ][idx]

    def _on_recipe_selected(self, row: int):
        if row < 0:
            self._set_recipe_dependent_controls_enabled(False)
            return
        recipe = self._current_recipe_from_list()
        if recipe is None:
            self._set_recipe_dependent_controls_enabled(False)
            return

        self._set_recipe_dependent_controls_enabled(True)

        # type/mode -> UI
        path = recipe.get("path", {})
        rtype = path.get("type", "meander")
        rmode = path.get("mode", "plane")
        type_label = self._type_label_for(rtype, rmode)

        idx = self.comboRecipeType.findText(type_label)
        if idx < 0:
            idx = 0
        self.comboRecipeType.setCurrentIndex(idx)
        self.stackTypes.setCurrentIndex(self._page_index_for(rtype, rmode))

        # Forme neu + Werte einblenden
        self._rebuild_all_forms(active_index=self.stackTypes.currentIndex())
        self._apply_recipe_to_forms(recipe)

        # Preview Presets in der UI
        if hasattr(self, "comboShape"):
            # nur Hemisphäre
            hemi_index = self.comboShape.findText("Hemisphäre")
            if hemi_index >= 0:
                self.comboShape.setCurrentIndex(hemi_index)
        if hasattr(self, "comboPath"):
            self.comboPath.setCurrentText("Meander" if rtype == "meander" else "Spiral")

        self._build_preview_from_ui()

    def _on_type_changed(self, idx: int):
        self.stackTypes.setCurrentIndex(idx)
        update_visibility_all(self._param_widgets, self._visibility_rules)

    # ---------------- Formulardynamik ----------------
    def _rebuild_all_forms(self, active_index: int):
        self._param_widgets.clear()
        self._visibility_rules.clear()

        clear_form(self.formGlobals)
        gspec = self._specs.get("globals", {})
        build_form_section(self.formGlobals, gspec, prefix="globals",
                           param_widgets=self._param_widgets,
                           visibility_rules=self._visibility_rules)

        for frm in (self.formMeanderPlane, self.formSpiralPlane, self.formSpiralCylinder, self.formExplicit):
            clear_form(frm)

        mapping = [
            (self.formMeanderPlane,   "path.meander.plane"),
            (self.formSpiralPlane,    "path.spiral.plane"),
            (self.formSpiralCylinder, "path.spiral.cylinder"),
            (self.formExplicit,       "path.explicit"),
        ]
        for form, skey in mapping:
            spec = self._specs.get(skey, {})
            build_form_section(form, spec, prefix="path",
                               param_widgets=self._param_widgets,
                               visibility_rules=self._visibility_rules)

        self.stackTypes.setCurrentIndex(active_index)
        update_visibility_all(self._param_widgets, self._visibility_rules)

    def _apply_recipe_to_forms(self, recipe: Dict[str, Any]):
        gvals = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        gvals.update(recipe.get("parameters", {}))
        apply_values("globals", gvals, self._param_widgets)

        apply_values("path", recipe.get("path", {}), self._param_widgets)
        update_visibility_all(self._param_widgets, self._visibility_rules)

    # ---------------- Rezept aus Formular ----------------
    def _collect_globals(self) -> Dict[str, Any]:
        res: Dict[str, Any] = {}
        gspec = self._specs.get("globals", {})
        for key in gspec.keys():
            w = self._param_widgets.get(f"globals.{key}")
            if w is None:
                continue
            if hasattr(w, "value"):
                res[key] = float(w.value())
            elif hasattr(w, "isChecked"):
                res[key] = bool(w.isChecked())
            elif hasattr(w, "currentText"):
                res[key] = str(w.currentText())
            elif isinstance(w, tuple):
                res[key] = [float(w[0].value()), float(w[1].value())]
        return res

    @staticmethod
    def _set_nested(d: Dict[str, Any], dotted: str, value: Any) -> None:
        cur = d
        parts = dotted.split(".")
        for p in parts[:-1]:
            if p not in cur or not isinstance(cur[p], dict):
                cur[p] = {}
            cur = cur[p]
        cur[parts[-1]] = value

    def _collect_path_for(self, spec_key: str) -> Dict[str, Any]:
        path_obj: Dict[str, Any] = {}
        _, ptype, pmode = spec_key.split(".")
        if ptype == "explicit":
            path_obj["type"] = "explicit"
        else:
            path_obj["type"] = ptype
            path_obj["mode"] = pmode

        section = self._specs.get(spec_key, {})
        for dotted_key in section.keys():
            w = self._param_widgets.get(f"path.{dotted_key}")
            if w is None:
                continue
            if hasattr(w, "value"):
                val = float(w.value())
            elif hasattr(w, "isChecked"):
                val = bool(w.isChecked())
            elif hasattr(w, "currentText"):
                val = str(w.currentText())
            elif isinstance(w, tuple):
                val = [float(w[0].value()), float(w[1].value())]
            else:
                continue
            self._set_nested(path_obj, dotted_key, val)
        return path_obj

    def _build_recipe_from_forms(self) -> Dict[str, Any] | None:
        src = self._current_recipe_from_list()
        if src is None:
            return None
        return {
            "id":          src.get("id"),
            "description": src.get("description"),
            "tool":        src.get("tool"),
            "substrates":  src.get("substrates", []),
            "mount":       src.get("mount"),
            "side":        src.get("side"),
            "parameters":  self._collect_globals(),
            "path":        self._collect_path_for(self._spec_key_for_page_index(self.stackTypes.currentIndex())),
        }

    # ---------------- Aktionen ----------------
    def _on_save_clicked(self):
        rec = self._build_recipe_from_forms()
        if not rec:
            return

        # Speicherdialog (kein Auto-Validate hier)
        rid = rec.get("id") or "recipe"
        default_name = os.path.join(self.ctx.paths.recipe_dir, f"{rid}.yaml")
        fname, _ = QFileDialog.getSaveFileName(
            self,
            "Rezept speichern",
            default_name,
            "YAML (*.yaml *.yml)"
        )
        if not fname:
            return

        try:
            with open(fname, "w", encoding="utf-8") as f:
                yaml.safe_dump(rec, f, allow_unicode=True, sort_keys=False)
            QMessageBox.information(self, "Gespeichert", f"Rezept gespeichert:\n{fname}")
        except Exception as e:
            QMessageBox.critical(self, "Speicherfehler", str(e))

    def _on_validate_clicked(self):
        rec = self._build_recipe_from_forms()
        if not rec:
            return
        try:
            # Bridge führt bereits eine syntaktische Prüfung durch; wenn verbunden, auch ROS-seitig.
            resp = self.bridge.validate(rec, syntactic_only=False, timeout=5.0)
            if resp.ok:
                QMessageBox.information(self, "Validate", resp.message or "OK")
            else:
                QMessageBox.warning(self, "Validate", resp.message or "Fehler")
        except Exception as e:
            QMessageBox.critical(self, "Validate-Fehler", str(e))

    def _on_optimize_clicked(self):
        rec = self._build_recipe_from_forms()
        if not rec:
            return
        try:
            resp = self.bridge.optimize(rec, timeout=10.0)
            if resp.ok:
                QMessageBox.information(self, "Optimize", resp.message or "OK")
            else:
                QMessageBox.warning(self, "Optimize", resp.message or "Fehler")
        except Exception as e:
            QMessageBox.critical(self, "Optimize-Fehler", str(e))

    # ---------------- Preview ----------------
    def _auto_preview(self):
        self._build_preview_from_ui()

    def _flip_from_recipe(self, recipe: Dict[str, Any]) -> bool:
        # Flip-Heuristik über 'side'
        flag = str(recipe.get("side", "")).strip().lower()
        return flag in {"back", "flipped", "flip", "reverse"}

    def _build_preview_from_ui(self):
        # Shape: nur Hemisphäre
        shape = getattr(self, "comboShape", None)
        shape_name = shape.currentText() if shape else "Hemisphäre"

        # Path: Meander/Spiral
        path  = getattr(self, "comboPath", None)
        path_name  = path.currentText() if path else "Meander"

        normals = getattr(self, "checkNormals", None)
        show_normals = bool(normals.isChecked()) if normals else False

        raycasts = getattr(self, "checkRaycasts", None)
        show_rays = bool(raycasts.isChecked()) if raycasts else False

        cur = self._current_recipe_from_list()
        flip = self._flip_from_recipe(cur) if cur else False

        try:
            self.preview.build_preview(
                shape_name, path_name, show_normals,
                flip=flip, show_rays=show_rays
            )
        except Exception as e:
            _LOG.exception("Preview failed: %s", e)