# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import copy
import logging
from typing import Any, Dict, List, Optional, Tuple

from PyQt5 import uic, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QWidget, QMessageBox, QFileDialog, QShortcut

from .form_builder import (
    clear_form,
    build_form_section,
    apply_values,
    update_visibility_all,
)
from .recipe_model import Recipe

_LOG = logging.getLogger("app.tabs.recipe.recipe_editor_panel")


# ---------- Hilfsfunktionen für UI/Dateipfade ----------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    # UI liegt unter resource/ui/tabs/recipe/
    return os.path.join(_project_root(), "resource", "ui", "tabs", "recipe", filename)

def _looks_like_app_context(ctx) -> bool:
    """
    Duck-typing statt harter Typprüfung:
    Wir prüfen nur, ob die vom Panel benötigten Felder vorhanden sind.
    """
    if ctx is None:
        return False
    needed = ("paths", "recipes_yaml", "recipe_params", "units")
    return all(hasattr(ctx, k) for k in needed)


class RecipeEditorPanel(QWidget):
    """
    LINKES PANEL – Rezept-Erstellung/-Edit:
      - "Delete | Load | Create" (Buttons)
      - Auswahl Tool/Substrate/Mount (Combos, in der UI definiert)
      - Rezept-Parameter-Formulare (Globals + Path*)
      - Seiten-Checkboxen (top/front/back/left/right/helix)
      - Button "Update Viewer" -> signalisiert der Preview, dass neu gerendert werden soll

    Externe Signals:
      - recipeChanged(model: Recipe, sides: List[str])   -> wenn Viewer-Update gewünscht ist
      - recipeDirty(model: Recipe, sides: List[str])     -> bei inhaltlichen Änderungen (z.B. Combos/Form/Seiten)
    """
    recipeChanged = pyqtSignal(object, list)
    recipeDirty = pyqtSignal(object, list)

    def __init__(self, *, ctx, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        if not _looks_like_app_context(self.ctx):
            raise RuntimeError(
                "RecipeEditorPanel: ungültiger ctx – erwartete Felder fehlen "
                "(benötigt: paths, recipes_yaml, recipe_params, units)"
            )

        uic.loadUi(_ui_path("recipe_editor_panel.ui"), self)

        # Form-Spezifikationen (aus recipes.yaml -> recipe_params)
        self._specs: Dict[str, Any] = dict(self.ctx.recipe_params)
        self._param_widgets: Dict[str, Any] = {}
        self._visibility_rules: Dict[str, Any] = {}

        # Aktuelles Rezept (als Dict, wird nach außen als Recipe-Model gewandelt)
        self._current_recipe: Dict[str, Any] = self._default_recipe()

        # Shortcuts
        QShortcut(QtGui.QKeySequence("Ctrl+S"), self, activated=self._on_save_clicked)

        # Wiring: Top-Leiste
        if hasattr(self, "btnDelete"):  self.btnDelete.clicked.connect(self._on_delete_clicked)
        if hasattr(self, "btnLoad"):    self.btnLoad.clicked.connect(self._on_load_clicked)
        if hasattr(self, "btnCreate"):  self.btnCreate.clicked.connect(self._on_create_clicked)
        if hasattr(self, "btnUpdateViewer"): self.btnUpdateViewer.clicked.connect(self._emit_recipe_changed)

        # Combo-Änderungen
        if hasattr(self, "comboTool"):      self.comboTool.currentIndexChanged.connect(self._on_any_input_changed)
        if hasattr(self, "comboSubstrate"): self.comboSubstrate.currentIndexChanged.connect(self._on_substrate_changed)
        if hasattr(self, "comboMount"):     self.comboMount.currentIndexChanged.connect(self._on_any_input_changed)

        # Seiten-Checkboxen
        for nm in ["checkTop","checkFront","checkBack","checkLeft","checkRight","checkHelix"]:
            if hasattr(self, nm):
                getattr(self, nm).toggled.connect(self._on_any_input_changed)

        # Rezeptname
        if hasattr(self, "editRecipeName"):
            self.editRecipeName.textChanged.connect(self._on_any_input_changed)

        # Rezept-Typ (StackedWidget-Page)
        if hasattr(self, "comboRecipeType"):
            self.comboRecipeType.currentIndexChanged.connect(self._on_recipe_type_changed)

        # Speichern (falls im linken Panel vorhanden)
        if hasattr(self, "btnSave"): self.btnSave.clicked.connect(self._on_save_clicked)

        # Formularbereiche initial aufbauen
        self._rebuild_all_forms(active_index=self._page_index_for("meander", "plane"))
        self._apply_recipe_to_forms(self._current_recipe)
        if hasattr(self, "editRecipeName"):
            self.editRecipeName.setText(str(self._current_recipe.get("id") or ""))

        # Standard-Auswahl-Seite (Heuristik nach Substrat, anfänglich "top")
        self._apply_side_to_ui(self._current_recipe.get("side", "top"))

    # ---------- Defaults ----------
    def _default_recipe(self) -> Dict[str, Any]:
        defaults = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        return {
            "id": "unsaved",
            "description": "",
            "tool": None,
            "substrate": None,
            "substrates": [],
            "substrate_mount": None,
            "mount": None,
            "side": defaults.get("side", "top"),
            "parameters": defaults,
            "path": {
                "type": "meander",
                "mode": "plane",
            },
        }

    # ---------- Formularaufbau ----------
    def _rebuild_all_forms(self, active_index: int):
        # Alten State verwerfen
        self._param_widgets.clear()
        self._visibility_rules.clear()

        # Globals
        if hasattr(self, "formGlobals"):
            clear_form(self.formGlobals)
            gspec = self._specs.get("globals", {})
            build_form_section(
                self.formGlobals, gspec, prefix="globals",
                param_widgets=self._param_widgets,
                visibility_rules=self._visibility_rules
            )

        # Path-Seiten
        for frm in ("formMeanderPlane", "formSpiralPlane", "formSpiralCylinder", "formExplicit"):
            if hasattr(self, frm):
                clear_form(getattr(self, frm))

        mapping: List[Tuple[str, str]] = [
            ("formMeanderPlane",   "path.meander.plane"),
            ("formSpiralPlane",    "path.spiral.plane"),
            ("formSpiralCylinder", "path.spiral.cylinder"),
            ("formExplicit",       "path.explicit"),
        ]
        for form_name, skey in mapping:
            if not hasattr(self, form_name):
                continue
            spec = self._specs.get(skey, {})
            build_form_section(
                getattr(self, form_name), spec,
                prefix="path",
                param_widgets=self._param_widgets,
                visibility_rules=self._visibility_rules
            )

        if hasattr(self, "stackTypes"):
            self.stackTypes.setCurrentIndex(int(active_index))
        update_visibility_all(self._param_widgets, self._visibility_rules)

    def _apply_recipe_to_forms(self, recipe: Dict[str, Any]):
        # globals: defaults + overrides
        gvals = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        gvals.update(recipe.get("parameters", {}))
        apply_values("globals", gvals, self._param_widgets)

        # path
        apply_values("path", recipe.get("path", {}), self._param_widgets)
        update_visibility_all(self._param_widgets, self._visibility_rules)

        # path-type/mode -> UI
        path = recipe.get("path", {})
        rtype = path.get("type", "meander")
        rmode = path.get("mode", "plane")
        if hasattr(self, "comboRecipeType"):
            idx = self.comboRecipeType.findText(self._type_label_for(rtype, rmode))
            self.comboRecipeType.setCurrentIndex(0 if idx < 0 else idx)
        if hasattr(self, "stackTypes"):
            self.stackTypes.setCurrentIndex(self._page_index_for(rtype, rmode))

    # ---------- Formular → Recipe ----------
    def _build_recipe_from_forms(self) -> Dict[str, Any]:
        src = copy.deepcopy(self._current_recipe) if self._current_recipe else {}
        rid = (self.editRecipeName.text() or "").strip() if hasattr(self, "editRecipeName") else ""
        if not rid:
            rid = src.get("id") or "recipe"

        tool = src.get("tool")
        if hasattr(self, "comboTool") and self.comboTool.currentText():
            t = self.comboTool.currentText().strip()
            tool = t or None

        substrate = src.get("substrate") or (src.get("substrates")[0] if src.get("substrates") else None)
        if hasattr(self, "comboSubstrate") and self.comboSubstrate.currentText():
            s = self.comboSubstrate.currentText().strip()
            substrate = s or None

        substrate_mount = src.get("substrate_mount") or src.get("mount")
        if hasattr(self, "comboMount") and self.comboMount.currentText():
            m = self.comboMount.currentText().strip()
            substrate_mount = m or None

        rec = {
            "id":               rid,
            "description":      src.get("description", ""),
            "tool":             tool,
            "substrate":        substrate,
            "substrates":       [substrate] if substrate else [],
            "substrate_mount":  substrate_mount,
            "mount":            substrate_mount,
            "side":             self._single_saved_side(),
            "parameters":       self._collect_globals(),
            "path":             self._collect_path_for(self._spec_key_for_page_index(self._current_page_index())),
        }
        return rec

    def _recipe_model(self) -> Recipe:
        return Recipe.from_dict(self._build_recipe_from_forms())

    # ---------- Collect helpers ----------
    def _collect_globals(self) -> Dict[str, Any]:
        res: Dict[str, Any] = {}
        gspec = self._specs.get("globals", {})
        for key in gspec.keys():
            w = self._param_widgets.get(f"globals.{key}")
            if w is None: continue
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
        parts = spec_key.split(".")  # "path.meander.plane"
        if len(parts) == 3:
            _, ptype, pmode = parts
        else:
            ptype, pmode = "meander", "plane"

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

    # ---------- Seiten-Logik ----------
    def _apply_side_to_ui(self, side: Optional[str]):
        side = (side or "top").strip().lower()
        states = {
            "top":   side == "top",
            "front": side == "front",
            "back":  side == "back",
            "left":  side == "left",
            "right": side == "right",
            "helix": side == "helix",
        }
        self._set_side_checks(**states)

    def _set_side_checks(self, *, top: bool, front: bool, back: bool, left: bool, right: bool, helix: bool):
        mapping = {
            "checkTop": top, "checkFront": front, "checkBack": back,
            "checkLeft": left, "checkRight": right, "checkHelix": helix
        }
        for name, val in mapping.items():
            w = getattr(self, name, None)
            if w is not None:
                prev = w.blockSignals(True)
                w.setChecked(bool(val))
                w.blockSignals(prev)

    def _selected_sides(self) -> List[str]:
        sides = []
        if getattr(self, "checkTop", None)   and self.checkTop.isChecked():   sides.append("top")
        if getattr(self, "checkFront", None) and self.checkFront.isChecked(): sides.append("front")
        if getattr(self, "checkBack", None)  and self.checkBack.isChecked():  sides.append("back")
        if getattr(self, "checkLeft", None)  and self.checkLeft.isChecked():  sides.append("left")
        if getattr(self, "checkRight", None) and self.checkRight.isChecked(): sides.append("right")
        if getattr(self, "checkHelix", None) and self.checkHelix.isChecked(): sides.append("helix")
        return sides or ["top"]

    def _single_saved_side(self) -> str:
        sel = self._selected_sides()
        return sel[0] if len(sel) >= 1 else "top"

    # ---------- UI Events ----------
    def _on_any_input_changed(self, *_):
        # Heuristik: Substratwechsel legt Seite fest
        if hasattr(self, "comboSubstrate") and self.sender() is self.comboSubstrate:
            self._heuristic_side_by_substrate()
        # Dirty -> notify
        self.recipeDirty.emit(self._recipe_model(), self._selected_sides())

    def _heuristic_side_by_substrate(self):
        s = self.comboSubstrate.currentText().lower() if hasattr(self, "comboSubstrate") else ""
        if "wafer" in s:
            self._apply_side_to_ui("top")
        elif "tube" in s or "cylinder" in s:
            self._apply_side_to_ui("helix")
        else:
            self._apply_side_to_ui("top")

    def _on_substrate_changed(self, *_):
        self._heuristic_side_by_substrate()
        self.recipeDirty.emit(self._recipe_model(), self._selected_sides())

    def _on_recipe_type_changed(self, idx: int):
        if hasattr(self, "stackTypes"):
            self.stackTypes.setCurrentIndex(idx)
        update_visibility_all(self._param_widgets, self._visibility_rules)
        self.recipeDirty.emit(self._recipe_model(), self._selected_sides())

    # ---------- Topbar Buttons ----------
    def _on_delete_clicked(self):
        # Zurück auf Defaults
        self._current_recipe = self._default_recipe()
        self._rebuild_all_forms(active_index=self._page_index_for("meander", "plane"))
        self._apply_recipe_to_forms(self._current_recipe)
        if hasattr(self, "editRecipeName"):
            self.editRecipeName.setText("unsaved")
        self._apply_side_to_ui("top")
        self.recipeDirty.emit(self._recipe_model(), self._selected_sides())

    def _on_load_clicked(self):
        start_dir = self.ctx.paths.recipe_dir
        fname, _ = QFileDialog.getOpenFileName(self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)
            rec = model.to_dict()
            self._current_recipe = rec

            rtype = rec.get("path", {}).get("type", "meander")
            rmode = rec.get("path", {}).get("mode", "plane")
            self._rebuild_all_forms(active_index=self._page_index_for(rtype, rmode))
            self._apply_recipe_to_forms(rec)

            if hasattr(self, "editRecipeName"):
                self.editRecipeName.setText(str(rec.get("id") or ""))

            self._apply_side_to_ui(rec.get("side", "top"))
            self.recipeChanged.emit(self._recipe_model(), self._selected_sides())
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_create_clicked(self):
        # neues leeres Rezept
        self._current_recipe = self._default_recipe()
        self._rebuild_all_forms(active_index=self._page_index_for("meander", "plane"))
        self._apply_recipe_to_forms(self._current_recipe)
        if hasattr(self, "editRecipeName"):
            self.editRecipeName.setText("recipe")
        self._apply_side_to_ui("top")
        self.recipeDirty.emit(self._recipe_model(), self._selected_sides())

    def _on_save_clicked(self):
        rec = self._recipe_model().to_dict()
        rid = rec.get("id") or "recipe"
        default_name = os.path.join(self.ctx.paths.recipe_dir, f"{rid}.yaml")
        fname, _ = QFileDialog.getSaveFileName(self, "Rezept speichern", default_name, "YAML (*.yaml *.yml)")
        if not fname:
            return
        try:
            import yaml
            with open(fname, "w", encoding="utf-8") as f:
                yaml.safe_dump(rec, f, allow_unicode=True, sort_keys=False)
            QMessageBox.information(self, "Gespeichert", f"Rezept gespeichert:\n{fname}")
        except Exception as e:
            QMessageBox.critical(self, "Speicherfehler", str(e))

    def _emit_recipe_changed(self):
        self.recipeChanged.emit(self._recipe_model(), self._selected_sides())

    # ---------- Helpers: Typ/Seiten-Mapping ----------
    def _current_page_index(self) -> int:
        if hasattr(self, "stackTypes"):
            return int(self.stackTypes.currentIndex())
        return 0

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
        keys = [
            "path.meander.plane",
            "path.spiral.plane",
            "path.spiral.cylinder",
            "path.explicit",
        ]
        if idx < 0 or idx >= len(keys):
            return keys[0]
        return keys[idx]

    # ---------- Öffentliche Convenience-APIs ----------
    def current_model(self) -> Recipe:
        """Aktuelles Rezept-Modell (für RecipeTab/PreviewTab)."""
        return self._recipe_model()

    def selected_sides(self) -> List[str]:
        """Aktuelle Seitenauswahl als Liste."""
        return self._selected_sides()
