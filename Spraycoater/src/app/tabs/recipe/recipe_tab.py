# Spraycoater/src/app/tabs/recipe/recipe_tab.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import copy
import yaml
import logging
from typing import Any, Dict, List, Optional

from PyQt5 import uic, QtCore, QtGui
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QMessageBox, QShortcut, QFileDialog
from pyvistaqt import QtInteractor

from .form_builder import (
    clear_form,
    build_form_section,
    apply_values,
    update_visibility_all,
)
from .preview import PreviewEngine
from .recipe_model import Recipe
from .trajectory_builder import TrajectoryBuilder

_LOG = logging.getLogger("app.tabs.recipe")


# ---------------- Pfad-Helfer ----------------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", filename)


class RecipeTab(QWidget):
    """
    Rezept-Editor (ohne Rezepte-Liste):
      - Rezeptname-Feld + Seiten-Checkboxen (Front/Back/Left/Right) -> Mehrfach-Vorschau (Overlay)
      - Speichern/Laden via Dateidialog (Startordner: ctx.paths.recipe_dir)
      - Validate/Optimize: Recipe.validate_* + Bridge
      - Kamera-Views: ISO / Top / Front / Right
      - Dynamische Formulare über 'recipe_params' (globals + path-sections)
      - Preview zeichnet die **echte TCP-Trajektorie** (1:1 ROS-Posen)
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

        # Trajektorien-Builder (echte TCP-Bahnen, inkl. optionaler Mesh-Projektion)
        self.traj_builder = TrajectoryBuilder()

        # Specs / Formularsteuerung
        self._specs: Dict[str, Any] = dict(self.ctx.recipe_params)
        self._param_widgets: Dict[str, Any] = {}
        self._visibility_rules: Dict[str, Any] = {}

        # aktuelles Rezept (ein einzelnes, dict-Form für UI)
        self._current_recipe: Dict[str, Any] = self._default_recipe()

        # Preview controls
        if hasattr(self, "btnViewIso"):   self.btnViewIso.clicked.connect(self.preview.view_iso)
        if hasattr(self, "btnViewTop"):   self.btnViewTop.clicked.connect(self.preview.view_top)
        if hasattr(self, "btnViewFront"): self.btnViewFront.clicked.connect(self.preview.view_front)
        if hasattr(self, "btnViewRight"): self.btnViewRight.clicked.connect(self.preview.view_right)
        if hasattr(self, "checkNormals"): self.checkNormals.toggled.connect(self._auto_preview)
        if hasattr(self, "checkRaycasts"): self.checkRaycasts.toggled.connect(self._auto_preview)
        for name in ("checkSideFront", "checkSideBack", "checkSideLeft", "checkSideRight"):
            w = getattr(self, name, None)
            if w is not None:
                w.toggled.connect(self._auto_preview)
        if hasattr(self, "editRecipeName"):
            self.editRecipeName.textChanged.connect(self._on_name_changed)

        # Typwechsel
        self.comboRecipeType.currentIndexChanged.connect(self._on_type_changed)

        # Aktionen
        if hasattr(self, "btnSaveRecipe"):   self.btnSaveRecipe.clicked.connect(self._on_save_clicked)
        if hasattr(self, "btnLoadRecipe"):   self.btnLoadRecipe.clicked.connect(self._on_load_clicked)
        if hasattr(self, "btnValidate"):     self.btnValidate.clicked.connect(self._on_validate_clicked)
        if hasattr(self, "btnOptimize"):     self.btnOptimize.clicked.connect(self._on_optimize_clicked)

        # Shortcut
        QShortcut(QtGui.QKeySequence("Ctrl+S"), self, activated=self._on_save_clicked)

        # UI initialisieren
        self._rebuild_all_forms(active_index=self.stackTypes.currentIndex())
        self._apply_recipe_to_forms(self._current_recipe)
        if hasattr(self, "editRecipeName"):
            self.editRecipeName.setText(str(self._current_recipe.get("id") or ""))
        self._apply_side_to_ui(self._current_recipe.get("side"))
        self._build_preview_from_ui()

    # ---------------- Defaults ----------------
    def _default_recipe(self) -> Dict[str, Any]:
        defaults = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        return {
            "id": "unsaved",
            "description": "",
            "tool": None,              # Pflicht (vor Start)
            "substrate": None,         # genau eines, Pflicht (vor Start)
            "substrates": [],          # legacy mirror
            "substrate_mount": None,   # Pflicht (vor Start)
            "mount": None,             # legacy mirror
            "side": defaults.get("side", "front"),  # UI-speicher (einzeln)
            "parameters": defaults,
            "path": {
                "type": "meander",
                "mode": "plane"
            },
        }

    # ---------------- Formulardynamik ----------------
    def _rebuild_all_forms(self, active_index: int):
        self._param_widgets.clear()
        self._visibility_rules.clear()

        # Globals
        clear_form(self.formGlobals)
        gspec = self._specs.get("globals", {})
        build_form_section(
            self.formGlobals, gspec, prefix="globals",
            param_widgets=self._param_widgets,
            visibility_rules=self._visibility_rules
        )

        # Path-seiten zurücksetzen
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
            build_form_section(
                form, spec, prefix="path",
                param_widgets=self._param_widgets,
                visibility_rules=self._visibility_rules
            )

        self.stackTypes.setCurrentIndex(active_index)
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
        idx = self.comboRecipeType.findText(self._type_label_for(rtype, rmode))
        self.comboRecipeType.setCurrentIndex(0 if idx < 0 else idx)
        self.stackTypes.setCurrentIndex(self._page_index_for(rtype, rmode))

    # ---------------- Helpers: Typ/Seite ----------------
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

    def _apply_side_to_ui(self, side: Optional[str]):
        side = (side or "front").strip().lower()
        self._set_side_checks(front=True, back=False, left=False, right=False)
        if side == "back":
            self._set_side_checks(front=False, back=True, left=False, right=False)
        elif side == "left":
            self._set_side_checks(front=False, back=False, left=True, right=False)
        elif side == "right":
            self._set_side_checks(front=False, back=False, left=False, right=True)

    def _set_side_checks(self, *, front: bool, back: bool, left: bool, right: bool):
        for name, val in (("checkSideFront", front),
                          ("checkSideBack",  back),
                          ("checkSideLeft",  left),
                          ("checkSideRight", right)):
            w = getattr(self, name, None)
            if w is not None:
                prev = w.blockSignals(True)  # _auto_preview nicht mehrfach feuern
                w.setChecked(val)
                w.blockSignals(prev)

    def _selected_sides(self) -> List[str]:
        sides = []
        if getattr(self, "checkSideFront", None) and self.checkSideFront.isChecked():
            sides.append("front")
        if getattr(self, "checkSideBack", None) and self.checkSideBack.isChecked():
            sides.append("back")
        if getattr(self, "checkSideLeft", None) and self.checkSideLeft.isChecked():
            sides.append("left")
        if getattr(self, "checkSideRight", None) and self.checkSideRight.isChecked():
            sides.append("right")
        return sides or ["front"]

    def _single_saved_side(self) -> str:
        sel = self._selected_sides()
        return sel[0] if len(sel) >= 1 else "front"

    # ---------------- Recipe-Objekt aus Formular ----------------
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

    def _build_recipe_from_forms(self) -> Dict[str, Any]:
        src = copy.deepcopy(self._current_recipe) if self._current_recipe else {}
        rid = (self.editRecipeName.text() or "").strip() if hasattr(self, "editRecipeName") else ""
        if not rid:
            rid = src.get("id") or "recipe"

        # Optional: Tool/Substrate/Mount aus Combos, falls vorhanden
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
            "mount":            substrate_mount,   # legacy mirror
            "side":             self._single_saved_side(),   # UI speichert eine Seite
            "parameters":       self._collect_globals(),
            "path":             self._collect_path_for(self._spec_key_for_page_index(self.stackTypes.currentIndex())),
        }
        return rec

    def _recipe_model_from_forms(self) -> Recipe:
        """Konvertiert die aktuelle Formularlage in ein Recipe-Model (mit Normalisierung)."""
        d = self._build_recipe_from_forms()
        return Recipe.from_dict(d)

    # ---------------- Aktionen ----------------
    def _on_save_clicked(self):
        rec = self._build_recipe_from_forms()
        if not rec:
            return
        rid = rec.get("id") or "recipe"
        default_name = os.path.join(self.ctx.paths.recipe_dir, f"{rid}.yaml")
        fname, _ = QFileDialog.getSaveFileName(
            self, "Rezept speichern", default_name, "YAML (*.yaml *.yml)"
        )
        if not fname:
            return

        try:
            # über Recipe-Model serialisieren (schreibt neues Schema + Legacy-Spiegel)
            model = Recipe.from_dict(rec)
            with open(fname, "w", encoding="utf-8") as f:
                yaml.safe_dump(model.to_dict(), f, allow_unicode=True, sort_keys=False)
            QMessageBox.information(self, "Gespeichert", f"Rezept gespeichert:\n{fname}")
            self._current_recipe = model.to_dict()
        except Exception as e:
            QMessageBox.critical(self, "Speicherfehler", str(e))

    def _on_load_clicked(self):
        start_dir = self.ctx.paths.recipe_dir
        fname, _ = QFileDialog.getOpenFileName(
            self, "Rezept laden", start_dir, "YAML (*.yaml *.yml)"
        )
        if not fname:
            return
        try:
            model = Recipe.load_yaml(fname)  # nutzt Model-Normalisierung
            rec = model.to_dict()

            self._current_recipe = rec

            # UI neu aufbauen anhand des (legacy) type/mode aus rec (falls vorhanden)
            rtype = rec.get("path", {}).get("type", "meander")
            rmode = rec.get("path", {}).get("mode", "plane")
            self._rebuild_all_forms(active_index=self._page_index_for(rtype, rmode))
            self._apply_recipe_to_forms(rec)

            if hasattr(self, "editRecipeName"):
                self.editRecipeName.setText(str(rec.get("id") or ""))

            # Seite in UI
            self._apply_side_to_ui(rec.get("side"))
            self._build_preview_from_ui()

            QMessageBox.information(self, "Geladen", f"Rezept geladen:\n{fname}")
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_validate_clicked(self):
        model = self._recipe_model_from_forms()

        # 1) lokale Pflicht-Prüfung
        errs = model.validate_required()

        # 2) Referenzprüfung gegen verfügbare Kataloge (falls im ctx vorhanden)
        tools_yaml       = getattr(self.ctx, "tools_yaml", None)
        mounts_yaml      = getattr(self.ctx, "mounts_yaml", None)
        substrates_yaml  = getattr(self.ctx, "substrates_yaml", None)
        errs += model.validate_references(
            tools_yaml=tools_yaml, mounts_yaml=mounts_yaml, substrates_yaml=substrates_yaml
        )

        if errs:
            QMessageBox.warning(self, "Validate", "Fehler:\n- " + "\n- ".join(map(str, errs)))
            return

        # 3) Bridge (volle Prüfung)
        try:
            resp = self.bridge.validate(model.to_dict(), syntactic_only=False, timeout=5.0)
            if getattr(resp, "ok", False):
                QMessageBox.information(self, "Validate", getattr(resp, "message", "OK") or "OK")
            else:
                QMessageBox.warning(self, "Validate", getattr(resp, "message", "Fehler") or "Fehler")
        except Exception as e:
            QMessageBox.critical(self, "Validate-Fehler", str(e))

    def _on_optimize_clicked(self):
        model = self._recipe_model_from_forms()

        # Mindestprüfung
        errs = model.validate_required()
        if errs:
            QMessageBox.warning(self, "Optimize", "Fehler:\n- " + "\n- ".join(map(str, errs)))
            return

        try:
            resp = self.bridge.optimize(model.to_dict(), timeout=10.0)
            if getattr(resp, "ok", False):
                QMessageBox.information(self, "Optimize", getattr(resp, "message", "OK") or "OK")
            else:
                QMessageBox.warning(self, "Optimize", getattr(resp, "message", "Fehler") or "Fehler")
        except Exception as e:
            QMessageBox.critical(self, "Optimize-Fehler", str(e))

    # ---------------- Preview ----------------
    def _on_name_changed(self, text: str):
        if isinstance(self._current_recipe, dict):
            self._current_recipe["id"] = (text or "").strip()

    def _on_type_changed(self, idx: int):
        self.stackTypes.setCurrentIndex(idx)
        update_visibility_all(self._param_widgets, self._visibility_rules)
        self._auto_preview()

    def _auto_preview(self):
        self._build_preview_from_ui()

    def _build_preview_from_ui(self):
        """Baut echte Trajektorien (pro ausgewählter Seite) und zeichnet sie."""
        show_normals = bool(self.checkNormals.isChecked()) if hasattr(self, "checkNormals") else False
        show_rays    = bool(self.checkRaycasts.isChecked()) if hasattr(self, "checkRaycasts") else False

        sides = self._selected_sides()  # niemals leer (mind. 'front')
        first = True

        try:
            model = self._recipe_model_from_forms()
        except Exception as e:
            _LOG.exception("RecipeModel aus Formular fehlgeschlagen: %s", e)
            return

        # Substrat-Mesh (mm) versuchen zu laden; wenn None -> ohne Projektion
        mesh_mm = self._load_preview_mesh_mm(model.substrate)

        for side in sides:
            try:
                traj = self.traj_builder.build(
                    model,
                    mesh_mm=mesh_mm,
                    side=side,              # Ray-Richtung für Projektion
                    T_world_scene=None,     # Preview im Scene-Frame
                    force_project=False     # bei planaren Typen projiziert er ohnehin
                )
                self.preview.render_trajectory(
                    traj,
                    show_normals=show_normals,
                    show_rays=show_rays,
                    append=(not first)
                )
                first = False
            except Exception as e:
                _LOG.exception("Preview/Build für Seite '%s' fehlgeschlagen: %s", side, e)
                # weiter zur nächsten Seite

    # ---------------- Mesh-Lader (Preview) ----------------
    def _load_preview_mesh_mm(self, substrate_key: Optional[str]):
        """
        Versucht ein PyVista-PolyData (mm) für das gegebene Substrat zu holen.
        Falls nicht verfügbar, None zurückgeben -> Traj ohne Projektion.
        Erwartete Integrationspunkte (projektabhängig):
          - ctx.get_substrate_mesh_mm(key)
          - ctx.mesh_cache[key]
          - ctx.load_substrate_mesh_mm(key)
        """
        if not substrate_key:
            return None
        # 1) Direkter Getter am Kontext?
        getter = getattr(self.ctx, "get_substrate_mesh_mm", None)
        if callable(getter):
            try:
                return getter(substrate_key)
            except Exception as e:
                _LOG.debug("get_substrate_mesh_mm('%s') schlug fehl: %s", substrate_key, e)
        # 2) Cache-Attr?
        cache = getattr(self.ctx, "mesh_cache", None)
        if isinstance(cache, dict) and substrate_key in cache:
            return cache.get(substrate_key)
        # 3) Loader-Funktion?
        loader = getattr(self.ctx, "load_substrate_mesh_mm", None)
        if callable(loader):
            try:
                return loader(substrate_key)
            except Exception as e:
                _LOG.debug("load_substrate_mesh_mm('%s') schlug fehl: %s", substrate_key, e)
        # Kein Mesh verfügbar -> ohne Projektion rendern
        return None
