# -*- coding: utf-8 -*-
import os
from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import QWidget, QListWidgetItem, QTabWidget

from .form_builder import (
    clear_form, build_form_section, apply_values, update_visibility_all
)
from .preview import ShapePreview


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/recipe
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", filename)


class RecipeTab(QWidget):
    """
    Dünne View-Klasse:
    - lädt UI
    - befüllt Recipe-Liste
    - schaltet StackedPages
    - delegiert Formlogik an form_builder
    - delegiert 3D-Preview an ShapePreview
    """
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        if self.ctx is None:
            raise RuntimeError("RecipeTab benötigt ctx=AppContext")

        uic.loadUi(_ui_path("recipe_tab.ui"), self)

        # 3D-Preview (PyVista) in Container initialisieren
        self.preview = ShapePreview(self.plotContainer)

        # Daten
        self._recipes = self.ctx.recipes
        self._specs   = self.ctx.recipe_params

        # Formular-Registry (Key -> Widget/Tupel) + visible_if-Regeln
        self._param_widgets = {}
        self._visibility_rules = {}

        # UI wiring
        self._fill_recipe_list()
        self.listRecipes.currentRowChanged.connect(self._on_recipe_selected)

        self.btnGenerate.clicked.connect(self._build_plot)
        self.btnResetCam.clicked.connect(self.preview.reset_camera)
        self.comboShape.currentIndexChanged.connect(self._build_plot)
        self.comboPath.currentIndexChanged.connect(self._build_plot)
        self.checkNormals.toggled.connect(self._build_plot)

        # Rezepttyp-Wechsel -> StackedPage
        self.comboRecipeType.currentIndexChanged.connect(self._on_type_changed)

        if self.listRecipes.count() > 0:
            self.listRecipes.setCurrentRow(0)
        else:
            self._rebuild_all_forms(active_index=0)
            self._build_plot()

    # ---------------- Recipe list ----------------
    def _fill_recipe_list(self):
        self.listRecipes.clear()
        for rec in self._recipes:
            item = QListWidgetItem(f"{rec.get('id','?')} — {rec.get('description','')}")
            item.setData(QtCore.Qt.UserRole, rec)
            self.listRecipes.addItem(item)

    # -------------- Type <-> Page mapping --------------
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

    # ---------------- selection -> forms ----------------
    def _on_recipe_selected(self, row: int):
        if row < 0:
            return
        recipe = self.listRecipes.item(row).data(QtCore.Qt.UserRole)

        # Typ/Mode ermitteln
        path = recipe.get("path", {})
        rtype = path.get("type", "meander")
        rmode = path.get("mode", "plane")

        # UI setzen
        type_label = self._type_label_for(rtype, rmode)
        idx = self.comboRecipeType.findText(type_label)
        self.comboRecipeType.setCurrentIndex(0 if idx < 0 else idx)
        self.stackTypes.setCurrentIndex(self._page_index_for(rtype, rmode))

        # Forms (Globals + Pages) neu bauen & Werte übernehmen
        self._rebuild_all_forms(active_index=self.stackTypes.currentIndex())
        self._apply_recipe_to_forms(recipe)

        # Visualizer preset
        self.comboPath.setCurrentText("Raster" if rtype == "meander" else "Spiral")
        self.comboShape.setCurrentText("Zylinder" if rmode == "cylinder" else "Würfel")
        self._build_plot()

    def _on_type_changed(self, idx: int):
        self.stackTypes.setCurrentIndex(idx)
        update_visibility_all(self._param_widgets, self._visibility_rules)

    # ---------------- dynamic forms ----------------
    def _rebuild_all_forms(self, active_index: int):
        # Clear registries
        self._param_widgets.clear()
        self._visibility_rules.clear()

        # Globals
        clear_form(self.formGlobals)
        gspec = self._specs.get("globals", {})
        build_form_section(self.formGlobals, gspec, prefix="globals",
                           param_widgets=self._param_widgets,
                           visibility_rules=self._visibility_rules)

        # Pages
        clear_form(self.formMeanderPlane)
        clear_form(self.formSpiralPlane)
        clear_form(self.formSpiralCylinder)
        clear_form(self.formExplicit)

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

    def _apply_recipe_to_forms(self, recipe: dict):
        # Globals: defaults aus spraycoater.defaults + recipe.parameters override
        gvals = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        gvals.update(recipe.get("parameters", {}))
        apply_values("globals", gvals, self._param_widgets)

        # Path values
        apply_values("path", recipe.get("path", {}), self._param_widgets)
        update_visibility_all(self._param_widgets, self._visibility_rules)

    # ---------------- Visualisierung ----------------
    def _build_plot(self):
        self.preview.build_plot(
            shape=self.comboShape.currentText(),
            path_type=self.comboPath.currentText(),
            show_normals=self.checkNormals.isChecked()
        )
