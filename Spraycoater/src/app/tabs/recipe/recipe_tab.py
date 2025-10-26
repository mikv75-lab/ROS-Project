# -*- coding: utf-8 -*-
import os
import numpy as np
import pyvista as pv

from PyQt5 import uic, QtCore
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QListWidgetItem, QDoubleSpinBox,
    QSpinBox, QCheckBox, QComboBox, QLabel, QHBoxLayout, QWidget as QW
)
from pyvistaqt import QtInteractor


def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))  # .../src/app/tabs/recipe
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", filename)


class RecipeTab(QWidget):
    def __init__(self, *, ctx, bridge, parent=None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        if self.ctx is None:
            raise RuntimeError("RecipeTab benötigt ctx=AppContext")

        uic.loadUi(_ui_path("recipe_tab.ui"), self)

        # Plotter-Host einsetzen
        if self.plotContainer.layout() is None:
            self.plotContainer.setLayout(QVBoxLayout(self.plotContainer))
        self.plotter = QtInteractor(self.plotContainer)
        self.plotContainer.layout().addWidget(self.plotter)

        # Daten
        self._recipes = self.ctx.recipes              # list[dict]
        self._specs   = self.ctx.recipe_params        # dict (globals + path.*.*)
        self._param_widgets = {}                      # key -> widget/tuple
        self._visibility_rules = {}                   # full_key -> rule

        # UI wiring
        self._fill_recipe_list()
        self.listRecipes.currentRowChanged.connect(self._on_recipe_selected)

        self.btnGenerate.clicked.connect(self.build_plot)
        self.btnResetCam.clicked.connect(self._reset_camera)
        self.comboShape.currentIndexChanged.connect(self._auto_preview)
        self.comboPath.currentIndexChanged.connect(self._auto_preview)
        self.checkNormals.toggled.connect(self._auto_preview)

        # Rezepttyp-Wechsel -> StackedPage
        self.comboRecipeType.currentIndexChanged.connect(self._on_type_changed)

        if self.listRecipes.count() > 0:
            self.listRecipes.setCurrentRow(0)
        else:
            self.build_plot()

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
        item = self.listRecipes.item(row)
        recipe = item.data(QtCore.Qt.UserRole)

        # Typ/Mode ermitteln
        path = recipe.get("path", {})
        rtype = path.get("type", "meander")
        rmode = path.get("mode", "plane")
        type_label = self._type_label_for(rtype, rmode)

        # UI setzen
        idx = self.comboRecipeType.findText(type_label)
        if idx < 0:
            idx = 0
        self.comboRecipeType.setCurrentIndex(idx)
        self.stackTypes.setCurrentIndex(self._page_index_for(rtype, rmode))

        # Forms (Globals + aktive Page) neu bauen
        self._rebuild_all_forms(active_index=self.stackTypes.currentIndex())
        # Werte übernehmen
        self._apply_recipe_to_forms(recipe)
        # Visualizer preset
        self.comboPath.setCurrentText("Raster" if rtype == "meander" else "Spiral")
        self.comboShape.setCurrentText("Zylinder" if rmode == "cylinder" else "Würfel")
        self.build_plot()

    def _on_type_changed(self, idx: int):
        # Nur die Page sichtbar schalten; Formular bleibt dynamisch
        self.stackTypes.setCurrentIndex(idx)
        # Sichtbarkeiten (visible_if kann sich ändern, z.B. andere Dep-Keys)
        self._update_visibility_all()

    # ---------------- dynamic forms ----------------
    def _clear_form(self, form):
        while form.rowCount():
            form.removeRow(0)

    def _rebuild_all_forms(self, active_index: int):
        # Clear old
        self._param_widgets.clear()
        self._visibility_rules.clear()

        # Globals
        self._clear_form(self.formGlobals)
        gspec = self._specs.get("globals", {})
        self._build_form_section(self.formGlobals, gspec, prefix="globals")

        # Pages
        self._clear_form(self.formMeanderPlane)
        self._clear_form(self.formSpiralPlane)
        self._clear_form(self.formSpiralCylinder)
        self._clear_form(self.formExplicit)

        mapping = [
            (self.formMeanderPlane,   "path.meander.plane"),
            (self.formSpiralPlane,    "path.spiral.plane"),
            (self.formSpiralCylinder, "path.spiral.cylinder"),
            (self.formExplicit,       "path.explicit"),
        ]
        for form, skey in mapping:
            spec = self._specs.get(skey, {})
            self._build_form_section(form, spec, prefix="path")

        self.stackTypes.setCurrentIndex(active_index)
        self._update_visibility_all()

    def _build_form_section(self, form, spec: dict, prefix: str):
        for key, sch in spec.items():
            full_key = f"{prefix}.{key}"
            w, _ = self._make_widget_for_spec(full_key, sch)
            label = QLabel(key.split(".")[-1].replace("_", " "))
            form.addRow(label, w)

            vis = sch.get("visible_if")
            if isinstance(vis, dict) and len(vis) == 1:
                dep_key, dep_val = next(iter(vis.items()))
                dep_full = f"{prefix}.{dep_key}"
                self._visibility_rules[full_key] = {"on": (dep_full, dep_val)}
                depw = self._param_widgets.get(dep_full)
                self._connect_change_for_visibility(depw)

    def _make_widget_for_spec(self, full_key: str, sch: dict):
        t = sch.get("type")
        unit = sch.get("unit", "")
        if t == "number":
            sb = QDoubleSpinBox()
            sb.setDecimals(3)
            sb.setMinimum(float(sch.get("min", -1e9)))
            sb.setMaximum(float(sch.get("max", 1e9)))
            sb.setSingleStep(float(sch.get("step", 0.1)))
            if "default" in sch:
                sb.setValue(float(sch["default"]))
            if unit:
                sb.setSuffix(f" {unit}")
            self._param_widgets[full_key] = sb
            return sb, sb

        if t == "boolean":
            cb = QCheckBox()
            cb.setChecked(bool(sch.get("default", False)))
            self._param_widgets[full_key] = cb
            return cb, cb

        if t == "enum":
            cmb = QComboBox()
            for v in sch.get("values", []):
                cmb.addItem(str(v))
            if "default" in sch:
                idx = cmb.findText(str(sch["default"]))
                if idx >= 0:
                    cmb.setCurrentIndex(idx)
            self._param_widgets[full_key] = cmb
            return cmb, cmb

        if t == "vec2":
            w = QW()
            lay = QHBoxLayout(w); lay.setContentsMargins(0,0,0,0)
            mins = sch.get("min", [-1e9, -1e9])
            maxs = sch.get("max", [1e9, 1e9])
            steps = sch.get("step", [0.1, 0.1])
            defaults = sch.get("default", [0.0, 0.0])
            s1, s2 = QDoubleSpinBox(), QDoubleSpinBox()
            for sb, mn, mx, st, dv in ((s1, mins[0], maxs[0], steps[0], defaults[0]),
                                        (s2, mins[1], maxs[1], steps[1], defaults[1])):
                sb.setDecimals(3); sb.setMinimum(float(mn)); sb.setMaximum(float(mx)); sb.setSingleStep(float(st)); sb.setValue(float(dv))
                if unit: sb.setSuffix(f" {unit}")
                lay.addWidget(sb)
            self._param_widgets[full_key] = (s1, s2)
            return w, w

        lab = QLabel("(unsupported)")
        self._param_widgets[full_key] = lab
        return lab, lab

    def _connect_change_for_visibility(self, dep_widget):
        if isinstance(dep_widget, (QDoubleSpinBox, QSpinBox)):
            dep_widget.valueChanged.connect(self._update_visibility_all)
        elif isinstance(dep_widget, QCheckBox):
            dep_widget.toggled.connect(self._update_visibility_all)
        elif isinstance(dep_widget, QComboBox):
            dep_widget.currentTextChanged.connect(self._update_visibility_all)
        elif isinstance(dep_widget, tuple):
            dep_widget[0].valueChanged.connect(self._update_visibility_all)
            dep_widget[1].valueChanged.connect(self._update_visibility_all)

    def _update_visibility_all(self):
        for full_key, rule in self._visibility_rules.items():
            dep_key, expected = rule["on"]
            depw = self._param_widgets.get(dep_key)
            cur = None
            if isinstance(depw, (QDoubleSpinBox, QSpinBox)):
                cur = depw.value()
            elif isinstance(depw, QCheckBox):
                cur = depw.isChecked()
            elif isinstance(depw, QComboBox):
                cur = depw.currentText()
            elif isinstance(depw, tuple):
                cur = (depw[0].value(), depw[1].value())

            row_widget = self._param_widgets.get(full_key)
            w = None
            if isinstance(row_widget, tuple):
                w = row_widget[0].parentWidget()
            elif hasattr(row_widget, "parentWidget"):
                w = row_widget
            if w is not None:
                w.setVisible(cur == expected)

    # --------------- apply values ----------------
    def _apply_recipe_to_forms(self, recipe: dict):
        # Globals: defaults aus spraycoater.defaults + recipe.parameters override
        gvals = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        gvals.update(recipe.get("parameters", {}))
        self._apply_values("globals", gvals)

        # Path values
        self._apply_values("path", recipe.get("path", {}))
        self._update_visibility_all()

    def _apply_values(self, prefix: str, values: dict):
        for key, widget in self._param_widgets.items():
            if not key.startswith(prefix + "."):
                continue
            short = key[len(prefix) + 1:]
            val = self._read_nested(values, short.split("."))
            if val is None:
                continue
            if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
                widget.setValue(float(val))
            elif isinstance(widget, QCheckBox):
                widget.setChecked(bool(val))
            elif isinstance(widget, QComboBox):
                idx = widget.findText(str(val))
                if idx >= 0:
                    widget.setCurrentIndex(idx)
            elif isinstance(widget, tuple):
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    widget[0].setValue(float(val[0]))
                    widget[1].setValue(float(val[1]))

    @staticmethod
    def _read_nested(d: dict, path):
        cur = d
        for p in path:
            if not isinstance(cur, dict) or p not in cur:
                return None
            cur = cur[p]
        return cur

    # ---------------- Visualisierung ----------------
    def _reset_camera(self):
        self.plotter.view_isometric()
        self.plotter.reset_camera()

    def _auto_preview(self):
        self.build_plot()

    def build_plot(self):
        shape = self.comboShape.currentText()
        path_type = self.comboPath.currentText()
        show_normals = self.checkNormals.isChecked()

        self.plotter.clear()

        if shape == "Würfel":
            mesh = pv.Cube(center=(0, 0, 0), x_length=2, y_length=2, z_length=2)
            pts = self._raster_on_cube() if path_type == "Raster" else self._spiral_on_cube()
        elif shape == "Kugel":
            mesh = pv.Sphere(radius=1.0, center=(0, 0, 0), theta_resolution=60, phi_resolution=60)
            pts = self._raster_on_sphere() if path_type == "Raster" else self._spiral_on_sphere()
        elif shape == "Zylinder":
            mesh = pv.Cylinder(center=(0, 0, 0), direction=(0, 0, 1), radius=1.0, height=2.0, resolution=100)
            pts = self._raster_on_cylinder() if path_type == "Raster" else self._spiral_on_cylinder()
        else:
            return

        self.plotter.add_mesh(mesh, color="lightgray", opacity=0.3)
        self.plotter.add_points(pts, color="red", point_size=8, render_points_as_spheres=True)

        if show_normals:
            normals = self._compute_normals(shape, pts)
            arrows = pv.PolyData(pts)
            arrows["vectors"] = normals
            glyphs = arrows.glyph(orient="vectors", scale=False, factor=0.15)
            self.plotter.add_mesh(glyphs, color="green")

        self._reset_camera()

    # ---- Dummy generatoren (Preview) ----
    def _compute_normals(self, shape: str, points: np.ndarray) -> np.ndarray:
        normals = []
        for p in points:
            if shape == "Kugel":
                v = p / (np.linalg.norm(p) + 1e-12)
            elif shape == "Würfel":
                v = np.sign(p); n = np.linalg.norm(v); v = v / n if n > 0 else np.array([0.0,0.0,1.0])
            elif shape == "Zylinder":
                v2 = np.array([p[0], p[1], 0.0]); n = np.linalg.norm(v2); v = v2 / n if n > 0 else np.array([0.0,0.0,1.0])
            else:
                v = np.array([0.0, 0.0, 1.0])
            normals.append(v)
        return np.array(normals)

    def _raster_on_cube(self) -> np.ndarray:
        x = np.linspace(-1, 1, 20); y = np.linspace(-1, 1, 20); z = 1.0
        pts = []
        for i, yy in enumerate(y):
            row = [np.array([xx, yy, z]) for xx in (x if i % 2 == 0 else x[::-1])]
            pts.extend(row)
        return np.array(pts)

    def _spiral_on_cube(self) -> np.ndarray:
        th = np.linspace(0, 4*np.pi, 200); r = np.linspace(0.2, 1.0, 200)
        x = r*np.cos(th); y = r*np.sin(th); z = np.ones_like(x)
        return np.vstack((x,y,z)).T

    def _raster_on_sphere(self) -> np.ndarray:
        ph = np.linspace(0.0, np.pi, 30)
        th = np.linspace(0.0, 2*np.pi, 60)
        pts = []
        for i, p in enumerate(ph):
            ring = [np.array([np.sin(p)*np.cos(t), np.sin(p)*np.sin(t), np.cos(p)])
                    for t in (th if i % 2 == 0 else th[::-1])]
            # ^ reverse every second ring for boustrophedon effect
            pts.extend(ring)
        return np.array(pts)

    def _spiral_on_sphere(self) -> np.ndarray:
        t = np.linspace(0, 6*np.pi, 400); z = np.linspace(-1, 1, 400)
        r = np.sqrt(np.clip(1 - z*z, 0.0, 1.0))
        x = r*np.cos(t); y = r*np.sin(t)
        return np.vstack((x,y,z)).T

    def _raster_on_cylinder(self) -> np.ndarray:
        z = np.linspace(-1, 1, 120); ang = np.linspace(0, 2*np.pi, 160)
        pts = []
        for i, zz in enumerate(z):
            circle = [np.array([np.cos(a), np.sin(a), zz])
                      for a in (ang if i%2==0 else ang[::-1])]
            pts.extend(circle)
        return np.array(pts)

    def _spiral_on_cylinder(self) -> np.ndarray:
        th = np.linspace(0, 12*np.pi, 600); z = np.linspace(-1, 1, 600)
        x = np.cos(th); y = np.sin(th)
        return np.vstack((x,y,z)).T
