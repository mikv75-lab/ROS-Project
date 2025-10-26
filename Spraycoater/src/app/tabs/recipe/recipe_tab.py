# -*- coding: utf-8 -*-
from __future__ import annotations
import os
import copy
import glob
import logging
from typing import Any, Dict, List, Optional

import pyvista as pv
from PyQt5 import uic, QtGui
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
from .path_builder import PathBuilder
from .trajectory_builder import TrajectoryBuilder

_LOG = logging.getLogger("app.tabs.recipe")


# ---------------- Pfad-Helfer ----------------
def _project_root() -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "..", ".."))

def _ui_path(filename: str) -> str:
    return os.path.join(_project_root(), "resource", "ui", filename)

def _ws_moveit_resource_root() -> str:
    # Dein Pfadbaum: ~/ws_moveit/src/mecademic_bringup/resource
    return os.path.expanduser("~/ws_moveit/src/mecademic_bringup/resource")


class RecipeTab(QWidget):
    """
    Rezept-Editor:
      - Formular mit form_builder
      - Preview rendert Mount/Substrat (STL) + Trajektorie (append)
      - Kamera-Buttons: ISO/Top/Front/Back/Left/Right
      - Normals/Raycasts lassen sich live toggeln
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

        # Trajektorien-Builder
        self.traj_builder = TrajectoryBuilder()

        # Specs / Formularsteuerung
        self._specs: Dict[str, Any] = dict(self.ctx.recipe_params)
        self._param_widgets: Dict[str, Any] = {}
        self._visibility_rules: Dict[str, Any] = {}

        # aktuelles Rezept (dict fürs UI)
        self._current_recipe: Dict[str, Any] = self._default_recipe()

        # Kamera-Buttons
        if hasattr(self, "btnViewIso"):   self.btnViewIso.clicked.connect(self.preview.view_iso)
        if hasattr(self, "btnViewTop"):   self.btnViewTop.clicked.connect(self.preview.view_top)
        if hasattr(self, "btnViewFront"): self.btnViewFront.clicked.connect(self.preview.view_front)
        if hasattr(self, "btnViewBack"):  self.btnViewBack.clicked.connect(self.preview.view_back)
        if hasattr(self, "btnViewLeft"):  self.btnViewLeft.clicked.connect(self.preview.view_left)
        if hasattr(self, "btnViewRight"): self.btnViewRight.clicked.connect(self.preview.view_right)

        # Live-Toggles
        if hasattr(self, "checkNormals"): self.checkNormals.toggled.connect(self._on_toggle_normals)
        if hasattr(self, "checkRaycasts"): self.checkRaycasts.toggled.connect(self._on_toggle_rays)

        # Optionaler „Preview aktualisieren“-Button, falls vorhanden
        if hasattr(self, "btnPreview"): self.btnPreview.clicked.connect(self._on_update_preview)

        if hasattr(self, "editRecipeName"):
            self.editRecipeName.textChanged.connect(self._on_name_changed)

        # Typwechsel -> kein Auto-Rebuild
        if hasattr(self, "comboRecipeType"):
            self.comboRecipeType.currentIndexChanged.connect(self._on_type_changed)

        # Datei-Aktionen
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

        # einmaliger initialer Preview-Build
        self._build_preview_from_ui()

    # ---------------- Defaults ----------------
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

        # Path-Seiten leeren
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
        keys = [
            "path.meander.plane",
            "path.spiral.plane",
            "path.spiral.cylinder",
            "path.explicit",
        ]
        if idx < 0 or idx >= len(keys):
            return keys[0]
        return keys[idx]

    def _apply_side_to_ui(self, side: Optional[str]):
        side = (side or "top").strip().lower()
        self._set_side_checks(top=True, front=False, back=False, left=False, right=False, helix=False)
        if side == "front":
            self._set_side_checks(top=False, front=True, back=False, left=False, right=False, helix=False)
        elif side == "back":
            self._set_side_checks(top=False, front=False, back=True, left=False, right=False, helix=False)
        elif side == "left":
            self._set_side_checks(top=False, front=False, back=False, left=True, right=False, helix=False)
        elif side == "right":
            self._set_side_checks(top=False, front=False, back=False, left=False, right=True, helix=False)
        elif side == "helix":
            self._set_side_checks(top=False, front=False, back=False, left=False, right=False, helix=True)

    def _set_side_checks(self, *, top: bool, front: bool, back: bool, left: bool, right: bool, helix: bool):
        mapping = {
            "checkSideTop": top, "checkSideFront": front, "checkSideBack": back,
            "checkSideLeft": left, "checkSideRight": right, "checkSideHelix": helix
        }
        for name, val in mapping.items():
            w = getattr(self, name, None)
            if w is not None:
                prev = w.blockSignals(True)
                w.setChecked(val)
                w.blockSignals(prev)

    def _selected_sides(self) -> List[str]:
        sides = []
        if getattr(self, "checkSideTop", None)   and self.checkSideTop.isChecked():   sides.append("top")
        if getattr(self, "checkSideFront", None) and self.checkSideFront.isChecked(): sides.append("front")
        if getattr(self, "checkSideBack", None)  and self.checkSideBack.isChecked():  sides.append("back")
        if getattr(self, "checkSideLeft", None)  and self.checkSideLeft.isChecked():  sides.append("left")
        if getattr(self, "checkSideRight", None) and self.checkSideRight.isChecked(): sides.append("right")
        if getattr(self, "checkSideHelix", None) and self.checkSideHelix.isChecked(): sides.append("helix")
        return sides or ["top"]

    def _single_saved_side(self) -> str:
        sel = self._selected_sides()
        return sel[0] if len(sel) >= 1 else "top"

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
            "path":             self._collect_path_for(self._spec_key_for_page_index(self.stackTypes.currentIndex())),
        }
        return rec

    def _recipe_model_from_forms(self) -> Recipe:
        d = self._build_recipe_from_forms()
        return Recipe.from_dict(d)

    # ---------------- Aktionen ----------------
    def _on_update_preview(self):
        self._build_preview_from_ui()

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
            model = Recipe.from_dict(rec)
            with open(fname, "w", encoding="utf-8") as f:
                import yaml
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
            model = Recipe.load_yaml(fname)
            rec = model.to_dict()

            self._current_recipe = rec

            rtype = rec.get("path", {}).get("type", "meander")
            rmode = rec.get("path", {}).get("mode", "plane")
            self._rebuild_all_forms(active_index=self._page_index_for(rtype, rmode))
            self._apply_recipe_to_forms(rec)

            if hasattr(self, "editRecipeName"):
                self.editRecipeName.setText(str(rec.get("id") or ""))

            self._apply_side_to_ui(rec.get("side"))

            self._build_preview_from_ui()

            QMessageBox.information(self, "Geladen", f"Rezept geladen:\n{fname}")
        except Exception as e:
            QMessageBox.critical(self, "Ladefehler", str(e))

    def _on_validate_clicked(self):
        model = self._recipe_model_from_forms()
        errs = model.validate_required()

        tools_yaml       = getattr(self.ctx, "tools_yaml", None)
        mounts_yaml      = getattr(self.ctx, "mounts_yaml", None)
        substrates_yaml  = getattr(self.ctx, "substrates_yaml", None)
        errs += model.validate_references(
            tools_yaml=tools_yaml, mounts_yaml=mounts_yaml, substrates_yaml=substrates_yaml
        )

        if errs:
            QMessageBox.warning(self, "Validate", "Fehler:\n- " + "\n- ".join(map(str, errs)))
            return

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
        # kein Auto-Preview – Nutzer drückt „Preview aktualisieren“ (oder initialer Build ist bereits erfolgt)

    def _on_toggle_normals(self, checked: bool):
        self.preview.set_normals_visible(bool(checked))

    def _on_toggle_rays(self, checked: bool):
        self.preview.set_rays_visible(bool(checked))

    def _build_preview_from_ui(self):
        """
        Baut Szene (Mount/Substrate) und Trajektorie und zeichnet sie.
        """
        show_normals = bool(self.checkNormals.isChecked()) if hasattr(self, "checkNormals") else False
        show_rays    = bool(self.checkRaycasts.isChecked()) if hasattr(self, "checkRaycasts") else False

        try:
            model = self._recipe_model_from_forms()
        except Exception as e:
            _LOG.exception("RecipeModel aus Formular fehlgeschlagen: %s", e)
            self._set_valid_label(text="Fehler beim Lesen des Formulars", ok=False)
            return

        errs = model.validate_required()
        if errs:
            self._set_valid_label(text="; ".join(errs), ok=False)
        else:
            self._set_valid_label(text="OK", ok=True)

        # Szene zurücksetzen und Mount/Substrate einfügen
        self.preview.clear()

        # Mount-Mesh (grau), Substrate-Mesh (leicht blau)
        try:
            self._try_add_scene_mesh(model.mount or model.to_dict().get("mount"), kind="mount",
                                     color="lightgray", opacity=0.25)
        except Exception as e:
            _LOG.warning("Mount-Mesh nicht geladen: %s", e)
        try:
            self._try_add_scene_mesh(model.substrate, kind="substrate",
                                     color="#4c6ef5", opacity=0.30)
        except Exception as e:
            _LOG.warning("Substrate-Mesh nicht geladen: %s", e)

        # Pfaddaten + Trajektorie
        try:
            path_data = PathBuilder.from_recipe(model)
        except Exception as e:
            _LOG.exception("PathBuilder fehlgeschlagen: %s", e)
            return

        sides = self._selected_sides()
        for _s in sides:
            try:
                traj = self.traj_builder.build_from_pathdata(model, path_data)
                self.preview.render_trajectory(
                    traj, show_normals=show_normals, show_rays=show_rays, append=True
                )
            except Exception as e:
                _LOG.exception("Preview/Build fehlgeschlagen: %s", e)

        # Standardkamera
        self.preview.view_iso()

    # ----- Scene mesh helpers -----
    def _try_add_scene_mesh(self, name_or_path: Optional[str], *, kind: str, color: str, opacity: float):
        if not name_or_path:
            return
        p = self._find_mesh_file(str(name_or_path), kind=kind)
        if not p:
            # second chance: Basisname ohne Endung
            base = os.path.splitext(str(name_or_path))[0]
            p = self._find_mesh_file(base, kind=kind)
        if not p:
            raise FileNotFoundError(f"Mesh für '{name_or_path}' (kind={kind}) nicht gefunden.")
        mesh = pv.read(p)
        self.preview.add_mesh(mesh, color=color, opacity=float(opacity))

    # ---- robustes Suchen in ws_moveit + Projekt ----
    def _find_mesh_file(self, name_or_path: str, *, kind: Optional[str] = None) -> Optional[str]:
        """
        Sucht STL/OBJ/VTP/VTPK für Mount/Substrate/Tool:
        - exakter Pfad
        - targeted roots (ws_moveit resource, nach 'kind' priorisiert)
        - Projekt-Roots
        - rekursiver Glob
        - Mount-Aliase werden auf reale Dateinamen abgebildet
        """
        if not name_or_path:
            return None

        # 0) Vollpfad?
        if os.path.isabs(name_or_path) and os.path.exists(name_or_path):
            return name_or_path

        # 1) ggf. Aliase für Mounts → mögliche Basen
        candidates: List[str] = []
        base = os.path.splitext(os.path.basename(name_or_path))[0]

        if (kind or "").lower() == "mount":
            candidates.extend(self._mount_alias_basenames(base))
        else:
            candidates.append(base)

        # 2) Endungen
        exts = (".stl", ".ply", ".obj", ".vtp", ".vtk")

        # 3) Root-Verzeichnisse (Priorität: ws_moveit → Projekt)
        r_ws = _ws_moveit_resource_root()
        roots_ws_mounts = [os.path.join(r_ws, "substrate_mounts")]
        roots_ws_subs   = [os.path.join(r_ws, "substrates")]
        roots_ws_tools  = [os.path.join(r_ws, "tools")]
        roots_ws_env    = [os.path.join(r_ws, "environment")]

        proj = _project_root()
        roots_proj = [
            os.path.join(proj, "resource", "stl"),
            os.path.join(proj, "resource", "meshes"),
            os.path.join(proj, "resource", "models"),
            os.path.join(proj, "data", "stl"),
            os.path.join(proj, "data", "meshes"),
            proj,
        ]

        # nach 'kind' ordnen
        roots: List[str] = []
        if (kind or "").lower() == "mount":
            roots += roots_ws_mounts
        elif (kind or "").lower() == "substrate":
            roots += roots_ws_subs
        elif (kind or "").lower() == "tool":
            roots += roots_ws_tools
        elif (kind or "").lower() == "environment":
            roots += roots_ws_env

        # Generische ws_moveit-Roots (falls z. B. falsches 'kind')
        roots += roots_ws_mounts + roots_ws_subs + roots_ws_tools + roots_ws_env
        # Projekt-Roots
        roots += roots_proj
        # aktuelle Working-Dir als allerletzter Versuch
        roots += [os.getcwd()]

        # 4) direkte Kandidaten
        for r in roots:
            for c in candidates:
                for ext in exts:
                    p = os.path.join(r, c + ext)
                    if os.path.exists(p):
                        return p

        # 5) rekursiv suchen
        for r in roots:
            for c in candidates:
                for ext in exts:
                    pat = os.path.join(r, "**", c + ext)
                    hits = glob.glob(pat, recursive=True)
                    if hits:
                        return hits[0]

        return None

    @staticmethod
    def _mount_alias_basenames(alias: str) -> List[str]:
        """
        Abbildung häufiger Mount-Kurznamen auf die tatsächlichen DateibasEN.
        Wir geben mehrere zurück (versuchsweise), um robust zu sein.
        """
        a = alias.lower().replace(" ", "").replace("-", "_")

        # Handgemappte Aliase → exakte Basen (ohne Endung)
        map_exact = {
            "flat_240x240":      "substrate_mount_flat_240x240x50",
            "flat240x240":       "substrate_mount_flat_240x240x50",
            "circle_d240":       "substrate_mount_circle_d240_h50",
            "circle240":         "substrate_mount_circle_d240_h50",
            "circle_d240_h50":   "substrate_mount_circle_d240_h50",
            "post_d30_h100":     "substrate_mount_post_d30mm_h100mm",
            "postd30h100":       "substrate_mount_post_d30mm_h100mm",
            "post_d30":          "substrate_mount_post_d30mm_h100mm",
        }

        # Wenn alias bereits exakt gemappt werden kann
        if a in map_exact:
            return [map_exact[a], a, "substrate_mount_" + a]

        # Heuristische Varianten
        cand = [a, "substrate_mount_" + a]

        # Zahlen-Extraktion für typische Muster
        if a.startswith("flat") and "240x240" in a:
            cand.append("substrate_mount_flat_240x240x50")
        if a.startswith("circle") and ("240" in a or "d240" in a):
            cand.append("substrate_mount_circle_d240_h50")
        if "post" in a and ("d30" in a or "30" in a) and ("h100" in a or "100" in a):
            cand.append("substrate_mount_post_d30mm_h100mm")

        # Duplikate entfernen, Reihenfolge erhalten
        seen = set()
        out: List[str] = []
        for c in cand:
            if c not in seen:
                out.append(c); seen.add(c)
        return out

    def _set_valid_label(self, *, text: str, ok: bool):
        if not hasattr(self, "lblValid"):
            return
        self.lblValid.setText("OK" if ok else f"Fehler: {text}")
        self.lblValid.setStyleSheet("color: #0a0;" if ok else "color: #a00;")
