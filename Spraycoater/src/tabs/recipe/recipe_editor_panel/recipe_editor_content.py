# -*- coding: utf-8 -*-
# File: src/tabs/recipe/recipe_editor_panel/recipe_editor_content.py
from __future__ import annotations

from typing import Optional, Dict, Any, Tuple, List

from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QFormLayout,
    QComboBox,
    QLabel,
    QFrame,
    QLineEdit,
    QSizePolicy,
    QTabBar,
    QCheckBox,
    QDoubleSpinBox,
    QSpinBox,
)

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from widgets.planner_groupbox import PlannerGroupBox
from .side_path_editor import SidePathEditor

Bounds = Tuple[float, float, float, float, float, float]


def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setFrameShadow(QFrame.Shadow.Sunken)
    return f


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


def _compact_form(f: QFormLayout) -> None:
    f.setContentsMargins(8, 8, 8, 8)
    f.setHorizontalSpacing(8)
    f.setVerticalSpacing(6)
    f.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    f.setRowWrapPolicy(QFormLayout.RowWrapPolicy.DontWrapRows)


def _intish(*vals: Any) -> bool:
    try:
        return all(float(v).is_integer() for v in vals)
    except Exception:
        return False


class GlobalParamsBox(QGroupBox):
    """
    Global recipe parameters:
      - model.parameters ist ein FLACHES dict (entspricht recipe_params.globals)
    """

    def __init__(self, *, store: RecipeStore, parent: Optional[QWidget] = None) -> None:
        super().__init__("Parameters", parent)
        self.store = store

        self._form = QFormLayout(self)
        _compact_form(self._form)

        self._fields: Dict[str, tuple[QWidget, str, Dict[str, Any]]] = {}
        self._build()

    def _build(self) -> None:
        schema = self.store.globals_schema()
        if not isinstance(schema, dict) or not schema:
            raise KeyError("RecipeStore.globals_schema() ist leer/ungültig.")

        keys = sorted([str(k) for k in schema.keys()])

        for key in keys:
            spec = schema.get(key)
            if not isinstance(spec, dict):
                raise TypeError(f"globals_schema['{key}'] ist kein dict (got {type(spec)}).")

            t = str(spec.get("type", "")).strip().lower()
            w: Optional[QWidget] = None
            kind: str = ""

            if t == "boolean":
                w = QCheckBox(self)
                kind = "check"

            elif t == "string":
                w = QLineEdit(self)
                kind = "string"

            elif t == "enum":
                cbx = QComboBox(self)
                vals = spec.get("values")
                if not isinstance(vals, list) or not vals:
                    raise KeyError(f"globals_schema['{key}']: enum values fehlt/leer.")
                cbx.addItems([str(v) for v in vals])
                w = cbx
                kind = "combo"

            elif t == "number":
                step = float(spec.get("step"))
                minv = float(spec.get("min"))
                maxv = float(spec.get("max"))
                unit = str(spec.get("unit", "") or "")

                if _intish(step, minv, maxv):
                    sb = QSpinBox(self)
                    sb.setMinimum(int(minv))
                    sb.setMaximum(int(maxv))
                    sb.setSingleStep(int(step))
                    w = sb
                    kind = "int"
                else:
                    sb = QDoubleSpinBox(self)
                    sb.setMinimum(minv)
                    sb.setMaximum(maxv)
                    sb.setSingleStep(step)
                    s = str(spec.get("step", "1"))
                    decimals = 0 if "." not in s else min(6, max(1, len(s.split(".")[1])))
                    sb.setDecimals(decimals)
                    w = sb
                    kind = "double"

                if unit:
                    suf = (" " + unit) if not unit.startswith(" ") else unit
                    w.setSuffix(suf)  # type: ignore[attr-defined]

            else:
                raise KeyError(f"globals_schema['{key}']: unsupported type '{t}'.")

            assert w is not None

            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

            if spec.get("description") and hasattr(w, "setToolTip"):
                w.setToolTip(str(spec["description"]))

            self._fields[key] = (w, kind, dict(spec))
            self._form.addRow(f"{key}:", w)

    def apply_model_to_ui(self, model: Recipe) -> None:
        params = getattr(model, "parameters", None)
        if params is None:
            raise KeyError("Recipe.parameters fehlt (None).")
        if not isinstance(params, dict):
            raise TypeError(f"Recipe.parameters ist kein dict (got {type(params)}).")

        for key, (w, kind, spec) in self._fields.items():
            val = params.get(key, spec.get("default"))

            if kind == "check" and isinstance(w, QCheckBox):
                w.setChecked(bool(val))
            elif kind == "string" and isinstance(w, QLineEdit):
                w.setText("" if val is None else str(val))
            elif kind == "combo" and isinstance(w, QComboBox):
                if val is None:
                    continue
                idx = w.findText(str(val))
                if idx < 0:
                    raise KeyError(f"globals param '{key}': ungültiger enum '{val}'.")
                w.setCurrentIndex(idx)
            elif kind == "int" and isinstance(w, QSpinBox):
                if val is None:
                    continue
                w.setValue(int(val))
            elif kind == "double" and isinstance(w, QDoubleSpinBox):
                if val is None:
                    continue
                w.setValue(float(val))

    def apply_ui_to_model(self, model: Recipe) -> None:
        if getattr(model, "parameters", None) is None:
            model.parameters = {}
        if not isinstance(model.parameters, dict):
            raise TypeError(f"Recipe.parameters ist kein dict (got {type(model.parameters)}).")

        for key, (w, kind, _spec) in self._fields.items():
            if kind == "check" and isinstance(w, QCheckBox):
                model.parameters[key] = bool(w.isChecked())
            elif kind == "string" and isinstance(w, QLineEdit):
                model.parameters[key] = str(w.text())
            elif kind == "combo" and isinstance(w, QComboBox):
                model.parameters[key] = str(w.currentText())
            elif kind == "int" and isinstance(w, QSpinBox):
                model.parameters[key] = int(w.value())
            elif kind == "double" and isinstance(w, QDoubleSpinBox):
                model.parameters[key] = float(w.value())


class _TabBarWithChecks(QTabBar):
    """
    Tabs mit Checkboxes (pro Side).
    SSoT: Der Check-Zustand ist die Definition, welche Sides "enabled" sind.
    """
    checkedChanged = pyqtSignal(str, bool)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self.setMovable(False)
        self.setExpanding(False)
        self.setUsesScrollButtons(True)
        self._checks: Dict[int, QCheckBox] = {}
        self._side_by_index: Dict[int, str] = {}
        self._index_by_side: Dict[str, int] = {}

    def addTabWithCheck(self, label: str, side_name: str, checked: bool = True) -> int:
        idx = self.addTab(label)

        cb = QCheckBox(self)
        cb.setChecked(bool(checked))

        def _emit_state(s: int, sn: str = side_name) -> None:
            is_checked = (s == int(Qt.CheckState.Checked.value))
            self.checkedChanged.emit(sn, is_checked)

        cb.stateChanged.connect(_emit_state)
        self.setTabButton(idx, QTabBar.ButtonPosition.RightSide, cb)

        self._checks[idx] = cb
        self._side_by_index[idx] = side_name
        self._index_by_side[side_name] = idx
        return idx

    def side_for_index(self, idx: int) -> str:
        return str(self._side_by_index.get(int(idx), "") or "")

    def is_side_checked(self, side: str) -> bool:
        idx = self._index_by_side.get(str(side))
        if idx is None:
            return False
        cb = self._checks.get(idx)
        return bool(cb.isChecked()) if cb is not None else False

    def set_side_checked(self, side: str, checked: bool) -> None:
        idx = self._index_by_side.get(str(side))
        if idx is None:
            return
        cb = self._checks.get(idx)
        if cb is None:
            return
        cb.blockSignals(True)
        cb.setChecked(bool(checked))
        cb.blockSignals(False)

    def checked_sides(self) -> List[str]:
        out: List[str] = []
        for idx in range(self.count()):
            side = self._side_by_index.get(idx)
            cb = self._checks.get(idx)
            if side and cb is not None and cb.isChecked():
                out.append(str(side))
        return out

    def clear_tabs(self) -> None:
        for i in range(self.count() - 1, -1, -1):
            try:
                w = self.tabButton(i, QTabBar.ButtonPosition.RightSide)
                if w is not None:
                    w.setParent(None)
                    w.deleteLater()
            except Exception:
                pass
            try:
                self.removeTab(i)
            except Exception:
                pass

        self._checks.clear()
        self._side_by_index.clear()
        self._index_by_side.clear()

    def clear(self) -> None:  # type: ignore[override]
        self.clear_tabs()


class RecipeEditorContent(QWidget):
    """
    Editor-Inhalt inkl. Sides.

    NEU (Fix):
      - Side-Checkboxen bestimmen, welche Sides im Model landen:
          model.parameters["enabled_sides"] = ["top","front",...]
      - apply_ui_to_model() schreibt paths_by_side NUR für enabled_sides
        (damit SceneManager nicht auf leere/untype'd Seiten läuft)
      - model.parameters["active_side"] wird auf aktuell selektierten Tab gesetzt
        (für 2D/Details)
    """

    sig_side_enable_changed = pyqtSignal(str, bool)  # side, enabled

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        self.sel_recipe: Optional[QComboBox] = None
        self.sel_tool: Optional[QComboBox] = None
        self.sel_substrate: Optional[QComboBox] = None
        self.sel_mount: Optional[QComboBox] = None

        # Meta
        self.e_recipe_name: Optional[QLineEdit] = None
        self.e_desc: Optional[QLineEdit] = None

        self._params_box: Optional[GlobalParamsBox] = None
        self._planner_box: Optional[PlannerGroupBox] = None
        self._optimize_box: Optional[PlannerGroupBox] = None
        self._side_tabbar: Optional[_TabBarWithChecks] = None

        self._paths_host: Optional[QWidget] = None
        self._paths_host_l: Optional[QVBoxLayout] = None

        self._side_editors: Dict[str, SidePathEditor] = {}
        self._side_order: List[str] = []

        # local cache of enabled sides (mirrors tabbar checks)
        self._side_enabled: Dict[str, bool] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ---------------- Meta (oben) ----------------
        gb_meta = QGroupBox("Meta", self)
        gb_meta_l = QFormLayout(gb_meta)
        gb_meta_l.setContentsMargins(8, 8, 8, 8)
        gb_meta_l.setSpacing(6)

        self.e_recipe_name = QLineEdit(gb_meta)
        self.e_recipe_name.setPlaceholderText("Recipe name (UI only) ...")

        self.e_desc = QLineEdit(gb_meta)
        self.e_desc.setPlaceholderText("Short description ...")

        gb_meta_l.addRow("Recipe Name:", self.e_recipe_name)
        gb_meta_l.addRow("Description:", self.e_desc)

        _set_policy(gb_meta, v=QSizePolicy.Policy.Fixed)
        root.addWidget(gb_meta)

        # ---------------- Selection (darunter) ----------------
        gb_sel = QGroupBox("Selection", self)
        gb_sel_l = QHBoxLayout(gb_sel)
        gb_sel_l.setContentsMargins(8, 8, 8, 8)
        gb_sel_l.setSpacing(8)

        self.sel_recipe = QComboBox(gb_sel)
        self.sel_recipe.setMinimumWidth(220)
        gb_sel_l.addWidget(QLabel("Recipe:", gb_sel))
        gb_sel_l.addWidget(self.sel_recipe)

        self.sel_tool = QComboBox(gb_sel)
        self.sel_substrate = QComboBox(gb_sel)
        self.sel_mount = QComboBox(gb_sel)

        gb_sel_l.addWidget(QLabel("Tool:", gb_sel))
        gb_sel_l.addWidget(self.sel_tool, 1)
        gb_sel_l.addWidget(QLabel("Substrate:", gb_sel))
        gb_sel_l.addWidget(self.sel_substrate, 1)
        gb_sel_l.addWidget(QLabel("Mount:", gb_sel))
        gb_sel_l.addWidget(self.sel_mount, 1)

        _set_policy(gb_sel, v=QSizePolicy.Policy.Fixed)
        root.addWidget(gb_sel)

        # ---- Params + Planner row (HBOX) ----
        row_params_planner = QHBoxLayout()
        row_params_planner.setContentsMargins(0, 0, 0, 0)
        row_params_planner.setSpacing(8)

        # Left: global params
        self._params_box = GlobalParamsBox(store=self.store, parent=self)

        # Right: planners stacked vertically (move + optimize)
        planners_host = QWidget(self)
        planners_l = QVBoxLayout(planners_host)
        planners_l.setContentsMargins(0, 0, 0, 0)
        planners_l.setSpacing(8)

        self._planner_box = PlannerGroupBox(parent=planners_host, title="Move planner", role="move", store=self.store)
        self._optimize_box = PlannerGroupBox(parent=planners_host, title="Optimize planner", role="optimize", store=self.store)

        planners_l.addWidget(self._planner_box)
        planners_l.addWidget(self._optimize_box)
        planners_l.addStretch(1)

        row_params_planner.addWidget(self._params_box, 3)
        row_params_planner.addWidget(planners_host, 2)

        root.addLayout(row_params_planner)

        root.addWidget(_hline())

        self._side_tabbar = _TabBarWithChecks(self)
        root.addWidget(self._side_tabbar)

        self._paths_host = QWidget(self)
        self._paths_host_l = QVBoxLayout(self._paths_host)
        self._paths_host_l.setContentsMargins(0, 0, 0, 0)
        self._paths_host_l.setSpacing(6)
        root.addWidget(self._paths_host, 1)

        _set_policy(self._paths_host, v=QSizePolicy.Policy.Expanding)

        if self._side_tabbar is not None:
            self._side_tabbar.currentChanged.connect(self._on_side_tab_changed)
            self._side_tabbar.checkedChanged.connect(self._on_side_checked_changed)

    # ---------------- Side enable state ----------------

    def checked_sides(self) -> List[str]:
        if self._side_tabbar is None:
            return []
        return self._side_tabbar.checked_sides()

    def _on_side_checked_changed(self, side: str, enabled: bool) -> None:
        s = str(side or "").strip()
        if not s:
            return
        self._side_enabled[s] = bool(enabled)
        self.sig_side_enable_changed.emit(s, bool(enabled))

    def _apply_side_checks_from_model(self, model: Recipe) -> None:
        """
        Restore enabled_sides from model.parameters if present.
        If missing -> default: all enabled.
        """
        if self._side_tabbar is None:
            return

        enabled_sides: Optional[List[str]] = None
        try:
            params = getattr(model, "parameters", None)
            if isinstance(params, dict):
                es = params.get("enabled_sides", None)
                if isinstance(es, list):
                    enabled_sides = [str(x).strip() for x in es if str(x).strip()]
        except Exception:
            enabled_sides = None

        if not enabled_sides:
            # default: all checked
            for s in self._side_order:
                self._side_tabbar.set_side_checked(s, True)
                self._side_enabled[s] = True
            return

        enabled_set = set(enabled_sides)
        for s in self._side_order:
            chk = (s in enabled_set)
            self._side_tabbar.set_side_checked(s, chk)
            self._side_enabled[s] = chk

        # Ensure at least one enabled (fallback: first)
        if not any(self._side_enabled.get(s, False) for s in self._side_order) and self._side_order:
            s0 = self._side_order[0]
            self._side_tabbar.set_side_checked(s0, True)
            self._side_enabled[s0] = True

    # ---------------- Recipe Name API (UI only) ----------------

    def get_recipe_name(self) -> str:
        return str(self.e_recipe_name.text() if self.e_recipe_name is not None else "").strip()

    def set_recipe_name(self, name: str) -> None:
        if self.e_recipe_name is None:
            return
        if self.e_recipe_name.hasFocus():
            return
        self.e_recipe_name.setText(str(name or "").strip())

    # ---------------- Meta ----------------

    def set_meta(self, *, name: str = "", desc: str = "") -> None:
        if self.e_recipe_name is not None and not self.e_recipe_name.hasFocus():
            self.e_recipe_name.setText(name or "")
        if self.e_desc is not None and not self.e_desc.hasFocus():
            self.e_desc.setText(desc or "")

    def _on_side_tab_changed(self, idx: int) -> None:
        if idx < 0 or idx >= len(self._side_order):
            return
        active_side = self._side_order[idx]
        for side, ed in self._side_editors.items():
            ed.setVisible(side == active_side)

    # ---------------- Model -> UI ----------------

    def apply_model_to_ui(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        rid = str(rec_def.get("id") or "").strip()

        tools = [str(x) for x in (self.store.tools_for_recipe(rec_def) or [])]
        subs = [str(x) for x in (self.store.substrates_for_recipe(rec_def) or [])]
        mounts = [str(x) for x in (self.store.mounts_for_recipe(rec_def) or [])]

        def _fill(cb: Optional[QComboBox], values: List[str], current: Optional[str]) -> None:
            if cb is None:
                return
            cb.blockSignals(True)
            cb.clear()
            for v in values:
                cb.addItem(v)
            if current and current in values:
                cb.setCurrentText(current)
            elif cb.count() > 0:
                cb.setCurrentIndex(0)
            cb.blockSignals(False)

        _fill(self.sel_tool, tools, getattr(model, "tool", None))
        _fill(self.sel_substrate, subs, getattr(model, "substrate", None))
        _fill(self.sel_mount, mounts, getattr(model, "substrate_mount", None))

        if self._params_box is not None:
            self._params_box.apply_model_to_ui(model)

        if self._planner_box is not None:
            planner = getattr(model, "planner", {}) or {}
            move_cfg = planner.get("move")
            cfg = move_cfg if isinstance(move_cfg, dict) else (planner if isinstance(planner, dict) else {})
            self._planner_box.apply_planner_model(cfg or {})

        if self._optimize_box is not None:
            planner = getattr(model, "planner", {}) or {}
            opt_cfg = planner.get("optimize")
            opt_model = opt_cfg if isinstance(opt_cfg, dict) else {}
            self._optimize_box.apply_planner_model(opt_model or {})

        sides_cfg = self.store.sides_for_recipe(rec_def)
        if not isinstance(sides_cfg, dict) or not sides_cfg:
            raise KeyError(f"Recipe '{rid}': sides fehlt/leer.")

        sides = list(sides_cfg.keys())
        self._rebuild_side_editors(sides=sides, model=model, rec_def=rec_def)

        # Restore checks after tabs exist
        self._apply_side_checks_from_model(model)

        # Restore active_side if present
        try:
            params = getattr(model, "parameters", {}) or {}
            a = str(params.get("active_side", "") or "").strip()
            if a and a in self._side_order and self._side_tabbar is not None:
                idx = self._side_order.index(a)
                self._side_tabbar.setCurrentIndex(idx)
                self._on_side_tab_changed(idx)
        except Exception:
            pass

    def _rebuild_side_editors(self, *, sides: List[str], model: Recipe, rec_def: Dict[str, Any]) -> None:
        for ed in self._side_editors.values():
            ed.setParent(None)
            ed.deleteLater()
        self._side_editors.clear()

        self._side_order = list(sides)
        self._side_enabled = {str(s): True for s in self._side_order}

        if self._side_tabbar is not None:
            self._side_tabbar.blockSignals(True)
            self._side_tabbar.clear_tabs()
            self._side_tabbar.blockSignals(False)

        if self._paths_host_l is None:
            raise RuntimeError("_paths_host_l ist None (Layout nicht initialisiert).")

        for side in self._side_order:
            if self._side_tabbar is not None:
                self._side_tabbar.addTabWithCheck(side, side, checked=True)

            ed = SidePathEditor(store=self.store, side_name=side, parent=self._paths_host)
            ed.enable_auto_reset(rec_def, side)

            model_path = (getattr(model, "paths_by_side", None) or {}).get(side) or {}
            if isinstance(model_path, dict) and model_path:
                ed.apply_default_path(dict(model_path))

            planner = getattr(model, "planner", {}) or {}
            path_cfg = planner.get("path")
            side_pl = (path_cfg.get(side) if isinstance(path_cfg, dict) else None)
            ed.apply_path_planner_model(side_pl or {})

            ed.setVisible(False)
            self._paths_host_l.addWidget(ed)
            self._side_editors[side] = ed

        if self._side_tabbar is not None and self._side_tabbar.count() > 0:
            self._side_tabbar.setCurrentIndex(0)
            self._on_side_tab_changed(0)

    # ---------------- UI -> Model ----------------

    def apply_ui_to_model(self, model: Recipe) -> None:
        if self.sel_tool is not None and self.sel_tool.currentIndex() >= 0:
            model.tool = self.sel_tool.currentText().strip()
        if self.sel_substrate is not None and self.sel_substrate.currentIndex() >= 0:
            model.substrate = self.sel_substrate.currentText().strip()
        if self.sel_mount is not None and self.sel_mount.currentIndex() >= 0:
            model.substrate_mount = self.sel_mount.currentText().strip()

        if self.e_desc is not None:
            model.description = self.e_desc.text().strip()

        if self._params_box is not None:
            self._params_box.apply_ui_to_model(model)

        if getattr(model, "planner", None) is None:
            model.planner = {}
        if not isinstance(model.planner, dict):
            raise TypeError(f"Recipe.planner ist kein dict (got {type(model.planner)}).")

        if self._planner_box is not None:
            model.planner["move"] = self._planner_box.collect_planner()
        if self._optimize_box is not None:
            model.planner["optimize"] = self._optimize_box.collect_planner()

        # --- Side enable SSoT ---
        enabled = self.checked_sides()
        if getattr(model, "parameters", None) is None:
            model.parameters = {}
        if not isinstance(model.parameters, dict):
            raise TypeError(f"Recipe.parameters ist kein dict (got {type(model.parameters)}).")

        model.parameters["enabled_sides"] = list(enabled)

        # Active side = currently selected tab (for 2D etc.)
        try:
            if self._side_tabbar is not None:
                idx = self._side_tabbar.currentIndex()
                if 0 <= idx < len(self._side_order):
                    model.parameters["active_side"] = self._side_order[idx]
        except Exception:
            pass

        if getattr(model, "paths_by_side", None) is None:
            model.paths_by_side = {}
        if not isinstance(model.paths_by_side, dict):
            raise TypeError(f"Recipe.paths_by_side ist kein dict (got {type(model.paths_by_side)}).")

        # Collect ONLY enabled sides (fix: prevents empty/untype'd sides from being used)
        model.paths_by_side.clear()

        # Path planner per side: also only enabled sides
        if not isinstance(model.planner.get("path"), dict):
            model.planner["path"] = {}
        else:
            model.planner["path"].clear()

        for side, ed in self._side_editors.items():
            if side not in enabled:
                continue

            p = ed.collect_path()
            if not isinstance(p, dict) or not str(p.get("type") or "").strip():
                # Hard fail is better than silently producing type=""
                raise ValueError(f"Side '{side}': path.type fehlt/leer (Side ist enabled).")

            model.paths_by_side[side] = p

            pl = ed.collect_path_planner()
            if pl:
                model.planner["path"][side] = pl
