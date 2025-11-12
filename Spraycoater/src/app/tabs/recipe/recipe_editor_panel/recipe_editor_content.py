# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Dict, Any, Tuple, List

from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QFrame,
    QTabWidget, QLineEdit, QLabel, QTabBar, QSizePolicy, QPushButton
)

from app.model.recipe.recipe import Recipe
    # type: ignore
from app.model.recipe.recipe_store import RecipeStore
from app.widgets.planner_groupbox import PlannerGroupBox
from .side_path_editor import SidePathEditor


def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setFrameShadow(QFrame.Sunken)
    return f


def _unique_str_list(items) -> List[str]:
    seen = set()
    out: List[str] = []
    for x in (items or []):
        s = str(x)
        if s not in seen:
            seen.add(s)
            out.append(s)
    return out


def _coalesce_options(
    model: Recipe,
    *,
    single: str,
    plurals: List[str],
    rec_def: Optional[Dict[str, Any]] = None,
) -> List[str]:
    opts: List[str] = []
    for attr in plurals:
        vals = getattr(model, attr, None)
        if vals:
            opts.extend(vals)
    if isinstance(rec_def, dict):
        for attr in plurals:
            vals = rec_def.get(attr)
            if vals:
                opts.extend(vals)
    single_val = getattr(model, single, None)
    if single_val and not opts:
        opts = [single_val]
    return _unique_str_list(opts)


def _coalesce_sides_from_rec_def(rec_def: Dict[str, Any]) -> Dict[str, Any]:
    sides = rec_def.get("sides") or {}
    return dict(sides) if isinstance(sides, dict) else {}


def _compact_form(form: QFormLayout) -> None:
    form.setContentsMargins(6, 6, 6, 6)
    form.setHorizontalSpacing(6)
    form.setVerticalSpacing(4)
    form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.DontWrapRows)


def _set_policy(w: QWidget, *, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


class CheckableTabWidget(QTabWidget):
    checkedChanged = pyqtSignal(str, bool)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMovable(True)
        self._side_checkboxes: Dict[int, QCheckBox] = {}

        tb = self.tabBar()
        tb.setExpanding(False)
        self.setDocumentMode(False)  # Pane-Rand sichtbar
        tb.setDrawBase(True)
        _set_policy(tb, h=QSizePolicy.Policy.Preferred, v=QSizePolicy.Policy.Preferred)
        _set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)

        self.setStyleSheet("""
            QTabWidget::pane { border: 1px solid palette(mid); top: -1px; }
            QTabBar::tab { margin: 1px; padding: 3px 8px; }
        """)

        if hasattr(tb, "tabMoved"):
            tb.tabMoved.connect(self._on_tab_moved)  # type: ignore[attr-defined]

    def add_checkable_tab(self, widget: QWidget, side_name: str, checked: bool = True) -> None:
        idx = self.addTab(widget, side_name)

        cb = QCheckBox()
        cb.setChecked(checked)
        cb.setToolTip(f"Aktiviert: {side_name}")
        _set_policy(cb, h=QSizePolicy.Policy.Preferred, v=QSizePolicy.Policy.Preferred)

        container = QWidget()
        lay = QHBoxLayout(container)
        lay.setContentsMargins(4, 0, 0, 0)
        lay.setSpacing(4)
        lay.addWidget(cb)
        _set_policy(container, h=QSizePolicy.Policy.Preferred, v=QSizePolicy.Policy.Preferred)

        self.tabBar().setTabButton(idx, QTabBar.ButtonPosition.LeftSide, container)
        self._side_checkboxes[idx] = cb

        cb.toggled.connect(lambda state, s=side_name: self.checkedChanged.emit(s, state))

    def _on_tab_moved(self, _from_idx: int, _to_idx: int) -> None:
        new_map: Dict[int, QCheckBox] = {}
        for i in range(self.count()):
            btn = self.tabBar().tabButton(i, QTabBar.ButtonPosition.LeftSide)
            if btn:
                cb = btn.findChild(QCheckBox)
                if cb:
                    new_map[i] = cb
        self._side_checkboxes = new_map

    def is_checked(self, side_name: str) -> bool:
        for i in range(self.count()):
            if self.tabText(i) == side_name:
                cb = self._side_checkboxes.get(i)
                return bool(cb.isChecked()) if cb else True
        return False

    def checked_sides(self) -> List[str]:
        out: List[str] = []
        for i in range(self.count()):
            cb = self._side_checkboxes.get(i)
            if not cb or cb.isChecked():
                out.append(self.tabText(i))
        return out


class RecipeEditorContent(QWidget):
    """
    Kopf wird extern (Panel) gebaut.
    Hier: zweispaltig — links (Meta oben + Selectors unten), rechts (Globals) + unten Paths + Planner + Footer.
    """
    validateRequested = pyqtSignal()
    optimizeRequested = pyqtSignal()
    updatePreviewRequested = pyqtSignal(object)  # model: Recipe

    def __init__(self, *, ctx=None, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        self._side_editors: Dict[str, SidePathEditor] = {}

        self.sel_recipe: Optional[QComboBox] = None
        self.sel_tool: Optional[QComboBox] = None
        self.sel_substrate: Optional[QComboBox] = None
        self.sel_mount: Optional[QComboBox] = None

        # Meta-Felder (verschoben aus Panel)
        self.e_name: Optional[QLineEdit] = None
        self.e_desc: Optional[QLineEdit] = None

        self._globals_widgets: Dict[str, QWidget] = {}

        self._model: Optional[Recipe] = None
        self._rec_def: Optional[Dict[str, Any]] = None
        self._last_ctx_key: Optional[str] = None

        # Planner unten im Content
        self.plannerBox: Optional[PlannerGroupBox] = None

        self._build_ui()
        self._rebuild_globals_from_schema()
        self.apply_defaults()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)
        _set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

        # ============ obere Zeile: links VBox(Meta, Selectors) | rechts Globals ============
        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(8)

        # -- linke VBox: Meta oben, Selectors unten --
        left_col = QVBoxLayout()
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(8)

        # Meta
        self.gb_meta = QGroupBox("Meta")
        _set_policy(self.gb_meta, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        metaForm = QFormLayout(self.gb_meta)
        _compact_form(metaForm)
        self.e_name = QLineEdit(self.gb_meta)
        self.e_name.setPlaceholderText("recipe_name (YAML id / Dateiname)")
        self.e_desc = QLineEdit(self.gb_meta)
        self.e_desc.setPlaceholderText("Short description ...")
        metaForm.addRow(QLabel("Name:", self.gb_meta), self.e_name)
        metaForm.addRow(QLabel("Description:", self.gb_meta), self.e_desc)

        # Selectors
        sel_gb = QGroupBox("Selectors")
        _set_policy(sel_gb, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        sf = QFormLayout(sel_gb)
        _compact_form(sf)
        self.sel_recipe = QComboBox()
        self.sel_tool = QComboBox()
        self.sel_substrate = QComboBox()
        self.sel_mount = QComboBox()
        for c in (self.sel_recipe, self.sel_tool, self.sel_substrate, self.sel_mount):
            _set_policy(c, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        sf.addRow("recipe", self.sel_recipe)
        sf.addRow("tool", self.sel_tool)
        sf.addRow("substrate", self.sel_substrate)
        sf.addRow("mount", self.sel_mount)

        left_col.addWidget(self.gb_meta, 0)
        left_col.addWidget(sel_gb, 1)

        # Globals (rechts)
        self.gb_globals = QGroupBox("Globals")
        _set_policy(self.gb_globals, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        self.globals_form = QFormLayout(self.gb_globals)
        _compact_form(self.globals_form)

        top.addLayout(left_col, 1)
        top.addWidget(self.gb_globals, 1)

        root.addLayout(top)

        # --- unten: Paths (Checkable Tabs im sichtbaren Pane) ---
        self.gb_paths = QGroupBox("Paths", self)
        paths_v = QVBoxLayout(self.gb_paths)
        paths_v.setContentsMargins(8, 6, 8, 8)
        paths_v.setSpacing(6)

        self.sideTabs = CheckableTabWidget(self)
        self.sideTabs.checkedChanged.connect(self._on_side_checked_changed)
        for w in (self.gb_paths, self.sideTabs):
            _set_policy(w, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)
        paths_v.addWidget(self.sideTabs)

        # Planner unter den Side-Tabs
        self.plannerBox = PlannerGroupBox(store=self.store, parent=self)
        _set_policy(self.plannerBox, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Maximum)
        paths_v.addWidget(self.plannerBox)

        root.addWidget(self.gb_paths, 1)

        # --- Footer-Buttons (3 Stück): Update Preview, Validate, Optimize ---
        foot = QHBoxLayout()
        foot.setContentsMargins(0, 0, 0, 0)
        foot.setSpacing(8)
        self.btnUpdatePreview = QPushButton("Update Preview", self)
        self.btnValidate = QPushButton("Validate", self)
        self.btnOptimize = QPushButton("Optimize", self)
        for b in (self.btnUpdatePreview, self.btnValidate, self.btnOptimize):
            sp = b.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Preferred)
            b.setSizePolicy(sp)
            foot.addWidget(b, 1)
        root.addLayout(foot)

        # Wiring Footer
        self.btnValidate.clicked.connect(self.validateRequested.emit)
        self.btnOptimize.clicked.connect(self.optimizeRequested.emit)
        self.btnUpdatePreview.clicked.connect(self._emit_update_preview)

    # --- Helpers für Footer ---
    def _emit_update_preview(self):
        model = self._model or self._new_fallback_model()
        if model is not None:
            self.updatePreviewRequested.emit(model)

    def _new_fallback_model(self) -> Optional[Recipe]:
        try:
            if self._rec_def:
                params = self.store.collect_global_defaults()
                planner = self.store.collect_planner_defaults()
                pbs = self.store.build_default_paths_for_recipe(self._rec_def)
                return Recipe.from_dict({
                    "id": "",
                    "description": self._rec_def.get("description") or "",
                    "tool": None,
                    "substrate": None,
                    "substrates": [],
                    "substrate_mount": None,
                    "parameters": params,
                    "planner": planner,
                    "paths_by_side": pbs,
                })
        except Exception:
            pass
        return None

    def _rebuild_globals_from_schema(self) -> None:
        while self.globals_form.rowCount() > 0:
            self.globals_form.removeRow(0)
        self._globals_widgets.clear()

        schema = self.store.globals_schema()

        priority = [
            "max_points",
            "sample_step_mm",
            "stand_off_mm",
            "max_angle_deg",
            "predispense.angle_deg",
            "predispense.distance_mm",
            "retreat.angle_deg",
            "retreat.distance_mm",
        ]
        keys_all = list(schema.keys())
        first = [k for k in priority if k in schema]
        rest = sorted([k for k in keys_all if k not in set(first)])

        def _add_row_for_key(key: str):
            spec = dict(schema[key] or {})
            t = (spec.get("type") or "").strip().lower()
            unit = str(spec.get("unit", "") or "")
            label = key

            if t == "boolean":
                w = QCheckBox(label)
                _set_policy(w)
                self.globals_form.addRow("", w)
            elif t == "string":
                w = QLineEdit()
                _set_policy(w)
                self.globals_form.addRow(label, w)
            elif t == "enum":
                w = QComboBox()
                _set_policy(w)
                w.addItems([str(v) for v in (spec.get("values") or [])])
                self.globals_form.addRow(label, w)
            elif t == "number":
                step = float(spec.get("step", 1.0))
                minv = float(spec.get("min", 0.0))
                maxv = float(spec.get("max", 0.0))
                intish = False
                try:
                    intish = float(step).is_integer() and float(minv).is_integer() and float(maxv).is_integer()
                except Exception:
                    intish = False

                if intish:
                    w = QSpinBox()
                    w.setMinimum(int(minv))
                    w.setMaximum(int(maxv))
                    w.setSingleStep(int(step))
                else:
                    w = QDoubleSpinBox()
                    w.setMinimum(minv)
                    w.setMaximum(maxv)
                    w.setSingleStep(step)
                    s = str(spec.get("step", "1"))
                    decimals = 0 if "." not in s else min(6, max(1, len(s.split(".")[1])))
                    w.setDecimals(decimals)
                if unit:
                    if not unit.startswith(" "):
                        unit = " " + unit
                    w.setSuffix(unit)
                _set_policy(w)
                self.globals_form.addRow(label, w)
            else:
                w = QLabel(f"(unsupported type: {t})")
                _set_policy(w, h=QSizePolicy.Policy.Preferred)
                self.globals_form.addRow(label, w)

            if "default" in spec:
                dv = spec["default"]
                if isinstance(w, QCheckBox):
                    w.setChecked(bool(dv))
                elif isinstance(w, QComboBox):
                    ix = w.findText(str(dv))
                    w.setCurrentIndex(ix if ix >= 0 else (0 if w.count() > 0 else -1))
                elif isinstance(w, QLineEdit):
                    w.setText(str(dv))
                elif isinstance(w, QSpinBox):
                    w.setValue(int(dv))
                elif isinstance(w, QDoubleSpinBox):
                    w.setValue(float(dv))

            self._globals_widgets[key] = w

        for k in first:
            _add_row_for_key(k)
        if first and rest:
            self.globals_form.addRow(_hline())
        for k in rest:
            _add_row_for_key(k)

    def apply_defaults(self) -> None:
        self._clear_recipe_tabs()

    def _clear_recipe_tabs(self) -> None:
        while self.sideTabs.count() > 0:
            w = self.sideTabs.widget(0)
            self.sideTabs.removeTab(0)
            w.deleteLater()
        self._side_editors.clear()
        self._disconnect_selectors()
        self._last_ctx_key = None
        self._rec_def = None

    def _disconnect_selectors(self) -> None:
        for combo in (self.sel_tool, self.sel_substrate, self.sel_mount):
            if not combo:
                continue
            try:
                combo.currentIndexChanged.disconnect(self._on_selectors_changed)
            except Exception:
                pass

    def apply_recipe_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        self._model = model
        self._rec_def = rec_def

        # Meta setzen
        if self.e_name is not None:
            self.e_name.setText(getattr(model, "id", "") or "")
        if self.e_desc is not None:
            self.e_desc.setText(getattr(model, "description", "") or "")

        while self.sideTabs.count() > 0:
            w = self.sideTabs.widget(0)
            self.sideTabs.removeTab(0)
            w.deleteLater()
        self._side_editors.clear()

        tools = _coalesce_options(model, single="tool", plurals=["tools", "tool_options"], rec_def=rec_def)
        substrates = _coalesce_options(model, single="substrate", plurals=["substrates", "substrate_options"], rec_def=rec_def)
        mounts = _coalesce_options(model, single="substrate_mount", plurals=["substrate_mounts", "mounts", "mount_options"], rec_def=rec_def)

        def _fill(combo: QComboBox, items: List[str]) -> None:
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(items)
            combo.blockSignals(False)

        _fill(self.sel_tool, tools)
        _fill(self.sel_substrate, substrates)
        _fill(self.sel_mount, mounts)

        def _set_or_first(combo: QComboBox, value: Optional[str]) -> None:
            if combo is None:
                return
            if value:
                idx = combo.findText(str(value))
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                    return
            if combo.count() > 0:
                combo.setCurrentIndex(0)

        _set_or_first(self.sel_tool, getattr(model, "tool", None))
        _set_or_first(self.sel_substrate, getattr(model, "substrate", None))
        _set_or_first(self.sel_mount, getattr(model, "substrate_mount", None))

        sides: Dict[str, Any] = _coalesce_sides_from_rec_def(rec_def)

        for side_name in sides.keys():
            editor = SidePathEditor(side_name=side_name, store=self.store)
            editor.setObjectName(f"side_editor__{side_name}")

            side_runtime = self.store.build_side_runtime_cfg_strict(rec_def, side_name)
            allowed = list(side_runtime.get("allowed_path_types") or [])
            editor._side_cfg = dict(side_runtime)
            editor.set_allowed_types(allowed)

            model_path = (model.paths_by_side or {}).get(side_name) if isinstance(model.paths_by_side, dict) else None
            default_path = dict(model_path or side_runtime.get("default_path") or {})
            default_path["_side_cfg"] = dict(side_runtime)
            editor.apply_default_path(default_path)
            editor.enable_auto_reset(rec_def, side_name)

            self.sideTabs.add_checkable_tab(editor, side_name, checked=True)
            self._side_editors[side_name] = editor

        if self.sideTabs.count() > 0:
            self.sideTabs.setCurrentIndex(0)

        if self.sel_substrate:
            self.sel_substrate.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_mount:
            self.sel_mount.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_tool:
            self.sel_tool.currentIndexChanged.connect(self._on_selectors_changed)

        tool, sub, mnt = self.active_selectors_values()
        if self._model:
            self._model.tool = tool
            self._model.substrate = sub
            self._model.substrates = [sub] if sub else []
            self._model.substrate_mount = mnt

        # Planner initial
        try:
            self.apply_planner_model(getattr(model, "planner", {}) or {})
        except Exception:
            pass

    def collect_globals(self) -> Dict[str, Any]:
        out: Dict[str, Any] = {}
        for key, w in self._globals_widgets.items():
            if isinstance(w, QCheckBox):
                out[key] = bool(w.isChecked())
            elif isinstance(w, QComboBox):
                out[key] = str(w.currentText())
            elif isinstance(w, QLineEdit):
                out[key] = str(w.text())
            elif isinstance(w, QSpinBox):
                out[key] = int(w.value())
            elif isinstance(w, QDoubleSpinBox):
                out[key] = float(w.value())
        return out

    def collect_paths_by_side(self) -> Dict[str, Any]:
        out: Dict[str, Dict[str, Any]] = {}
        for side, ed in (self._side_editors or {}).items():
            out[side] = ed.collect_path()
        return out

    def active_selectors_values(self) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        tool = (self.sel_tool.currentText().strip() if self.sel_tool and self.sel_tool.currentIndex() >= 0 else None)
        sub = (self.sel_substrate.currentText().strip() if self.sel_substrate and self.sel_substrate.currentIndex() >= 0 else None)
        mnt = (self.sel_mount.currentText().strip() if self.sel_mount and self.sel_mount.currentIndex() >= 0 else None)
        return tool or None, sub or None, mnt or None

    def meta_values(self) -> Tuple[str, str]:
        name = (self.e_name.text().strip() if self.e_name else "")
        desc = (self.e_desc.text().strip() if self.e_desc else "")
        return name, desc

    def set_meta(self, *, name: str = "", desc: str = "") -> None:
        if self.e_name is not None:
            self.e_name.setText(name or "")
        if self.e_desc is not None:
            self.e_desc.setText(desc or "")

    def _current_ctx_key(self) -> str:
        tool = self.sel_tool.currentText().strip() if self.sel_tool and self.sel_tool.currentIndex() >= 0 else ""
        sub = self.sel_substrate.currentText().strip() if self.sel_substrate and self.sel_substrate.currentIndex() >= 0 else ""
        mnt = self.sel_mount.currentText().strip() if self.sel_mount and self.sel_mount.currentIndex() >= 0 else ""
        return f"{mnt}|{sub}|{tool}"

    def _on_selectors_changed(self, _idx: int = 0) -> None:
        key = self._current_ctx_key()
        if key == self._last_ctx_key:
            return
        self._last_ctx_key = key
        if self._model and hasattr(self._model, "on_context_changed"):
            try:
                tool, sub, mnt = self.active_selectors_values()
                self._model.on_context_changed(tool, sub, mnt)
            except Exception:
                pass

    def _on_side_checked_changed(self, side_name: str, checked: bool) -> None:
        try:
            if hasattr(self.ctx, "on_side_check_changed"):
                self.ctx.on_side_check_changed(side_name, checked, self.sideTabs.checked_sides())
        except Exception:
            pass

    def checked_sides(self) -> List[str]:
        return self.sideTabs.checked_sides()

    # ---- Planner-API nach außen (vom Panel genutzt) ----
    def apply_planner_model(self, model: Dict[str, Any]) -> None:
        if self.plannerBox and hasattr(self.plannerBox, "apply_model"):
            try:
                self.plannerBox.apply_model(model)
            except Exception:
                pass

    def collect_planner(self) -> Dict[str, Any]:
        if self.plannerBox and hasattr(self.plannerBox, "collect"):
            try:
                return dict(self.plannerBox.collect() or {})
            except Exception:
                return {}
        return {}
