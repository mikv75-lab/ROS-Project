# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Optional, Dict, Any, Tuple, List

from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGroupBox,
    QFormLayout,
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
    QComboBox,
    QFrame,
    QLineEdit,
    QLabel,
    QTabBar,
    QSizePolicy,
    QStackedWidget,
)

from.model.recipe.recipe import Recipe
from.model.recipe.recipe_store import RecipeStore
from.widgets.planner_groupbox import PlannerGroupBox
from .side_path_editor import SidePathEditor


Bounds = Tuple[float, float, float, float, float, float]


# ----------------------------- UI helpers -----------------------------

def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setFrameShadow(QFrame.Sunken)
    return f


def _compact_form(form: QFormLayout) -> None:
    form.setContentsMargins(6, 6, 6, 6)
    form.setHorizontalSpacing(6)
    form.setVerticalSpacing(4)
    form.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)
    form.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.DontWrapRows)


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


# ----------------------- checkable tab bar ----------------------

class CheckableTabWidget(QTabBar):
    """
    QTabBar mit Checkbox pro Tab. Dient als TabBar-Teil eines "QTabWidget-ähnlichen" Aufbaus.
    - currentChanged: wird wie üblich vom QTabBar gesendet
    - checkedChanged(side_name, state): eigenes Signal für Checkbox-Status
    """

    checkedChanged = pyqtSignal(str, bool)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMovable(True)
        self.setExpanding(False)
        self.setDrawBase(True)
        _set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)

        self._side_names: List[str] = []
        self._side_boxes: Dict[int, QCheckBox] = {}

        self.setStyleSheet("QTabBar::tab { margin: 1px; padding: 3px 8px; }")

        if hasattr(self, "tabMoved"):
            self.tabMoved.connect(self._on_tab_moved)  # type: ignore[attr-defined]

    def clear_tabs(self) -> None:
        while self.count() > 0:
            self.removeTab(0)
        self._side_boxes.clear()
        self._side_names.clear()

    def add_checkable(self, side_name: str, checked: bool = True) -> None:
        idx = self.addTab(side_name)
        self._side_names.append(side_name)

        container = QWidget()
        lay = QHBoxLayout(container)
        lay.setContentsMargins(4, 0, 0, 0)
        lay.setSpacing(4)

        cb = QCheckBox()
        cb.setChecked(checked)
        cb.setToolTip(f"Aktiviert: {side_name}")
        _set_policy(cb, h=QSizePolicy.Policy.Preferred, v=QSizePolicy.Policy.Preferred)
        lay.addWidget(cb)

        self.setTabButton(idx, QTabBar.ButtonPosition.LeftSide, container)
        self._side_boxes[idx] = cb

        cb.toggled.connect(lambda state, s=side_name: self.checkedChanged.emit(s, state))

    def is_checked(self, side_name: str) -> bool:
        for i in range(self.count()):
            if self.tabText(i) == side_name:
                cb = self._side_boxes.get(i)
                return bool(cb.isChecked()) if cb else True
        return False

    def checked_sides(self) -> List[str]:
        out: List[str] = []
        for i in range(self.count()):
            cb = self._side_boxes.get(i)
            if (cb is None) or cb.isChecked():
                out.append(self.tabText(i))
        return out

    def _on_tab_moved(self, _from_idx: int, _to_idx: int) -> None:
        # Rebuild index->checkbox map after drag & drop.
        new_map: Dict[int, QCheckBox] = {}
        for i in range(self.count()):
            btn = self.tabButton(i, QTabBar.ButtonPosition.LeftSide)
            if btn is None:
                continue
            cb = btn.findChild(QCheckBox)
            if cb is not None:
                new_map[i] = cb
        self._side_boxes = new_map


# ----------------------- main content widget -----------------------

class RecipeEditorContent(QWidget):
    """
    Layout:
      Row 1: Meta | Setup
      Row 2: Globals | Move planner
      Below: Side tabs + Side pages

    Strikt:
      - tool/substrate/mount nur aus rec_def (recipes[..] in YAML)
      - Globals nur aus recipe_params.globals
      - Side-Path-Schemata nur aus recipe_params.path.* (via RecipeStore)
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

        self.e_name: Optional[QLineEdit] = None
        self.e_desc: Optional[QLineEdit] = None

        self._globals_widgets: Dict[str, QWidget] = {}

        self._model: Optional[Recipe] = None
        self._rec_def: Optional[Dict[str, Any]] = None
        self._last_ctx_key: Optional[str] = None

        self.plannerWidget: Optional[PlannerGroupBox] = None

        self._build_ui()
        self._rebuild_globals_from_schema()
        self.apply_defaults()

    # ---------------- UI build ----------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(8)
        _set_policy(self, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

        # ===== Row 1: Meta | Setup =====
        row1 = QHBoxLayout()
        row1.setContentsMargins(0, 0, 0, 0)
        row1.setSpacing(8)

        # Meta
        self.gb_meta = QGroupBox("Meta")
        _set_policy(self.gb_meta, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        meta_form = QFormLayout(self.gb_meta)
        _compact_form(meta_form)

        self.sel_recipe = QComboBox(self.gb_meta)
        self.e_name = QLineEdit(self.gb_meta)
        self.e_name.setPlaceholderText("recipe_name (YAML id / Dateiname)")
        self.e_desc = QLineEdit(self.gb_meta)
        self.e_desc.setPlaceholderText("Short description ...")

        _set_policy(self.sel_recipe)
        _set_policy(self.e_name)
        _set_policy(self.e_desc)

        meta_form.addRow(QLabel("Recipe:", self.gb_meta), self.sel_recipe)
        meta_form.addRow(QLabel("Name:", self.gb_meta), self.e_name)
        meta_form.addRow(QLabel("Description:", self.gb_meta), self.e_desc)

        # Setup (tool/substrate/mount)
        ctx_gb = QGroupBox("Setup")
        _set_policy(ctx_gb, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        setup_form = QFormLayout(ctx_gb)
        _compact_form(setup_form)

        self.sel_tool = QComboBox()
        self.sel_substrate = QComboBox()
        self.sel_mount = QComboBox()
        for c in (self.sel_tool, self.sel_substrate, self.sel_mount):
            _set_policy(c, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)

        setup_form.addRow("tool", self.sel_tool)
        setup_form.addRow("substrate", self.sel_substrate)
        setup_form.addRow("mount", self.sel_mount)

        row1.addWidget(self.gb_meta, 1)
        row1.addWidget(ctx_gb, 1)
        root.addLayout(row1)

        # ===== Row 2: Globals | Move planner =====
        row2 = QHBoxLayout()
        row2.setContentsMargins(0, 0, 0, 0)
        row2.setSpacing(8)

        self.gb_globals = QGroupBox("Globals")
        _set_policy(self.gb_globals, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        self.globals_form = QFormLayout(self.gb_globals)
        _compact_form(self.globals_form)

        self.plannerWidget = PlannerGroupBox(
            store=self.store,
            parent=self,
            title="Move planner",
            role="move",
        )
        _set_policy(self.plannerWidget, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)

        row2.addWidget(self.gb_globals, 1)
        row2.addWidget(self.plannerWidget, 1)
        root.addLayout(row2)

        # ===== Sides (TabBar + Pages) =====
        self.sideTabsBar = CheckableTabWidget(self)
        _set_policy(self.sideTabsBar, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        root.addWidget(self.sideTabsBar)

        self.sidePages = QStackedWidget(self)
        _set_policy(self.sidePages, h=QSizePolicy.Policy.Preferred, v=QSizePolicy.Policy.Preferred)
        root.addWidget(self.sidePages)

        self.sideTabsBar.currentChanged.connect(self.sidePages.setCurrentIndex)
        self.sideTabsBar.checkedChanged.connect(self._on_side_checked_changed)

    # ---------------- Globals UI from schema ----------------

    def _rebuild_globals_from_schema(self) -> None:
        """
        Baut Globals strikt anhand von recipe_params.globals (RecipeStore).
        """
        while self.globals_form.rowCount() > 0:
            self.globals_form.removeRow(0)
        self._globals_widgets.clear()

        schema = self.store.globals_schema()

        priority = ["stand_off_mm", "max_angle_deg", "sample_step_mm", "max_points", "scene_z_offset_mm"]
        keys_all = list(schema.keys())
        first = [k for k in priority if k in schema]
        rest = sorted([k for k in keys_all if k not in set(first)])

        def _apply_default(w: QWidget, dv: Any) -> None:
            if isinstance(w, QCheckBox):
                w.setChecked(bool(dv))
            elif isinstance(w, QComboBox):
                ix = w.findText(str(dv))
                if ix >= 0:
                    w.setCurrentIndex(ix)
            elif isinstance(w, QLineEdit):
                w.setText(str(dv))
            elif isinstance(w, QSpinBox):
                w.setValue(int(dv))
            elif isinstance(w, QDoubleSpinBox):
                w.setValue(float(dv))

        def _add_row_for_key(key: str) -> None:
            spec = dict(schema.get(key) or {})
            t = str(spec.get("type") or "").strip().lower()
            unit = str(spec.get("unit", "") or "")
            label = key

            w: QWidget

            if t == "boolean":
                w = QCheckBox(label)
                self.globals_form.addRow("", w)

            elif t == "string":
                w = QLineEdit()
                self.globals_form.addRow(label, w)

            elif t == "enum":
                w = QComboBox()
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
                    sb = QSpinBox()
                    sb.setMinimum(int(minv))
                    sb.setMaximum(int(maxv))
                    sb.setSingleStep(int(step))
                    w = sb
                else:
                    dsb = QDoubleSpinBox()
                    dsb.setMinimum(minv)
                    dsb.setMaximum(maxv)
                    dsb.setSingleStep(step)

                    s = str(spec.get("step", "1"))
                    decimals = 0 if "." not in s else min(6, max(1, len(s.split(".")[1])))
                    dsb.setDecimals(decimals)
                    w = dsb

                if unit:
                    suf = unit if unit.startswith(" ") else f" {unit}"
                    if hasattr(w, "setSuffix"):
                        w.setSuffix(suf)  # type: ignore[attr-defined]

                self.globals_form.addRow(label, w)

            else:
                w = QLabel(f"(unsupported type: {t})")
                self.globals_form.addRow(label, w)

            # Policy auf Field-Widget (falls vorhanden)
            field_item = self.globals_form.itemAt(self.globals_form.rowCount() - 1, QFormLayout.ItemRole.FieldRole)
            field_widget = field_item.widget() if field_item else None
            if field_widget is not None:
                _set_policy(field_widget)

            if "default" in spec:
                _apply_default(w, spec["default"])

            self._globals_widgets[key] = w

        for k in first:
            _add_row_for_key(k)
        if first and rest:
            self.globals_form.addRow(_hline())
        for k in rest:
            _add_row_for_key(k)

    # ---------------- lifecycle / model ----------------

    def apply_defaults(self) -> None:
        self._clear_recipe_tabs()

    def _clear_recipe_tabs(self) -> None:
        self.sideTabsBar.clear_tabs()
        while self.sidePages.count() > 0:
            w = self.sidePages.widget(0)
            self.sidePages.removeWidget(w)
            w.deleteLater()
        self._side_editors.clear()
        self._disconnect_selectors()
        self._last_ctx_key = None
        self._rec_def = None

    def _disconnect_selectors(self) -> None:
        for combo in (self.sel_tool, self.sel_substrate, self.sel_mount):
            if combo is None:
                continue
            try:
                combo.currentIndexChanged.disconnect(self._on_selectors_changed)
            except Exception:
                pass

    def apply_recipe_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        """
        Setup/Globals/Sides aus dem Modell setzen.
        Meta (Name/Description) wird extern via set_meta() gesetzt.
        """
        self._model = model
        self._rec_def = rec_def

        self._clear_recipe_tabs()

        # tool/substrate/mount strikt aus rec_def
        tools = [str(t) for t in (rec_def.get("tools") or [])]
        substrates = [str(s) for s in (rec_def.get("substrates") or [])]
        mounts = [str(m) for m in (rec_def.get("substrate_mounts") or [])]

        def _fill(combo: QComboBox, items: List[str]) -> None:
            combo.blockSignals(True)
            combo.clear()
            combo.addItems(items)
            combo.blockSignals(False)

        _fill(self.sel_tool, tools)
        _fill(self.sel_substrate, substrates)
        _fill(self.sel_mount, mounts)

        def _set_or_first(combo: QComboBox, value: Optional[str]) -> None:
            if value:
                idx = combo.findText(str(value))
                if idx >= 0:
                    combo.setCurrentIndex(idx)
                    return
            if combo.count() > 0:
                combo.setCurrentIndex(0)

        _set_or_first(self.sel_tool, model.tool)
        _set_or_first(self.sel_substrate, model.substrate)
        _set_or_first(self.sel_mount, model.substrate_mount)

        # sides strikt aus rec_def["sides"]
        sides_node = rec_def.get("sides") or {}
        if not isinstance(sides_node, dict):
            sides_node = {}

        for side_name in sides_node.keys():
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

            self.sideTabsBar.add_checkable(side_name, checked=True)
            self.sidePages.addWidget(editor)
            self._side_editors[side_name] = editor

        if self.sideTabsBar.count() > 0:
            self.sideTabsBar.setCurrentIndex(0)
            self.sidePages.setCurrentIndex(0)

        # selectors wiring
        if self.sel_substrate is not None:
            self.sel_substrate.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_mount is not None:
            self.sel_mount.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_tool is not None:
            self.sel_tool.currentIndexChanged.connect(self._on_selectors_changed)

        tool, sub, mnt = self.active_selectors_values()
        if self._model is not None:
            self._model.tool = tool
            self._model.substrate = sub
            self._model.substrate_mount = mnt

        # globals from model.parameters
        params = dict(model.parameters or {})
        for key, w in self._globals_widgets.items():
            if key not in params:
                continue
            val = params.get(key)
            if isinstance(w, QCheckBox):
                w.setChecked(bool(val))
            elif isinstance(w, QComboBox):
                if val is not None:
                    idx = w.findText(str(val))
                    if idx >= 0:
                        w.setCurrentIndex(idx)
            elif isinstance(w, QLineEdit):
                w.setText("" if val is None else str(val))
            elif isinstance(w, QSpinBox):
                w.setValue(int(val))
            elif isinstance(w, QDoubleSpinBox):
                w.setValue(float(val))

        # planner
        self.apply_planner_model(model.planner or {})

    # ---------------- collect / getters ----------------

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
        tool = self.sel_tool.currentText().strip() if (self.sel_tool and self.sel_tool.currentIndex() >= 0) else None
        sub = self.sel_substrate.currentText().strip() if (self.sel_substrate and self.sel_substrate.currentIndex() >= 0) else None
        mnt = self.sel_mount.currentText().strip() if (self.sel_mount and self.sel_mount.currentIndex() >= 0) else None
        return tool or None, sub or None, mnt or None

    def meta_values(self) -> Tuple[str, str]:
        name = self.e_name.text().strip() if self.e_name else ""
        desc = self.e_desc.text().strip() if self.e_desc else ""
        return name, desc

    def set_meta(self, *, name: str = "", desc: str = "") -> None:
        if self.e_name is not None:
            self.e_name.setText(name or "")
        if self.e_desc is not None:
            self.e_desc.setText(desc or "")

    # ---------------- internal handlers ----------------

    def _current_ctx_key(self) -> str:
        tool = self.sel_tool.currentText().strip() if (self.sel_tool and self.sel_tool.currentIndex() >= 0) else ""
        sub = self.sel_substrate.currentText().strip() if (self.sel_substrate and self.sel_substrate.currentIndex() >= 0) else ""
        mnt = self.sel_mount.currentText().strip() if (self.sel_mount and self.sel_mount.currentIndex() >= 0) else ""
        return f"{mnt}|{sub}|{tool}"

    def _on_selectors_changed(self, _idx: int = 0) -> None:
        key = self._current_ctx_key()
        if key == self._last_ctx_key:
            return
        self._last_ctx_key = key
        if self._model is not None and hasattr(self._model, "on_context_changed"):
            tool, sub, mnt = self.active_selectors_values()
            self._model.on_context_changed(tool, sub, mnt)

    def _on_side_checked_changed(self, side_name: str, checked: bool) -> None:
        if hasattr(self.ctx, "on_side_check_changed"):
            self.ctx.on_side_check_changed(side_name, checked, self.sideTabsBar.checked_sides())

    def checked_sides(self) -> List[str]:
        return self.sideTabsBar.checked_sides()

    # ---------------- planner integration ----------------

    def apply_planner_model(self, planner_cfg: Dict[str, Any]) -> None:
        if self.plannerWidget is None:
            return
        self.plannerWidget.apply_planner_model(planner_cfg)

    def collect_planner(self) -> Dict[str, Any]:
        if self.plannerWidget is None:
            return {}
        return dict(self.plannerWidget.collect_planner() or {})
