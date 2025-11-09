# -*- coding: utf-8 -*-
# File: tabs/recipe/recipe_editor_content.py
from __future__ import annotations
from typing import Optional, Dict, Any, Tuple, List

from PyQt6.QtCore import pyqtSignal, Qt
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QFrame,
    QTabWidget, QLineEdit, QLabel, QHBoxLayout, QTabBar
)

from app.model.recipe.recipe import Recipe
from app.model.recipe.recipe_store import RecipeStore
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


class CheckableTabWidget(QTabWidget):
    """
    QTabWidget mit Checkbox je Tab-Header.
    - Tabs sind per Drag & Drop umsortierbar (setMovable(True)).
    - Signal checkedChanged(side_name, checked).
    - Helfer: is_checked(side), set_checked(side, bool), checked_sides().
    """
    checkedChanged = pyqtSignal(str, bool)

    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.setMovable(True)
        self._side_checkboxes: Dict[int, QCheckBox] = {}

        tb = self.tabBar()
        if hasattr(tb, "tabMoved"):
            tb.tabMoved.connect(self._on_tab_moved)  # type: ignore[attr-defined]

    def add_checkable_tab(self, widget: QWidget, side_name: str, checked: bool = True) -> None:
        idx = self.addTab(widget, side_name)

        cb = QCheckBox()
        cb.setChecked(checked)
        cb.setToolTip(f"Aktiviert: {side_name}")

        container = QWidget()
        lay = QHBoxLayout(container)
        lay.setContentsMargins(6, 0, 0, 0)
        lay.setSpacing(4)
        lay.addWidget(cb)
        lay.addStretch(1)

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

    def set_checked(self, side_name: str, checked: bool) -> None:
        for i in range(self.count()):
            if self.tabText(i) == side_name:
                cb = self._side_checkboxes.get(i)
                if cb:
                    cb.setChecked(checked)
                return

    def checked_sides(self) -> List[str]:
        out: List[str] = []
        for i in range(self.count()):
            cb = self._side_checkboxes.get(i)
            if not cb or cb.isChecked():
                out.append(self.tabText(i))
        return out


class RecipeEditorContent(QWidget):
    """
    Layout:
      VBox
        ├─ GroupBox "Globals" (aus recipe_params.globals dynamisch gebaut)
        ├─ GroupBox "Selectors" (recipe/tool/substrate/mount)
        └─ CheckableTabWidget (jede Side = eigener Tab; direkt SidePathEditor als Inhalt)
             └─ SidePathEditor
    """
    def __init__(self, *, ctx=None, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        # Per Rezeptseite:
        self._side_editors: Dict[str, SidePathEditor] = {}

        # Selectors
        self.sel_recipe: Optional[QComboBox] = None
        self.sel_tool: Optional[QComboBox] = None
        self.sel_substrate: Optional[QComboBox] = None
        self.sel_mount: Optional[QComboBox] = None

        # Globals (dynamisch aus Schema erzeugt)
        self._globals_widgets: Dict[str, QWidget] = {}

        # State
        self._model: Optional[Recipe] = None
        self._rec_def: Optional[Dict[str, Any]] = None
        self._last_ctx_key: Optional[str] = None

        self._build_ui()
        self._rebuild_globals_from_schema()  # dynamisch aus YAML
        self.apply_defaults()

    # ------------------- UI Build -------------------
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(10)

        # === Globals (dynamisch aus recipe_params.globals) ===
        self.gb_globals = QGroupBox("Globals")
        self.globals_form = QFormLayout(self.gb_globals)
        root.addWidget(self.gb_globals)

        # === Selectors (inkl. recipe) ===
        sel_gb = QGroupBox("Selectors")
        sf = QFormLayout(sel_gb)
        self.sel_recipe = QComboBox()
        self.sel_tool = QComboBox()
        self.sel_substrate = QComboBox()
        self.sel_mount = QComboBox()
        sf.addRow("recipe", self.sel_recipe)
        sf.addRow("tool", self.sel_tool)
        sf.addRow("substrate", self.sel_substrate)
        sf.addRow("mount", self.sel_mount)
        root.addWidget(sel_gb)

        # === Sides als Tabs (checkbar + movable) ===
        self.sideTabs = CheckableTabWidget(self)
        self.sideTabs.checkedChanged.connect(self._on_side_checked_changed)
        root.addWidget(self.sideTabs)

    # ------------------- Globals dynamisch aus Schema -------------------
    def _rebuild_globals_from_schema(self) -> None:
        while self.globals_form.rowCount() > 0:
            self.globals_form.removeRow(0)
        self._globals_widgets.clear()

        schema = self.store.globals_schema()  # Dict[key] = spec
        keys = sorted(schema.keys())

        for key in keys:
            spec = dict(schema[key] or {})
            t = str(spec.get("type", "")).lower()
            unit = str(spec.get("unit", "") or "")
            label = key

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
                # int vs double Heuristik
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
                self.globals_form.addRow(label, w)
            else:
                w = QLabel(f"(unsupported type: {t})")
                self.globals_form.addRow(label, w)

            if "default" in spec:
                dv = spec["default"]
                if isinstance(w, QCheckBox):
                    w.setChecked(bool(dv))
                elif isinstance(w, QComboBox):
                    idx = w.findText(str(dv))
                    w.setCurrentIndex(idx if idx >= 0 else (0 if w.count() > 0 else -1))
                elif isinstance(w, QLineEdit):
                    w.setText(str(dv))
                elif isinstance(w, QSpinBox):
                    w.setValue(int(dv))
                elif isinstance(w, QDoubleSpinBox):
                    w.setValue(float(dv))

            self._globals_widgets[key] = w

        self.globals_form.addRow(_hline())

    # ------------------- Defaults (nur Formular-Reset) -------------------
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

    # ------------------- Model-Driven Aufbau -------------------
    def apply_recipe_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        """Baut Selectors und die Sides (Tabs) neu auf (Globals bleiben erhalten)."""
        self._model = model
        self._rec_def = rec_def

        # Tabs leeren
        while self.sideTabs.count() > 0:
            w = self.sideTabs.widget(0)
            self.sideTabs.removeTab(0)
            w.deleteLater()
        self._side_editors.clear()

        # Selectors
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

        # Immer einen gültigen Index wählen; wenn value None -> 0 (falls vorhanden)
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

        # Sides → Tabs
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

        # Selector-Signale für Context-Wechsel
        if self.sel_substrate:
            self.sel_substrate.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_mount:
            self.sel_mount.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_tool:
            self.sel_tool.currentIndexChanged.connect(self._on_selectors_changed)

        # Direkt Model auffüllen
        tool, sub, mnt = self.active_selectors_values()
        if self._model:
            self._model.tool = tool
            self._model.substrate = sub
            self._model.substrates = [sub] if sub else []
            self._model.substrate_mount = mnt

    # ------------------- Collectors -------------------
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

    # ------------------- Priming/Context -------------------
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

    def _prime_all_sides_from_defaults(self) -> None:
        if not (self._rec_def and self.store):
            return
        for side, editor in (self._side_editors or {}).items():
            side_runtime = self.store.build_side_runtime_cfg_strict(self._rec_def, side)
            default_path = dict(side_runtime.get("default_path") or {})
            default_path["_side_cfg"] = dict(side_runtime)
            editor.apply_default_path(default_path)

    # ---- Public Helper für Preview/Rendering ----
    def checked_sides(self) -> List[str]:
        return self.sideTabs.checked_sides()

    def is_side_checked(self, side: str) -> bool:
        return self.sideTabs.is_checked(side)
