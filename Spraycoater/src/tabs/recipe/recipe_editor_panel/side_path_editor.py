# -*- coding: utf-8 -*-
# File: src/tabs/recipe/recipe_editor_panel/side_path_editor.py
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt6.QtCore import QSignalBlocker
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QFormLayout,
    QLabel,
    QDoubleSpinBox,
    QSpinBox,
    QCheckBox,
    QComboBox,
    QStackedWidget,
    QLineEdit,
    QSizePolicy,
)

# WICHTIG: KEIN Import von RecipeEditorContent hier, sonst Circular Import!
from model.recipe.recipe_store import RecipeStore
from widgets.planner_groupbox import PlannerGroupBox


# ------------------------------------------------------------
# STRICT nested-dict mapping for dotted schema keys
# ------------------------------------------------------------

def _nested_get(d: Any, key: str, default: Any = None) -> Any:
    """
    STRICT:
      - Dotted keys (e.g. "area.shape") are stored as nested dicts: {"area": {"shape": ...}}
      - No legacy support for literal dotted keys in dict.
    """
    if not isinstance(d, dict):
        return default
    key = str(key or "").strip()
    if not key:
        return default
    if "." not in key:
        return d.get(key, default)

    cur: Any = d
    for part in key.split("."):
        if not isinstance(cur, dict):
            return default
        if part not in cur:
            return default
        cur = cur[part]
    return cur


def _nested_set(d: Dict[str, Any], key: str, value: Any) -> None:
    """
    STRICT:
      - Dotted keys are written as nested dicts.
      - Example: key="area.shape" -> d["area"]["shape"] = value
    """
    key = str(key or "").strip()
    if not key:
        return
    if "." not in key:
        d[key] = value
        return

    parts = key.split(".")
    cur: Dict[str, Any] = d
    for p in parts[:-1]:
        nxt = cur.get(p)
        if not isinstance(nxt, dict):
            nxt = {}
            cur[p] = nxt
        cur = nxt
    cur[parts[-1]] = value


class Vec2Edit(QWidget):
    """2D-Vektor-Editor (x/y) als zwei Spinboxen."""

    def __init__(self, step: float = 0.1, unit: str = "", parent: Optional[QWidget] = None):
        super().__init__(parent)
        self._changed_cb = None

        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)

        self.x = QDoubleSpinBox(self)
        self.x.setDecimals(6)
        self.x.setSingleStep(float(step))

        self.y = QDoubleSpinBox(self)
        self.y.setDecimals(6)
        self.y.setSingleStep(float(step))

        if unit:
            suf = (" " + unit) if not unit.startswith(" ") else unit
            self.x.setSuffix(suf)
            self.y.setSuffix(suf)

        lay.addWidget(QLabel("x:", self))
        lay.addWidget(self.x)
        lay.addSpacing(6)
        lay.addWidget(QLabel("y:", self))
        lay.addWidget(self.y)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

        # Forward signals
        self.x.valueChanged.connect(self._on_change)
        self.y.valueChanged.connect(self._on_change)

    def set_on_change(self, cb):
        self._changed_cb = cb

    def _on_change(self, *args):
        if self._changed_cb:
            self._changed_cb()

    def set_limits(self, minv: float, maxv: float):
        self.x.setMinimum(minv)
        self.x.setMaximum(maxv)
        self.y.setMinimum(minv)
        self.y.setMaximum(maxv)

    def set(self, v: Sequence[float]) -> None:
        self.x.setValue(float(v[0] if v else 0.0))
        self.y.setValue(float(v[1] if v and len(v) > 1 else 0.0))

    def get(self) -> List[float]:
        return [float(self.x.value()), float(self.y.value())]


def _compact_form(f: QFormLayout) -> None:
    f.setContentsMargins(4, 4, 4, 4)
    f.setHorizontalSpacing(6)
    f.setVerticalSpacing(4)
    f.setFieldGrowthPolicy(QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    f.setRowWrapPolicy(QFormLayout.RowWrapPolicy.DontWrapRows)


def _intish(*vals: Any) -> bool:
    try:
        return all(float(v).is_integer() for v in vals)
    except Exception:
        return False


class SidePathEditor(QWidget):
    """
    Editor für eine Seite. Baut sich VOLLSTÄNDIG dynamisch aus dem RecipeStore auf.

    STRICT:
      - Schema keys like "area.shape" are persisted as nested dicts (area: {shape: ...}).
      - No legacy support for literal dotted keys in recipe dict.
    """

    def __init__(
        self,
        *,
        side_name: str,
        store: RecipeStore,
        ctx: Any | None = None,
        parent: Optional[QWidget] = None,  # Parent ist generisch QWidget, kein Import nötig
    ):
        super().__init__(parent)
        self.side_name = side_name
        self.store = store
        self.ctx = ctx

        self._side_cfg: Dict[str, Any] = {}

        # ptype -> (page_widget, fields_map, params_schema, deps_map)
        self._type_pages: Dict[str, Tuple[QWidget, Dict, Dict, Dict]] = {}

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # -------- Type selector (oben) --------
        top_form = QFormLayout()
        _compact_form(top_form)

        self.type_combo = QComboBox(self)
        top_form.addRow(f"type ({self.side_name})", self.type_combo)
        root.addLayout(top_form)

        # -------- Bottom split (Parameter | Planner) --------
        bottom = QHBoxLayout()
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.setSpacing(8)
        root.addLayout(bottom)

        # Links: Parameter-Stack
        self.stack = QStackedWidget(self)
        sp_stack = self.stack.sizePolicy()
        sp_stack.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_stack.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.stack.setSizePolicy(sp_stack)
        bottom.addWidget(self.stack, 3)

        # Rechts: Planner Settings
        self.pathPlanner = PlannerGroupBox(
            parent=self,
            title="Path planner",
            role="path",
            store=self.store,
        )
        sp_pl = self.pathPlanner.sizePolicy()
        sp_pl.setHorizontalPolicy(QSizePolicy.Policy.Preferred)
        sp_pl.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.pathPlanner.setSizePolicy(sp_pl)
        bottom.addWidget(self.pathPlanner, 2)

        sp_self = self.sizePolicy()
        sp_self.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_self.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp_self)

        self.type_combo.currentIndexChanged.connect(self._on_type_changed)

    # ------------------------------------------------------------------ strict SSoT init

    def enable_auto_reset(self, rec_def: Dict[str, Any], side: str) -> None:
        side_cfg = self.store.allowed_and_default_for(rec_def, side)
        allowed = side_cfg.get("allowed_path_types")
        if not isinstance(allowed, list) or not allowed:
            raise KeyError(f"Side '{side}': allowed_path_types fehlt/leer.")

        allowed_types = [str(x).strip() for x in allowed if str(x).strip()]
        default_path = side_cfg.get("default_path") or {}

        schemas: Dict[str, Any] = {}
        for ptype in allowed_types:
            schemas[ptype] = dict(self.store.schema_for_type_strict(ptype) or {})

        self._side_cfg = {
            "allowed_types": allowed_types,
            "default_path": dict(default_path),
            "schemas": schemas,
        }

        self.set_allowed_types(allowed_types)

        dp = dict(default_path)
        dp["_side_cfg"] = dict(self._side_cfg)
        self.apply_default_path(dp)

    # ------------------------------------------------------------------ Pages Builder

    def _clear_pages(self) -> None:
        self._type_pages.clear()
        while self.stack.count() > 0:
            w = self.stack.widget(0)
            self.stack.removeWidget(w)
            w.deleteLater()

    def set_allowed_types(self, types: List[str]) -> None:
        allowed = [str(t).strip() for t in types if str(t).strip()]
        schemas = self._side_cfg.get("schemas", {})

        with QSignalBlocker(self.type_combo):
            self.type_combo.clear()
            self.type_combo.addItems(allowed)

        self._clear_pages()

        for ptype in allowed:
            params_schema = dict(schemas.get(ptype) or {})
            page, fields_map, deps_map = self._build_page_for_type(ptype, params_schema)

            self._type_pages[ptype] = (page, fields_map, params_schema, deps_map)
            self.stack.addWidget(page)

        # Reset selection
        with QSignalBlocker(self.type_combo):
            if self.type_combo.count() > 0:
                self.type_combo.setCurrentIndex(0)

        self._on_type_changed(0)

    def _build_page_for_type(
        self,
        ptype: str,
        params_schema: Dict[str, Any],
    ) -> Tuple[QWidget, Dict, Dict]:
        page = QWidget(self)
        form = QFormLayout(page)
        _compact_form(form)

        def sort_key(k: str) -> tuple:
            k = str(k)
            prio = 50
            if k.startswith("area."):
                prio = 10
            elif "radius" in k:
                prio = 11
            elif "size" in k:
                prio = 12
            elif "pitch" in k:
                prio = 20
            elif "loops" in k:
                prio = 21
            elif "direction" in k:
                prio = 30
            elif "angle" in k and "predispense" not in k and "retreat" not in k:
                prio = 31
            elif "start" in k:
                prio = 32
            elif "margin" in k or "offset" in k:
                prio = 40
            elif "z_mm" in k or "height" in k:
                prio = 41
            elif "predispense" in k:
                prio = 90
            elif "retreat" in k:
                prio = 91
            return (prio, k)

        keys = sorted(list(params_schema.keys()), key=sort_key)
        fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]] = {}
        deps: Dict[str, Tuple[str, Any]] = {}

        for key in keys:
            spec = dict(params_schema.get(key) or {})
            t = str(spec.get("type", "")).strip().lower()
            default_val = spec.get("default")
            desc_text = str(spec.get("description") or "")

            w: Optional[QWidget] = None
            kind: str = ""
            row_label = str(key)

            # --- Widget Creation ---
            if t == "boolean":
                w = QCheckBox(str(key), page)
                if default_val is not None:
                    w.setChecked(bool(default_val))
                kind = "check"
                row_label = ""
            elif t == "string":
                w = QLineEdit(page)
                if default_val is not None:
                    w.setText(str(default_val))
                kind = "string"
            elif t == "enum":
                w = QComboBox(page)
                kind = "combo"
                vals = spec.get("values") or []
                w.addItems([str(v) for v in vals])
                if default_val is not None:
                    idx = w.findText(str(default_val))
                    if idx >= 0:
                        w.setCurrentIndex(idx)
            elif t == "vec2":
                step_spec = spec.get("step", 0.1)
                step = float(step_spec[0] if isinstance(step_spec, (list, tuple)) and step_spec else step_spec)
                unit = str(spec.get("unit", "") or "")
                w = Vec2Edit(step=step, unit=unit, parent=page)
                min_spec, max_spec = spec.get("min"), spec.get("max")
                if isinstance(min_spec, (list, tuple)) and isinstance(max_spec, (list, tuple)):
                    w.set_limits(float(min_spec[0]), float(max_spec[0]))
                if default_val is not None and isinstance(default_val, (list, tuple)):
                    w.set(default_val)
                kind = "vec2"
            elif t == "number":
                step = float(spec.get("step", 1.0))
                minv, maxv = float(spec.get("min", 0.0)), float(spec.get("max", 9999.0))
                unit = str(spec.get("unit", "") or "")
                if _intish(step, minv, maxv, default_val if default_val is not None else 0):
                    sb = QSpinBox(page)
                    sb.setMinimum(int(minv))
                    sb.setMaximum(int(maxv))
                    sb.setSingleStep(int(step))
                    if default_val is not None:
                        sb.setValue(int(default_val))
                    w = sb
                    kind = "int"
                else:
                    sb = QDoubleSpinBox(page)
                    sb.setMinimum(minv)
                    sb.setMaximum(maxv)
                    sb.setSingleStep(step)
                    s_step = str(spec.get("step", "1"))
                    decimals = 0 if "." not in s_step else min(6, max(1, len(s_step.split(".")[1])))
                    sb.setDecimals(decimals)
                    if default_val is not None:
                        sb.setValue(float(default_val))
                    w = sb
                    kind = "double"
                if unit:
                    suf = (" " + unit) if not unit.startswith(" ") else unit
                    w.setSuffix(suf)
            else:
                continue

            assert w is not None
            w.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
            if desc_text:
                w.setToolTip(desc_text)

            fields[str(key)] = (w, kind, spec)
            form.addRow(row_label, w)

            # Check visibility dependency
            vcond = spec.get("visible_if")
            if isinstance(vcond, dict) and len(vcond) == 1:
                dep_key, dep_val = next(iter(vcond.items()))
                deps[str(key)] = (str(dep_key), dep_val)

        # Wire visibility signals
        update_vis_lambda = lambda *_: self._update_current_page_visibility()

        for _target_key, (src_key, _) in deps.items():
            if src_key not in fields:
                continue
            w_src, kind_src, _ = fields[src_key]

            if kind_src == "combo":
                w_src.currentTextChanged.connect(update_vis_lambda)
            elif kind_src == "check":
                w_src.stateChanged.connect(update_vis_lambda)
            elif kind_src in ("int", "double"):
                w_src.valueChanged.connect(update_vis_lambda)
            elif kind_src == "string":
                w_src.textChanged.connect(update_vis_lambda)
            elif kind_src == "vec2":
                w_src.set_on_change(update_vis_lambda)

        return page, fields, deps

    def _update_current_page_visibility(self) -> None:
        """Manually runs the visibility check for the active page."""
        ptype = self.type_combo.currentText()
        if ptype not in self._type_pages:
            return

        page, fields, _, deps = self._type_pages[ptype]
        layout = page.layout()
        if not isinstance(layout, QFormLayout):
            return

        # Helper to get current val from widget
        def _get_val(k: str) -> Any:
            w, kind, _ = fields[k]
            if kind == "check":
                return w.isChecked()
            if kind == "combo":
                return w.currentText()
            if kind == "int":
                return w.value()
            if kind == "double":
                return w.value()
            if kind == "string":
                return w.text()
            return None

        current_vals = {k: _get_val(k) for k in fields}

        for target_key, (src_key, expected_val) in deps.items():
            if src_key not in current_vals:
                continue

            is_visible = (str(current_vals[src_key]) == str(expected_val))

            w_target = fields[target_key][0]
            idx = layout.indexOf(w_target)
            if idx >= 0:
                row, _role = layout.getItemPosition(idx)
                layout.setRowVisible(row, is_visible)

        self._lock_stack_height()

    # ------------------------------------------------------------------ Values Apply / Collect

    def apply_default_path(self, path: Dict[str, Any]) -> None:
        if isinstance(path, dict) and "_side_cfg" in path:
            new_cfg = dict(path["_side_cfg"] or {})
            if new_cfg != self._side_cfg:
                self._side_cfg = new_cfg
                allowed_types = [str(t).strip() for t in (self._side_cfg.get("allowed_types") or []) if str(t).strip()]
                self.set_allowed_types(allowed_types)

        ptype = str(path.get("type") or "").strip()
        if not ptype:
            allowed = self._side_cfg.get("allowed_types", [])
            if allowed:
                ptype = allowed[0]
            else:
                return

        # 1) Set Type
        idx = self.type_combo.findText(ptype)
        if idx >= 0:
            with QSignalBlocker(self.type_combo):
                self.type_combo.setCurrentIndex(idx)
            self._on_type_changed(idx)

        # 2) Set Values (STRICT nested get for dotted keys)
        _page, fields_map, _, _ = self._type_pages.get(ptype, (None, {}, {}, {}))

        for key, (w, kind, spec) in fields_map.items():
            val = _nested_get(path, key, spec.get("default"))

            if kind == "check" and isinstance(w, QCheckBox):
                w.setChecked(bool(val))
            elif kind == "combo" and isinstance(w, QComboBox):
                if val is not None:
                    cidx = w.findText(str(val))
                    if cidx >= 0:
                        w.setCurrentIndex(cidx)
            elif kind == "int" and isinstance(w, QSpinBox):
                if val is not None:
                    w.setValue(int(val))
            elif kind == "double" and isinstance(w, QDoubleSpinBox):
                if val is not None:
                    w.setValue(float(val))
            elif kind == "string" and isinstance(w, QLineEdit):
                if val is not None:
                    w.setText(str(val))
            elif kind == "vec2" and isinstance(w, Vec2Edit):
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    w.set([float(val[0]), float(val[1])])

        # 3) Trigger visibility update
        self._update_current_page_visibility()

    def collect_path(self) -> Dict[str, Any]:
        """
        STRICT:
          - All dotted schema keys are written as nested dictionaries.
          - No legacy literal dotted keys are produced.
        """
        ptype = self.type_combo.currentText().strip()
        out: Dict[str, Any] = {"type": ptype}

        _page, fields_map, _, _ = self._type_pages.get(ptype, (None, {}, {}, {}))

        for key, (w, kind, _) in fields_map.items():
            if kind == "double":
                _nested_set(out, key, float(w.value()))
            elif kind == "int":
                _nested_set(out, key, int(w.value()))
            elif kind == "check":
                _nested_set(out, key, bool(w.isChecked()))
            elif kind == "combo":
                _nested_set(out, key, str(w.currentText()))
            elif kind == "string":
                _nested_set(out, key, str(w.text()))
            elif kind == "vec2":
                _nested_set(out, key, w.get())

        return out

    # ------------------------------------------------------------------ Internals

    def apply_path_planner_model(self, cfg: Dict[str, Any] | None) -> None:
        if hasattr(self.pathPlanner, "apply_planner_model"):
            self.pathPlanner.apply_planner_model(cfg or {})

    def collect_path_planner(self) -> Dict[str, Any]:
        if hasattr(self.pathPlanner, "collect_planner"):
            return self.pathPlanner.collect_planner()
        return {}

    def _on_type_changed(self, _idx: int) -> None:
        idx = self.type_combo.currentIndex()
        if idx < 0:
            return
        if 0 <= idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
            self._update_current_page_visibility()

    def _lock_stack_height(self) -> None:
        w = self.stack.currentWidget()
        if w is not None:
            self.stack.setMaximumHeight(w.sizeHint().height())
        self.stack.adjustSize()
        self.adjustSize()
