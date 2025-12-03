# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QStackedWidget, QLineEdit,
    QSizePolicy
)

from app.model.recipe.recipe_store import RecipeStore
from app.widgets.planner_groupbox import PlannerGroupBox


class Vec2Edit(QWidget):
    def __init__(self, step: float = 0.1, unit: str = "", parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        self.x = QDoubleSpinBox(); self.x.setDecimals(6); self.x.setSingleStep(step)
        self.y = QDoubleSpinBox(); self.y.setDecimals(6); self.y.setSingleStep(step)
        if unit:
            suf = (" " + unit) if not unit.startswith(" ") else unit
            self.x.setSuffix(suf); self.y.setSuffix(suf)
        lay.addWidget(QLabel("x:")); lay.addWidget(self.x)
        lay.addSpacing(6)
        lay.addWidget(QLabel("y:")); lay.addWidget(self.y)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    def set(self, v: Sequence[float]):
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


def _intish(*vals) -> bool:
    try:
        return all(float(v).is_integer() for v in vals)
    except Exception:
        return False


class SidePathEditor(QWidget):
    """
    Editor für EINE Side:

      oben:   type (Combo)
      unten:  HBox:
                [links]  Path-Parameter-Stack
                [rechts] Path-Planner (role="path")
    """

    def __init__(self, *, side_name: str, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.side_name = side_name
        self.store = store

        self._side_cfg: Dict[str, Any] = {}
        # ptype -> (page, fields_map, params_schema)
        self._type_pages: Dict[str, Tuple[QWidget, Dict[str, Tuple[QWidget, str, Dict[str, Any]]], Dict[str, Any]]] = {}

        # ==================================================================
        # Root-Layout: VBox
        # ==================================================================
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # --- TYPE oben in eigenem FormLayout ---
        top_form = QFormLayout()
        _compact_form(top_form)
        self.type_combo = QComboBox()
        top_form.addRow(f"type ({self.side_name})", self.type_combo)
        root.addLayout(top_form)

        # ==================================================================
        # Untere HBox: [links] Path-Stack  |  [rechts] Path-Planner
        # ==================================================================
        bottom = QHBoxLayout()
        bottom.setContentsMargins(0, 0, 0, 0)
        bottom.setSpacing(8)
        root.addLayout(bottom)

        # LINKS: Stack mit Parametern
        self.stack = QStackedWidget()
        sp_stack = self.stack.sizePolicy()
        sp_stack.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_stack.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.stack.setSizePolicy(sp_stack)
        bottom.addWidget(self.stack, 3)

        # RECHTS: Path-Planner (role="path")
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

        # Gesamt-SizePolicy
        sp_self = self.sizePolicy()
        sp_self.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_self.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp_self)

        self.type_combo.currentIndexChanged.connect(self._on_type_changed)

    # ------------------------------------------------------------------ UI-Pages

    def _clear_pages(self):
        self._type_pages.clear()
        while self.stack.count() > 0:
            w = self.stack.widget(0)
            self.stack.removeWidget(w)
            w.deleteLater()

    def set_allowed_types(self, types: List[str]) -> None:
        allowed = list(types or [])
        self.type_combo.blockSignals(True)
        self.type_combo.clear()
        self.type_combo.addItems(allowed)
        self.type_combo.blockSignals(False)

        self._clear_pages()
        schemas = (self._side_cfg.get("schemas") or {}) if isinstance(self._side_cfg, dict) else {}
        for ptype in allowed:
            params = dict((schemas.get(ptype) or {}))
            page, fields_map = self._build_page_for_type(ptype, params)
            self._type_pages[ptype] = (page, fields_map, params)
            self.stack.addWidget(page)

        if self.type_combo.count() > 0:
            self._on_type_changed(0)

    def _build_page_for_type(
        self,
        ptype: str,
        params: Dict[str, Any]
    ) -> Tuple[QWidget, Dict[str, Tuple[QWidget, str, Dict[str, Any]]]]:
        page = QWidget()
        f = QFormLayout(page)
        _compact_form(f)

        sp_page = page.sizePolicy()
        sp_page.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_page.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        page.setSizePolicy(sp_page)

        fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]] = {}

        keys = list(params.keys())
        keys.sort(key=lambda k: (0 if k.endswith("shape") else 1, k))

        for key in keys:
            spec = dict(params.get(key) or {})
            t = str(spec.get("type", "")).strip().lower()
            w: Optional[QWidget] = None
            kind = ""
            row_label = key

            if t == "boolean":
                w = QCheckBox(key); kind = "check"; row_label = ""
            elif t == "string":
                w = QLineEdit(); kind = "string"
            elif t == "enum":
                w = QComboBox(); kind = "combo"
                w.addItems([str(v) for v in (spec.get("values") or [])])
            elif t == "vec2":
                step = float(
                    spec.get("step", [0.1, 0.1])[0]
                    if isinstance(spec.get("step"), (list, tuple))
                    else spec.get("step", 0.1)
                )
                unit = str(spec.get("unit", "") or "")
                w = Vec2Edit(step=step, unit=unit); kind = "vec2"
            elif t == "number":
                step = float(spec.get("step", 1.0))
                minv = float(spec.get("min", 0.0))
                maxv = float(spec.get("max", 0.0))
                unit = str(spec.get("unit", "") or "")
                if _intish(step, minv, maxv):
                    sb = QSpinBox()
                    sb.setMinimum(int(minv)); sb.setMaximum(int(maxv)); sb.setSingleStep(int(step))
                    w = sb; kind = "int"
                else:
                    sb = QDoubleSpinBox()
                    sb.setMinimum(minv); sb.setMaximum(maxv); sb.setSingleStep(step)
                    s = str(spec.get("step", "1"))
                    decimals = 0 if "." not in s else min(6, max(1, len(s.split(".")[1])))
                    sb.setDecimals(decimals)
                    w = sb; kind = "double"
                if unit:
                    if not unit.startswith(" "):
                        unit = " " + unit
                    w.setSuffix(unit)
            else:
                continue

            sp = w.sizePolicy()
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

            fields[key] = (w, kind, spec)
            f.addRow(row_label, w)

            if spec.get("help") and hasattr(w, "setToolTip"):
                w.setToolTip(str(spec["help"]))

        self._wire_visibility(fields, form=f)
        return page, fields

    # ------------------------------------------------------------------ Sichtbarkeit

    def _wire_visibility(
        self,
        fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]],
        *,
        form: QFormLayout
    ) -> None:
        deps: Dict[str, Tuple[str, Any]] = {}
        for key, (_w, _kind, spec) in fields.items():
            vcond = spec.get("visible_if")
            if isinstance(vcond, dict) and len(vcond) == 1:
                dep_key, dep_val = next(iter(vcond.items()))
                deps[key] = (dep_key, dep_val)

        def _val(widget: QWidget, kind: str):
            if kind == "double": return float(widget.value())
            if kind == "int":    return int(widget.value())
            if kind == "check":  return bool(widget.isChecked())
            if kind == "combo":  return str(widget.currentText())
            if kind == "string": return str(widget.text())
            return None

        def _form_row_for_widget(form: QFormLayout, w: QWidget) -> int:
            idx = form.indexOf(w)
            if idx >= 0:
                row, _role = form.getItemPosition(idx)
                return row
            rc = form.rowCount()
            for row in range(rc):
                for role in (QFormLayout.ItemRole.LabelRole, QFormLayout.ItemRole.FieldRole):
                    it = form.itemAt(row, role)
                    ww = it.widget() if it is not None else None
                    if ww and (ww is w or ww == w or ww.isAncestorOf(w) or w.isAncestorOf(ww)):
                        return row
            return -1

        def _set_row_visible(w: QWidget, visible: bool):
            row = _form_row_for_widget(form, w)
            if row < 0:
                w.setVisible(visible); return
            li = form.itemAt(row, QFormLayout.ItemRole.LabelRole)
            lw = li.widget() if li is not None else None
            if lw is not None: lw.setVisible(visible)
            fi = form.itemAt(row, QFormLayout.ItemRole.FieldRole)
            fw = fi.widget() if fi is not None else None
            if fw is not None: fw.setVisible(visible)

        def _apply_visibility():
            values: Dict[str, Any] = {k: _val(w, kind) for k, (w, kind, _) in fields.items()}
            for tgt, (dep_k, dep_v) in deps.items():
                w_tgt = fields[tgt][0]
                visible = (str(values.get(dep_k)) == str(dep_v))
                _set_row_visible(w_tgt, visible)
            self._lock_stack_height()

        for dep_tgt, (dep_key, _) in deps.items():
            if dep_key not in fields:
                continue
            w, kind, _ = fields[dep_key]
            if kind == "combo":
                w.currentTextChanged.connect(lambda _t, fn=_apply_visibility: fn())
            elif kind in ("int", "double"):
                w.valueChanged.connect(lambda _v, fn=_apply_visibility: fn())
            elif kind == "check":
                w.stateChanged.connect(lambda _v, fn=_apply_visibility: fn())
            elif kind == "string":
                w.textChanged.connect(lambda _v, fn=_apply_visibility: fn())

        _apply_visibility()

    # ------------------------------------------------------------------ API: Path-Defaults

    def apply_default_path(self, path: Dict[str, Any], *, helix_enums=None, plane_enums=None) -> None:
        if isinstance(path, dict) and "_side_cfg" in path:
            self._side_cfg = dict(path["_side_cfg"] or {})

        t = str(path.get("type") or "").strip()
        if self.type_combo.findText(t) < 0 and self.type_combo.count() > 0:
            t = self.type_combo.itemText(0)
        if t:
            self.type_combo.setCurrentText(t)
        self._on_type_changed(self.type_combo.currentIndex())

        page, fields_map, _params = self._type_pages.get(t, (None, {}, {}))
        for key, (w, kind, spec) in fields_map.items():
            val = path[key] if key in path else spec.get("default", None)
            if kind == "check":
                w.setChecked(bool(val))
            elif kind == "combo":
                if val is not None:
                    idx = w.findText(str(val))
                    w.setCurrentIndex(idx if idx >= 0 else (0 if w.count() > 0 else -1))
            elif kind == "string":
                w.setText("" if val is None else str(val))
            elif kind == "int":
                if val is not None:
                    w.setValue(int(val))
            elif kind == "double":
                if val is not None:
                    w.setValue(float(val))
            elif isinstance(w, Vec2Edit):
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    w.set([float(val[0]), float(val[1])])

        self._lock_stack_height()

    def collect_path(self) -> Dict[str, Any]:
        t = self.type_combo.currentText().strip()
        out: Dict[str, Any] = {"type": t} if t else {}
        page, fields_map, _params = self._type_pages.get(t, (None, {}, {}))
        for key, (w, kind, _spec) in fields_map.items():
            if kind == "double":
                out[key] = float(w.value())
            elif kind == "int":
                out[key] = int(w.value())
            elif kind == "check":
                out[key] = bool(w.isChecked())
            elif kind == "combo":
                out[key] = str(w.currentText())
            elif kind == "string":
                out[key] = str(w.text())
            elif isinstance(w, Vec2Edit):
                out[key] = w.get()
        return out

    # ------------------------------------------------------------------ Path-Planner API (pro Side)

    def apply_path_planner_model(self, cfg: Dict[str, Any] | None) -> None:
        """Recipe.planner['path'][side_name] → UI."""
        self.pathPlanner.apply_planner_model(cfg or {})

    def collect_path_planner(self) -> Dict[str, Any]:
        """UI → Planner-Config für diese Side."""
        return self.pathPlanner.collect_planner()

    # ------------------------------------------------------------------ Auto-Reset (nur Path-Defaults)

    def enable_auto_reset(self, rec_def: Dict[str, Any], side_name: str) -> None:
        self._side_cfg = self.store.build_side_runtime_cfg_strict(rec_def, side_name)

        def _reload_defaults():
            side_default = dict((self._side_cfg.get("default_path") or {}))
            cur_t = self.type_combo.currentText().strip() or side_default.get("type") or ""
            if cur_t:
                side_default["type"] = cur_t
            self.apply_default_path(side_default)

        self.type_combo.currentTextChanged.connect(lambda _t: _reload_defaults())

    # ------------------------------------------------------------------ intern

    def _on_type_changed(self, _idx: int) -> None:
        idx = max(0, self.type_combo.currentIndex())
        if 0 <= idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
            self._lock_stack_height()

    def _lock_stack_height(self) -> None:
        w = self.stack.currentWidget()
        if w is not None:
            h = w.sizeHint().height()
            self.stack.setMaximumHeight(h)
        self.stack.adjustSize()
        self.adjustSize()
