# -*- coding: utf-8 -*-
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence, Tuple

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

from.model.recipe.recipe_store import RecipeStore
from.widgets.planner_groupbox import PlannerGroupBox


class Vec2Edit(QWidget):
    """2D-Vektor-Editor (x/y) als zwei Spinboxen."""

    def __init__(self, step: float = 0.1, unit: str = "", parent: Optional[QWidget] = None):
        super().__init__(parent)

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
    """True, wenn alle Werte integer-kompatibel sind (z. B. step/min/max)."""
    return all(float(v).is_integer() for v in vals)


class SidePathEditor(QWidget):
    """
    Editor für genau eine Side.

    Oben:
      - Path-Type (Combo)

    Unten:
      - links:  Stack der Path-Parameter (abhängig vom Type)
      - rechts: Path-Planner (role="path")
    """

    def __init__(self, *, side_name: str, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.side_name = side_name
        self.store = store

        self._side_cfg: Dict[str, Any] = {}
        # ptype -> (page, fields_map, params_schema)
        self._type_pages: Dict[str, Tuple[QWidget, Dict[str, Tuple[QWidget, str, Dict[str, Any]]], Dict[str, Any]]] = {}

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

        self.stack = QStackedWidget(self)
        sp_stack = self.stack.sizePolicy()
        sp_stack.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_stack.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.stack.setSizePolicy(sp_stack)
        bottom.addWidget(self.stack, 3)

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

    # ------------------------------------------------------------------ pages

    def _clear_pages(self) -> None:
        self._type_pages.clear()
        while self.stack.count() > 0:
            w = self.stack.widget(0)
            self.stack.removeWidget(w)
            w.deleteLater()

    def set_allowed_types(self, types: List[str]) -> None:
        """Setzt erlaubte Path-Typen und baut die Parameter-Pages strikt aus Schema."""
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
            self.type_combo.setCurrentIndex(0)
            self._on_type_changed(0)

    def _build_page_for_type(
        self,
        ptype: str,
        params: Dict[str, Any],
    ) -> Tuple[QWidget, Dict[str, Tuple[QWidget, str, Dict[str, Any]]]]:
        """Erzeugt eine Parameter-Page für genau einen Path-Type."""
        page = QWidget(self)
        form = QFormLayout(page)
        _compact_form(form)

        sp_page = page.sizePolicy()
        sp_page.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp_page.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        page.setSizePolicy(sp_page)

        fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]] = {}

        keys = sorted(list(params.keys()), key=lambda k: (0 if k.endswith("shape") else 1, k))

        for key in keys:
            spec = dict(params.get(key) or {})
            t = str(spec.get("type", "")).strip().lower()

            w: Optional[QWidget] = None
            kind: str = ""
            row_label = key

            if t == "boolean":
                w = QCheckBox(key, page)
                kind = "check"
                row_label = ""
            elif t == "string":
                w = QLineEdit(page)
                kind = "string"
            elif t == "enum":
                w = QComboBox(page)
                kind = "combo"
                w.addItems([str(v) for v in (spec.get("values") or [])])
            elif t == "vec2":
                step_spec = spec.get("step", 0.1)
                if isinstance(step_spec, (list, tuple)):
                    step = float(step_spec[0] if step_spec else 0.1)
                else:
                    step = float(step_spec)

                unit = str(spec.get("unit", "") or "")
                w = Vec2Edit(step=step, unit=unit, parent=page)
                kind = "vec2"
            elif t == "number":
                step = float(spec.get("step", 1.0))
                minv = float(spec.get("min", 0.0))
                maxv = float(spec.get("max", 0.0))
                unit = str(spec.get("unit", "") or "")

                if _intish(step, minv, maxv):
                    sb = QSpinBox(page)
                    sb.setMinimum(int(minv))
                    sb.setMaximum(int(maxv))
                    sb.setSingleStep(int(step))
                    w = sb
                    kind = "int"
                else:
                    sb = QDoubleSpinBox(page)
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
                    w.setSuffix(suf)
            else:
                # unbekannter Typ -> ignoriere (strikt)
                continue

            assert w is not None

            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

            if spec.get("help") and hasattr(w, "setToolTip"):
                w.setToolTip(str(spec["help"]))

            fields[key] = (w, kind, spec)
            form.addRow(row_label, w)

        self._wire_visibility(fields, form=form)
        return page, fields

    # ------------------------------------------------------------------ visibility

    def _wire_visibility(
        self,
        fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]],
        *,
        form: QFormLayout,
    ) -> None:
        """
        visible_if:
          key:
            visible_if: { other_key: other_value }
        """
        deps: Dict[str, Tuple[str, Any]] = {}
        for key, (_w, _kind, spec) in fields.items():
            vcond = spec.get("visible_if")
            if isinstance(vcond, dict) and len(vcond) == 1:
                dep_key, dep_val = next(iter(vcond.items()))
                deps[key] = (str(dep_key), dep_val)

        def _value_of(widget: QWidget, kind: str) -> Any:
            if kind == "double":
                return float(getattr(widget, "value")())
            if kind == "int":
                return int(getattr(widget, "value")())
            if kind == "check":
                return bool(getattr(widget, "isChecked")())
            if kind == "combo":
                return str(getattr(widget, "currentText")())
            if kind == "string":
                return str(getattr(widget, "text")())
            if kind == "vec2" and isinstance(widget, Vec2Edit):
                return widget.get()
            return None

        def _form_row_for_widget(w: QWidget) -> int:
            idx = form.indexOf(w)
            if idx >= 0:
                row, _role = form.getItemPosition(idx)
                return row

            rc = form.rowCount()
            for row in range(rc):
                for role in (QFormLayout.ItemRole.LabelRole, QFormLayout.ItemRole.FieldRole):
                    it = form.itemAt(row, role)
                    ww = it.widget() if it is not None else None
                    if ww and (ww is w or ww.isAncestorOf(w) or w.isAncestorOf(ww)):
                        return row
            return -1

        def _set_row_visible(w: QWidget, visible: bool) -> None:
            row = _form_row_for_widget(w)
            if row < 0:
                w.setVisible(visible)
                return

            li = form.itemAt(row, QFormLayout.ItemRole.LabelRole)
            lw = li.widget() if li is not None else None
            if lw is not None:
                lw.setVisible(visible)

            fi = form.itemAt(row, QFormLayout.ItemRole.FieldRole)
            fw = fi.widget() if fi is not None else None
            if fw is not None:
                fw.setVisible(visible)

        def _apply_visibility() -> None:
            values: Dict[str, Any] = {k: _value_of(w, kind) for k, (w, kind, _) in fields.items()}
            for tgt, (dep_k, dep_v) in deps.items():
                w_tgt = fields[tgt][0]
                visible = (str(values.get(dep_k)) == str(dep_v))
                _set_row_visible(w_tgt, visible)
            self._lock_stack_height()

        # connect dependency sources
        for _tgt, (dep_key, _dep_v) in deps.items():
            if dep_key not in fields:
                continue
            w_dep, kind_dep, _spec_dep = fields[dep_key]
            if kind_dep == "combo":
                w_dep.currentTextChanged.connect(lambda _t: _apply_visibility())
            elif kind_dep in ("int", "double"):
                w_dep.valueChanged.connect(lambda _v: _apply_visibility())
            elif kind_dep == "check":
                w_dep.stateChanged.connect(lambda _v: _apply_visibility())
            elif kind_dep == "string":
                w_dep.textChanged.connect(lambda _v: _apply_visibility())
            elif kind_dep == "vec2" and isinstance(w_dep, Vec2Edit):
                w_dep.x.valueChanged.connect(lambda _v: _apply_visibility())
                w_dep.y.valueChanged.connect(lambda _v: _apply_visibility())

        _apply_visibility()

    # ------------------------------------------------------------------ defaults / collect

    def apply_default_path(self, path: Dict[str, Any]) -> None:
        """
        Schreibt Default-Parameter in UI.
        Wenn `_side_cfg` im dict enthalten ist, wird er übernommen.
        """
        if isinstance(path, dict) and "_side_cfg" in path:
            self._side_cfg = dict(path["_side_cfg"] or {})

        ptype = str(path.get("type") or "").strip()
        if ptype and self.type_combo.findText(ptype) >= 0:
            self.type_combo.setCurrentText(ptype)
        elif self.type_combo.count() > 0:
            self.type_combo.setCurrentIndex(0)

        self._on_type_changed(self.type_combo.currentIndex())

        ptype = self.type_combo.currentText().strip()
        _page, fields_map, _params = self._type_pages.get(ptype, (None, {}, {}))

        for key, (w, kind, spec) in fields_map.items():
            val = path[key] if key in path else spec.get("default", None)

            if kind == "check" and isinstance(w, QCheckBox):
                w.setChecked(bool(val))

            elif kind == "combo" and isinstance(w, QComboBox):
                if val is None:
                    continue
                idx = w.findText(str(val))
                w.setCurrentIndex(idx if idx >= 0 else (0 if w.count() > 0 else -1))

            elif kind == "string" and isinstance(w, QLineEdit):
                w.setText("" if val is None else str(val))

            elif kind == "int" and isinstance(w, QSpinBox):
                if val is None:
                    continue
                w.setValue(int(val))

            elif kind == "double" and isinstance(w, QDoubleSpinBox):
                if val is None:
                    continue
                w.setValue(float(val))

            elif kind == "vec2" and isinstance(w, Vec2Edit):
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    w.set([float(val[0]), float(val[1])])

        self._lock_stack_height()

    def collect_path(self) -> Dict[str, Any]:
        """UI → Dict für paths_by_side[side]."""
        ptype = self.type_combo.currentText().strip()
        out: Dict[str, Any] = {"type": ptype} if ptype else {}

        _page, fields_map, _params = self._type_pages.get(ptype, (None, {}, {}))
        for key, (w, kind, _spec) in fields_map.items():
            if kind == "double" and isinstance(w, QDoubleSpinBox):
                out[key] = float(w.value())
            elif kind == "int" and isinstance(w, QSpinBox):
                out[key] = int(w.value())
            elif kind == "check" and isinstance(w, QCheckBox):
                out[key] = bool(w.isChecked())
            elif kind == "combo" and isinstance(w, QComboBox):
                out[key] = str(w.currentText())
            elif kind == "string" and isinstance(w, QLineEdit):
                out[key] = str(w.text())
            elif kind == "vec2" and isinstance(w, Vec2Edit):
                out[key] = w.get()

        return out

    # ------------------------------------------------------------------ planner (per side)

    def apply_path_planner_model(self, cfg: Dict[str, Any] | None) -> None:
        """Recipe.planner['path'][side_name] → UI."""
        self.pathPlanner.apply_planner_model(cfg or {})

    def collect_path_planner(self) -> Dict[str, Any]:
        """UI → Planner-Config für diese Side."""
        return self.pathPlanner.collect_planner()

    # ------------------------------------------------------------------ auto-reset (defaults only)

    def enable_auto_reset(self, rec_def: Dict[str, Any], side_name: str) -> None:
        """
        Aktiviert Auto-Reset der Path-Defaults bei Type-Wechsel.
        (Keine Legacy/Fallback-Keys – strikt aus store/runtime-cfg.)
        """
        self._side_cfg = self.store.build_side_runtime_cfg_strict(rec_def, side_name)

        def _reload_defaults() -> None:
            side_default = dict((self._side_cfg.get("default_path") or {}))
            cur_t = self.type_combo.currentText().strip()
            if cur_t:
                side_default["type"] = cur_t
            self.apply_default_path(side_default)

        self.type_combo.currentTextChanged.connect(lambda _t: _reload_defaults())

    # ------------------------------------------------------------------ intern

    def _on_type_changed(self, _idx: int) -> None:
        idx = self.type_combo.currentIndex()
        if idx < 0:
            return
        if 0 <= idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
            self._lock_stack_height()

    def _lock_stack_height(self) -> None:
        """
        Fixiert die Stack-Höhe auf die aktuelle Page, damit das Layout stabil bleibt
        (insbesondere bei visible_if).
        """
        w = self.stack.currentWidget()
        if w is not None:
            self.stack.setMaximumHeight(w.sizeHint().height())
        self.stack.adjustSize()
        self.adjustSize()
