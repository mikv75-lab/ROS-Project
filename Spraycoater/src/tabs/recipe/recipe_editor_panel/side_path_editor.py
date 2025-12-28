# app/tabs/recipe/recipe_editor_panel/side_path_editor.py
# -*- coding: utf-8 -*-
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

from model.recipe.recipe_store import RecipeStore
from widgets.planner_groupbox import PlannerGroupBox


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

    SSoT (strict):
      - allowed types / default_path kommen aus RecipeStore.allowed_and_default_for(rec_def, side)
      - schemas kommen strikt aus RecipeStore.schema_for_type_strict(ptype)
      - YAML-Key: allowed_path_types (nicht allowed_types)
    """

    def __init__(
        self,
        *,
        side_name: str,
        store: RecipeStore,
        ctx: Any | None = None,
        parent: Optional[QWidget] = None,
    ):
        super().__init__(parent)
        self.side_name = side_name
        self.store = store
        self.ctx = ctx

        # contract:
        # {
        #   "allowed_types": [...],          # intern normalisiert
        #   "default_path": {...},
        #   "schemas": { ptype: {param: spec, ...}, ... }
        # }
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

    # ------------------------------------------------------------------ strict SSoT init

    def enable_auto_reset(self, rec_def: Dict[str, Any], side: str) -> None:
        """
        Strict SSoT:
          - liest side_cfg aus store.allowed_and_default_for(rec_def, side)
          - erwartet YAML-Key: allowed_path_types (list[str])
          - baut schemas strikt via store.schema_for_type_strict(ptype)
          - setzt allowed types + default_path in UI

        Wirft KeyError, wenn irgendetwas fehlt/inkonsistent ist.
        """
        side_cfg = self.store.allowed_and_default_for(rec_def, side)
        if not isinstance(side_cfg, dict):
            raise TypeError(f"Side '{side}': sides[{side}] ist kein dict (got {type(side_cfg)}).")

        allowed = side_cfg.get("allowed_path_types")
        if not isinstance(allowed, list) or not [x for x in allowed if str(x).strip()]:
            raise KeyError(f"Side '{side}': allowed_path_types fehlt/leer in recipe_catalog.sides")

        allowed_types = [str(x).strip() for x in allowed if str(x).strip()]

        default_path = side_cfg.get("default_path")
        if not isinstance(default_path, dict):
            raise KeyError(f"Side '{side}': default_path fehlt oder ist kein dict.")
        if "type" not in default_path or not str(default_path.get("type") or "").strip():
            raise KeyError(f"Side '{side}': default_path.type fehlt/leer.")

        # build schemas strictly for each allowed type
        schemas: Dict[str, Any] = {}
        for ptype in allowed_types:
            schemas[ptype] = dict(self.store.schema_for_type_strict(ptype) or {})

        self._side_cfg = {
            "allowed_types": allowed_types,
            "default_path": dict(default_path),
            "schemas": schemas,
        }

        # build pages strictly from schemas
        self.set_allowed_types(allowed_types)

        # apply default path (inject cfg so apply_default_path can rebuild if needed)
        dp = dict(default_path)
        dp["_side_cfg"] = dict(self._side_cfg)
        self.apply_default_path(dp)

    # ------------------------------------------------------------------ pages

    def _clear_pages(self) -> None:
        self._type_pages.clear()
        while self.stack.count() > 0:
            w = self.stack.widget(0)
            self.stack.removeWidget(w)
            w.deleteLater()

    def set_allowed_types(self, types: List[str]) -> None:
        """
        Strict:
          - schemas für alle types müssen in self._side_cfg["schemas"] existieren
          - keine stillen Fallbacks
        """
        allowed = [str(t).strip() for t in (types or []) if str(t).strip()]
        if not allowed:
            raise KeyError(f"Side '{self.side_name}': allowed types leer.")

        schemas = self._side_cfg.get("schemas")
        if not isinstance(schemas, dict):
            raise KeyError(f"Side '{self.side_name}': _side_cfg['schemas'] fehlt oder ist kein dict.")

        for ptype in allowed:
            if ptype not in schemas or not isinstance(schemas.get(ptype), dict) or not schemas.get(ptype):
                raise KeyError(
                    f"Side '{self.side_name}': Schema fehlt/leer für type '{ptype}' "
                    f"(erwartet via RecipeStore.schema_for_type_strict)."
                )

        with QSignalBlocker(self.type_combo):
            self.type_combo.clear()
            self.type_combo.addItems(allowed)

        self._clear_pages()

        for ptype in allowed:
            params = dict(schemas[ptype] or {})
            page, fields_map = self._build_page_for_type(ptype, params)
            self._type_pages[ptype] = (page, fields_map, params)
            self.stack.addWidget(page)

        with QSignalBlocker(self.type_combo):
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

        keys = sorted(list(params.keys()), key=lambda k: (0 if str(k).endswith("shape") else 1, str(k)))

        for key in keys:
            spec = dict(params.get(key) or {})
            t = str(spec.get("type", "")).strip().lower()

            w: Optional[QWidget] = None
            kind: str = ""
            row_label = str(key)

            if t == "boolean":
                w = QCheckBox(str(key), page)
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
                step = float(step_spec[0] if isinstance(step_spec, (list, tuple)) and step_spec else step_spec)
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
                # strict: ignore unknown types
                continue

            assert w is not None

            sp = w.sizePolicy()
            sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
            sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
            w.setSizePolicy(sp)

            if spec.get("help") and hasattr(w, "setToolTip"):
                w.setToolTip(str(spec["help"]))

            fields[str(key)] = (w, kind, spec)
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

        Wichtig:
          - Wenn sich _side_cfg ändert, müssen die Pages ggf. neu gebaut werden.
        """
        if isinstance(path, dict) and "_side_cfg" in path:
            new_cfg = dict(path["_side_cfg"] or {})
            cfg_changed = (new_cfg != self._side_cfg)
            self._side_cfg = new_cfg

            if cfg_changed:
                allowed_types = [str(t).strip() for t in (self._side_cfg.get("allowed_types") or []) if str(t).strip()]
                if not allowed_types:
                    raise KeyError(f"Side '{self.side_name}': _side_cfg.allowed_types fehlt/leer.")
                self.set_allowed_types(allowed_types)

        ptype = str(path.get("type") or "").strip()
        if not ptype:
            raise KeyError(f"Side '{self.side_name}': path.type fehlt/leer.")
        if self.type_combo.findText(ptype) < 0:
            raise KeyError(f"Side '{self.side_name}': type '{ptype}' ist nicht erlaubt.")

        with QSignalBlocker(self.type_combo):
            self.type_combo.setCurrentText(ptype)

        self._on_type_changed(self.type_combo.currentIndex())

        _page, fields_map, _params = self._type_pages.get(ptype, (None, {}, {}))
        if not fields_map:
            raise KeyError(f"Side '{self.side_name}': keine Felder für type '{ptype}' (schema leer?).")

        for key, (w, kind, spec) in fields_map.items():
            val = path[key] if key in path else spec.get("default", None)

            if kind == "check" and isinstance(w, QCheckBox):
                w.setChecked(bool(val))

            elif kind == "combo" and isinstance(w, QComboBox):
                if val is None:
                    continue
                idx = w.findText(str(val))
                if idx < 0:
                    raise KeyError(f"Side '{self.side_name}': ungültiger enum '{val}' für '{key}'.")
                w.setCurrentIndex(idx)

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
        if not ptype:
            raise KeyError(f"Side '{self.side_name}': current type leer.")
        out: Dict[str, Any] = {"type": ptype}

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
        if hasattr(self.pathPlanner, "apply_planner_model"):
            self.pathPlanner.apply_planner_model(cfg or {})
        else:
            self.pathPlanner.apply_model_to_ui(cfg or {})  # type: ignore[attr-defined]

    def collect_path_planner(self) -> Dict[str, Any]:
        """UI → Planner-Config für diese Side."""
        if hasattr(self.pathPlanner, "collect_planner"):
            return self.pathPlanner.collect_planner()
        tmp: Dict[str, Any] = {}
        self.pathPlanner.apply_ui_to_model(tmp)  # type: ignore[attr-defined]
        return tmp

    # ------------------------------------------------------------------ intern

    def _on_type_changed(self, _idx: int) -> None:
        idx = self.type_combo.currentIndex()
        if idx < 0:
            return
        if 0 <= idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
            self._lock_stack_height()

    def _lock_stack_height(self) -> None:
        """Fixiert die Stack-Höhe auf die aktuelle Page."""
        w = self.stack.currentWidget()
        if w is not None:
            self.stack.setMaximumHeight(w.sizeHint().height())
        self.stack.adjustSize()
        self.adjustSize()
