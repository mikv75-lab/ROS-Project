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
    """True, wenn alle Werte integer-kompatibel sind (z. B. step/min/max)."""
    try:
        return all(float(v).is_integer() for v in vals)
    except:
        return False


class SidePathEditor(QWidget):
    """
    Editor für eine Seite. Baut sich VOLLSTÄNDIG dynamisch aus dem RecipeStore auf.
    
    1. Holt erlaubte Typen aus recipe_catalog.yaml (für das aktuelle Rezept-Template).
    2. Holt Parameter-Schema aus recipe_params.yaml (min/max/defaults).
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

        self._side_cfg: Dict[str, Any] = {}
        # ptype -> (page_widget, fields_map={key: (widget, kind, spec)}, params_schema)
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
        """
        Initialisiert den Editor basierend auf dem Template (rec_def) und dem Store.
        """
        side_cfg = self.store.allowed_and_default_for(rec_def, side)
        
        # Erlaubte Typen laden
        allowed = side_cfg.get("allowed_path_types")
        if not isinstance(allowed, list) or not allowed:
            # Fallback oder Fehler. Hier Fehler, da Config kaputt wäre.
            raise KeyError(f"Side '{side}': allowed_path_types fehlt/leer.")

        allowed_types = [str(x).strip() for x in allowed if str(x).strip()]

        default_path = side_cfg.get("default_path") or {}
        
        # Schemas für alle erlaubten Typen aus dem Store laden (recipe_params.yaml)
        schemas: Dict[str, Any] = {}
        for ptype in allowed_types:
            schemas[ptype] = dict(self.store.schema_for_type_strict(ptype) or {})

        self._side_cfg = {
            "allowed_types": allowed_types,
            "default_path": dict(default_path),
            "schemas": schemas,
        }

        # Dropdown & Pages aufbauen
        self.set_allowed_types(allowed_types)

        # Default anwenden (nur initial, wird später durch Model überschrieben)
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
            # Schema für diesen Typ holen
            params_schema = dict(schemas.get(ptype) or {})
            
            # Seite dynamisch generieren
            page, fields_map = self._build_page_for_type(ptype, params_schema)
            
            self._type_pages[ptype] = (page, fields_map, params_schema)
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
    ) -> Tuple[QWidget, Dict[str, Tuple[QWidget, str, Dict[str, Any]]]]:
        """
        Baut die Eingabemaske basierend auf der YAML-Definition.
        Nutzt min/max/step/default/description aus dem Schema.
        """
        page = QWidget(self)
        form = QFormLayout(page)
        _compact_form(form)

        # Sortierung: Geometrie oben, Details unten
        def sort_key(k: str) -> tuple:
            k = str(k)
            prio = 50
            if k.startswith("area."): prio = 10
            elif "radius" in k: prio = 11
            elif "size" in k: prio = 12
            elif "pitch" in k: prio = 20
            elif "loops" in k: prio = 21
            elif "direction" in k: prio = 30
            elif "angle" in k and "predispense" not in k and "retreat" not in k: prio = 31
            elif "start" in k: prio = 32
            elif "boustrophedon" in k: prio = 33
            elif "margin" in k or "offset" in k: prio = 40
            elif "z_mm" in k or "height" in k: prio = 41
            elif "predispense" in k: prio = 90
            elif "retreat" in k: prio = 91
            return (prio, k)

        keys = sorted(list(params_schema.keys()), key=sort_key)
        fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]] = {}

        for key in keys:
            spec = dict(params_schema.get(key) or {})
            t = str(spec.get("type", "")).strip().lower()
            
            # Defaults und Limits aus YAML
            default_val = spec.get("default")
            desc_text = str(spec.get("description") or "")
            
            w: Optional[QWidget] = None
            kind: str = ""
            row_label = str(key)

            if t == "boolean":
                w = QCheckBox(str(key), page)
                # Default setzten, falls im Schema definiert
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
                    if idx >= 0: w.setCurrentIndex(idx)
                    
            elif t == "vec2":
                step_spec = spec.get("step", 0.1)
                step = float(step_spec[0] if isinstance(step_spec, (list, tuple)) and step_spec else step_spec)
                unit = str(spec.get("unit", "") or "")
                w = Vec2Edit(step=step, unit=unit, parent=page)
                
                # Min/Max für Vec2 aus YAML lesen (falls vorhanden)
                min_spec = spec.get("min")
                max_spec = spec.get("max")
                if isinstance(min_spec, (list, tuple)) and isinstance(max_spec, (list, tuple)):
                     w.set_limits(float(min_spec[0]), float(max_spec[0]))
                
                if default_val is not None and isinstance(default_val, (list, tuple)):
                    w.set(default_val)
                    
                kind = "vec2"
                
            elif t == "number":
                step = float(spec.get("step", 1.0))
                minv = float(spec.get("min", 0.0))
                maxv = float(spec.get("max", 9999.0))
                unit = str(spec.get("unit", "") or "")

                # Entscheidung Int vs Double SpinBox
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
                    
                    # Decimals aus step ableiten (0.1 -> 1, 0.05 -> 2)
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
                continue # Unbekannter Typ

            assert w is not None

            # Layout Policy
            w.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

            # Tooltip aus YAML description
            if desc_text:
                w.setToolTip(desc_text)
                
            fields[str(key)] = (w, kind, spec)
            form.addRow(row_label, w)

        self._wire_visibility(fields, form=form)
        return page, fields

    # ------------------------------------------------------------------ visibility logic

    def _wire_visibility(self, fields: Dict[str, Tuple[QWidget, str, Dict[str, Any]]], *, form: QFormLayout) -> None:
        deps: Dict[str, Tuple[str, Any]] = {}
        for key, (_w, _kind, spec) in fields.items():
            vcond = spec.get("visible_if")
            if isinstance(vcond, dict) and len(vcond) == 1:
                dep_key, dep_val = next(iter(vcond.items()))
                deps[key] = (str(dep_key), dep_val)

        def _get_val(k):
            w, kind, _ = fields[k]
            if kind == "check": return w.isChecked()
            if kind == "combo": return w.currentText()
            if kind == "int": return w.value()
            if kind == "double": return w.value()
            if kind == "string": return w.text()
            if kind == "vec2": return w.get()
            return None

        def _update():
            # Aktuelle Werte holen
            vals = {k: _get_val(k) for k in fields}
            for target_key, (src_key, expected_val) in deps.items():
                if src_key not in vals: continue
                
                # Sichtbarkeit prüfen (String-Vergleich für Robustheit bei Enums/Bools)
                is_visible = (str(vals[src_key]) == str(expected_val))
                
                # Row im FormLayout finden und verstecken/zeigen
                w_target = fields[target_key][0]
                idx = form.indexOf(w_target)
                if idx >= 0:
                    row, _ = form.getItemPosition(idx)
                    form.setRowVisible(row, is_visible)

            self._lock_stack_height()

        # Listener anhängen
        for target_key, (src_key, _) in deps.items():
            if src_key not in fields: continue
            w_src, kind_src, _ = fields[src_key]
            
            if kind_src == "combo": w_src.currentTextChanged.connect(lambda _: _update())
            elif kind_src == "check": w_src.stateChanged.connect(lambda _: _update())
            elif kind_src in ("int", "double"): w_src.valueChanged.connect(lambda _: _update())
            elif kind_src == "string": w_src.textChanged.connect(lambda _: _update())

        # Initial update
        _update()

    # ------------------------------------------------------------------ Values Apply / Collect

    def apply_default_path(self, path: Dict[str, Any]) -> None:
        """
        Lädt Werte in die UI.
        
        WICHTIG:
        Wenn das `path` Dictionary einen Wert NICHT enthält (weil es z.B. eine alte Datei ist),
        dann nimmt er AUTOMATISCH den Default aus dem Schema (`spec.get("default")`),
        da wir die Widgets ja initial mit den Defaults gebaut haben.
        """
        if isinstance(path, dict) and "_side_cfg" in path:
            new_cfg = dict(path["_side_cfg"] or {})
            if new_cfg != self._side_cfg:
                self._side_cfg = new_cfg
                allowed_types = [str(t).strip() for t in (self._side_cfg.get("allowed_types") or []) if str(t).strip()]
                self.set_allowed_types(allowed_types)

        ptype = str(path.get("type") or "").strip()
        if not ptype:
            # Fallback auf ersten erlaubten Typen
            allowed = self._side_cfg.get("allowed_types", [])
            if allowed:
                ptype = allowed[0]
            else:
                return # Should not happen

        # Combo setzen
        idx = self.type_combo.findText(ptype)
        if idx >= 0:
            with QSignalBlocker(self.type_combo):
                self.type_combo.setCurrentIndex(idx)
            self._on_type_changed(idx)
        
        # Werte setzen
        _page, fields_map, _params = self._type_pages.get(ptype, (None, {}, {}))
        
        for key, (w, kind, spec) in fields_map.items():
            # HIER ist die Magie: Wenn key in path, nimm ihn. Sonst nimm Default aus Schema.
            val = path.get(key, spec.get("default"))

            if kind == "check" and isinstance(w, QCheckBox):
                w.setChecked(bool(val))
            
            elif kind == "combo" and isinstance(w, QComboBox):
                if val is not None:
                    cidx = w.findText(str(val))
                    if cidx >= 0: w.setCurrentIndex(cidx)
            
            elif kind == "int" and isinstance(w, QSpinBox):
                if val is not None: w.setValue(int(val))
            
            elif kind == "double" and isinstance(w, QDoubleSpinBox):
                if val is not None: w.setValue(float(val))
            
            elif kind == "string" and isinstance(w, QLineEdit):
                if val is not None: w.setText(str(val))

            elif kind == "vec2" and isinstance(w, Vec2Edit):
                if isinstance(val, (list, tuple)) and len(val) >= 2:
                    w.set([float(val[0]), float(val[1])])
        
        self._lock_stack_height()

    def collect_path(self) -> Dict[str, Any]:
        """Liest Werte aus der UI aus."""
        ptype = self.type_combo.currentText().strip()
        out: Dict[str, Any] = {"type": ptype}

        _page, fields_map, _params = self._type_pages.get(ptype, (None, {}, {}))
        
        for key, (w, kind, _) in fields_map.items():
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
            elif kind == "vec2":
                out[key] = w.get()

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
        if idx < 0: return
        if 0 <= idx < self.stack.count():
            self.stack.setCurrentIndex(idx)
            self._lock_stack_height()

    def _lock_stack_height(self) -> None:
        w = self.stack.currentWidget()
        if w is not None:
            self.stack.setMaximumHeight(w.sizeHint().height())
        self.stack.adjustSize()
        self.adjustSize()