# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QFormLayout, QLabel, QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox,
    QHBoxLayout, QWidget
)

# -------- helpers --------

def clear_form(form: QFormLayout) -> None:
    while form.rowCount():
        form.removeRow(0)

def _read_nested(d: dict, path):
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur

def _make_widget_for_spec(full_key: str, sch: dict, param_widgets: dict):
    t = sch.get("type")
    unit = sch.get("unit", "")

    if t == "number":
        sb = QDoubleSpinBox()
        sb.setDecimals(3)
        sb.setMinimum(float(sch.get("min", -1e9)))
        sb.setMaximum(float(sch.get("max",  1e9)))
        sb.setSingleStep(float(sch.get("step", 0.1)))
        if "default" in sch:
            sb.setValue(float(sch["default"]))
        if unit:
            sb.setSuffix(f" {unit}")
        param_widgets[full_key] = sb
        return sb

    if t == "boolean":
        cb = QCheckBox()
        cb.setChecked(bool(sch.get("default", False)))
        param_widgets[full_key] = cb
        return cb

    if t == "enum":
        cmb = QComboBox()
        for v in sch.get("values", []):
            cmb.addItem(str(v))
        if "default" in sch:
            idx = cmb.findText(str(sch["default"]))
            if idx >= 0:
                cmb.setCurrentIndex(idx)
        param_widgets[full_key] = cmb
        return cmb

    if t == "vec2":
        w = QWidget()
        lay = QHBoxLayout(w); lay.setContentsMargins(0,0,0,0)
        mins = sch.get("min",  [-1e9, -1e9])
        maxs = sch.get("max",  [ 1e9,  1e9])
        steps= sch.get("step", [0.1, 0.1])
        defs = sch.get("default", [0.0, 0.0])
        s1, s2 = QDoubleSpinBox(), QDoubleSpinBox()
        for sb, mn, mx, st, dv in ((s1, mins[0], maxs[0], steps[0], defs[0]),
                                    (s2, mins[1], maxs[1], steps[1], defs[1])):
            sb.setDecimals(3); sb.setMinimum(float(mn)); sb.setMaximum(float(mx)); sb.setSingleStep(float(st)); sb.setValue(float(dv))
            if unit: sb.setSuffix(f" {unit}")
            lay.addWidget(sb)
        param_widgets[full_key] = (s1, s2)
        return w

    lab = QLabel("(unsupported)")
    param_widgets[full_key] = lab
    return lab

def _connect_change_for_visibility(dep_widget, callback):
    if dep_widget is None:
        return
    if isinstance(dep_widget, (QDoubleSpinBox, QSpinBox)):
        dep_widget.valueChanged.connect(callback)
    elif isinstance(dep_widget, QCheckBox):
        dep_widget.toggled.connect(callback)
    elif isinstance(dep_widget, QComboBox):
        dep_widget.currentTextChanged.connect(callback)
    elif isinstance(dep_widget, tuple):
        dep_widget[0].valueChanged.connect(callback)
        dep_widget[1].valueChanged.connect(callback)

# -------- public API --------

def build_form_section(form: QFormLayout, spec: dict, *, prefix: str,
                       param_widgets: dict, visibility_rules: dict) -> None:
    """
    Baut eine Form-Section gemäß spec (keys -> widgets) und registriert visible_if-Regeln.
    """
    for key, sch in spec.items():
        full_key = f"{prefix}.{key}"
        w = _make_widget_for_spec(full_key, sch, param_widgets)
        label = QLabel(key.split(".")[-1].replace("_", " "))
        form.addRow(label, w)

        vis = sch.get("visible_if")
        if isinstance(vis, dict) and len(vis) == 1:
            dep_key, dep_val = next(iter(vis.items()))
            dep_full = f"{prefix}.{dep_key}"
            visibility_rules[full_key] = {"on": (dep_full, dep_val)}
            depw = param_widgets.get(dep_full)
            _connect_change_for_visibility(depw, lambda *_: update_visibility_all(param_widgets, visibility_rules))

def apply_values(prefix: str, values: dict, param_widgets: dict) -> None:
    """
    Setzt Werte aus 'values' in die Widgets (anhand verschachtelter Keys).
    """
    for full_key, widget in param_widgets.items():
        if not full_key.startswith(prefix + "."):
            continue
        short = full_key[len(prefix) + 1:]
        val = _read_nested(values, short.split("."))
        if val is None:
            continue

        if isinstance(widget, (QDoubleSpinBox, QSpinBox)):
            widget.setValue(float(val))
        elif isinstance(widget, QCheckBox):
            widget.setChecked(bool(val))
        elif isinstance(widget, QComboBox):
            idx = widget.findText(str(val))
            if idx >= 0:
                widget.setCurrentIndex(idx)
        elif isinstance(widget, tuple):
            if isinstance(val, (list, tuple)) and len(val) >= 2:
                widget[0].setValue(float(val[0]))
                widget[1].setValue(float(val[1]))

def update_visibility_all(param_widgets: dict, visibility_rules: dict) -> None:
    """
    Aktualisiert die Sichtbarkeit aller Felder mit visible_if-Regeln.
    """
    for full_key, rule in visibility_rules.items():
        dep_key, expected = rule["on"]
        depw = param_widgets.get(dep_key)
        cur = None
        if isinstance(depw, (QDoubleSpinBox, QSpinBox)):
            cur = depw.value()
        elif hasattr(depw, "isChecked"):
            cur = depw.isChecked()
        elif hasattr(depw, "currentText"):
            cur = depw.currentText()
        elif isinstance(depw, tuple):
            cur = (depw[0].value(), depw[1].value())

        # Widget-Container aus Registry holen
        row_widget = param_widgets.get(full_key)
        container = None
        if isinstance(row_widget, tuple):
            container = row_widget[0].parentWidget()
        else:
            container = getattr(row_widget, "parentWidget", lambda: None)()
        if container is not None:
            container.setVisible(cur == expected)
