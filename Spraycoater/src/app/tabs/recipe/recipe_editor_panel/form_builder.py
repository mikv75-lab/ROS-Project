# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Tuple
from PyQt5 import QtWidgets, QtCore

# ---------- Public API ----------

def clear_form(container: QtWidgets.QWidget) -> None:
    layout = container.layout()
    if not layout:
        return
    while layout.count():
        item = layout.takeAt(0)
        w = item.widget()
        if w:
            w.deleteLater()

def build_form_section(
    container: QtWidgets.QWidget,
    spec: Dict[str, Any],
    *,
    prefix: str,
    param_widgets: Dict[str, Any],
    visibility_rules: Dict[str, Dict[str, Any]],
) -> None:
    """
    Unterstützte Typen: number, boolean, enum, vec2
    Für vec2 sind min/max/step Listen mit 2 Werten.
    Sichtbarkeitsregeln: 'visible_if': { '<andere.key>': <wert> }
    """
    layout = container.layout()
    if layout is None:
        layout = QtWidgets.QFormLayout(container)

    for key, sch in spec.items():
        full_key = f"{prefix}.{key}" if prefix else key
        w = _make_widget_for_spec(full_key, sch, param_widgets)
        if w is None:
            continue
        label = sch.get("label", key.split(".")[-1])
        layout.addRow(label, _as_widget(w))

        # Sichtbarkeitsregeln speichern
        if "visible_if" in sch and isinstance(sch["visible_if"], dict):
            visibility_rules[full_key] = {
                "if": dict(sch["visible_if"]),
                "widget": _as_widget(w),
            }

def apply_values(prefix: str, values: Dict[str, Any], param_widgets: Dict[str, Any]) -> None:
    for dotted, w in param_widgets.items():
        if not dotted.startswith(prefix + "."):
            continue
        # hole value aus values anhand "prefix.x.y"
        rel = dotted[len(prefix)+1:]
        val = _dict_get(values, rel)
        if val is None:
            continue
        _set_widget_value(w, val)

def update_visibility_all(param_widgets: Dict[str, Any], visibility_rules: Dict[str, Dict[str, Any]]) -> None:
    # map: key -> widget/value-getter
    def current_value(dotted_key: str):
        w = param_widgets.get(dotted_key)
        return _get_widget_value(w)

    for target_key, rule in visibility_rules.items():
        cond = rule.get("if", {})
        visible = True
        for dep_key, expected in cond.items():
            val = current_value(dep_key)
            if val != expected:
                visible = False
                break
        widget = rule.get("widget")
        if widget is not None:
            widget.setVisible(visible)

# ---------- Helpers ----------

def _make_widget_for_spec(full_key: str, sch: Dict[str, Any], param_widgets: Dict[str, Any]):
    t = (sch.get("type") or "").strip().lower()

    # number
    if t == "number":
        sb = QtWidgets.QDoubleSpinBox()
        sb.setDecimals(4)
        if "min" in sch:   sb.setMinimum(float(sch["min"]))
        if "max" in sch:   sb.setMaximum(float(sch["max"]))
        if "step" in sch:  sb.setSingleStep(float(sch["step"]))
        if "default" in sch: sb.setValue(float(sch["default"]))
        if "unit" in sch:
            sb.setSuffix(" " + str(sch["unit"]))
        param_widgets[full_key] = sb
        return sb

    # boolean
    if t == "boolean":
        cb = QtWidgets.QCheckBox()
        cb.setChecked(bool(sch.get("default", False)))
        param_widgets[full_key] = cb
        return cb

    # enum
    if t == "enum":
        combo = QtWidgets.QComboBox()
        vals = list(sch.get("values") or [])
        for v in vals:
            combo.addItem(str(v))
        if "default" in sch:
            idx = combo.findText(str(sch["default"]))
            combo.setCurrentIndex(0 if idx < 0 else idx)
        param_widgets[full_key] = combo
        return combo

    # vec2
    if t == "vec2":
        minv = sch.get("min", [0.0, 0.0])
        maxv = sch.get("max", [1e9, 1e9])
        step = sch.get("step", [0.1, 0.1])
        default = sch.get("default", [0.0, 0.0])

        sbx = QtWidgets.QDoubleSpinBox()
        sby = QtWidgets.QDoubleSpinBox()
        for sb, mn, mx, stp, dv in (
            (sbx, float(minv[0]), float(maxv[0]), float(step[0]), float(default[0])),
            (sby, float(minv[1]), float(maxv[1]), float(step[1]), float(default[1])),
        ):
            sb.setDecimals(4)
            sb.setMinimum(mn); sb.setMaximum(mx); sb.setSingleStep(stp); sb.setValue(dv)

        if "unit" in sch:
            # als Tooltip, damit UI kompakt bleibt
            sbx.setToolTip(str(sch["unit"]))
            sby.setToolTip(str(sch["unit"]))

        wrap = QtWidgets.QWidget()
        hl = QtWidgets.QHBoxLayout(wrap); hl.setContentsMargins(0,0,0,0)
        hl.addWidget(sbx); hl.addWidget(sby)

        # speichern als Tuple (Roh-Widgets)
        param_widgets[full_key] = (sbx, sby)
        return wrap

    # Fallback: nicht unterstützt
    return None

def _as_widget(w) -> QtWidgets.QWidget:
    if isinstance(w, QtWidgets.QWidget):
        return w
    if isinstance(w, tuple):
        wrap = QtWidgets.QWidget()
        hl = QtWidgets.QHBoxLayout(wrap); hl.setContentsMargins(0,0,0,0)
        for sb in w:
            hl.addWidget(sb)
        return wrap
    return QtWidgets.QLabel("<unsupported>")

def _set_widget_value(w, value):
    from PyQt5 import QtWidgets  # type: ignore
    if isinstance(w, QtWidgets.QDoubleSpinBox):
        try:
            w.setValue(float(value))
        except Exception:
            pass
        return
    if isinstance(w, QtWidgets.QCheckBox):
        w.setChecked(bool(value)); return
    if isinstance(w, QtWidgets.QComboBox):
        idx = w.findText(str(value))
        w.setCurrentIndex(0 if idx < 0 else idx); return
    if isinstance(w, tuple) and len(w) == 2:
        try:
            x, y = value
            w[0].setValue(float(x)); w[1].setValue(float(y))
        except Exception:
            pass
        return
    # unbekannt: ignorieren

def _get_widget_value(w):
    from PyQt5 import QtWidgets  # type: ignore
    if isinstance(w, QtWidgets.QDoubleSpinBox):
        return float(w.value())
    if isinstance(w, QtWidgets.QCheckBox):
        return bool(w.isChecked())
    if isinstance(w, QtWidgets.QComboBox):
        return str(w.currentText())
    if isinstance(w, tuple) and len(w) == 2:
        return [float(w[0].value()), float(w[1].value())]
    return None

def _dict_get(d: Dict[str, Any], dotted: str):
    cur = d
    for p in dotted.split("."):
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur
