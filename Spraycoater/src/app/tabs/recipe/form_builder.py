# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Optional, Callable

from PyQt5.QtWidgets import (
    QFormLayout, QLabel, QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox,
    QHBoxLayout, QWidget
)


# -------- helpers --------

def clear_form(form: QFormLayout) -> None:
    """Entfernt alle Zeilen des Formulars sauber."""
    while form.rowCount():
        # removeRow räumt intern die Widgets ab; explizites deleteLater nicht nötig
        form.removeRow(0)


def _read_nested(d: dict, path):
    cur = d
    for p in path:
        if not isinstance(cur, dict) or p not in cur:
            return None
        cur = cur[p]
    return cur


def _connect_value_change(widget: Any, cb: Callable[[], None]) -> None:
    """Bindet cb an 'Wert hat sich geändert' für verschiedene Widgettypen."""
    if widget is None:
        return
    if isinstance(widget, QDoubleSpinBox):
        widget.valueChanged.connect(cb)
    elif isinstance(widget, QSpinBox):
        widget.valueChanged.connect(cb)
    elif isinstance(widget, QCheckBox):
        widget.toggled.connect(cb)
    elif isinstance(widget, QComboBox):
        widget.currentTextChanged.connect(cb)
    elif isinstance(widget, tuple) and len(widget) == 2 and all(hasattr(w, "valueChanged") for w in widget):
        widget[0].valueChanged.connect(cb)
        widget[1].valueChanged.connect(cb)


def _make_widget_for_spec(
    full_key: str,
    sch: dict,
    param_widgets: dict,
) -> QWidget | tuple:
    """
    Erzeugt ein Eingabe-Widget gem. Schema.
    Registriert das Widget unter 'full_key' in param_widgets.
    Unterstützte Typen: number, integer, boolean, enum, vec2
    """
    t = sch.get("type")
    unit = sch.get("unit", "")
    read_only = bool(sch.get("read_only", False))

    # number (float)
    if t == "number":
        sb = QDoubleSpinBox()
        sb.setDecimals(int(sch.get("decimals", 3)))
        sb.setMinimum(float(sch.get("min", -1e9)))
        sb.setMaximum(float(sch.get("max",  1e9)))
        sb.setSingleStep(float(sch.get("step", 0.1)))
        if "default" in sch:
            sb.setValue(float(sch["default"]))
        if unit:
            sb.setSuffix(f" {unit}")
        sb.setReadOnly(read_only)
        sb.setEnabled(not read_only)
        param_widgets[full_key] = sb
        return sb

    # integer
    if t == "integer":
        sb = QSpinBox()
        sb.setMinimum(int(sch.get("min", -1_000_000_000)))
        sb.setMaximum(int(sch.get("max",  1_000_000_000)))
        sb.setSingleStep(int(sch.get("step", 1)))
        if "default" in sch:
            sb.setValue(int(sch["default"]))
        # Einheit bei QSpinBox nicht üblich; wenn gewünscht, per QLabel lösen
        sb.setReadOnly(read_only)
        sb.setEnabled(not read_only)
        param_widgets[full_key] = sb
        return sb

    # boolean
    if t == "boolean":
        cb = QCheckBox()
        cb.setChecked(bool(sch.get("default", False)))
        cb.setEnabled(not read_only)
        param_widgets[full_key] = cb
        return cb

    # enum (string values)
    if t == "enum":
        cmb = QComboBox()
        for v in sch.get("values", []):
            cmb.addItem(str(v))
        if "default" in sch:
            idx = cmb.findText(str(sch["default"]))
            if idx >= 0:
                cmb.setCurrentIndex(idx)
        cmb.setEnabled(not read_only)
        param_widgets[full_key] = cmb
        return cmb

    # vec2 (zwei Doubles nebeneinander)
    if t == "vec2":
        w = QWidget()
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        mins  = sch.get("min",  [-1e9, -1e9])
        maxs  = sch.get("max",  [ 1e9,  1e9])
        steps = sch.get("step", [0.1, 0.1])
        defs  = sch.get("default", [0.0, 0.0])
        s1, s2 = QDoubleSpinBox(), QDoubleSpinBox()
        for sb, mn, mx, st, dv in (
            (s1, mins[0], maxs[0], steps[0], defs[0]),
            (s2, mins[1], maxs[1], steps[1], defs[1]),
        ):
            sb.setDecimals(int(sch.get("decimals", 3)))
            sb.setMinimum(float(mn)); sb.setMaximum(float(mx)); sb.setSingleStep(float(st)); sb.setValue(float(dv))
            if unit:
                sb.setSuffix(f" {unit}")
            sb.setReadOnly(read_only)
            sb.setEnabled(not read_only)
            lay.addWidget(sb)
        param_widgets[full_key] = (s1, s2)
        return w

    # Fallback
    lab = QLabel("(unsupported)")
    lab.setEnabled(False)
    param_widgets[full_key] = lab
    return lab


def _connect_change_for_visibility(dep_widget, callback):
    """Bindet Sichtbarkeits-Update an Änderungen des abhängigen Widgets."""
    _connect_value_change(dep_widget, callback)


# -------- public API --------

def build_form_section(
    form: QFormLayout,
    spec: dict,
    *,
    prefix: str,
    param_widgets: dict,
    visibility_rules: dict,
    on_change: Optional[Callable[[], None]] = None,
) -> None:
    """
    Baut eine Form-Section gemäß spec (keys -> widgets)
    und registriert visible_if-Regeln. Optional: on_change-Callback
    bei jeder Werteänderung.
    """
    for key, sch in spec.items():
        full_key = f"{prefix}.{key}"

        # Label vorbereiten (mit optionalem override/tooltip)
        label_text = sch.get("label") or key.split(".")[-1].replace("_", " ")
        label = QLabel(label_text)
        if sch.get("tooltip"):
            label.setToolTip(str(sch["tooltip"]))

        # Widget bauen
        w = _make_widget_for_spec(full_key, sch, param_widgets)
        if sch.get("tooltip"):
            # Falls Widget tooltips soll
            if isinstance(w, tuple):
                for ww in w:
                    ww.setToolTip(str(sch["tooltip"]))
            else:
                w.setToolTip(str(sch["tooltip"]))

        # Zeile hinzufügen
        form.addRow(label, w)

        # on_change (Live-Updates, z.B. Preview)
        if on_change is not None:
            _connect_value_change(param_widgets[full_key], on_change)

        # Sichtbarkeitsregeln (visible_if: { "dep.key": value })
        vis = sch.get("visible_if")
        if isinstance(vis, dict) and len(vis) == 1:
            dep_key, dep_val = next(iter(vis.items()))
            dep_full = f"{prefix}.{dep_key}"
            visibility_rules[full_key] = {
                "on": (dep_full, dep_val),
                "label": label,
                "widget": w,
            }
            depw = param_widgets.get(dep_full)
            # wichtig: late-binding-Falle vermeiden
            def _cb(*_):
                update_visibility_all(param_widgets, visibility_rules)
            _connect_change_for_visibility(depw, _cb)


def apply_values(prefix: str, values: dict, param_widgets: dict) -> None:
    """
    Setzt Werte aus 'values' in die Widgets (anhand verschachtelter Keys).
    """
    for full_key, widget in param_widgets.items():
        if not full_key.startswith(prefix + "."):
            continue
        short = full_key[len(prefix) + 1:]
        val = _read_nested(values, short.split(".")) if isinstance(values, dict) else None
        if val is None:
            continue

        if isinstance(widget, QDoubleSpinBox):
            widget.setValue(float(val))
        elif isinstance(widget, QSpinBox):
            widget.setValue(int(val))
        elif isinstance(widget, QCheckBox):
            widget.setChecked(bool(val))
        elif isinstance(widget, QComboBox):
            sval = str(val)
            idx = widget.findText(sval)
            if idx < 0:
                # falls Wert nicht in Liste ist, füge temporär hinzu
                widget.addItem(sval)
                idx = widget.findText(sval)
            if idx >= 0:
                widget.setCurrentIndex(idx)
        elif isinstance(widget, tuple):
            # vec2
            if isinstance(val, (list, tuple)) and len(val) >= 2:
                widget[0].setValue(float(val[0]))
                widget[1].setValue(float(val[1]))


def update_visibility_all(param_widgets: dict, visibility_rules: dict) -> None:
    """
    Aktualisiert die Sichtbarkeit aller Felder mit visible_if-Regeln.
    Blendet **Label und Widget** gemeinsam ein/aus.
    """
    for full_key, rule in visibility_rules.items():
        dep_key, expected = rule["on"]
        depw = param_widgets.get(dep_key)

        # aktuellen Wert des Abhängigkeits-Widgets bestimmen
        cur = None
        if isinstance(depw, QDoubleSpinBox):
            cur = depw.value()
        elif isinstance(depw, QSpinBox):
            cur = depw.value()
        elif isinstance(depw, QCheckBox):
            cur = depw.isChecked()
        elif isinstance(depw, QComboBox):
            cur = depw.currentText()
        elif isinstance(depw, tuple):
            cur = (depw[0].value(), depw[1].value())

        visible = (cur == expected)

        # Label & Widget sichtbar/unsichtbar schalten
        label_widget = rule.get("label")
        value_widget = rule.get("widget")

        if label_widget is not None:
            label_widget.setVisible(visible)

        if isinstance(value_widget, tuple):
            # vec2-Container sichtbar schalten
            container = value_widget[0].parentWidget()
            if container is not None:
                container.setVisible(visible)
        elif hasattr(value_widget, "setVisible"):
            value_widget.setVisible(visible)
