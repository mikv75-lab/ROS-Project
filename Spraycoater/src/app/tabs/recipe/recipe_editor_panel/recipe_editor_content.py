# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Tuple, List, Optional

from PyQt5 import QtWidgets
from PyQt5.QtWidgets import (
    QWidget, QGroupBox, QGridLayout, QVBoxLayout, QHBoxLayout,
    QFormLayout, QComboBox
)


class RecipeEditorContent(QWidget):
    """
    Eigenständiges QWidget, das den kompletten dynamischen Inhalt aufbaut:
      - GroupBox "Globals" (aus ctx.recipe_params['globals'])
      - GroupBox "Path" -> Type-Combo + QStackedWidget
          * für jede path.*.*-Sektion eine Page:
              - "Selection" (Tool/Substrate/Mount)
              - dynamisches Formular aus der Sektion

    Öffentliche API (wie der alte Builder):
      - apply_defaults()
      - apply_recipe_to_forms(recipe_dict)
      - set_active_by_type(rtype=..., rmode=...)
      - fill_selectors_for_recipe(recipe_like)
      - clear_active_selectors()
      - collect_globals() -> dict
      - collect_path_current() -> dict
      - active_selectors_values() -> (tool, substrate, mount)
      - update_visibility_all()

    Exponierte Widgets:
      - type_combo: QComboBox
      - stack: QStackedWidget
    """

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # Datenstrukturen
        self.specs: Dict[str, Any] = dict(self.ctx.recipe_params)
        self.param_widgets: Dict[str, Any] = {}
        self.visibility_rules: Dict[str, Dict[str, Any]] = {}
        self.index_to_spec: List[str] = []
        self.spec_to_selectors: Dict[str, Dict[str, QComboBox]] = {}

        # UI-Grundgerüst
        vroot = QVBoxLayout(self)
        vroot.setContentsMargins(0, 0, 0, 0)

        # ---- Globals ----------------------------------------------------
        gspec = self.specs.get("globals", {}) or {}
        grp_glob = QGroupBox("Globals", self)
        vroot.addWidget(grp_glob)
        v_glob = QVBoxLayout(grp_glob); v_glob.setContentsMargins(6, 6, 6, 6)
        w_form_globals = QWidget(grp_glob)
        v_glob.addWidget(w_form_globals)
        form_globals = QFormLayout(w_form_globals)

        self._build_form_section(
            container=w_form_globals, layout=form_globals, spec=gspec,
            prefix="globals", on_change=self.update_visibility_all
        )

        # ---- Path -------------------------------------------------------
        grp_path = QGroupBox("Path", self)
        vroot.addWidget(grp_path)
        v_path = QVBoxLayout(grp_path); v_path.setContentsMargins(6, 6, 6, 6)

        # Type-Row
        htype = QHBoxLayout(); v_path.addLayout(htype)
        lbl_type = QtWidgets.QLabel("Type:", grp_path)
        self.type_combo = QComboBox(grp_path)
        htype.addWidget(lbl_type); htype.addWidget(self.type_combo, 1)
        htype.addStretch(1)

        # Stacked
        self.stack = QtWidgets.QStackedWidget(grp_path)
        v_path.addWidget(self.stack, 1)

        # Pages erzeugen
        self._rebuild_pages()

        # Type ↔ Stack koppeln
        self.type_combo.currentIndexChanged.connect(lambda i: self.stack.setCurrentIndex(i))

        # Defaults in die Globals
        self.apply_defaults()
        # initiale Sichtbarkeit
        self.update_visibility_all()

    # ------------------------------------------------------------------ #
    # Build / Pages
    # ------------------------------------------------------------------ #
    def _rebuild_pages(self) -> None:
        # clean
        while self.stack.count():
            w = self.stack.widget(0)
            self.stack.removeWidget(w)
            w.deleteLater()

        self.index_to_spec.clear()
        self.spec_to_selectors.clear()

        # alle path.*.*-Sektionen sortiert
        for spec_key, spec in self._iter_path_specs_sorted():
            page = QWidget(self.stack)
            vpage = QVBoxLayout(page); vpage.setContentsMargins(0, 0, 0, 0)

            # Selection (Tool/Substrate/Mount)
            gb_sel = QGroupBox("Selection", page)
            grid = QGridLayout(gb_sel); grid.setContentsMargins(6, 6, 6, 6)
            cb_tool  = QComboBox(gb_sel)
            cb_sub   = QComboBox(gb_sel)
            cb_mount = QComboBox(gb_sel)
            grid.addWidget(QtWidgets.QLabel("Tool", gb_sel),       0, 0); grid.addWidget(cb_tool,  0, 1)
            grid.addWidget(QtWidgets.QLabel("Substrate", gb_sel),  1, 0); grid.addWidget(cb_sub,   1, 1)
            grid.addWidget(QtWidgets.QLabel("Mount", gb_sel),      2, 0); grid.addWidget(cb_mount, 2, 1)
            vpage.addWidget(gb_sel)

            # Formular der Section
            w_form = QWidget(page); vpage.addWidget(w_form)
            form = QFormLayout(w_form)
            self._build_form_section(
                container=w_form, layout=form, spec=spec,
                prefix="path", on_change=self.update_visibility_all
            )

            self.stack.addWidget(page)
            self.index_to_spec.append(spec_key)
            self.spec_to_selectors[spec_key] = {
                "tool": cb_tool, "substrate": cb_sub, "mount": cb_mount
            }

        # Type-Combo füllen
        self.type_combo.clear()
        for spec_key in self.index_to_spec:
            rtype, rmode = self._ptype_pmode_from_spec_key(spec_key)
            self.type_combo.addItem(self._type_label_for(rtype, rmode))
        if self.stack.count() > 0:
            self.stack.setCurrentIndex(0)
            self.type_combo.setCurrentIndex(0)

    # ------------------------------------------------------------------ #
    # Öffentliche API
    # ------------------------------------------------------------------ #
    def apply_defaults(self) -> None:
        defaults = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        self._apply_values("globals", defaults)

    def apply_recipe_to_forms(self, recipe: Dict[str, Any]) -> None:
        defaults = dict(self.ctx.recipes_yaml.get("spraycoater", {}).get("defaults", {}))
        gvals = dict(defaults); gvals.update(recipe.get("parameters", {}) or {})
        self._apply_values("globals", gvals)
        self._apply_values("path", dict(recipe.get("path") or {}))
        self.update_visibility_all()

    def set_active_by_type(self, *, rtype: str, rmode: str) -> None:
        spec_key = self._spec_key_for(rtype, rmode)
        if spec_key in self.index_to_spec:
            idx = self.index_to_spec.index(spec_key)
            self.stack.setCurrentIndex(idx)
            self.type_combo.setCurrentIndex(idx)

    def fill_selectors_for_recipe(self, recipe_like: Dict[str, Any]) -> None:
        if not self.index_to_spec or self.stack.count() == 0:
            return
        idx = self.stack.currentIndex()
        spec_key = self.index_to_spec[idx]
        self._fill_selectors_for_spec(spec_key, recipe_like)

    def clear_active_selectors(self) -> None:
        if not self.index_to_spec or self.stack.count() == 0:
            return
        idx = self.stack.currentIndex()
        spec_key = self.index_to_spec[idx]
        for nm in ("tool", "substrate", "mount"):
            cb = self.spec_to_selectors.get(spec_key, {}).get(nm)
            if cb:
                cb.clear()

    def collect_globals(self) -> Dict[str, Any]:
        res: Dict[str, Any] = {}
        for key in (self.specs.get("globals", {}) or {}).keys():
            w = self.param_widgets.get(f"globals.{key}")
            if not w:
                continue
            res[key] = self._get_widget_value(w)
        return res

    def collect_path_current(self) -> Dict[str, Any]:
        if not self.index_to_spec or self.stack.count() == 0:
            return {"type": "meander_plane"}
        spec_key = self.index_to_spec[self.stack.currentIndex()]
        ptype, pmode = self._ptype_pmode_from_spec_key(spec_key)
        out: Dict[str, Any] = {"type": f"{ptype}_{pmode}" if pmode else ptype}

        section = self.specs.get(spec_key, {}) or {}
        for k in section.keys():
            w = self.param_widgets.get(f"path.{k}")
            if not w:
                continue
            val = self._get_widget_value(w)
            # nested write
            cur = out
            parts = k.split(".")
            for p in parts[:-1]:
                cur = cur.setdefault(p, {})
            cur[parts[-1]] = val
        return out

    def active_selectors_values(self) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        if not self.index_to_spec or self.stack.count() == 0:
            return None, None, None
        spec_key = self.index_to_spec[self.stack.currentIndex()]
        sels = self.spec_to_selectors.get(spec_key) or {}
        tool = self._combo_text(sels.get("tool"))
        sub  = self._combo_text(sels.get("substrate"))
        mnt  = self._combo_text(sels.get("mount"))
        return tool, sub, mnt

    def update_visibility_all(self) -> None:
        def current_value(dotted_key: str):
            w = self.param_widgets.get(dotted_key)
            return self._get_widget_value(w)

        for target_key, rule in self.visibility_rules.items():
            cond = rule.get("if", {})
            visible = True
            for dep_key, expected in cond.items():
                if current_value(dep_key) != expected:
                    visible = False
                    break
            widget = rule.get("widget")
            if widget is not None:
                widget.setVisible(bool(visible))

    # ------------------------------------------------------------------ #
    # Build-Helpers
    # ------------------------------------------------------------------ #
    def _iter_path_specs_sorted(self) -> List[Tuple[str, Dict[str, Any]]]:
        items: List[Tuple[str, Dict[str, Any]]] = []
        for k, v in self.specs.items():
            if k.startswith("path.") and isinstance(v, dict):
                items.append((k, v))
        order = {
            "path.meander.plane": 0,
            "path.spiral.plane": 1,
            "path.spiral.cylinder": 2,
            "path.explicit": 3,
        }
        items.sort(key=lambda it: (order.get(it[0], 100), it[0]))
        return items

    def _build_form_section(
        self,
        *,
        container: QWidget,
        layout: QFormLayout,
        spec: Dict[str, Any],
        prefix: str,
        on_change=None,
    ) -> None:
        for key, sch in spec.items():
            full_key = f"{prefix}.{key}" if prefix else key
            w = self._make_widget_for_spec(full_key, sch)
            if w is None:
                continue
            label = sch.get("label", key.split(".")[-1])
            layout.addRow(label, self._as_widget(w))

            # sichtbarkeitsregeln
            if "visible_if" in sch and isinstance(sch["visible_if"], dict):
                self.visibility_rules[full_key] = {
                    "if": dict(sch["visible_if"]),
                    "widget": self._as_widget(w),
                }
            if on_change is not None:
                self._connect_change_signal(w, on_change)

    def _connect_change_signal(self, w, on_change):
        if isinstance(w, QtWidgets.QDoubleSpinBox) or isinstance(w, QtWidgets.QSpinBox):
            w.valueChanged.connect(lambda *_: on_change())
        elif isinstance(w, QtWidgets.QCheckBox):
            w.toggled.connect(lambda *_: on_change())
        elif isinstance(w, QtWidgets.QComboBox):
            w.currentIndexChanged.connect(lambda *_: on_change())
        elif isinstance(w, tuple) and len(w) == 2:
            for sb in w:
                sb.valueChanged.connect(lambda *_: on_change())

    def _apply_values(self, prefix: str, values: Dict[str, Any]) -> None:
        for dotted, w in self.param_widgets.items():
            if not dotted.startswith(prefix + "."):
                continue
            rel = dotted[len(prefix)+1:]
            val = self._dict_get(values, rel)
            if val is None:
                continue
            self._set_widget_value(w, val)

    def _fill_selectors_for_spec(self, spec_key: str, recipe_like: Dict[str, Any]) -> None:
        sels = self.spec_to_selectors.get(spec_key) or {}
        # tool
        cb = sels.get("tool")
        if cb:
            cb.clear()
            tool = recipe_like.get("tool")
            if tool:
                cb.addItem(str(tool)); cb.setCurrentIndex(0)
        # substrates
        subs = recipe_like.get("substrates") or []
        if not subs and recipe_like.get("substrate"):
            subs = [recipe_like.get("substrate")]
        cb = sels.get("substrate")
        if cb:
            cb.clear()
            for s in subs:
                cb.addItem(str(s))
            if cb.count() > 0:
                cb.setCurrentIndex(0)
        # mounts
        mts = recipe_like.get("substrate_mounts") or []
        if not mts and (recipe_like.get("substrate_mount") or recipe_like.get("mount")):
            mts = [recipe_like.get("substrate_mount") or recipe_like.get("mount")]
        cb = sels.get("mount")
        if cb:
            cb.clear()
            for m in mts:
                cb.addItem(str(m))
            if cb.count() > 0:
                cb.setCurrentIndex(0)

    # ------------------------------------------------------------------ #
    # Widget factory / values
    # ------------------------------------------------------------------ #
    def _make_widget_for_spec(self, full_key: str, sch: Dict[str, Any]):
        t = (sch.get("type") or "").strip().lower()

        # number -> int/double
        if t == "number":
            step = float(sch.get("step", 1))
            is_int = (
                float(sch.get("min", 0)).is_integer()
                and float(sch.get("max", 0)).is_integer()
                and step.is_integer()
            )
            if is_int:
                sb = QtWidgets.QSpinBox()
                sb.setMinimum(int(sch.get("min", 0)))
                sb.setMaximum(int(sch.get("max", 1000000)))
                sb.setSingleStep(int(step))
                if "default" in sch: sb.setValue(int(sch["default"]))
                if "unit" in sch: sb.setSuffix(" " + str(sch["unit"]))
                self.param_widgets[full_key] = sb
                return sb
            else:
                sb = QtWidgets.QDoubleSpinBox()
                sb.setDecimals(int(sch.get("decimals", 4)))
                sb.setMinimum(float(sch.get("min", 0)))
                sb.setMaximum(float(sch.get("max", 1e9)))
                sb.setSingleStep(float(step))
                if "default" in sch: sb.setValue(float(sch["default"]))
                if "unit" in sch: sb.setSuffix(" " + str(sch["unit"]))
                self.param_widgets[full_key] = sb
                return sb

        if t == "boolean":
            cb = QtWidgets.QCheckBox()
            cb.setChecked(bool(sch.get("default", False)))
            self.param_widgets[full_key] = cb
            return cb

        if t == "enum":
            combo = QtWidgets.QComboBox()
            if isinstance(sch.get("items"), list):
                for it in sch["items"]:
                    combo.addItem(str(it.get("label", it.get("value"))), it.get("value"))
                if "default" in sch:
                    for i in range(combo.count()):
                        if combo.itemData(i) == sch["default"]:
                            combo.setCurrentIndex(i); break
            else:
                for v in list(sch.get("values") or []):
                    combo.addItem(str(v))
                if "default" in sch:
                    idx = combo.findText(str(sch["default"]))
                    combo.setCurrentIndex(0 if idx < 0 else idx)
            self.param_widgets[full_key] = combo
            return combo

        if t == "vec2":
            minv = sch.get("min", [0.0, 0.0]); maxv = sch.get("max", [1e9, 1e9])
            step = sch.get("step", [0.1, 0.1]); default = sch.get("default", [0.0, 0.0])
            sbx = QtWidgets.QDoubleSpinBox(); sby = QtWidgets.QDoubleSpinBox()
            for sb, mn, mx, stp, dv, pref in (
                (sbx, float(minv[0]), float(maxv[0]), float(step[0]), float(default[0]), "x: "),
                (sby, float(minv[1]), float(maxv[1]), float(step[1]), float(default[1]), "y: "),
            ):
                sb.setDecimals(int(sch.get("decimals", 4)))
                sb.setMinimum(mn); sb.setMaximum(mx); sb.setSingleStep(stp); sb.setValue(dv)
                sb.setPrefix(pref)
                if "unit" in sch: sb.setSuffix(" " + str(sch["unit"]))
            wrap = QWidget()
            hl = QHBoxLayout(wrap); hl.setContentsMargins(0,0,0,0)
            hl.addWidget(sbx); hl.addWidget(sby)
            self.param_widgets[full_key] = (sbx, sby)
            return wrap

        return None

    @staticmethod
    def _as_widget(w) -> QWidget:
        if isinstance(w, QWidget): return w
        if isinstance(w, tuple):
            wrap = QWidget()
            hl = QHBoxLayout(wrap); hl.setContentsMargins(0,0,0,0)
            for sb in w: hl.addWidget(sb)
            return wrap
        return QtWidgets.QLabel("<unsupported>")

    @staticmethod
    def _set_widget_value(w, value):
        from PyQt5 import QtWidgets  # type: ignore
        if isinstance(w, QtWidgets.QSpinBox):
            try: w.setValue(int(value))
            except Exception: pass
            return
        if isinstance(w, QtWidgets.QDoubleSpinBox):
            try: w.setValue(float(value))
            except Exception: pass
            return
        if isinstance(w, QtWidgets.QCheckBox):
            w.setChecked(bool(value)); return
        if isinstance(w, QtWidgets.QComboBox):
            # match userData oder Text
            for i in range(w.count()):
                if w.itemData(i) == value or w.itemText(i) == str(value):
                    w.setCurrentIndex(i); return
            idx = w.findText(str(value))
            w.setCurrentIndex(0 if idx < 0 else idx); return
        if isinstance(w, tuple) and len(w) == 2:
            try:
                x, y = value
                w[0].setValue(float(x)); w[1].setValue(float(y))
            except Exception:
                pass
            return

    @staticmethod
    def _get_widget_value(w):
        from PyQt5 import QtWidgets  # type: ignore
        if isinstance(w, QtWidgets.QSpinBox): return int(w.value())
        if isinstance(w, QtWidgets.QDoubleSpinBox): return float(w.value())
        if isinstance(w, QtWidgets.QCheckBox): return bool(w.isChecked())
        if isinstance(w, QtWidgets.QComboBox):
            data = w.currentData()
            return data if data is not None else str(w.currentText())
        if isinstance(w, tuple) and len(w) == 2:
            return [float(w[0].value()), float(w[1].value())]
        return None

    @staticmethod
    def _dict_get(d: Dict[str, Any], dotted: str):
        cur = d
        for p in dotted.split("."):
            if not isinstance(cur, dict) or p not in cur:
                return None
            cur = cur[p]
        return cur

    @staticmethod
    def _ptype_pmode_from_spec_key(spec_key: str) -> Tuple[str, str]:
        parts = spec_key.split(".")
        return (parts[1], parts[2]) if len(parts) >= 3 else ("meander", "plane")

    @staticmethod
    def _spec_key_for(rtype: str, rmode: str) -> str:
        key = f"path.{rtype}"
        if rmode:
            key += f".{rmode}"
        return key

    @staticmethod
    def _type_label_for(rtype: str, rmode: str) -> str:
        if rtype == "meander" and rmode == "plane":    return "Meander (plane)"
        if rtype == "spiral"  and rmode == "plane":    return "Spiral (plane)"
        if rtype == "spiral"  and rmode == "cylinder": return "Spiral (cylinder)"
        if rtype == "explicit":                        return "Explicit"
        return "Meander (plane)"

    @staticmethod
    def _combo_text(cb: Optional[QComboBox]) -> Optional[str]:
        if cb and cb.currentText():
            return cb.currentText().strip()
        return None
