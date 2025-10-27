# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, Optional, Tuple

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGroupBox, QFormLayout, QLabel, QLineEdit,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QStackedWidget, QFrame,
    QScrollArea, QSpacerItem, QSizePolicy
)

from .side_path_editor import SidePathEditor


def _hline() -> QFrame:
    f = QFrame(); f.setFrameShape(QFrame.HLine); f.setFrameShadow(QFrame.Sunken); return f


class RecipeEditorContent(QWidget):
    """
    Layout (mit globaler ScrollArea):
      [ScrollArea]
        └─ [VBox]
           ├─ [Globals]  <-- enthält jetzt AUCH 'description' (Rezeptname)
           ├─ [recipe_stack]  <-- pro ausgewähltem Rezept eine Seite:
           │     - Selectors (tool/substrate/mount)
           │     - pro Side eine GroupBox mit SidePathEditor
           └─ (Expanding Spacer)
    """
    def __init__(self, *, ctx=None, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # Widgets, die pro Seite wechseln:
        self._side_editors: Dict[str, SidePathEditor] = {}
        self.sel_tool: Optional[QComboBox] = None
        self.sel_substrate: Optional[QComboBox] = None
        self.sel_mount: Optional[QComboBox] = None

        # Global widgets
        self.e_desc: Optional[QLineEdit] = None
        self.g_speed: Optional[QDoubleSpinBox] = None
        self.g_standoff: Optional[QDoubleSpinBox] = None
        self.g_angle: Optional[QDoubleSpinBox] = None
        self.g_pre: Optional[QDoubleSpinBox] = None
        self.g_post: Optional[QDoubleSpinBox] = None
        self.g_flow: Optional[QDoubleSpinBox] = None
        self.g_overlap: Optional[QSpinBox] = None
        self.g_purge: Optional[QCheckBox] = None
        self.g_sample: Optional[QDoubleSpinBox] = None
        self.g_maxpts: Optional[QSpinBox] = None
        self.g_maxang: Optional[QDoubleSpinBox] = None

        self._build_ui()
        self.apply_defaults()

    # ------------------- UI Build -------------------
    def _build_ui(self) -> None:
        outer = QVBoxLayout(self); outer.setContentsMargins(0,0,0,0)

        # Eine ScrollArea für den GESAMTEN Content (Globals + Seite)
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll)

        container = QWidget()
        scroll.setWidget(container)
        root = QVBoxLayout(container)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        # === Globals (inkl. description oben) ===
        self.gb_globals = QGroupBox("Globals")
        g = QFormLayout(self.gb_globals)

        # description / name
        self.e_desc = QLineEdit()
        g.addRow("description", self.e_desc)

        self.g_speed = QDoubleSpinBox(); self.g_speed.setRange(10, 2000); self.g_speed.setSingleStep(10); self.g_speed.setSuffix(" mm/s")
        self.g_standoff = QDoubleSpinBox(); self.g_standoff.setRange(0.1, 200); self.g_standoff.setSingleStep(0.1); self.g_standoff.setSuffix(" mm")
        self.g_angle = QDoubleSpinBox(); self.g_angle.setRange(0, 90); self.g_angle.setSingleStep(1); self.g_angle.setSuffix(" °")
        self.g_pre = QDoubleSpinBox(); self.g_pre.setRange(0, 10); self.g_pre.setSingleStep(0.1); self.g_pre.setSuffix(" s")
        self.g_post = QDoubleSpinBox(); self.g_post.setRange(0, 10); self.g_post.setSingleStep(0.1); self.g_post.setSuffix(" s")
        self.g_flow = QDoubleSpinBox(); self.g_flow.setRange(0, 100); self.g_flow.setSingleStep(0.1); self.g_flow.setSuffix(" ml/min")
        self.g_overlap = QSpinBox(); self.g_overlap.setRange(0, 100); self.g_overlap.setSuffix(" %")
        self.g_purge = QCheckBox("enable_purge")
        self.g_sample = QDoubleSpinBox(); self.g_sample.setRange(0.05, 50); self.g_sample.setSingleStep(0.05); self.g_sample.setSuffix(" mm")
        self.g_maxpts = QSpinBox(); self.g_maxpts.setRange(100, 2_000_000); self.g_maxpts.setSingleStep(100)
        self.g_maxang = QDoubleSpinBox(); self.g_maxang.setRange(0, 45); self.g_maxang.setSingleStep(0.5); self.g_maxang.setSuffix(" °")

        g.addRow("speed_mm_s", self.g_speed)
        g.addRow("stand_off_mm", self.g_standoff)
        g.addRow("spray_angle_deg", self.g_angle)
        g.addRow("pre_dispense_s", self.g_pre)
        g.addRow("post_dispense_s", self.g_post)
        g.addRow("flow_ml_min", self.g_flow)
        g.addRow("overlap_pct", self.g_overlap)
        g.addRow("", self.g_purge)
        g.addRow(_hline())
        g.addRow("sample_step_mm", self.g_sample)
        g.addRow("max_points", self.g_maxpts)
        g.addRow("max_angle_deg", self.g_maxang)

        root.addWidget(self.gb_globals)

        # === Stacked: pro Rezept eine Seite (Selectors + Sides) ===
        self.recipe_stack = QStackedWidget()
        root.addWidget(self.recipe_stack)

        # Spacer am Ende (drückt alles nach oben, kompakte Boxen)
        root.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

    # ------------------- Seiten/Defaults -------------------
    def apply_defaults(self) -> None:
        # Globals
        self.e_desc.setText("")
        self.g_speed.setValue(200.0)
        self.g_standoff.setValue(10.0)
        self.g_angle.setValue(0.0)
        self.g_pre.setValue(0.2)
        self.g_post.setValue(0.1)
        self.g_flow.setValue(5.0)
        self.g_overlap.setValue(50)
        self.g_purge.setChecked(False)
        self.g_sample.setValue(1.0)
        self.g_maxpts.setValue(200000)
        self.g_maxang.setValue(0.0)

        # Seite entfernen (Selectors/Sides werden neu aufgebaut)
        self._clear_recipe_page()

    def _clear_recipe_page(self) -> None:
        while self.recipe_stack.count() > 0:
            w = self.recipe_stack.widget(0)
            self.recipe_stack.removeWidget(w)
            w.deleteLater()

        self._side_editors.clear()
        self.sel_tool = None
        self.sel_substrate = None
        self.sel_mount = None

    # ------------------- Seite aufbauen -------------------
    def apply_recipe_to_forms(self, rec: Dict[str, Any]) -> None:
        """Erzeugt eine neue Stack-Seite für das konkrete Rezept (ohne description – das ist global)."""
        self._clear_recipe_page()

        # description aus YAML in das globale Feld schreiben
        self.e_desc.setText(str(rec.get("description", "")))

        page = QWidget()
        v = QVBoxLayout(page); v.setContentsMargins(0,0,0,0); v.setSpacing(10)

        # --- Selectors (rezept-spezifisch) ---
        sel_gb = QGroupBox("Selectors")
        sf = QFormLayout(sel_gb)
        self.sel_tool = QComboBox()
        self.sel_substrate = QComboBox()
        self.sel_mount = QComboBox()
        sf.addRow("tool", self.sel_tool)
        sf.addRow("substrate", self.sel_substrate)
        sf.addRow("mount", self.sel_mount)
        v.addWidget(sel_gb)

        # substrates
        self.sel_substrate.clear()
        subs = [str(s) for s in (rec.get("substrates") or [])]
        self.sel_substrate.addItems(subs)
        if subs:
            self.sel_substrate.setCurrentIndex(0)

        # mounts
        self.sel_mount.clear()
        mounts = [str(m) for m in (rec.get("substrate_mounts") or [])]
        self.sel_mount.addItems(mounts)
        if mounts:
            self.sel_mount.setCurrentIndex(0)

        # tools (NEU: nutze Liste aus recipes.yaml -> rec["tools"])
        self.sel_tool.clear()
        tools_from_recipe = [str(t) for t in (rec.get("tools") or [])]

        if tools_from_recipe:
            tool_items = tools_from_recipe
        else:
            # optionaler Fallback: versuche ctx.tools_yaml
            tool_items = []
            tools_cfg = getattr(self.ctx, "tools_yaml", None)
            if isinstance(tools_cfg, dict):
                d = tools_cfg.get("tools") if isinstance(tools_cfg.get("tools"), dict) else tools_cfg
                tool_items = list(d.keys())

            if not tool_items:
                # harter Fallback, damit nie leer
                tool_items = ["spray_nozzle_01", "spray_nozzle_02"]

        self.sel_tool.addItems(tool_items)
        if tool_items:
            self.sel_tool.setCurrentIndex(0)
        # bei Bedarf frei editierbar:
        # self.sel_tool.setEditable(True)

        # --- Sides/Editors ---
        self._side_editors = {}
        stype = str(rec.get("substrate_type") or "").strip()
        cfg = getattr(getattr(self.ctx, "store", None), "substrate_types", None)
        if cfg is None:
            cfg = (getattr(self.ctx, "recipes_yaml", {}) or {}).get("substrate_types", {})
        sides = dict((cfg.get(stype) or {}).get("sides") or {})

        for side_name, scfg in sides.items():
            gb = QGroupBox(f"Side: {side_name}")
            vg = QVBoxLayout(gb); vg.setContentsMargins(8, 8, 8, 8); vg.setSpacing(6)
            editor = SidePathEditor(side_name)
            vg.addWidget(editor)

            allowed = list(scfg.get("allowed_path_types") or [])
            if not allowed: allowed = ["meander_plane"]
            editor.set_allowed_types(allowed)

            editor.apply_default_path(
                dict(scfg.get("default_path") or {}),
                helix_enums={k: scfg[k] for k in ("start_froms","directions") if k in scfg}
            )

            v.addWidget(gb)
            self._side_editors[side_name] = editor

        # Spacer pro Seite
        from PyQt5.QtWidgets import QSpacerItem, QSizePolicy
        v.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # Seite in Stack einsetzen
        self.recipe_stack.addWidget(page)
        self.recipe_stack.setCurrentWidget(page)

        # Globale Parameter aus `rec.parameters` (override der Defaults)
        params = dict(rec.get("parameters") or {})
        def _opt_set(name, setter):
            if name in params: setter(params[name])
        _opt_set("speed_mm_s", lambda v: self.g_speed.setValue(float(v)))
        _opt_set("stand_off_mm", lambda v: self.g_standoff.setValue(float(v)))
        _opt_set("spray_angle_deg", lambda v: self.g_angle.setValue(float(v)))
        _opt_set("pre_dispense_s", lambda v: self.g_pre.setValue(float(v)))
        _opt_set("post_dispense_s", lambda v: self.g_post.setValue(float(v)))
        _opt_set("flow_ml_min", lambda v: self.g_flow.setValue(float(v)))
        _opt_set("overlap_pct", lambda v: self.g_overlap.setValue(int(v)))
        _opt_set("enable_purge", lambda v: self.g_purge.setChecked(bool(v)))
        _opt_set("sample_step_mm", lambda v: self.g_sample.setValue(float(v)))
        _opt_set("max_points", lambda v: self.g_maxpts.setValue(int(v)))
        _opt_set("max_angle_deg", lambda v: self.g_maxang.setValue(float(v)))


    # ------------------- Collectors -------------------
    def collect_globals(self) -> Dict[str, Any]:
        return {
            "speed_mm_s": float(self.g_speed.value()),
            "stand_off_mm": float(self.g_standoff.value()),
            "spray_angle_deg": float(self.g_angle.value()),
            "pre_dispense_s": float(self.g_pre.value()),
            "post_dispense_s": float(self.g_post.value()),
            "flow_ml_min": float(self.g_flow.value()),
            "overlap_pct": int(self.g_overlap.value()),
            "enable_purge": bool(self.g_purge.isChecked()),
            "sample_step_mm": float(self.g_sample.value()),
            "max_points": int(self.g_maxpts.value()),
            "max_angle_deg": float(self.g_maxang.value()),
        }

    def collect_paths_by_side(self) -> Dict[str, Dict[str, Any]]:
        out: Dict[str, Dict[str, Any]] = {}
        for side, ed in (self._side_editors or {}).items():
            out[side] = ed.collect_path()
        return out

    def active_selectors_values(self) -> Tuple[Optional[str], Optional[str], Optional[str]]:
        tool = (self.sel_tool.currentText().strip() if self.sel_tool and self.sel_tool.currentIndex() >= 0 else None)
        sub = (self.sel_substrate.currentText().strip() if self.sel_substrate and self.sel_substrate.currentIndex() >= 0 else None)
        mnt = (self.sel_mount.currentText().strip() if self.sel_mount and self.sel_mount.currentIndex() >= 0 else None)
        return tool or None, sub or None, mnt or None

    def get_description(self) -> str:
        return (self.e_desc.text().strip() if self.e_desc else "")
