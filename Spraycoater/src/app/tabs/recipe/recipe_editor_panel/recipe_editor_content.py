# recipe_editor_content.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Dict, Any, Tuple, List

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QGroupBox, QFormLayout, QLineEdit,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QStackedWidget, QFrame,
    QScrollArea, QSpacerItem, QSizePolicy
)

from app.model.recipe.recipe import Recipe
from app.model.recipe.recipe_store import RecipeStore
from .side_path_editor import SidePathEditor


def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.HLine)
    f.setFrameShadow(QFrame.Sunken)
    return f


# ------------------- Helper -------------------

def _unique_str_list(items) -> List[str]:
    seen = set()
    out: List[str] = []
    for x in (items or []):
        s = str(x)
        if s not in seen:
            seen.add(s)
            out.append(s)
    return out


def _coalesce_options(
    model: Recipe,
    *,
    single: str,                 # z.B. "tool" oder "substrate_mount"
    plurals: List[str],          # z.B. ["tools"] oder ["substrate_mounts", "mounts"]
    rec_def: Optional[Dict[str, Any]] = None,  # liest ggf. direkt aus Definition
) -> List[str]:
    opts: List[str] = []

    # 1) Plural-Felder direkt am Model
    for attr in plurals:
        vals = getattr(model, attr, None)
        if vals:
            opts.extend(vals)

    # 2) Recipe-Definition (Vorzugsquelle)
    if isinstance(rec_def, dict):
        for attr in plurals:
            vals = rec_def.get(attr)
            if vals:
                opts.extend(vals)

    # 3) Single-Feld als Fallback, wenn sonst nichts gefunden
    single_val = getattr(model, single, None)
    if single_val and not opts:
        opts = [single_val]

    return _unique_str_list(opts)


def _coalesce_sides_from_rec_def(rec_def: Dict[str, Any]) -> Dict[str, Any]:
    sides = rec_def.get("sides") or {}
    return dict(sides) if isinstance(sides, dict) else {}


class RecipeEditorContent(QWidget):
    """
    Model-driven Editor:
      [ScrollArea]
        └─ [VBox]
           ├─ [Globals]  (inkl. 'description')
           ├─ [recipe_stack]  (Selectors + Side-Path-Editor je Side)
           └─ Spacer
    """
    def __init__(self, *, ctx=None, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.store = store

        # Per Rezeptseite:
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

        # aktives Recipe-Model + Definition
        self._model: Optional[Recipe] = None
        self._rec_def: Optional[Dict[str, Any]] = None

        # Context-Key für Re-Priming bei Mount/Substrate/Tool-Wechsel
        self._last_ctx_key: Optional[str] = None

        self._build_ui()
        self.apply_defaults()

    # ------------------- UI Build -------------------
    def _build_ui(self) -> None:
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)

        # ScrollArea für (Globals + Seite)
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        outer.addWidget(scroll)

        container = QWidget()
        scroll.setWidget(container)
        root = QVBoxLayout(container)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(10)

        # === Globals (inkl. description) ===
        self.gb_globals = QGroupBox("Globals")
        g = QFormLayout(self.gb_globals)

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

        # === Stack: pro Recipe-Model eine Seite (Selectors + Sides) ===
        self.recipe_stack = QStackedWidget()
        self.recipe_stack.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        root.addWidget(self.recipe_stack)

        # Spacer am Ende
        root.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

    # ------------------- Seiten/Defaults -------------------
    def apply_defaults(self) -> None:
        # Global Defaults (harte Fallbacks – echte Defaults kommen über Store!)
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

        self._clear_recipe_page()

    def _clear_recipe_page(self) -> None:
        while self.recipe_stack.count() > 0:
            w = self.recipe_stack.widget(0)
            self.recipe_stack.removeWidget(w)
            w.deleteLater()

        self._side_editors.clear()
        self._disconnect_selectors()
        self.sel_tool = None
        self.sel_substrate = None
        self.sel_mount = None
        self._last_ctx_key = None
        self._rec_def = None

    def _disconnect_selectors(self) -> None:
        for combo in (self.sel_tool, self.sel_substrate, self.sel_mount):
            if not combo:
                continue
            try:
                combo.currentIndexChanged.disconnect(self._on_selectors_changed)
            except Exception:
                pass

    # ------------------- Model-Driven Aufbau -------------------
    def apply_recipe_model(self, model: Recipe, rec_def: Dict[str, Any]) -> None:
        """
        Baut die Formularseite vollständig aus dem Recipe-Model + Definition auf.
        """
        self._model = model
        self._rec_def = rec_def
        self._clear_recipe_page()

        # description/parameters aus Model → Globals
        self.e_desc.setText(str(getattr(model, "description", "") or ""))

        params = dict(getattr(model, "parameters", {}) or {})
        def _opt_set(name, setter):
            if name in params:
                setter(params[name])

        _opt_set("speed_mm_s", self.g_speed.setValue)
        _opt_set("stand_off_mm", self.g_standoff.setValue)
        _opt_set("spray_angle_deg", self.g_angle.setValue)
        _opt_set("pre_dispense_s", self.g_pre.setValue)
        _opt_set("post_dispense_s", self.g_post.setValue)
        _opt_set("flow_ml_min", self.g_flow.setValue)
        _opt_set("overlap_pct", lambda v: self.g_overlap.setValue(int(v)))
        _opt_set("enable_purge", lambda v: self.g_purge.setChecked(bool(v)))
        _opt_set("sample_step_mm", self.g_sample.setValue)
        _opt_set("max_points", lambda v: self.g_maxpts.setValue(int(v)))
        _opt_set("max_angle_deg", self.g_maxang.setValue)

        # Seite
        page = QWidget()
        v = QVBoxLayout(page)
        v.setContentsMargins(0, 0, 0, 0)
        v.setSpacing(10)

        # --- Selectors (aus rec_def) ---
        sel_gb = QGroupBox("Selectors")
        sf = QFormLayout(sel_gb)
        self.sel_tool = QComboBox()
        self.sel_substrate = QComboBox()
        self.sel_mount = QComboBox()
        sf.addRow("tool", self.sel_tool)
        sf.addRow("substrate", self.sel_substrate)
        sf.addRow("mount", self.sel_mount)
        v.addWidget(sel_gb)

        tools = _coalesce_options(model, single="tool", plurals=["tools", "tool_options"], rec_def=rec_def)
        substrates = _coalesce_options(model, single="substrate", plurals=["substrates", "substrate_options"], rec_def=rec_def)
        mounts = _coalesce_options(model, single="substrate_mount", plurals=["substrate_mounts", "mounts", "mount_options"], rec_def=rec_def)

        self.sel_tool.clear();        self.sel_tool.addItems(tools)
        self.sel_substrate.clear();   self.sel_substrate.addItems(substrates)
        self.sel_mount.clear();       self.sel_mount.addItems(mounts)

        def _set_if_present(combo: QComboBox, value: Optional[str]) -> None:
            if not combo:
                return
            if value is None:
                if combo.count() > 0:
                    combo.setCurrentIndex(0)
                return
            idx = combo.findText(str(value))
            if idx >= 0:
                combo.setCurrentIndex(idx)
            elif combo.count() > 0:
                combo.setCurrentIndex(0)

        _set_if_present(self.sel_tool, getattr(model, "tool", None))
        _set_if_present(self.sel_substrate, getattr(model, "substrate", None))
        _set_if_present(self.sel_mount, getattr(model, "substrate_mount", None))

        # --- Sides & Editors direkt aus rec_def ---
        self._side_editors = {}
        sides: Dict[str, Any] = _coalesce_sides_from_rec_def(rec_def)

        for side_name in sides.keys():
            gb = QGroupBox(f"Side: {side_name}")
            vg = QVBoxLayout(gb); vg.setContentsMargins(8, 8, 8, 8); vg.setSpacing(6)

            editor = SidePathEditor(side_name=side_name, store=self.store)
            vg.addWidget(editor)

            # Allowed + Default für Side aus Store
            side_cfg = self.store.allowed_and_default_for(rec_def, side_name)
            allowed = list(side_cfg.get("allowed_path_types") or ["meander_plane"])
            editor.set_allowed_types(allowed)

            # Defaults: entweder Model-Path, sonst default_path der Definition
            model_path = (model.paths_by_side or {}).get(side_name) if isinstance(model.paths_by_side, dict) else None
            default_path = dict(model_path or side_cfg.get("default_path") or {"type": allowed[0]})
            # Enums
            plane_enums = self.store.spiral_plane_enums()
            helix_enums = self.store.spiral_cylinder_enums()

            editor.apply_default_path(default_path, helix_enums=helix_enums, plane_enums=plane_enums)

            # Auto-Reset aktivieren (Type/Shape -> Defaults je Side)
            editor.enable_auto_reset(rec_def, side_name)

            v.addWidget(gb)
            self._side_editors[side_name] = editor

        # Spacer pro Seite
        v.addItem(QSpacerItem(0, 0, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # Seite in Stack einsetzen
        self.recipe_stack.addWidget(page)
        self.recipe_stack.setCurrentWidget(page)

        # Selector-Signale (Context-Wechsel)
        if self.sel_substrate:
            self.sel_substrate.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_mount:
            self.sel_mount.currentIndexChanged.connect(self._on_selectors_changed)
        if self.sel_tool:
            self.sel_tool.currentIndexChanged.connect(self._on_selectors_changed)

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

    def collect_paths_by_side(self) -> Dict[str, Any]:
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

    def set_path_for_side(self, side: str, path: Dict[str, Any]) -> None:
        ed = self._side_editors.get(side)
        if not ed:
            return
        plane_enums = self.store.spiral_plane_enums()
        helix_enums = self.store.spiral_cylinder_enums()
        ed.apply_default_path(path, helix_enums=helix_enums, plane_enums=plane_enums)

    # ------------------- Priming-Logik bei Contextwechsel -------------------
    def _current_ctx_key(self) -> str:
        tool = self.sel_tool.currentText().strip() if self.sel_tool and self.sel_tool.currentIndex() >= 0 else ""
        sub = self.sel_substrate.currentText().strip() if self.sel_substrate and self.sel_substrate.currentIndex() >= 0 else ""
        mnt = self.sel_mount.currentText().strip() if self.sel_mount and self.sel_mount.currentIndex() >= 0 else ""
        return f"{mnt}|{sub}|{tool}"

    def _on_selectors_changed(self, _idx: int = 0) -> None:
        key = self._current_ctx_key()
        if key == self._last_ctx_key:
            return
        self._last_ctx_key = key

        # Optionaler Callback am Model
        if self._model and hasattr(self._model, "on_context_changed"):
            try:
                tool, sub, mnt = self.active_selectors_values()
                self._model.on_context_changed(tool, sub, mnt)  # optional
            except Exception:
                pass

    def _prime_all_sides_from_defaults(self) -> None:
        if not (self._rec_def and self.store):
            return
        plane_enums = self.store.spiral_plane_enums()
        helix_enums = self.store.spiral_cylinder_enums()
        for side, editor in (self._side_editors or {}).items():
            side_cfg = self.store.allowed_and_default_for(self._rec_def, side)
            default_path = dict(side_cfg.get("default_path") or {})
            editor.apply_default_path(default_path, helix_enums=helix_enums, plane_enums=plane_enums)
