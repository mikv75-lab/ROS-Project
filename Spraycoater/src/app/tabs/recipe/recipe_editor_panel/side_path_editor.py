# side_path_editor.py
# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, List, Optional, Sequence

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QDoubleSpinBox, QSpinBox, QCheckBox, QComboBox, QStackedWidget, QSizePolicy
)

from app.model.recipe.recipe_store import RecipeStore

# -------------------- kleine UI-Helper --------------------

def _make_compact(widget: QWidget) -> None:
    sp = widget.sizePolicy()
    sp.setVerticalPolicy(QSizePolicy.Maximum)
    widget.setSizePolicy(sp)
    lay = widget.layout()
    if lay is not None:
        lay.setSizeConstraint(QVBoxLayout.SetMinimumSize)


def _compact_form(f: QFormLayout) -> None:
    f.setContentsMargins(4, 4, 4, 4)
    f.setHorizontalSpacing(6)
    f.setVerticalSpacing(4)
    f.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
    f.setRowWrapPolicy(QFormLayout.DontWrapRows)


def _safe_set_combo_text(combo: QComboBox, text: str) -> None:
    idx = combo.findText(text)
    if idx < 0:
        combo.addItem(text)
    combo.setCurrentText(text)


# -------------------- kleine Daten-Helper --------------------

def _nested_get(data: dict, dotted: str, default: Any = None) -> Any:
    cur = data
    for part in dotted.split('.'):
        if not isinstance(cur, dict) or part not in cur:
            return default
        cur = cur[part]
    return cur


def _nested_set(data: dict, dotted: str, value: Any) -> None:
    parts = dotted.split('.')
    cur = data
    for p in parts[:-1]:
        if p not in cur or not isinstance(cur[p], dict):
            cur[p] = {}
        cur = cur[p]
    cur[parts[-1]] = value


def _set_widget_value(widget: QWidget, kind: str, value: Any) -> None:
    if kind == "double":
        widget.setValue(float(value))
    elif kind == "int":
        widget.setValue(int(value))
    elif kind == "check":
        widget.setChecked(bool(value))
    elif kind == "combo":
        _safe_set_combo_text(widget, str(value))
    elif kind == "vec2":
        widget.set(value if isinstance(value, (list, tuple)) else [0.0, 0.0])
    else:
        try:
            widget.setValue(float(value))
        except Exception:
            pass


def _get_widget_value(widget: QWidget, kind: str) -> Any:
    if kind == "double":
        return float(widget.value())
    if kind == "int":
        return int(widget.value())
    if kind == "check":
        return bool(widget.isChecked())
    if kind == "combo":
        return str(widget.currentText())
    if kind == "vec2":
        return widget.get()
    try:
        return float(widget.value())
    except Exception:
        return None


# -------------------- Vec2 Widget --------------------

class Vec2Edit(QWidget):
    def __init__(self, step: float = 0.1, unit: str = "mm", parent: Optional[QWidget] = None):
        super().__init__(parent)
        lay = QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(6)
        self.x = QDoubleSpinBox(); self.x.setDecimals(3); self.x.setSingleStep(step)
        self.y = QDoubleSpinBox(); self.y.setDecimals(3); self.y.setSingleStep(step)
        if unit:
            self.x.setSuffix(" " + unit)
            self.y.setSuffix(" " + unit)
        lay.addWidget(QLabel("x:")); lay.addWidget(self.x)
        lay.addSpacing(6)
        lay.addWidget(QLabel("y:")); lay.addWidget(self.y)

    def set(self, v: Sequence[float]):
        self.x.setValue(float(v[0] if v else 0.0))
        self.y.setValue(float(v[1] if v and len(v) > 1 else 0.0))

    def get(self) -> List[float]:
        return [float(self.x.value()), float(self.y.value())]


# -------------------- Haupteditor --------------------

class SidePathEditor(QWidget):
    """
    Editor für genau EINE Side: eigene type-Combo + Path-Form.
    Nutzt RecipeStore für Allowed/Enums/Defaults.
    Unterstützte Typen:
      - meander_plane
      - spiral_plane
      - spiral_cylinder
      - perimeter_follow_plane
      - polyhelix_cube
      - polyhelix_pyramid
    """
    TYPES_ORDER = [
        "meander_plane",
        "spiral_plane",
        "spiral_cylinder",
        "perimeter_follow_plane",
        "polyhelix_cube",
        "polyhelix_pyramid",
    ]

    def __init__(self, *, side_name: str, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.side_name = side_name
        self.store = store
        self._SCHEMA: Dict[str, Dict[str, tuple]] = {}
        self._rec_def_for_reset: Optional[Dict[str, Any]] = None  # für Auto-Reset
        self._side_for_reset: Optional[str] = None                 # für Auto-Reset
        self._build_ui()

    # ------------------- UI Build -------------------
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        form = QFormLayout()
        _compact_form(form)
        root.addLayout(form)
        self.type_combo = QComboBox()
        form.addRow(f"type ({self.side_name})", self.type_combo)

        self.stack = QStackedWidget()
        self.stack.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        root.addWidget(self.stack)

        # --- Page: meander_plane ---
        self.pg_meander = QWidget()
        f1 = QFormLayout(self.pg_meander); _compact_form(f1)
        self.m_area_shape = QComboBox(); self.m_area_shape.addItems(["circle", "rect"])
        self.m_center = Vec2Edit()
        self.m_radius = QDoubleSpinBox(); self.m_radius.setRange(0.0, 1e6); self.m_radius.setSingleStep(0.1); self.m_radius.setSuffix(" mm")
        self.m_size = Vec2Edit()
        self.m_pitch = QDoubleSpinBox(); self.m_pitch.setRange(0.0, 1e6); self.m_pitch.setSingleStep(0.1); self.m_pitch.setSuffix(" mm")
        self.m_angle = QDoubleSpinBox(); self.m_angle.setRange(0.0, 180.0); self.m_angle.setSingleStep(1.0); self.m_angle.setSuffix(" °")
        self.m_edge = QDoubleSpinBox(); self.m_edge.setRange(0.0, 1e6); self.m_edge.setSingleStep(0.1); self.m_edge.setSuffix(" mm")
        self.m_margin = QDoubleSpinBox(); self.m_margin.setRange(0.0, 1e6); self.m_margin.setSingleStep(0.1); self.m_margin.setSuffix(" mm")
        self.m_bou = QCheckBox()
        self.m_start = QComboBox(); self.m_start.addItems(["auto","minx_miny","minx_maxy","maxx_miny","maxx_maxy"])
        self.m_lead = QDoubleSpinBox(); self.m_lead.setRange(0.0, 1e6); self.m_lead.setSingleStep(0.1); self.m_lead.setSuffix(" mm")
        f1.addRow("area.shape", self.m_area_shape)
        f1.addRow("area.center_xy_mm", self.m_center)
        f1.addRow("area.radius_mm", self.m_radius)
        f1.addRow("area.size_mm", self.m_size)
        f1.addRow("pitch_mm", self.m_pitch)
        f1.addRow("angle_deg", self.m_angle)
        f1.addRow("edge_extend_mm", self.m_edge)
        f1.addRow("margin_mm", self.m_margin)
        f1.addRow("boustrophedon", self.m_bou)
        f1.addRow("start", self.m_start)
        f1.addRow("lead_in_mm", self.m_lead)

        # --- Page: spiral_plane ---
        self.pg_spiral_plane = QWidget()
        f2 = QFormLayout(self.pg_spiral_plane); _compact_form(f2)
        self.s_center = Vec2Edit()
        self.s_rout = QDoubleSpinBox(); self.s_rout.setRange(0.0, 1e6); self.s_rout.setSingleStep(0.1); self.s_rout.setSuffix(" mm")
        self.s_rin = QDoubleSpinBox(); self.s_rin.setRange(0.0, 1e6); self.s_rin.setSingleStep(0.1); self.s_rin.setSuffix(" mm")
        self.s_pitch = QDoubleSpinBox(); self.s_pitch.setRange(0.0, 1e6); self.s_pitch.setSingleStep(0.1); self.s_pitch.setSuffix(" mm")
        self.s_z = QDoubleSpinBox(); self.s_z.setRange(-1e6, 1e6); self.s_z.setSingleStep(0.1); self.s_z.setSuffix(" mm")
        self.s_lead = QDoubleSpinBox(); self.s_lead.setRange(0.0, 1e6); self.s_lead.setSingleStep(0.1); self.s_lead.setSuffix(" mm")
        self.s_direction = QComboBox(); self.s_direction.addItems(["ccw", "cw"])
        f2.addRow("center_xy_mm", self.s_center)
        f2.addRow("r_outer_mm", self.s_rout)
        f2.addRow("r_inner_mm", self.s_rin)
        f2.addRow("pitch_mm", self.s_pitch)
        f2.addRow("z_mm", self.s_z)
        f2.addRow("lead_in_mm", self.s_lead)
        f2.addRow("direction", self.s_direction)

        # --- Page: spiral_cylinder ---
        self.pg_spiral_cyl = QWidget()
        f3 = QFormLayout(self.pg_spiral_cyl); _compact_form(f3)
        self.c_pitch = QDoubleSpinBox(); self.c_pitch.setRange(0.0, 1e6); self.c_pitch.setSingleStep(0.1); self.c_pitch.setSuffix(" mm")
        self.c_out = QDoubleSpinBox(); self.c_out.setRange(0.0, 1e6); self.c_out.setSingleStep(0.1); self.c_out.setSuffix(" mm")
        self.c_top = QDoubleSpinBox(); self.c_top.setRange(0.0, 1e6); self.c_top.setSingleStep(0.1); self.c_top.setSuffix(" mm")
        self.c_bot = QDoubleSpinBox(); self.c_bot.setRange(0.0, 1e6); self.c_bot.setSingleStep(0.1); self.c_bot.setSuffix(" mm")
        self.c_lin = QDoubleSpinBox(); self.c_lin.setRange(0.0, 1e6); self.c_lin.setSingleStep(0.1); self.c_lin.setSuffix(" mm")
        self.c_lout = QDoubleSpinBox(); self.c_lout.setRange(0.0, 1e6); self.c_lout.setSingleStep(0.1); self.c_lout.setSuffix(" mm")
        self.c_start_from = QComboBox(); self.c_start_from.addItems(["top","bottom"])
        self.c_direction = QComboBox(); self.c_direction.addItems(["cw","ccw"])
        f3.addRow("pitch_mm", self.c_pitch)
        f3.addRow("outside_mm", self.c_out)
        f3.addRow("margin_top_mm", self.c_top)
        f3.addRow("margin_bottom_mm", self.c_bot)
        f3.addRow("lead_in_mm", self.c_lin)
        f3.addRow("lead_out_mm", self.c_lout)
        f3.addRow("start_from", self.c_start_from)
        f3.addRow("direction", self.c_direction)

        # --- Page: perimeter_follow_plane ---
        self.pg_perimeter = QWidget()
        f4 = QFormLayout(self.pg_perimeter); _compact_form(f4)
        self.p_area_shape = QComboBox(); self.p_area_shape.addItems(["circle","rect"])
        self.p_center = Vec2Edit()
        self.p_radius = QDoubleSpinBox(); self.p_radius.setRange(0.0, 1e6); self.p_radius.setSingleStep(0.1); self.p_radius.setSuffix(" mm")
        self.p_size = Vec2Edit()
        self.p_loops = QSpinBox(); self.p_loops.setRange(1, 100)
        self.p_off0 = QDoubleSpinBox(); self.p_off0.setRange(-1e3, 1e3); self.p_off0.setSingleStep(0.1); self.p_off0.setSuffix(" mm")
        self.p_offs = QDoubleSpinBox(); self.p_offs.setRange(0.0, 1e3); self.p_offs.setSingleStep(0.1); self.p_offs.setSuffix(" mm")
        self.p_blend = QDoubleSpinBox(); self.p_blend.setRange(0.0, 1e3); self.p_blend.setSingleStep(0.1); self.p_blend.setSuffix(" mm")
        self.p_lead = QDoubleSpinBox(); self.p_lead.setRange(0.0, 1e3); self.p_lead.setSingleStep(0.1); self.p_lead.setSuffix(" mm")
        self.p_angle = QDoubleSpinBox(); self.p_angle.setRange(0.0, 90.0); self.p_angle.setSingleStep(1.0); self.p_angle.setSuffix(" °")
        self.p_flow_min = QDoubleSpinBox(); self.p_flow_min.setRange(0.0, 1.0); self.p_flow_min.setSingleStep(0.05)
        self.p_flow_max = QDoubleSpinBox(); self.p_flow_max.setRange(0.0, 1.0); self.p_flow_max.setSingleStep(0.05)
        f4.addRow("area.shape", self.p_area_shape)
        f4.addRow("area.center_xy_mm", self.p_center)
        f4.addRow("area.radius_mm", self.p_radius)
        f4.addRow("area.size_mm", self.p_size)
        f4.addRow("loops", self.p_loops)
        f4.addRow("offset_start_mm", self.p_off0)
        f4.addRow("offset_step_mm", self.p_offs)
        f4.addRow("corner_blend_mm", self.p_blend)
        f4.addRow("lead_in_mm", self.p_lead)
        f4.addRow("spray_angle_deg", self.p_angle)
        f4.addRow("flow_scale_min", self.p_flow_min)
        f4.addRow("flow_scale_max", self.p_flow_max)

        # --- Page: polyhelix_cube ---
        self.pg_poly_cube = QWidget()
        f5 = QFormLayout(self.pg_poly_cube); _compact_form(f5)
        self.pc_edge = QDoubleSpinBox(); self.pc_edge.setRange(0.0, 1e6); self.pc_edge.setSingleStep(0.1); self.pc_edge.setSuffix(" mm")
        self.pc_height = QDoubleSpinBox(); self.pc_height.setRange(0.0, 1e6); self.pc_height.setSingleStep(0.1); self.pc_height.setSuffix(" mm")
        self.pc_pitch = QDoubleSpinBox(); self.pc_pitch.setRange(0.0, 1e6); self.pc_pitch.setSingleStep(0.1); self.pc_pitch.setSuffix(" mm")
        self.pc_dz = QDoubleSpinBox(); self.pc_dz.setRange(0.0, 1e3); self.pc_dz.setSingleStep(0.05); self.pc_dz.setSuffix(" mm")
        self.pc_phase = QDoubleSpinBox(); self.pc_phase.setRange(0.0, 360.0); self.pc_phase.setSingleStep(1.0); self.pc_phase.setSuffix(" °")
        self.pc_standoff = QDoubleSpinBox(); self.pc_standoff.setRange(0.0, 1e3); self.pc_standoff.setSingleStep(0.1); self.pc_standoff.setSuffix(" mm")
        self.pc_corner_r = QDoubleSpinBox(); self.pc_corner_r.setRange(0.0, 1e3); self.pc_corner_r.setSingleStep(0.1); self.pc_corner_r.setSuffix(" mm")
        self.pc_azim = Vec2Edit(step=1.0, unit="deg")
        self.pc_flow_min = QDoubleSpinBox(); self.pc_flow_min.setRange(0.0, 1.0); self.pc_flow_min.setSingleStep(0.05)
        self.pc_flow_max = QDoubleSpinBox(); self.pc_flow_max.setRange(0.0, 1.0); self.pc_flow_max.setSingleStep(0.05)
        self.pc_cos = QCheckBox()
        f5.addRow("edge_len_mm", self.pc_edge)
        f5.addRow("height_mm", self.pc_height)
        f5.addRow("pitch_mm", self.pc_pitch)
        f5.addRow("dz_mm", self.pc_dz)
        f5.addRow("start_phase_deg", self.pc_phase)
        f5.addRow("stand_off_mm", self.pc_standoff)
        f5.addRow("corner_roll_radius_mm", self.pc_corner_r)
        f5.addRow("azimuth_sector_deg", self.pc_azim)
        f5.addRow("flow_scale_min", self.pc_flow_min)
        f5.addRow("flow_scale_max", self.pc_flow_max)
        f5.addRow("cos_alpha_comp", self.pc_cos)

        # --- Page: polyhelix_pyramid ---
        self.pg_poly_pyr = QWidget()
        f6 = QFormLayout(self.pg_poly_pyr); _compact_form(f6)
        self.pp_sides = QSpinBox(); self.pp_sides.setRange(3, 128)
        self.pp_edge = QDoubleSpinBox(); self.pp_edge.setRange(0.0, 1e6); self.pp_edge.setSingleStep(0.1); self.pp_edge.setSuffix(" mm")
        self.pp_height = QDoubleSpinBox(); self.pp_height.setRange(0.0, 1e6); self.pp_height.setSingleStep(0.1); self.pp_height.setSuffix(" mm")
        self.pp_pitch = QDoubleSpinBox(); self.pp_pitch.setRange(0.0, 1e6); self.pp_pitch.setSingleStep(0.1); self.pp_pitch.setSuffix(" mm")
        self.pp_dz = QDoubleSpinBox(); self.pp_dz.setRange(0.0, 1e3); self.pp_dz.setSingleStep(0.05); self.pp_dz.setSuffix(" mm")
        self.pp_phase = QDoubleSpinBox(); self.pp_phase.setRange(0.0, 360.0); self.pp_phase.setSingleStep(1.0); self.pp_phase.setSuffix(" °")
        self.pp_standoff = QDoubleSpinBox(); self.pp_standoff.setRange(0.0, 1e3); self.pp_standoff.setSingleStep(0.1); self.pp_standoff.setSuffix(" mm")
        self.pp_corner_r = QDoubleSpinBox(); self.pp_corner_r.setRange(0.0, 1e3); self.pp_corner_r.setSingleStep(0.1); self.pp_corner_r.setSuffix(" mm")
        self.pp_cap = QDoubleSpinBox(); self.pp_cap.setRange(0.0, 1e3); self.pp_cap.setSingleStep(0.1); self.pp_cap.setSuffix(" mm")
        self.pp_azim = Vec2Edit(step=1.0, unit="deg")
        self.pp_flow_min = QDoubleSpinBox(); self.pp_flow_min.setRange(0.0, 1.0); self.pp_flow_min.setSingleStep(0.05)
        self.pp_flow_max = QDoubleSpinBox(); self.pp_flow_max.setRange(0.0, 1.0); self.pp_flow_max.setSingleStep(0.05)
        self.pp_cos = QCheckBox()
        f6.addRow("base_polygon_sides", self.pp_sides)
        f6.addRow("base_edge_len_mm", self.pp_edge)
        f6.addRow("height_mm", self.pp_height)
        f6.addRow("pitch_mm", self.pp_pitch)
        f6.addRow("dz_mm", self.pp_dz)
        f6.addRow("start_phase_deg", self.pp_phase)
        f6.addRow("stand_off_mm", self.pp_standoff)
        f6.addRow("corner_roll_radius_mm", self.pp_corner_r)
        f6.addRow("cap_top_mm", self.pp_cap)
        f6.addRow("azimuth_sector_deg", self.pp_azim)
        f6.addRow("flow_scale_min", self.pp_flow_min)
        f6.addRow("flow_scale_max", self.pp_flow_max)
        f6.addRow("cos_alpha_comp", self.pp_cos)

        # Pages registrieren
        self.stack.addWidget(self.pg_meander)       # 0
        self.stack.addWidget(self.pg_spiral_plane)  # 1
        self.stack.addWidget(self.pg_spiral_cyl)    # 2
        self.stack.addWidget(self.pg_perimeter)     # 3
        self.stack.addWidget(self.pg_poly_cube)     # 4
        self.stack.addWidget(self.pg_poly_pyr)      # 5

        # Verhalten
        self.type_combo.currentIndexChanged.connect(self._on_type_changed)
        self.m_area_shape.currentIndexChanged.connect(self._sync_meander_area_visibility)
        self.p_area_shape.currentIndexChanged.connect(self._sync_perimeter_area_visibility)
        self._sync_meander_area_visibility()
        self._sync_perimeter_area_visibility()

        # kompakt
        for w in (self.pg_meander, self.pg_spiral_plane, self.pg_spiral_cyl,
                  self.pg_perimeter, self.pg_poly_cube, self.pg_poly_pyr, self):
            _make_compact(w)

        self._lock_stack_height()
        self._build_schema()

    # ------------------- Schema (Key -> (Widget, Kind, Default)) -------------------

    def _build_schema(self) -> None:
        self._SCHEMA = {
            "meander_plane": {
                "area.shape":            (self.m_area_shape, "combo",  "circle"),
                "area.center_xy_mm":     (self.m_center,     "vec2",   [0.0, 0.0]),
                "area.radius_mm":        (self.m_radius,     "double", 50.0),
                "area.size_mm":          (self.m_size,       "vec2",   [100.0, 100.0]),
                "pitch_mm":              (self.m_pitch,      "double", 5.0),
                "angle_deg":             (self.m_angle,      "double", 0.0),
                "edge_extend_mm":        (self.m_edge,       "double", 0.0),
                "margin_mm":             (self.m_margin,     "double", 0.0),
                "boustrophedon":         (self.m_bou,        "check",  True),
                "start":                 (self.m_start,      "combo",  "auto"),
                "lead_in_mm":            (self.m_lead,       "double", 5.0),
            },
            "spiral_plane": {
                "center_xy_mm":          (self.s_center,     "vec2",   [0.0, 0.0]),
                "r_outer_mm":            (self.s_rout,       "double", 45.0),
                "r_inner_mm":            (self.s_rin,        "double", 5.0),
                "pitch_mm":              (self.s_pitch,      "double", 4.0),
                "z_mm":                  (self.s_z,          "double", 10.0),
                "lead_in_mm":            (self.s_lead,       "double", 3.0),
                "direction":             (self.s_direction,  "combo",  "ccw"),
            },
            "spiral_cylinder": {
                "pitch_mm":              (self.c_pitch,      "double", 10.0),
                "outside_mm":            (self.c_out,        "double", 1.0),
                "margin_top_mm":         (self.c_top,        "double", 0.0),
                "margin_bottom_mm":      (self.c_bot,        "double", 0.0),
                "lead_in_mm":            (self.c_lin,        "double", 5.0),
                "lead_out_mm":           (self.c_lout,       "double", 0.0),
                "start_from":            (self.c_start_from, "combo",  "top"),
                "direction":             (self.c_direction,  "combo",  "cw"),
            },
            "perimeter_follow_plane": {
                "area.shape":            (self.p_area_shape, "combo",  "circle"),
                "area.center_xy_mm":     (self.p_center,     "vec2",   [0.0, 0.0]),
                "area.radius_mm":        (self.p_radius,     "double", 50.0),
                "area.size_mm":          (self.p_size,       "vec2",   [100.0, 100.0]),
                "loops":                 (self.p_loops,      "int",    2),
                "offset_start_mm":       (self.p_off0,       "double", 1.0),
                "offset_step_mm":        (self.p_offs,       "double", 1.0),
                "corner_blend_mm":       (self.p_blend,      "double", 5.0),
                "lead_in_mm":            (self.p_lead,       "double", 5.0),
                "spray_angle_deg":       (self.p_angle,      "double", 8.0),
                "flow_scale_min":        (self.p_flow_min,   "double", 0.6),
                "flow_scale_max":        (self.p_flow_max,   "double", 1.0),
            },
            "polyhelix_cube": {
                "edge_len_mm":           (self.pc_edge,      "double", 100.0),
                "height_mm":             (self.pc_height,    "double", 100.0),
                "pitch_mm":              (self.pc_pitch,     "double", 6.0),
                "dz_mm":                 (self.pc_dz,        "double", 1.0),
                "start_phase_deg":       (self.pc_phase,     "double", 0.0),
                "stand_off_mm":          (self.pc_standoff,  "double", 25.0),
                "corner_roll_radius_mm": (self.pc_corner_r,  "double", 8.0),
                "azimuth_sector_deg":    (self.pc_azim,      "vec2",   [-180.0, 180.0]),
                "flow_scale_min":        (self.pc_flow_min,  "double", 0.3),
                "flow_scale_max":        (self.pc_flow_max,  "double", 1.0),
                "cos_alpha_comp":        (self.pc_cos,       "check",  True),
            },
            "polyhelix_pyramid": {
                "base_polygon_sides":    (self.pp_sides,     "int",    4),
                "base_edge_len_mm":      (self.pp_edge,      "double", 100.0),
                "height_mm":             (self.pp_height,    "double", 60.0),
                "pitch_mm":              (self.pp_pitch,     "double", 6.0),
                "dz_mm":                 (self.pp_dz,        "double", 1.0),
                "start_phase_deg":       (self.pp_phase,     "double", 0.0),
                "stand_off_mm":          (self.pp_standoff,  "double", 25.0),
                "corner_roll_radius_mm": (self.pp_corner_r,  "double", 8.0),
                "cap_top_mm":            (self.pp_cap,       "double", 8.0),
                "azimuth_sector_deg":    (self.pp_azim,      "vec2",   [-180.0, 180.0]),
                "flow_scale_min":        (self.pp_flow_min,  "double", 0.3),
                "flow_scale_max":        (self.pp_flow_max,  "double", 1.0),
                "cos_alpha_comp":        (self.pp_cos,       "check",  True),
            },
        }

    # ------------------- Public API -------------------

    def set_allowed_types(self, types: List[str]) -> None:
        current = self.type_combo.currentText()
        self.type_combo.blockSignals(True)
        self.type_combo.clear()
        ordered = [t for t in self.TYPES_ORDER if t in set(types)] or ["meander_plane"]
        self.type_combo.addItems(ordered)
        self.type_combo.blockSignals(False)
        if current in ordered:
            self.type_combo.setCurrentText(current)
        self._on_type_changed(self.type_combo.currentIndex())

    def apply_default_path(
        self,
        path: Dict[str, Any],
        *,
        helix_enums: Optional[Dict[str, Any]] = None,
        plane_enums: Optional[Dict[str, Any]] = None,
    ) -> None:
        t = str(path.get("type", "meander_plane"))
        if t not in self.TYPES_ORDER:
            t = "meander_plane"
        if self.type_combo.findText(t) < 0:
            self.type_combo.addItem(t)
        self.type_combo.setCurrentText(t)

        if t == "spiral_plane" and plane_enums and isinstance(plane_enums.get("direction"), list):
            self.s_direction.blockSignals(True)
            self.s_direction.clear()
            self.s_direction.addItems([str(x) for x in plane_enums["direction"]])
            self.s_direction.blockSignals(False)

        if t == "spiral_cylinder" and helix_enums:
            sfs = helix_enums.get("start_froms"); dirs = helix_enums.get("directions")
            if isinstance(sfs, (list, tuple)) and sfs:
                self.c_start_from.blockSignals(True)
                self.c_start_from.clear()
                self.c_start_from.addItems([str(x) for x in sfs])
                self.c_start_from.blockSignals(False)
            if isinstance(dirs, (list, tuple)) and dirs:
                self.c_direction.blockSignals(True)
                self.c_direction.clear()
                self.c_direction.addItems([str(x) for x in dirs])
                self.c_direction.blockSignals(False)

        schema = self._SCHEMA.get(t, {})
        for dotted_key, (widget, kind, default) in schema.items():
            val = _nested_get(path, dotted_key, default)
            if dotted_key == "area.shape" and val == default:
                if t == "meander_plane":
                    val = self.m_area_shape.currentText() or default
                elif t == "perimeter_follow_plane":
                    val = self.p_area_shape.currentText() or default
            _set_widget_value(widget, kind, val)

        self._sync_meander_area_visibility()
        self._sync_perimeter_area_visibility()
        self._lock_stack_height()

    def collect_path(self) -> Dict[str, Any]:
        t = self.type_combo.currentText() or "meander_plane"
        out: Dict[str, Any] = {"type": t}
        schema = self._SCHEMA.get(t, {})
        if not schema:
            return out

        shape_val = None
        if "area.shape" in schema:
            widget, kind, _ = schema["area.shape"]
            shape_val = _get_widget_value(widget, kind)
            _nested_set(out, "area.shape", shape_val)

        for dotted_key, (widget, kind, _default) in schema.items():
            if dotted_key == "area.shape":
                continue
            if dotted_key == "area.radius_mm" and shape_val and str(shape_val) != "circle":
                continue
            if dotted_key == "area.size_mm" and shape_val and str(shape_val) != "rect":
                continue
            val = _get_widget_value(widget, kind)
            if kind == "vec2" and (not isinstance(val, (list, tuple)) or len(val) != 2):
                continue
            _nested_set(out, dotted_key, val)
        return out

    def enable_auto_reset(self, rec_def: Dict[str, Any], side_name: str) -> None:
        """
        Bei Änderung von 'type' oder 'area.shape' wird die Side auf Defaults
        (aus Store/Schemen) zurückgesetzt – unter Beibehalt der aktuellen
        Auswahl von type/shape.
        """
        self._rec_def_for_reset = dict(rec_def or {})
        self._side_for_reset = str(side_name)

        def _reload_defaults():
            if not (self._rec_def_for_reset and self._side_for_reset):
                return
            # 1) Basis-Default aus YAML für diese Side
            side_cfg = self.store.allowed_and_default_for(self._rec_def_for_reset, self._side_for_reset)
            base = dict(side_cfg.get("default_path") or {})

            # 2) aktuellen Typ durchsetzen
            t = self.type_combo.currentText().strip() or base.get("type") or "meander_plane"
            base["type"] = t

            # 3) falls Shape-relevant, aktuelle Shape übernehmen
            if t == "meander_plane":
                shape = self.m_area_shape.currentText().strip()
                if shape:
                    _nested_set(base, "area.shape", shape)
            elif t == "perimeter_follow_plane":
                shape = self.p_area_shape.currentText().strip()
                if shape:
                    _nested_set(base, "area.shape", shape)

            # 4) Enums ziehen & anwenden
            plane_enums = self.store.spiral_plane_enums()
            helix_enums = self.store.spiral_cylinder_enums()
            self.apply_default_path(base, helix_enums=helix_enums, plane_enums=plane_enums)

        # Signale koppeln
        self.type_combo.currentTextChanged.connect(_reload_defaults)
        self.m_area_shape.currentTextChanged.connect(_reload_defaults)
        self.p_area_shape.currentTextChanged.connect(_reload_defaults)

    # ------------------- intern -------------------
    def _on_type_changed(self, _idx: int) -> None:
        t = self.type_combo.currentText()
        try:
            idx = self.TYPES_ORDER.index(t)
        except ValueError:
            idx = 0
        self.stack.setCurrentIndex(idx)
        self._sync_meander_area_visibility()
        self._sync_perimeter_area_visibility()
        self._lock_stack_height()

    def _sync_meander_area_visibility(self) -> None:
        is_circle = (self.m_area_shape.currentText() == "circle")
        self.m_radius.setEnabled(is_circle)
        self.m_size.setEnabled(not is_circle)

    def _sync_perimeter_area_visibility(self) -> None:
        is_circle = (self.p_area_shape.currentText() == "circle")
        self.p_radius.setEnabled(is_circle)
        self.p_size.setEnabled(not is_circle)

    def _lock_stack_height(self) -> None:
        w = self.stack.currentWidget()
        if w is None:
            return
        w.adjustSize()
        self.stack.adjustSize()
        h = max(w.sizeHint().height(), self.stack.sizeHint().height())
        self.stack.setMinimumHeight(h)
        self.stack.setMaximumHeight(h)
        self.stack.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        self.adjustSize()
