# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Any, Dict, List, Optional, Sequence

from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QFormLayout, QLabel,
    QDoubleSpinBox, QCheckBox, QComboBox, QStackedWidget, QSizePolicy
)


def _make_compact(widget: QWidget) -> None:
    """Verhindert vertikales Strecken: Höhe orientiert sich am sizeHint()."""
    sp = widget.sizePolicy()
    sp.setVerticalPolicy(QSizePolicy.Maximum)
    widget.setSizePolicy(sp)
    lay = widget.layout()
    if lay is not None:
        lay.setSizeConstraint(QVBoxLayout.SetMinimumSize)


def _compact_form(f: QFormLayout) -> None:
    """Kleine Abstände, keine unnötigen Ränder im FormLayout."""
    f.setContentsMargins(4, 4, 4, 4)
    f.setHorizontalSpacing(6)
    f.setVerticalSpacing(4)
    f.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
    f.setRowWrapPolicy(QFormLayout.DontWrapRows)


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


class SidePathEditor(QWidget):
    """
    Editor für genau EINE Side: eigene type-Combo + Path-Form.
    Unterstützte Typen: meander_plane, spiral_plane, spiral_cylinder
    """
    def __init__(self, side_name: str, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.side_name = side_name
        self._build_ui()

    # ------------------- UI Build -------------------
    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # Typ-Combo (nur erlaubte Typen)
        form = QFormLayout()
        _compact_form(form)
        root.addLayout(form)
        self.type_combo = QComboBox()
        form.addRow(f"type ({self.side_name})", self.type_combo)

        # Stacked: meander_plane / spiral_plane / spiral_cylinder
        self.stack = QStackedWidget()
        # stack darf NICHT strecken: Höhe wird auf sizeHint gelockt
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
        self.s_direction = QComboBox();  # cw/ccw (optional – aus schema/YAML)
        # Default-Werte einsetzen; tatsächliche Items werden ggf. durch apply_default_path(plane_enums=...) ersetzt
        self.s_direction.addItems(["ccw", "cw"])
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

        self.stack.addWidget(self.pg_meander)       # index 0
        self.stack.addWidget(self.pg_spiral_plane)  # index 1
        self.stack.addWidget(self.pg_spiral_cyl)    # index 2

        # Verhalten
        self.type_combo.currentIndexChanged.connect(self._on_type_changed)
        self.m_area_shape.currentIndexChanged.connect(self._sync_meander_area_visibility)
        self._sync_meander_area_visibility()

        # kompakte Policies
        _make_compact(self.pg_meander)
        _make_compact(self.pg_spiral_plane)
        _make_compact(self.pg_spiral_cyl)
        _make_compact(self)

        # Stack-Höhe auf Content locken
        self._lock_stack_height()

    # ------------------- API -------------------
    def set_allowed_types(self, types: List[str]) -> None:
        """Liste wie ['meander_plane','spiral_plane']"""
        current = self.type_combo.currentText()
        self.type_combo.blockSignals(True)
        self.type_combo.clear()
        self.type_combo.addItems([str(t) for t in types])
        # Stack passend setzen
        if "meander_plane" in types and current == "meander_plane":
            self.stack.setCurrentIndex(0)
        elif "spiral_plane" in types and current == "spiral_plane":
            self.stack.setCurrentIndex(1)
        elif "spiral_cylinder" in types and current == "spiral_cylinder":
            self.stack.setCurrentIndex(2)
        self.type_combo.blockSignals(False)
        self._on_type_changed(self.type_combo.currentIndex())

    def apply_default_path(
        self,
        path: Dict[str, Any],
        *,
        helix_enums: Optional[Dict[str, Any]] = None,
        plane_enums: Optional[Dict[str, Any]] = None,
    ) -> None:
        """
        Setzt Defaultwerte gemäß path + (optional) Enum-Listen.
        - helix_enums: {'start_froms': [...], 'directions': [...]}
        - plane_enums: {'direction': [...]}
        """
        t = str(path.get("type", "meander_plane"))
        if t not in ("meander_plane", "spiral_plane", "spiral_cylinder"):
            t = "meander_plane"
        self.type_combo.setCurrentText(t)

        if t == "meander_plane":
            area = dict(path.get("area") or {})
            self.m_area_shape.setCurrentText(str(area.get("shape", "circle")))
            self.m_center.set(area.get("center_xy_mm", [0.0, 0.0]))
            if self.m_area_shape.currentText() == "circle":
                self.m_radius.setValue(float(area.get("radius_mm", 50.0)))
            else:
                self.m_size.set(area.get("size_mm", [100.0, 100.0]))
            self.m_pitch.setValue(float(path.get("pitch_mm", 5.0)))
            self.m_angle.setValue(float(path.get("angle_deg", 0.0)))
            self.m_edge.setValue(float(path.get("edge_extend_mm", 0.0)))
            self.m_margin.setValue(float(path.get("margin_mm", 0.0)))
            self.m_bou.setChecked(bool(path.get("boustrophedon", True)))
            self.m_start.setCurrentText(str(path.get("start", "auto")))
            self.m_lead.setValue(float(path.get("lead_in_mm", 5.0)))

        elif t == "spiral_plane":
            # Enums (direction) ggf. aus Schema setzen
            if plane_enums and isinstance(plane_enums.get("direction"), list) and plane_enums["direction"]:
                self.s_direction.clear()
                self.s_direction.addItems([str(x) for x in plane_enums["direction"]])

            self.s_center.set(path.get("center_xy_mm", [0.0, 0.0]))
            self.s_rout.setValue(float(path.get("r_outer_mm", 45.0)))
            self.s_rin.setValue(float(path.get("r_inner_mm", 5.0)))
            self.s_pitch.setValue(float(path.get("pitch_mm", 4.0)))
            self.s_z.setValue(float(path.get("z_mm", 10.0)))
            self.s_lead.setValue(float(path.get("lead_in_mm", 3.0)))
            # direction (falls vorhanden)
            self.s_direction.setCurrentText(str(path.get("direction", self.s_direction.currentText())))

        else:  # spiral_cylinder
            self.c_pitch.setValue(float(path.get("pitch_mm", 10.0)))
            self.c_out.setValue(float(path.get("outside_mm", 1.0)))
            self.c_top.setValue(float(path.get("margin_top_mm", 0.0)))
            self.c_bot.setValue(float(path.get("margin_bottom_mm", 0.0)))
            self.c_lin.setValue(float(path.get("lead_in_mm", 5.0)))
            self.c_lout.setValue(float(path.get("lead_out_mm", 0.0)))

            # helix enums (start_from / direction)
            if helix_enums:
                sfs = helix_enums.get("start_froms"); dirs = helix_enums.get("directions")
                if isinstance(sfs, (list, tuple)) and sfs:
                    self.c_start_from.clear(); self.c_start_from.addItems([str(x) for x in sfs])
                if isinstance(dirs, (list, tuple)) and dirs:
                    self.c_direction.clear(); self.c_direction.addItems([str(x) for x in dirs])

        self._sync_meander_area_visibility()
        self._lock_stack_height()

    def collect_path(self) -> Dict[str, Any]:
        t = self.type_combo.currentText()
        out: Dict[str, Any] = {"type": t}
        if t == "meander_plane":
            shape = self.m_area_shape.currentText()
            area: Dict[str, Any] = {"shape": shape, "center_xy_mm": self.m_center.get()}
            if shape == "circle":
                area["radius_mm"] = float(self.m_radius.value())
            else:
                area["size_mm"] = self.m_size.get()
            out.update({
                "area": area,
                "pitch_mm": float(self.m_pitch.value()),
                "angle_deg": float(self.m_angle.value()),
                "edge_extend_mm": float(self.m_edge.value()),
                "margin_mm": float(self.m_margin.value()),
                "boustrophedon": bool(self.m_bou.isChecked()),
                "start": self.m_start.currentText(),
                "lead_in_mm": float(self.m_lead.value()),
            })
        elif t == "spiral_plane":
            out.update({
                "center_xy_mm": self.s_center.get(),
                "r_outer_mm": float(self.s_rout.value()),
                "r_inner_mm": float(self.s_rin.value()),
                "pitch_mm": float(self.s_pitch.value()),
                "z_mm": float(self.s_z.value()),
                "lead_in_mm": float(self.s_lead.value()),
                "direction": self.s_direction.currentText(),
            })
        else:
            out.update({
                "pitch_mm": float(self.c_pitch.value()),
                "outside_mm": float(self.c_out.value()),
                "margin_top_mm": float(self.c_top.value()),
                "margin_bottom_mm": float(self.c_bot.value()),
                "lead_in_mm": float(self.c_lin.value()),
                "lead_out_mm": float(self.c_lout.value()),
            })
            out["_selected_start_from"] = self.c_start_from.currentText()
            out["_selected_direction"] = self.c_direction.currentText()
        return out

    # ------------------- intern -------------------
    def _on_type_changed(self, _idx: int) -> None:
        t = self.type_combo.currentText()
        self.stack.setCurrentIndex(0 if t == "meander_plane" else 1 if t == "spiral_plane" else 2)
        self._sync_meander_area_visibility()
        self._lock_stack_height()

    def _sync_meander_area_visibility(self) -> None:
        is_circle = (self.m_area_shape.currentText() == "circle")
        self.m_radius.setEnabled(is_circle)
        self.m_size.setEnabled(not is_circle)

    def _lock_stack_height(self) -> None:
        """Fixiert die Stack-Höhe auf die sizeHint der aktiven Seite -> kein leerer Puffer."""
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
