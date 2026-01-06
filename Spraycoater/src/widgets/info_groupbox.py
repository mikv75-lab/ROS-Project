# -*- coding: utf-8 -*-
# File: widgets/info_groupbox.py
from __future__ import annotations
from typing import Optional, Dict, Any, Tuple

from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QHBoxLayout, QLabel, QSizePolicy
)

Number = float | int
Dims = Tuple[Number, Number, Number]


class InfoGroupBox(QGroupBox):
    """
    Kompakte Infozeile.

    Zeigt:
      Points | Path length (mm) | ETA (s) | Medium (ml) | Mesh tris | Mesh L×B×H (mm)

    Erwartet ein info-Dict – typischerweise recipe.info.
    Unterstützt beide Varianten:
      - total_points / total_length_mm  (aus Recipe)
      - points / length_mm             (Panel-kompatibel)
    """

    def __init__(self, parent: Optional[QWidget] = None, *, title: str = "Info"):
        super().__init__(title, parent)
        self._build_ui()

    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add(label: str):
            k = QLabel(label + ":", self)
            k.setStyleSheet("font-weight:600;")
            v = QLabel("-", self)
            lay.addWidget(k)
            lay.addWidget(v)
            return v

        self._v_points    = add("Points")
        self._v_len_mm    = add("Path length (mm)")
        self._v_eta_s     = add("ETA (s)")
        self._v_medium_ml = add("Medium (ml)")
        self._v_mesh_tris = add("Mesh tris")
        self._v_mesh_dims = add("Mesh L×B×H (mm)")

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    # ---- formatting helpers ----
    @staticmethod
    def _fmt_num(v: Any, nd: int = 3) -> str:
        if v is None:
            return "-"
        try:
            fv = float(v)
            if nd <= 0 or fv.is_integer():
                return f"{int(round(fv)):,}".replace(",", " ")
            return f"{fv:.{nd}f}"
        except Exception:
            if isinstance(v, int):
                return f"{v:,}".replace(",", " ")
            return str(v)

    @staticmethod
    def _fmt_with_unit(v: Any, unit: str | None = None, nd: int = 3) -> str:
        s = InfoGroupBox._fmt_num(v, nd)
        if s == "-" or not unit:
            return s
        return f"{s} {unit}"

    @staticmethod
    def _pick(info: Dict[str, Any], *keys: str, default=None):
        for k in keys:
            if k in info and info.get(k) is not None:
                return info.get(k)
        return default

    @staticmethod
    def _fmt_dims_mm(dims: Any) -> str:
        try:
            if isinstance(dims, (list, tuple)) and len(dims) == 3:
                a, b, c = float(dims[0]), float(dims[1]), float(dims[2])
                return f"{a:.1f} × {b:.1f} × {c:.1f}"
        except Exception:
            pass
        return "-"

    # ---- public API ----
    def set_values(self, info: Dict[str, Any]) -> None:
        info = dict(info or {}) if isinstance(info, dict) else {}

        points = self._pick(info, "total_points", "points")
        length_mm = self._pick(info, "total_length_mm", "length_mm")
        eta_s = self._pick(info, "eta_s", "eta")
        medium_ml = self._pick(info, "medium_ml", "medium")
        mesh_tris = self._pick(info, "mesh_tris", "tris", "triangles")
        mesh_dims = self._pick(info, "mesh_dims_mm", "mesh_dims", "dims_mm")

        self._v_points.setText(self._fmt_num(points, nd=0))
        self._v_len_mm.setText(self._fmt_with_unit(length_mm, "mm", nd=1))
        self._v_eta_s.setText(self._fmt_with_unit(eta_s, "s", nd=1))
        self._v_medium_ml.setText(self._fmt_with_unit(medium_ml, "ml", nd=2))
        self._v_mesh_tris.setText(self._fmt_num(mesh_tris, nd=0))
        self._v_mesh_dims.setText(self._fmt_dims_mm(mesh_dims))
