# -*- coding: utf-8 -*-
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
      Valid | Points | Path length (mm) | ETA (s) | Medium (ml) | Mesh tris | Mesh L×B×H (mm)

    Erwartet ein info-Dict – typischerweise recipe.info.
    Unterstützt beide Varianten:
      - total_points / total_length_mm  (aus Recipe)
      - points / length_mm             (Panel-kompatibel)
    """
    def __init__(self, parent: Optional[QWidget] = None):
        super().__init__("Info", parent)
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

        self._v_valid     = add("Valid")
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
        if isinstance(v, float):
            return f"{v:.{nd}f}"
        if isinstance(v, int):
            # Tausendertrennzeichen als schmales Leerzeichen
            return f"{v:,}".replace(",", " ")
        try:
            # Things like np.int64 / np.float64
            if float(v).is_integer():
                return f"{int(v):,}".replace(",", " ")
            return f"{float(v):.{nd}f}"
        except Exception:
            return str(v)

    @staticmethod
    def _fmt_with_unit(v: Any, unit: str | None = None, nd: int = 3) -> str:
        s = InfoGroupBox._fmt_num(v, nd)
        return f"{s} {unit}".strip() if unit else s

    @staticmethod
    def _fmt_dims(dims: Any, nd: int = 1) -> str:
        """Erwartet (L, B, H) in mm."""
        try:
            L, B, H = [float(x) for x in (dims or ())]
        except Exception:
            return "-"
        return f"{L:.{nd}f} × {B:.{nd}f} × {H:.{nd}f}"

    @staticmethod
    def _fmt_bool(v: Any) -> str:
        if v is None:
            return "-"
        return "yes" if bool(v) else "no"

    # ---- Public API ----
    def set_values(self, info: Dict[str, Any] | None) -> None:
        """
        Erwartet recipe.info oder ein ähnliches Dict.

        Unterstützte Keys:
          - valid
          - points          oder total_points
          - length_mm       oder total_length_mm
          - eta_s
          - medium_ml
          - mesh_tris
          - mesh_bounds / mesh_bounds_mm (L,B,H in mm)
        """
        info = info or {}

        valid = info.get("valid")

        points = info.get("points")
        if points is None:
            points = info.get("total_points")

        length_mm = info.get("length_mm")
        if length_mm is None:
            length_mm = info.get("total_length_mm")

        eta_s = info.get("eta_s")
        medium_ml = info.get("medium_ml")
        mesh_tris = info.get("mesh_tris")

        mesh_bounds = info.get("mesh_bounds")
        if mesh_bounds is None:
            mesh_bounds = info.get("mesh_bounds_mm")

        self._v_valid.setText(self._fmt_bool(valid))
        self._v_points.setText(self._fmt_num(points, nd=0))
        self._v_len_mm.setText(self._fmt_with_unit(length_mm, "mm", nd=3))
        self._v_eta_s.setText(self._fmt_with_unit(eta_s, "s", nd=3))
        self._v_medium_ml.setText(self._fmt_with_unit(medium_ml, "ml", nd=3))
        self._v_mesh_tris.setText(self._fmt_num(mesh_tris, nd=0))
        self._v_mesh_dims.setText(self._fmt_dims(mesh_bounds, nd=1))
