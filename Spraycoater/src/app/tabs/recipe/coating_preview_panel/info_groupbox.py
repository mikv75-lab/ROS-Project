# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Dict, Any

from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QHBoxLayout, QLabel, QSizePolicy
)


class InfoGroupBox(QGroupBox):
    """
    Compact, single-row info panel, styled like OverlaysGroupBox.

    Shows:
      Points | Path length (mm) | ETA (s) | Medium (ml) | Mesh tris
    """
    def __init__(self, parent: Optional[Widget] = None):
        super().__init__("Info", parent)
        self._build_ui()

    def _build_ui(self) -> None:
        lay = QHBoxLayout(self)
        lay.setContentsMargins(8, 8, 8, 8)
        lay.setSpacing(14)

        def add(k: str):
            lab = QLabel(k + ":", self)
            lab.setStyleSheet("font-weight:600;")
            val = QLabel("-", self)
            lay.addWidget(lab)
            lay.addWidget(val)
            return val

        self._v_points     = add("Points")
        self._v_len_mm     = add("Path length (mm)")
        self._v_eta_s      = add("ETA (s)")
        self._v_medium_ml  = add("Medium (ml)")
        self._v_mesh_tris  = add("Mesh tris")

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    # ---- Public API ----
    @staticmethod
    def _fmt(v: Any, unit: str | None = None, nd: int = 3) -> str:
        if v is None:
            return "-"
        if isinstance(v, float):
            s = f"{v:.{nd}f}"
        else:
            s = str(v)
        return f"{s} {unit}".strip() if unit else s

    def set_values(self, info: Dict[str, Any]) -> None:
        self._v_points.setText(self._fmt(info.get("points"), None, 0))
        self._v_len_mm.setText(self._fmt(info.get("length_mm"), "mm"))
        self._v_eta_s.setText(self._fmt(info.get("eta_s"), "s"))
        self._v_medium_ml.setText(self._fmt(info.get("medium_ml"), "ml"))
        self._v_mesh_tris.setText(self._fmt(info.get("mesh_tris"), None, 0))
