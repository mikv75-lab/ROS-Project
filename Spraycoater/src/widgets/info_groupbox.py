# -*- coding: utf-8 -*-
# File: widgets/info_groupbox.py
from __future__ import annotations
from typing import Optional, Dict, Any, TYPE_CHECKING

import numpy as np

from PyQt6.QtWidgets import (
    QGroupBox, QWidget, QHBoxLayout, QLabel, QSizePolicy
)

if TYPE_CHECKING:
    from model.recipe.recipe import Recipe

class InfoGroupBox(QGroupBox):
    """
    Intelligente Infozeile.
    
    Berechnet live Statistiken basierend auf Rezept und Pfad:
      Points | Path length | ETA | Medium
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
            k.setStyleSheet("font-weight:600; color: #555;")
            v = QLabel("-", self)
            lay.addWidget(k)
            lay.addWidget(v)
            return v

        # UI Elemente erstellen (Nur noch die geforderten 4)
        self._v_points    = add("Points")
        self._v_len_mm    = add("Path length (mm)")
        self._v_eta_s     = add("ETA (s)")
        self._v_medium_ml = add("Medium (ml)")

        lay.addStretch(1)

        sp = self.sizePolicy()
        sp.setHorizontalPolicy(QSizePolicy.Policy.Expanding)
        sp.setVerticalPolicy(QSizePolicy.Policy.Preferred)
        self.setSizePolicy(sp)

    # ---- Rechenlogik ----

    def update_from_recipe(self, recipe: Optional['Recipe'], points: Optional[np.ndarray] = None) -> None:
        """
        Berechnet Statistiken (Länge, Zeit, Verbrauch) direkt hier.
        """
        # 1. Standardwerte
        stats = {
            "points": 0, 
            "length_mm": 0.0, 
            "eta_s": 0.0, 
            "medium_ml": 0.0
        }

        # 2. Wenn Punkte da sind, rechnen wir los
        if points is not None and len(points) > 0:
            pts = np.asarray(points).reshape(-1, 3)
            stats["points"] = pts.shape[0]

            if pts.shape[0] > 1:
                # Länge (Summe der euklidischen Distanzen)
                diffs = pts[1:] - pts[:-1]
                dists = np.linalg.norm(diffs, axis=1)
                total_len = float(np.sum(dists))
                stats["length_mm"] = total_len

                # Rezept-Parameter holen
                if recipe:
                    params = getattr(recipe, "parameters", {}) or {}
                    
                    # Speed (mm/s) - Prüft auf Screenshot-Name 'speed_mm_s' oder Fallback
                    speed = float(params.get("speed_mm_s", params.get("robot_speed_mm_s", 50.0)))
                    if speed < 0.1: speed = 1.0
                    
                    # Flow (ml/min) - Prüft auf Screenshot-Name 'syringe_flow_ml_min' oder Fallback
                    flow_rate = float(params.get("syringe_flow_ml_min", params.get("flow_rate_ml_min", 0.0)))
                    
                    # --- A) ETA (Sekunden) ---
                    # Zeit = Weg / Geschwindigkeit
                    eta = total_len / speed
                    stats["eta_s"] = eta

                    # --- B) Medium (ml) ---
                    # Verbrauch = (Flow pro Minute / 60) * Sekunden
                    stats["medium_ml"] = (flow_rate / 60.0) * eta

        # 3. Werte anzeigen
        self._apply_values(stats)

    def _apply_values(self, info: Dict[str, Any]) -> None:
        self._v_points.setText(self._fmt_num(info["points"], nd=0))
        self._v_len_mm.setText(self._fmt_with_unit(info["length_mm"], "mm", nd=1))
        self._v_eta_s.setText(self._fmt_with_unit(info["eta_s"], "s", nd=1))
        self._v_medium_ml.setText(self._fmt_with_unit(info["medium_ml"], "ml", nd=3))

    # ---- Formatierung ----
    
    @staticmethod
    def _fmt_num(v: Any, nd: int = 3) -> str:
        if v is None: return "-"
        try:
            fv = float(v)
            if fv == 0.0: return "0"
            if nd <= 0 or fv.is_integer():
                return f"{int(round(fv)):,}".replace(",", " ")
            return f"{fv:.{nd}f}"
        except Exception:
            return str(v)

    @staticmethod
    def _fmt_with_unit(v: Any, unit: str | None = None, nd: int = 3) -> str:
        if not v: return "-" 
        s = InfoGroupBox._fmt_num(v, nd)
        if s == "-" or s == "0": return s
        return f"{s} {unit}" if unit else s