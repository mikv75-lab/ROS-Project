# src/app/tabs/recipe/coating_preview_panel/coating_preview_panel.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Any, Iterable

import numpy as np
import pyvista as pv
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSizePolicy
from pyvistaqt import QtInteractor

from .preview import PreviewEngine
from .mesh_utils import (
    get_mount_scene_offset_from_key,
    apply_transform,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
    load_mount_mesh_from_key,
)

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")


class CoatingPreviewPanel(QWidget):
    """
    3D-Preview (PyVistaQt) für Mount + genau EIN Substrat + Pfade.
    Erwartet im Recipe:
      - recipe.substrate (str, erforderlich)
      - recipe.substrate_mount (str, erforderlich)
      - recipe.paths_by_side (optional): pro Side
          { "points": [[x,y,z], ...] } oder { "polyline": [[x,y,z], ...] }
        Die Punkte sind IMMER im Substrat-Lokalsystem angegeben.
        Sie werden mit demselben Ground-Shift und Mount-Offset transformiert
        wie das Substrat selbst.
    """

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        # Layout + Plotter
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)

        self._pv = QtInteractor(self)
        self._pv.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root.addWidget(self._pv)

        self._engine: PreviewEngine = PreviewEngine(self._pv)

        # Start: leere Szene + ISO + Achsen
        self._reset_view()

    # ---------- Public API ----------
    def render_from_model(self, model: Any, _sides: Iterable[str]) -> None:
        """Ablauf: clear → mount → substrate → recipe path(s) (auf Substrat-Position)."""
        mount_key = self._get_attr(model, "substrate_mount")
        if not isinstance(mount_key, str) or not mount_key.strip():
            raise ValueError("Recipe benötigt ein gültiges 'substrate_mount'.")
        mount_key = mount_key.strip()

        substrate_key = self._get_attr(model, "substrate")
        if not isinstance(substrate_key, str) or not substrate_key.strip():
            raise ValueError("Recipe benötigt ein gültiges 'substrate'.")
        substrate_key = substrate_key.strip()

        _LOG.info("Preview: mount=%s, substrate=%s", mount_key, substrate_key)

        # 0) Szene leeren
        self._engine.clear_scene()

        # 1) Mount laden + platzieren (nur anhand mount_key)
        mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
        xyz_mm, rpy_deg = get_mount_scene_offset_from_key(self.ctx, mount_key)
        mount_scene = apply_transform(mount_mesh, translate_mm=xyz_mm, rpy_deg=rpy_deg)
        self._engine.add_mesh(mount_scene, color="lightgray", opacity=1.0, show_edges=False)

        # 2) Substrat laden + exakt so platzieren wie in place_substrate_on_mount
        sub_mesh = load_substrate_mesh_from_key(self.ctx, substrate_key)

        # Ground-Shift (wie in place_substrate_on_mount)
        zmin = float(sub_mesh.bounds[4])  # axis-aligned bbox
        ground_shift = np.array([0.0, 0.0, -zmin], dtype=float)

        # Mount-Transform
        R_mount = _rpy_deg_to_matrix_np(rpy_deg)
        t_mount = np.asarray(xyz_mm, dtype=float)

        # Visual: wir nutzen die bestehende Utility zum Platzieren fürs Mesh
        sub_scene = place_substrate_on_mount(self.ctx, sub_mesh, mount_key=mount_key)
        self._engine.add_mesh(sub_scene, color="orange", opacity=0.85, show_edges=False)

        # 3) Pfade (Substrat-Lokal → Ground-Shift → Mount-Transform)
        self._draw_recipe_paths(model, R_mount, t_mount, ground_shift)

        # 4) Ansicht / Achsen
        self._add_axes()
        self._engine.view_iso()

    # ---------- Intern ----------
    def _reset_view(self):
        self._engine.clear_scene()
        self._add_axes()
        self._engine.view_iso()

    def _add_axes(self):
        try:
            self._pv.add_axes()
        except Exception:
            pass
        L = 100.0  # mm
        self._engine.add_mesh(pv.Line((0, 0, 0), (L, 0, 0)))  # X
        self._engine.add_mesh(pv.Line((0, 0, 0), (0, L, 0)))  # Y
        self._engine.add_mesh(pv.Line((0, 0, 0), (0, 0, L)))  # Z

    def _draw_recipe_paths(self, model: Any, R_mount: np.ndarray, t_mount: np.ndarray, ground_shift: np.ndarray):
        pbs = self._get_attr(model, "paths_by_side")
        if not isinstance(pbs, dict) or not pbs:
            _LOG.info("Keine paths_by_side im Recipe – überspringe Pfad-Render.")
            return

        for side, entry in pbs.items():
            if not isinstance(entry, dict):
                continue

            pts = entry.get("points")
            if pts is None:
                pts = entry.get("polyline")
            if pts is None:
                _LOG.debug("Side '%s' ohne points/polyline – überspringe.", side)
                continue

            try:
                arr = np.asarray(pts, dtype=float)
                if arr.ndim != 2 or arr.shape[1] not in (2, 3) or arr.shape[0] < 2:
                    _LOG.warning("Side '%s': ungültiges Punkteformat.", side)
                    continue
                if arr.shape[1] == 2:  # XY → XY0
                    z = np.zeros((arr.shape[0], 1), dtype=float)
                    arr = np.hstack([arr, z])

                # Substrat-Lokal → Ground-Shift
                arr = arr + ground_shift[None, :]

                # → Mount-Frame (Rotation) → Welt (Translation)
                arr = (arr @ R_mount.T) + t_mount[None, :]

                line = pv.lines_from_points(arr, close=False)
                self._engine.add_mesh(line, color="yellow", line_width=2.0, opacity=1.0)
            except Exception as e:
                _LOG.error("Side '%s' Pfad-Render fehlgeschlagen: %s", side, e, exc_info=True)

    # ---------- Helpers ----------
    @staticmethod
    def _get_attr(model: Any, key: str):
        if hasattr(model, key):
            return getattr(model, key)
        if isinstance(model, dict):
            return model.get(key)
        return None


def _rpy_deg_to_matrix_np(rpy_deg):
    import math
    r, p, y = [math.radians(float(v)) for v in (rpy_deg or (0.0, 0.0, 0.0))]
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])
    return Rz @ Ry @ Rx
