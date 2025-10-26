# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Sequence
import numpy as np
import pyvista as pv
from PyQt5 import QtWidgets
from pyvistaqt import QtInteractor


class PathVisualizer(QtWidgets.QWidget):
    """
    Reiner Visualizer für **Pfad in mm**.
    - Kein Generieren, keine Defaults, keine Fallbacks.
    - Zeichnet Polyline + Punkte.
    - Optional: kleine 'K'-Frames (x/y/z-Achsen) an jedem Punkt – dafür werden
      **normals (z-Achse)** und **tangents (x-Achse)** benötigt. y = z x x.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        self.plotter = QtInteractor(self)
        layout.addWidget(self.plotter)

    # ---------- public ----------
    def clear(self):
        self.plotter.clear()

    def view_iso(self):
        self.plotter.view_isometric()
        self.plotter.reset_camera()

    def render_polyline_mm(
        self,
        points_mm: Sequence[Sequence[float]],
        *,
        show_line: bool,
        point_size: int,
        show_frames: bool = False,
        normals: Optional[Sequence[Sequence[float]]] = None,
        tangents: Optional[Sequence[Sequence[float]]] = None,
        frame_scale_mm: Optional[float] = None,
        append: bool = False,
    ) -> None:
        """
        points_mm: Nx3 (mm)
        show_frames=True erfordert: normals, tangents, frame_scale_mm.
        """
        P = np.asarray(points_mm, dtype=float).reshape(-1, 3)
        if P.size == 0:
            if not append:
                self.plotter.clear()
            return

        if not append:
            self.plotter.clear()

        n = len(P)
        if show_line and n >= 2:
            line = pv.PolyData(P)
            line.lines = np.hstack(([n], np.arange(n, dtype=np.int64))).astype(np.int64)
            self.plotter.add_mesh(line, line_width=2)

        self.plotter.add_points(P, point_size=int(point_size), render_points_as_spheres=True)

        if show_frames:
            if normals is None or tangents is None or frame_scale_mm is None:
                raise ValueError("show_frames=True benötigt normals, tangents und frame_scale_mm.")
            N = np.asarray(normals, dtype=float).reshape(-1, 3)
            T = np.asarray(tangents, dtype=float).reshape(-1, 3)
            if len(N) != n or len(T) != n:
                raise ValueError("Länge von normals/tangents passt nicht zur Punktzahl.")

            # z = normalisiert(N), x = T orth proj auf Ebene ⟂ z, y = z x x
            z = N / (np.linalg.norm(N, axis=1, keepdims=True) + 1e-12)
            Tproj = T - (np.sum(T * z, axis=1, keepdims=True)) * z
            x = Tproj / (np.linalg.norm(Tproj, axis=1, keepdims=True) + 1e-12)
            y = np.cross(z, x)
            y /= (np.linalg.norm(y, axis=1, keepdims=True) + 1e-12)

            s = float(frame_scale_mm)
            # Achsen-Segmente
            seg_x = self._segments_polydata(P, P + x * s)
            seg_y = self._segments_polydata(P, P + y * s)
            seg_z = self._segments_polydata(P, P + z * s)
            self.plotter.add_mesh(seg_x, line_width=2)  # Farbe via Theme/Renderer
            self.plotter.add_mesh(seg_y, line_width=2)
            self.plotter.add_mesh(seg_z, line_width=2)

        self.view_iso()

    # ---------- helpers ----------
    @staticmethod
    def _segments_polydata(A: np.ndarray, B: np.ndarray) -> pv.PolyData:
        assert len(A) == len(B)
        N = len(A)
        all_pts = np.empty((2 * N, 3), dtype=float)
        all_pts[0::2] = A
        all_pts[1::2] = B
        lines = np.empty((N, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(0, 2 * N, 2, dtype=np.int64)
        lines[:, 2] = np.arange(1, 2 * N, 2, dtype=np.int64)
        poly = pv.PolyData(all_pts)
        poly.lines = lines.reshape(-1)
        return poly
