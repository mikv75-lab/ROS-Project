# Spraycoater/src/app/widgets/path_visualizer.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets

class RobotPathVisualizer(QtWidgets.QWidget):
    """
    Minimaler Pfad-Viewer (mm):
      - set_path(points_mm[, normals]) und render()
      - Optionen: Polyline, Punkte, Kreuze, Start/Ende (Größen einstellbar)
      - Keine Pfad-Generierung, keine Farben gesetzt
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        root = QtWidgets.QHBoxLayout(self)
        controls = QtWidgets.QVBoxLayout()
        root.addLayout(controls, 0)

        # Optionen
        self.cb_polyline   = QtWidgets.QCheckBox("Polyline verbinden"); self.cb_polyline.setChecked(True)
        self.cb_points     = QtWidgets.QCheckBox("Punkte anzeigen");     self.cb_points.setChecked(True)
        self.cb_crosses    = QtWidgets.QCheckBox("Kreuze je Punkt");     self.cb_crosses.setChecked(True)
        self.cb_startend   = QtWidgets.QCheckBox("Start/Ende markieren");self.cb_startend.setChecked(True)
        self.cb_normals    = QtWidgets.QCheckBox("Normalen anzeigen");   self.cb_normals.setChecked(False)

        controls.addWidget(self.cb_polyline)
        controls.addWidget(self.cb_points)
        controls.addWidget(self.cb_crosses)
        controls.addWidget(self.cb_startend)
        controls.addWidget(self.cb_normals)

        # Größen
        controls.addWidget(QtWidgets.QLabel("Kreuzgröße [mm]"))
        self.spin_cross = QtWidgets.QDoubleSpinBox(); self.spin_cross.setDecimals(2); self.spin_cross.setRange(0.1, 1000.0); self.spin_cross.setValue(1.0)
        controls.addWidget(self.spin_cross)

        controls.addWidget(QtWidgets.QLabel("Start/Ende Größe [mm]"))
        self.spin_marker = QtWidgets.QDoubleSpinBox(); self.spin_marker.setDecimals(2); self.spin_marker.setRange(0.1, 5000.0); self.spin_marker.setValue(2.0)
        controls.addWidget(self.spin_marker)

        btn_render = QtWidgets.QPushButton("Render")
        btn_render.clicked.connect(self.render)
        controls.addWidget(btn_render)
        controls.addStretch(1)

        # 3D-Viewer
        self.plotter = QtInteractor(self)
        root.addWidget(self.plotter, 1)

        # Daten
        self._points_mm = np.zeros((0,3), dtype=float)
        self._normals   = None  # optional Nx3

    # -------- API --------
    def set_path(self, points_mm, normals=None):
        self._points_mm = np.asarray(points_mm, dtype=float).reshape(-1, 3)
        if normals is None:
            self._normals = None
        else:
            N = np.asarray(normals, dtype=float).reshape(-1, 3)
            self._normals = N if len(N) == len(self._points_mm) else None

    def clear(self):
        self._points_mm = np.zeros((0,3), dtype=float)
        self._normals = None
        self.plotter.clear()

    def render(self):
        pts = self._points_mm
        self.plotter.clear()
        if pts.size == 0:
            self.plotter.view_isometric(); self.plotter.reset_camera(); return

        # Polyline
        if self.cb_polyline.isChecked() and len(pts) >= 2:
            line = pv.PolyData(pts)
            n = len(pts)
            line.lines = np.hstack(([n], np.arange(n, dtype=np.int64)))
            self.plotter.add_mesh(line, line_width=2)

        # Punkte
        if self.cb_points.isChecked():
            self.plotter.add_points(pts, point_size=6, render_points_as_spheres=True)

        # Kreuze (XY)
        if self.cb_crosses.isChecked():
            s = float(self.spin_cross.value())
            seg_pts = np.empty((len(pts) * 4, 3), dtype=float)
            lines = np.empty((len(pts) * 2, 3), dtype=np.int64)
            for i, p in enumerate(pts):
                seg_pts[4*i+0] = p + np.array([ s, 0, 0])
                seg_pts[4*i+1] = p - np.array([ s, 0, 0])
                seg_pts[4*i+2] = p + np.array([0,  s, 0])
                seg_pts[4*i+3] = p - np.array([0,  s, 0])
                lines[2*i+0] = [2, 4*i+0, 4*i+1]
                lines[2*i+1] = [2, 4*i+2, 4*i+3]
            cross_pd = pv.PolyData(seg_pts)
            cross_pd.lines = lines.reshape(-1)
            self.plotter.add_mesh(cross_pd, line_width=1)

        # Start/Ende Marker
        if self.cb_startend.isChecked():
            m = float(self.spin_marker.value())
            if len(pts) >= 1:
                # Start: Kegel entlang Anfangstangente (Fallback +X)
                if len(pts) >= 2:
                    t0 = pts[1] - pts[0]
                    n = np.linalg.norm(t0)
                    t0 = t0 / (n if n > 0 else 1.0)
                else:
                    t0 = np.array([1.0, 0.0, 0.0])
                start_cone = pv.Cone(center=pts[0], direction=t0, height=3.0*m, radius=m)
                self.plotter.add_mesh(start_cone)
            if len(pts) >= 1:
                end_cube = pv.Cube(center=pts[-1], x_length=2.0*m, y_length=2.0*m, z_length=2.0*m)
                self.plotter.add_mesh(end_cube)

        # Normals (optional)
        if self.cb_normals.isChecked() and self._normals is not None and len(self._normals) == len(pts):
            arrows = pv.PolyData(pts)
            arrows["vectors"] = self._normals
            glyphs = arrows.glyph(orient="vectors", scale=False, factor=0.15)
            self.plotter.add_mesh(glyphs)

        # Kamera
        self.plotter.view_isometric()
        self.plotter.reset_camera()
