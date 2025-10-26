# -*- coding: utf-8 -*-
import numpy as np
import pyvista as pv
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from pyvistaqt import QtInteractor


class ShapePreview:
    """
    Kapselt die PyVista-Ansicht:
    - initialisiert QtInteractor in einen beliebigen Container
    - bietet build_plot(...) & reset_camera()
    """
    def __init__(self, container: QWidget):
        if container.layout() is None:
            container.setLayout(QVBoxLayout(container))
        self.plotter = QtInteractor(container)
        container.layout().addWidget(self.plotter)

    # ---- Public API ----
    def reset_camera(self):
        self.plotter.view_isometric()
        self.plotter.reset_camera()

    def build_plot(self, *, shape: str, path_type: str, show_normals: bool):
        self.plotter.clear()

        if shape == "Würfel":
            mesh = pv.Cube(center=(0, 0, 0), x_length=2, y_length=2, z_length=2)
            pts = self._raster_on_cube() if path_type == "Raster" else self._spiral_on_cube()
        elif shape == "Kugel":
            mesh = pv.Sphere(radius=1.0, center=(0, 0, 0), theta_resolution=60, phi_resolution=60)
            pts = self._raster_on_sphere() if path_type == "Raster" else self._spiral_on_sphere()
        elif shape == "Zylinder":
            mesh = pv.Cylinder(center=(0, 0, 0), direction=(0, 0, 1), radius=1.0, height=2.0, resolution=100)
            pts = self._raster_on_cylinder() if path_type == "Raster" else self._spiral_on_cylinder()
        else:
            return

        self.plotter.add_mesh(mesh, color="lightgray", opacity=0.3)
        self.plotter.add_points(pts, color="red", point_size=8, render_points_as_spheres=True)

        if show_normals:
            normals = self._compute_normals(shape, pts)
            arrows = pv.PolyData(pts)
            arrows["vectors"] = normals
            glyphs = arrows.glyph(orient="vectors", scale=False, factor=0.15)
            self.plotter.add_mesh(glyphs, color="green")

        self.reset_camera()

    # ---- intern: simple Dummygeneratoren ----
    def _compute_normals(self, shape: str, points: np.ndarray) -> np.ndarray:
        normals = []
        for p in points:
            if shape == "Kugel":
                v = p / (np.linalg.norm(p) + 1e-12)
            elif shape == "Würfel":
                v = np.sign(p); n = np.linalg.norm(v); v = v / n if n > 0 else np.array([0.0,0.0,1.0])
            elif shape == "Zylinder":
                v2 = np.array([p[0], p[1], 0.0]); n = np.linalg.norm(v2); v = v2 / n if n > 0 else np.array([0.0,0.0,1.0])
            else:
                v = np.array([0.0, 0.0, 1.0])
            normals.append(v)
        return np.array(normals)

    def _raster_on_cube(self) -> np.ndarray:
        x = np.linspace(-1, 1, 20); y = np.linspace(-1, 1, 20); z = 1.0
        pts = []
        for i, yy in enumerate(y):
            row = [np.array([xx, yy, z]) for xx in (x if i % 2 == 0 else x[::-1])]
            pts.extend(row)
        return np.array(pts)

    def _spiral_on_cube(self) -> np.ndarray:
        th = np.linspace(0, 4*np.pi, 200); r = np.linspace(0.2, 1.0, 200)
        x = r*np.cos(th); y = r*np.sin(th); z = np.ones_like(x)
        return np.vstack((x,y,z)).T

    def _raster_on_sphere(self) -> np.ndarray:
        ph = np.linspace(0.0, np.pi, 30); th = np.linspace(0.0, 2*np.pi, 60)
        pts = []
        for i, p in enumerate(ph):
            ring = [np.array([np.sin(p)*np.cos(t), np.sin(p)*np.sin(t), np.cos(p)])
                    for t in (th if i % 2 == 0 else th[::-1])]
            pts.extend(ring)
        return np.array(pts)

    def _spiral_on_sphere(self) -> np.ndarray:
        t = np.linspace(0, 6*np.pi, 400); z = np.linspace(-1, 1, 400)
        r = np.sqrt(np.clip(1 - z*z, 0.0, 1.0))
        x = r*np.cos(t); y = r*np.sin(t)
        return np.vstack((x,y,z)).T

    def _raster_on_cylinder(self) -> np.ndarray:
        z = np.linspace(-1, 1, 120); ang = np.linspace(0, 2*np.pi, 160)
        pts = []
        for i, zz in enumerate(z):
            circle = [np.array([np.cos(a), np.sin(a), zz])
                      for a in (ang if i%2==0 else ang[::-versor(ang)])]
            # small helper to reverse every second line
            pts.extend(circle)
        return np.array(pts)

    def _spiral_on_cylinder(self) -> np.ndarray:
        th = np.linspace(0, 12*np.pi, 600); z = np.linspace(-1, 1, 600)
        x = np.cos(th); y = np.sin(th)
        return np.vstack((x,y,z)).T

def versor(arr):
    # returns reversed array (helper for readability)
    return arr[::-1]
