# -*- coding: utf-8 -*-
import numpy as np
import pyvista as pv
from pyvistaqt import QtInteractor
from PyQt5 import QtWidgets

# (Optional) falls du später Mesh-Erzeugung/-Import brauchst:
# from .mesh_utils import generate_mesh, import_mesh_from_stl


class RobotPathVisualizer(QtWidgets.QWidget):
    """
    Eingebettetes PyVista-Widget: links einfache Controls (Shape/Path/Normals),
    rechts die 3D-Ansicht via QtInteractor. Keine extra Fenster (kein BackgroundPlotter).
    """
    def __init__(self, parent=None):
        super().__init__(parent)

        # --- Layout Grundaufbau ---
        root = QtWidgets.QHBoxLayout(self)
        controls = QtWidgets.QVBoxLayout()
        root.addLayout(controls, 0)

        # 3D Viewer (eingebettet)
        self.plotter = QtInteractor(self)
        root.addWidget(self.plotter, 1)

        # --- Controls wie in deiner Vorlage ---
        self.shape_combo = QtWidgets.QComboBox()
        self.shape_combo.addItems(["Würfel", "Kugel", "Zylinder"])
        controls.addWidget(self.shape_combo)

        self.path_combo = QtWidgets.QComboBox()
        self.path_combo.addItems(["Raster", "Spiral"])
        controls.addWidget(self.path_combo)

        self.show_normals_cb = QtWidgets.QCheckBox("Normalen anzeigen")
        controls.addWidget(self.show_normals_cb)

        generate_btn = QtWidgets.QPushButton("Pfad generieren")
        generate_btn.clicked.connect(self.build_plot)
        controls.addWidget(generate_btn)

        controls.addStretch(1)

        # Initiale Szene
        self.build_plot()

    # ----------------- Szene aufbauen -----------------
    def build_plot(self):
        shape = self.shape_combo.currentText()
        path_type = self.path_combo.currentText()
        show_normals = self.show_normals_cb.isChecked()

        self.plotter.clear()

        if shape == "Würfel":
            mesh = pv.Cube(center=(0, 0, 0), x_length=2, y_length=2, z_length=2)
            path_points = (
                self.generate_raster_path_on_cube(mesh)
                if path_type == "Raster" else
                self.generate_spiral_path_on_cube(mesh)
            )

        elif shape == "Kugel":
            mesh = pv.Sphere(radius=1.0, center=(0, 0, 0), theta_resolution=60, phi_resolution=60)
            path_points = (
                self.generate_raster_path_on_sphere(mesh)
                if path_type == "Raster" else
                self.generate_spiral_path_on_sphere(mesh)
            )

        elif shape == "Zylinder":
            mesh = pv.Cylinder(center=(0, 0, 0), direction=(0, 0, 1), radius=1.0, height=2.0, resolution=100)
            path_points = (
                self.generate_raster_path_on_cylinder(mesh)
                if path_type == "Raster" else
                self.generate_spiral_path_on_cylinder(mesh)
            )
        else:
            return

        self.plotter.add_mesh(mesh, color='lightgray', opacity=0.3)
        self.plotter.add_points(path_points, color='red', point_size=8, render_points_as_spheres=True)

        if show_normals:
            normals = []
            for point in path_points:
                if shape == "Kugel":
                    normal = point / np.linalg.norm(point)
                elif shape == "Würfel":
                    normal = np.sign(point)
                    # normieren (0-Fälle vermeiden)
                    n = np.linalg.norm(normal)
                    normal = normal / n if n > 0 else np.array([0.0, 0.0, 1.0])
                elif shape == "Zylinder":
                    direction = np.array([point[0], point[1], 0.0])
                    norm = np.linalg.norm(direction)
                    normal = direction / norm if norm > 0 else np.array([0.0, 0.0, 1.0])
                else:
                    normal = np.array([0.0, 0.0, 1.0])
                normals.append(normal)
            normals = np.array(normals)

            arrows = pv.PolyData(path_points)
            arrows["vectors"] = normals
            arrows_glyphs = arrows.glyph(orient="vectors", scale=False, factor=0.15)
            self.plotter.add_mesh(arrows_glyphs, color="green")

        # Kamera nett setzen
        self.plotter.view_isometric()
        self.plotter.reset_camera()

    # ----------------- Pfad-Generatoren -----------------
    def generate_raster_path_on_cube(self, mesh):
        x_vals = np.linspace(-1, 1, 20)
        y_vals = np.linspace(-1, 1, 20)
        z = 1.0
        points = []
        for i, y in enumerate(y_vals):
            row = [np.array([x, y, z]) for x in (x_vals if i % 2 == 0 else x_vals[::-1])]
            points.extend(row)
        return np.array(points)

    def generate_spiral_path_on_cube(self, mesh):
        theta = np.linspace(0, 4 * np.pi, 200)
        r = np.linspace(0.2, 1.0, 200)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        z = np.ones_like(x)
        return np.vstack((x, y, z)).T

    def generate_raster_path_on_sphere(self, mesh):
        phi_vals = np.linspace(0.0, np.pi, 30)
        theta_vals = np.linspace(0.0, 2.0 * np.pi, 60)
        points = []
        for i, phi in enumerate(phi_vals):
            ring = [np.array([
                np.sin(phi) * np.cos(theta),
                np.sin(phi) * np.sin(theta),
                np.cos(phi)
            ]) for theta in (theta_vals if i % 2 == 0 else theta_vals[::-1])]
            points.extend(ring)
        return np.array(points)

    def generate_spiral_path_on_sphere(self, mesh):
        t = np.linspace(0, 6 * np.pi, 400)
        z = np.linspace(-1, 1, 400)
        r = np.sqrt(np.clip(1 - z**2, 0.0, 1.0))
        x = r * np.cos(t)
        y = r * np.sin(t)
        return np.vstack((x, y, z)).T

    def generate_raster_path_on_cylinder(self, mesh):
        height = np.linspace(-1, 1, 80)
        angles = np.linspace(0, 2 * np.pi, 120)
        points = []
        for i, z in enumerate(height):
            circle = [np.array([np.cos(a), np.sin(a), z])
                      for a in (angles if i % 2 == 0 else angles[::-1])]
            points.extend(circle)
        return np.array(points)

    def generate_spiral_path_on_cylinder(self, mesh):
        theta = np.linspace(0, 12 * np.pi, 600)
        z = np.linspace(-1, 1, 600)
        x = np.cos(theta)
        y = np.sin(theta)
        return np.vstack((x, y, z)).T
