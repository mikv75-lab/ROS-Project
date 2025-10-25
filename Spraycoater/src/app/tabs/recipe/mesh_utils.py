# -*- coding: utf-8 -*-
import os
import numpy as np
import pyvista as pv


def generate_mesh(shape: str) -> pv.PolyData:
    """
    Erzeugt ein einfaches Mesh basierend auf dem Schlüsselwort `shape`.
    Unterstützt: "Kugel", "Würfel", "Zylinder".
    """
    if shape == "Kugel":
        mesh = pv.Sphere(radius=5.0, center=(0, 0, 5), theta_resolution=60, phi_resolution=60)
    elif shape == "Würfel":
        mesh = pv.Cube(center=(0, 0, 5), x_length=10, y_length=10, z_length=10)
    elif shape == "Zylinder":
        mesh = pv.Cylinder(radius=5.0, height=10.0, center=(0, 0, 0), resolution=60)
    else:
        raise ValueError(f"Unbekannte Form: {shape}")
    return align_to_table(mesh)


def align_to_table(mesh: pv.PolyData) -> pv.PolyData:
    """
    Richtet das Mesh so aus, dass sein tiefster Punkt bei Z=0 liegt.
    """
    min_z = np.min(mesh.points[:, 2])
    mesh.translate((0, 0, -min_z))
    return mesh


def export_mesh_to_stl(mesh: pv.PolyData, output_path: str) -> None:
    mesh.save(output_path)


def import_mesh_from_stl(input_path: str) -> pv.PolyData:
    mesh = pv.read(input_path)
    return align_to_table(mesh)
