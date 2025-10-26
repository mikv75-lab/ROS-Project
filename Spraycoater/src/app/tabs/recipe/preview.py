# -*- coding: utf-8 -*-
from __future__ import annotations
import numpy as np
import pyvista as pv


class PreviewEngine:
    """
    Vorschau-Engine (nur Hemisphäre):
      - Shape: obere Hemisphäre (z >= 0)
      - Pfade: Meander (boustrophedon) oder Spiral
      - Zeichnet einen durchgehenden Polyline-Pfad + optional Punkte + Normals + Raycasts
      - flip=True = 180° Roll um X-Achse (Mesh, Punkte, Normale)
    """

    def __init__(self, plotter: pv.Plotter):
        self.plotter = plotter

    # ------------- public API -------------

    def view_iso(self) -> None:
        self.plotter.view_isometric()
        self.plotter.reset_camera()

    def view_top(self) -> None:
        self.plotter.view_xy()
        self.plotter.reset_camera()

    def view_front(self) -> None:
        self.plotter.view_yz()
        self.plotter.reset_camera()

    def view_right(self) -> None:
        self.plotter.view_xz()
        self.plotter.reset_camera()

    def build_preview(
        self,
        shape_name: str,
        path_name: str,
        show_normals: bool,
        *,
        flip: bool = False,
        show_rays: bool = False,
    ) -> None:
        """
        Rendert die Hemisphäre + Pfad.
        Unterstützt nur:
          shape_name ∈ {"Hemisphäre", "Hemisphere"}
          path_name  ∈ {"Meander","Spiral"}
        """
        self.plotter.clear()

        # --- 1) Mesh Hemisphäre ---
        mesh = self._make_hemisphere()

        # --- 2) Punkte + Normals für Hemisphäre ---
        if path_name.lower().startswith("mea"):
            pts = self._meander_on_hemisphere()
        else:
            pts = self._spiral_on_hemisphere()

        normals = self._compute_normals_sphere(pts)

        # --- 3) Flip (Roll 180° um X) ---
        if flip:
            mesh = mesh.copy(deep=True)
            try:
                mesh.rotate_x(180.0, inplace=True)
            except Exception:
                poly = mesh.extract_geometry().copy(deep=True)
                poly.rotate_x(180.0, inplace=True)
                mesh = poly
            pts = self._rot_x(pts, 180.0)
            normals = self._rot_x(normals, 180.0)

        # --- 4) Zeichnen: Mesh + Pfad als Polyline ---
        self.plotter.add_mesh(mesh, color="lightgray", opacity=0.3)

        # Polyline bauen (eine zusammenhängende Linie)
        line = pv.PolyData(pts)
        n = len(pts)
        if n >= 2:
            line.lines = np.hstack(([n], np.arange(n))).astype(np.int64)
            self.plotter.add_mesh(line, line_width=2)

        # Punkte als Spheres (dezent)
        self.plotter.add_points(pts, point_size=6, render_points_as_spheres=True)

        # Normals anzeigen?
        if show_normals:
            arrows = pv.PolyData(pts)
            arrows["vectors"] = normals
            glyphs = arrows.glyph(orient="vectors", scale=False, factor=0.12)
            self.plotter.add_mesh(glyphs)

        # Raycasts (kleine Linien entlang der Normale)?
        if show_rays:
            rays = self._normals_as_lines(pts, normals, length=0.25)
            self.plotter.add_mesh(rays, line_width=1)

        self.view_iso()

    # ------------- geometry helpers -------------

    @staticmethod
    def _rot_x(arr: np.ndarray, deg: float) -> np.ndarray:
        """Dreht Nx3-Vektoren um X-Achse (Roll) um 'deg' Grad."""
        if arr is None or len(arr) == 0:
            return arr
        rad = np.deg2rad(deg)
        c, s = np.cos(rad), np.sin(rad)
        R = np.array([[1.0, 0.0, 0.0],
                      [0.0,  c, -s ],
                      [0.0,  s,  c ]], dtype=float)
        return arr @ R.T

    def _make_hemisphere(self) -> pv.PolyData:
        # obere Hemisphäre (phi: 0..90°)
        return pv.Sphere(
            radius=1.0,
            theta_resolution=80,
            phi_resolution=40,
            start_theta=0.0,
            end_theta=360.0,
            start_phi=0.0,
            end_phi=90.0,
        )

    @staticmethod
    def _compute_normals_sphere(points: np.ndarray) -> np.ndarray:
        nrm = np.linalg.norm(points, axis=1, keepdims=True) + 1e-12
        return points / nrm

    @staticmethod
    def _sph_to_xyz(phi: np.ndarray, theta: np.ndarray) -> np.ndarray:
        # phi: 0..pi/2 (oben), theta: 0..2pi
        sp, cp = np.sin(phi), np.cos(phi)
        ct, st = np.cos(theta), np.sin(theta)
        x = sp * ct
        y = sp * st
        z = cp
        return np.vstack((x, y, z)).T

    def _meander_on_hemisphere(self) -> np.ndarray:
        """
        Boustrophedon auf der oberen Hemisphäre:
          - ϕ (lat) in 0..π/2 -> Ringe
          - θ (lon) 0..2π, jede zweite Zeile invertiert
        """
        rows = 30
        cols = 200
        phis = np.linspace(0.0, 0.5 * np.pi, rows)
        thetas = np.linspace(0.0, 2.0 * np.pi, cols)
        pts = []
        for i, phi in enumerate(phis):
            th = thetas if (i % 2 == 0) else thetas[::-1]
            ring = self._sph_to_xyz(np.full_like(th, phi), th)
            pts.extend(ring)
        return np.asarray(pts, dtype=float)

    def _spiral_on_hemisphere(self) -> np.ndarray:
        """
        Sphärische Spiral auf oberer Hemisphäre (ϕ nimmt zu, θ windet sich).
        """
        k = 6.0  # Windungsfaktor
        m = 1500
        phi = np.linspace(0.0, 0.5 * np.pi, m)
        theta = k * phi * (1.0 + 0.2 * np.sin(5.0 * phi))
        return self._sph_to_xyz(phi, theta)

    @staticmethod
    def _normals_as_lines(points: np.ndarray, normals: np.ndarray, *, length: float) -> pv.PolyData:
        """
        Erzeugt ein PolyData mit vielen kurzen Linien (Raycasts) entlang der Normale.
        VTK-Konvention: lines = [2,p0,p1, 2,p2,p3, ...] für Segmente.
        """
        N = len(points)
        all_pts = np.empty((2 * N, 3), dtype=float)
        all_pts[0::2] = points
        all_pts[1::2] = points + normals * float(length)

        # Jede Linie: 2-Punkte-Segment
        seg_ids = np.arange(0, 2 * N, dtype=np.int64)
        lines = np.empty((N, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = seg_ids[0::2]
        lines[:, 2] = seg_ids[1::2]
        lines = lines.reshape(-1)

        pd = pv.PolyData(all_pts)
        pd.lines = lines
        return pd