# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Tuple

import numpy as np
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.collections import PolyCollection

try:
    import pyvista as pv
except Exception:
    pv = None  # type: ignore

_LOG = logging.getLogger("app.tabs.recipe.preview.matplot2d")


def _project_points_xyz_to_plane(P: np.ndarray, plane: str) -> np.ndarray:
    if plane == "top":
        U = P[:, [0, 1]]
    elif plane in ("front", "back"):
        U = P[:, [0, 2]]
    elif plane in ("left", "right"):
        U = P[:, [1, 2]]
    else:
        raise ValueError(f"Unknown plane '{plane}'")
    return U.astype(float, copy=False)


def _polyline_from_polydata(poly: "pv.PolyData") -> Optional[np.ndarray]:
    if pv is None or poly is None:
        return None
    try:
        if poly.n_lines <= 0:
            return None
        lines = poly.lines.reshape(-1, 3)
        idxs = []
        for rec in lines:
            if int(rec[0]) != 2:
                continue
            idxs.extend([int(rec[1]), int(rec[2])])
        if not idxs:
            return None
        idxs = np.asarray(idxs, dtype=int)
        P = np.asarray(poly.points, dtype=float)
        return P[idxs]
    except Exception:
        _LOG.exception("Polyline extraction from PolyData failed")
        return None


class Matplot2DView(FigureCanvas):
    def __init__(self, parent=None):
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100)
        super().__init__(self._fig)
        # WICHTIG: kein setParent(parent)! Reparenting erfolgt durch addWidget(...)
        # self.setParent(parent)

        self._ax = self._fig.add_subplot(111)
        self._ax.set_aspect("equal", adjustable="box")

        self._bounds: Tuple[float, float, float, float, float, float] = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)
        self._plane: str = "top"

        self._mesh: Optional["pv.PolyData"] = None
        self._path_xyz: Optional[np.ndarray] = None
        self._mask_poly: Optional["pv.PolyData"] = None

        self._fig.tight_layout()

    # --- Public API ---
    def set_scene(
        self,
        *,
        substrate_mesh: "pv.PolyData | None",
        path_xyz: np.ndarray | None,
        bounds: Tuple[float, float, float, float, float, float] | None = None,
        mask_poly: "pv.PolyData | None" = None,
    ):
        self._mesh = substrate_mesh
        self._path_xyz = np.asarray(path_xyz, dtype=float) if path_xyz is not None else None
        self._mask_poly = mask_poly
        if bounds is not None:
            self._bounds = bounds
        self._redraw()

    def show_top(self):   self._set_plane("top")
    def show_front(self): self._set_plane("front")
    def show_back(self):  self._set_plane("back")
    def show_left(self):  self._set_plane("left")
    def show_right(self): self._set_plane("right")

    def force_redraw(self):
        self._redraw()

    # --- intern ---
    def _set_plane(self, plane: str):
        if plane not in ("top", "front", "back", "left", "right"):
            _LOG.warning("Unknown plane '%s'", plane); return
        self._plane = plane
        self._redraw()

    def _set_labels(self):
        if self._plane == "top":
            self._ax.set_title("Top (Z in Bildtiefe)")
            self._ax.set_xlabel("X (mm)")
            self._ax.set_ylabel("Y (mm)")
        elif self._plane == "front":
            self._ax.set_title("Front (Y in Bildtiefe)")
            self._ax.set_xlabel("X (mm)")
            self._ax.set_ylabel("Z (mm)")
        elif self._plane == "back":
            self._ax.set_title("Back (Y in Bildtiefe)")
            self._ax.set_xlabel("X (mm)")
            self._ax.set_ylabel("Z (mm)")
        elif self._plane == "left":
            self._ax.set_title("Left (X in Bildtiefe)")
            self._ax.set_xlabel("Y (mm)")
            self._ax.set_ylabel("Z (mm)")
        elif self._plane == "right":
            self._ax.set_title("Right (X in Bildtiefe)")
            self._ax.set_xlabel("Y (mm)")
            self._ax.set_ylabel("Z (mm)")

    def _set_limits_from_bounds(self):
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        if self._plane == "top":
            self._ax.set_xlim(xmin, xmax)
            self._ax.set_ylim(ymin, ymax)
        elif self._plane in ("front", "back"):
            self._ax.set_xlim(xmin, xmax)
            self._ax.set_ylim(zmin, zmax)
        else:  # left/right
            self._ax.set_xlim(ymin, ymax)
            self._ax.set_ylim(zmin, zmax)

    def _configure_grid(self):
        self._ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.5)

    def _projected_triangles(self, plane: str):
        if pv is None or self._mesh is None or self._mesh.n_points == 0:
            return None
        try:
            mesh = self._mesh
            try:
                mesh = mesh.extract_surface()
            except Exception:
                pass
            try:
                mesh = mesh.triangulate(copy=True)
            except Exception:
                pass

            faces = getattr(mesh, "faces", None)
            if faces is None:
                return None
            faces = np.asarray(faces)
            if faces.size == 0:
                return None

            P = np.asarray(mesh.points, dtype=float).reshape(-1, 3)
            U = _project_points_xyz_to_plane(P, plane)

            # VTK faces: [3, i, j, k, 3, i, j, k, ...]
            try:
                faces_r = faces.reshape(-1, 4)
                tri_rows = faces_r[:, 0] == 3
                if not np.any(tri_rows):
                    return None
                tris_idx = faces_r[tri_rows, 1:4]
            except Exception:
                cursor = 0
                idx_list = []
                f = faces
                n = f.size
                while cursor < n:
                    m = int(f[cursor]); cursor += 1
                    if m == 3 and cursor + 3 <= n:
                        idx_list.append([int(f[cursor]), int(f[cursor+1]), int(f[cursor+2])])
                    cursor += m
                if not idx_list:
                    return None
                tris_idx = np.asarray(idx_list, dtype=int)

            tris_2d = []
            for tri in tris_idx:
                if np.any(tri < 0) or np.any(tri >= U.shape[0]):
                    continue
                tri2d = U[tri]
                if np.all(np.isfinite(tri2d)):
                    tris_2d.append(tri2d)
            return tris_2d
        except Exception:
            _LOG.exception("projected triangles failed for plane=%s", plane)
            return None

    def _draw_mesh_fill(self):
        tris = self._projected_triangles(self._plane)
        if not tris:
            return
        try:
            coll = PolyCollection(
                tris,
                facecolor="#d0d6dd",  # hellgrau
                edgecolor="none",
                alpha=0.45,
                zorder=1,
            )
            self._ax.add_collection(coll)
        except Exception:
            _LOG.exception("draw mesh fill via triangles failed")

    def _draw_mask_poly(self):
        if pv is None or self._mask_poly is None:
            return
        try:
            P3 = _polyline_from_polydata(self._mask_poly)
            if P3 is None or len(P3) == 0:
                return
            U = _project_points_xyz_to_plane(P3, self._plane)
            self._ax.plot(
                U[:, 0], U[:, 1],
                linestyle="-", linewidth=1.0,
                color="#4169E1", alpha=0.9, zorder=3
            )
        except Exception:
            _LOG.exception("draw mask poly failed")

    def _draw_path(self):
        if self._path_xyz is None or len(self._path_xyz) == 0:
            return
        try:
            U = _project_points_xyz_to_plane(self._path_xyz, self._plane)
            self._ax.plot(
                U[:, 0], U[:, 1], "-",
                linewidth=2.0, color="#2ecc71",
                alpha=0.95, zorder=4
            )
        except Exception:
            _LOG.exception("draw path failed")

    def _redraw(self):
        try:
            self._ax.clear()
            self._ax.set_aspect("equal", adjustable="box")
            self._set_labels()
            self._set_limits_from_bounds()
            self._configure_grid()

            # Reihenfolge: Fläche -> Maske -> Pfad
            self._draw_mesh_fill()
            self._draw_mask_poly()
            self._draw_path()

            self._fig.tight_layout()
            self.draw_idle()
        except Exception:
            _LOG.exception("Matplot2DView redraw failed")
