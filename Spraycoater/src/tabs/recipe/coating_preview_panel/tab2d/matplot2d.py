# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/matplot2d.py
"""
This module contains the Matplot2DView class used in the coating preview panel.
It handles Z-offsets to display the substrate at Z=0 relative to world coordinates.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import matplotlib

# Use the QtAgg backend because this module is intended to run inside a Qt GUI
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavToolbar
from matplotlib.figure import Figure
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator

_LOG = logging.getLogger("tabs.recipe.preview.matplot2d")

# Supported 2D projection planes for the 3D data
PLANES = ("top", "front", "back", "left", "right")


def _project_xyz_to_plane(P: np.ndarray, plane: str) -> np.ndarray:
    """Project an (N,3) array of XYZ points into a 2D plane."""
    if plane == "top":
        return P[:, [0, 1]]          # X,Y
    if plane in ("front", "back"):
        return P[:, [0, 2]]          # X,Z
    if plane in ("left", "right"):
        return P[:, [1, 2]]          # Y,Z
    raise ValueError(f"Unknown plane '{plane}'")


def _vtk_faces_to_tris(faces: np.ndarray) -> np.ndarray:
    """Convert a VTK/PyVista faces array into a list of triangle indices."""
    faces = np.asarray(faces, dtype=np.int64).ravel()
    tris: List[List[int]] = []
    i = 0
    N = faces.size
    while i < N:
        n = int(faces[i]); i += 1
        if n == 3 and i + 3 <= N:
            tris.append([int(faces[i]), int(faces[i+1]), int(faces[i+2])])
        i += n
    return np.array(tris, dtype=np.int64) if tris else np.empty((0, 3), dtype=np.int64)


class Matplot2DView(FigureCanvas):
    """
    A Matplotlib canvas for a simplified 2D preview of 3D coating data.
    """

    def __init__(self, parent=None):
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100, layout="constrained")
        super().__init__(self._fig)
        self.setParent(parent)

        self._ax = self._fig.add_subplot(111)
        self._ax.set_aspect("equal", adjustable="box")
        self._ax.set_navigate(True)

        self._legend_pad_frac = 0.18

        self._world = {
            "x": (-120.0, 120.0),
            "y": (-120.0, 120.0),
            "z": (0.0, 200.0),
        }

        self._bounds: Tuple[float, float, float, float, float, float] = (
            self._world["x"][0], self._world["x"][1],
            self._world["y"][0], self._world["y"][1],
            self._world["z"][0], self._world["z"][1],
        )

        self._plane: str = "top"
        self._path_xyz: Optional[np.ndarray] = None
        self._mesh_pts: Optional[np.ndarray] = None
        self._mesh_tris: Optional[np.ndarray] = None

        self._path2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}
        self._mesh2d: Dict[str, Optional[Tuple[np.ndarray, np.ndarray]]] = {p: None for p in PLANES}

        self._user_limits: Optional[Tuple[float, float, float, float]] = None

        self._style = {
            "grid_alpha_major": 0.35,
            "grid_alpha_minor": 0.15,
            "substrate_face": "#d0d6dd",
            "substrate_edge": "#aaaaaa",
            "substrate_alpha": 0.65,
            "path_color": "#2ecc71",
            "path_lw": 1.8,
            "start_face": "#5dade2",
            "start_edge": "#2e86c1",
            "end_face":   "#1b4f72",
            "end_edge":   "#154360",
            "marker_size": 28.0,
        }

        self._z_offset: float = 0.0
        self._apply_layout_margins()

    def make_toolbar(self, parent=None):
        tb = NavToolbar(self, parent)
        self.toolbar = tb
        return tb

    def dispose(self):
        tb = getattr(self, "toolbar", None)
        if tb is not None:
            tb.setParent(None)
            tb.deleteLater()
            self.toolbar = None

        self._fig.clf()
        self.setParent(None)
        self.deleteLater()

    def set_plane(self, plane: str):
        if plane not in PLANES:
            _LOG.warning("Unknown plane '%s'", plane)
            return
        self._plane = plane
        keep = self._user_limits is not None
        self._redraw(keep_limits=keep)

    def get_plane(self) -> str:
        return self._plane

    def refresh(self):
        self._redraw(keep_limits=True)

    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]):
        self._bounds = tuple(map(float, bounds))
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        self._z_offset = zmin
        self._world["z"] = (0.0, max(10.0, float(zmax - zmin)))
        self._redraw(keep_limits=self._user_limits is not None)

    def set_path_xyz(self, path_xyz: np.ndarray | None):
        if path_xyz is not None:
            P = np.asarray(path_xyz, float).reshape(-1, 3).copy()
            P[:, 2] -= self._z_offset
            self._path_xyz = P
        else:
            self._path_xyz = None
        for p in PLANES:
            self._path2d[p] = None if self._path_xyz is None else _project_xyz_to_plane(self._path_xyz, p)
        self._redraw(keep_limits=True)

    def _set_mesh(self, mesh: Any | None):
        if mesh is None or not hasattr(mesh, "points"):
            self._mesh_pts = None
            self._mesh_tris = None
        else:
            try:
                P = np.asarray(mesh.points, dtype=float).reshape(-1, 3).copy()
                P[:, 2] -= self._z_offset
                tris = None
                if hasattr(mesh, "faces"):
                    tris = _vtk_faces_to_tris(np.asarray(mesh.faces))
                self._mesh_pts = P
                self._mesh_tris = tris if tris is not None and tris.size else None
            except Exception:
                _LOG.exception("Failed to extract triangles from substrate mesh")
                self._mesh_pts = None
                self._mesh_tris = None
        for p in PLANES:
            if self._mesh_pts is None or self._mesh_tris is None:
                self._mesh2d[p] = None
            else:
                V2 = _project_xyz_to_plane(self._mesh_pts, p)
                self._mesh2d[p] = (V2, self._mesh_tris)

    def set_scene(self, *, substrate_mesh=None, path_xyz=None, bounds=None, **_kwargs):
        if bounds is not None:
            self.set_bounds(bounds)
        self.set_path_xyz(path_xyz)
        self._set_mesh(substrate_mesh)
        self._redraw(keep_limits=True)

    def show_top(self):   self.set_plane("top")
    def show_front(self): self.set_plane("front")
    def show_back(self):  self.set_plane("back")
    def show_left(self):  self.set_plane("left")
    def show_right(self): self.set_plane("right")

    def _apply_layout_margins(self):
        engine = self._fig.get_layout_engine()
        if engine is not None:
            engine.set(w_pad=0.02, h_pad=0.08, hspace=0.02, wspace=0.02)

    def _set_labels(self):
        if self._plane == "top":
            self._ax.set_title("Top (Z in depth)")
            self._ax.set_xlabel("X (mm)")
            self._ax.set_ylabel("Y (mm)")
        elif self._plane in ("front", "back"):
            self._ax.set_title(f"{self._plane.capitalize()} (Y in depth)")
            self._ax.set_xlabel("X (mm)")
            self._ax.set_ylabel("Z (mm)")
        else:
            self._ax.set_title(f"{self._plane.capitalize()} (X in depth)")
            self._ax.set_xlabel("Y (mm)")
            self._ax.set_ylabel("Z (mm)")

    def _fixed_plane_extents(self) -> Tuple[float, float, float, float]:
        X = self._world["x"]
        Y = self._world["y"]
        Z = self._world["z"]
        if self._plane == "top":
            return X[0], X[1], Y[0], Y[1]
        if self._plane in ("front", "back"):
            return X[0], X[1], Z[0], Z[1]
        return Y[0], Y[1], Z[0], Z[1]

    def _extents_from_data(self) -> Tuple[float, float, float, float]:
        return self._fixed_plane_extents()

    def _configure_grid(self):
        self._ax.xaxis.set_major_locator(MultipleLocator(10.0))
        self._ax.yaxis.set_major_locator(MultipleLocator(10.0))
        self._ax.xaxis.set_minor_locator(MultipleLocator(1.0))
        self._ax.yaxis.set_minor_locator(MultipleLocator(1.0))
        self._ax.grid(True, which="major",
                      alpha=self._style["grid_alpha_major"],
                      linestyle=":", linewidth=0.8)
        self._ax.grid(True, which="minor",
                      alpha=self._style["grid_alpha_minor"],
                      linestyle=":", linewidth=0.5)

    def _draw_substrate(self):
        item = self._mesh2d.get(self._plane)
        if not item: return
        V2, tris = item
        if V2 is None or tris is None or len(tris) == 0: return
        polys = [V2[idx] for idx in tris]
        pc = PolyCollection(
            polys,
            facecolor=self._style["substrate_face"],
            edgecolor=self._style["substrate_edge"],
            linewidths=0.3,
            alpha=self._style["substrate_alpha"],
            zorder=0,
        )
        self._ax.add_collection(pc)

    def _draw_path(self):
        U = self._path2d.get(self._plane)
        if U is None or len(U) == 0: return
        self._ax.plot(
            U[:, 0], U[:, 1],
            color=self._style["path_color"],
            linewidth=self._style["path_lw"],
            zorder=2,
        )
        ms = (self._style["marker_size"] ** 0.5) * 1.2
        self._ax.plot(
            [U[0, 0]], [U[0, 1]],
            marker="o", linestyle="None", markersize=ms,
            markerfacecolor=self._style["start_face"],
            markeredgecolor=self._style["start_edge"],
            zorder=3,
        )
        self._ax.plot(
            [U[-1, 0]], [U[-1, 1]],
            marker="o", linestyle="None", markersize=ms,
            markerfacecolor=self._style["end_face"],
            markeredgecolor=self._style["end_edge"],
            zorder=3,
        )

    def _add_legend(self):
        handles = [
            Line2D([0], [0], marker='s', linestyle='None', markersize=10,
                   markerfacecolor=self._style["substrate_face"],
                   markeredgecolor=self._style["substrate_edge"],
                   label="Substrate"),
            Line2D([0], [0], color=self._style["path_color"], lw=self._style["path_lw"], label="Path"),
            Line2D([0], [0], marker='o', linestyle='None', markersize=8,
                   markerfacecolor=self._style["start_face"], markeredgecolor=self._style["start_edge"], label="Start"),
            Line2D([0], [0], marker='o', linestyle='None', markersize=8,
                   markerfacecolor=self._style["end_face"], markeredgecolor=self._style["end_edge"], label="End"),
        ]
        self._ax.legend(
            handles=handles,
            loc="upper center",
            bbox_to_anchor=(0.5, -0.02 - self._legend_pad_frac),
            ncol=4,
            frameon=True,
            borderaxespad=0.0,
            handlelength=2.0,
            columnspacing=1.2,
        )

    def _redraw(self, *, keep_limits: bool = False):
        try:
            saved = self._user_limits if keep_limits and (self._user_limits is not None) else None
            self._ax.clear()
            self._ax.set_aspect("equal", adjustable="box")
            self._apply_layout_margins()
            self._set_labels()
            if saved is None:
                x0, x1, y0, y1 = self._extents_from_data()
                self._ax.set_xlim(x0, x1)
                self._ax.set_ylim(y0, y1)
            else:
                self._ax.set_xlim(saved[0], saved[1])
                self._ax.set_ylim(saved[2], saved[3])
            self._configure_grid()
            self._draw_substrate()
            self._draw_path()
            self._add_legend()
            if saved is None:
                x0, x1 = self._ax.get_xlim()
                y0, y1 = self._ax.get_ylim()
                self._user_limits = (x0, x1, y0, y1)
        except Exception:
            _LOG.exception("Matplot2DView._redraw failed")
        finally:
            self.draw_idle()