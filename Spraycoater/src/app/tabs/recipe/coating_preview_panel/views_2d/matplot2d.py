# app/tabs/recipe/coating_preview_panel/matplot2d.py
# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavToolbar
from matplotlib.figure import Figure
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D

_LOG = logging.getLogger("app.tabs.recipe.preview.matplot2d")

PLANES = ("top", "front", "back", "left", "right")


def _project_xyz_to_plane(P: np.ndarray, plane: str) -> np.ndarray:
    if plane == "top":
        return P[:, [0, 1]]          # X,Y
    if plane in ("front", "back"):
        return P[:, [0, 2]]          # X,Z
    if plane in ("left", "right"):
        return P[:, [1, 2]]          # Y,Z
    raise ValueError(f"Unknown plane '{plane}'")


def _vtk_faces_to_tris(faces: np.ndarray) -> np.ndarray:
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
    2D-Renderer: Substrat (grau) + Pfad (grün) + Start/End-Marker + Legende.
    KEINE Maske mehr.
    """
    def __init__(self, parent=None):
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100)
        super().__init__(self._fig)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_aspect("equal", adjustable="box")

        self._bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self._plane: str = "top"

        # Scene caches
        self._path_xyz: Optional[np.ndarray] = None
        self._mesh_pts: Optional[np.ndarray] = None
        self._mesh_tris: Optional[np.ndarray] = None

        # Pre-projected caches by plane
        self._path2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}
        self._mesh2d: Dict[str, Optional[Tuple[np.ndarray, np.ndarray]]] = {p: None for p in PLANES}

        # UI state
        self._user_limits: Optional[Tuple[float, float, float, float]] = None

        # style
        self._style = {
            "grid_alpha": 0.35,
            "grid_ls": ":",
            "substrate_face": "#d0d6dd",
            "substrate_edge": "#aaaaaa",
            "substrate_alpha": 0.65,
            "path_color": "#2ecc71",
            "path_lw": 1.8,
            "start_face": "#5dade2",  # hellblau
            "start_edge": "#2e86c1",
            "end_face":   "#1b4f72",  # dunkelblau
            "end_edge":   "#154360",
            "marker_size": 28.0,
        }

        self._fig.tight_layout()

    # ---------- Toolbar (optional für Einbau im Panel) ----------
    def make_toolbar(self, parent=None):
        try:
            return NavToolbar(self, parent)
        except Exception:
            return None

    # ---------- Public API ----------
    def set_plane(self, plane: str):
        if plane not in PLANES:
            _LOG.warning("Unknown plane '%s'", plane)
            return
        self._plane = plane
        self._user_limits = None
        self._redraw()

    def get_plane(self) -> str:
        return self._plane

    def refresh(self):
        self._redraw(keep_limits=True)

    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]):
        self._bounds = tuple(map(float, bounds))
        self._redraw(keep_limits=self._user_limits is not None)

    def set_path_xyz(self, path_xyz: np.ndarray | None):
        self._path_xyz = None if path_xyz is None else np.asarray(path_xyz, float).reshape(-1, 3)
        for p in PLANES:
            self._path2d[p] = None if self._path_xyz is None else _project_xyz_to_plane(self._path_xyz, p)
        self._redraw(keep_limits=True)

    def _set_mesh(self, mesh: Any | None):
        if mesh is None or not hasattr(mesh, "points"):
            self._mesh_pts = None
            self._mesh_tris = None
        else:
            try:
                P = np.asarray(mesh.points, dtype=float).reshape(-1, 3)
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
            self._bounds = tuple(map(float, bounds))
        self.set_path_xyz(path_xyz)
        self._set_mesh(substrate_mesh)
        self._redraw(keep_limits=True)

    def show_top(self):   self.set_plane("top")
    def show_front(self): self.set_plane("front")
    def show_back(self):  self.set_plane("back")
    def show_left(self):  self.set_plane("left")
    def show_right(self): self.set_plane("right")

    # ---------- drawing ----------
    def _set_labels(self):
        if self._plane == "top":
            self._ax.set_title("Top (Z in depth)")
            self._ax.set_xlabel("X (mm)"); self._ax.set_ylabel("Y (mm)")
        elif self._plane in ("front", "back"):
            self._ax.set_title(f"{self._plane.capitalize()} (Y in depth)")
            self._ax.set_xlabel("X (mm)"); self._ax.set_ylabel("Z (mm)")
        else:
            self._ax.set_title(f"{self._plane.capitalize()} (X in depth)")
            self._ax.set_xlabel("Y (mm)"); self._ax.set_ylabel("Z (mm)")

    def _extents_from_data(self) -> Tuple[float, float, float, float]:
        """
        Limits aus Mesh+Pfad (projiziert) – damit ist der Pfad auch sichtbar,
        wenn er z.B. auf Z=10 liegt, das Mesh aber nur 50..52 abdeckt.
        """
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        if self._plane == "top":
            lo = np.array([xmin, ymin], dtype=float)
            hi = np.array([xmax, ymax], dtype=float)
        elif self._plane in ("front", "back"):
            lo = np.array([xmin, zmin], dtype=float)
            hi = np.array([xmax, zmax], dtype=float)
        else:
            lo = np.array([ymin, zmin], dtype=float)
            hi = np.array([ymax, zmax], dtype=float)

        item = self._mesh2d.get(self._plane)
        if item:
            V2, _ = item
            if V2 is not None and len(V2) > 0:
                lo = np.minimum(lo, np.min(V2, axis=0))
                hi = np.maximum(hi, np.max(V2, axis=0))

        U = self._path2d.get(self._plane)
        if U is not None and len(U) > 0:
            lo = np.minimum(lo, np.min(U, axis=0))
            hi = np.maximum(hi, np.max(U, axis=0))

        span = np.maximum(hi - lo, 1e-6)
        pad = 0.05 * span
        lo -= pad; hi += pad
        return float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1])

    def _configure_grid(self):
        self._ax.grid(True, linestyle=self._style["grid_ls"],
                      alpha=self._style["grid_alpha"], linewidth=0.5)

    def _draw_substrate(self):
        item = self._mesh2d.get(self._plane)
        if not item:
            return
        V2, tris = item
        if V2 is None or tris is None or len(tris) == 0:
            return
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
        if U is None or len(U) == 0:
            return
        self._ax.plot(U[:, 0], U[:, 1],
                      color=self._style["path_color"],
                      linewidth=self._style["path_lw"],
                      zorder=2)
        # Start/End
        self._ax.scatter([U[0, 0]], [U[0, 1]],
                         s=self._style["marker_size"],
                         edgecolors=self._style["start_edge"],
                         facecolors=self._style["start_face"],
                         zorder=3)
        self._ax.scatter([U[-1, 0]], [U[-1, 1]],
                         s=self._style["marker_size"],
                         edgecolors=self._style["end_edge"],
                         facecolors=self._style["end_face"],
                         zorder=3)

    def _add_legend(self):
        handles = [
            Line2D([0], [0], marker='s', linestyle='None',
                   markersize=10, markerfacecolor=self._style["substrate_face"],
                   markeredgecolor=self._style["substrate_edge"], label="Substrate"),
            Line2D([0], [0], color=self._style["path_color"], lw=self._style["path_lw"], label="Path"),
            Line2D([0], [0], marker='o', linestyle='None', markersize=8,
                   markerfacecolor=self._style["start_face"], markeredgecolor=self._style["start_edge"], label="Start"),
            Line2D([0], [0], marker='o', linestyle='None', markersize=8,
                   markerfacecolor=self._style["end_face"], markeredgecolor=self._style["end_edge"], label="End"),
        ]
        self._ax.legend(handles=handles, loc="upper right", frameon=True)

    def _redraw(self, *, keep_limits: bool = False):
        try:
            saved = self._user_limits if keep_limits and (self._user_limits is not None) else None

            self._ax.clear()
            self._ax.set_aspect("equal", adjustable="box")
            self._set_labels()

            if saved is None:
                x0, x1, y0, y1 = self._extents_from_data()
                self._ax.set_xlim(x0, x1); self._ax.set_ylim(y0, y1)
            else:
                self._ax.set_xlim(saved[0], saved[1]); self._ax.set_ylim(saved[2], saved[3])

            self._configure_grid()
            self._draw_substrate()
            self._draw_path()
            self._add_legend()

            if saved is None:
                x0, x1 = self._ax.get_xlim(); y0, y1 = self._ax.get_ylim()
                self._user_limits = (x0, x1, y0, y1)

            self._fig.tight_layout()
            self.draw_idle()
        except Exception:
            _LOG.exception("Matplot2DView redraw failed")
