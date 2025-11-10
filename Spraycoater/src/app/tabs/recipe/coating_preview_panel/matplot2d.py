# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Tuple, Dict

import numpy as np
import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

_LOG = logging.getLogger("app.tabs.recipe.preview.matplot2d")

PLANES = ("top", "front", "back", "left", "right")


def _project_points_xyz_to_plane(P: np.ndarray, plane: str) -> np.ndarray:
    if plane == "top":
        return P[:, [0, 1]]
    if plane in ("front", "back"):
        return P[:, [0, 2]]
    if plane in ("left", "right"):
        return P[:, [1, 2]]
    raise ValueError(f"Unknown plane '{plane}'")


class Matplot2DView(FigureCanvas):
    """Minimaler 2D-Renderer: zeigt ausschließlich den Pfad."""
    def __init__(self, parent=None):
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100)
        super().__init__(self._fig)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_aspect("equal", adjustable="box")

        self._bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self._plane: str = "top"
        self._path_xyz: Optional[np.ndarray] = None

        # Cache der projizierten Pfade je Ebene
        self._path2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}

        # UI-Zustand (Pan/Zoom)
        self._user_limits: Optional[Tuple[float, float, float, float]] = None

        self._style = {
            "grid_alpha": 0.4,
            "grid_ls": ":",
            "path_color": "#2ecc71",
            "path_lw": 1.8,
            "marker_size": 20.0,
            "marker_edge": "#1e824c",
            "marker_face": "#2ecc71",
        }

        self._fig.tight_layout()

    # ---------- Public API ----------
    def set_plane(self, plane: str):
        if plane not in PLANES:
            _LOG.warning("Unknown plane '%s'", plane)
            return
        self._plane = plane
        # plane-Wechsel: Limits verlieren (sonst projizierte Werte können off sein)
        self._user_limits = None
        self._redraw()

    def get_plane(self) -> str:
        return self._plane

    def redraw(self):
        """Öffentliche Neuzeichnung, behält Pan/Zoom falls vorhanden."""
        self._redraw(keep_limits=True)

    # Synonym, falls du in anderem Code `refresh()` nutzt:
    def refresh(self):
        self._redraw(keep_limits=True)

    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]):
        self._bounds = tuple(map(float, bounds))
        # Wenn Nutzer bereits gezoomt hat, lass es so – nur neu zeichnen.
        self._redraw(keep_limits=self._user_limits is not None)

    def set_path_xyz(self, path_xyz: np.ndarray | None):
        self._path_xyz = None if path_xyz is None else np.asarray(path_xyz, dtype=float).reshape(-1, 3)
        # Cache neu aufbauen
        for p in PLANES:
            self._path2d[p] = None if self._path_xyz is None else _project_points_xyz_to_plane(self._path_xyz, p)
        # Pfadwechsel: Pan/Zoom behalten, wenn Nutzer es gesetzt hat
        self._redraw(keep_limits=True)

    def set_scene(self, *, substrate_mesh, path_xyz, bounds=None, mask_poly=None, **_kwargs):
        # ignoriert Mesh/Mask – zeigt NUR den Pfad
        if bounds is not None:
            self._bounds = tuple(map(float, bounds))
        self.set_path_xyz(path_xyz)

    def show_top(self):   self.set_plane("top")
    def show_front(self): self.set_plane("front")
    def show_back(self):  self.set_plane("back")
    def show_left(self):  self.set_plane("left")
    def show_right(self): self.set_plane("right")

    # ---------- intern ----------
    def _set_labels(self):
        if self._plane == "top":
            self._ax.set_title("Top (Z in Bildtiefe)")
            self._ax.set_xlabel("X (mm)"); self._ax.set_ylabel("Y (mm)")
        elif self._plane in ("front", "back"):
            self._ax.set_title(f"{self._plane.capitalize()} (Y in Bildtiefe)")
            self._ax.set_xlabel("X (mm)"); self._ax.set_ylabel("Z (mm)")
        else:
            self._ax.set_title(f"{self._plane.capitalize()} (X in Bildtiefe)")
            self._ax.set_xlabel("Y (mm)"); self._ax.set_ylabel("Z (mm)")

    def _default_limits_from_bounds(self) -> Tuple[float, float, float, float]:
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        if self._plane == "top":
            return xmin, xmax, ymin, ymax
        if self._plane in ("front", "back"):
            return xmin, xmax, zmin, zmax
        return ymin, ymax, zmin, zmax

    def _configure_grid(self):
        self._ax.grid(True, linestyle=self._style["grid_ls"], alpha=self._style["grid_alpha"], linewidth=0.5)

    def _draw_path(self):
        U = self._path2d.get(self._plane)
        if U is None or len(U) == 0:
            return
        self._ax.plot(U[:, 0], U[:, 1], color=self._style["path_color"], linewidth=self._style["path_lw"])
        # Start/Ende
        self._ax.scatter([U[0, 0], U[-1, 0]], [U[0, 1], U[-1, 1]],
                         s=self._style["marker_size"],
                         edgecolors=self._style["marker_edge"],
                         facecolors=self._style["marker_face"],
                         zorder=3)

    def _redraw(self, *, keep_limits: bool = False):
        try:
            saved = self._user_limits if keep_limits and (self._user_limits is not None) else None

            self._ax.clear()
            self._ax.set_aspect("equal", adjustable="box")
            self._set_labels()

            if saved is None:
                x0, x1, y0, y1 = self._default_limits_from_bounds()
                self._ax.set_xlim(x0, x1); self._ax.set_ylim(y0, y1)
            else:
                self._ax.set_xlim(saved[0], saved[1]); self._ax.set_ylim(saved[2], saved[3])

            self._configure_grid()
            self._draw_path()

            if saved is None:
                x0, x1 = self._ax.get_xlim(); y0, y1 = self._ax.get_ylim()
                self._user_limits = (x0, x1, y0, y1)

            self._fig.tight_layout()
            self.draw_idle()
        except Exception:
            _LOG.exception("Matplot2DView redraw failed")
