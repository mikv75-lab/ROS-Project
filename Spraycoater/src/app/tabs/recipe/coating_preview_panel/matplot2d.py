# -*- coding: utf-8 -*-
# app/tabs/recipe/coating_preview_panel/matplot2d.py
from __future__ import annotations
import logging
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.collections import PolyCollection

_LOG = logging.getLogger("app.tabs.recipe.preview.matplot2d")

PLANES = ("top", "front", "back", "left", "right")


def _project_xyz_to_plane(P: np.ndarray, plane: str) -> np.ndarray:
    if plane == "top":
        return P[:, [0, 1]]
    if plane in ("front", "back"):
        return P[:, [0, 2]]
    if plane in ("left", "right"):
        return P[:, [1, 2]]
    raise ValueError(f"Unknown plane '{plane}'")


def _vtk_faces_to_tris(faces: np.ndarray) -> np.ndarray:
    """
    Decode a VTK faces array into a (M, 3) triangle index array.
    faces format: [n, i0, i1, i2, n, j0, j1, j2, ...]
    """
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
    """2D-Renderer mit Substrat-Fill, Pfad, Maskenlinie, Pan/Zoom."""
    def __init__(self, parent=None):
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100)
        super().__init__(self._fig)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_aspect("equal", adjustable="box")

        self._bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self._plane: str = "top"

        # Scene caches
        self._path_xyz: Optional[np.ndarray] = None
        self._mask_xyz: Optional[np.ndarray] = None
        self._mesh_pts: Optional[np.ndarray] = None      # (N, 3)
        self._mesh_tris: Optional[np.ndarray] = None     # (M, 3)

        # Pre-projected caches by plane
        self._path2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}
        self._mask2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}
        self._mesh2d: Dict[str, Optional[Tuple[np.ndarray, np.ndarray]]] = {p: None for p in PLANES}
        # -> mesh2d[plane] = (V2, Tris) where V2 is (N,2), Tris is (M,3) over V2

        # UI state (manual pan/zoom)
        self._user_limits: Optional[Tuple[float, float, float, float]] = None
        self._panning: bool = False
        self._pan_start_xy: Optional[Tuple[float, float]] = None
        self._pan_lim0: Optional[Tuple[float, float, float, float]] = None

        # style
        self._style = {
            "grid_alpha": 0.4,
            "grid_ls": ":",
            "substrate_face": "#d0d6dd",
            "substrate_edge": "#aaaaaa",
            "substrate_alpha": 0.6,
            "path_color": "#2ecc71",
            "path_lw": 1.8,
            "marker_size": 20.0,
            "marker_edge": "#1e824c",
            "marker_face": "#2ecc71",
            "mask_color": "#4b7bec",
            "mask_lw": 1.2,
        }

        # mouse interactions
        self.mpl_connect("scroll_event", self._on_scroll_zoom)
        self.mpl_connect("button_press_event", self._on_button_press)
        self.mpl_connect("button_release_event", self._on_button_release)
        self.mpl_connect("motion_notify_event", self._on_mouse_move)

        self._fig.tight_layout()

    # ---------- Public API ----------
    def set_plane(self, plane: str):
        if plane not in PLANES:
            _LOG.warning("Unknown plane '%s'", plane)
            return
        self._plane = plane
        # plane change: drop user limits (different axes)
        self._user_limits = None
        self._redraw()

    def get_plane(self) -> str:
        return self._plane

    def redraw(self):
        self._redraw(keep_limits=True)

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

    def _set_mask_xyz(self, mask_xyz: np.ndarray | None):
        self._mask_xyz = None if mask_xyz is None else np.asarray(mask_xyz, float).reshape(-1, 3)
        for p in PLANES:
            self._mask2d[p] = None if self._mask_xyz is None else _project_xyz_to_plane(self._mask_xyz, p)

    def _set_mesh(self, mesh: Any | None):
        """Accepts a pyvista.PolyData-like object with .points and .faces."""
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

        # refresh 2D caches
        for p in PLANES:
            if self._mesh_pts is None or self._mesh_tris is None:
                self._mesh2d[p] = None
            else:
                V2 = _project_xyz_to_plane(self._mesh_pts, p)
                self._mesh2d[p] = (V2, self._mesh_tris)

    def set_scene(self, *, substrate_mesh=None, path_xyz=None, bounds=None, mask_poly=None, **_kwargs):
        if bounds is not None:
            self._bounds = tuple(map(float, bounds))

        # path
        self.set_path_xyz(path_xyz)

        # substrate (mesh)
        self._set_mesh(substrate_mesh)

        # optional mask polyline → turn vtk poly lines into coordinates
        if mask_poly is not None and hasattr(mask_poly, "points") and hasattr(mask_poly, "lines"):
            try:
                pts = np.asarray(mask_poly.points, dtype=float).reshape(-1, 3)
                lines = np.asarray(mask_poly.lines, dtype=np.int64).ravel()
                # lines are encoded [n, i0, i1, n, j0, j1, ...] (n should be 2 for segments)
                segs = []
                i = 0
                N = lines.size
                while i < N:
                    n = int(lines[i]); i += 1
                    if n >= 2 and i + n <= N:
                        idx = lines[i:i+n]; i += n
                        # draw as contiguous polyline
                        segs.append(pts[idx])
                if segs:
                    self._set_mask_xyz(np.vstack(segs))
                else:
                    self._set_mask_xyz(None)
            except Exception:
                _LOG.exception("Failed to decode mask_poly for 2D")
                self._set_mask_xyz(None)
        else:
            self._set_mask_xyz(None)

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

    def _default_limits_from_bounds(self) -> Tuple[float, float, float, float]:
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        if self._plane == "top":
            return xmin, xmax, ymin, ymax
        if self._plane in ("front", "back"):
            return xmin, xmax, zmin, zmax
        return ymin, ymax, zmin, zmax

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
        # build polygons per triangle
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
        # Start/End markers
        self._ax.scatter([U[0, 0], U[-1, 0]],
                         [U[0, 1], U[-1, 1]],
                         s=self._style["marker_size"],
                         edgecolors=self._style["marker_edge"],
                         facecolors=self._style["marker_face"],
                         zorder=3)

    def _draw_mask(self):
        M = self._mask2d.get(self._plane)
        if M is None or len(M) == 0:
            return
        self._ax.plot(M[:, 0], M[:, 1],
                      color=self._style["mask_color"],
                      linewidth=self._style["mask_lw"],
                      zorder=1)

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
            # draw order: substrate (zorder 0) → mask (1) → path (2/3)
            self._draw_substrate()
            self._draw_mask()
            self._draw_path()

            if saved is None:
                x0, x1 = self._ax.get_xlim(); y0, y1 = self._ax.get_ylim()
                self._user_limits = (x0, x1, y0, y1)

            self._fig.tight_layout()
            self.draw_idle()
        except Exception:
            _LOG.exception("Matplot2DView redraw failed")

    # ---------- mouse interactions ----------
    def _on_scroll_zoom(self, ev):
        try:
            if ev.inaxes != self._ax:
                return
            # zoom towards cursor
            scale = 0.9 if ev.button == "up" else 1.1
            x0, x1 = self._ax.get_xlim(); y0, y1 = self._ax.get_ylim()
            cx, cy = ev.xdata, ev.ydata
            nx0 = cx + (x0 - cx) * scale; nx1 = cx + (x1 - cx) * scale
            ny0 = cy + (y0 - cy) * scale; ny1 = cy + (y1 - cy) * scale
            self._ax.set_xlim(nx0, nx1); self._ax.set_ylim(ny0, ny1)
            self._user_limits = (nx0, nx1, ny0, ny1)
            self.draw_idle()
        except Exception:
            _LOG.exception("scroll zoom failed")

    def _on_button_press(self, ev):
        # right button drag = pan
        if ev.inaxes == self._ax and ev.button == 3:
            self._panning = True
            self._pan_start_xy = (ev.xdata, ev.ydata)
            x0, x1 = self._ax.get_xlim(); y0, y1 = self._ax.get_ylim()
            self._pan_lim0 = (x0, x1, y0, y1)

    def _on_button_release(self, _ev):
        self._panning = False
        self._pan_start_xy = None
        self._pan_lim0 = None

    def _on_mouse_move(self, ev):
        if not self._panning or ev.inaxes != self._ax or self._pan_start_xy is None or self._pan_lim0 is None:
            return
        try:
            sx, sy = self._pan_start_xy
            dx = ev.xdata - sx
            dy = ev.ydata - sy
            x0, x1, y0, y1 = self._pan_lim0
            self._ax.set_xlim(x0 - dx, x1 - dx)
            self._ax.set_ylim(y0 - dy, y1 - dy)
            self._user_limits = (x0 - dx, x1 - dx, y0 - dy, y1 - dy)
            self.draw_idle()
        except Exception:
            _LOG.exception("pan move failed")
