# -*- coding: utf-8 -*-
# File: src/tabs/recipe/coating_preview_panel/tab2d/matplot2d.py
"""
Matplot2DView (STRICT for Spraycoater v1 preview):

2D view consumes ONLY:
  - substrate_mesh: PyVista mesh already in substrate-frame (origin at mount-top reference)
  - path_xyz: final path points (postprocessed) in the SAME substrate-frame

No bounds-based shifting, no mask, no tcp, no legacy fallbacks.
"""

from __future__ import annotations

import logging
from typing import Optional, Tuple, Dict, Any, List

import numpy as np
import matplotlib

# Intended to run inside Qt GUI
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavToolbar
from matplotlib.figure import Figure
from matplotlib.collections import PolyCollection
from matplotlib.lines import Line2D
from matplotlib.ticker import MultipleLocator

_LOG = logging.getLogger("tabs.recipe.preview.matplot2d")

PLANES = ("top", "front", "back", "left", "right")


def _project_xyz_to_plane(P: np.ndarray, plane: str) -> np.ndarray:
    """Project (N,3) XYZ -> (N,2) for the plane."""
    if plane == "top":
        return P[:, [0, 1]]          # X,Y
    if plane in ("front", "back"):
        return P[:, [0, 2]]          # X,Z
    if plane in ("left", "right"):
        return P[:, [1, 2]]          # Y,Z
    raise ValueError(f"Unknown plane '{plane}'")


def _vtk_faces_to_tris(faces: np.ndarray) -> np.ndarray:
    """
    Convert VTK/PyVista faces array to triangle indices (M,3).
    Assumes mesh is triangulated upstream (or we ignore non-tris).
    """
    faces = np.asarray(faces, dtype=np.int64).ravel()
    tris: List[List[int]] = []
    i = 0
    N = faces.size
    while i < N:
        n = int(faces[i])
        i += 1
        if n == 3 and i + 3 <= N:
            tris.append([int(faces[i]), int(faces[i + 1]), int(faces[i + 2])])
        i += n
    return np.array(tris, dtype=np.int64) if tris else np.empty((0, 3), dtype=np.int64)


def _sanitize_points(P: Any) -> np.ndarray:
    A = np.asarray(P, dtype=float).reshape(-1, 3)
    if A.size == 0:
        return np.zeros((0, 3), dtype=float)
    m = np.isfinite(A).all(axis=1)
    return A[m]


class Matplot2DView(FigureCanvas):
    """
    Matplotlib canvas for simplified 2D preview (substrate + FINAL path).

    Interactions:
      - Mouse wheel zoom
      - Left-click pan
    """

    def __init__(self, parent=None):
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100, layout="constrained")
        super().__init__(self._fig)
        self.setParent(parent)

        self._ax = self._fig.add_subplot(111)
        # Use adjustable="box" to avoid aspect-vs-limits warnings.
        self._ax.set_aspect("equal", adjustable="box")

        # Navigation state
        self._dragging = False
        self._drag_start = (0.0, 0.0)
        self._drag_xlim = (0.0, 1.0)
        self._drag_ylim = (0.0, 1.0)

        # User-defined limits (None => autoscale)
        self._user_limits: Optional[Tuple[float, float, float, float]] = None

        self._legend_pad_frac = 0.18
        self._plane: str = "top"

        # Data (already in substrate-frame)
        self._path_xyz: Optional[np.ndarray] = None
        self._mesh_pts: Optional[np.ndarray] = None
        self._mesh_tris: Optional[np.ndarray] = None

        self._path2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}
        self._mesh2d: Dict[str, Optional[Tuple[np.ndarray, np.ndarray]]] = {p: None for p in PLANES}

        self._style = {
            "grid_alpha_major": 0.35,
            "substrate_face": "#d0d6dd",
            "substrate_edge": "#aaaaaa",
            "substrate_alpha": 0.65,
            "path_color": "#2ecc71",
            "path_lw": 1.8,
            "start_face": "#5dade2",
            "start_edge": "#2e86c1",
            "end_face": "#1b4f72",
            "end_edge": "#154360",
            "marker_size": 28.0,
        }

        self._connect_events()
        self._apply_layout_margins()

    def _connect_events(self) -> None:
        self.mpl_connect("scroll_event", self._on_scroll)
        self.mpl_connect("button_press_event", self._on_press)
        self.mpl_connect("button_release_event", self._on_release)
        self.mpl_connect("motion_notify_event", self._on_motion)

    # -------------------------
    # Interaction handlers
    # -------------------------

    def _on_scroll(self, event) -> None:
        if event.inaxes != self._ax or event.xdata is None or event.ydata is None:
            return

        base_scale = 1.15
        if event.button == "up":
            scale = 1.0 / base_scale
        elif event.button == "down":
            scale = base_scale
        else:
            return

        xlim = self._ax.get_xlim()
        ylim = self._ax.get_ylim()
        xdata, ydata = float(event.xdata), float(event.ydata)

        new_xlim = (xdata + (xlim[0] - xdata) * scale, xdata + (xlim[1] - xdata) * scale)
        new_ylim = (ydata + (ylim[0] - ydata) * scale, ydata + (ylim[1] - ydata) * scale)

        self._ax.set_xlim(new_xlim)
        self._ax.set_ylim(new_ylim)
        self._user_limits = (new_xlim[0], new_xlim[1], new_ylim[0], new_ylim[1])
        self.draw_idle()

    def _on_press(self, event) -> None:
        if event.button == 1 and event.inaxes == self._ax:
            self._dragging = True
            self._drag_start = (float(event.x), float(event.y))
            self._drag_xlim = self._ax.get_xlim()
            self._drag_ylim = self._ax.get_ylim()

    def _on_motion(self, event) -> None:
        if not self._dragging or event.inaxes != self._ax:
            return

        dx_pix = float(event.x) - self._drag_start[0]
        dy_pix = float(event.y) - self._drag_start[1]

        bbox = self._ax.get_window_extent()
        if bbox.width == 0 or bbox.height == 0:
            return

        dx_data_range = self._drag_xlim[1] - self._drag_xlim[0]
        dy_data_range = self._drag_ylim[1] - self._drag_ylim[0]

        scale_x = dx_data_range / bbox.width
        scale_y = dy_data_range / bbox.height

        dx_data = -dx_pix * scale_x
        dy_data = -dy_pix * scale_y

        new_xlim = (self._drag_xlim[0] + dx_data, self._drag_xlim[1] + dx_data)
        new_ylim = (self._drag_ylim[0] + dy_data, self._drag_ylim[1] + dy_data)

        self._ax.set_xlim(new_xlim)
        self._ax.set_ylim(new_ylim)
        self._user_limits = (new_xlim[0], new_xlim[1], new_ylim[0], new_ylim[1])
        self.draw_idle()

    def _on_release(self, event) -> None:
        if event.button == 1:
            self._dragging = False

    # -------------------------
    # Public API
    # -------------------------

    def make_toolbar(self, parent=None):
        tb = NavToolbar(self, parent)
        self.toolbar = tb
        return tb

    def dispose(self) -> None:
        self._fig.clf()
        self.setParent(None)
        self.deleteLater()

    def set_plane(self, plane: str) -> None:
        if plane not in PLANES:
            raise ValueError(f"Unknown plane '{plane}'")
        self._plane = plane
        self._user_limits = None
        self._redraw(keep_limits=False)

    def get_plane(self) -> str:
        return self._plane

    def refresh(self) -> None:
        self._redraw(keep_limits=True)

    # -------------------------
    # Data setters (strict)
    # -------------------------

    def _set_path_xyz_data(self, path_xyz: np.ndarray | None) -> None:
        if path_xyz is None:
            self._path_xyz = None
            for p in PLANES:
                self._path2d[p] = None
            return

        P = _sanitize_points(path_xyz)
        self._path_xyz = P
        for p in PLANES:
            self._path2d[p] = _project_xyz_to_plane(P, p) if P.size else None

    def _set_mesh_data(self, mesh: Any | None) -> None:
        if mesh is None or not hasattr(mesh, "points") or not hasattr(mesh, "faces"):
            self._mesh_pts = None
            self._mesh_tris = None
            for p in PLANES:
                self._mesh2d[p] = None
            return

        P = _sanitize_points(mesh.points)
        tris = _vtk_faces_to_tris(np.asarray(mesh.faces))

        # If not triangulated, we treat as absent (strict: 2D needs triangles)
        if P.size == 0 or tris.size == 0:
            self._mesh_pts = None
            self._mesh_tris = None
            for p in PLANES:
                self._mesh2d[p] = None
            return

        self._mesh_pts = P
        self._mesh_tris = tris

        for p in PLANES:
            V2 = _project_xyz_to_plane(P, p)
            self._mesh2d[p] = (V2, tris)

    def set_path_xyz(self, path_xyz: np.ndarray | None) -> None:
        self._set_path_xyz_data(path_xyz)
        self._redraw(keep_limits=True)

    def set_scene(self, *, substrate_mesh=None, path_xyz=None, **_kwargs) -> None:
        """
        STRICT scene update:
          - substrate_mesh: PyVista mesh in substrate-frame
          - path_xyz: FINAL path in substrate-frame
        """
        self._set_mesh_data(substrate_mesh)
        self._set_path_xyz_data(path_xyz)
        self._redraw(keep_limits=self._user_limits is not None)

    def show_top(self):   self.set_plane("top")
    def show_front(self): self.set_plane("front")
    def show_back(self):  self.set_plane("back")
    def show_left(self):  self.set_plane("left")
    def show_right(self): self.set_plane("right")

    # -------------------------
    # Rendering
    # -------------------------

    def _apply_layout_margins(self) -> None:
        engine = self._fig.get_layout_engine()
        if engine is not None:
            engine.set(w_pad=0.02, h_pad=0.08, hspace=0.02, wspace=0.02)

    def _set_labels(self) -> None:
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

    def _extents_from_data(self) -> Tuple[float, float, float, float]:
        xs: List[np.ndarray] = []
        ys: List[np.ndarray] = []

        m = self._mesh2d.get(self._plane)
        if m is not None:
            V2, _tris = m
            if V2 is not None and V2.size:
                xs.append(V2[:, 0])
                ys.append(V2[:, 1])

        U = self._path2d.get(self._plane)
        if U is not None and U.size:
            xs.append(U[:, 0])
            ys.append(U[:, 1])

        if not xs:
            # strict empty view extents
            return (-100.0, 100.0, -100.0, 100.0)

        all_x = np.concatenate(xs)
        all_y = np.concatenate(ys)
        xmin, xmax = float(np.min(all_x)), float(np.max(all_x))
        ymin, ymax = float(np.min(all_y)), float(np.max(all_y))

        dx = (xmax - xmin) * 0.1 if xmax != xmin else 10.0
        dy = (ymax - ymin) * 0.1 if ymax != ymin else 10.0
        return (xmin - dx, xmax + dx, ymin - dy, ymax + dy)

    def _configure_grid(self) -> None:
        self._ax.xaxis.set_major_locator(MultipleLocator(10.0))
        self._ax.yaxis.set_major_locator(MultipleLocator(10.0))
        self._ax.grid(True, which="major", alpha=self._style["grid_alpha_major"], linestyle=":", linewidth=0.8)

    def _draw_substrate(self) -> None:
        item = self._mesh2d.get(self._plane)
        if item is None:
            return
        V2, tris = item
        if V2 is None or tris is None or tris.size == 0:
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

    def _draw_path(self) -> None:
        U = self._path2d.get(self._plane)
        if U is None or U.size == 0:
            return

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

    def _add_legend(self) -> None:
        handles = [
            Line2D([0], [0], marker="s", linestyle="None", markersize=10,
                   markerfacecolor=self._style["substrate_face"],
                   markeredgecolor=self._style["substrate_edge"],
                   label="Substrate"),
            Line2D([0], [0], color=self._style["path_color"], lw=self._style["path_lw"], label="Path"),
            Line2D([0], [0], marker="o", linestyle="None", markersize=8,
                   markerfacecolor=self._style["start_face"], markeredgecolor=self._style["start_edge"], label="Start"),
            Line2D([0], [0], marker="o", linestyle="None", markersize=8,
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

    def _redraw(self, *, keep_limits: bool = False) -> None:
        try:
            saved = self._user_limits if (keep_limits and self._user_limits is not None) else None

            self._ax.clear()
            self._apply_layout_margins()
            self._set_labels()
            self._ax.set_aspect("equal", adjustable="box")

            if saved is None:
                x0, x1, y0, y1 = self._extents_from_data()
            else:
                x0, x1, y0, y1 = saved

            self._ax.set_xlim(x0, x1)
            self._ax.set_ylim(y0, y1)

            self._configure_grid()
            self._draw_substrate()
            self._draw_path()
            self._add_legend()

        except Exception:
            _LOG.exception("Matplot2DView._redraw failed")
        finally:
            self.draw_idle()
