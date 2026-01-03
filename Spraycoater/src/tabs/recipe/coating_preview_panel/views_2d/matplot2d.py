"""
This module contains a modified version of the Matplot2DView class used in
the coating preview panel.  The modifications adjust how Z‑coordinates are
handled so that the substrate sits at Z = 0 in all 2D views and the path
heights are displayed relative to the substrate.  Previously, the Z‑axis
assumed a fixed world coordinate range (0–200 mm) which resulted in the
substrate appearing at an offset (typically around 50 mm).  The changes
introduced here allow dynamic re‑anchoring of the Z‑axis based on the
currently loaded substrate bounds and path extents:

* A new attribute ``_z_offset`` stores the world‑space Z coordinate of the
  substrate base.  This offset is determined whenever new bounds are set via
  ``set_bounds``.
* When paths or meshes are provided, their Z coordinates are shifted by
  ``_z_offset`` so that they are plotted relative to the substrate base.
* The internal ``_world['z']`` range is updated to reflect the height of
  the substrate and any path above it, starting from zero.  This ensures the
  axes labels in front/back/left/right views read "0 mm" at the substrate
  rather than some arbitrary world height (e.g. 50 mm).

These adjustments align the 2D projection with the 3D scene where the
CubeAxes grid origin is likewise translated to the substrate base.  Users
should substitute this file for the original ``matplot2d.py`` in their
project to obtain the corrected behaviour.
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
    """Project an (N,3) array of XYZ points into a 2D plane depending
    on the selected view.  For top views, X and Y are used; for front/back,
    X and Z; and for left/right, Y and Z.

    Parameters
    ----------
    P : np.ndarray
        The (N,3) array of points.
    plane : str
        One of "top", "front", "back", "left", "right" specifying the
        projection plane.

    Returns
    -------
    np.ndarray
        An (N,2) array of projected coordinates.
    """
    if plane == "top":
        return P[:, [0, 1]]          # X,Y
    if plane in ("front", "back"):
        return P[:, [0, 2]]          # X,Z
    if plane in ("left", "right"):
        return P[:, [1, 2]]          # Y,Z
    raise ValueError(f"Unknown plane '{plane}'")


def _vtk_faces_to_tris(faces: np.ndarray) -> np.ndarray:
    """Convert a VTK/PyVista faces array into a list of triangle indices.

    PyVista stores faces in a flat array where each polygon is preceded by
    the number of vertices, followed by that many indices.  This helper
    extracts only faces with exactly three vertices (triangles) and returns
    them as a (M,3) integer array.

    Parameters
    ----------
    faces : np.ndarray
        The faces array from a PyVista mesh (k0,i0,i1,i2,k1,i3,i4,i5,...).

    Returns
    -------
    np.ndarray
        An (M,3) array of triangle vertex indices, or an empty array if
        no triangles were found.
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
    """
    A Matplotlib canvas for a simplified 2D preview of 3D coating data.  It
    draws the substrate mesh as filled polygons, the spray path as a green
    polyline, and start/end markers.  The class caches projected 2D data
    for the different viewing planes and supports switching between views.

    This modified version introduces a Z‑offset to ensure that the substrate
    appears at Z = 0 in front/back/left/right views and that the path heights
    are plotted relative to the substrate base.  Without this offset, the
    Z‑axis tick labels and path positions reflected the raw world coordinates
    (e.g. the substrate might appear at 50 mm).  The offset is updated
    whenever new bounds are supplied via ``set_bounds``, and the path/mesh
    points are shifted accordingly when set via ``set_path_xyz`` or
    ``_set_mesh``.
    """

    def __init__(self, parent=None):
        # Create a figure with constrained layout so that subplots (single
        # axes) use available space efficiently.  DPI 100 yields crisp
        # rendering in most Qt applications.
        self._fig: Figure = Figure(figsize=(6, 6), dpi=100, layout="constrained")
        super().__init__(self._fig)
        self.setParent(parent)

        # Single Axes for the 2D view; maintain equal aspect ratio so that
        # distances on both axes are measured in millimetres.  Navigation
        # is enabled (pan/zoom with right/middle mouse by default).
        self._ax = self._fig.add_subplot(111)
        self._ax.set_aspect("equal", adjustable="box")
        self._ax.set_navigate(True)

        # Extra vertical space below the axes so that the legend fits
        # comfortably without overlapping the plot.
        self._legend_pad_frac = 0.18

        # World coordinate extents (min,max) for each axis.  These values
        # define the default plot limits when no data is present.  The
        # Z‑range is updated dynamically based on the scene bounds and the
        # Z‑offset.
        self._world = {
            "x": (-120.0, 120.0),
            "y": (-120.0, 120.0),
            "z": (0.0, 200.0),
        }

        # Store the complete bounds of the current scene (world coordinates).
        # These are updated via ``set_bounds`` and used to derive the
        # Z‑offset and to recalculate the world extents.  The tuple is
        # (xmin, xmax, ymin, ymax, zmin, zmax).
        self._bounds: Tuple[float, float, float, float, float, float] = (
            self._world["x"][0], self._world["x"][1],
            self._world["y"][0], self._world["y"][1],
            self._world["z"][0], self._world["z"][1],
        )

        # Viewing plane currently selected.  One of the entries from PLANES.
        self._plane: str = "top"

        # Cache of 3D data for the current scene.  ``_path_xyz`` holds an
        # (N,3) array of points for the spray path.  ``_mesh_pts`` and
        # ``_mesh_tris`` store the substrate mesh geometry.  These values
        # reflect the *world coordinates* of the data (not offset).
        self._path_xyz: Optional[np.ndarray] = None
        self._mesh_pts: Optional[np.ndarray] = None
        self._mesh_tris: Optional[np.ndarray] = None

        # Cache of pre‑projected 2D data per plane.  Each entry is
        # recalculated when the underlying 3D points change or when the
        # Z‑offset changes.  For the mesh, the entry is a tuple (V2, tris)
        # where V2 is (M,2) projected vertices and tris is (K,3) indices.
        self._path2d: Dict[str, Optional[np.ndarray]] = {p: None for p in PLANES}
        self._mesh2d: Dict[str, Optional[Tuple[np.ndarray, np.ndarray]]] = {p: None for p in PLANES}

        # User limits (xmin,xmax,ymin,ymax) are stored when the user
        # interacts with the axes (e.g. zoom/pan).  When refreshing or
        # switching planes, these limits are restored if present.  A value
        # of None indicates that the plot should be reset to the default
        # extents derived from the world coordinate range.
        self._user_limits: Optional[Tuple[float, float, float, float]] = None

        # Style parameters for grid, colors, line widths and marker sizes.
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

        # Initialize Z‑offset to zero.  When bounds are set for the first
        # time, this value is updated to the world coordinate of the
        # substrate base (zmin).  All subsequent path and mesh points will
        # be shifted by this amount for plotting.
        self._z_offset: float = 0.0

        # Apply initial layout margins for a bit of breathing room around
        # the axes.  Without this call, elements like tick labels can
        # overlap the edges of the figure.
        self._apply_layout_margins()

    # ---------- Toolbar (optional) ----------
    def make_toolbar(self, parent=None):
        """Create an optional Matplotlib toolbar for the canvas."""
        tb = NavToolbar(self, parent)
        self.toolbar = tb
        return tb

    # ---------- Public API ----------
    def dispose(self):
        """Clean up the canvas, toolbar and figure (for Qt lifetime)."""
        tb = getattr(self, "toolbar", None)
        if tb is not None:
            tb.setParent(None)
            tb.deleteLater()
            self.toolbar = None  # type: ignore[assignment]

        self._fig.clf()
        self.setParent(None)
        self.deleteLater()

    def set_plane(self, plane: str):
        """Switch the view to a different projection plane and redraw."""
        if plane not in PLANES:
            _LOG.warning("Unknown plane '%s'", plane)
            return
        self._plane = plane
        keep = self._user_limits is not None
        self._redraw(keep_limits=keep)

    def get_plane(self) -> str:
        """Return the currently active plane."""
        return self._plane

    def refresh(self):
        """Force a redraw, preserving current axis limits."""
        self._redraw(keep_limits=True)

    def set_bounds(self, bounds: Tuple[float, float, float, float, float, float]):
        """
        Store the scene bounds and update the Z‑offset.  The bounds are
        expected to be world coordinate values (xmin,xmax,ymin,ymax,zmin,zmax).
        The Z‑offset is set to zmin so that subsequent plotting operations
        display the substrate at Z=0.  The internal world Z range is
        redefined to start at zero and extend to zmax - zmin.  A redraw is
        triggered to update the plot.
        """
        # Store the raw bounds
        self._bounds = tuple(map(float, bounds))
        # Unpack for clarity
        xmin, xmax, ymin, ymax, zmin, zmax = self._bounds
        # Update Z‑offset to align the substrate base with Z=0
        self._z_offset = zmin
        # Update world extents: Z starts at 0 and goes to the height of the
        # scene above the substrate
        self._world["z"] = (0.0, max(10.0, float(zmax - zmin)))
        # Trigger a redraw (preserve user limits if set)
        self._redraw(keep_limits=self._user_limits is not None)

    def set_path_xyz(self, path_xyz: np.ndarray | None):
        """
        Store the path as an (N,3) array and compute its 2D projections.  If
        the path is not None, Z coordinates are shifted by the current
        Z‑offset so that the path is plotted relative to the substrate base.

        Parameters
        ----------
        path_xyz : np.ndarray or None
            The path points in world coordinates.  If None, any cached
            path data will be cleared.
        """
        # Store the raw world coordinates
        if path_xyz is not None:
            P = np.asarray(path_xyz, float).reshape(-1, 3).copy()
            # Shift Z by the offset so that Z=0 corresponds to substrate base
            P[:, 2] -= self._z_offset
            self._path_xyz = P
        else:
            self._path_xyz = None
        # Invalidate cached projections
        for p in PLANES:
            self._path2d[p] = None if self._path_xyz is None else _project_xyz_to_plane(self._path_xyz, p)
        self._redraw(keep_limits=True)

    def _set_mesh(self, mesh: Any | None):
        """
        Extract points and triangle indices from a PyVista mesh and update
        the cached 2D projections.  Z coordinates are shifted by the
        current Z‑offset before projection.  If no mesh is provided, any
        existing mesh data is cleared.
        """
        if mesh is None or not hasattr(mesh, "points"):
            self._mesh_pts = None
            self._mesh_tris = None
        else:
            try:
                # Copy points so that we can modify them without affecting
                # the original mesh.  Reshape to ensure (N,3) shape.
                P = np.asarray(mesh.points, dtype=float).reshape(-1, 3).copy()
                # Subtract Z‑offset from world coordinates
                P[:, 2] -= self._z_offset
                tris = None
                if hasattr(mesh, "faces"):
                    tris = _vtk_faces_to_tris(np.asarray(mesh.faces))
                self._mesh_pts = P
                # Only store triangles if they are present and non‑empty
                self._mesh_tris = tris if tris is not None and tris.size else None
            except Exception:
                _LOG.exception("Failed to extract triangles from substrate mesh")
                self._mesh_pts = None
                self._mesh_tris = None
        # Update 2D cache
        for p in PLANES:
            if self._mesh_pts is None or self._mesh_tris is None:
                self._mesh2d[p] = None
            else:
                V2 = _project_xyz_to_plane(self._mesh_pts, p)
                self._mesh2d[p] = (V2, self._mesh_tris)

    def set_scene(self, *, substrate_mesh=None, path_xyz=None, bounds=None, **_kwargs):
        """
        Convenience method to set multiple scene components at once.  You
        can pass any combination of ``bounds``, ``path_xyz`` and
        ``substrate_mesh``.  All three are optional.
        """
        if bounds is not None:
            self.set_bounds(bounds)
        # Order matters: set path and mesh after updating bounds (so they
        # use the latest Z‑offset).
        self.set_path_xyz(path_xyz)
        self._set_mesh(substrate_mesh)
        self._redraw(keep_limits=True)

    # Convenience functions to switch planes
    def show_top(self):   self.set_plane("top")
    def show_front(self): self.set_plane("front")
    def show_back(self):  self.set_plane("back")
    def show_left(self):  self.set_plane("left")
    def show_right(self): self.set_plane("right")

    # ---------- layout & labels ----------
    def _apply_layout_margins(self):
        """Fine‑tune the constrained layout (padding/hspacing)."""
        engine = self._fig.get_layout_engine()
        if engine is not None:
            engine.set(w_pad=0.02, h_pad=0.08, hspace=0.02, wspace=0.02)

    def _set_labels(self):
        """Set the title and axis labels according to the current view."""
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
        """
        Return default plot limits for the current plane.  These are derived
        from the internal ``_world`` range.  For front/back/left/right
        views, the Z range uses the dynamic values updated by ``set_bounds``.
        """
        X = self._world["x"]
        Y = self._world["y"]
        Z = self._world["z"]
        if self._plane == "top":
            return X[0], X[1], Y[0], Y[1]
        if self._plane in ("front", "back"):
            return X[0], X[1], Z[0], Z[1]
        # left/right planes use Y for horizontal and Z for vertical
        return Y[0], Y[1], Z[0], Z[1]

    def _extents_from_data(self) -> Tuple[float, float, float, float]:
        """
        Determine plot limits based on stored data.  Currently this
        implementation simply uses the fixed world extents, but could be
        extended to compute extents from the actual data points if
        desired.
        """
        return self._fixed_plane_extents()

    # ---------- drawing ----------
    def _configure_grid(self):
        """Configure the major/minor grid lines with 10 mm and 1 mm spacing."""
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
        """
        Draw the substrate mesh as a PolyCollection in the current 2D
        projection.  If no mesh or triangles exist, nothing is drawn.
        """
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
        """
        Draw the path polyline and the start/end markers.  Path points
        have been shifted by Z‑offset in ``set_path_xyz``.
        """
        U = self._path2d.get(self._plane)
        if U is None or len(U) == 0:
            return
        self._ax.plot(
            U[:, 0], U[:, 1],
            color=self._style["path_color"],
            linewidth=self._style["path_lw"],
            zorder=2,
        )
        # Marker size calculation: convert the marker area into a diameter in
        # points squared, then scale for better visibility.
        ms = (self._style["marker_size"] ** 0.5) * 1.2
        # Start marker
        self._ax.plot(
            [U[0, 0]], [U[0, 1]],
            marker="o",
            linestyle="None",
            markersize=ms,
            markerfacecolor=self._style["start_face"],
            markeredgecolor=self._style["start_edge"],
            zorder=3,
        )
        # End marker
        self._ax.plot(
            [U[-1, 0]], [U[-1, 1]],
            marker="o",
            linestyle="None",
            markersize=ms,
            markerfacecolor=self._style["end_face"],
            markeredgecolor=self._style["end_edge"],
            zorder=3,
        )

    def _add_legend(self):
        """
        Add a legend beneath the axes with four columns: Substrate, Path,
        Start, End.  The location is anchored just below the axes using
        ``bbox_to_anchor`` and the legend pad fraction.
        """
        handles = [
            Line2D([0], [0], marker='s', linestyle='None',
                   markersize=10,
                   markerfacecolor=self._style["substrate_face"],
                   markeredgecolor=self._style["substrate_edge"],
                   label="Substrate"),
            Line2D([0], [0],
                   color=self._style["path_color"],
                   lw=self._style["path_lw"],
                   label="Path"),
            Line2D([0], [0], marker='o', linestyle='None', markersize=8,
                   markerfacecolor=self._style["start_face"],
                   markeredgecolor=self._style["start_edge"],
                   label="Start"),
            Line2D([0], [0], marker='o', linestyle='None', markersize=8,
                   markerfacecolor=self._style["end_face"],
                   markeredgecolor=self._style["end_edge"],
                   label="End"),
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
        """
        Redraw the scene.  This method clears the axes, sets up labels,
        grid lines, draws the substrate and path, and updates the legend.
        If ``keep_limits`` is True and user limits exist, those limits are
        preserved; otherwise, the axes limits are reset to the default
        extents derived from ``_world``.
        """
        try:
            saved = self._user_limits if keep_limits and (self._user_limits is not None) else None
            # Clear current axes contents
            self._ax.clear()
            self._ax.set_aspect("equal", adjustable="box")
            self._apply_layout_margins()
            self._set_labels()
            # Set limits either from saved user settings or default extents
            if saved is None:
                x0, x1, y0, y1 = self._extents_from_data()
                self._ax.set_xlim(x0, x1)
                self._ax.set_ylim(y0, y1)
            else:
                self._ax.set_xlim(saved[0], saved[1])
                self._ax.set_ylim(saved[2], saved[3])
            # Configure the grid
            self._configure_grid()
            # Draw the substrate and path
            self._draw_substrate()
            self._draw_path()
            # Add a legend below the plot
            self._add_legend()
            # Save limits if not already saved (when no user limits exist)
            if saved is None:
                x0, x1 = self._ax.get_xlim()
                y0, y1 = self._ax.get_ylim()
                self._user_limits = (x0, x1, y0, y1)
        except Exception:
            _LOG.exception("Matplot2DView._redraw failed")
        finally:
            # Always trigger a redraw of the canvas
            self.draw_idle()