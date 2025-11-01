# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Tuple

_LOG = logging.getLogger("app.tabs.recipe.preview.grid")

class GridManager:
    """
    Verantwortlich NUR fürs Grid (Actor).
    - Kein ia.clear() mehr.
    - Spawnt/ersetzt nur den eigenen Grid-Actor.
    - Liefert Utility-Spawner für feste 240x240x240-Bounds auf dem Substratemount.
    """

    def __init__(self, interactor_getter: callable):
        self._get_ia = interactor_getter
        # Standard: 240x240x240 mit Boden bei z=0 (Mount-Ebene)
        self.bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self.step: float = 10.0   # aktuell nur informativ
        self._grid_actor = None
        self.visible: bool = True

    # ---- internes Zeichnen (nur Grid) ----
    def _draw_grid(self):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("_draw_grid(): kein Interactor")
            return
        try:
            # alten Grid entfernen
            if self._grid_actor is not None:
                try:
                    ia.remove_actor(self._grid_actor, render=False)
                except Exception:
                    pass
                self._grid_actor = None

            if not self.visible:
                ia.render()
                return

            # neuen Grid zeichnen (PyVista Axes-Grid)
            self._grid_actor = ia.show_grid(
                bounds=self.bounds,
                xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
                show_xaxis=True, show_yaxis=True, show_zaxis=True,
                show_xlabels=True, show_ylabels=True, show_zlabels=True,
                ticks='both', grid='back', render=False,
            )
            try:
                ax = self._grid_actor
                ax.SetShowEdges(False)
                ax.SetDrawXGridlines(True); ax.SetDrawYGridlines(True); ax.SetDrawZGridlines(True)
                ax.SetDrawXInnerGridlines(True); ax.SetDrawYInnerGridlines(True); ax.SetDrawZInnerGridlines(True)
                ax.SetUseTextActor3D(1)
            except Exception:
                pass

            ia.render()
        except Exception:
            _LOG.exception("_draw_grid() failed")

    # ---- öffentliche API (ohne Szene zu löschen!) ----
    def set_bounds(self, bounds):
        """Setzt neue Bounds und zeichnet den Grid-Actor neu (ohne andere Actors zu berühren)."""
        self.bounds = tuple(map(float, bounds))
        self._draw_grid()

    def set_grid(self, *, bounds=None, step: Optional[float] = None):
        """Komfort: Bounds/Step setzen → neu zeichnen."""
        if bounds is not None:
            self.bounds = tuple(map(float, bounds))
        if step is not None:
            self.step = float(step)
        self._draw_grid()

    def set_visible(self, visible: bool):
        """Grid-Actor ein-/ausblenden (andere Actors bleiben)."""
        self.visible = bool(visible)
        self._draw_grid()

    # ---- feste 240x240x240 Spawns auf Mount-Ebene ----
    def spawn_fixed_grid_at(self, *, center_xy=(0.0, 0.0), z0: float = 0.0,
                            span_xy: float = 240.0, span_z: float = 240.0, step: Optional[float] = None):
        """
        Zeichnet ein 240x240x240-Grid (Standard) mit Boden bei z0 (typisch 0 = Mount-Ebene),
        zentriert in X/Y um center_xy.
        """
        if step is not None:
            self.step = float(step)

        half_xy = float(span_xy) * 0.5
        xmin = float(center_xy[0]) - half_xy
        xmax = float(center_xy[0]) + half_xy
        ymin = float(center_xy[1]) - half_xy
        ymax = float(center_xy[1]) + half_xy
        zmin = float(z0)
        zmax = zmin + float(span_z)

        self.set_bounds((xmin, xmax, ymin, ymax, zmin, zmax))

    def spawn_fixed_grid_from_mesh(self, mesh, *, span_xy: float = 240.0, span_z: float = 240.0,
                                   use_contact_plane: bool = True, step: Optional[float] = None):
        """
        Legt ein festes Grid 240x240x240 über dem Substratemount:
          - center_xy = (mesh.cx, mesh.cy)
          - z0 = mesh.zmin (Kontakt mit Mount), falls use_contact_plane=True
        """
        if not hasattr(mesh, "bounds"):
            _LOG.warning("spawn_fixed_grid_from_mesh(): mesh hat keine bounds")
            return
        (xmin, xmax, ymin, ymax, zmin, zmax) = mesh.bounds
        cx = 0.5 * (xmin + xmax)
        cy = 0.5 * (ymin + ymax)
        z0 = zmin if use_contact_plane else (0.5 * (zmin + zmax) - float(span_z) * 0.5)
        self.spawn_fixed_grid_at(center_xy=(cx, cy), z0=z0, span_xy=span_xy, span_z=span_z, step=step)

    # ---- weiterhin hilfreiche Zentrierung (optional) ----
    def center_on_point(self, cx, cy, cz, *, span_xy=240.0, span_z=240.0, step=None):
        """Behalte als generischen Helfer (ohne ia.clear)."""
        self.spawn_fixed_grid_at(center_xy=(cx, cy), z0=cz, span_xy=span_xy, span_z=span_z, step=step)

    def center_on_mesh(self, mesh, *, on_contact_plane=True,
                       margin_xy=20.0, margin_z_top=60.0, margin_z_bottom=20.0,
                       min_span_xy=200.0, min_span_z=200.0, step=None):
        """
        Legacy-Helfer: belässt flexibles Spannungsmodell.
        (Falls du künftig NUR das feste 240er-Grid möchtest, ruf stattdessen spawn_fixed_* auf.)
        """
        if not hasattr(mesh, "bounds"):
            return
        (xmin, xmax, ymin, ymax, zmin, zmax) = mesh.bounds
        width, height, depth = (xmax-xmin), (ymax-ymin), (zmax-zmin)
        span_xy = max(width, height) + 2.0*margin_xy
        span_xy = max(span_xy, float(min_span_xy))
        if on_contact_plane:
            cz = zmin
            span_z = depth + margin_z_top + margin_z_bottom
        else:
            cz = 0.5*(zmin + zmax)
            span_z = depth + margin_z_top + margin_z_bottom
        span_z = max(span_z, float(min_span_z))
        cx = 0.5*(xmin + xmax); cy = 0.5*(ymin + ymax)
        self.center_on_point(cx, cy, cz, span_xy=span_xy, span_z=span_z, step=step)
