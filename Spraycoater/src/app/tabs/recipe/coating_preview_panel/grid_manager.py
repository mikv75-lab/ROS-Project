# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Tuple, Any

_LOG = logging.getLogger("app.tabs.recipe.preview.grid")


class GridManager:
    def __init__(self, interactor_getter: callable):
        self._get_ia = interactor_getter
        self.bounds: Tuple[float, float, float, float, float, float] = (-120, 120, -120, 120, 0, 240)
        self.step: float = 10.0
        self._grid_actor = None
        self.visible: bool = True

    def build_init_scene_mainstyle(self):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("build_init_scene_mainstyle(): kein Interactor angehängt")
            return
        ia.clear()
        axes = ia.show_grid(
            bounds=self.bounds,
            xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
            show_xaxis=True, show_yaxis=True, show_zaxis=True,
            show_xlabels=True, show_ylabels=True, show_zlabels=True,
            n_xlabels=6, n_ylabels=6, n_zlabels=7,
            ticks='both', grid='back', render=False
        )
        try:
            axes.SetShowEdges(False)
            axes.SetDrawXGridlines(True); axes.SetDrawYGridlines(True); axes.SetDrawZGridlines(True)
            axes.SetDrawXInnerGridlines(True); axes.SetDrawYInnerGridlines(True); axes.SetDrawZInnerGridlines(True)
            axes.SetUseTextActor3D(1)
        except Exception:
            pass
        try:
            ia.view_isometric()
            ia.reset_camera(bounds=self.bounds)
        except Exception:
            pass
        ia.render()

    def build_init_scene(self):
        ia = self._get_ia()
        if ia is None:
            _LOG.warning("build_init_scene(): kein Interactor")
            return
        ia.clear()
        self._grid_actor = None
        if self.visible:
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
        try:
            ia.view_isometric()
            ia.reset_camera(bounds=self.bounds)
        except Exception:
            pass
        ia.render()

    def set_bounds(self, bounds):
        self.bounds = bounds
        ia = self._get_ia()
        if ia is None:
            return
        try:
            if self._grid_actor is not None:
                try: ia.remove_actor(self._grid_actor, render=False)
                except Exception: pass
                self._grid_actor = None
            if self.visible:
                self._grid_actor = ia.show_grid(
                    bounds=bounds,
                    xtitle="X (mm)", ytitle="Y (mm)", ztitle="Z (mm)",
                    ticks='both', grid='back', render=False
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
            _LOG.exception("set_bounds() failed")

    def set_grid(self, *, bounds=None, step: Optional[float]=None):
        if bounds is not None:
            self.bounds = bounds
        if step is not None:
            self.step = float(step)
        self.set_bounds(self.bounds)

    def set_visible(self, visible: bool):
        self.visible = bool(visible)
        ia = self._get_ia()
        if ia is None:
            return
        try:
            if not visible and self._grid_actor is not None:
                try: ia.remove_actor(self._grid_actor, render=False)
                except Exception: pass
                self._grid_actor = None
            elif visible and self._grid_actor is None:
                self.set_bounds(self.bounds)
            ia.render()
        except Exception:
            _LOG.exception("set_visible() failed")

    # — Zentrierung —
    def center_on_point(self, cx, cy, cz, *, span_xy=240.0, span_z=240.0, step=None):
        if step is not None:
            self.step = float(step)
        half_xy = float(span_xy) * 0.5
        half_z  = float(span_z)  * 0.5
        b = (cx - half_xy, cx + half_xy,
             cy - half_xy, cy + half_xy,
             cz - half_z,  cz + half_z)
        self.set_bounds(b)

    def center_on_mesh(self, mesh, *, on_contact_plane=True,
                       margin_xy=20.0, margin_z_top=60.0, margin_z_bottom=20.0,
                       min_span_xy=200.0, min_span_z=200.0, step=None):
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
        cx = 0.5*(xmin + xmax)
        cy = 0.5*(ymin + ymax)
        self.center_on_point(cx, cy, cz, span_xy=span_xy, span_z=span_z, step=step)
