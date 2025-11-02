# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable, Any

import numpy as np
import pyvista as pv
from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .planning_panel.planning_panel import PlanningPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from .coating_preview_panel.mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)
from .coating_preview_panel.path_builder import PathBuilder
from .coating_preview_panel.raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("app.tabs.recipe")


class RecipeTab(QWidget):
    def __init__(self, *, ctx, bridge, attach_preview_widget: Callable[[QWidget], None], parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        hroot = QHBoxLayout(self); hroot.setContentsMargins(6, 6, 6, 6); hroot.setSpacing(8)
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self); hroot.addWidget(self.recipePanel, 0)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self); hroot.addWidget(self.previewPanel, 1)
        self._attach_preview_widget(self.previewPanel.preview_host())
        try:
            win = self.window() or self.parent()
            if win is not None and hasattr(win, "previewPlot") and win.previewPlot is not None:
                self.previewPanel.attach_interactor(win.previewPlot)
        except Exception:
            _LOG.exception("RecipeTab: adopt previewPlot into previewPanel failed")
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self); hroot.addWidget(self.planningPanel, 0)

        self.recipePanel.updatePreviewRequested.connect(lambda model: self._render_preview(model), Qt.ConnectionType.QueuedConnection)
        if hasattr(self.planningPanel, "set_model_provider"): self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):  self.planningPanel.set_traj_provider(lambda: None)
        if hasattr(self.planningPanel, "set_bridge"):         self.planningPanel.set_bridge(self.bridge)

    # -------- Render orchestration --------
    def _render_preview(self, model: object):
        try:
            mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt 'substrate_mount'.")
            substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt 'substrate'.")

            try: self.previewPanel.clear()
            except Exception: _LOG.exception("previewPanel.clear() failed")

            # 1) Mount
            try:
                mmesh = load_mount_mesh_from_key(self.ctx, mount_key)
                self.previewPanel.add_mesh(mmesh, color="lightgray", opacity=0.3, lighting=False)
            except Exception as e:
                _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

            # 2) Substrat (trianguliert)
            smesh = None
            try:
                smesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
                smesh = place_substrate_on_mount(self.ctx, smesh, mount_key=mount_key)
                try: smesh = smesh.triangulate()
                except Exception: pass
                self.previewPanel.add_mesh(smesh, color="#3498db", opacity=0.95, lighting=False)
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

            # 3) Bounds am Substrat
            try:
                if smesh is not None:
                    self.previewPanel.set_bounds_from_mesh(smesh, use_contact_plane=True, span_xy=240.0, span_z=240.0)
            except Exception:
                _LOG.exception("Bounds-Setzen (am Substrat) failed")

            # 4) Spray-Visualisierung
            try:
                if smesh is not None:
                    if isinstance(model, dict):
                        paths_by_side = dict(model.get("paths_by_side") or {})
                    else:
                        paths_by_side = dict(getattr(model, "paths_by_side", {}) or {})
                    if not paths_by_side:
                        _LOG.warning("Recipe hat keine paths_by_side – breche Spray-Visualisierung ab.")
                        self._finalize_view_and_render(); return

                    side = "top" if "top" in paths_by_side else next(iter(paths_by_side.keys()))
                    sample_step = float(paths_by_side.get(side, {}).get("sample_step_mm", 1.0))

                    pd = PathBuilder.from_side(model, side=side, sample_step_mm=sample_step)
                    P0 = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
                    if len(P0) == 0:
                        _LOG.warning("PathBuilder lieferte 0 Punkte – zeige nur Mount/Substrat.")
                        self._finalize_view_and_render(); return

                    xmin, xmax, ymin, ymax, zmin, zmax = smesh.bounds
                    cx, cy = 0.5*(xmin+xmax), 0.5*(ymin+ymax)
                    z_plane = zmin
                    P0_world = P0 + np.array([cx, cy, z_plane], dtype=float)

                    # Maske 50 mm über zmax
                    try:
                        z_mask = float(zmax + 50.0)
                        P_mask = P0_world.copy(); P_mask[:, 2] = z_mask
                        line_mask = pv.lines_from_points(P_mask, close=False)
                        self.previewPanel.clear_layer("mask")
                        self.previewPanel.add_mesh(line_mask, color="royalblue", opacity=1.0, lighting=False, layer="mask")
                    except Exception:
                        _LOG.exception("Maskenlinie fehlgeschlagen")

                    dir_map = {"top": np.array([0.0,0.0,-1.0]), "bottom": np.array([0.0,0.0,1.0]),
                               "front": np.array([0.0,-1.0,0.0]), "back": np.array([0.0,1.0,0.0]),
                               "left": np.array([1.0,0.0,0.0]), "right": np.array([-1.0,0.0,0.0])}
                    ray_dir = dir_map.get(side, np.array([0.0,0.0,-1.0]))
                    ray_len = 1000.0
                    stand_off = float(getattr(model, "stand_off_mm", None) or (isinstance(model, dict) and model.get("stand_off_mm")) or 10.0)

                    rc, _rays_poly_unused, tcp_poly = cast_rays_for_side(
                        P_world_start=P_mask, sub_mesh_world=smesh, side=side,
                        source=pd.meta.get("source", "points"), stand_off_mm=stand_off,
                        ray_len_mm=ray_len, start_lift_mm=0.0,
                        flip_normals_to_face_rays=True, invert_dirs=False, lock_xy=True,
                    )

                    valid = rc.valid if rc is not None else None

                    def _lines_from_pairs(A: np.ndarray, B: np.ndarray) -> pv.PolyData:
                        A = np.asarray(A, dtype=float).reshape(-1, 3)
                        B = np.asarray(B, dtype=float).reshape(-1, 3)
                        m = min(len(A), len(B))
                        if m == 0: return pv.PolyData()
                        pts = np.vstack([A[:m], B[:m]])
                        lines = np.empty((m, 3), dtype=np.int64)
                        lines[:, 0] = 2; lines[:, 1] = np.arange(m, dtype=np.int64); lines[:, 2] = np.arange(m, dtype=np.int64) + m
                        poly = pv.PolyData(pts); poly.lines = lines.reshape(-1); return poly

                    Pstart = P_mask

                    # Hits
                    rays_hit = pv.PolyData()
                    if valid is not None and np.any(valid):
                        rays_hit = _lines_from_pairs(Pstart[valid], rc.hit_mm[valid])

                    # Misses — beim vertikalen Fall an zmin kappen
                    rays_miss = pv.PolyData()
                    if valid is not None:
                        miss = ~valid
                        Pmiss_start = Pstart[miss]
                        if len(Pmiss_start):
                            if abs(ray_dir[2]) > 1e-6:
                                z_floor = float(zmin)
                                t = (z_floor - Pmiss_start[:, 2]) / float(ray_dir[2])
                                t = np.maximum(0.0, t)
                                Pmiss_end = Pmiss_start + np.outer(t, ray_dir)
                            else:
                                Pmiss_end = Pmiss_start + ray_dir * ray_len
                            rays_miss = _lines_from_pairs(Pmiss_start, Pmiss_end)

                    # zeichnen in getrennten Layern
                    self.previewPanel.clear_layer("rays_hit")
                    self.previewPanel.clear_layer("rays_miss")
                    if rays_hit.n_lines:
                        self.previewPanel.add_mesh(rays_hit, color="#85C1E9", line_width=1.5, lighting=False, layer="rays_hit")
                    if rays_miss.n_lines:
                        self.previewPanel.add_mesh(rays_miss, color="#e74c3c", line_width=1.0, lighting=False, layer="rays_miss")

                    # TCP / Path / Normals / Frames
                    if rc is not None and hasattr(rc, "tcp_mm") and rc.tcp_mm is not None:
                        if tcp_poly is not None:
                            self.previewPanel.show_poly(tcp_poly, layer="tcp", color="#5dade2", line_width=1.0)
                        tcp = rc.tcp_mm[valid] if (valid is not None and np.any(valid)) else rc.tcp_mm
                        if tcp is not None and len(tcp):
                            self.previewPanel.clear_layer("path")
                            self.previewPanel.clear_layer("path_markers")
                            self.previewPanel.add_path_polyline(tcp, color="#e74c3c", line_width=2.0, as_tube=False, layer="path")
                            self.previewPanel.add_path_markers(tcp, step=max(1, len(tcp)//50), color="#c0392b", layer="path_markers")
                            if valid is not None and np.any(valid):
                                hits = rc.hit_mm[valid]; norms = rc.normal[valid]
                                try:
                                    self.previewPanel.show_normals_from_hits(hits, norms, layer="normals", length_mm=8.0, color="#27ae60", line_width=1.0)
                                except Exception:
                                    _LOG.exception("show_normals_from_hits() failed")
                            try:
                                z_dirs = rc.refl_dir[valid] if (valid is not None and np.any(valid)) else rc.refl_dir
                                if z_dirs is not None and len(z_dirs):
                                    self.previewPanel.show_frames_at(origins=tcp, z_dirs=z_dirs, scale_mm=1.0, line_width=None, layer="frames")
                            except Exception:
                                _LOG.exception("show_frames_at() failed")

            except Exception as e:
                _LOG.error("Spray-Preview failed: %s", e, exc_info=True)

            self._finalize_view_and_render()

        except Exception:
            _LOG.exception("Render orchestration failed")

    def _finalize_view_and_render(self):
        try: self.previewPanel.view_isometric()
        except Exception: pass
        self.previewPanel.render(reset_camera=True)

    @staticmethod
    def _get_required_str(model: object, key: str, err: str) -> str:
        if hasattr(model, key): val = getattr(model, key)
        elif isinstance(model, dict): val = model.get(key)
        else: val = None
        if not isinstance(val, str) or not val.strip(): raise ValueError(err)
        return val.strip()
