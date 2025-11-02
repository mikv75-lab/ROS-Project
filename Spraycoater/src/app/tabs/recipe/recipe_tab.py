# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Optional, Callable

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
    def __init__(self, *, ctx, bridge, attach_preview_widget: Callable[[QWidget], None],
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        # Layout
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Interactor aus dem MainWindow übernehmen
        self._attach_preview_widget(self.previewPanel.preview_host())
        try:
            win = self.window() or self.parent()
            if win is not None and hasattr(win, "previewPlot") and win.previewPlot is not None:
                self.previewPanel.attach_interactor(win.previewPlot)
        except Exception:
            _LOG.exception("RecipeTab: adopt previewPlot into previewPanel failed")

        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)
        hroot.addWidget(self.planningPanel, 0)

        # Wiring
        self.recipePanel.updatePreviewRequested.connect(
            lambda model: self._render_preview(model),
            Qt.ConnectionType.QueuedConnection
        )
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            self.planningPanel.set_traj_provider(lambda: None)
        if hasattr(self.planningPanel, "set_bridge"):
            self.planningPanel.set_bridge(self.bridge)

    # --- helpers ---
    @staticmethod
    def _get_required_str(model: object, key: str, err: str) -> str:
        if hasattr(model, key):
            val = getattr(model, key)
        elif isinstance(model, dict):
            val = model.get(key)
        else:
            val = None
        if not isinstance(val, str) or not val.strip():
            raise ValueError(err)
        return val.strip()

    # --- Render orchestration ---
    def _render_preview(self, model: object):
        try:
            mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt 'substrate_mount'.")
            substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt 'substrate'.")

            # Szene leeren
            self.previewPanel.clear()

            # 1) Mount-Mesh laden
            mmesh: pv.PolyData | None = None
            try:
                mmesh = load_mount_mesh_from_key(self.ctx, mount_key)
            except Exception as e:
                _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

            # 2) Substrat (auf Mount positioniert)
            smesh: pv.PolyData | None = None
            try:
                smesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
                smesh = place_substrate_on_mount(self.ctx, smesh, mount_key=mount_key)
                try:
                    if hasattr(smesh, "is_all_triangles") and not smesh.is_all_triangles():
                        smesh = smesh.triangulate()
                except Exception:
                    pass
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

            # 3) Bounds & Ebenen
            # z_plane: Mount-Oberfläche (Kontakt zur Substrat-Unterseite)
            z_plane = 0.0
            cx = cy = 0.0
            if smesh is not None:
                xmin, xmax, ymin, ymax, zmin, zmax = smesh.bounds
                cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
                z_plane = float(zmin)
                self.previewPanel.set_bounds_from_mesh(smesh, use_contact_plane=True, span_xy=240.0, span_z=240.0)
            elif mmesh is not None:
                xmin, xmax, ymin, ymax, zmin, zmax = mmesh.bounds
                cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
                z_plane = float(zmax)  # Top der Mount
                self.previewPanel.set_bounds_from_mesh(mmesh, use_contact_plane=False, span_xy=240.0, span_z=240.0)

            # 4) Ground-Plane (vor Mount, dunkler)
            try:
                xmin, xmax, ymin, ymax, _, _ = self.previewPanel._bounds
                width  = xmax - xmin
                height = ymax - ymin
                z_ground = z_plane - 0.5  # kleiner Offset gegen Z-Fighting
                ground = pv.Plane(center=(cx, cy, z_ground), direction=(0, 0, 1),
                                  i_size=width, j_size=height,
                                  i_resolution=1, j_resolution=1)
                self.previewPanel.add_mesh(
                    ground, color=self.previewPanel.DEFAULT_GROUND_COLOR, opacity=1.0,
                    lighting=False, layer="ground"
                )
            except Exception:
                _LOG.exception("Ground-Plane Erzeugung fehlgeschlagen")

            # 5) Mount
            if mmesh is not None:
                try:
                    self.previewPanel.add_mesh(mmesh, color=self.previewPanel.DEFAULT_MOUNT_COLOR,
                                               opacity=0.95, lighting=False, layer="mount")
                except Exception:
                    _LOG.exception("Mount zeichnen fehlgeschlagen")

            # 6) Substrat
            if smesh is not None:
                try:
                    self.previewPanel.add_mesh(smesh, color=self.previewPanel.DEFAULT_SUBSTRATE_COLOR,
                                               opacity=1.0, lighting=False, layer="substrate")
                except Exception:
                    _LOG.exception("Substrat zeichnen fehlgeschlagen")

            # 7) Spray-Visualisierung
            mask_poly = None
            tcp_for_2d = None

            try:
                if smesh is None:
                    raise RuntimeError("Kein Substratmesh für Spray-Preview")

                # Pfad aus recipe
                pbs = dict(model.get("paths_by_side") or {}) if isinstance(model, dict) \
                      else dict(getattr(model, "paths_by_side", {}) or {})
                if not pbs:
                    _LOG.warning("Recipe hat keine paths_by_side – zeige nur Ground/Mount/Substrat.")
                    self._finalize_view_and_render()
                    self.previewPanel.update_2d_scene(substrate_mesh=smesh, path_xyz=None, mask_poly=None)
                    return

                side = "top" if "top" in pbs else next(iter(pbs.keys()))
                sample_step = float(pbs.get(side, {}).get("sample_step_mm", 1.0))

                pd = PathBuilder.from_side(model, side=side, sample_step_mm=sample_step)
                P0 = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
                if len(P0) == 0:
                    _LOG.warning("PathBuilder lieferte 0 Punkte – zeige nur Ground/Mount/Substrat.")
                    self._finalize_view_and_render()
                    self.previewPanel.update_2d_scene(substrate_mesh=smesh, path_xyz=None, mask_poly=None)
                    return

                # In Welt: an Substrat-Bounds andocken
                xmin, xmax, ymin, ymax, szmin, szmax = smesh.bounds
                cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
                z_plane = float(szmin)
                P0_world = P0 + np.array([cx, cy, z_plane], dtype=float)

                # Maske (blau) + Marker
                z_mask = float(szmax + 50.0)
                P_mask = P0_world.copy(); P_mask[:, 2] = z_mask
                mask_poly = pv.lines_from_points(P_mask, close=False)
                self.previewPanel.show_poly(mask_poly, layer="mask", color="royalblue", line_width=2.0)
                self.previewPanel.show_start_end_markers(P_mask, layer_prefix="mask", color="royalblue", size=3.0)

                # Rays + TCP
                stand_off = float(
                    getattr(model, "stand_off_mm", None)
                    or (isinstance(model, dict) and model.get("stand_off_mm"))
                    or 10.0
                )
                rc, rays_hit_poly, tcp_poly = cast_rays_for_side(
                    P_world_start=P_mask,
                    sub_mesh_world=smesh,
                    side=side,
                    source=pd.meta.get("source", "points"),
                    stand_off_mm=stand_off,
                    ray_len_mm=1000.0,
                    start_lift_mm=0.0,
                    flip_normals_to_face_rays=True,
                    invert_dirs=False,
                    lock_xy=True,
                )

                # Hits (hellblau)
                self.previewPanel.clear_layer("rays_hit")
                if getattr(rays_hit_poly, "n_lines", 0):
                    self.previewPanel.add_mesh(rays_hit_poly, color="#85C1E9",
                                               line_width=1.5, lighting=False, layer="rays_hit")

                # „Normals“ = gelbe TCP-Poly (ehemals reflected)
                self.previewPanel.clear_layer("normals")
                if getattr(tcp_poly, "n_lines", 0):
                    self.previewPanel.add_mesh(tcp_poly, color="#f1c40f",
                                               line_width=1.3, lighting=False, layer="normals")

                # Misses (rot) — bis zur Mount-Oberfläche schneiden (z = z_plane)
                self.previewPanel.clear_layer("rays_miss")
                valid = getattr(rc, "valid", None)
                if valid is not None and len(valid) == len(P_mask) and np.any(~valid):
                    base_dir = {
                        "top":   np.array([0.0,  0.0, -1.0], dtype=float),
                        "front": np.array([0.0,  1.0,  0.0], dtype=float),
                        "back":  np.array([0.0, -1.0,  0.0], dtype=float),
                        "left":  np.array([1.0,  0.0,  0.0], dtype=float),
                        "right": np.array([-1.0, 0.0,  0.0], dtype=float),
                    }.get(side, np.array([0.0, 0.0, -1.0], dtype=float))

                    Pmiss_start = P_mask[~valid]
                    D = np.tile(base_dir / (np.linalg.norm(base_dir) + 1e-12), (len(Pmiss_start), 1))

                    have_z = np.abs(D[:, 2]) > 1e-9
                    if np.any(have_z):
                        S = Pmiss_start[have_z]
                        Dz = D[have_z]
                        t = (z_plane - S[:, 2]) / Dz[:, 2]
                        pos = t > 0
                        S = S[pos]; Dz = Dz[pos]; t = t[pos]
                        if len(S):
                            Pend = S + Dz * t.reshape(-1, 1)

                            pts = np.vstack([S, Pend])
                            nseg = len(S)
                            lines = np.empty((nseg, 3), dtype=np.int64)
                            lines[:, 0] = 2
                            lines[:, 1] = np.arange(nseg, dtype=np.int64)
                            lines[:, 2] = lines[:, 1] + nseg

                            rays_miss = pv.PolyData(pts)
                            rays_miss.lines = lines.reshape(-1)
                            self.previewPanel.add_mesh(rays_miss, color="#e74c3c",
                                                       line_width=1.2, lighting=False, layer="rays_miss")

                # TCP-Polyline (grün) + Marker, Frames
                tcp_for_2d = None
                if rc is not None and getattr(rc, "tcp_mm", None) is not None:
                    use_valid = (valid is not None and np.any(valid))
                    tcp = rc.tcp_mm[valid] if use_valid else rc.tcp_mm
                    tcp_for_2d = tcp
                    if tcp is not None and len(tcp):
                        self.previewPanel.clear_layer("path")
                        self.previewPanel.add_path_polyline(
                            tcp, layer="path", color="#2ecc71", line_width=2.0, lighting=False
                        )
                        self.previewPanel.show_start_end_markers(tcp, layer_prefix="path",
                                                                 color="#2ecc71", size=3.0)

                        z_dirs = rc.refl_dir[valid] if (valid is not None and np.any(valid)) else rc.refl_dir
                        if z_dirs is not None and len(z_dirs):
                            self.previewPanel.show_frames_at(origins=tcp, z_dirs=z_dirs, scale_mm=1.0, layer="frames")

            except Exception as e:
                _LOG.error("Spray-Preview failed: %s", e, exc_info=True)

            # 8) 2D-Szene (Substrat + TCP + Maske)
            try:
                self.previewPanel.update_2d_scene(substrate_mesh=smesh, path_xyz=tcp_for_2d, mask_poly=mask_poly)
            except Exception:
                _LOG.exception("update_2d_scene failed")

            self._finalize_view_and_render()

        except Exception:
            _LOG.exception("Render orchestration failed")

    def _finalize_view_and_render(self):
        try:
            self.previewPanel.view_isometric()
        except Exception:
            pass
        self.previewPanel.render(reset_camera=True)
