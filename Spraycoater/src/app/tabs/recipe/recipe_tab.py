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

# unverändert verwenden:
from .coating_preview_panel.path_builder import PathBuilder
from .coating_preview_panel.raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("app.tabs.recipe")


class RecipeTab(QWidget):
    """
    Layout: [RecipeEditorPanel] | [CoatingPreviewPanel(host)] | [PlanningPanel]
    - KEIN initialer Scene-Build mehr. Rendering passiert nur, wenn _render_preview() aufgerufen wird
      (z. B. über deinen Init-Button oder updatePreviewRequested).
    """

    def __init__(
        self,
        *,
        ctx,
        bridge,
        attach_preview_widget: Callable[[QWidget], None],
        parent: Optional[QWidget] = None
    ):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Left: Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Center: Preview (Host + Buttons)
        self.previewPanel = CoatingPreviewPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.previewPanel, 1)

        # Interactor einhängen (MainWindow hostet den QtInteractor)
        self._attach_preview_widget(self.previewPanel.preview_host())

        # optional: Interactor direkt ans Panel koppeln (falls vorhanden)
        try:
            win = self.window() or self.parent()
            if win is not None and hasattr(win, "previewPlot") and win.previewPlot is not None:
                self.previewPanel.attach_interactor(win.previewPlot)
        except Exception:
            _LOG.exception("RecipeTab: adopt previewPlot into previewPanel failed")

        # Right: Planner
        self.planningPanel = PlanningPanel(ctx=self.ctx, bridge=self.bridge, parent=self)
        hroot.addWidget(self.planningPanel, 0)

        # Editor → Preview render trigger (Queued)
        self.recipePanel.updatePreviewRequested.connect(
            lambda model: self._render_preview(model),
            Qt.ConnectionType.QueuedConnection,
        )

        # Optional: Provider an PlanningPanel
        if hasattr(self.planningPanel, "set_model_provider"):
            self.planningPanel.set_model_provider(self.recipePanel.current_model)
        if hasattr(self.planningPanel, "set_traj_provider"):
            self.planningPanel.set_traj_provider(lambda: None)
        if hasattr(self.planningPanel, "set_bridge"):
            self.planningPanel.set_bridge(self.bridge)

    # -------- Render orchestration: Panel macht clear/add_mesh/render --------
    def _render_preview(self, model: object):
        """
        Reihenfolge: clear -> Mount -> Substrat -> Grid(center on Substrat, contact plane)
                    -> Maske -> Rays -> TCP-Pfad/Marker -> Normale/Frames -> Kamera/Render
        """
        try:
            # Pflichtfelder
            mount_key = self._get_required_str(model, "substrate_mount", "Recipe benötigt 'substrate_mount'.")
            substrate_key = self._get_required_str(model, "substrate", "Recipe benötigt 'substrate'.")

            # Szene komplett leeren (ohne Grid-Rebuild)
            try:
                self.previewPanel.clear()
            except Exception:
                _LOG.exception("previewPanel.clear() failed")

            # 1) Mount @ (0,0,0)
            mmesh = None
            try:
                mmesh = load_mount_mesh_from_key(self.ctx, mount_key)
                self.previewPanel.add_mesh(mmesh, color="lightgray", opacity=0.3, lighting=False)
            except Exception as e:
                _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

            # 2) Substrat platzieren (auf Mount) + triangulieren für robustes ray_trace
            smesh = None
            try:
                smesh = load_substrate_mesh_from_key(self.ctx, substrate_key)
                smesh = place_substrate_on_mount(self.ctx, smesh, mount_key=mount_key)
                try:
                    smesh = smesh.triangulate()
                except Exception:
                    pass
                self.previewPanel.add_mesh(smesh, color="#3498db", opacity=0.95, lighting=False)
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

            # 3) Grid nach dem Substrat (Kontakt-Ebene)
            try:
                if smesh is not None and hasattr(self.previewPanel, "center_grid_on_mesh"):
                    self.previewPanel.center_grid_on_mesh(smesh, on_contact_plane=True)
            except Exception:
                _LOG.exception("center_grid_on_mesh() failed")

            # 4) Spray-Visualisierung (Maske → Rays → TCP → Normale/Frames)
            try:
                if smesh is not None:
                    # side + sample_step
                    if isinstance(model, dict):
                        paths_by_side = dict(model.get("paths_by_side") or {})
                    else:
                        paths_by_side = dict(getattr(model, "paths_by_side", {}) or {})
                    if not paths_by_side:
                        _LOG.warning("Recipe hat keine paths_by_side – breche Spray-Visualisierung ab.")
                        self._finalize_view_and_render()
                        return

                    side = "top" if "top" in paths_by_side else next(iter(paths_by_side.keys()))
                    sample_step = float(paths_by_side.get(side, {}).get("sample_step_mm", 1.0))

                    # Rohpfad (lokal, Z=0)
                    pd = PathBuilder.from_side(model, side=side, sample_step_mm=sample_step)
                    P0 = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
                    if len(P0) == 0:
                        _LOG.warning("PathBuilder lieferte 0 Punkte – zeige nur Mount/Substrat.")
                        self._finalize_view_and_render()
                        return

                    # Pfad in Welt verschieben: auf Kontakt-Ebene des Substrats
                    xmin, xmax, ymin, ymax, zmin, zmax = smesh.bounds
                    cx = 0.5 * (xmin + xmax)
                    cy = 0.5 * (ymin + ymax)
                    z_plane = zmin  # Kontakt-Ebene für „top“
                    P0_world = P0 + np.array([cx, cy, z_plane], dtype=float)

                    # Blaue Maskenlinie (über Substrat, 100 mm Clearance je Side)
                    try:
                        # einfache Top-Maske: +100 mm in Z (für andere Sides analog anpassen)
                        mask_height = (zmax + 100.0)
                        P_mask = P0_world.copy()
                        P_mask[:, 2] = mask_height
                        line_mask = pv.lines_from_points(P_mask, close=False)
                        self.previewPanel.clear_layer("mask")
                        self.previewPanel.add_mesh(line_mask, color="royalblue", opacity=1.0, lighting=False)
                    except Exception:
                        _LOG.exception("Maskenlinie fehlgeschlagen")

                    # Stand-off
                    stand_off = float(
                        getattr(model, "stand_off_mm", None)
                        or (isinstance(model, dict) and model.get("stand_off_mm"))
                        or 10.0
                    )

                    # Rays/Reflexion/TCP
                    rc, rays_poly, tcp_poly = cast_rays_for_side(
                        P_world_start=P0_world,
                        sub_mesh_world=smesh,
                        side=side,
                        source=pd.meta.get("source", "points"),
                        stand_off_mm=stand_off,
                        ray_len_mm=1000.0,
                        start_lift_mm=10.0,
                        flip_normals_to_face_rays=True,
                        invert_dirs=False,
                        lock_xy=True,
                    )

                    valid = rc.valid if rc is not None else None
                    if valid is not None and np.any(valid):
                        hits = rc.hit_mm[valid]
                        norms = rc.normal[valid]
                        tcp  = rc.tcp_mm[valid]

                        # Rays + TCP-Verbindungen
                        self.previewPanel.show_poly(rays_poly, layer="rays", color="#f39c12", line_width=1.0)
                        self.previewPanel.show_poly(tcp_poly,  layer="tcp",  color="#5dade2", line_width=1.0)

                        # Finaler Spraypfad = TCP-Polyline + Marker
                        self.previewPanel.clear_layer("path")
                        self.previewPanel.clear_layer("path_markers")
                        self.previewPanel.add_path_polyline(tcp, color="#e74c3c", line_width=2.0, as_tube=False, layer="path")
                        self.previewPanel.add_path_markers(tcp, step=max(1, len(tcp)//50), color="#c0392b", layer="path_markers")

                        # Normale und lokale Frames (optional)
                        try:
                            self.previewPanel.show_normals_from_hits(hits, norms, layer="normals", length_mm=8.0, color="#27ae60", line_width=1.0)
                        except Exception:
                            _LOG.exception("show_normals_from_hits() failed")

                        try:
                            self.previewPanel.show_frames_at(origins=tcp, z_dirs=rc.refl_dir[valid],
                                                             scale_mm=10.0, layer="frames", line_width=2.0)
                        except Exception:
                            _LOG.exception("show_frames_at() failed")
                    else:
                        _LOG.warning("Keine gültigen Ray-Treffer – zeige nur Maskenlinie (und ggf. Rohpfad).")

            except Exception as e:
                _LOG.error("Spray-Preview failed: %s", e, exc_info=True)

            # 5) Kamera + Render
            self._finalize_view_and_render()

        except Exception:
            _LOG.exception("Render orchestration failed")

    def _finalize_view_and_render(self):
        try:
            self.previewPanel.view_isometric()
        except Exception:
            pass
        self.previewPanel.render(reset_camera=True)

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
