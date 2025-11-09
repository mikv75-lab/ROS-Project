# -*- coding: utf-8 -*-
# File: app/tabs/recipe/recipe_tab.py
from __future__ import annotations
import logging
from typing import Optional, Callable, Any, Dict

import numpy as np
import pyvista as pv
from PyQt6.QtWidgets import QWidget, QHBoxLayout
from PyQt6.QtCore import Qt

from .recipe_editor_panel.recipe_editor_panel import RecipeEditorPanel
from .coating_preview_panel.coating_preview_panel import CoatingPreviewPanel

from .coating_preview_panel.mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)

_LOG = logging.getLogger("app.tabs.recipe")


class RecipeTab(QWidget):
    def __init__(self, *, ctx, bridge, attach_preview_widget: Callable[[QWidget], None],
                 parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self.bridge = bridge
        self._attach_preview_widget = attach_preview_widget

        # Interner Traj-/Meta-Puffer (ohne PlanningPanel)
        self._last_traj: Optional[Dict[str, Any]] = None
        self._last_yaml: str = ""

        # Layout
        hroot = QHBoxLayout(self)
        hroot.setContentsMargins(6, 6, 6, 6)
        hroot.setSpacing(8)

        # Links: Recipe-Editor
        self.recipePanel = RecipeEditorPanel(ctx=self.ctx, parent=self)
        hroot.addWidget(self.recipePanel, 0)

        # Mitte/Rechts: Preview
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

        # --- Wiring: Recipe → Preview (Render anstoßen)
        self.recipePanel.updatePreviewRequested.connect(
            lambda payload: self._render_preview(payload.get("model"), payload.get("sides")),
            Qt.ConnectionType.QueuedConnection
        )

        # --- Preview → interne Puffer (ohne PlanningPanel) -------------------
        try:
            # YAML-Text (z.B. für Debug/Export)
            self.previewPanel.previewYamlReady.connect(self._on_preview_yaml_ready)
            # Kamera sanft updaten
            self.previewPanel.previewYamlReady.connect(lambda *_: self._on_preview_event())
        except Exception:
            _LOG.exception("connect previewYamlReady failed")

        try:
            # Pfadpunkte (mm) als einfacher Snapshot
            self.previewPanel.pathReady.connect(self._on_path_ready)
            self.previewPanel.pathReady.connect(lambda *_: self._on_preview_event())
        except Exception:
            _LOG.exception("connect pathReady failed")

    # --- Slots für Preview-Signale ---
    def _on_preview_yaml_ready(self, text: str) -> None:
        """Puffert den YAML-Text lokal (PlanningPanel existiert nicht mehr)."""
        try:
            self._last_yaml = text or ""
        except Exception:
            _LOG.exception("_on_preview_yaml_ready failed")

    def _on_path_ready(self, path_xyz: Optional[np.ndarray]) -> None:
        """
        Aktualisiert den Trajektorie-Puffer. Erwartet (N,3) mm-Punkte oder None.
        """
        try:
            if path_xyz is None:
                self._last_traj = {"points_mm": [], "count": 0}
            else:
                P = np.asarray(path_xyz, dtype=float).reshape(-1, 3)
                self._last_traj = {
                    "points_mm": P.tolist(),
                    "count": int(P.shape[0]),
                }
        except Exception:
            _LOG.exception("_on_path_ready failed")

    def _on_preview_event(self) -> None:
        """
        Gemeinsames Kamera-Update für dieselben Events (YAML/Pfad ready).
        Absichtlich ohne harten Reset für smoothes Verhalten.
        """
        try:
            self.previewPanel.view_isometric()
        except Exception:
            pass
        try:
            self.previewPanel.render(reset_camera=False)
        except Exception:
            _LOG.exception("preview camera update failed")

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

    def _visibility_snapshot(self) -> dict:
        """Liest die Checkboxen aus dem Panel und gibt eine Sichtbarkeits-Map zurück."""
        p = self.previewPanel
        def b(x):
            try:
                return bool(x.isChecked())
            except Exception:
                return False
        return {
            "mask":    b(getattr(p, "chkShowMask", None)),
            "path":    b(getattr(p, "chkShowPath", None)),
            "hits":    b(getattr(p, "chkShowHits", None)),
            "misses":  b(getattr(p, "chkShowMisses", None)),
            "normals": b(getattr(p, "chkShowNormals", None)),
            "frames":  b(getattr(p, "chkShowLocalFrames", None)),
        }

    # --- Render orchestration ---
    def _render_preview(self, model_or_payload: object, sides: Optional[list] = None):
        try:
            # robust entpacken (falls doch jemand direkt das Model durchreicht)
            model = model_or_payload
            if isinstance(model_or_payload, dict):
                model = model_or_payload.get("model", model_or_payload)

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

            # 3) Bounds setzen
            if smesh is not None:
                self.previewPanel.set_bounds_from_mesh(smesh, use_contact_plane=True, span_xy=240.0, span_z=240.0)
            elif mmesh is not None:
                self.previewPanel.set_bounds_from_mesh(mmesh, use_contact_plane=False, span_xy=240.0, span_z=240.0)

            # --- Center/Levels bestimmen ---
            cx = cy = 0.0
            z_ground = 0.0
            if mmesh is not None:
                xmin, xmax, ymin, ymax, zmin_m, _ = mmesh.bounds
                cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
                z_ground = float(zmin_m)
            elif smesh is not None:
                xmin, xmax, ymin, ymax, zmin_s, _ = smesh.bounds
                cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
                z_ground = float(zmin_s)

            # 4) Ground zuerst (gefüllte Fläche 500x500 an z_ground)
            try:
                ground = pv.Plane(
                    center=(cx, cy, z_ground),
                    direction=(0, 0, 1),
                    i_size=500.0, j_size=500.0,
                    i_resolution=1, j_resolution=1
                )
                self.previewPanel.add_mesh(
                    ground,
                    color=self.previewPanel.DEFAULT_GROUND_COLOR,
                    opacity=1.0,
                    lighting=False,
                    layer="ground",
                    render=False,
                    reset_camera=False,
                )
            except Exception:
                _LOG.exception("Ground-Plane Erzeugung fehlgeschlagen")

            # 5) Mount
            if mmesh is not None:
                try:
                    self.previewPanel.add_mesh(mmesh, color=self.previewPanel.DEFAULT_MOUNT_COLOR,
                                               opacity=0.95, lighting=False, layer="mount",
                                               render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("Mount zeichnen fehlgeschlagen")

            # 6) Substrat
            if smesh is not None:
                try:
                    self.previewPanel.add_mesh(smesh, color=self.previewPanel.DEFAULT_SUBSTRATE_COLOR,
                                               opacity=1.0, lighting=False, layer="substrate",
                                               render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("Substrat zeichnen fehlgeschlagen")

            # 7) Spray-Visualisierung – alles berechnen, nur selektiv anzeigen
            try:
                if smesh is None:
                    raise RuntimeError("Kein Substratmesh für Spray-Preview")

                vis = self._visibility_snapshot()  # Checkboxen lesen
                self.previewPanel.overlays.render_from_model(
                    model=model,
                    substrate_mesh=smesh,
                    side=None,                    # auto: 'top' oder erster key
                    default_stand_off_mm=10.0,
                    mask_lift_mm=50.0,
                    ray_len_mm=1000.0,
                    visibility=vis,
                )
                if hasattr(self.previewPanel.overlays, "apply_visibility"):
                    self.previewPanel.overlays.apply_visibility(vis)
            except Exception:
                _LOG.exception("Overlays.render_from_model failed")

            # 8) View finalisieren + genau EIN Render
            self._finalize_view_and_render()

        except Exception:
            _LOG.exception("Render orchestration failed")

    def _finalize_view_and_render(self):
        try:
            self.previewPanel.view_isometric()
        except Exception:
            pass
        self.previewPanel.render(reset_camera=True)
