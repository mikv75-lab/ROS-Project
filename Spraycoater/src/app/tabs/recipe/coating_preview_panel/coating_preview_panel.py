# -*- coding: utf-8 -*-
# Spraycoater/src/app/tabs/recipe/coating_preview_panel/coating_preview_panel.py
from __future__ import annotations
import logging
from typing import Optional, Any, Iterable, Tuple

import numpy as np
import pyvista as pv
from PyQt5.QtWidgets import QWidget
from pyvistaqt import BackgroundPlotter

from .preview import PreviewEngine
from .mesh_utils import (
    get_mount_scene_offset_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
    load_mount_mesh_from_key,
)
from .path_builder import PathBuilder, PathData
from .trajectory_builder import TrajectoryBuilder
from .raycast_projector import cast_rays_for_side  # robust, mit Miss-Handling

_LOG = logging.getLogger("app.tabs.recipe.coating_preview_panel")


class CoatingPreviewPanel(QWidget):
    """Externer 3D-Preview (PyVista BackgroundPlotter) für Mount + Substrat + Pfad."""

    def __init__(self, *, ctx, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx
        self._pv: pv.Plotter = BackgroundPlotter(show=True, title="Spraycoater Preview")
        self._engine: PreviewEngine = PreviewEngine(self._pv)

    # ---------- Public ----------
    def render_from_model(self, model: Any, sides: Iterable[str]) -> None:
        # --- Input prüfen ---
        mount_key = self._get_attr(model, "substrate_mount")
        if not isinstance(mount_key, str) or not mount_key.strip():
            raise ValueError("Recipe benötigt ein gültiges 'substrate_mount'.")
        mount_key = mount_key.strip()

        substrate_key = self._get_attr(model, "substrate")
        if not isinstance(substrate_key, str) or not substrate_key.strip():
            raise ValueError("Recipe benötigt ein gültiges 'substrate'.")
        substrate_key = substrate_key.strip()

        _LOG.info("Preview: mount=%s, substrate=%s", mount_key, substrate_key)

        # --- Szene vorbereiten ---
        self._engine.clear_scene()
        self._engine.view_iso()  # Startpose stabil

        # Mount (solid, dunkelgrau)
        mount_mesh = load_mount_mesh_from_key(self.ctx, mount_key)
        self._engine.add_mesh(
            mount_mesh,
            color=(0.25, 0.25, 0.25),  # dunkelgrau
            opacity=1.0,
            smooth_shading=True,
            lighting=False,
            show_edges=False,
        )

        # Substrat laden (lokal) + platziertes Substrat (Welt) für Anzeige
        sub_mesh_local = load_substrate_mesh_from_key(self.ctx, substrate_key)

        # Ground shift (lokal z_min -> 0)
        zmin = float(sub_mesh_local.bounds[4])
        ground_shift = np.array([0.0, 0.0, -zmin], dtype=float)

        # Mount scene_offset (xyz + rpy)
        xyz_mm, rpy_deg = get_mount_scene_offset_from_key(self.ctx, mount_key)
        R_sub = _rpy_deg_to_matrix_np(rpy_deg)  # 3x3
        t_sub = np.asarray(xyz_mm, dtype=float) # 3,

        # Substrat positionieren und rendern (hellgrau)
        sub_scene = place_substrate_on_mount(self.ctx, sub_mesh_local, mount_key=mount_key)
        self._engine.add_mesh(
            sub_scene,
            color=(0.78, 0.78, 0.78),  # hellgrau/silber
            opacity=1.0,
            smooth_shading=True,
            lighting=False,
            show_edges=False,
        )

        # Zusätzlich: lokales Substrat -> Welt transformieren (für Rays/Bounds in Welt)
        T = np.eye(4, dtype=float)
        T[:3, :3] = R_sub
        T[:3, 3] = t_sub + ground_shift
        sub_mesh_world = sub_mesh_local.copy(deep=True)
        sub_mesh_world.transform(T, inplace=True)
        sub_mesh_world.compute_normals(cell_normals=True, point_normals=False, inplace=True)

        try:
            # ---------------------------
            # 3 STEPS: PATH → (WELT)RAYCAST (seitenabhängig) → TRAJECTORY/VIS
            # ---------------------------
            side = next(iter(sides), "top") if sides else "top"

            # 1) PATH (lokal in Substrat-Koords, Z=0)
            params = getattr(model, "parameters", {}) if hasattr(model, "parameters") else {}
            sample_step = float(params.get("sample_step_mm", 1.0))
            max_points  = int(params.get("max_points", 200000))

            pd_local: PathData = PathBuilder.from_side(
                model,
                side=side,
                sample_step_mm=sample_step,
                max_points=max_points,
            )

            if pd_local.points_mm is None or len(pd_local.points_mm) < 2:
                _LOG.warning("Kein Pfad generiert.")
                self._focus_camera_isometric(sub_scene)
                return

            # --- 1b) „Blaue Maske“: dynamische Start-Ebene anhand Bounds + Offsets ---
            path_offset_mm   = float(params.get("path_offset_mm", 0.0))     # Rezeptoffset entlang Anfahrachse
            ray_clearance_mm = float(params.get("ray_clearance_mm", 100.0)) # 10 cm Freiraum

            P_local0 = np.array(pd_local.points_mm, dtype=float, copy=True)
            P_local0[:, 2] = 0.0
            P_world_base = ((P_local0 + ground_shift) @ R_sub.T) + t_sub  # (N,3)

            plane_axis, plane_value = _side_plane_from_bounds(
                sub_mesh_world.bounds, side, path_offset_mm, ray_clearance_mm
            )
            P_world_start = P_world_base.copy()
            P_world_start[:, plane_axis] = plane_value

            # Blaue Maske
            line_mask = pv.lines_from_points(P_world_start, close=False)
            self._engine.add_mesh(
                line_mask,
                color="royalblue",
                opacity=1.0,
                smooth_shading=False,
                lighting=False,
                line_width=2.0,
            )

            # 2) RAYCAST (liefert Hits, Normals, reflektierte Richtungen, TCP)
            stand_off_mm = float(params.get("stand_off_mm", 10.0))
            src = str(pd_local.meta.get("source", "")).lower()

            rc, rays_hit_poly, _tcp_poly = cast_rays_for_side(
                P_world_start,
                sub_mesh_world=sub_mesh_world,
                side=side,
                source=src,
                stand_off_mm=stand_off_mm,
                ray_len_mm=1000.0,
                lock_xy=True,
            )

            # Hellblaue Messstrahlen
            if rays_hit_poly.n_points > 0:
                self._engine.add_mesh(
                    rays_hit_poly,
                    color="#87CEFA",
                    opacity=1.0,
                    smooth_shading=False,
                    lighting=False,
                    line_width=1.5,
                )

            mask = rc.valid
            if not np.any(mask):
                _LOG.warning("Raycast ergab keine Trefferpunkte.")
                self._focus_camera_isometric(sub_scene)
                return

            # --- Visualisierung direkt aus dem Raycaster ---
            # Gelber Pfad = TCP (immer +stand_off in korrekter Richtung)
            P_tcp = rc.tcp_mm[mask]                   # (N,3) mm
            line_tcp = pv.lines_from_points(P_tcp, close=False)
            self._engine.add_mesh(
                line_tcp,
                color="yellow",
                opacity=1.0,
                smooth_shading=False,
                lighting=False,
                line_width=2.0,
            )

            # Grüne Pfeile = reflektierte Richtung (Spray-Achse)
            dirs = rc.refl_dir[mask]                  # (N,3) normiert
            arrows_pd = pv.PolyData(P_tcp)
            arrows_pd["vectors"] = dirs
            factor = max(5.0, 0.6 * stand_off_mm)
            glyphs = arrows_pd.glyph(orient="vectors", scale=False, factor=factor)
            self._engine.add_mesh(
                glyphs,
                color="green",
                opacity=1.0,
                smooth_shading=False,
                lighting=False,
            )

            # (Optional) Wenn du dennoch eine Trajectory bauen willst, kannst du pd_world
            # aus den Hitpunkten/Nromals erzeugen und TrajectoryBuilder nutzen – für
            # die Visualisierung ist das oben aber korrekt & robust.

        except Exception as e:
            _LOG.error("Path/Raycast/Trajectory fehlgeschlagen: %s", e, exc_info=True)

        # Kamera sauber auf Substrat fokussieren
        self._focus_camera_isometric(sub_scene)

    # ---------- Camera helpers ----------
    def _focus_camera_isometric(self, mesh: pv.PolyData) -> None:
        try:
            center = _center_of_bounds(mesh.bounds)
            diag = _diag_of_bounds(mesh.bounds)
            dist = max(50.0, 2.0 * diag)
            dir_iso = np.array([1.0, 1.0, 1.0], dtype=float)
            dir_iso /= np.linalg.norm(dir_iso) + 1e-12
            pos = center + dir_iso * dist
            viewup = (0.0, 0.0, 1.0)
            self._pv.camera_position = (tuple(pos), tuple(center), viewup)
            self._pv.reset_camera_clipping_range()
            self._pv.render()
        except Exception:
            self._pv.view_isometric()
            self._pv.reset_camera()

    # ---------- Intern ----------
    @staticmethod
    def _get_attr(model: Any, key: str):
        if hasattr(model, key):
            return getattr(model, key)
        if isinstance(model, dict):
            return model.get(key)
        return None


# ---------- Helpers ----------
def _center_of_bounds(bounds):
    return np.array([
        0.5 * (bounds[0] + bounds[1]),
        0.5 * (bounds[2] + bounds[3]),
        0.5 * (bounds[4] + bounds[5]),
    ], dtype=float)

def _diag_of_bounds(bounds) -> float:
    p0 = np.array([bounds[0], bounds[2], bounds[4]], dtype=float)
    p1 = np.array([bounds[1], bounds[3], bounds[5]], dtype=float)
    return float(np.linalg.norm(p1 - p0))

def _rpy_deg_to_matrix_np(rpy_deg):
    import math
    r, p, y = [math.radians(float(v)) for v in (rpy_deg or (0.0, 0.0, 0.0))]
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(r), -math.sin(r)],
                   [0, math.sin(r),  math.cos(r)]])
    Ry = np.array([[ math.cos(p), 0, math.sin(p)],
                   [0,            1, 0           ],
                   [-math.sin(p), 0, math.cos(p)]])
    Rz = np.array([[math.cos(y), -math.sin(y), 0],
                   [math.sin(y),  math.cos(y), 0],
                   [0,            0,           1]])
    return Rz @ Ry @ Rx

def _side_plane_from_bounds(bounds, side: str, path_offset_mm: float, clearance_mm: float) -> Tuple[int, float]:
    """
    Liefert (axis, value) für die Start-Ebene der blauen Maske abhängig von der Side.
    axis: 0=x, 1=y, 2=z. value ist der Ebenenwert in Weltkoordinaten (mm).

    Regeln:
      - top:   z = z_max + offset + clearance
      - front: y = y_max + offset + clearance
      - back:  y = y_min - offset - clearance
      - left:  x = x_min - offset - clearance
      - right: x = x_max + offset + clearance
    """
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    s = (side or "").lower()

    if s == "top":
        return 2, float(z_max + path_offset_mm + clearance_mm)
    if s == "front":
        return 1, float(y_max + path_offset_mm + clearance_mm)
    if s == "back":
        return 1, float(y_min - path_offset_mm - clearance_mm)
    if s == "left":
        return 0, float(x_min - path_offset_mm - clearance_mm)
    if s == "right":
        return 0, float(x_max + path_offset_mm + clearance_mm)

    # Fallback wie "top"
    return 2, float(z_max + path_offset_mm + clearance_mm)
