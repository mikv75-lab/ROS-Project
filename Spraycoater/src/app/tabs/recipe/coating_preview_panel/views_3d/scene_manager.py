# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Any, Tuple

import numpy as np
import pyvista as pv

try:
    from pyvistaqt import QtInteractor  # type: ignore
except Exception:
    QtInteractor = None  # type: ignore

from .mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)
from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.recipe.scene")

Bounds = Tuple[float, float, float, float, float, float]


# ---------------------- Data ----------------------

@dataclass
class PreviewScene:
    bounds: Bounds
    center: Tuple[float, float, float]
    ground_z: float
    ground_mesh: pv.PolyData
    mount_mesh: Optional[pv.PolyData]
    substrate_mesh: Optional[pv.PolyData]
    mesh_tris: Optional[int]


# ---------------------- Manager ----------------------

class SceneManager:
    """
    Dünne Schicht über dem PyVista-Interactor mit Layer-Verwaltung + Helpers.
    3-Schritt-Ablauf:
      1) build_scene(...)      -> Floor (z=0), Mount, Grid (z=0, ohne Labels), Substrate
      2) build_overlays(...)   -> Overlay-Actors erzeugen/aktualisieren
      3) toggle_overlays(...)  -> Sichtbarkeit je Checkbox-States
    """

    # Layers: Basisszene
    L_GROUND     = "ground"
    L_GRID       = "grid"
    L_MOUNT      = "mount"
    L_SUBSTRATE  = "substrate"

    # Layers: Overlays
    L_PATH       = "path"
    L_PATH_MRK   = "path_markers"
    L_RAYS_HIT   = "rays_hit"
    L_RAYS_MISS  = "rays_miss"
    L_NORMALS    = "normals"
    L_FR_X       = "frames_x"
    L_FR_Y       = "frames_y"
    L_FR_Z       = "frames_z"
    # (keine labels mehr)

    def __init__(self, *, interactor_getter: Callable[[], Any]):
        self._get_ia = interactor_getter
        self._layers: Dict[str, List[Any]] = {}

    # --- Interactor ---------------------------------------------------------
    def _ia(self):
        ia = None
        try:
            ia = self._get_ia()
        except Exception:
            pass
        if ia is None:
            _LOG.warning("SceneManager: Interactor not available (external getter returned None)")
        return ia

    # --- Layer mgmt ----------------------------------------------------------
    def _ensure_layer(self, layer: str) -> List[Any]:
        if layer not in self._layers:
            self._layers[layer] = []
        return self._layers[layer]

    def _remove_layer_actors(self, layer: str) -> None:
        ia = self._ia()
        if ia is None:
            return
        for a in self._layers.get(layer, []):
            try:
                ia.remove_actor(a)
            except Exception:
                pass
        self._layers[layer] = []

    def clear_layer(self, layer: str) -> None:
        self._remove_layer_actors(layer)

    def clear(self) -> None:
        ia = self._ia()
        if ia is None:
            return
        try:
            for layer in list(self._layers.keys()):
                self._remove_layer_actors(layer)
        except Exception:
            _LOG.exception("clear failed")

    def set_layer_visible(self, layer: str, visible: bool) -> None:
        ia = self._ia()
        if ia is None:
            return
        actors = self._layers.get(layer, [])
        for a in actors:
            try:
                a.SetVisibility(1 if visible else 0)
            except Exception:
                pass

    def get_layer_bounds(self, layer: str) -> Optional[Bounds]:
        actors = self._layers.get(layer, [])
        if not actors:
            return None
        mins = np.array([np.inf, np.inf, np.inf], float)
        maxs = np.array([-np.inf, -np.inf, -np.inf], float)
        any_ok = False
        for a in actors:
            try:
                b = a.GetBounds()
                if b is None:
                    continue
                b = np.asarray(b, float).reshape(6)
                if not np.all(np.isfinite(b)):
                    continue
                mins = np.minimum(mins, [b[0], b[2], b[4]])
                maxs = np.maximum(maxs, [b[1], b[3], b[5]])
                any_ok = True
            except Exception:
                pass
        if not any_ok:
            return None
        return (float(mins[0]), float(maxs[0]),
                float(mins[1]), float(maxs[1]),
                float(mins[2]), float(maxs[2]))

    # --- Primitive Zeichen-Helper -------------------------------------------
    def add_mesh(self, mesh, *, layer: str, **kwargs) -> Optional[Any]:
        """
        Zeichnet ein beliebiges pv-Objekt, speichert Actor in Layer.
        Niemals automatisch rendern – das macht der Aufrufer am Ende EINMAL.
        """
        ia = self._ia()
        if ia is None or mesh is None:
            return None
        try:
            actor = ia.add_mesh(mesh, **kwargs)
            self._ensure_layer(layer).append(actor)
            return actor
        except Exception:
            _LOG.exception("add_mesh failed")
            return None

    def add_path_polyline(
        self,
        points_mm: np.ndarray,
        *,
        layer: str = L_PATH,
        color: str = "#2ecc71",
        line_width: float = 2.0,
        lighting: bool = False,
        as_tube: bool = False,
        tube_radius: float = 0.8,
        tube_sides: int = 12,
        tube_capping: bool = False,
    ) -> Optional[Any]:
        try:
            P = np.asarray(points_mm, float).reshape(-1, 3)
            if len(P) < 2:
                return None
            lines = np.hstack([[len(P)], np.arange(len(P), dtype=np.int64)])
            poly = pv.PolyData(P)
            poly.lines = lines
            if as_tube:
                try:
                    tube = poly.tube(
                        radius=float(tube_radius),
                        n_sides=int(tube_sides),
                        capping=bool(tube_capping),
                    )
                    return self.add_mesh(
                        tube, layer=layer, color=color, lighting=lighting
                    )
                except Exception:
                    _LOG.exception("tube generation from polyline failed; fallback to line")
            return self.add_mesh(
                poly, layer=layer, color=color, line_width=float(line_width), lighting=lighting
            )
        except Exception:
            _LOG.exception("add_path_polyline failed")
            return None

    # --- Floor primitives (ohne Labels!) ------------------------------------
    def _make_floor_plane(self, bounds: Bounds) -> pv.PolyData:
        # Floor immer bei z = 0
        xmin, xmax, ymin, ymax, _zmin, _ = bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        plane = pv.Plane(
            center=(cx, cy, 0.0),
            direction=(0, 0, 1),
            i_size=500.0,
            j_size=500.0,
            i_resolution=1,
            j_resolution=1,
        )
        return plane

    def add_floor_grid(self, bounds: Bounds, *, step: float = 10.0, layer: str = L_GRID, color: str = "#cfcfcf"):
        # Grid wieder an der alten Position: z = zmin (Kontakt-Ebene Mount/Substrat)
        xmin, xmax, ymin, ymax, zmin, _ = bounds
        width, height = xmax - xmin, ymax - ymin
        i_res = max(1, int(round(width / max(1e-6, step))))
        j_res = max(1, int(round(height / max(1e-6, step))))
        center = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin)  # <- zmin statt 0.0

        plane = pv.Plane(
            center=center,
            direction=(0, 0, 1),
            i_size=width,
            j_size=height,
            i_resolution=i_res,
            j_resolution=j_res,
        )
        return self.add_mesh(
            plane, layer=layer, style="wireframe", color=color, line_width=1.0, lighting=False
        )

    # ===================== API: 1) Build Scene =====================
    def build_scene(
        self,
        ctx,
        model: Recipe,
        *,
        grid_step_mm: float = 10.0,
    ) -> PreviewScene:
        """
        Baut NUR die Basisszene:
          - Floor-Plane (z=0, massiv)
          - Mount (falls vorhanden)
          - Grid (z=0, wireframe)  -> KEINE LABELS
          - Substrate (falls vorhanden)
        Kein Render-Call hier – der Aufrufer rendert am Ende EINMAL.
        """
        # vorherige Szene löschen
        for lyr in (self.L_GROUND, self.L_GRID, self.L_MOUNT, self.L_SUBSTRATE):
            self.clear_layer(lyr)

        mount_key = model.substrate_mount
        substrate_key = model.substrate

        mmesh: pv.PolyData | None = None
        smesh: pv.PolyData | None = None

        if mount_key:
            try:
                mmesh = load_mount_mesh_from_key(ctx, mount_key)
            except Exception as e:
                _LOG.error("Mount-Mesh Fehler: %s", e, exc_info=True)

        if substrate_key:
            try:
                smesh = load_substrate_mesh_from_key(ctx, substrate_key)
                if mount_key:
                    smesh = place_substrate_on_mount(ctx, smesh, mount_key=mount_key)
                try:
                    is_tris = smesh.is_all_triangles() if callable(getattr(smesh, "is_all_triangles", None)) else True
                    if not bool(is_tris):
                        smesh = smesh.triangulate()
                except Exception:
                    pass
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

        # sichere Default-Bounds
        if smesh is not None:
            bounds: Bounds = smesh.bounds  # type: ignore[assignment]
        elif mmesh is not None:
            bounds = mmesh.bounds  # type: ignore[assignment]
        else:
            bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)

        # Floor plane (massiv, z=0)
        ground = self._make_floor_plane(bounds)
        self.add_mesh(ground, layer=self.L_GROUND, color="#3a3a3a", opacity=1.0, lighting=False)

        # Mount
        if mmesh is not None:
            self.add_mesh(mmesh, layer=self.L_MOUNT, color="#5d5d5d", opacity=0.95, lighting=False)

        # Grid (wireframe, z=0) – KEINE Labels
        self.add_floor_grid(bounds, step=grid_step_mm, layer=self.L_GRID, color="#cfcfcf")

        # Substrate
        if smesh is not None:
            self.add_mesh(smesh, layer=self.L_SUBSTRATE, color="#d0d6dd", opacity=1.0, lighting=False)

        mesh_tris = None
        if smesh is not None:
            try:
                mesh_tris = int(smesh.n_faces) if hasattr(smesh, "n_faces") else None
            except Exception:
                mesh_tris = None

        return PreviewScene(
            bounds=bounds,
            center=(cx, cy, 0.5 * (zmin + zmax)),
            ground_z=0.0,  # immer 0
            ground_mesh=ground,
            mount_mesh=mmesh,
            substrate_mesh=smesh,
            mesh_tris=mesh_tris,
        )

    # ===================== API: 2) Build Overlays =====================
    def build_overlays(
        self,
        *,
        path_xyz: Optional[np.ndarray] = None,
        normals_xyz: Optional[np.ndarray] = None,
        frames_at: Optional[np.ndarray] = None,
        rays_hit: Optional[np.ndarray] = None,
        rays_miss: Optional[np.ndarray] = None,
        as_tube: bool = False,
        tube_radius: float = 0.8,
    ) -> None:
        """
        Erstellt/aktualisiert die Overlay-Actors in ihren Layern.
        Sichtbarkeit wird HIER NICHT gesetzt – das passiert in toggle_overlays().
        """

        # Layer leeren (nur Overlays)
        for lyr in (self.L_PATH, self.L_PATH_MRK, self.L_RAYS_HIT, self.L_RAYS_MISS,
                    self.L_NORMALS, self.L_FR_X, self.L_FR_Y, self.L_FR_Z):
            self.clear_layer(lyr)

        # Path
        if path_xyz is not None:
            try:
                self.add_path_polyline(
                    path_xyz, layer=self.L_PATH, color="#2ecc71",
                    as_tube=as_tube, tube_radius=tube_radius, lighting=False
                )
            except Exception:
                _LOG.exception("build_overlays: path build failed")

        # Rays
        def _add_pts(pts: Optional[np.ndarray], layer: str, color: str):
            if pts is None:
                return
            try:
                pts = np.asarray(pts, float).reshape(-1, 3)
                poly = pv.PolyData(pts)
                self.add_mesh(poly, layer=layer, color=color, point_size=6.0, render_points_as_spheres=True)
            except Exception:
                _LOG.exception("build_overlays: points failed (%s)", layer)

        _add_pts(rays_hit,  self.L_RAYS_HIT,  "#3498db")
        _add_pts(rays_miss, self.L_RAYS_MISS, "#e74c3c")

        # Normals (als Linien)
        if normals_xyz is not None:
            try:
                P = np.asarray(normals_xyz, float).reshape(-1, 6)  # [x,y,z,nx,ny,nz]
                segs = []
                for x, y, z, nx, ny, nz in P:
                    p0 = (x, y, z)
                    p1 = (x + nx, y + ny, z + nz)
                    segs.extend([p0, p1])
                if segs:
                    poly = pv.PolyData(np.asarray(segs, float))
                    # 2-Punkt-Segmente
                    lines = []
                    for i in range(0, len(segs), 2):
                        lines.extend([2, i, i + 1])
                    poly.lines = np.asarray(lines, dtype=np.int64)
                    self.add_mesh(poly, layer=self.L_NORMALS, color="#9b59b6", line_width=1.0, lighting=False)
            except Exception:
                _LOG.exception("build_overlays: normals failed")

        # Frames (x/y/z Achsen ohne Labels)
        if frames_at is not None:
            try:
                P = np.asarray(frames_at, float).reshape(-1, 6)  # [x,y,z, ux,uy,uz] -> ux..uz = up/scale
                seg_x, seg_y, seg_z = [], [], []
                for x, y, z, sx, sy, sz in P:
                    # einfache Achsen mit Länge = sqrt(sx^2 + sy^2 + sz^2)
                    L = float(np.linalg.norm([sx, sy, sz])) or 10.0
                    seg_x.extend([(x, y, z), (x + L, y, z)])
                    seg_y.extend([(x, y, z), (x, y + L, z)])
                    seg_z.extend([(x, y, z), (x, y, z + L)])
                def _segs_to_actor(segs, layer, color):
                    if not segs:
                        return
                    poly = pv.PolyData(np.asarray(segs, float))
                    lines = []
                    for i in range(0, len(segs), 2):
                        lines.extend([2, i, i + 1])
                    poly.lines = np.asarray(lines, dtype=np.int64)
                    self.add_mesh(poly, layer=layer, color=color, line_width=1.0, lighting=False)
                _segs_to_actor(seg_x, self.L_FR_X, "#e67e22")
                _segs_to_actor(seg_y, self.L_FR_Y, "#16a085")
                _segs_to_actor(seg_z, self.L_FR_Z, "#2980b9")
            except Exception:
                _LOG.exception("build_overlays: frames failed")

    # ===================== API: 3) Toggle Overlays =====================
    def toggle_overlays(self, visibility: Dict[str, bool]) -> None:
        """
        Sichtbarkeit der Overlay-Layer schalten (Checkbox-States).
        Keys erwartet wie in deinem UI: path, hits, misses, normals, frames
        """
        show_path   = bool(visibility.get("path", True))
        show_hits   = bool(visibility.get("hits", True))
        show_misses = bool(visibility.get("misses", True))
        show_norms  = bool(visibility.get("normals", False))
        show_frames = bool(visibility.get("frames", False))

        self.set_layer_visible(self.L_PATH,      show_path)
        self.set_layer_visible(self.L_PATH_MRK,  show_path)
        self.set_layer_visible(self.L_RAYS_HIT,  show_hits)
        self.set_layer_visible(self.L_RAYS_MISS, show_misses)
        self.set_layer_visible(self.L_NORMALS,   show_norms)
        self.set_layer_visible(self.L_FR_X,      show_frames)
        self.set_layer_visible(self.L_FR_Y,      show_frames)
        self.set_layer_visible(self.L_FR_Z,      show_frames)

    # ===================== Utilities =====================
    def update_current_views_once(self, *, refresh_2d: Callable[[], None] | None = None) -> None:
        """
        Einmaliger Update der aktuellen Views:
          - 3D: clipping-range reset + render()
          - 2D: refresh() (falls Callback gegeben)
        """
        ia = self._ia()
        try:
            if ia is not None and hasattr(ia, "reset_camera_clipping_range"):
                ia.reset_camera_clipping_range()
            if ia is not None and hasattr(ia, "render"):
                ia.render()
        except Exception:
            _LOG.exception("update_current_views_once: 3D failed")
        if callable(refresh_2d):
            try:
                refresh_2d()
            except Exception:
                _LOG.exception("update_current_views_once: 2D failed")
