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

# VTK: CubeAxesActor für das (Substrat ∪ Pfad)-Gitter
try:
    from vtkmodules.vtkRenderingAnnotation import vtkCubeAxesActor
except Exception:  # pragma: no cover
    vtkCubeAxesActor = None  # type: ignore

from .mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)
from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.recipe.scene")

Bounds = Tuple[float, float, float, float, float, float]


@dataclass
class PreviewScene:
    bounds: Bounds
    center: Tuple[float, float, float]
    ground_z: float
    ground_mesh: pv.PolyData
    mount_mesh: Optional[pv.PolyData]
    substrate_mesh: Optional[pv.PolyData]
    mesh_tris: Optional[int]


class SceneManager:
    """
    3-Schritt:
      1) build_scene(...)    -> Boden @ z=0, Mount, Substrat, Grid (nur um Substrat, falls vorhanden)
      2) build_overlays(...) -> Overlays erzeugen und Grid ggf. auf (Substrat ∪ Pfad) ausdehnen
      3) toggle_overlays(...) -> Sichtbarkeit
    """

    # Basis-Layer
    L_GROUND     = "ground"
    L_GRID       = "grid"
    L_MOUNT      = "mount"
    L_SUBSTRATE  = "substrate"

    # Overlay-Layer
    L_PATH       = "path"
    L_PATH_MRK   = "path_markers"
    L_RAYS_HIT   = "rays_hit"
    L_RAYS_MISS  = "rays_miss"
    L_NORMALS    = "normals"
    L_FR_X       = "frames_x"
    L_FR_Y       = "frames_y"
    L_FR_Z       = "frames_z"

    def __init__(self, *, interactor_getter: Callable[[], Any]):
        self._get_ia = interactor_getter
        self._layers: Dict[str, List[Any]] = {}
        # Merker für Grid-Logik
        self._scene_bounds: Optional[Bounds] = None          # Bounds aus build_scene (Substrat oder Mount oder Default)
        self._substrate_bounds: Optional[Bounds] = None      # Nur Substrat-Bounds (falls vorhanden)
        self._last_grid_bounds: Optional[Bounds] = None      # Zuletzt gesetzte Grid-Bounds

    # ------------------------------------------------------------------ helpers
    def _ia(self):
        try:
            return self._get_ia()
        except Exception:
            _LOG.warning("SceneManager: no interactor")
            return None

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
        for lyr in list(self._layers.keys()):
            self._remove_layer_actors(lyr)

    def set_layer_visible(self, layer: str, visible: bool) -> None:
        ia = self._ia()
        if ia is None:
            return
        for a in self._layers.get(layer, []):
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
                if not b:
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

    def add_mesh(self, mesh, *, layer: str, **kwargs) -> Optional[Any]:
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

    def add_raw_actor(self, actor: Any, *, layer: str) -> Optional[Any]:
        ia = self._ia()
        if ia is None or actor is None:
            return None
        try:
            ia.renderer.AddActor(actor)
            self._ensure_layer(layer).append(actor)
            return actor
        except Exception:
            _LOG.exception("add_raw_actor failed")
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
            poly = pv.PolyData(P); poly.lines = lines
            if as_tube:
                try:
                    tube = poly.tube(radius=float(tube_radius),
                                     n_sides=int(tube_sides),
                                     capping=bool(tube_capping))
                    return self.add_mesh(tube, layer=layer, color=color, lighting=lighting)
                except Exception:
                    _LOG.exception("tube generation failed; fallback to line")
            return self.add_mesh(poly, layer=layer, color=color, line_width=float(line_width), lighting=lighting)
        except Exception:
            _LOG.exception("add_path_polyline failed")
            return None

    # ------------------------------------------------------ bounds utilities
    @staticmethod
    def _bounds_from_points(P: np.ndarray) -> Optional[Bounds]:
        try:
            A = np.asarray(P, float).reshape(-1, 3)
            if A.size == 0:
                return None
            lo = np.min(A, axis=0); hi = np.max(A, axis=0)
            return (float(lo[0]), float(hi[0]),
                    float(lo[1]), float(hi[1]),
                    float(lo[2]), float(hi[2]))
        except Exception:
            return None

    @staticmethod
    def _union_bounds(b1: Optional[Bounds], b2: Optional[Bounds]) -> Optional[Bounds]:
        if b1 is None and b2 is None:
            return None
        if b1 is None:
            return b2
        if b2 is None:
            return b1
        xmin = min(b1[0], b2[0]); xmax = max(b1[1], b2[1])
        ymin = min(b1[2], b2[2]); ymax = max(b1[3], b2[3])
        zmin = min(b1[4], b2[4]); zmax = max(b1[5], b2[5])
        return (xmin, xmax, ymin, ymax, zmin, zmax)

    # ------------------------------------------------------ floor / grid utils
    def _make_floor_plane_at_z0(self, bounds: Bounds) -> pv.PolyData:
        xmin, xmax, ymin, ymax, _, _ = bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        return pv.Plane(center=(cx, cy, 0.0), direction=(0, 0, 1),
                        i_size=500.0, j_size=500.0, i_resolution=1, j_resolution=1)

    @staticmethod
    def _pad_bounds(b: Bounds, k: float = 1.02) -> Bounds:
        xmin, xmax, ymin, ymax, zmin, zmax = map(float, b)
        cx, cy, cz = (0.5*(xmin+xmax), 0.5*(ymin+ymax), 0.5*(zmin+zmax))
        sx, sy, sz = (max(xmax-xmin, 1e-6), max(ymax-ymin, 1e-6), max(zmax-zmin, 1e-6))
        return (cx-0.5*sx*k, cx+0.5*sx*k,
                cy-0.5*sy*k, cy+0.5*sy*k,
                cz-0.5*sz*k, cz+0.5*sz*k)

    @staticmethod
    def _hex_to_rgb01(hex_color: str) -> Tuple[float, float, float]:
        h = hex_color.lstrip("#")
        r, g, b = tuple(int(h[i:i+2], 16) for i in (0, 2, 4))
        return (r/255.0, g/255.0, b/255.0)

    def _add_cube_axes_around_bounds(self, *, bounds: Bounds, color: str = "#cfcfcf") -> Optional[Any]:
        """
        Erzeugt einen vtkCubeAxesActor nur um 'bounds'.
        Kompatibel mit älteren VTKs: MinorTicks per Achse abschalten.
        """
        ia = self._ia()
        if ia is None or vtkCubeAxesActor is None:
            return None
        try:
            b = self._pad_bounds(bounds, 1.02)

            axes = vtkCubeAxesActor()
            axes.SetBounds(b)
            try:
                axes.SetCamera(ia.camera)
            except Exception:
                pass

            axes.SetXTitle("X (mm)")
            axes.SetYTitle("Y (mm)")
            axes.SetZTitle("Z (mm)")
            axes.SetTickLocationToOutside()
            axes.DrawXGridlinesOn()
            axes.DrawYGridlinesOn()
            axes.DrawZGridlinesOn()

            # Minor Ticks (versionssicher) abschalten
            for fn in ("SetXAxisMinorTickVisibility",
                       "SetYAxisMinorTickVisibility",
                       "SetZAxisMinorTickVisibility"):
                if hasattr(axes, fn):
                    getattr(axes, fn)(0)

            r, g, bl = self._hex_to_rgb01(color)
            try:
                for i in (0, 1, 2):  # X,Y,Z
                    axes.GetTitleTextProperty(i).SetColor(r, g, bl)
                    axes.GetLabelTextProperty(i).SetColor(r, g, bl)
                    axes.GetLabelTextProperty(i).SetFontSize(14)
            except Exception:
                pass

            try:
                prop = axes.GetXAxesLinesProperty()
                if prop: prop.SetLineWidth(1.0)
                prop = axes.GetYAxesLinesProperty()
                if prop: prop.SetLineWidth(1.0)
                prop = axes.GetZAxesLinesProperty()
                if prop: prop.SetLineWidth(1.0)
            except Exception:
                pass

            ia.renderer.AddActor(axes)
            self._ensure_layer(self.L_GRID).append(axes)
            self._last_grid_bounds = bounds
            return axes
        except Exception:
            _LOG.exception("vtkCubeAxesActor creation failed")
            return None

    def _add_plane_grid_fallback(self, bounds: Bounds, *, step: float = 10.0, color: str = "#cfcfcf"):
        xmin, xmax, ymin, ymax, zmin, _ = bounds
        width, height = xmax - xmin, ymax - ymin
        i_res = max(1, int(round(width / max(1e-6, step))))
        j_res = max(1, int(round(height / max(1e-6, step))))
        center = (0.5*(xmin+xmax), 0.5*(ymin+ymax), zmin)
        plane = pv.Plane(center=center, direction=(0, 0, 1),
                         i_size=width, j_size=height,
                         i_resolution=i_res, j_resolution=j_res)
        act = self.add_mesh(plane, layer=self.L_GRID, style="wireframe",
                            color=color, line_width=1.0, lighting=False)
        if act is not None:
            self._last_grid_bounds = bounds
        return act

    def _set_grid_around(self, bounds: Bounds, *, color: str = "#cfcfcf", step: float = 10.0) -> None:
        """
        Layer L_GRID auf neue Bounds setzen (löscht vorherige Grid-Actors).
        """
        self.clear_layer(self.L_GRID)
        ok = self._add_cube_axes_around_bounds(bounds=bounds, color=color)
        if ok is None:
            self._add_plane_grid_fallback(bounds, step=step, color=color)

    # ============================================================ 1) build_scene
    def build_scene(
        self,
        ctx,
        model: Recipe,
        *,
        grid_step_mm: float = 10.0,
    ) -> PreviewScene:

        for lyr in (self.L_GROUND, self.L_GRID, self.L_MOUNT, self.L_SUBSTRATE):
            self.clear_layer(lyr)

        self._substrate_bounds = None
        self._last_grid_bounds = None

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
                    if callable(getattr(smesh, "is_all_triangles", None)) and not smesh.is_all_triangles():
                        smesh = smesh.triangulate()
                except Exception:
                    pass
            except Exception as e:
                _LOG.error("Substrat-Mesh Fehler: %s", e, exc_info=True)

        # Bounds ableiten
        if smesh is not None:
            bounds: Bounds = smesh.bounds  # type: ignore[assignment]
            self._substrate_bounds = bounds
        elif mmesh is not None:
            bounds = mmesh.bounds  # type: ignore[assignment]
        else:
            bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        self._scene_bounds = bounds

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        cx, cy = 0.5*(xmin+xmax), 0.5*(ymin+ymax)

        # Boden @ z=0
        ground = self._make_floor_plane_at_z0(bounds)
        self.add_mesh(ground, layer=self.L_GROUND, color="#3a3a3a", opacity=1.0, lighting=False)

        # Mount
        if mmesh is not None:
            self.add_mesh(mmesh, layer=self.L_MOUNT, color="#5d5d5d", opacity=0.95, lighting=False)

        # Substrat
        if smesh is not None:
            self.add_mesh(smesh, layer=self.L_SUBSTRATE, color="#d0d6dd", opacity=1.0, lighting=False)
            # Erstes Grid nur um Substrat
            self._set_grid_around(smesh.bounds, color="#cfcfcf", step=grid_step_mm)
        else:
            # kein Substrat -> Fallback-Grid um Szene
            self._set_grid_around(bounds, color="#cfcfcf", step=grid_step_mm)

        mesh_tris = None
        if smesh is not None:
            try:
                mesh_tris = int(smesh.n_faces) if hasattr(smesh, "n_faces") else None
            except Exception:
                mesh_tris = None

        return PreviewScene(
            bounds=bounds,
            center=(cx, cy, 0.5*(zmin+zmax)),
            ground_z=0.0,
            ground_mesh=ground,
            mount_mesh=mmesh,
            substrate_mesh=smesh,
            mesh_tris=mesh_tris,
        )

    # ======================================================= 2) build_overlays
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

        for lyr in (self.L_PATH, self.L_PATH_MRK, self.L_RAYS_HIT, self.L_RAYS_MISS,
                    self.L_NORMALS, self.L_FR_X, self.L_FR_Y, self.L_FR_Z):
            self.clear_layer(lyr)

        path_bounds = None
        if path_xyz is not None:
            try:
                self.add_path_polyline(path_xyz, layer=self.L_PATH, color="#2ecc71",
                                       as_tube=as_tube, tube_radius=tube_radius, lighting=False)
            except Exception:
                _LOG.exception("build_overlays: path failed")
            # Bounds des Pfades bestimmen
            path_bounds = self._bounds_from_points(path_xyz)

        # Grid nun ggf. auf (Substrat ∪ Pfad) ausdehnen:
        union_b = self._union_bounds(self._substrate_bounds, path_bounds)
        if union_b is None:
            union_b = self._scene_bounds
        # Nur neu setzen, wenn sich die Bounds merklich geändert haben
        if union_b is not None:
            if (self._last_grid_bounds is None) or any(abs(a - b) > 1e-9 for a, b in zip(union_b, self._last_grid_bounds)):
                self._set_grid_around(union_b, color="#cfcfcf", step=10.0)

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

        if normals_xyz is not None:
            try:
                P = np.asarray(normals_xyz, float).reshape(-1, 6)  # [x,y,z,nx,ny,nz]
                segs = []
                for x, y, z, nx, ny, nz in P:
                    segs.extend([(x, y, z), (x+nx, y+ny, z+nz)])
                if segs:
                    poly = pv.PolyData(np.asarray(segs, float))
                    lines = []
                    for i in range(0, len(segs), 2):
                        lines.extend([2, i, i+1])
                    poly.lines = np.asarray(lines, dtype=np.int64)
                    self.add_mesh(poly, layer=self.L_NORMALS, color="#9b59b6", line_width=1.0, lighting=False)
            except Exception:
                _LOG.exception("build_overlays: normals failed")

        if frames_at is not None:
            try:
                P = np.asarray(frames_at, float).reshape(-1, 6)  # [x,y,z, sx,sy,sz]
                def _axis_segments(dir_idx):
                    seg = []
                    for x, y, z, sx, sy, sz in P:
                        L = float(np.linalg.norm([sx, sy, sz])) or 10.0
                        dx, dy, dz = (L, 0, 0) if dir_idx == 0 else ((0, L, 0) if dir_idx == 1 else (0, 0, L))
                        seg.extend([(x, y, z), (x+dx, y+dy, z+dz)])
                    return seg
                def _to_actor(segs, layer, color):
                    if not segs: return
                    poly = pv.PolyData(np.asarray(segs, float))
                    lines = []
                    for i in range(0, len(segs), 2):
                        lines.extend([2, i, i+1])
                    poly.lines = np.asarray(lines, dtype=np.int64)
                    self.add_mesh(poly, layer=layer, color=color, line_width=1.0, lighting=False)
                _to_actor(_axis_segments(0), self.L_FR_X, "#e67e22")
                _to_actor(_axis_segments(1), self.L_FR_Y, "#16a085")
                _to_actor(_axis_segments(2), self.L_FR_Z, "#2980b9")
            except Exception:
                _LOG.exception("build_overlays: frames failed")

    # ======================================================= 3) toggle_overlays
    def toggle_overlays(self, visibility: Dict[str, bool]) -> None:
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

    # --------------------------------------------------------------- view update
    def update_current_views_once(self, *, refresh_2d: Callable[[], None] | None = None) -> None:
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
