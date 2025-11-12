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

try:
    from vtkmodules.vtkRenderingAnnotation import vtkCubeAxesActor
except Exception:
    vtkCubeAxesActor = None  # type: ignore

from .mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)
from app.model.recipe.recipe import Recipe

_LOG = logging.getLogger("app.tabs.recipe.scene")

Bounds = Tuple[float, float, float, float, float, float]

# ------------------------------------------------------------
# Skalen-Parameter
# ------------------------------------------------------------
MIN_STEP_MM: float = 1.0
MAX_STEP_MM: float = 10.0
PREFERRED_STEPS = (1.0, 2.0, 5.0, 10.0)

MAX_LABELS_PER_AXIS = 60     # Sicherheitskappe (gegen Label-Spam)
TARGET_LABELS_PER_AXIS = 20  # Wunschdichte

AXIS_LINE_COLOR = "#000000"  # Achsen/Gitter
TITLE_COLOR     = "#000000"  # Titelfarbe
LABEL_COLOR     = "#000000"  # Zahlen
TITLE_SIZE_PX   = 14
LABEL_SIZE_PX   = 14


# ============================ Data ============================

@dataclass
class PreviewScene:
    bounds: Bounds
    center: Tuple[float, float, float]
    ground_z: float
    ground_mesh: pv.PolyData
    mount_mesh: Optional[pv.PolyData]
    substrate_mesh: Optional[pv.PolyData]
    mesh_tris: Optional[int]


# ============================ Manager ============================

class SceneManager:
    """
    3-Schritt:
      1) build_scene(...)    -> Boden @ z=0, Mount, Substrat, Grid (CubeAxes) um Substrat
      2) build_overlays(...) -> Overlays erzeugen; Grid ggf. auf Union(Substrat, Pfad) ausdehnen
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
        self._last_grid_bounds: Optional[Bounds] = None
        self._substrate_bounds: Optional[Bounds] = None

    # ------------------- Interactor / Layer mgmt -------------------

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
        self._last_grid_bounds = None
        self._substrate_bounds = None

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

    # ------------------- Zeichen-Helfer -------------------

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

    # ------------------- Floor / Grid utils -------------------

    def _make_floor_plane_at_z0(self, bounds: Bounds) -> pv.PolyData:
        xmin, xmax, ymin, ymax, _, _ = bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        return pv.Plane(center=(cx, cy, 0.0), direction=(0, 0, 1),
                        i_size=500.0, j_size=500.0, i_resolution=1, j_resolution=1)

    @staticmethod
    def _hex_to_rgb01(hex_color: str) -> Tuple[float, float, float]:
        s = hex_color.strip().lstrip("#")
        if len(s) == 3:
            s = "".join(ch*2 for ch in s)
        try:
            r = int(s[0:2], 16) / 255.0
            g = int(s[2:4], 16) / 255.0
            b = int(s[4:6], 16) / 255.0
        except Exception:
            r, g, b = (0.0, 0.0, 0.0)
        return (r, g, b)

    # ------------------- Dynamische Stepwahl -------------------

    @staticmethod
    def _pick_step(span: float,
                   *,
                   min_step: float = MIN_STEP_MM,
                   max_step: float = MAX_STEP_MM,
                   preferred: Tuple[float, ...] = PREFERRED_STEPS,
                   target_labels: int = TARGET_LABELS_PER_AXIS,
                   max_labels: int = MAX_LABELS_PER_AXIS) -> float:
        """
        Wählt einen 'schönen' Schritt (1,2,5,10) zwischen min_step..max_step,
        sodass span/step ~ target_labels (und <= max_labels) bleibt.
        """
        if not np.isfinite(span) or span <= 0:
            return min_step
        # erste grobe Schätzung
        rough = span / max(1, target_labels)
        # auf preferred snappen
        # suche den kleinsten preferred >= rough
        cand = None
        for s in preferred:
            if s >= rough:
                cand = s
                break
        if cand is None:
            cand = preferred[-1]
        # clamp 1..10
        cand = float(min(max(cand, min_step), max_step))
        # falls zu viele Labels -> nächstgrößeres preferred nehmen
        while (span / cand) > max_labels:
            # erhöhe auf nächstes preferred (falls möglich)
            idx = max(0, list(preferred).index(cand))
            if idx < len(preferred) - 1:
                cand = preferred[idx + 1]
            else:
                break
        # falls extrem wenige Labels und wir dürfen kleiner
        while (span / cand) < max(4, target_labels * 0.35) and cand > min_step:
            idx = max(0, list(preferred).index(cand))
            if idx > 0:
                cand = preferred[idx - 1]
            else:
                break
        return float(min(max(cand, min_step), max_step))

    @staticmethod
    def _snap_range(min_v: float, max_v: float, step: float, *, force_min: Optional[float] = None) -> Tuple[float, float, int]:
        """
        Snappt [min_v,max_v] auf Vielfache von 'step'.
        force_min: wenn gesetzt, wird das Minimum auf diesen Wert fixiert (z.B. Z=0).
        Rückgabe: (snapped_min, snapped_max, label_count)
        """
        if not np.isfinite(min_v) or not np.isfinite(max_v):
            return (0.0, 1.0, 2)
        if max_v < min_v:
            min_v, max_v = max_v, min_v

        smin = float(force_min) if force_min is not None else np.floor(min_v / step) * step
        smax = np.ceil(max_v / step) * step
        n = int(round((smax - smin) / step)) + 1
        n = max(2, min(MAX_LABELS_PER_AXIS, n))
        return (float(smin), float(smax), n)

    def _snap_bounds_dynamic(self, b: Bounds) -> Tuple[Bounds, Tuple[int, int, int]]:
        xmin, xmax, ymin, ymax, zmin, zmax = map(float, b)

        x_step = self._pick_step(xmax - xmin)
        y_step = self._pick_step(ymax - ymin)
        z_step = self._pick_step(zmax - zmin)

        sxmin, sxmax, nx = self._snap_range(xmin, xmax, x_step)
        symin, symax, ny = self._snap_range(ymin, ymax, y_step)
        # Z unten auf 0 erzwingen
        szmin, szmax, nz = self._snap_range(zmin, zmax, z_step, force_min=0.0)

        return (sxmin, sxmax, symin, symax, szmin, szmax), (nx, ny, nz)

    # ------------------- CubeAxes -------------------

    def _add_cube_axes_around_bounds(self, *, bounds: Bounds) -> Optional[Any]:
        ia = self._ia()
        if ia is None or vtkCubeAxesActor is None:
            return None
        try:
            snapped_bounds, (nx, ny, nz) = self._snap_bounds_dynamic(bounds)

            axes = vtkCubeAxesActor()
            axes.SetBounds(snapped_bounds)
            try:
                axes.SetCamera(ia.camera)
            except Exception:
                pass

            # Ticks / Gridlines
            axes.SetTickLocationToOutside()
            axes.DrawXGridlinesOn()
            axes.DrawYGridlinesOn()
            axes.DrawZGridlinesOn()

            # Minor Ticks aus
            for fn in ("SetXAxisMinorTickVisibility",
                       "SetYAxisMinorTickVisibility",
                       "SetZAxisMinorTickVisibility"):
                if hasattr(axes, fn):
                    getattr(axes, fn)(0)

            # Anzahl Labels entsprechend Snap
            if hasattr(axes, "SetXAxisNumberOfLabels"): axes.SetXAxisNumberOfLabels(int(nx))
            if hasattr(axes, "SetYAxisNumberOfLabels"): axes.SetYAxisNumberOfLabels(int(ny))
            if hasattr(axes, "SetZAxisNumberOfLabels"): axes.SetZAxisNumberOfLabels(int(nz))

            # Titel
            if hasattr(axes, "SetXTitle"):
                axes.SetXTitle("X (mm)")
                axes.SetYTitle("Y (mm)")
                axes.SetZTitle("Z (mm)")

            # Text-Properties
            title_rgb = self._hex_to_rgb01(TITLE_COLOR)
            label_rgb = self._hex_to_rgb01(LABEL_COLOR)
            for i in (0, 1, 2):
                try:
                    tp = axes.GetTitleTextProperty(i)
                    if tp:
                        if hasattr(tp, "SetBold"):     tp.SetBold(1)
                        if hasattr(tp, "SetFontSize"): tp.SetFontSize(TITLE_SIZE_PX)
                        if hasattr(tp, "SetColor"):    tp.SetColor(*title_rgb)
                except Exception:
                    pass
                try:
                    lp = axes.GetLabelTextProperty(i)
                    if lp:
                        if hasattr(lp, "SetBold"):     lp.SetBold(1)
                        if hasattr(lp, "SetFontSize"): lp.SetFontSize(LABEL_SIZE_PX)
                        if hasattr(lp, "SetColor"):    lp.SetColor(*label_rgb)
                except Exception:
                    pass

            # Linienfarben schwarz
            r, g, bl = self._hex_to_rgb01(AXIS_LINE_COLOR)
            try:
                p = axes.GetXAxesLinesProperty();          p and p.SetColor(r, g, bl)
                p = axes.GetYAxesLinesProperty();          p and p.SetColor(r, g, bl)
                p = axes.GetZAxesLinesProperty();          p and p.SetColor(r, g, bl)
                p = axes.GetXAxesGridlinesProperty();      p and p.SetColor(r, g, bl)
                p = axes.GetYAxesGridlinesProperty();      p and p.SetColor(r, g, bl)
                p = axes.GetZAxesGridlinesProperty();      p and p.SetColor(r, g, bl)
                p = axes.GetXAxesInnerGridlinesProperty(); p and p.SetColor(r, g, bl)
                p = axes.GetYAxesInnerGridlinesProperty(); p and p.SetColor(r, g, bl)
                p = axes.GetZAxesInnerGridlinesProperty(); p and p.SetColor(r, g, bl)
            except Exception:
                pass

            # Vorherige Grid-Actors ersetzen
            self.clear_layer(self.L_GRID)
            ia.renderer.AddActor(axes)
            self._ensure_layer(self.L_GRID).append(axes)
            self._last_grid_bounds = snapped_bounds
            return axes
        except Exception:
            _LOG.exception("vtkCubeAxesActor creation failed")
            return None

    # ===================== 1) build_scene =====================

    def build_scene(
        self,
        ctx,
        model: Recipe,
        *,
        grid_step_mm: float = 10.0,  # Signatur-kompatibel, aktuell ungenutzt
    ) -> PreviewScene:

        for lyr in (self.L_GROUND, self.L_GRID, self.L_MOUNT, self.L_SUBSTRATE):
            self.clear_layer(lyr)
        self._last_grid_bounds = None
        self._substrate_bounds = None

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

        # Bounds ableiten (für Floor/Sicht)
        if smesh is not None:
            bounds: Bounds = smesh.bounds  # type: ignore[assignment]
        elif mmesh is not None:
            bounds = mmesh.bounds  # type: ignore[assignment]
        else:
            bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        cx, cy = 0.5*(xmin+xmax), 0.5*(ymin+ymax)

        # Boden @ z=0
        ground = self._make_floor_plane_at_z0(bounds)
        self.add_mesh(ground, layer=self.L_GROUND, color="#3a3a3a", opacity=1.0, lighting=False)

        # Mount
        if mmesh is not None:
            self.add_mesh(mmesh, layer=self.L_MOUNT, color="#5d5d5d", opacity=0.95, lighting=False)

        # Substrat + Grid
        if smesh is not None:
            self.add_mesh(smesh, layer=self.L_SUBSTRATE, color="#d0d6dd", opacity=1.0, lighting=False)
            self._substrate_bounds = smesh.bounds  # merken
            self._add_cube_axes_around_bounds(bounds=self._substrate_bounds)
        else:
            self.clear_layer(self.L_GRID)
            self._substrate_bounds = None
            self._last_grid_bounds = None

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

    # ===================== 2) build_overlays =====================

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

        # Overlays zurücksetzen
        for lyr in (self.L_PATH, self.L_PATH_MRK, self.L_RAYS_HIT, self.L_RAYS_MISS,
                    self.L_NORMALS, self.L_FR_X, self.L_FR_Y, self.L_FR_Z):
            self.clear_layer(lyr)

        # Pfad
        if path_xyz is not None:
            try:
                self.add_path_polyline(path_xyz, layer=self.L_PATH, color="#2ecc71",
                                       as_tube=as_tube, tube_radius=tube_radius, lighting=False)
            except Exception:
                _LOG.exception("build_overlays: path failed")

        # Rays: Punkte
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

        # Normals
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

        # Frames (x/y/z)
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

        # Grid an Pfad ausdehnen (mit dynamischem Snap)
        if self._substrate_bounds is not None and path_xyz is not None and len(np.asarray(path_xyz).reshape(-1, 3)) > 0:
            try:
                P = np.asarray(path_xyz, float).reshape(-1, 3)
                pxmin, pymin, pzmin = np.min(P, axis=0)
                pxmax, pymax, pzmax = np.max(P, axis=0)
                pb: Bounds = (float(pxmin), float(pxmax),
                              float(pymin), float(pymax),
                              float(pzmin), float(pzmax))
                sb = self._substrate_bounds
                merged: Bounds = (min(sb[0], pb[0]), max(sb[1], pb[1]),
                                  min(sb[2], pb[2]), max(sb[3], pb[3]),
                                  min(sb[4], pb[4]), max(sb[5], pb[5]))
                if self._last_grid_bounds is None or any(abs(a - b) > 1e-9 for a, b in zip(self._last_grid_bounds, merged)):
                    self._add_cube_axes_around_bounds(bounds=merged)
            except Exception:
                _LOG.exception("build_overlays: extend grid to path failed")

    # ===================== 3) toggle_overlays =====================

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

    # ===================== View Update =====================

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
