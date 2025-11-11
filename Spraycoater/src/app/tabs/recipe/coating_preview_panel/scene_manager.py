# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Any, Tuple

import numpy as np
import pyvista as pv

from .mesh_utils import (
    load_mount_mesh_from_key,
    load_substrate_mesh_from_key,
    place_substrate_on_mount,
)
from .interactor_host import InteractorHost

_LOG = logging.getLogger("app.tabs.recipe.scene")


@dataclass
class PreviewScene:
    bounds: Tuple[float, float, float, float, float, float]
    center: Tuple[float, float, float]
    ground_z: float
    ground_mesh: pv.PolyData
    mount_mesh: Optional[pv.PolyData]
    substrate_mesh: Optional[pv.PolyData]
    mesh_tris: Optional[int]


class SceneManager:
    """
    Dünne Schicht über dem PyVista-Interactor mit Layer-Verwaltung + Helpers.
    Keine UI-/Panel-Aufrufe. Orchestrierung geschieht im Panel.
    """

    def __init__(
        self,
        *,
        parent_widget: Any,
        container_widget: Any,
        init_scene_builder: Callable[[], None] | None = None,
    ):
        self._host = InteractorHost(parent_widget, container_widget)
        self._init_scene_builder = init_scene_builder
        self._layers: Dict[str, List[Any]] = {}

    # --- Interactor ---------------------------------------------------------
    def ensure_interactor(self) -> bool:
        try:
            ok = bool(self._host.ensure())
            if not ok or getattr(self._host, "ia", None) is None:
                _LOG.warning("SceneManager.ensure_interactor(): Interactor not available")
                return False
            return True
        except Exception:
            _LOG.exception("SceneManager.ensure_interactor() failed")
            return False

    @property
    def ia(self):
        return getattr(self._host, "ia", None)

    def _ia(self):
        ia = self.ia
        if ia is None:
            _LOG.warning("SceneManager: Interactor not available")
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

    def clear_layer(self, layer: str, *, render: bool = False) -> None:
        self._remove_layer_actors(layer)
        if render:
            ia = self._ia()
            if ia is not None:
                try:
                    ia.render()
                except Exception:
                    pass

    def clear(self, *, render: bool = False) -> None:
        ia = self._ia()
        if ia is None:
            return
        try:
            for layer in list(self._layers.keys()):
                self._remove_layer_actors(layer)
            if render:
                ia.render()
        except Exception:
            _LOG.exception("clear failed")

    def set_layer_visible(self, layer: str, visible: bool, *, render: bool = True) -> None:
        ia = self._ia()
        if ia is None:
            return
        actors = self._layers.get(layer, [])
        for a in actors:
            try:
                a.SetVisibility(1 if visible else 0)
            except Exception:
                pass
        if render:
            try:
                ia.render()
            except Exception:
                pass

    def get_layer_bounds(
        self, layer: str
    ) -> Optional[Tuple[float, float, float, float, float, float]]:
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
    def add_mesh(self, mesh, *, layer: str = "default", render: bool = False,
                 reset_camera: bool = False, **kwargs) -> Optional[Any]:
        ia = self._ia()
        if ia is None or mesh is None:
            return None
        try:
            actor = ia.add_mesh(mesh, **kwargs)
            self._ensure_layer(layer).append(actor)
            if reset_camera:
                b = getattr(mesh, "bounds", None)
                try:
                    ia.reset_camera(bounds=b if b is not None else None)
                except Exception:
                    _LOG.exception("reset_camera failed in add_mesh")
            if render:
                try:
                    ia.render()
                except Exception:
                    _LOG.exception("ia.render failed in add_mesh")
            return actor
        except Exception:
            _LOG.exception("add_mesh failed")
            return None

    def add_path_polyline(
        self,
        points_mm: np.ndarray,
        *,
        layer: str = "path",
        color: str = "#2ecc71",
        line_width: float = 2.0,
        render: bool = False,
        reset_camera: bool = False,
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
                        tube,
                        layer=layer,
                        color=color,
                        render=render,
                        reset_camera=reset_camera,
                        lighting=lighting,
                    )
                except Exception:
                    _LOG.exception("tube generation from polyline failed; fallback to line")
            return self.add_mesh(
                poly,
                layer=layer,
                color=color,
                line_width=float(line_width),
                render=render,
                reset_camera=reset_camera,
                lighting=lighting,
            )
        except Exception:
            _LOG.exception("add_path_polyline failed")
            return None

    # --- Floor ---------------------------------------------------------------
    def add_floor_grid(self, bounds, *, step: float = 10.0,
                       layer: str = "floor_grid", color: str = "#cfcfcf"):
        ia = self._ia()
        if ia is None:
            return None
        xmin, xmax, ymin, ymax, zmin, _ = bounds
        width, height = xmax - xmin, ymax - ymin
        i_res = max(1, int(round(width / max(1e-6, step))))
        j_res = max(1, int(round(height / max(1e-6, step))))
        center = ((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, zmin)
        plane = pv.Plane(
            center=center,
            direction=(0, 0, 1),
            i_size=width,
            j_size=height,
            i_resolution=i_res,
            j_resolution=j_res,
        )
        return self.add_mesh(
            plane, style="wireframe", color=color, line_width=1.0,
            lighting=False, render=False, layer=layer,
        )

    def add_floor_axes(self, bounds, *, step: float = 10.0,
                       layer_lines: str = "floor_axes", layer_labels: str = "floor_labels"):
        ia = self._ia()
        if ia is None:
            return None

        xmin, xmax, ymin, ymax, zmin, _ = bounds
        step = max(1.0, float(step))

        def ticks(vmin, vmax):
            start = np.ceil(vmin / step) * step
            vals = np.arange(start, vmax + 0.5 * step, step, dtype=float)
            return vals[(vals >= vmin + 1e-6) & (vals <= vmax - 1e-6)]

        xt, yt = ticks(xmin, xmax), ticks(ymin, ymax)
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)

        pts, lines = [], []
        base = 0

        def add_seg(p0, p1):
            nonlocal base
            pts.extend([p0, p1])
            lines.extend([2, base, base + 1])
            base += 2

        add_seg((xmin, cy, zmin), (xmax, cy, zmin))
        add_seg((cx, ymin, zmin), (cx, ymax, zmin))

        tick_len = max(1.0, min(3.0, step * 0.15))
        for xv in xt:
            add_seg((xv, cy - tick_len, zmin), (xv, cy + tick_len, zmin))
        for yv in yt:
            add_seg((cx - tick_len, yv, zmin), (cx + tick_len, yv, zmin))

        self.clear_layer(layer_lines)
        actor = None
        if pts:
            poly = pv.PolyData(np.asarray(pts, dtype=float))
            poly.lines = np.asarray(lines, dtype=np.int64)
            actor = self.add_mesh(
                poly, style="wireframe", color="#5a5a5a", line_width=1.0,
                lighting=False, render=False, layer=layer_lines,
            )

        self.clear_layer(layer_labels)
        try:
            ia.add_point_labels(
                np.c_[xt, np.full_like(xt, cy), np.full_like(xt, zmin + tick_len * 0.5)],
                [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in xt],
                point_size=0, font_size=10, text_color="black", shape_opacity=0.0, render=False,
            )
            ia.add_point_labels(
                np.c_[np.full_like(yt, cx), yt, np.full_like(yt, zmin + tick_len * 0.5)],
                [f"{int(v)}" if abs(v - int(v)) < 1e-6 else f"{v:.1f}" for v in yt],
                point_size=0, font_size=10, text_color="black", shape_opacity=0.0, render=False,
            )
            ia.add_point_labels(
                [(xmax, cy, zmin + tick_len * 0.5), (cx, ymax, zmin + tick_len * 0.5)],
                ["X (mm)", "Y (mm)"],
                point_size=0, font_size=12, text_color="black", shape_opacity=0.0, render=False,
            )
        except Exception:
            _LOG.exception("floor labels failed")
        return actor

    def refresh_floor(self, bounds, *, step: float = 10.0) -> None:
        self.clear_layer("floor_grid")
        self.clear_layer("floor_axes")
        self.clear_layer("floor_labels")
        self.add_floor_grid(bounds, step=step, layer="floor_grid")
        self.add_floor_axes(bounds, step=step, layer_lines="floor_axes", layer_labels="floor_labels")
        ia = self._ia()
        try:
            if ia is not None:
                ia.reset_camera(bounds=bounds)
                ia.render()
        except Exception:
            _LOG.exception("refresh_floor: reset/render failed")

    # --- Build & Draw Scene --------------------------------------------------
    @staticmethod
    def _get_optional_str(model: object, key: str) -> Optional[str]:
        val = getattr(model, key, None) if hasattr(model, key) else (model.get(key) if isinstance(model, dict) else None)
        return val.strip() if isinstance(val, str) and val.strip() else None

    def build_scene(self, ctx, model: object) -> PreviewScene:
        # Felder weich lesen – keine Exceptions wenn noch nicht gesetzt
        mount_key = self._get_optional_str(model, "substrate_mount")
        substrate_key = self._get_optional_str(model, "substrate")

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

        # sichere Default-Bounds, wenn noch nichts geladen ist
        if smesh is not None:
            bounds = smesh.bounds
        elif mmesh is not None:
            bounds = mmesh.bounds
        else:
            bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)

        xmin, xmax, ymin, ymax, zmin, zmax = bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        cz = zmin

        ground = pv.Plane(
            center=(cx, cy, cz),
            direction=(0, 0, 1),
            i_size=500.0,
            j_size=500.0,
            i_resolution=1,
            j_resolution=1,
        )

        mesh_tris = None
        if smesh is not None:
            try:
                mesh_tris = int(smesh.n_faces) if hasattr(smesh, "n_faces") else None
            except Exception:
                mesh_tris = None

        return PreviewScene(
            bounds=bounds,
            center=(cx, cy, 0.5 * (zmin + zmax)),
            ground_z=cz,
            ground_mesh=ground,
            mount_mesh=mmesh,
            substrate_mesh=smesh,
            mesh_tris=mesh_tris,
        )

    def draw_scene(self, scene: PreviewScene, *, visibility: Dict[str, bool] | None = None) -> None:
        # Boden
        self.add_mesh(
            scene.ground_mesh,
            color="#3a3a3a",
            opacity=1.0,
            lighting=False,
            layer="ground",
            render=False,
            reset_camera=False,
        )
        # Mount
        if scene.mount_mesh is not None:
            self.add_mesh(
                scene.mount_mesh,
                color="#5d5d5d",
                opacity=0.95,
                lighting=False,
                layer="mount",
                render=False,
                reset_camera=False,
            )
        # Substrat
        if scene.substrate_mesh is not None:
            self.add_mesh(
                scene.substrate_mesh,
                color="#d0d6dd",
                opacity=1.0,
                lighting=False,
                layer="substrate",
                render=False,
                reset_camera=False,
            )
