# app/tabs/recipe/coating_preview_panel/coating_preview_panel.py
# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
import inspect
from typing import Optional, Any, Dict, Tuple, List

import numpy as np
import pyvista as pv
import math
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSizePolicy, QSpacerItem
from PyQt6.sip import isdeleted

from model.recipe.recipe import Recipe, Draft 
from model.recipe.recipe_store import RecipeStore
from model.recipe.path_builder import PathBuilder

from widgets.info_groupbox import InfoGroupBox

from .views_3d.scene_manager import SceneManager, PreviewScene
from .views_2d.matplot2d import Matplot2DView
from .overlays_groupbox import OverlaysGroupBox
from .views_2d_box import Views2DBox
from .views_3d_box import Views3DBox

from .views_3d.raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("tabs.recipe.preview.panel")

Bounds = Tuple[float, float, float, float, float, float]

_ALLOWED_SIDES = ("top", "front", "back", "left", "right", "polyhelix", "helix")


def _set_policy(
    w: QWidget,
    *,
    h: QSizePolicy.Policy = QSizePolicy.Policy.Expanding,
    v: QSizePolicy.Policy = QSizePolicy.Policy.Preferred,
) -> None:
    sp = w.sizePolicy()
    sp.setHorizontalPolicy(h)
    sp.setVerticalPolicy(v)
    w.setSizePolicy(sp)


def _normalize(v: np.ndarray, eps: float = 1e-12) -> np.ndarray:
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v, axis=-1, keepdims=True)
    n = np.where(n < eps, 1.0, n)
    return v / n

def _limit_angle_vector(v_in: np.ndarray, max_deg: float, up_vec: np.ndarray) -> np.ndarray:
    """
    Limitiert die Neigung von v_in relativ zu up_vec auf max_deg.
    Dreht v_in in Richtung up_vec, falls der Winkel zu groß ist.
    """
    # Winkel berechnen
    dot = np.clip(np.dot(v_in, up_vec), -1.0, 1.0)
    angle = np.arccos(dot)
    max_rad = np.radians(float(max_deg))

    # Wenn innerhalb des Limits -> return original
    if angle <= max_rad:
        return v_in
    
    # Rotationsachse berechnen (Senkrecht zu Ebene aus v_in und up_vec)
    axis = np.cross(up_vec, v_in)
    n_axis = np.linalg.norm(axis)
    
    # Singularität: wenn v_in genau entgegengesetzt zu up_vec ist (180 grad)
    if n_axis < 1e-6:
        # Beliebige Achse senkrecht zu up_vec nehmen
        if abs(up_vec[0]) < 0.9: axis = np.array([1.0, 0.0, 0.0])
        else: axis = np.array([0.0, 1.0, 0.0])
    else:
        axis = axis / n_axis

    # Neuer Vektor: Rotiere Up-Vector um 'axis' mit 'max_rad'
    # Rodrigues Rotation: v_rot = v*cos + (k x v)*sin
    c = np.cos(max_rad)
    s = np.sin(max_rad)
    
    # up_vec um axis rotieren
    v_out = up_vec * c + np.cross(axis, up_vec) * s
    return _normalize(v_out)

def _quats_from_basis(X: np.ndarray, Y: np.ndarray, Z: np.ndarray) -> np.ndarray:
    """
    Konvertiert Nx3 Basis-Vektoren (Spalten einer Rotationsmatrix) in Nx4 Quaternionen (qx, qy, qz, qw).
    X (Tangente/Side), Y (Binormale), Z (Normale) müssen normiert sein.
    """
    N = len(X)
    R = np.stack((X, Y, Z), axis=2) # Shape (N, 3, 3)

    trace = R[:, 0, 0] + R[:, 1, 1] + R[:, 2, 2]
    Q = np.zeros((N, 4), dtype=float)

    mask_pos = trace > 0
    if np.any(mask_pos):
        idx = mask_pos
        s = np.sqrt(trace[idx] + 1.0) * 2.0
        inv_s = 1.0 / s
        Q[idx, 3] = 0.25 * s
        Q[idx, 0] = (R[idx, 2, 1] - R[idx, 1, 2]) * inv_s
        Q[idx, 1] = (R[idx, 0, 2] - R[idx, 2, 0]) * inv_s
        Q[idx, 2] = (R[idx, 1, 0] - R[idx, 0, 1]) * inv_s

    mask_neg = ~mask_pos
    if np.any(mask_neg):
        idx = np.where(mask_neg)[0]
        for i in idx:
            r = R[i]
            d0, d1, d2 = r[0,0], r[1,1], r[2,2]
            if d0 > d1 and d0 > d2:
                s = np.sqrt(1.0 + d0 - d1 - d2) * 2.0
                inv_s = 1.0 / s
                Q[i, 3] = (r[2, 1] - r[1, 2]) * inv_s
                Q[i, 0] = 0.25 * s
                Q[i, 1] = (r[0, 1] + r[1, 0]) * inv_s
                Q[i, 2] = (r[0, 2] + r[2, 0]) * inv_s
            elif d1 > d2:
                s = np.sqrt(1.0 + d1 - d0 - d2) * 2.0
                inv_s = 1.0 / s
                Q[i, 3] = (r[0, 2] - r[2, 0]) * inv_s
                Q[i, 0] = (r[0, 1] + r[1, 0]) * inv_s
                Q[i, 1] = 0.25 * s
                Q[i, 2] = (r[1, 2] + r[2, 1]) * inv_s
            else:
                s = np.sqrt(1.0 + d2 - d0 - d1) * 2.0
                inv_s = 1.0 / s
                Q[i, 3] = (r[1, 0] - r[0, 1]) * inv_s
                Q[i, 0] = (r[0, 2] + r[2, 0]) * inv_s
                Q[i, 1] = (r[1, 2] + r[2, 1]) * inv_s
                Q[i, 2] = 0.25 * s
    return Q


def _polydata_from_segments(A: np.ndarray, B: np.ndarray) -> pv.PolyData:
    A = np.asarray(A, dtype=float).reshape(-1, 3)
    B = np.asarray(B, dtype=float).reshape(-1, 3)
    M = min(len(A), len(B))
    if M <= 0:
        return pv.PolyData()
    pts = np.vstack([A[:M], B[:M]])
    lines = np.empty((M, 3), dtype=np.int64)
    lines[:, 0] = 2
    lines[:, 1] = np.arange(0, M, dtype=np.int64)
    lines[:, 2] = np.arange(M, 2 * M, dtype=np.int64)
    pd = pv.PolyData()
    pd.points = pts
    pd.lines = lines.reshape(-1)
    return pd


def _bounds_center(bounds: Bounds) -> np.ndarray:
    xmin, xmax, ymin, ymax, zmin, zmax = map(float, bounds)
    return np.array([0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax)], dtype=float)


def _side_cfg(side: str):
    s = str(side or "").lower()
    cfgs = {
        "top": {"anchor": ("z", "max"), "axes": ("x", "y")},
        "front": {"anchor": ("y", "min"), "axes": ("x", "z")},
        "back": {"anchor": ("y", "max"), "axes": ("x", "z")},
        "left": {"anchor": ("x", "min"), "axes": ("y", "z")},
        "right": {"anchor": ("x", "max"), "axes": ("y", "z")},
    }
    return cfgs.get(s)


def _embed_path_on_face(P_local: np.ndarray, side: str, bounds: Bounds) -> np.ndarray:
    cfg = _side_cfg(side)
    if cfg is None:
        raise KeyError(f"Unknown side_id {side!r}")

    ax0, ax1 = cfg["axes"]
    anchor_axis, anchor_side = cfg["anchor"]

    xmin, xmax, ymin, ymax, zmin, zmax = map(float, bounds)
    c = _bounds_center(bounds)
    cx, cy, cz = float(c[0]), float(c[1]), float(c[2])

    anchor_val = {
        "x": (xmin if anchor_side == "min" else xmax),
        "y": (ymin if anchor_side == "min" else ymax),
        "z": (zmin if anchor_side == "min" else zmax),
    }[anchor_axis]

    P0 = np.asarray(P_local, float).reshape(-1, 3)
    out = np.empty((P0.shape[0], 3), dtype=float)

    def place(axis_name: str, arr: np.ndarray):
        if axis_name == "x":
            return cx + arr
        if axis_name == "y":
            return cy + arr
        if axis_name == "z":
            return cz + arr
        return arr

    a0 = P0[:, 0]
    a1 = P0[:, 1]

    axes: Dict[str, Optional[np.ndarray]] = {"x": None, "y": None, "z": None}
    axes[ax0] = place(ax0, a0)
    axes[ax1] = place(ax1, a1)
    axes[anchor_axis] = np.full(P0.shape[0], anchor_val, dtype=float)

    out[:, 0] = axes["x"]
    out[:, 1] = axes["y"]
    out[:, 2] = axes["z"]
    return out


def _embed_3d_world_centered(P_local: np.ndarray, bounds: Bounds) -> np.ndarray:
    P0 = np.asarray(P_local, float).reshape(-1, 3)
    c = _bounds_center(bounds)
    return P0 + c[None, :]


def _call_cast_rays_for_side_robust(*args, **kwargs):
    ret = cast_rays_for_side(*args, **kwargs)
    if not isinstance(ret, tuple) or len(ret) < 2:
        raise RuntimeError("cast_rays_for_side returned invalid result")

    rc = ret[0]
    hit_poly = pv.PolyData()
    miss_poly = pv.PolyData()
    tcp_poly = pv.PolyData()

    polys = [x for x in ret[1:] if isinstance(x, pv.PolyData)]

    if len(polys) == 0:
        return rc, hit_poly, miss_poly, tcp_poly
    if len(polys) == 1:
        hit_poly = polys[0]
        return rc, hit_poly, miss_poly, tcp_poly
    if len(polys) == 2:
        hit_poly = polys[0]
        tcp_poly = polys[1]
        return rc, hit_poly, miss_poly, tcp_poly

    hit_poly = polys[0]
    miss_poly = polys[1]
    tcp_poly = polys[2]
    return rc, hit_poly, miss_poly, tcp_poly


def _is_empty_poly(mesh) -> bool:
    try:
        if mesh is None:
            return True
        if isinstance(mesh, pv.DataSet):
            return int(getattr(mesh, "n_points", 0)) <= 0
    except Exception:
        return True
    return False


def _substrate_origin_world(scene: PreviewScene, substrate_mesh: pv.DataSet) -> np.ndarray:
    mount = None
    for attr in ("mount_mesh", "substrate_mount_mesh", "mount", "substrate_mount"):
        m = getattr(scene, attr, None)
        if isinstance(m, pv.DataSet):
            mount = m
            break

    if isinstance(mount, pv.DataSet):
        b = mount.bounds
        cx = 0.5 * (float(b[0]) + float(b[1]))
        cy = 0.5 * (float(b[2]) + float(b[3]))
        z_top = float(b[5])
        return np.array([cx, cy, z_top], dtype=float)

    sb = substrate_mesh.bounds
    cx = 0.5 * (float(sb[0]) + float(sb[1]))
    cy = 0.5 * (float(sb[2]) + float(sb[3]))
    z0 = float(sb[4])
    return np.array([cx, cy, z0], dtype=float)


class CoatingPreviewPanel(QWidget):
    interactorReady = pyqtSignal()

    def __init__(self, *, ctx, store: RecipeStore, parent: Optional[QWidget] = None):
        super().__init__(parent)
        self.ctx = ctx

        if store is None or not isinstance(store, RecipeStore):
            raise RuntimeError("CoatingPreviewPanel: store fehlt/ungültig (RecipeTab muss store übergeben).")
        self.store: RecipeStore = store

        self._dying: bool = False
        self.destroyed.connect(lambda *_: setattr(self, "_dying", True))

        self._interactor: Any = None
        self._retry_left: int = 10
        self._retry_delay_ms: int = 50

        self._vis: Dict[str, bool] = {
            "path": True,
            "mask": True,
            "hits": True,
            "misses": True,
            "normals": True,
            "frames": True,
        }

        self._logged_pathbuilder_origin: bool = False

        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        self.grpInfo = InfoGroupBox(self)
        _set_policy(self.grpInfo, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        root.addWidget(self.grpInfo, 0)

        split = QHBoxLayout()
        split.setContentsMargins(0, 0, 0, 0)
        split.setSpacing(8)
        root.addLayout(split, 1)

        vleft = QVBoxLayout()
        vleft.setContentsMargins(0, 0, 0, 0)
        vleft.setSpacing(6)
        split.addLayout(vleft, 1)

        self._mat2d = Matplot2DView(parent=self)
        _set_policy(self._mat2d, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)

        try:
            tb = self._mat2d.make_toolbar(self)
            if tb is not None:
                self._mat2d_toolbar = tb
                vleft.addWidget(tb, 0)
        except Exception:
            _LOG.exception("Matplot2D toolbar creation failed")

        vleft.addWidget(self._mat2d, 1)
        vleft.addSpacerItem(QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding))
        self.destroyed.connect(lambda *_: getattr(self._mat2d, "dispose", lambda: None)())

        self.views2d = Views2DBox(
            switch_2d=self._switch_2d_plane,
            refresh_callable=self._mat2d.refresh,
            get_bounds=self.get_bounds,
            set_bounds=self.set_bounds,
            parent=self,
        )
        _set_policy(self.views2d, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        vleft.insertWidget(0, self.views2d, 0)

        vright = QVBoxLayout()
        vright.setContentsMargins(0, 0, 0, 0)
        vright.setSpacing(6)
        split.addLayout(vright, 1)

        self.scene = SceneManager(interactor_getter=lambda: self._interactor)

        self.views3d = Views3DBox(
            interactor_getter=lambda: self._interactor,
            render_callable=self.render,
            bounds_getter=self.get_bounds,
            substrate_bounds_getter=lambda: self.scene.get_layer_bounds("substrate"),
            cam_pad=1.0,
            iso_extra_zoom=3.0,
            parent=self,
        )
        _set_policy(self.views3d, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        vright.addWidget(self.views3d, 0)

        self.grpOverlays = OverlaysGroupBox(
            self,
            add_mesh_fn=self.add_mesh,
            clear_layer_fn=self.clear_layer,
            add_path_polyline_fn=self.add_path_polyline,
            show_poly_fn=self.show_poly,
            show_frames_at_fn=self.show_frames_at,
            set_layer_visible_fn=lambda layer, vis, render=True: self.scene.set_layer_visible(layer, vis),
            update_2d_scene_fn=lambda mesh, path_xyz, _mask_poly: self.update_2d_scene(
                substrate_mesh=mesh,
                path_xyz=path_xyz,
            ),
            layers={
                "ground": "ground",
                "mount": "mount",
                "substrate": "substrate",
                "path": "path",
                "path_mrk": "path_markers",
                "mask": "mask",
                "mask_mrk": "mask_markers",
                "rays_hit": "rays_hit",
                "rays_miss": "rays_miss",
                "normals": "normals",
                "frames_x": "frames_x",
                "frames_y": "frames_y",
                "frames_z": "frames_z",
                "planned": "planned",
                "executed": "executed",
            },
            get_bounds=self.get_bounds,
        )
        _set_policy(self.grpOverlays, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Preferred)
        vright.addWidget(self.grpOverlays, 0)

        self._pvHost = QWidget(self)
        self._pvHost.setObjectName("pvHost")
        _set_policy(self._pvHost, h=QSizePolicy.Policy.Expanding, v=QSizePolicy.Policy.Expanding)
        self._pvHostLayout = QVBoxLayout(self._pvHost)
        self._pvHostLayout.setContentsMargins(0, 0, 0, 0)
        self._pvHostLayout.setSpacing(0)
        vright.addWidget(self._pvHost, 1)

        self._bounds: Bounds = (-120.0, 120.0, -120.0, 120.0, 0.0, 240.0)
        QTimer.singleShot(0, self._push_initial_visibility)

    def get_pv_host(self) -> QWidget:
        return self._pvHost

    def set_interactor(self, ia) -> None:
        self._interactor = ia
        try:
            if self._interactor is not None and hasattr(self._interactor, "render"):
                self._interactor.render()
        except Exception:
            _LOG.exception("set_interactor: initial render failed")
        self.interactorReady.emit()

    def update_preview(self, model: Recipe) -> None:
        self.handle_update_preview(model)

    def handle_update_preview(self, model: Recipe) -> None:
        if self._dying or isdeleted(self):
            return

        ia = self._interactor
        if ia is None:
            if self._retry_left > 0:
                self._retry_left -= 1
                QTimer.singleShot(self._retry_delay_ms, lambda m=model: self.handle_update_preview(m))
            else:
                self._set_info_defaults()
            return
        self._retry_left = 10

        if not self._logged_pathbuilder_origin:
            self._logged_pathbuilder_origin = True
            try:
                _LOG.info(
                    "PathBuilder loaded from module=%s file=%s",
                    getattr(PathBuilder, "__module__", "?"),
                    inspect.getsourcefile(PathBuilder),
                )
            except Exception:
                _LOG.exception("Could not log PathBuilder origin")

        cam_snap = self.snapshot_camera()

        try:
            scene: PreviewScene = self.scene.build_scene(self.ctx, model, grid_step_mm=10.0)
        except Exception:
            _LOG.exception("build_scene failed")
            self._set_info_defaults()
            return

        self.set_bounds(scene.bounds)

        try:
            vis = self.grpOverlays.current_visibility()
        except Exception:
            vis = dict(self._vis)

        substrate_mesh = scene.substrate_mesh
        if substrate_mesh is None:
            self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)
            return

        # Params merge
        defaults = self.store.collect_global_defaults()
        globals_params = dict(defaults)
        globals_params.update(getattr(model, "globals_params", {}) or {})
        recipe_params = getattr(model, "parameters", {}) or {}
        globals_params.update(recipe_params)

        sample_step_mm = float(globals_params.get("sample_step_mm", 1.0))
        max_points = int(globals_params.get("max_points", 500))
        stand_off_mm = float(globals_params.get("stand_off_mm", 10.0))
        
        # ✅ MAX ANGLE für Z-Achse (default 45 Grad)
        max_angle_deg = float(globals_params.get("max_angle_deg", 45.0))

        MASK_OFFSET_MM = 50.0

        pbs = getattr(model, "paths_by_side", {}) or {}
        sides = [str(k).lower() for k in pbs.keys() if str(k).lower() in _ALLOWED_SIDES]

        built = PathBuilder.from_recipe_paths(
            recipe=model,
            sides=sides,
            globals_params=globals_params,
            sample_step_mm=sample_step_mm,
            max_points=max_points,
        )

        for lyr in (
            "path", "path_markers",
            "mask", "mask_markers",
            "rays_hit", "rays_miss",
            "normals",
            "frames_x", "frames_y", "frames_z",
            "planned", "executed"
        ):
            self.scene.clear_layer(lyr)

        planned_tcp = getattr(model, "planned_tcp", None)
        executed_tcp = getattr(model, "executed_tcp", None)

        self.grpOverlays.overlays.render_compiled(
            substrate_mesh=substrate_mesh,
            compiled=getattr(model, "paths_compiled", None),
            planned_tcp=planned_tcp,
            executed_tcp=executed_tcp,
            visibility=vis,
            mask_lift_mm=50.0,
            default_stand_off_mm=stand_off_mm,
            sides=pbs
        )

        tcp_all_world: List[np.ndarray] = []
        origin_world = _substrate_origin_world(scene, substrate_mesh)

        for side_id, pdata in built:
            side = str(side_id).lower()
            if side not in _ALLOWED_SIDES:
                continue

            pts = getattr(pdata, "points_mm", None)
            if pts is None:
                pts = getattr(pdata, "points", None)
            if pts is None:
                continue

            P_local = np.asarray(pts, dtype=float).reshape(-1, 3)
            if P_local.size == 0:
                continue

            source = ""
            try:
                source = str((pdata.meta or {}).get("source", ""))
            except Exception:
                source = ""

            if side in ("helix", "polyhelix"):
                P_start = _embed_3d_world_centered(P_local, substrate_mesh.bounds)
                if side == "helix" and (not source):
                    source = "helix"
                side_for_rays = "top"
            else:
                P_start = _embed_path_on_face(P_local, side, substrate_mesh.bounds)
                side_for_rays = side

            try:
                rc, rays_hit_poly, rays_miss_poly, _tcp_poly = _call_cast_rays_for_side_robust(
                    P_start,
                    sub_mesh_world=substrate_mesh,
                    side=side_for_rays,
                    source=source,
                    stand_off_mm=stand_off_mm,
                    ray_len_mm=2000.0,
                    start_lift_mm=max(5.0, stand_off_mm + 5.0),
                    invert_dirs=False,
                    lock_xy=True,
                )
            except Exception:
                _LOG.exception("cast_rays_for_side failed for side=%s (source=%s)", side, source)
                continue

            valid = np.asarray(rc.valid, bool).reshape(-1)
            if not np.any(valid):
                continue

            hit = np.asarray(rc.hit_mm, float).reshape(-1, 3)[valid]
            
            # --- 1. Z-Achse (Normale) mit Max-Angle-Limit ---
            raw_nrm = _normalize(np.asarray(rc.normal, float).reshape(-1, 3)[valid])
            Z = np.zeros_like(raw_nrm)
            
            # LimitLogic: Winkel gegen Welt-Z begrenzen
            up_vec = np.array([0.0, 0.0, 1.0]) 
            for i in range(len(raw_nrm)):
                Z[i] = _limit_angle_vector(raw_nrm[i], max_angle_deg, up_vec)

            tcp_world = np.asarray(rc.tcp_mm, float).reshape(-1, 3)[valid]
            tcp_all_world.append(tcp_world)

            mask_world = tcp_world + Z * float(MASK_OFFSET_MM)

            # --- 2. X-Achse (Fixed Orientation / World-Locked) ---
            # Wir wählen eine Referenz-Richtung je nach Seite
            if side in ("left", "right"):
                base_ref = np.array([0.0, 1.0, 0.0], float) # Y für Left/Right
            elif "helix" in side or "cylinder" in side:
                base_ref = np.array([0.0, 0.0, 1.0], float) # Z für Cylinder
            else:
                base_ref = np.array([1.0, 0.0, 0.0], float) # X für Top/Front/Back

            N_pts = len(tcp_world)
            ref_X = np.tile(base_ref, (N_pts, 1))
            
            # Projektion auf Ebene senkrecht zu Z: X = ref_X - (ref_X . Z) * Z
            dot = np.sum(ref_X * Z, axis=1, keepdims=True)
            X = ref_X - Z * dot
            X = _normalize(X)

            # Fallback bei Singularität
            bad = np.linalg.norm(X, axis=1) < 1e-6
            if np.any(bad):
                if base_ref[0] > 0.9: 
                    alt_ref = np.array([0.0, 1.0, 0.0], float)
                else: 
                    alt_ref = np.array([1.0, 0.0, 0.0], float)
                fallback = np.tile(alt_ref, (len(X), 1))
                X[bad] = _normalize(fallback - Z[bad] * np.sum(fallback * Z[bad], axis=1, keepdims=True))

            # --- 3. Y-Achse ---
            Y = _normalize(np.cross(Z, X))

            # Vis (Path/Mask/Lines)
            if vis.get("path", True):
                self.scene.add_path_polyline(tcp_world, layer="path", color="#32CD32", line_width=2.2, lighting=False)

            if vis.get("mask", True):
                self.scene.add_path_polyline(mask_world, layer="mask", color="royalblue", line_width=3.0, lighting=False)

            if vis.get("hits", True) and (not _is_empty_poly(rays_hit_poly)):
                self.scene.add_mesh(rays_hit_poly, layer="rays_hit", color="#3498db", line_width=1.2, lighting=False)

            if vis.get("misses", True) and (not _is_empty_poly(rays_miss_poly)):
                self.scene.add_mesh(rays_miss_poly, layer="rays_miss", color="#e74c3c", line_width=1.2, lighting=False)

            if vis.get("normals", False):
                # Zeichne korrigierte Normalen (Z)
                st_corr = _polydata_from_segments(hit, hit + Z * 20.0) 
                if not _is_empty_poly(st_corr):
                    self.scene.add_mesh(st_corr, layer="normals", color="#f1c40f", line_width=1.5, lighting=False)

            # Frames anzeigen (gedünnt)
            if vis.get("frames", False):
                n_pts = len(tcp_world)
                sstep = max(1, int(round(max(1, n_pts // 60))))
                O2 = tcp_world[::sstep]
                X2 = X[::sstep]
                Y2 = Y[::sstep]
                Z2 = Z[::sstep]
                sx = 12.0
                mx = _polydata_from_segments(O2, O2 + X2 * sx)
                my = _polydata_from_segments(O2, O2 + Y2 * sx)
                mz = _polydata_from_segments(O2, O2 + Z2 * sx)
                if not _is_empty_poly(mx): self.scene.add_mesh(mx, layer="frames_x", color="#e67e22", line_width=1.0, lighting=False)
                if not _is_empty_poly(my): self.scene.add_mesh(my, layer="frames_y", color="#16a085", line_width=1.0, lighting=False)
                if not _is_empty_poly(mz): self.scene.add_mesh(mz, layer="frames_z", color="#2980b9", line_width=1.0, lighting=False)

            # ------------------------------------------------------------------
            # 💾 Persist final TCP points & orientations into model.draft (Draft)
            try:
                tcp_local = tcp_world - origin_world[None, :]

                # Konvertiere Basisvektoren (Rotationsmatrix) in Quaternionen
                quats = _quats_from_basis(X, Y, Z)

                poses_quat = []
                for i in range(len(tcp_local)):
                    poses_quat.append({
                        "x": float(tcp_local[i, 0]),
                        "y": float(tcp_local[i, 1]),
                        "z": float(tcp_local[i, 2]),
                        "qx": float(quats[i, 0]),
                        "qy": float(quats[i, 1]),
                        "qz": float(quats[i, 2]),
                        "qw": float(quats[i, 3]),
                    })

                normals_xyz = [{"x": float(nx), "y": float(ny), "z": float(nz)} for (nx, ny, nz) in Z.reshape(-1, 3)]

                base: Dict[str, Any]
                if isinstance(getattr(model, "draft", None), Draft):
                    try:
                        base = dict(model.draft.to_yaml_dict() or {}) 
                    except Exception:
                        base = {}
                else:
                    base = {}

                base.setdefault("version", 1)
                base.setdefault("sides", {})

                if not isinstance(base.get("sides"), dict):
                    base["sides"] = {}

                base["sides"][str(side)] = {
                    "poses_quat": poses_quat,
                    "normals_xyz": normals_xyz,
                }

                model.draft = Draft.from_yaml_dict(base)

            except Exception:
                _LOG.exception("Persisting model.draft failed")

        path_xyz = None
        if tcp_all_world:
            try:
                path_xyz = np.vstack(tcp_all_world)
            except Exception:
                path_xyz = None

        try:
            self.update_2d_scene(substrate_mesh=scene.substrate_mesh, path_xyz=path_xyz)
        except Exception:
            _LOG.exception("update_2d_scene failed")

        try:
            info = {
                "points": int(0 if path_xyz is None else len(path_xyz)),
                "tris": scene.mesh_tris,
                "bounds": scene.bounds,
            }
            if planned_tcp: info["Planned"] = "Available"
            if executed_tcp: info["Executed"] = "Available"
            self.grpInfo.set_values(info)
        except Exception:
            _LOG.exception("InfoGroupBox update failed")

        try:
            if cam_snap:
                self.restore_camera(cam_snap)
        except Exception:
            _LOG.exception("restore_camera failed")

        self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)

    # -------- Helpers --------

    def _set_info_defaults(self) -> None:
        try:
            if not self._dying and self.grpInfo is not None and not isdeleted(self.grpInfo):
                self.grpInfo.set_values(None)
        except Exception:
            pass

    def _push_initial_visibility(self) -> None:
        try:
            self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)
        except Exception:
            _LOG.exception("initial visibility push failed")

    def render(self) -> None:
        ia = self._interactor
        try:
            if ia is not None and hasattr(ia, "render"):
                ia.render()
        except Exception:
            _LOG.exception("render failed")

    def snapshot_camera(self) -> Optional[Dict[str, Any]]:
        ia = self._interactor
        if ia is None or not hasattr(ia, "camera"):
            return None
        cam = ia.camera
        try:
            return {
                "position": tuple(cam.position),
                "focal_point": tuple(cam.focal_point),
                "view_up": tuple(cam.up),
                "clipping_range": tuple(cam.clipping_range),
            }
        except Exception:
            return None

    def restore_camera(self, snap: Optional[Dict[str, Any]]) -> None:
        if not snap:
            return
        ia = self._interactor
        if ia is None or not hasattr(ia, "camera"):
            return
        try:
            cam = ia.camera
            cam.position = snap.get("position", cam.position)
            cam.focal_point = snap.get("focal_point", cam.focal_point)
            cam.up = snap.get("view_up", cam.up)
            cr = snap.get("clipping_range")
            if cr:
                cam.clipping_range = cr
        except Exception:
            _LOG.exception("restore_camera failed")

    def set_bounds(self, bounds: Bounds) -> None:
        self._bounds = tuple(map(float, bounds))  # type: ignore[assignment]
        try:
            self._mat2d.set_bounds(self._bounds)
        except Exception:
            _LOG.exception("set_bounds -> 2D failed")

    def get_bounds(self) -> Bounds:
        return self._bounds

    def _switch_2d_plane(self, plane: str) -> None:
        try:
            self._mat2d.set_plane(plane)
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("2D plane switch failed: %s", plane)

    def update_2d_scene(self, *, substrate_mesh=None, path_xyz: Optional[np.ndarray] = None) -> None:
        try:
            self._mat2d.set_scene(
                substrate_mesh=substrate_mesh,
                path_xyz=None if path_xyz is None else np.asarray(path_xyz, float).reshape(-1, 3),
                bounds=self._bounds,
            )
            self._mat2d.refresh()
        except Exception:
            _LOG.exception("update_2d_scene failed")

    # SceneManager passthroughs
    def clear(self) -> None:
        self.scene.clear()

    def add_mesh(self, mesh, **kwargs) -> None:
        self.scene.add_mesh(mesh, **kwargs)

    def clear_layer(self, layer: str) -> None:
        self.scene.clear_layer(layer)

    def add_path_polyline(self, *a, **kw):
        return self.scene.add_path_polyline(*a, **kw)

    def show_poly(
        self,
        poly,
        *,
        layer: str,
        color: str = "royalblue",
        line_width: float = 2.0,
        lighting: bool = False
    ) -> None:
        try:
            self.clear_layer(layer)
            self.add_mesh(poly, color=color, line_width=float(line_width), lighting=lighting, layer=layer)
        except Exception:
            _LOG.exception("show_poly() failed")

    def show_frames_at(self, **_kwargs) -> None:
        return