# -*- coding: utf-8 -*-
from __future__ import annotations

import logging
from typing import Optional, Any, Dict, Tuple, List

import numpy as np
import pyvista as pv
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QSizePolicy, QSpacerItem
from PyQt6.sip import isdeleted

from model.recipe.recipe import Recipe
from model.recipe.recipe_store import RecipeStore
from model.recipe.path_builder import PathBuilder

from widgets.info_groupbox import InfoGroupBox

from .views_3d.scene_manager import SceneManager, PreviewScene
from .views_2d.matplot2d import Matplot2DView
from .overlays_groupbox import OverlaysGroupBox
from .views_2d_box import Views2DBox
from .views_3d_box import Views3DBox

# ✅ Passe den Importpfad ggf. an dein Projekt an
from .views_3d.raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("tabs.recipe.preview.panel")

Bounds = Tuple[float, float, float, float, float, float]
_ALLOWED_SIDES = ("top", "front", "back", "left", "right")


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


# ---------------------------------------------------------------------
# Side embedding (lokales Muster -> "Startpunkte" im World anhand Bounds)
# ---------------------------------------------------------------------
def _side_cfg(side: str):
    """
    Side-Konfiguration:
      - ray_dir: Basis-Ray-Richtung (falls cast_rays_for_side sie nutzen will)
      - anchor:  auf welcher Face-Ebene gestartet wird (x/y/z, min/max)
      - axes:    welche World-Achsen das lokale Muster (x,y) belegt
      - u_world/v_world: definiert das lokale Muster-KS (für Frames X/Y), NON-HELIX
    """
    s = str(side or "").lower()
    cfgs = {
        "top": {
            "ray_dir": np.array([0.0, 0.0, -1.0], float),
            "anchor": ("z", "max"),
            "axes": ("x", "y"),
            "u_world": np.array([1.0, 0.0, 0.0], float),  # Muster-X = +X
            "v_world": np.array([0.0, 1.0, 0.0], float),  # Muster-Y = +Y
        },
        "front": {
            "ray_dir": np.array([0.0, +1.0, 0.0], float),
            "anchor": ("y", "min"),
            "axes": ("x", "z"),
            "u_world": np.array([1.0, 0.0, 0.0], float),  # Muster-X = +X
            "v_world": np.array([0.0, 0.0, 1.0], float),  # Muster-Y = +Z
        },
        "back": {
            "ray_dir": np.array([0.0, -1.0, 0.0], float),
            "anchor": ("y", "max"),
            "axes": ("x", "z"),
            "u_world": np.array([1.0, 0.0, 0.0], float),  # Muster-X = +X
            "v_world": np.array([0.0, 0.0, 1.0], float),  # Muster-Y = +Z
        },
        "left": {
            "ray_dir": np.array([+1.0, 0.0, 0.0], float),
            "anchor": ("x", "min"),
            "axes": ("y", "z"),
            "u_world": np.array([0.0, 1.0, 0.0], float),  # Muster-X = +Y
            "v_world": np.array([0.0, 0.0, 1.0], float),  # Muster-Y = +Z
        },
        "right": {
            "ray_dir": np.array([-1.0, 0.0, 0.0], float),
            "anchor": ("x", "max"),
            "axes": ("y", "z"),
            "u_world": np.array([0.0, 1.0, 0.0], float),  # Muster-X = +Y
            "v_world": np.array([0.0, 0.0, 1.0], float),  # Muster-Y = +Z
        },
    }
    return cfgs.get(s)


def _embed_path_on_face(P_local: np.ndarray, side: str, bounds: Bounds) -> np.ndarray:
    cfg = _side_cfg(side)
    if cfg is None:
        raise KeyError(f"Unknown side_id {side!r}")

    ax0, ax1 = cfg["axes"]
    anchor_axis, anchor_side = cfg["anchor"]

    xmin, xmax, ymin, ymax, zmin, zmax = map(float, bounds)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)

    anchor_val = {
        "x": (xmin if anchor_side == "min" else xmax),
        "y": (ymin if anchor_side == "min" else ymax),
        "z": (zmin if anchor_side == "min" else zmax),
    }[anchor_axis]

    P0 = np.asarray(P_local, float).reshape(-1, 3)
    out = np.empty((P0.shape[0], 3), dtype=float)

    def place(axis_name: str, arr: np.ndarray):
        # lokales Muster um Bounds-Center herum zentrieren
        if axis_name == "x":
            return cx + arr
        if axis_name == "y":
            return cy + arr
        if axis_name == "z":
            return cz + arr
        return arr

    # Muster-2D: local.x, local.y
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


def _tangents_from_path(P: np.ndarray) -> np.ndarray:
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    n = P.shape[0]
    if n == 0:
        return np.zeros((0, 3), float)
    if n == 1:
        return np.array([[1.0, 0.0, 0.0]], float)

    T = np.empty((n, 3), float)
    T[0] = P[1] - P[0]
    T[-1] = P[-1] - P[-2]
    if n > 2:
        T[1:-1] = P[2:] - P[:-2]

    # normalize + stabilisieren
    for i in range(n):
        ln = float(np.linalg.norm(T[i]))
        if ln > 1e-12:
            T[i] = T[i] / ln
        else:
            T[i] = T[i - 1] if i else np.array([1.0, 0.0, 0.0], float)

    for i in range(1, n):
        if float(np.dot(T[i - 1], T[i])) < 0.0:
            T[i] = -T[i]

    return T


def _call_cast_rays_for_side_robust(*args, **kwargs):
    """
    Robust gegen unterschiedliche Rückgaben:
      - (rc, poly1, poly2)
      - (rc, poly1, poly2, poly3)
    Wir extrahieren:
      rc, hit_poly, miss_poly
    """
    ret = cast_rays_for_side(*args, **kwargs)
    if not isinstance(ret, tuple) or len(ret) < 1:
        raise RuntimeError("cast_rays_for_side returned invalid result")

    rc = ret[0]
    polys = [x for x in ret[1:] if isinstance(x, pv.PolyData)]

    hit_poly = polys[0] if len(polys) >= 1 else pv.PolyData()
    miss_poly = polys[1] if len(polys) >= 2 else pv.PolyData()
    return rc, hit_poly, miss_poly


class CoatingPreviewPanel(QWidget):
    """
    Layout:
      Info
      Split (HBox):
        - Left:  Views2D + Matplotlib
        - Right: Views3D-Buttons + Overlays + pvHost (PyVista interactor)
    """

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

        # Default-Visibility (OverlaysGroupBox liefert das üblicherweise)
        self._vis: Dict[str, bool] = {
            "path": True,
            "mask": True,
            "hits": True,
            "misses": True,
            "normals": True,
            "frames": True,
        }

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

        # ---------------- Left (2D) ----------------
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

        # ---------------- Right (3D) ----------------
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

    # -------- Public API --------

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

    # -------- Preview Pipeline --------

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

        cam_snap = self.snapshot_camera()

        # 1) Szene (Substrat/Mount/Ground)
        try:
            scene: PreviewScene = self.scene.build_scene(self.ctx, model, grid_step_mm=10.0)
        except Exception:
            _LOG.exception("build_scene failed")
            self._set_info_defaults()
            return

        self.set_bounds(scene.bounds)

        # Sichtbarkeit aus Groupbox (oder Default)
        try:
            vis = self.grpOverlays.current_visibility()
        except Exception:
            vis = dict(self._vis)

        substrate_mesh = scene.substrate_mesh
        if substrate_mesh is None:
            self.scene.update_current_views_once(refresh_2d=self._mat2d.refresh)
            return

        defaults = self.store.collect_global_defaults()
        globals_params = dict(defaults)
        globals_params.update(getattr(model, "globals_params", {}) or {})

        sample_step_mm = float(globals_params.get("sample_step_mm", 1.0))
        max_points = int(globals_params.get("max_points", 500))
        stand_off_mm = float(globals_params.get("stand_off_mm", 10.0))

        # ✅ Mask konstant 50 mm über TCP (entlang Normalen)
        MASK_OFFSET_MM = 50.0

        # sides: aus recipe.paths_by_side
        pbs = getattr(model, "paths_by_side", {}) or {}
        sides = [str(k).lower() for k in pbs.keys() if str(k).lower() in _ALLOWED_SIDES]

        built = PathBuilder.from_recipe_paths(
            recipe=model,
            sides=sides,
            globals_params=globals_params,
            sample_step_mm=sample_step_mm,
            max_points=max_points,
        )

        # Clear overlays
        for lyr in (
            "path", "path_markers",
            "mask", "mask_markers",
            "rays_hit", "rays_miss",
            "normals",
            "frames_x", "frames_y", "frames_z",
        ):
            self.scene.clear_layer(lyr)

        tcp_all: List[np.ndarray] = []

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

            # Startpunkte auf Side-Ebene
            P_start = _embed_path_on_face(P_local, side, substrate_mesh.bounds)

            # Raycast (robust gegen 3/4 returns)
            try:
                rc, rays_hit_poly, rays_miss_poly = _call_cast_rays_for_side_robust(
                    P_start,
                    sub_mesh_world=substrate_mesh,
                    side=side,
                    source=source,
                    stand_off_mm=stand_off_mm,
                    ray_len_mm=2000.0,
                    start_lift_mm=max(5.0, stand_off_mm + 5.0),
                    invert_dirs=False,
                    lock_xy=True,
                )
            except Exception:
                _LOG.exception("cast_rays_for_side failed for side=%s", side)
                continue

            valid = np.asarray(rc.valid, bool).reshape(-1)
            if not np.any(valid):
                continue

            hit = np.asarray(rc.hit_mm, float).reshape(-1, 3)[valid]
            nrm = _normalize(np.asarray(rc.normal, float).reshape(-1, 3)[valid])
            tcp = np.asarray(rc.tcp_mm, float).reshape(-1, 3)[valid]

            tcp_all.append(tcp)

            # ✅ Mask = TCP + Normal * 50mm (konstant "über dem Pfad")
            mask_world = tcp + nrm * float(MASK_OFFSET_MM)

            # -------- Draw --------
            if vis.get("path", True):
                self.scene.add_path_polyline(tcp, layer="path", color="#2ecc71", line_width=2.2, lighting=False)

            if vis.get("mask", True):
                self.scene.add_path_polyline(mask_world, layer="mask", color="royalblue", line_width=3.0, lighting=False)

            if vis.get("hits", True):
                self.scene.add_mesh(rays_hit_poly, layer="rays_hit", color="#3498db", line_width=1.2, lighting=False)

            if vis.get("misses", True):
                self.scene.add_mesh(rays_miss_poly, layer="rays_miss", color="#e74c3c", line_width=1.2, lighting=False)

            if vis.get("normals", False):
                # Stand-off + Normalen
                st = _polydata_from_segments(hit, tcp)
                self.scene.add_mesh(st, layer="normals", color="#f39c12", line_width=1.2, lighting=False)
                nm = _polydata_from_segments(tcp, tcp + nrm * 10.0)
                self.scene.add_mesh(nm, layer="normals", color="#f1c40f", line_width=1.2, lighting=False)

            if vis.get("frames", False):
                # ✅ Frames:
                # - HELIX: tangential
                # - sonst: folgt Side-KS (u_world/v_world), aber auf Tangentialebene projiziert
                use_helix_logic = ("spiral_cylinder" in (source or "").lower()) or ("helix" in (source or "").lower())

                cfg = _side_cfg(side)
                if cfg is None:
                    continue

                if use_helix_logic:
                    T = _tangents_from_path(tcp)
                    X = T - nrm * np.sum(T * nrm, axis=1, keepdims=True)
                    X = _normalize(X)

                    bad = np.linalg.norm(X, axis=1) < 1e-6
                    if np.any(bad):
                        fallback = np.tile(np.array([1.0, 0.0, 0.0], float), (len(X), 1))
                        X[bad] = _normalize(fallback - nrm[bad] * np.sum(fallback * nrm[bad], axis=1, keepdims=True))

                    Y = _normalize(np.cross(nrm, X))

                else:
                    u = _normalize(np.asarray(cfg["u_world"], float).reshape(1, 3))[0]
                    v = _normalize(np.asarray(cfg["v_world"], float).reshape(1, 3))[0]

                    u_dot = np.sum(nrm * u[None, :], axis=1, keepdims=True)
                    X = u[None, :] - nrm * u_dot
                    X = _normalize(X)

                    bad = np.linalg.norm(X, axis=1) < 1e-6
                    if np.any(bad):
                        v_dot = np.sum(nrm[bad] * v[None, :], axis=1, keepdims=True)
                        Xb = v[None, :] - nrm[bad] * v_dot
                        X[bad] = _normalize(Xb)

                    Y = _normalize(np.cross(nrm, X))

                    # Stabil: Y soll in Richtung v_world zeigen (kein Flip-Flop)
                    sgn = np.sum(Y * v[None, :], axis=1)
                    flip = sgn < 0.0
                    if np.any(flip):
                        X[flip] = -X[flip]
                        Y[flip] = -Y[flip]

                # subsample damit es sichtbar bleibt
                n_pts = len(tcp)
                step = max(1, int(round(max(1, n_pts // 60))))
                O2 = tcp[::step]
                X2 = X[::step]
                Y2 = Y[::step]
                Z2 = nrm[::step]

                sx = 12.0  # mm
                self.scene.add_mesh(_polydata_from_segments(O2, O2 + X2 * sx), layer="frames_x", color="#e67e22", line_width=1.0, lighting=False)
                self.scene.add_mesh(_polydata_from_segments(O2, O2 + Y2 * sx), layer="frames_y", color="#16a085", line_width=1.0, lighting=False)
                self.scene.add_mesh(_polydata_from_segments(O2, O2 + Z2 * sx), layer="frames_z", color="#2980b9", line_width=1.0, lighting=False)

        # 2D update: concat tcp
        path_xyz = None
        if tcp_all:
            try:
                path_xyz = np.vstack(tcp_all)
            except Exception:
                path_xyz = None

        try:
            self.update_2d_scene(substrate_mesh=scene.substrate_mesh, path_xyz=path_xyz)
        except Exception:
            _LOG.exception("update_2d_scene failed")

        # Info
        try:
            info = {
                "points": int(0 if path_xyz is None else len(path_xyz)),
                "tris": scene.mesh_tris,
                "bounds": scene.bounds,
            }
            self.grpInfo.set_values(info)
        except Exception:
            _LOG.exception("InfoGroupBox update failed")

        # Camera restore + render
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
