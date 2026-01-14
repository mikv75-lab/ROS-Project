# -*- coding: utf-8 -*-
from __future__ import annotations
from typing import Optional, Callable, Dict, Any, List, Tuple

import numpy as np
import pyvista as pv
import logging

from .raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("tabs.recipe.preview.overlays")

# -------- Helpers & Mathe ----------------------------------------------------
def _safe_norm(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n < eps:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return v / n

def _orthonormal_basis_from_z(z: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    z = _safe_norm(z)
    h = np.array([1.0, 0.0, 0.0], dtype=float) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0], dtype=float)
    x = _safe_norm(np.cross(h, z))
    y = _safe_norm(np.cross(z, x))
    return x, y, z

def _quat_to_axes(qx: float, qy: float, qz: float, qw: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    x2, y2, z2 = 2*qx, 2*qy, 2*qz
    xx, yy, zz = qx*x2, qy*y2, qz*z2
    xy, xz, yz = qx*y2, qx*z2, qy*z2
    wx, wy, wz = qw*x2, qw*y2, qw*z2
    R = np.array([
        [1.0 - (yy + zz), xy - wz,        xz + wy       ],
        [xy + wz,         1.0 - (xx + zz), yz - wx       ],
        [xz - wy,         yz + wx,        1.0 - (xx + yy)]
    ], dtype=float)
    X = R[:, 0]; Y = R[:, 1]; Z = R[:, 2]
    return _safe_norm(X), _safe_norm(Y), _safe_norm(Z)

def _tangents_from_path(P: np.ndarray) -> np.ndarray:
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    n = P.shape[0]
    if n == 0:
        return np.zeros((0, 3), dtype=float)
    if n == 1:
        return np.array([[1.0, 0.0, 0.0]], dtype=float)
    T = np.empty((n, 3), dtype=float)
    T[0]  = P[1]  - P[0]
    T[-1] = P[-1] - P[-2]
    if n > 2:
        T[1:-1] = P[2:] - P[:-2]
    for i in range(n):
        nrm = np.linalg.norm(T[i])
        T[i] = (T[i] / nrm) if nrm > 1e-12 else (T[i-1] if i else np.array([1.0, 0.0, 0.0]))
    for i in range(1, n):
        if np.dot(T[i-1], T[i]) < 0.0:
            T[i] = -T[i]
    return T

# -------- Side-Frame & Einbettung -------------------------------------------
def _side_frame(bounds: Tuple[float, float, float, float, float, float]):
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    cz = 0.5 * (zmin + zmax)
    cfgs = {
        "top":   {"normal": np.array([0,  0, -1.0]), "anchor": ("z", "max"), "axes": ("x", "y")},
        "front": {"normal": np.array([0, +1,  0.0]), "anchor": ("y", "min"), "axes": ("x", "z")},
        "back":  {"normal": np.array([0, -1,  0.0]), "anchor": ("y", "max"), "axes": ("x", "z")},
        "left":  {"normal": np.array([+1, 0,  0.0]), "anchor": ("x", "min"), "axes": ("y", "z")},
        "right": {"normal": np.array([-1, 0,  0.0]), "anchor": ("x", "max"), "axes": ("y", "z")},
    }
    return cfgs, (cx, cy, cz)

def _embed_path_on_face(P0_local: np.ndarray, side: str, bounds) -> np.ndarray:
    (cfgs, (cx, cy, cz)) = _side_frame(bounds)
    cfg = cfgs.get(side, cfgs["top"])
    ax0, ax1 = cfg["axes"]
    anchor_axis, anchor_side = cfg["anchor"]
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    anchor_val = {"x": (xmin if anchor_side == "min" else xmax),
                  "y": (ymin if anchor_side == "min" else ymax),
                  "z": (zmin if anchor_side == "min" else zmax)}[anchor_axis]

    P0 = np.asarray(P0_local, float).reshape(-1, 3)
    out = np.empty_like(P0)

    def place(axis_name: str, arr: np.ndarray):
        if axis_name == "x": return cx + arr
        if axis_name == "y": return cy + arr
        if axis_name == "z": return cz + arr
        return arr

    a0 = P0[:, 0]
    a1 = P0[:, 1]

    axes = {"x": None, "y": None, "z": None}
    axes[ax0] = place(ax0, a0)
    axes[ax1] = place(ax1, a1)
    axes[anchor_axis] = np.full(P0.shape[0], anchor_val, dtype=float)

    out[:, 0] = axes["x"]
    out[:, 1] = axes["y"]
    out[:, 2] = axes["z"]
    return out

def _mk_poly_segments(starts: np.ndarray, dirs: np.ndarray, length: float) -> Optional[pv.PolyData]:
    try:
        if starts is None or dirs is None or len(starts) == 0:
            return None
        S = np.asarray(starts, float).reshape(-1, 3)
        D = np.asarray(dirs, float).reshape(-1, 3)
        n = min(len(S), len(D))
        if n == 0: return None
        S = S[:n]; D = D[:n]
        D = np.array([_safe_norm(d) for d in D], float)
        E = S + D * float(length)
        pts = np.vstack([S, E])
        lines = np.empty((n, 3), dtype=np.int64)
        lines[:, 0] = 2
        lines[:, 1] = np.arange(n, dtype=np.int64)
        lines[:, 2] = lines[:, 1] + n
        poly = pv.PolyData(pts)
        poly.lines = lines.reshape(-1)
        return poly
    except Exception:
        _LOG.exception("mk_poly_segments failed")
        return None

# -------- OverlayRenderer -----------------------------------------------------
class OverlayRenderer:
    def __init__(
        self,
        *,
        add_mesh_fn: Callable[..., Any],
        clear_layer_fn: Callable[[str], None],
        add_path_polyline_fn: Callable[..., Any],
        show_poly_fn: Callable[..., Any],
        show_frames_at_fn: Callable[..., None],
        set_layer_visible_fn: Callable[[str], bool] | Callable[[str, bool], None] | Callable[[str, bool, bool], None],
        update_2d_scene_fn: Callable[[Optional[pv.PolyData], Optional[np.ndarray], Optional[pv.PolyData]], None],
        layers: Dict[str, str],
        get_bounds: Callable[[], Any],
        yaml_out_fn: Optional[Callable[[str], None]] = None,
    ):
        self._add_mesh = add_mesh_fn
        self._clear_layer = clear_layer_fn
        self._add_path_polyline = add_path_polyline_fn
        self._show_poly = show_poly_fn
        self._show_frames_at = show_frames_at_fn
        self._set_layer_visible = set_layer_visible_fn
        self._update_2d = update_2d_scene_fn
        self._layers = dict(layers or {})
        self._get_bounds = get_bounds
        self._yaml_out = yaml_out_fn

        self._vis = {"mask": False, "path": True, "hits": False, "misses": False, "normals": False, "frames": False}
        self._cache: Dict[str, Any] = {}

    # ---- Sichtbarkeit ----
    def _L(self, name: str) -> str:
        return self._layers.get(name, name)

    def _set_vis_layer(self, layer: str, vis: bool, *, render: bool):
        try:
            self._set_layer_visible(layer, vis, render)  # type: ignore[misc]
        except TypeError:
            self._set_layer_visible(layer, vis)          # type: ignore[misc]

    def set_mask_visible(self, vis: bool, *, render: bool = True):
        self._vis["mask"] = bool(vis)
        self._set_vis_layer(self._L("mask"), vis, render=render)
        self._set_vis_layer(self._L("mask_mrk"), vis, render=render)

    def set_path_visible(self, vis: bool, *, render: bool = True):
        self._vis["path"] = bool(vis)
        self._set_vis_layer(self._L("path"), vis, render=render)
        self._set_vis_layer(self._L("path_mrk"), vis, render=render)

    def set_hits_visible(self, vis: bool, *, render: bool = True):
        self._vis["hits"] = bool(vis)
        self._set_vis_layer(self._L("rays_hit"), vis, render=render)

    def set_misses_visible(self, vis: bool, *, render: bool = True):
        self._vis["misses"] = bool(vis)
        self._set_vis_layer(self._L("rays_miss"), vis, render=render)

    def set_normals_visible(self, vis: bool, *, render: bool = True):
        self._vis["normals"] = bool(vis)
        self._set_vis_layer(self._L("normals"), vis, render=render)

    def set_frames_visible(self, vis: bool, *, render: bool = True):
        self._vis["frames"] = bool(vis)
        for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
            self._set_vis_layer(self._L(lyr), vis, render=render)

    def apply_visibility(self, vis: dict | None):
        if vis is not None:
            self._vis.update({
                "mask":    bool(vis.get("mask",    self._vis["mask"])),
                "path":    bool(vis.get("path",    self._vis["path"])),
                "hits":    bool(vis.get("hits",    self._vis["hits"])),
                "misses":  bool(vis.get("misses",  self._vis["misses"])),
                "normals": bool(vis.get("normals", self._vis["normals"])),
                "frames":  bool(vis.get("frames",  self._vis["frames"])),
            })
        self.set_path_visible(self._vis["path"], render=False)
        self.set_mask_visible(self._vis["mask"], render=False)
        self.set_hits_visible(self._vis["hits"], render=False)
        self.set_misses_visible(self._vis["misses"], render=False)
        self.set_normals_visible(self._vis["normals"], render=False)
        self.set_frames_visible(self._vis["frames"], render=True)

    # ---- Kompiliertes Zeichnen (FIXED) --------------------------------------
    def _compiled_iter(self, compiled: Any, sides_filter: Optional[List[str]]) -> List[tuple[str, dict]]:
        """
        Extrahiert sicher die Sides und Daten aus einem compiled Objekt (Dict oder Class).
        Schützt vor None (AttributError).
        """
        if compiled is None:
            return []

        src = {}
        # Versuche Dict-Access
        if isinstance(compiled, dict):
            src = compiled.get("sides") or compiled.get("paths_by_side") or {}
        else:
            # Versuche Attribut-Access
            src = getattr(compiled, "sides", None) or getattr(compiled, "paths_by_side", None) or {}

        if not isinstance(src, dict):
            return []

        items = list(src.items())
        if sides_filter:
            keep = set(map(str, sides_filter))
            items = [(s, e) for (s, e) in items if s in keep]
        return items

    @staticmethod
    def _entry_to_tcp_and_dirs(side: str, entry: dict, bounds, *, default_mask_dir: np.ndarray) -> tuple[
        Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[pv.PolyData]
    ]:
        if isinstance(entry, dict) and "poses_quat" in entry:
            poses = entry.get("poses_quat") or []
            if not poses:
                return None, None, None, None
            P = np.array([[p["x"], p["y"], p["z"]] for p in poses], dtype=float).reshape(-1, 3)
            X = []; Z = []
            for p in poses:
                qx, qy, qz, qw = float(p["qx"]), float(p["qy"]), float(p["qz"]), float(p["qw"])
                x_axis, _y_axis, z_axis = _quat_to_axes(qx, qy, qz, qw)
                X.append(x_axis); Z.append(z_axis)
            X = np.asarray(X, float)
            Z = np.asarray(Z, float)

            mask_poly = None
            try:
                Pmask = P - default_mask_dir.reshape(1, 3) * 10.0
                mask_poly = pv.lines_from_points(Pmask, close=False)
            except Exception:
                mask_poly = None
            return P, X, Z, mask_poly

        if isinstance(entry, dict) and "points_mm" in entry:
            P_local = np.asarray(entry.get("points_mm") or [], dtype=float).reshape(-1, 3)
            if P_local.size == 0:
                return None, None, None, None
            P_world = _embed_path_on_face(P_local, side, bounds)

            cfgs, _ = _side_frame(bounds)
            n = cfgs.get(side, cfgs["top"])["normal"]
            T = _tangents_from_path(P_world)
            X = np.array([_safe_norm(t - n * float(np.dot(t, n))) for t in T], float)
            for i in range(1, len(X)):
                if float(np.dot(X[i-1], X[i])) < 0.0:
                    X[i] = -X[i]
            Z = np.tile(_safe_norm(n), (len(P_world), 1))

            try:
                Pmask = P_world - _safe_norm(n).reshape(1, 3) * 10.0
                mask_poly = pv.lines_from_points(Pmask, close=False)
            except Exception:
                mask_poly = None
            return P_world, X, Z, mask_poly

        return None, None, None, None

    def render_compiled(
        self,
        *,
        substrate_mesh: Optional[pv.PolyData],
        compiled: Any,  # Draft object or dict or None
        visibility: Dict[str, bool],
        mask_lift_mm: float = 50.0,
        default_stand_off_mm: float = 10.0,
        ray_len_mm: float = 1000.0,
        sides: Optional[Dict[str, Any]] = None,
        only_selected: bool = True,
        **kwargs: Any  # Catch-all für zusätzliche Argumente (verhindert TypeError)
    ) -> Dict[str, Dict[str, Any]]:
        
        self._cache = {
            "substrate_mesh": substrate_mesh,
            "compiled": compiled,
            "mask_lift_mm": mask_lift_mm,
            "default_stand_off_mm": default_stand_off_mm,
            "ray_len_mm": ray_len_mm,
            "sides": sides,
        }

        vis = visibility or {}
        show_mask    = bool(vis.get("mask", False))
        show_path    = bool(vis.get("path", True))
        show_hits    = bool(vis.get("hits", False))
        show_misses  = bool(vis.get("misses", False))
        show_normals = bool(vis.get("normals", False))
        show_frames  = bool(vis.get("frames", False))

        def _clear_layer_if(name: str, cond: bool):
            if cond:
                self._clear_layer(self._L(name))

        _clear_layer_if("mask",         (show_mask or not only_selected))
        _clear_layer_if("mask_mrk",     (show_mask or not only_selected))
        _clear_layer_if("path",         (show_path or not only_selected))
        _clear_layer_if("path_mrk",     (show_path or not only_selected))
        _clear_layer_if("rays_hit",     (show_hits or not only_selected))
        _clear_layer_if("rays_miss",    (show_misses or not only_selected))
        _clear_layer_if("normals",      (show_normals or not only_selected))
        _clear_layer_if("frames_x",     (show_frames or not only_selected))
        _clear_layer_if("frames_y",     (show_frames or not only_selected))
        _clear_layer_if("frames_z",     (show_frames or not only_selected))
        _clear_layer_if("frames_labels",(show_frames or not only_selected))

        outputs_by_side: Dict[str, Dict[str, Any]] = {}

        # WICHTIG: Null-Check für compiled (verhindert den Crash im nachfolgenden Aufruf)
        if compiled is None:
            self.apply_visibility(vis)
            return outputs_by_side

        if substrate_mesh is None:
            self.apply_visibility(vis)
            return outputs_by_side

        bounds = substrate_mesh.bounds
        (cfgs, _) = _side_frame(bounds)

        items = self._compiled_iter(compiled, sides)
        if not items:
            self.apply_visibility(vis)
            return outputs_by_side

        for side, entry in items:
            cfg = cfgs.get(side, cfgs["top"])
            face_n = _safe_norm(cfg["normal"])

            tcp_points, x_dirs, z_dirs, mask_poly = self._entry_to_tcp_and_dirs(
                side, entry, bounds, default_mask_dir=face_n
            )

            if show_path and tcp_points is not None and len(tcp_points):
                self._add_path_polyline(tcp_points, layer=self._L("path"), color="#2ecc71",
                                        line_width=2.0, lighting=False, render=False, reset_camera=False)
                try:
                    step = max(1, int(round(max(1, tcp_points.shape[0] // 200))))
                    pts_s = tcp_points[::step]
                    self._add_mesh(pv.PolyData(pts_s), color="#2ecc71", layer=self._L("path_mrk"),
                                   point_size=8.0, lighting=False, render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("path markers failed (%s)", side)

            if show_normals and tcp_points is not None and z_dirs is not None:
                try:
                    normals_poly = _mk_poly_segments(tcp_points, z_dirs, length=10.0)
                    if normals_poly is not None:
                        self._add_mesh(normals_poly, color="#f1c40f", layer=self._L("normals"),
                                       line_width=1.3, lighting=False, render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("normals poly failed (%s)", side)

            if show_mask and mask_poly is not None:
                try:
                    self._add_mesh(mask_poly, color="royalblue", layer=self._L("mask"),
                                   line_width=2.0, lighting=False, render=False, reset_camera=False)
                except Exception:
                    _LOG.exception("mask poly failed (%s)", side)

            if show_frames and tcp_points is not None and z_dirs is not None and len(tcp_points):
                try:
                    step = max(1, int(round(max(1, tcp_points.shape[0] // 50))))
                    self._show_frames_at(
                        origins=tcp_points[::step],
                        z_dirs=z_dirs[::step],
                        x_dirs=None,
                        scale_mm=10.0,
                        line_width=1.0,
                        layer_prefix="frames",
                        add_labels=False,
                        labels=None,
                    )
                except Exception:
                    _LOG.exception("frames render failed (%s)", side)

            outputs_by_side[side] = {
                "tcp_points": tcp_points,
                "mask_poly": mask_poly,
                "z_dirs": z_dirs,
                "x_dirs": x_dirs,
                "valid_mask": None,
            }

        self.apply_visibility(vis)

        try:
            tcp_for_2d: Optional[np.ndarray] = None
            mask_for_2d: Optional[pv.PolyData] = None
            for s, _ in items:
                out = outputs_by_side.get(s) or {}
                t = out.get("tcp_points")
                if t is not None and len(t):
                    tcp_for_2d = t
                    mask_for_2d = out.get("mask_poly")
                    break
            self._update_2d(substrate_mesh=substrate_mesh, path_xyz=tcp_for_2d, mask_poly=mask_for_2d)
        except Exception:
            _LOG.exception("update_2d (compiled) failed")

        return outputs_by_side

    # ---- Rebuild einzelner Layer beim Aktivieren --------------------------
    def rebuild_layers(self, layer_names: List[str]) -> None:
        if not self._cache:
            _LOG.debug("rebuild_layers: no cache yet; nothing to rebuild")
            return
        vis = {"mask": False, "path": False, "hits": False, "misses": False, "normals": False, "frames": False}
        for key in layer_names:
            if key in vis:
                vis[key] = True
        try:
            self.render_compiled(
                substrate_mesh=self._cache.get("substrate_mesh"),
                compiled=self._cache.get("compiled"),
                visibility=vis,
                mask_lift_mm=self._cache.get("mask_lift_mm", 50.0),
                default_stand_off_mm=self._cache.get("default_stand_off_mm", 10.0),
                ray_len_mm=self._cache.get("ray_len_mm", 1000.0),
                sides=self._cache.get("sides"),
                only_selected=True,
            )
        except Exception:
            _LOG.exception("rebuild_layers failed")