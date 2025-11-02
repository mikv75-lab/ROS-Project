# -*- coding: utf-8 -*-
from __future__ import annotations
import logging
from typing import Any, Callable, Dict, Optional, List

import numpy as np
import pyvista as pv

from .path_builder import PathBuilder
from .raycast_projector import cast_rays_for_side

_LOG = logging.getLogger("app.tabs.recipe.preview.overlays")


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


def _orthonormal_basis_from_zx(z: np.ndarray, x_hint: np.ndarray | None) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    z = _safe_norm(z)
    if x_hint is None:
        return _orthonormal_basis_from_z(z)
    x = _safe_norm(x_hint)
    x = _safe_norm(np.cross(np.cross(x, z), z))  # X gegen Z orthogonalisieren
    y = _safe_norm(np.cross(z, x))
    return x, y, z


def _rotmat_to_rpy_xyz(R: np.ndarray) -> tuple[float, float, float]:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    eps = 1e-9
    sy = -R[2, 0]
    sy = np.clip(sy, -1.0, 1.0)
    ry = np.arcsin(sy)
    cy = np.cos(ry)
    if abs(cy) > eps:
        rx = np.arctan2(R[2, 1] / cy, R[2, 2] / cy)
        rz = np.arctan2(R[1, 0] / cy, R[0, 0] / cy)
    else:
        rx = 0.0
        rz = np.arctan2(-R[0, 1], R[1, 1])
    return float(np.degrees(rx)), float(np.degrees(ry)), float(np.degrees(rz))


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


def _make_yaml_hits_with_defaults(
    points_hits: Optional[np.ndarray],
    z_dirs_hits: Optional[np.ndarray],
    x_dirs_hits: Optional[np.ndarray],
    *,
    default_rpy: tuple[float, float, float] = (0.0, 0.0, -90.0)
) -> Optional[str]:
    if points_hits is None or z_dirs_hits is None:
        return None
    O = np.asarray(points_hits, float).reshape(-1, 3)
    Z = np.asarray(z_dirs_hits, float).reshape(-1, 3)
    if O.shape != Z.shape or O.shape[0] == 0:
        return None
    X = None
    if x_dirs_hits is not None:
        X = np.asarray(x_dirs_hits, float).reshape(-1, 3)
        if X.shape != O.shape:
            X = None

    poses: List[Dict[str, float]] = []
    for i in range(O.shape[0]):
        z = Z[i]
        if not np.all(np.isfinite(z)) or np.linalg.norm(z) < 1e-9:
            rx, ry, rz = default_rpy
        else:
            x_hint = X[i] if X is not None else None
            x, y, z = _orthonormal_basis_from_zx(z, x_hint)
            R = np.column_stack([x, y, z])
            rx, ry, rz = _rotmat_to_rpy_xyz(R)
        poses.append({"x": float(O[i, 0]), "y": float(O[i, 1]), "z": float(O[i, 2]),
                      "rx": rx, "ry": ry, "rz": rz})

    header = {"units": {"position": "mm", "angles": "deg"}, "count": len(poses), "frames": poses}
    try:
        import yaml
        return yaml.safe_dump(header, sort_keys=False, allow_unicode=True)
    except Exception:
        lines = ["units: { position: mm, angles: deg }", f"count: {len(poses)}", "frames:"]
        for p in poses:
            lines += [
                f"  - x: {p['x']:.6f}",
                f"    y: {p['y']:.6f}",
                f"    z: {p['z']:.6f}",
                f"    rx: {p['rx']:.6f}",
                f"    ry: {p['ry']:.6f}",
                f"    rz: {p['rz']:.6f}",
            ]
        return "\n".join(lines)


class OverlayRenderer:
    def __init__(
        self,
        *,
        add_mesh_fn: Callable[..., Any],
        clear_layer_fn: Callable[[str], None],
        add_path_polyline_fn: Callable[..., Any],
        show_poly_fn: Callable[..., Any],
        show_frames_at_fn: Callable[..., None],
        set_layer_visible_fn: Callable[[str, bool, bool], None] | Callable[[str, bool], None],
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

        self._out: Dict[str, Any] = {
            "substrate_mesh": None,
            "mask_poly": None,
            "rays_hit_poly": None,
            "rays_miss_poly": None,
            "tcp_poly": None,
            "tcp_points": None,
            "valid_mask": None,
            "rc": None,
            "x_dirs": None,
            "z_dirs": None,
            "side": None,
            "stand_off_mm": None,
        }

        self._vis = {"mask": False, "path": True, "hits": False, "misses": False, "normals": False, "frames": False}

    def _L(self, name: str) -> str:
        return self._layers.get(name, name)

    def _set_vis_layer(self, layer: str, vis: bool, *, render: bool):
        try:
            self._set_layer_visible(layer, vis, render)  # type: ignore[misc]
        except TypeError:
            self._set_layer_visible(layer, vis)          # type: ignore[misc]

    def get_outputs(self) -> Dict[str, Any]:
        return self._out

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
        out = self.get_outputs()
        P  = out.get("tcp_points")
        X  = out.get("x_dirs")
        Z  = out.get("z_dirs")
        try:
            if vis and P is not None and Z is not None and len(P) and len(Z):
                self._show_frames_at(
                    origins=P, z_dirs=Z, x_dirs=X,
                    scale_mm=None, line_width=1.0, clear_old=True, add_labels=False,
                )
            else:
                for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                    self._clear_layer(self._L(lyr))
        except Exception:
            _LOG.exception("show_frames_at (toggle) failed")

    def apply_visibility(self, vis: Dict[str, bool] | None):
        if vis:
            self._vis.update({
                "mask":    bool(vis.get("mask",    self._vis["mask"])),
                "path":    bool(vis.get("path",    self._vis["path"])),
                "hits":    bool(vis.get("hits",    self._vis["hits"])),
                "misses":  bool(vis.get("misses",  self._vis["misses"])),
                "normals": bool(vis.get("normals", self._vis["normals"])),
                "frames":  bool(vis.get("frames",  self._vis["frames"])),
            })
        self.set_mask_visible(self._vis["mask"], render=False)
        self.set_path_visible(self._vis["path"], render=False)
        self.set_hits_visible(self._vis["hits"], render=False)
        self.set_misses_visible(self._vis["misses"], render=False)
        self.set_normals_visible(self._vis["normals"], render=True)

    def render_from_model(
        self,
        *,
        model: object,
        substrate_mesh: pv.PolyData,
        side: Optional[str] = None,
        default_stand_off_mm: float = 10.0,
        mask_lift_mm: float = 50.0,
        ray_len_mm: float = 1000.0,
        visibility: Optional[Dict[str, bool]] = None,
    ):
        self._out.update({
            "substrate_mesh": substrate_mesh,
            "mask_poly": None,
            "rays_hit_poly": None,
            "rays_miss_poly": None,
            "tcp_poly": None,
            "tcp_points": None,
            "valid_mask": None,
            "rc": None,
            "x_dirs": None,
            "z_dirs": None,
            "side": side,
            "stand_off_mm": None,
        })

        # 1) Pfad sammeln
        pbs = dict(model.get("paths_by_side") or {}) if isinstance(model, dict) \
              else dict(getattr(model, "paths_by_side", {}) or {})
        if not pbs:
            for ly in ("mask", "rays_hit", "rays_miss", "normals", "path", "path_mrk"):
                self._clear_layer(self._L(ly))
            try:
                self._update_2d(substrate_mesh, None, None)
            except Exception:
                _LOG.exception("update_2d (no paths) failed")
            if self._yaml_out:
                self._yaml_out("units: { position: mm, angles: deg }\ncount: 0\nframes: []\n")
            return

        if side is None:
            side = "top" if "top" in pbs else next(iter(pbs.keys()))
        self._out["side"] = side

        sample_step = float(pbs.get(side, {}).get("sample_step_mm", 1.0))
        pd = PathBuilder.from_side(model, side=side, sample_step_mm=sample_step)
        P0 = np.asarray(pd.points_mm, dtype=float).reshape(-1, 3)
        if len(P0) == 0:
            for ly in ("mask", "rays_hit", "rays_miss", "normals", "path", "path_mrk"):
                self._clear_layer(self._L(ly))
            try:
                self._update_2d(substrate_mesh, None, None)
            except Exception:
                _LOG.exception("update_2d (empty path) failed")
            if self._yaml_out:
                self._yaml_out("units: { position: mm, angles: deg }\ncount: 0\nframes: []\n")
            return

        # 2) Welt-Offset
        xmin, xmax, ymin, ymax, zmin, zmax = substrate_mesh.bounds
        cx, cy = 0.5 * (xmin + xmax), 0.5 * (ymin + ymax)
        z_plane = float(zmin)  # <— gebraucht für Miss-Rays
        P0_world = P0 + np.array([cx, cy, z_plane], dtype=float)

        # 3) Maske
        P_mask = None
        mask_poly = None
        try:
            z_mask = float(zmax + mask_lift_mm)
            P_mask = P0_world.copy()
            P_mask[:, 2] = z_mask
            mask_poly = pv.lines_from_points(P_mask, close=False)
        except Exception:
            mask_poly = None
        self._out["mask_poly"] = mask_poly

        # 4) Rays & TCP
        rc = None
        rays_hit_poly = None
        tcp_poly = None
        tcp_points = None
        valid_mask = None
        x_dirs = None
        z_dirs = None
        try:
            stand_off = float(
                getattr(model, "stand_off_mm", None)
                or (isinstance(model, dict) and model.get("stand_off_mm"))
                or default_stand_off_mm
            )
            self._out["stand_off_mm"] = stand_off

            rc, rays_hit_poly, tcp_poly = cast_rays_for_side(
                P_world_start=P_mask,
                sub_mesh_world=substrate_mesh,
                side=side,
                source=pd.meta.get("source", "points"),
                stand_off_mm=stand_off,
                ray_len_mm=float(ray_len_mm),
                start_lift_mm=0.0,
                flip_normals_to_face_rays=True,
                invert_dirs=False,
                lock_xy=True,
            )
            valid_mask = getattr(rc, "valid", None)

            if rc is not None and getattr(rc, "tcp_mm", None) is not None:
                tcp_all = rc.tcp_mm
                tcp_points = tcp_all[valid_mask] if (valid_mask is not None and np.any(valid_mask)) else tcp_all

            z_full = getattr(rc, "refl_dir", None)
            x_full = getattr(rc, "x_dir", None) or getattr(rc, "frame_x", None)

            if z_full is not None:
                ZF = np.asarray(z_full, dtype=float).reshape(-1, 3)
                z_dirs = ZF[valid_mask] if (valid_mask is not None and len(ZF) == len(valid_mask)) else ZF

            if x_full is not None:
                XF = np.asarray(x_full, dtype=float).reshape(-1, 3)
                x_dirs = XF[valid_mask] if (valid_mask is not None and len(XF) == len(valid_mask)) else XF

        except Exception:
            _LOG.exception("cast_rays_for_side failed")

        # Fallback X aus Tangenten
        if tcp_points is not None and (x_dirs is None or len(x_dirs) != len(tcp_points)):
            try:
                T = _tangents_from_path(tcp_points)
                if z_dirs is not None and len(z_dirs) == len(T):
                    Xo = np.cross(np.cross(T, z_dirs), z_dirs)
                    for i in range(Xo.shape[0]):
                        nrm = np.linalg.norm(Xo[i])
                        Xo[i] = (Xo[i] / nrm) if nrm > 1e-12 else _orthonormal_basis_from_z(z_dirs[i])[0]
                    for i in range(1, Xo.shape[0]):
                        if np.dot(Xo[i-1], Xo[i]) < 0.0:
                            Xo[i] = -Xo[i]
                    x_dirs = Xo
            except Exception:
                _LOG.exception("Fallback X via tangents failed")

        self._out.update({
            "rc": rc,
            "rays_hit_poly": rays_hit_poly,
            "tcp_poly": tcp_poly,
            "tcp_points": tcp_points,
            "valid_mask": valid_mask,
            "x_dirs": x_dirs,
            "z_dirs": z_dirs,
        })

        # 5) Miss-Rays (immer vorbereiten)
        rays_miss = None
        try:
            if valid_mask is not None and P_mask is not None and len(P_mask) == len(valid_mask) and np.any(~valid_mask):
                base_dir = {
                    "top":   np.array([0.0,  0.0, -1.0], dtype=float),
                    "front": np.array([0.0,  1.0,  0.0], dtype=float),
                    "back":  np.array([0.0, -1.0,  0.0], dtype=float),
                    "left":  np.array([1.0,  0.0,  0.0], dtype=float),
                    "right": np.array([-1.0, 0.0,  0.0], dtype=float),
                }.get(side, np.array([0.0, 0.0, -1.0], dtype=float))

                Pmiss_start = P_mask[~valid_mask]
                D = np.tile(base_dir / (np.linalg.norm(base_dir) + 1e-12), (len(Pmiss_start), 1))
                have_z = np.abs(D[:, 2]) > 1e-9
                if np.any(have_z):
                    S = Pmiss_start[have_z]
                    Dz = D[have_z]
                    t = (z_plane - S[:, 2]) / Dz[:, 2]
                    pos = t > 0
                    S = S[pos]; Dz = Dz[pos]; t = t[pos]
                    if len(S):
                        Pend = S + Dz * t.reshape(-1, 1)
                        pts = np.vstack([S, Pend])
                        nseg = len(S)
                        lines = np.empty((nseg, 3), dtype=np.int64)
                        lines[:, 0] = 2
                        lines[:, 1] = np.arange(nseg, dtype=np.int64)
                        lines[:, 2] = lines[:, 1] + nseg
                        rays_miss = pv.PolyData(pts)
                        rays_miss.lines = lines.reshape(-1)
        except Exception:
            _LOG.exception("miss rays build failed")
        self._out["rays_miss_poly"] = rays_miss

        # 6) LAYER IMMER BEFÜLLEN (sichtbar/nicht sichtbar kommt danach)
        # Mask
        self._clear_layer(self._L("mask"))
        if mask_poly is not None:
            self._add_mesh(mask_poly, color="royalblue", layer=self._L("mask"),
                           line_width=2.0, lighting=False, render=False, reset_camera=False)

        # Rays (Hits)
        self._clear_layer(self._L("rays_hit"))
        if getattr(rays_hit_poly, "n_lines", 0):
            self._add_mesh(rays_hit_poly, color="#85C1E9", layer=self._L("rays_hit"),
                           line_width=1.5, lighting=False, render=False, reset_camera=False)

        # Misses
        self._clear_layer(self._L("rays_miss"))
        if rays_miss is not None:
            self._add_mesh(rays_miss, color="#e74c3c", layer=self._L("rays_miss"),
                           line_width=1.2, lighting=False, render=False, reset_camera=False)

        # Normals
        self._clear_layer(self._L("normals"))
        if getattr(tcp_poly, "n_lines", 0):
            self._add_mesh(tcp_poly, color="#f1c40f", layer=self._L("normals"),
                           line_width=1.3, lighting=False, render=False, reset_camera=False)

        # TCP-Polyline (grün)
        self._clear_layer(self._L("path"))
        self._clear_layer(self._L("path_mrk"))
        if tcp_points is not None and len(tcp_points):
            self._add_path_polyline(tcp_points, layer=self._L("path"), color="#2ecc71",
                                    line_width=2.0, lighting=False, render=False, reset_camera=False)
            try:
                pts = np.asarray(tcp_points, float)
                step = max(1, int(round(max(1, pts.shape[0] // 200))))
                pts_s = pts[::step]
                self._add_mesh(pv.PolyData(pts_s), color="#2ecc71", layer=self._L("path_mrk"),
                               point_size=8.0, lighting=False, render=False, reset_camera=False)
            except Exception:
                _LOG.exception("path markers failed")

        # 2D-Szene (nur das anzeigen, was aktiv ist)
        try:
            self._update_2d(
                substrate_mesh,
                tcp_points if (visibility or {}).get("path", self._vis["path"]) else None,
                mask_poly  if (visibility or {}).get("mask", self._vis["mask"]) else None
            )
        except Exception:
            _LOG.exception("update_2d_scene failed")

        # Frames (nur wenn sichtbar)
        try:
            if (visibility or {}).get("frames", self._vis["frames"]) and tcp_points is not None:
                if z_dirs is not None and len(z_dirs):
                    self._show_frames_at(
                        origins=tcp_points, z_dirs=z_dirs, x_dirs=x_dirs,
                        scale_mm=None, line_width=1.0, clear_old=True, add_labels=False,
                    )
            else:
                for lyr in ("frames_x", "frames_y", "frames_z", "frames_labels"):
                    self._clear_layer(self._L(lyr))
        except Exception:
            _LOG.exception("render frames failed")

        # Sichtbarkeit final anwenden (damit Checkboxen später wirken)
        self.apply_visibility(visibility)

        # YAML ausgeben (nur Hits; Misses bekommen Default-RPY)
        if self._yaml_out:
            try:
                yaml_txt = _make_yaml_hits_with_defaults(
                    tcp_points, z_dirs, x_dirs,
                    default_rpy=(0.0, 0.0, -90.0)
                )
                if yaml_txt is not None:
                    self._yaml_out(yaml_txt)
            except Exception:
                _LOG.exception("yaml_out_fn failed")
